#!/usr/bin/env python3
"""Check print-file names, print records, LFS attributes, and stray G-code.

A print file is a ``.stl`` or ``.3mf`` file under a folder named ``prints``,
at any depth (``prints/``, ``projects/<name>/prints/``). Its records are in
that same ``prints`` folder. A mesh outside every ``prints`` folder, such as
a CFD geometry, is not a print file and is not checked here. G-code is
allowed only under a ``prints/frozen/`` folder.

Files come from git's file list (``tools/_repo_files.py``): tracked files and
untracked files that ``.gitignore`` does not exclude.

Usage::

    python3 tools/check_prints.py tests/fixtures/additive-manufacturing/pass
"""

from __future__ import annotations

import argparse
import csv
import re
import subprocess
import sys
from pathlib import Path

from _repo_files import tracked_files

SKIP = frozenset({".agents", "third_party", ".git"})
MESH_SUFFIXES = {".3mf", ".stl"}
LFS_SUFFIXES = {".3mf", ".stl", ".step", ".stp"}
NAME = re.compile(
    r"^(?P<part>.+)_r(?P<rev>\d{2,})_(?P<material>[^_]+)_(?P<process>[^_]+)"
    r"\.(?:3mf|stl)$",
    re.IGNORECASE,
)
LFS_LINES = (
    "*.stl filter=lfs diff=lfs merge=lfs -text",
    "*.3mf filter=lfs diff=lfs merge=lfs -text",
    "*.step filter=lfs diff=lfs merge=lfs -text",
    "*.stp filter=lfs diff=lfs merge=lfs -text",
)
REQUIRED = (
    "printer",
    "nozzle",
    "material",
    "brand",
    "color",
    "lot",
    "profile",
    "commit",
    "orientation",
    "layer height",
    "infill",
    "walls",
    "supports",
    "print time",
    "mass",
    "outcome",
    "measured",
)


def _skipped(path: Path, root: Path) -> bool:
    try:
        parts = path.relative_to(root).parts
    except ValueError:
        parts = path.parts
    if any(part in SKIP for part in parts):
        return True
    return "tests" in parts and "fixtures" in parts


def _norm_key(key: str) -> str:
    return key.strip().lower().replace("_", " ")


def markdown_fields(text: str) -> dict[str, str]:
    """Return ``Key: value`` fields from a print record.

    Parameters
    ----------
    text : str
        Markdown record.

    Returns
    -------
    dict of str to str
        Lower-case keys. Underscores are spaces. Blank values are dropped.
    """
    found: dict[str, str] = {}
    for raw in text.splitlines():
        line = raw.strip()
        if line.startswith("- "):
            line = line[2:].strip()
        if ":" not in line:
            continue
        key, value = line.split(":", 1)
        if value.strip():
            found[_norm_key(key)] = value.strip()
    return found


def _has(fields: dict[str, str], name: str) -> bool:
    if fields.get(name):
        return True
    return any(key.startswith(name) and fields[key] for key in fields)


def missing_fields(fields: dict[str, str]) -> list[str]:
    """Return required print-record fields that are empty.

    Parameters
    ----------
    fields : dict of str to str
        Parsed record fields.

    Returns
    -------
    list of str
        Missing field names.
    """
    return [name for name in REQUIRED if not _has(fields, name)]


def _csv_rows(path: Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        rows: list[dict[str, str]] = []
        for row in reader:
            rows.append(
                {
                    _norm_key(key): value.strip()
                    for key, value in row.items()
                    if key is not None and value is not None
                }
            )
        return rows


def _rev_values(rev: str) -> set[str]:
    return {rev, f"r{rev}", rev.lstrip("0") or "0"}


def _profile_problem(root: Path, profile: str, commit: str) -> str | None:
    if not profile or not commit:
        return "profile and commit are required"
    if (root / ".git").exists():
        spec = f"{commit}:{profile}"
        result = subprocess.run(  # noqa: S603
            ["git", "cat-file", "-e", spec],  # noqa: S607
            cwd=root,
            check=False,
            capture_output=True,
            text=True,
        )
        if result.returncode != 0:
            return f"profile {profile} is missing at commit {commit}"
        return None
    if not (root / profile).is_file():
        return f"profile {profile} is missing"
    return None


def _record_problems(
    root: Path, prints: Path, files: list[Path], part: str, rev: str
) -> list[str]:
    problems: list[str] = []
    candidates: list[dict[str, str]] = []
    name_re = re.compile(
        rf"^\d{{4}}-\d{{2}}-\d{{2}}_{re.escape(part)}_r{re.escape(rev)}\.md$"
    )
    for path in files:
        if path.suffix == ".md" and name_re.match(path.name):
            candidates.append(markdown_fields(path.read_text(encoding="utf-8")))
    csv_path = prints / "prints.csv"
    if csv_path in files:
        allowed = _rev_values(rev)
        for row in _csv_rows(csv_path):
            row_part = row.get("part", "")
            row_rev = row.get("revision", "")
            if row_part == part and row_rev in allowed:
                candidates.append(row)
    if not candidates:
        return [f"no print record for {part} revision {rev}"]
    field_gaps = [missing_fields(fields) for fields in candidates]
    if not any(not gaps for gaps in field_gaps):
        problems.append(
            f"print record for {part} r{rev} is missing {', '.join(field_gaps[0])}"
        )
        return problems
    profile_gaps = []
    for fields in candidates:
        if missing_fields(fields):
            continue
        profile = fields.get("profile", "")
        commit = fields.get("commit", "")
        gap = _profile_problem(root, profile, commit)
        if gap is None:
            return []
        profile_gaps.append(gap)
    problems.extend(profile_gaps)
    return problems


def _prints_index(parts: tuple[str, ...]) -> int | None:
    """Return the index of the nearest ``prints`` folder above a file."""
    for index in range(len(parts) - 2, -1, -1):
        if parts[index] == "prints":
            return index
    return None


def _frozen(parts: tuple[str, ...]) -> bool:
    """Return true for a file under a ``prints/frozen/`` folder."""
    return any(
        parts[i] == "prints" and parts[i + 1] == "frozen" for i in range(len(parts) - 2)
    )


def check_tree(root: Path) -> list[str]:
    """Return print-workflow problems under ``root``.

    Parameters
    ----------
    root : pathlib.Path
        Project root, or a fixture root.

    Returns
    -------
    list of str
        Name, record, LFS, and G-code problems. Name, record, and LFS checks
        cover files under a ``prints`` folder only.
    """
    problems: list[str] = []
    lfs_found = False
    files = [path for path in tracked_files(root) if not _skipped(path, root)]
    by_folder: dict[Path, list[Path]] = {}
    for path in files:
        by_folder.setdefault(path.parent, []).append(path)
    for path in files:
        suffix = path.suffix.lower()
        parts = path.relative_to(root).parts
        rel = "/".join(parts)
        if suffix in {".gcode", ".bgcode"} and not _frozen(parts):
            problems.append(f"{rel}: G-code is allowed only under prints/frozen/")
        index = _prints_index(parts)
        if index is None:
            continue
        if suffix in LFS_SUFFIXES:
            lfs_found = True
        if suffix not in MESH_SUFFIXES:
            continue
        match = NAME.match(path.name)
        if match is None:
            problems.append(f"{rel}: name must be <part>_r<NN>_<material>_<process>")
            continue
        prints = root.joinpath(*parts[: index + 1])
        problems.extend(
            _record_problems(
                root,
                prints,
                by_folder.get(prints, []),
                match.group("part"),
                match.group("rev"),
            )
        )
    if lfs_found:
        attributes = root / ".gitattributes"
        if not attributes.is_file():
            problems.append(".gitattributes: mesh and STEP files must use Git LFS")
        else:
            text = attributes.read_text(encoding="utf-8")
            for line in LFS_LINES:
                if line not in text:
                    problems.append(f".gitattributes: missing `{line}`")
    return problems


def main(argv: list[str] | None = None) -> int:
    """Check a project tree and return an exit code."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("root", nargs="?", type=Path, default=Path("."))
    args = parser.parse_args(argv)
    problems = check_tree(args.root)
    for problem in problems:
        print(problem)
    if problems:
        print(f"{len(problems)} problem(s)", file=sys.stderr)
        return 1
    print("ok")
    return 0


if __name__ == "__main__":
    sys.exit(main())
