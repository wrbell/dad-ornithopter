#!/usr/bin/env python3
"""Lint engineering BOM CSV files and CAD-data hygiene.

The ``SD-<C>-<NNNN>`` scheme is a Research pick for Willem
(recommended synthesis). It is not an ASME or ISO scheme.
It is Willem's policy for this collection.

The tool reads files. It does not edit a BOM.
It does not rewrite Git history.
It does not open graded senior-design CAD files except to test a path.
Directory scans read git's file list (``tools/_repo_files.py``): tracked
files and untracked files that ``.gitignore`` does not exclude. Ignored
folders such as ``dist/`` or ``.local-evidence/`` are not read.
"""

from __future__ import annotations

import argparse
import csv
import fnmatch
import json
import re
import sys
from dataclasses import dataclass
from pathlib import Path, PurePosixPath

from _repo_files import git_listing, tracked_files

SKIP_DIR_NAMES = frozenset({".agents", "third_party", ".git"})
LFS_POINTER = b"version https://git-lfs.github.com/spec/v1"
ECO_LABELS = (
    "Part numbers:",
    "Old revision:",
    "New revision:",
    "Interchangeable:",
    "Reason:",
)


@dataclass(frozen=True)
class Finding:
    """One lint finding."""

    path: str
    line: int
    message: str
    severity: str = "error"

    def __str__(self) -> str:
        """Return a one-line report."""
        return f"{self.severity}: {self.path}:{self.line}: {self.message}"


def default_config_path() -> Path:
    """Return the part-number scheme shipped with this repo."""
    return (
        Path(__file__).resolve().parent.parent / "enforcement" / "bom" / "scheme.json"
    )


def load_config(path: Path) -> dict:
    """Load the part-number scheme."""
    return json.loads(path.read_text(encoding="utf-8"))


def _skipped(path: Path) -> bool:
    parts = path.parts
    if any(part in SKIP_DIR_NAMES for part in parts):
        return True
    return "fixtures" in parts and "fail" in parts


def _rel(path: Path, root: Path) -> str:
    try:
        return path.resolve().relative_to(root.resolve()).as_posix()
    except ValueError:
        return path.as_posix()


def _rows(path: Path) -> tuple[list[str], list[tuple[int, dict[str, str]]]]:
    text = path.read_text(encoding="utf-8-sig")
    reader = csv.DictReader(text.splitlines())
    if reader.fieldnames is None:
        return [], []
    header = [name.strip() for name in reader.fieldnames]
    rows: list[tuple[int, dict[str, str]]] = []
    for offset, raw in enumerate(reader, start=2):
        if raw is None or all(not (value or "").strip() for value in raw.values()):
            continue
        item = {key.strip(): (value or "").strip() for key, value in raw.items() if key}
        rows.append((offset, item))
    return header, rows


def discover_boms(roots: list[Path], root: Path) -> list[Path]:
    """Return BOM CSV files under the given roots."""
    found: list[Path] = []
    for start in roots:
        candidates = [start] if start.is_file() else tracked_files(start, (".csv",))
        for path in candidates:
            if not path.is_file() or _skipped(path):
                continue
            if path.name.lower().startswith("bom"):
                found.append(path)
    return sorted({_rel(path, root): path for path in found}.values())


def _cad_exists(csv_path: Path, root: Path, raw: str) -> bool:
    rel = Path(raw)
    if rel.is_absolute() or any(part == ".." for part in rel.parts):
        return False
    for candidate in (root / rel, csv_path.parent / rel):
        resolved = candidate.resolve()
        try:
            resolved.relative_to(root.resolve())
        except ValueError:
            continue
        if resolved.is_file():
            return True
    return False


def lint_bom(path: Path, config: dict, root: Path) -> list[Finding]:
    """Lint one BOM CSV file."""
    rel = _rel(path, root)
    header, rows = _rows(path)
    if not header:
        return [Finding(rel, 1, "BOM has no header")]
    findings: list[Finding] = []
    present = set(header)
    for name in config["columns"]:
        if name not in present:
            findings.append(Finding(rel, 1, f"missing column {name}"))
    if findings:
        return findings
    part_re = re.compile(config["part_pattern"])
    rev_re = re.compile(config["revision_pattern"])
    forbidden = tuple(config["forbidden_in_part_no"])
    seen: set[tuple[str, str]] = set()
    previous_level: int | None = None
    for line, row in rows:
        findings.extend(
            _lint_bom_row(
                rel,
                line,
                row,
                path,
                root,
                part_re,
                rev_re,
                forbidden,
                seen,
                previous_level,
            )
        )
        if row.get("level", "").isdigit():
            previous_level = int(row["level"])
    return findings


def _lint_bom_row(
    rel: str,
    line: int,
    row: dict[str, str],
    csv_path: Path,
    root: Path,
    part_re: re.Pattern[str],
    rev_re: re.Pattern[str],
    forbidden: tuple[str, ...],
    seen: set[tuple[str, str]],
    previous_level: int | None,
) -> list[Finding]:
    findings: list[Finding] = []
    part_no = row.get("part_no", "")
    rev = row.get("rev", "")
    description = row.get("description", "")
    make_buy = row.get("make_buy", "").lower()
    if not part_no:
        findings.append(Finding(rel, line, "part_no is empty"))
    if any(mark in part_no for mark in forbidden):
        findings.append(Finding(rel, line, "part_no contains a slash"))
    if make_buy not in {"make", "buy"}:
        findings.append(Finding(rel, line, "make_buy must be make or buy"))
    elif make_buy == "make":
        if part_no and not part_re.fullmatch(part_no):
            findings.append(
                Finding(
                    rel,
                    line,
                    "make part_no must match SD-<C>-<NNNN>. "
                    "This scheme is a recommended synthesis.",
                )
            )
        cad_file = row.get("cad_file", "")
        if not cad_file:
            findings.append(Finding(rel, line, "make row needs cad_file"))
        elif not _cad_exists(csv_path, root, cad_file):
            findings.append(Finding(rel, line, f"cad_file {cad_file} is missing"))
    elif make_buy == "buy" and not part_no:
        findings.append(Finding(rel, line, "buy row needs a vendor part number"))
    elif make_buy == "buy" and not row.get("source"):
        findings.append(Finding(rel, line, "buy row needs a source"))
    if not description:
        findings.append(Finding(rel, line, "description is empty"))
    if not rev_re.fullmatch(rev):
        findings.append(
            Finding(
                rel,
                line,
                "rev must be - or 1 to 3 letters. Skip I, O, Q, S, X, and Z.",
            )
        )
    key = (part_no, rev)
    if part_no and key in seen:
        findings.append(
            Finding(rel, line, f"duplicate part_no and rev {part_no} {rev}")
        )
    elif part_no:
        seen.add(key)
    qty_raw = row.get("qty", "")
    try:
        qty = float(qty_raw)
    except ValueError:
        findings.append(Finding(rel, line, "qty must be a number greater than 0"))
    else:
        if qty <= 0:
            findings.append(Finding(rel, line, "qty must be greater than 0"))
    if not row.get("uom"):
        findings.append(Finding(rel, line, "uom is empty"))
    level_raw = row.get("level", "")
    if not level_raw.isdigit():
        findings.append(Finding(rel, line, "level must be an integer >= 0"))
    else:
        level = int(level_raw)
        if previous_level is not None and level > previous_level + 1:
            findings.append(Finding(rel, line, "level rises by more than 1"))
    return findings


def large_file_findings(root: Path, max_bytes: int) -> list[Finding]:
    """Fail when a file over the limit is not a Git LFS pointer."""
    findings: list[Finding] = []
    if not root.is_dir():
        return findings
    for path in tracked_files(root):
        if not path.is_file() or _skipped(path):
            continue
        if path.stat().st_size <= max_bytes:
            continue
        with path.open("rb") as handle:
            head = handle.read(len(LFS_POINTER))
        if head.startswith(LFS_POINTER):
            continue
        findings.append(
            Finding(
                _rel(path, root),
                1,
                f"file is over {max_bytes} bytes and is not a Git LFS pointer",
            )
        )
    return findings


def tree_byte_findings(root: Path, warn_bytes: int, fail_bytes: int) -> list[Finding]:
    """Warn near 1 GB and fail at 5 GB. Skip .git and vendored trees."""
    total = 0
    for path in tracked_files(root):
        if not path.is_file() or _skipped(path):
            continue
        total += path.stat().st_size
    rel = _rel(root, root) or "."
    if total >= fail_bytes:
        return [
            Finding(rel, 1, f"tree is {total} bytes. Keep it under {fail_bytes} bytes.")
        ]
    if total >= warn_bytes:
        return [
            Finding(
                rel,
                1,
                f"tree is {total} bytes. GitHub recommends under {warn_bytes} bytes.",
                "warning",
            )
        ]
    return []


def junk_findings(root: Path, config: dict) -> list[Finding]:
    """Fail on CAD lock, backup, and autosave files.

    KiCad junk is in the scheme too: ``*-backups/`` and ``.history/``
    folders, ``fp-info-cache``, ``#auto_saved_files#``, ``_autosave-*``,
    ``*.lck`` and ``*.kicad_prl``. KiCad keeps ``.history/`` as a git
    repository, so git lists it as one folder entry, not as files.
    """
    suffixes = tuple(config.get("junk_suffixes", [".bak"]))
    names = set(config.get("junk_names", []))
    prefixes = ("~$", *config.get("junk_prefixes", []))
    dir_names = set(config.get("junk_dir_names", ["autosave"]))
    dir_suffixes = tuple(config.get("junk_dir_suffixes", []))
    findings: list[Finding] = []
    for path in tracked_files(root):
        if not path.is_file() or _skipped(path):
            continue
        name = path.name
        rel_parts = path.relative_to(root).parts
        folders = rel_parts[:-1]
        if (
            name.startswith(prefixes)
            or name.endswith(suffixes)
            or name in names
            or dir_names.intersection(rel_parts)
            or (dir_suffixes and any(d.endswith(dir_suffixes) for d in folders))
        ):
            findings.append(
                Finding(_rel(path, root), 1, "CAD lock, temp, or backup file")
            )
    for rel in git_listing(root) or []:
        folder = root / rel.rstrip("/")
        if not folder.is_dir() or _skipped(folder):
            continue
        parts = PurePosixPath(rel).parts
        if dir_names.intersection(parts) or any(
            d.endswith(dir_suffixes) for d in parts if dir_suffixes
        ):
            findings.append(Finding(_rel(folder, root) + "/", 1, "CAD backup folder"))
    return findings


def _attribute_lines(text: str) -> list[tuple[str, list[str]]]:
    """Return ``(pattern, attributes)`` for each ``.gitattributes`` line.

    A line that starts with ``#`` is a comment. A pattern in double
    quotes may hold spaces.
    """
    lines: list[tuple[str, list[str]]] = []
    for raw in text.splitlines():
        line = raw.strip()
        if not line or line.startswith("#"):
            continue
        if line.startswith('"') and '"' in line[1:]:
            end = line.index('"', 1)
            lines.append((line[1:end], line[end + 1 :].split()))
        else:
            fields = line.split()
            lines.append((fields[0], fields[1:]))
    return lines


def _lfs_patterns(text: str) -> list[str]:
    """Return the ``.gitattributes`` patterns that set ``filter=lfs``.

    A pattern counts when it sets ``filter=lfs`` itself or names an
    ``[attr]`` macro that does.
    """
    lines = _attribute_lines(text)
    macros = {
        pattern.removeprefix("[attr]")
        for pattern, attrs in lines
        if pattern.startswith("[attr]") and "filter=lfs" in attrs
    }
    return [
        pattern
        for pattern, attrs in lines
        if not pattern.startswith("[attr]")
        and ("filter=lfs" in attrs or macros.intersection(attrs))
    ]


def _pattern_takes(pattern: str, ext: str) -> bool:
    """Return true when a ``.gitattributes`` pattern can match ``*<ext>``.

    The last path part is compared, so ``**/*.kicad_sch``,
    ``ecad/*.kicad_sch``, ``*.kicad_*`` and ``board.kicad_pcb`` all count.
    """
    last = pattern.rstrip("/").rsplit("/", 1)[-1]
    return last.endswith(ext) or fnmatch.fnmatchcase(f"x{ext}", last)


def text_cad_lfs_findings(root: Path, config: dict) -> list[Finding]:
    """Warn when Git LFS holds a text CAD format.

    KiCad source files are text, so Git can diff and merge them. Every
    ``.gitattributes`` in the tree is read, not only the root one, and a
    KiCad file that is already a Git LFS pointer is reported too.
    """
    extensions = tuple(config.get("cad_text_extensions", []))
    findings: list[Finding] = []
    attribute_files = [
        path
        for path in tracked_files(root)
        if path.name == ".gitattributes" and path.is_file() and not _skipped(path)
    ]
    for attributes in attribute_files:
        lfs = _lfs_patterns(attributes.read_text(encoding="utf-8"))
        for ext in extensions:
            hits = [pattern for pattern in lfs if _pattern_takes(pattern, ext)]
            if hits:
                findings.append(
                    Finding(
                        _rel(attributes, root),
                        1,
                        f"*{ext} is text CAD. Do not store it in Git LFS "
                        f"({', '.join(hits)}).",
                        "warning",
                    )
                )
    for path in tracked_files(root, extensions):
        if not path.is_file() or _skipped(path):
            continue
        with path.open("rb") as handle:
            head = handle.read(len(LFS_POINTER))
        if head.startswith(LFS_POINTER):
            findings.append(
                Finding(
                    _rel(path, root),
                    1,
                    "text CAD file is a Git LFS pointer. Do not store it in Git LFS.",
                    "warning",
                )
            )
    return findings


def gitattributes_findings(root: Path, config: dict) -> list[Finding]:
    """Require an LFS pattern when a native CAD file is present."""
    extensions = set(config.get("cad_extensions", []))
    present: set[str] = set()
    for path in tracked_files(root):
        if not path.is_file() or _skipped(path):
            continue
        if path.suffix.lower() in extensions:
            present.add(path.suffix.lower())
    if not present:
        return []
    attributes = root / ".gitattributes"
    text = attributes.read_text(encoding="utf-8") if attributes.is_file() else ""
    findings: list[Finding] = []
    for ext in sorted(present):
        if f"*{ext}" not in text or "filter=lfs" not in text:
            findings.append(
                Finding(
                    ".gitattributes",
                    1,
                    f"track *{ext} with Git LFS before you add the file",
                )
            )
    return findings


def attributes_template_findings(path: Path, config: dict) -> list[Finding]:
    """Check the copyable CAD attributes template."""
    if not path.is_file():
        return [Finding(path.as_posix(), 1, "CAD .gitattributes template is missing")]
    text = path.read_text(encoding="utf-8")
    findings: list[Finding] = []
    for ext in config.get("cad_extensions", []):
        if f"*{ext}" not in text or "filter=lfs" not in text:
            findings.append(
                Finding(path.as_posix(), 1, f"template does not track *{ext}")
            )
    if "git lfs migrate" in text:
        findings.append(
            Finding(path.as_posix(), 1, "template must not run git lfs migrate")
        )
    lfs = _lfs_patterns(text)
    for ext in config.get("cad_text_extensions", []):
        if any(_pattern_takes(pattern, ext) for pattern in lfs):
            findings.append(
                Finding(path.as_posix(), 1, f"template must not put *{ext} in LFS")
            )
    return findings


def eco_findings(path: Path) -> list[Finding]:
    """Check that an ECO template still has the required labels."""
    if not path.is_file():
        return [Finding(path.as_posix(), 1, "ECO template is missing")]
    text = path.read_text(encoding="utf-8")
    return [
        Finding(path.as_posix(), 1, f"ECO template is missing {label}")
        for label in ECO_LABELS
        if label not in text
    ]


def lint_repo(
    root: Path,
    config: dict,
    *,
    bom_roots: list[Path] | None = None,
    check_tree: bool = True,
    attributes_template: Path | None = None,
    eco_template: Path | None = None,
    max_bytes: int | None = None,
) -> list[Finding]:
    """Lint BOMs and the CAD-data checks for one repository root."""
    findings: list[Finding] = []
    for path in discover_boms(bom_roots or [root], root):
        findings.extend(lint_bom(path, config, root))
    if check_tree:
        limit = int(max_bytes if max_bytes is not None else config["lfs_max_bytes"])
        findings.extend(large_file_findings(root, limit))
        findings.extend(
            tree_byte_findings(
                root, int(config["warn_bytes"]), int(config["fail_bytes"])
            )
        )
        findings.extend(junk_findings(root, config))
        findings.extend(gitattributes_findings(root, config))
        findings.extend(text_cad_lfs_findings(root, config))
    if attributes_template is not None:
        findings.extend(attributes_template_findings(attributes_template, config))
    if eco_template is not None:
        findings.extend(eco_findings(eco_template))
    return findings


def main(argv: list[str] | None = None) -> int:
    """Lint BOM files and CAD-data hygiene. Return 0 when no error is present."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("paths", nargs="*", type=Path, help="BOM files or directories")
    parser.add_argument("--config", type=Path, default=None)
    parser.add_argument("--root", type=Path, default=Path())
    parser.add_argument("--max-bytes", type=int, default=None)
    parser.add_argument("--skip-tree", action="store_true")
    args = parser.parse_args(argv)
    root = args.root.resolve()
    config = load_config(args.config or default_config_path())
    attributes = root / "templates" / "cad-data" / ".gitattributes"
    eco = root / "templates" / "cad-data" / "eco.md"
    findings = lint_repo(
        root,
        config,
        bom_roots=args.paths or None,
        check_tree=not args.skip_tree,
        attributes_template=attributes if attributes.is_file() else None,
        eco_template=eco if eco.is_file() else None,
        max_bytes=args.max_bytes,
    )
    errors = [item for item in findings if item.severity == "error"]
    for item in findings:
        print(item, file=sys.stderr if item.severity == "warning" else sys.stdout)
    if errors:
        print(f"{len(errors)} error(s)", file=sys.stderr)
        return 1
    print(f"ok: {len(findings)} warning(s)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
