#!/usr/bin/env python3
"""Check AI instruction files for lines that go stale or point nowhere.

``agents_md_lint.py`` checks that ``AGENTS.md`` exists, the root
``AGENTS.md`` length, the Clarity section, ``@AGENTS.md`` in
``CLAUDE.md`` and ``.mdc`` scoping. This script does not repeat those
checks. It checks every ``AGENTS.md``, ``CLAUDE*.md``, ``GEMINI.md`` and
``.cursor/rules/**/*.mdc`` under each PATH (a PATH can also be one file)
for:

- a status phrase: ``in progress``, ``not drafted`` (also ``not been
  drafted``), ``% complete``, ``auto mode``, ``WIP``
- a dated due line: ``Due`` (not ``Due to``), ``due:``, or a weekday,
  month and day such as ``Thu Oct 1``
- a machine path: ``/mnt/...``, ``/Users/...``, a Windows drive path
  (``C:`` and a backslash) or ``~/.claude/plans/...``, except an allowed
  prefix
- an ``@path`` import whose target does not exist
- a pointer in backticks to an AI file in another folder, such as
  ``pipeline/AGENTS.md``, that does not exist in the file's folder or
  any folder above it up to PATH (``standards/...`` paths are exempt)
- a relative Markdown link whose target does not exist
- byte-identical AI files (one line per group); a pointer file, whose
  only lines outside the managed block are ``@`` imports, is exempt
- more than 200 lines; ``PATH/AGENTS.md`` is left to
  ``agents_md_lint.py``

The managed block (``<!-- standards:begin -->`` to
``<!-- standards:end -->``) is not checked here; ``apply_standards.py
--check`` owns it. Text in backticks and in fenced code blocks is not
checked for status phrases, due lines, imports or links, so a rule can
quote a phrase. Machine paths are checked in code too.
``/Users/willem/Code/standards`` is always allowed; ``--allow-abs-path``
adds another prefix.

Discovery does not enter ``.git/``, ``third_party/``, ``node_modules/``,
``venv/``, ``.venv/``, ``.cache/``, ``site-packages/``, ``vendor/``,
``stt_env/``, ``.local-evidence/`` or ``.claude/worktrees/``.

Usage::

    python3 tools/ai_file_lint.py PATH... [--allow-abs-path PATH]...

The exit code is 1 when a problem is found and 2 when a PATH does not
exist. The script uses the Python 3.11 standard library only.
"""

from __future__ import annotations

import argparse
import hashlib
import os
import re
import sys
from dataclasses import dataclass
from pathlib import Path
from urllib.parse import unquote

MAX_LINES = 200
DEFAULT_ALLOWED = ("/Users/willem/Code/standards",)
BEGIN = "<!-- standards:begin -->"
END = "<!-- standards:end -->"
SKIP_DIR_NAMES = frozenset(
    {
        ".git",
        "third_party",
        "node_modules",
        "venv",
        ".venv",
        ".cache",
        "site-packages",
        "vendor",
        "stt_env",
        ".local-evidence",
        "__pycache__",
    }
)
STATUS_PATTERNS = (
    ("in progress", re.compile(r"\bin progress\b", re.I)),
    (
        "not drafted",
        re.compile(r"\bnot\s+(?:yet\s+)?(?:been\s+)?drafted\b", re.I),
    ),
    ("% complete", re.compile(r"%\s*complete\b", re.I)),
    ("auto mode", re.compile(r"\bauto mode\b", re.I)),
    ("WIP", re.compile(r"\bWIP\b")),
)
WEEKDAY = (
    r"(?:Monday|Tuesday|Wednesday|Thursday|Friday|Saturday|Sunday"
    r"|Mon|Tues|Tue|Wed|Thurs|Thur|Thu|Fri|Sat|Sun)"
)
MONTH = (
    r"(?:January|February|March|April|May|June|July|August|September"
    r"|October|November|December|Jan|Feb|Mar|Apr|Jun|Jul|Aug|Sept|Sep"
    r"|Oct|Nov|Dec)"
)
DUE_PATTERNS = (
    re.compile(r"(?<![A-Za-z])Due(?=[\s:])(?!\s+to\b)"),
    re.compile(r"\bdue:", re.I),
    re.compile(rf"\b{WEEKDAY}\.?,?\s+{MONTH}\.?\s+\d{{1,2}}(?!\d)"),
)
PATH_END = r"[^\s`'\"()<>\[\]|*]"
MACHINE_PATTERNS = (
    re.compile(rf"(?<![\w.:/~-])/Users/{PATH_END}+"),
    re.compile(rf"(?<![\w.:/~-])/mnt/{PATH_END}+"),
    re.compile(r"(?<![A-Za-z0-9])[A-Za-z]:\\[^\s`'\"]+"),
    re.compile(rf"~/\.claude/plans/{PATH_END}+"),
)
IMPORT_RE = re.compile(r"(?<![\w@./+-])@([^\s`]+)")
POINTER_RE = re.compile(
    r"^(?![/~]|standards/)[^`*<>{}$]*/(?:AGENTS|CLAUDE|GEMINI)\.md$"
)
LINK_RE = re.compile(r"\[[^\]]*\]\(\s*(<[^>]*>|[^)\s]+)[^)]*\)")
REF_DEF_RE = re.compile(r"^\s{0,3}\[[^\]]+\]:\s*(<[^>]*>|\S+)")
CODE_SPAN_RE = re.compile(r"(`+)(.*?)\1")
FENCE_RE = re.compile(r"^\s{0,3}(```|~~~)")
SCHEME_RE = re.compile(r"^[A-Za-z][A-Za-z0-9+.-]*:")
TRAILING = ".,;:)]}'\"*_\\"


@dataclass(frozen=True)
class Problem:
    """Hold one AI-file lint problem."""

    path: str
    line: int
    message: str

    def __str__(self) -> str:
        return f"{self.path}:{self.line}: {self.message}"


@dataclass(frozen=True)
class AiFile:
    """Hold one discovered AI file."""

    path: Path
    shown: str
    root_agents: bool
    root: Path


def is_ai_file(rel: str) -> bool:
    """Return True when a relative POSIX path names an AI file.

    Parameters
    ----------
    rel : str
        Path relative to the scan root.

    Returns
    -------
    bool
        True for ``AGENTS.md``, ``CLAUDE*.md``, ``GEMINI.md`` and
        ``.cursor/rules/**/*.mdc``.
    """
    name = rel.rsplit("/", 1)[-1]
    if name in {"AGENTS.md", "GEMINI.md"}:
        return True
    if name.startswith("CLAUDE") and name.endswith(".md"):
        return True
    return name.endswith(".mdc") and f"/{rel}".find("/.cursor/rules/") != -1


def _skip_dir(parent: Path, name: str) -> bool:
    if name in SKIP_DIR_NAMES:
        return True
    return parent.name == ".claude" and name == "worktrees"


def discover(target: Path, shown: str) -> list[AiFile]:
    """Return the AI files under one PATH argument.

    Parameters
    ----------
    target : Path
        A folder or one file.
    shown : str
        The PATH as given, used as the prefix in messages.

    Returns
    -------
    list of AiFile
        The files, sorted by path.
    """
    if target.is_file():
        return [AiFile(target, shown, root_agents=False, root=target.parent)]
    found: list[AiFile] = []
    for dirpath, dirnames, filenames in os.walk(target):
        here = Path(dirpath)
        dirnames[:] = sorted(d for d in dirnames if not _skip_dir(here, d))
        for name in sorted(filenames):
            path = here / name
            rel = path.relative_to(target).as_posix()
            if is_ai_file(rel):
                shown_path = str(Path(shown) / rel)
                found.append(AiFile(path, shown_path, rel == "AGENTS.md", target))
    return found


def checked_lines(text: str) -> list[tuple[int, str, str]]:
    """Return the lines that content checks read.

    Parameters
    ----------
    text : str
        The file text.

    Returns
    -------
    list of tuple of (int, str, str)
        Line number, the raw line, and the prose part (code spans and
        fenced blocks blanked). Lines in the managed block are left out.
    """
    out: list[tuple[int, str, str]] = []
    in_block = False
    in_fence = False
    for number, raw in enumerate(text.splitlines(), start=1):
        stripped = raw.strip()
        if stripped == BEGIN:
            in_block = True
            continue
        if stripped == END:
            in_block = False
            continue
        if in_block:
            continue
        if FENCE_RE.match(raw):
            in_fence = not in_fence
            out.append((number, raw, ""))
            continue
        prose = "" if in_fence else CODE_SPAN_RE.sub(" ", raw)
        out.append((number, raw, prose))
    return out


def is_pointer(text: str) -> bool:
    """Return True when every line outside the managed block is an import.

    Parameters
    ----------
    text : str
        The file text.

    Returns
    -------
    bool
        True for a pointer file such as ``@AGENTS.md`` plus the block.
    """
    lines = [raw.strip() for _, raw, _ in checked_lines(text) if raw.strip()]
    return bool(lines) and all(
        line.startswith("@") and " " not in line for line in lines
    )


def _allowed(path: str, allowed: tuple[str, ...]) -> bool:
    for prefix in allowed:
        base = prefix.rstrip("/\\")
        if path == base or path.startswith((base + "/", base + "\\")):
            return True
    return False


def _resolve(source: Path, target: str) -> Path:
    if target.startswith("~/"):
        return Path.home() / target[2:]
    if target.startswith("/"):
        return Path(target)
    return source.parent / target


def _link_target(raw: str) -> str | None:
    target = raw.strip()
    if target.startswith("<") and target.endswith(">"):
        target = target[1:-1].strip()
    if not target or target.startswith(("#", "/", "~", "{", "$")):
        return None
    if SCHEME_RE.match(target):
        return None
    target = target.split("#", 1)[0].split("?", 1)[0]
    return unquote(target) or None


def _pointer_exists(path: Path, target: str, root: Path | None) -> bool:
    folder = path.parent
    while True:
        if (folder / target).exists():
            return True
        if root is None or folder == root or folder.parent == folder:
            return False
        folder = folder.parent


def check_text(
    path: Path, text: str, allowed: tuple[str, ...], root: Path | None = None
) -> list[Problem]:
    """Run the line checks on one file.

    Parameters
    ----------
    path : Path
        The file on disk. Imports and links resolve from its folder.
    text : str
        The file text.
    allowed : tuple of str
        Allowed machine-path prefixes.
    root : Path or None
        The scan root. A backticked AI-file pointer may resolve from any
        folder between the file and this root.

    Returns
    -------
    list of Problem
        Problems with an empty path; the caller fills it in.
    """
    problems: list[Problem] = []
    for number, raw, prose in checked_lines(text):
        for label, pattern in STATUS_PATTERNS:
            if pattern.search(prose):
                problems.append(
                    Problem("", number, f"status phrase '{label}'; use the README")
                )
        if any(pattern.search(prose) for pattern in DUE_PATTERNS):
            problems.append(
                Problem("", number, "dated due line; dates go in the README")
            )
        for pattern in MACHINE_PATTERNS:
            for match in pattern.finditer(raw):
                found = match.group(0).rstrip(TRAILING)
                if not _allowed(found, allowed):
                    problems.append(Problem("", number, f"machine path {found}"))
        for match in IMPORT_RE.finditer(prose):
            target = match.group(1).rstrip(TRAILING)
            if not re.search(r"[/.~]", target):
                continue
            if not _resolve(path, target).exists():
                problems.append(Problem("", number, f"@ import not found: {target}"))
        spans = [] if not prose else [m.group(2) for m in CODE_SPAN_RE.finditer(raw)]
        for span in spans:
            target = span.strip()
            if POINTER_RE.match(target) and not _pointer_exists(path, target, root):
                problems.append(Problem("", number, f"pointer not found: {target}"))
        targets = [m.group(1) for m in LINK_RE.finditer(prose)]
        ref = REF_DEF_RE.match(prose)
        if ref is not None:
            targets.append(ref.group(1))
        for raw_target in targets:
            target = _link_target(raw_target)
            if target is not None and not _resolve(path, target).exists():
                problems.append(Problem("", number, f"link target not found: {target}"))
    return problems


def check_paths(targets: list[str], allowed: tuple[str, ...]) -> list[Problem]:
    """Check every AI file under the given PATH arguments.

    Parameters
    ----------
    targets : list of str
        Folders or files, as given on the command line.
    allowed : tuple of str
        Allowed machine-path prefixes.

    Returns
    -------
    list of Problem
        Every problem, sorted by path and line.
    """
    files: list[AiFile] = []
    for target in targets:
        files.extend(discover(Path(target), target))
    problems: list[Problem] = []
    by_hash: dict[str, list[AiFile]] = {}
    for item in files:
        data = item.path.read_bytes()
        text = data.decode("utf-8", errors="replace")
        for problem in check_text(item.path, text, allowed, item.root):
            problems.append(Problem(item.shown, problem.line, problem.message))
        count = len(text.splitlines())
        if count > MAX_LINES and not item.root_agents:
            problems.append(Problem(item.shown, 1, f"{count} lines (>{MAX_LINES})"))
        if not is_pointer(text):
            by_hash.setdefault(hashlib.sha256(data).hexdigest(), []).append(item)
    for group in by_hash.values():
        if len(group) > 1:
            others = ", ".join(item.shown for item in group[1:])
            problems.append(Problem(group[0].shown, 1, f"byte-identical to {others}"))
    return sorted(problems, key=lambda p: (p.path, p.line, p.message))


def main(argv: list[str] | None = None) -> int:
    """Check AI files and return an exit code."""
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("paths", nargs="+", help="folders or files to check")
    parser.add_argument(
        "--allow-abs-path",
        action="append",
        default=[],
        metavar="PATH",
        help="another allowed absolute-path prefix (repeatable)",
    )
    args = parser.parse_args(argv)
    missing = [p for p in args.paths if not Path(p).exists()]
    if missing:
        for path in missing:
            print(f"{path}: not found", file=sys.stderr)
        return 2
    allowed = DEFAULT_ALLOWED + tuple(args.allow_abs_path)
    problems = check_paths(args.paths, allowed)
    if problems:
        for problem in problems:
            print(problem)
        print(f"{len(problems)} problem(s)", file=sys.stderr)
        return 1
    count = sum(len(discover(Path(p), p)) for p in args.paths)
    print(f"ok: {count} AI file(s)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
