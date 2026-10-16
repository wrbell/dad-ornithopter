#!/usr/bin/env python3
"""List the files a checker reads: git's view of a tree, or a plain walk.

``tracked_files(root)`` returns the files under ``root`` in one of three
ways, in this order:

1. ``root`` is inside a git work tree: ``git ls-files -z --cached --others
   --exclude-standard`` run in ``root``. That is every tracked file plus
   every untracked file that ``.gitignore`` does not exclude, so a file is
   checked before ``git add`` and an ignored folder (``dist/``,
   ``.virtual_documents/``, ``.local-evidence/``, a case-output folder) is
   never read. A CI checkout sees the same set minus untracked files.
2. git cannot list ``root`` but a repository is found above it: an
   ``os.walk`` that drops every path ``git check-ignore`` accepts.
3. Otherwise: an ``os.walk`` that skips ``.git``, ``node_modules``,
   ``.venv``, ``venv``, ``.cache`` and ``third_party``.

Every git failure (no git binary, a broken or empty ``.git``, a non-zero
exit) falls through to the next way. The function never raises for git.

The result is a sorted list of ``root / relative_path`` paths, so it is a
drop-in for ``sorted(p for p in root.rglob("*") if p.is_file())``.
A checker still applies its own skips (``.agents/``, fixture ``fail``
trees) to the result.

Python 3.11 standard library only.
"""

# The only subprocess is git, resolved to an absolute path.
# ruff: noqa: S603

from __future__ import annotations

import os
import shutil
import subprocess
from collections.abc import Iterable
from pathlib import Path

WALK_SKIP = frozenset(
    {".git", "node_modules", ".venv", "venv", ".cache", "third_party"}
)


def _git(cwd: Path, *args: str, stdin: bytes | None = None) -> tuple[int, bytes]:
    """Return ``(exit code, stdout)`` of ``git -C cwd args``; 128 on failure."""
    git = shutil.which("git")
    if git is None:
        return 128, b""
    try:
        proc = subprocess.run(
            [git, "-C", str(cwd), *args],
            input=stdin,
            capture_output=True,
            check=False,
        )
    except OSError:
        return 128, b""
    return proc.returncode, proc.stdout


def _split(out: bytes) -> list[str]:
    return [p for p in out.decode("utf-8", "surrogateescape").split("\0") if p]


def git_listing(root: Path) -> list[str] | None:
    """Return git's file list for ``root``, relative to it, or ``None``.

    Parameters
    ----------
    root : pathlib.Path
        A directory.

    Returns
    -------
    list of str or None
        Tracked plus untracked-but-not-ignored paths, POSIX form, relative
        to ``root``. ``None`` when ``root`` is not inside a git work tree
        or git fails.
    """
    code, out = _git(root, "rev-parse", "--is-inside-work-tree")
    if code != 0 or out.strip() != b"true":
        return None
    code, out = _git(
        root, "ls-files", "-z", "--cached", "--others", "--exclude-standard"
    )
    if code != 0:
        return None
    return sorted(set(_split(out)))


def _repo_above(root: Path) -> Path | None:
    for folder in (root, *root.parents):
        if (folder / ".git").exists():
            return folder
    return None


def _ignored(top: Path, paths: list[Path]) -> set[Path] | None:
    """Return the paths ``git check-ignore`` accepts, or ``None`` on error."""
    if not paths:
        return set()
    stdin = b"\0".join(os.fsencode(str(p)) for p in paths) + b"\0"
    code, out = _git(top, "check-ignore", "-z", "--stdin", stdin=stdin)
    if code == 1:
        return set()
    if code != 0:
        return None
    return {Path(p) for p in _split(out)}


def _walk_ignoring(root: Path, top: Path) -> list[str] | None:
    """Walk ``root`` and drop what git ignores in the repository at ``top``."""
    found: list[str] = []
    for dirpath, dirnames, filenames in os.walk(root):
        here = Path(dirpath)
        dirnames[:] = sorted(d for d in dirnames if d != ".git")
        filenames = [f for f in filenames if f != ".git"]
        candidates = [here / d for d in dirnames] + [here / f for f in filenames]
        ignored = _ignored(top, candidates)
        if ignored is None:
            return None
        dirnames[:] = [d for d in dirnames if here / d not in ignored]
        for name in filenames:
            if here / name not in ignored:
                found.append((here / name).relative_to(root).as_posix())
    return sorted(found)


def _walk(root: Path) -> list[str]:
    found: list[str] = []
    for dirpath, dirnames, filenames in os.walk(root):
        here = Path(dirpath)
        dirnames[:] = sorted(d for d in dirnames if d not in WALK_SKIP)
        for name in filenames:
            if name == ".git":
                continue
            found.append((here / name).relative_to(root).as_posix())
    return sorted(found)


def _relative_files(root: Path) -> list[str]:
    listed = git_listing(root)
    if listed is not None:
        return listed
    top = _repo_above(root.resolve())
    if top is not None:
        walked = _walk_ignoring(root.resolve(), top)
        if walked is not None:
            return walked
    return _walk(root)


def tracked_files(
    root: Path | str, suffixes: Iterable[str] | None = None
) -> list[Path]:
    """Return the files under ``root`` that git tracks or would track.

    Parameters
    ----------
    root : pathlib.Path or str
        A directory (or one file, returned as is when its suffix matches).
    suffixes : iterable of str, optional
        Keep only files whose suffix is in this set, compared in lower case
        (``(".csv",)``). ``None`` keeps every file.

    Returns
    -------
    list of pathlib.Path
        ``root / relative_path`` for each regular file, sorted by the
        relative path. Entries git lists that are not files on disk
        (deleted files, submodules) are dropped.
    """
    root = Path(root)
    wanted = None if suffixes is None else {s.lower() for s in suffixes}
    if root.is_file():
        if wanted is None or root.suffix.lower() in wanted:
            return [root]
        return []
    if not root.is_dir():
        return []
    files: list[Path] = []
    for rel in _relative_files(root):
        path = root / rel
        if wanted is not None and path.suffix.lower() not in wanted:
            continue
        if path.is_file():
            files.append(path)
    return files
