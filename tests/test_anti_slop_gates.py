#!/usr/bin/env python3
"""
tests/test_anti_slop_gates.py
=============================
Automated structural gates that prevent vibe-coded bloat, duplicate function
redefinitions, hardcoded user paths, and broken documentation links from
re-entering the repository.
"""

from __future__ import annotations

import sys
import unittest
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT))

from scripts import mco_cli


class TestAntiSlopGates(unittest.TestCase):
    def test_no_duplicate_top_level_ast_symbols_in_active_modules(self) -> None:
        dup_errors, _ = mco_cli._check_ast_duplicates()
        self.assertEqual(
            dup_errors,
            [],
            f"Duplicate top-level function/class definitions found: {dup_errors}",
        )

    def test_no_hardcoded_user_paths(self) -> None:
        bad_paths = mco_cli._check_hardcoded_user_paths()
        self.assertEqual(
            bad_paths,
            [],
            f"Hardcoded user paths found in active files: {bad_paths}",
        )

    def test_ssot_sync_gate(self) -> None:
        errors = mco_cli._check_ssot_sync()
        self.assertEqual(errors, [], f"Single Source of Truth sync errors: {errors}")

    def test_active_markdown_links_resolve(self) -> None:
        md_files = [
            REPO_ROOT / "README.md",
            REPO_ROOT / "INDEX.md",
            REPO_ROOT / "PROJECT_STATE.md",
            REPO_ROOT / "CONTRIBUTING.md",
            REPO_ROOT / "docs" / "README.md",
            *sorted((REPO_ROOT / "docs" / "mark1").glob("*.md")),
            *sorted(REPO_ROOT.rglob("AGENTS.md")),
        ]
        broken: list[str] = []
        for md in md_files:
            broken.extend(mco_cli._check_markdown_links(md))
        self.assertEqual(broken, [], f"Broken markdown links found: {broken}")


if __name__ == "__main__":
    unittest.main()
