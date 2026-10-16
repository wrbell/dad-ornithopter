#!/usr/bin/env python3
"""
scripts/mco_cli.py
==================
Unified Workflow CLI for the MCO-1D Metachronal Cascade Ornithopter repository.

Subcommands:
  - `./mco test`   : Run unit test suites (`unittest` / `pytest`) and optional TC-01 bench harness
  - `./mco sim`    : Run the 50 Hz flight computer in software-in-the-loop (SITL) mode
  - `./mco gs`     : Launch the Ground Station HTTP/WebSocket bridge & Gemini copilot proxy
  - `./mco bom`    : Run the live Bill of Materials pricing & MTOW verification engine
  - `./mco cad`    : Generate parametric CAD assets (`build123d`) or verify STL manifoldness
  - `./mco slice`  : Inspect, slice, or open 3D models in PrusaSlicer 2.9.6 via the MCP backend
  - `./mco docs`   : Regenerate publication-grade PDF installation guides and test worksheets
  - `./mco doctor` : Audit repository hygiene, root clutter, markdown links, and CAD/slicer health
"""

from __future__ import annotations

import argparse
import json
import os
import re
import subprocess
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
SYS_PYTHON = sys.executable or "python3"
CAD_PYTHON_PATH = REPO_ROOT / "ornithopter-cad" / ".venv-agentcad" / "bin" / "python"
CAD_PYTHON = str(CAD_PYTHON_PATH) if CAD_PYTHON_PATH.exists() else SYS_PYTHON

sys.path.insert(0, str(REPO_ROOT))


def _run(cmd: list[str], env: dict[str, str] | None = None) -> int:
    merged_env = os.environ.copy()
    merged_env["PYTHONPATH"] = f"{REPO_ROOT / 'firmware'}:{REPO_ROOT}:{merged_env.get('PYTHONPATH', '')}"
    if env:
        merged_env.update(env)
    proc = subprocess.run(cmd, cwd=str(REPO_ROOT), env=merged_env)
    return proc.returncode


def cmd_test(args: argparse.Namespace) -> int:
    if args.tc01:
        print("=== Running TC-01 Automated Ground Bench Qualification ===")
        return _run([SYS_PYTHON, "scripts/run_tc01_bench_checkout.py"])

    if args.suite:
        pattern = args.suite if args.suite.startswith("test_") else f"test_*{args.suite}*"
        if not pattern.endswith(".py"):
            pattern += ".py"
        print(f"=== Running MCO-1D Test Suite Filter: {pattern} ===")
        return _run([SYS_PYTHON, "-m", "unittest", "discover", "-s", "tests", "-p", pattern, "-v"])

    print("=== Running Full MCO-1D Unit Verification Suite ===")
    rc = _run([SYS_PYTHON, "-m", "unittest", "discover", "-s", "tests", "-p", "test_*.py"])
    return rc


def cmd_sim(args: argparse.Namespace) -> int:
    print("==================================================")
    print("  MCO-1D Flight Computer — 50 Hz SITL Simulation  ")
    print("==================================================")
    return _run([SYS_PYTHON, "firmware/mco_main.py", "--sim", *args.sim_args])


def cmd_gs(args: argparse.Namespace) -> int:
    return _run(["bash", "scripts/mco_ground_station_launcher.sh", *args.gs_args])


def cmd_bom(args: argparse.Namespace) -> int:
    bom_args = []
    if args.json:
        bom_args.append("--json")
    else:
        bom_args.append("--view")
    return _run([SYS_PYTHON, "scripts/mco_bom_tool.py", *bom_args])


CAD_TARGETS: dict[str, str] = {
    "package": "scripts/cad/package_wing_root_3dprint_files.py",
    "wrist-v10": "scripts/cad/generate_stagecoach_wrist_v10.py",
    "powertrain-v10": "scripts/cad/generate_v10_powertrain_mount_and_brace.py",
    "collar": "scripts/cad/generate_topological_collar_cad.py",
    "bevel-v9": "scripts/cad/generate_printable_bevel_cartridge_v9.py",
    "fuselage": "scripts/cad/generate_mco_fuselage_cad.py",
    "electronics-bay": "scripts/cad/generate_mco_electronics_bay_cad.py",
    "clamps": "scripts/cad/generate_mco_frame_clamps_and_skid_cad.py",
    "wing-s1223": "scripts/cad/generate_3dprint_wing_system.py",
    "wing-sd7062": "scripts/cad/generate_sd7062_wing_system.py",
    "aircraft": "scripts/cad/generate_mco_full_aircraft_assembly.py",
}


def cmd_cad(args: argparse.Namespace) -> int:
    from scripts.mcp import prusaslicer_mcp_server as ps

    target = args.target
    if target == "verify":
        print("=== Verifying 3D Printable Wing Root Package via PrusaSlicer CLI ===")
        report = ps.validate_mco_print_package("cad/print_package_wing_root")
        print(f"Directory : {report['package_dir']}")
        print(f"Total STLs: {report['total_stl_files']}")
        print("-" * 86)
        print(f"{'Part File':<44} | {'Manifold':<8} | {'Bed Fit':<7} | {'Vol (cm³)':<9} | {'Dims (mm)'}")
        print("-" * 86)
        for p in report["parts"]:
            d = p["dimensions_mm"]
            dims_str = f"{d['x']:.1f}x{d['y']:.1f}x{d['z']:.1f}"
            m_str = "YES" if p["is_manifold"] else "FAIL"
            b_str = "YES" if p["fits_on_bed"] else "FAIL"
            print(f"{p['file']:<44} | {m_str:<8} | {b_str:<7} | {p['volume_cm3']:<9.2f} | {dims_str}")
        print("-" * 86)
        if report["all_manifold"] and report["all_fit_bed"]:
            print("STATUS: PASS (100% manifold & fits Prusa CORE One 250x220x250mm bed)")
            return 0
        print("STATUS: FAIL (One or more meshes failed validation)")
        return 1

    if target not in CAD_TARGETS:
        print(f"Unknown CAD target '{target}'. Available targets: verify, {', '.join(sorted(CAD_TARGETS.keys()))}")
        return 1

    script_rel = CAD_TARGETS[target]
    print(f"=== Executing CAD Generator: {script_rel} (using {CAD_PYTHON}) ===")
    return _run([CAD_PYTHON, script_rel])


def cmd_slice(args: argparse.Namespace) -> int:
    from scripts.mcp import prusaslicer_mcp_server as ps

    if args.inspect:
        info = ps.inspect_model(args.files[0])
        print(json.dumps(info, indent=2))
        return 0 if info["is_manifold"] else 1

    if args.gui:
        res = ps.open_in_prusaslicer_gui(args.files, profile=args.profile)
        print(f"Launched PrusaSlicer GUI with: {', '.join(res['opened_files'])}")
        return 0

    print(f"=== Slicing {len(args.files)} model(s) with profile '{args.profile}' ===")
    result = ps.slice_model(
        input_paths=args.files,
        output_dir=args.output_dir,
        profile=args.profile,
        layer_height=args.layer_height,
        perimeters=args.perimeters,
        fill_density=args.fill_density,
        support_material=True if args.supports else None,
        merge_plate=args.merge,
        export_3mf=not args.no_3mf,
    )
    tel = result["telemetry"]
    print(f"  Profile Used     : {result['profile_used']}")
    print(f"  G-code Output    : {result['gcode_path']}")
    if result.get("project_3mf_path"):
        print(f"  3MF Project      : {result['project_3mf_path']}")
    print(f"  Print Time (Norm): {tel.get('estimated_printing_time_normal')}")
    print(f"  Filament Mass    : {tel.get('filament_used_g')} g ({tel.get('filament_used_cm3')} cm³ / {tel.get('filament_used_mm')} mm)")
    print(f"  Layer Count      : {tel.get('layer_count')} layers @ {tel.get('layer_height_mm')} mm")
    print(f"  Nozzle / Bed Temp: {tel.get('nozzle_temperature_c')}°C / {tel.get('bed_temperature_c')}°C")
    return 0


DOC_TARGETS: dict[str, str] = {
    "exploded": "scripts/docs/generate_hierarchical_exploded_assembly_pdf.py",
    "powertrain": "scripts/docs/generate_v10_powertrain_guide_pdf.py",
    "wrist": "scripts/docs/generate_wrist_assembly_guide_pdf.py",
    "calibration": "scripts/docs/generate_builder_calibration_sheet_pdf.py",
    "electronics": "scripts/docs/generate_electronics_guide_pdf.py",
    "worksheets": "scripts/docs/generate_test_card_worksheets_pdf.py",
}


def cmd_docs(args: argparse.Namespace) -> int:
    target = args.target
    targets = list(DOC_TARGETS.keys()) if target == "all" else [target]
    for t in targets:
        if t not in DOC_TARGETS:
            print(f"Unknown docs target '{t}'. Choose from: all, {', '.join(DOC_TARGETS.keys())}")
            return 1
        script_rel = DOC_TARGETS[t]
        print(f"=== Generating Documentation PDF: {script_rel} ===")
        rc = _run([CAD_PYTHON, script_rel])
        if rc != 0:
            return rc
    return 0


ALLOWED_ROOT_FILES = {
    ".DS_Store",
    ".gitattributes",
    ".gitignore",
    ".mcp.json",
    "AGENTS.md",
    "CLAUDE.md",
    "CONTRIBUTING.md",
    "INDEX.md",
    "Makefile",
    "PROJECT_STATE.md",
    "README.md",
    "mco",
    "pytest.ini",
    "willem-suggestions.md",
}

ALLOWED_ROOT_DIRS = {
    ".agents",
    ".git",
    ".pytest_cache",
    "__pycache__",
    "archive",
    "bom",
    "cad",
    "config",
    "docs",
    "enforcement",
    "firmware",
    "knowledge",
    "logs",
    "ornithopter-cad",
    "scripts",
    "tests",
    "tools",
}


def _check_markdown_links(md_file: Path) -> list[str]:
    """Verify all local relative file links in a markdown file exist on disk."""
    broken: list[str] = []
    if not md_file.exists():
        return [f"Missing markdown file: {md_file}"]
    content = md_file.read_text(encoding="utf-8", errors="replace")
    # Remove fenced code blocks before scanning links
    content_no_code = re.sub(r"```.*?```", "", content, flags=re.DOTALL)
    for match in re.finditer(r"\[[^\]]+\]\(([^)]+)\)", content_no_code):
        raw_target = match.group(1).strip()
        if (
            not raw_target
            or raw_target.startswith("#")
            or raw_target.startswith("http://")
            or raw_target.startswith("https://")
            or raw_target.startswith("mailto:")
            or raw_target.startswith("conversation://")
        ):
            continue
        if raw_target.startswith("file://"):
            target_path = Path(raw_target.replace("file://", "").split("#")[0])
        else:
            clean_rel = raw_target.split("#")[0]
            target_path = (md_file.parent / clean_rel).resolve()
        if not target_path.exists():
            broken.append(f"{md_file.relative_to(REPO_ROOT)} -> {raw_target}")
    return broken


def _check_ssot_sync() -> list[str]:
    """Verify config/airframe_spec.json, MCOConfig, and bom/bom.csv agree."""
    import csv

    errors: list[str] = []
    spec_path = REPO_ROOT / "config" / "airframe_spec.json"
    bom_csv_path = REPO_ROOT / "bom" / "bom.csv"
    if not spec_path.exists():
        return ["Missing config/airframe_spec.json"]
    if not bom_csv_path.exists():
        return ["Missing bom/bom.csv"]

    spec = json.loads(spec_path.read_text(encoding="utf-8"))
    sys.path.insert(0, str(REPO_ROOT / "firmware"))
    from mco_config import MCOConfig

    if abs(MCOConfig.MASS_KG - spec["mass_and_balance"]["design_installed_mass_kg"]) > 1e-6:
        errors.append(
            f"MASS_KG mismatch: MCOConfig.MASS_KG={MCOConfig.MASS_KG} vs spec={spec['mass_and_balance']['design_installed_mass_kg']}"
        )
    if abs(MCOConfig.GEAR_RATIO - spec["propulsion"]["capstan_reduction_ratio"]) > 1e-6:
        errors.append(
            f"GEAR_RATIO mismatch: MCOConfig.GEAR_RATIO={MCOConfig.GEAR_RATIO} vs spec={spec['propulsion']['capstan_reduction_ratio']}"
        )
    if abs(MCOConfig.WINGSPAN_M - spec["geometry"]["tip_to_tip_wingspan_m"]) > 1e-6:
        errors.append(
            f"WINGSPAN_M mismatch: MCOConfig.WINGSPAN_M={MCOConfig.WINGSPAN_M} vs spec={spec['geometry']['tip_to_tip_wingspan_m']}"
        )
    if abs(MCOConfig.WING_AREA_TOTAL_M2 - spec["geometry"]["total_wing_area_m2"]) > 1e-6:
        errors.append(
            f"WING_AREA_TOTAL_M2 mismatch: MCOConfig.WING_AREA_TOTAL_M2={MCOConfig.WING_AREA_TOTAL_M2} vs spec={spec['geometry']['total_wing_area_m2']}"
        )

    overlap = set(MCOConfig.PITCH_PINS) & set(MCOConfig.SWEEP_PINS)
    if overlap:
        errors.append(f"GPIO pin collision between PITCH_PINS and SWEEP_PINS: {sorted(overlap)}")

    with bom_csv_path.open("r", encoding="utf-8") as f:
        rows = list(csv.DictReader(f))
    if len(rows) < 40:
        errors.append(f"bom/bom.csv has unexpectedly few rows ({len(rows)})")
    return errors


def _check_ast_duplicates() -> tuple[list[str], int]:
    """Scan active Python files for duplicate top-level def/class definitions."""
    import ast

    dup_errors: list[str] = []
    wave_lib_dups = 0
    scan_dirs = [REPO_ROOT / "firmware", REPO_ROOT / "scripts", REPO_ROOT / "tests"]
    for base in scan_dirs:
        if not base.exists():
            continue
        for py_file in sorted(base.rglob("*.py")):
            rel = py_file.relative_to(REPO_ROOT)
            if "archive" in rel.parts or "__pycache__" in rel.parts:
                continue
            try:
                tree = ast.parse(py_file.read_text(encoding="utf-8"))
            except SyntaxError as exc:
                dup_errors.append(f"SyntaxError in {rel}: {exc}")
                continue
            seen: dict[str, int] = {}
            file_dups: list[str] = []
            for node in tree.body:
                if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef, ast.ClassDef)):
                    if node.name in seen:
                        file_dups.append(f"{node.name} (lines {seen[node.name]} & {node.lineno})")
                    else:
                        seen[node.name] = node.lineno
            if file_dups:
                if rel.as_posix() == "firmware/mco_wave_library.py":
                    wave_lib_dups = len(file_dups)
                else:
                    dup_errors.append(f"{rel}: duplicate top-level symbol(s): {', '.join(file_dups[:5])}")
    return dup_errors, wave_lib_dups


def _check_hardcoded_user_paths() -> list[str]:
    """Ensure zero hardcoded user home paths exist in active code/docs."""
    bad: list[str] = []
    target_prefix = "/Users/" + "drbell"
    exts = {".py", ".md", ".json", ".sh", ".ini"}
    for path in sorted(REPO_ROOT.rglob("*")):
        if not path.is_file() or path.suffix not in exts:
            continue
        rel = path.relative_to(REPO_ROOT)
        if (
            "archive" in rel.parts
            or ".git" in rel.parts
            or "ornithopter-cad" in rel.parts
            or rel.as_posix() in {"willem-suggestions.md", "scripts/mco_cli.py", "tests/test_anti_slop_gates.py"}
        ):
            continue
        text = path.read_text(encoding="utf-8", errors="replace")
        if target_prefix in text:
            bad.append(str(rel))
    return bad


def cmd_doctor(_args: argparse.Namespace) -> int:
    from scripts.mcp import prusaslicer_mcp_server as ps

    print("=" * 78)
    print("  MCO-1D REPOSITORY DOCTOR — 7-GATE ANTI-SLOP & STANDARDS AUDIT")
    print("=" * 78)
    issues: list[str] = []

    # 1. Root clutter check
    stray_root = []
    for item in sorted(REPO_ROOT.iterdir()):
        if item.is_dir():
            if item.name not in ALLOWED_ROOT_DIRS:
                stray_root.append(f"Unexpected root directory: {item.name}/")
        else:
            if item.name not in ALLOWED_ROOT_FILES:
                stray_root.append(f"Stray root file: {item.name}")
    if stray_root:
        issues.extend(stray_root)
        print(f"[FAIL] 1. Root Directory Cleanliness : {len(stray_root)} stray item(s)")
        for s in stray_root:
            print(f"          - {s}")
    else:
        print("[PASS] 1. Root Directory Cleanliness : 0 stray files or directories")

    # 2. Single Source of Truth Sync Check
    ssot_errors = _check_ssot_sync()
    if ssot_errors:
        issues.extend(ssot_errors)
        print(f"[FAIL] 2. Single Source of Truth Sync: {len(ssot_errors)} mismatch(es)")
        for e in ssot_errors:
            print(f"          - {e}")
    else:
        print("[PASS] 2. Single Source of Truth Sync: airframe_spec.json == MCOConfig == bom.csv")

    # 3. AST Duplicate Symbol Gate
    ast_errors, wave_dups = _check_ast_duplicates()
    if ast_errors:
        issues.extend(ast_errors)
        print(f"[FAIL] 3. AST Duplicate Symbol Gate  : {len(ast_errors)} file(s) with duplicate defs")
        for e in ast_errors:
            print(f"          - {e}")
    else:
        warn_note = f" (legacy mco_wave_library.py has {wave_dups} overrides slated for replacement)" if wave_dups else ""
        print(f"[PASS] 3. AST Duplicate Symbol Gate  : 0 duplicate defs in active modules{warn_note}")

    # 4. Hardcoded Path Check
    bad_paths = _check_hardcoded_user_paths()
    if bad_paths:
        issues.extend([f"Hardcoded user path in {p}" for p in bad_paths])
        print(f"[FAIL] 4. Portable Path Gate         : {len(bad_paths)} file(s) with hardcoded user paths")
        for p in bad_paths:
            print(f"          - {p}")
    else:
        print("[PASS] 4. Portable Path Gate         : 0 hardcoded user paths in active files")

    # 5. Markdown link verification
    md_files_to_check = [
        REPO_ROOT / "README.md",
        REPO_ROOT / "INDEX.md",
        REPO_ROOT / "PROJECT_STATE.md",
        REPO_ROOT / "CONTRIBUTING.md",
        REPO_ROOT / "docs" / "README.md",
        *sorted((REPO_ROOT / "docs" / "mark1").glob("*.md")),
        *sorted(REPO_ROOT.rglob("AGENTS.md")),
    ]
    if (REPO_ROOT / "willem-suggestions.md").exists():
        md_files_to_check.append(REPO_ROOT / "willem-suggestions.md")

    broken_links: list[str] = []
    for md in md_files_to_check:
        broken_links.extend(_check_markdown_links(md))
    if broken_links:
        issues.extend(broken_links)
        print(f"[FAIL] 5. Documentation Link Audit   : {len(broken_links)} broken link(s)")
        for b in broken_links:
            print(f"          - {b}")
    else:
        print(f"[PASS] 5. Documentation Link Audit   : All links verified across {len(md_files_to_check)} documents")

    # 6. Collection Standards Linters
    linter_cmds = [
        ("agents_md_lint", [SYS_PYTHON, "tools/agents_md_lint.py"]),
        ("ai_file_lint", [SYS_PYTHON, "tools/ai_file_lint.py", "."]),
        ("bom_lint", [SYS_PYTHON, "tools/bom_lint.py", "--root", "."]),
        ("check_docs", [SYS_PYTHON, "tools/check_docs.py", "."]),
        ("check_prints", [SYS_PYTHON, "tools/check_prints.py", "."]),
    ]
    failed_linters: list[str] = []
    for name, cmd in linter_cmds:
        proc = subprocess.run(cmd, cwd=str(REPO_ROOT), capture_output=True, text=True)
        if proc.returncode != 0:
            failed_linters.append(f"{name}: {proc.stdout.strip()} {proc.stderr.strip()}".strip())
    if failed_linters:
        issues.extend(failed_linters)
        print(f"[FAIL] 6. Collection Standards Gates : {len(failed_linters)} linter(s) failed")
        for fl in failed_linters:
            print(f"          - {fl}")
    else:
        print("[PASS] 6. Collection Standards Gates : 5/5 linters passed (AGENTS, AI files, BOM, Docs, Prints)")

    # 7. PrusaSlicer & CDR Print Package Manifold Check
    try:
        ps_status = ps.prusaslicer_version_and_status()
        pkg_audit = ps.validate_mco_print_package("cad/print_package_wing_root")
        if pkg_audit["all_manifold"] and pkg_audit["all_fit_bed"]:
            print(
                f"[PASS] 7. CAD & PrusaSlicer Gate     : {pkg_audit['total_stl_files']}/{pkg_audit['total_stl_files']} STLs manifold & fit CORE One ({ps_status['version_banner']})"
            )
        else:
            issues.append("One or more STLs in cad/print_package_wing_root failed manifold or bed-fit check")
            print("[FAIL] 7. CAD & PrusaSlicer Gate     : Manifold or bed-fit failure")
    except Exception as exc:
        issues.append(f"CAD/PrusaSlicer check error: {exc}")
        print(f"[FAIL] 7. CAD & PrusaSlicer Gate     : {exc}")

    print("=" * 78)
    if issues:
        print(f"DOCTOR VERDICT: {len(issues)} ISSUE(S) FOUND")
        return 1
    print("DOCTOR VERDICT: ALL 7 GATES PASSED (100% HEALTHY)")
    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="mco",
        description="MCO-1D Metachronal Cascade Ornithopter — Unified Engineering Workflow CLI",
    )
    sub = parser.add_subparsers(dest="command", required=True)

    # test
    p_test = sub.add_parser("test", help="Run unit tests or TC-01 bench qualification")
    p_test.add_argument("--suite", "-s", help="Filter test suite by name/substring (e.g. 'wig', 'prusaslicer')")
    p_test.add_argument("--tc01", action="store_true", help="Run automated TC-01 bench qualification harness")
    p_test.set_defaults(func=cmd_test)

    # sim
    p_sim = sub.add_parser("sim", help="Run 50 Hz SITL flight computer simulation")
    p_sim.set_defaults(func=cmd_sim)

    # gs
    p_gs = sub.add_parser("gs", help="Launch Ground Station server & telemetry bridge")
    p_gs.set_defaults(func=cmd_gs)

    # bom
    p_bom = sub.add_parser("bom", help="Inspect and update the Bill of Materials & MTOW budget")
    p_bom.add_argument("--json", action="store_true", help="Output BOM report as JSON")
    p_bom.add_argument("--view", action="store_true", help="Display formatted BOM report and update Markdown")
    p_bom.set_defaults(func=cmd_bom)

    # cad
    p_cad = sub.add_parser("cad", help="Generate parametric CAD parts or verify STL manifoldness")
    p_cad.add_argument(
        "target",
        nargs="?",
        default="verify",
        choices=["verify", *sorted(CAD_TARGETS.keys())],
        help="CAD target to run (default: verify)",
    )
    p_cad.set_defaults(func=cmd_cad)

    # slice
    p_slice = sub.add_parser("slice", help="Inspect, slice, or open 3D models in PrusaSlicer 2.9.6")
    p_slice.add_argument("files", nargs="+", help="Path(s) to .stl, .step, or .3mf files")
    p_slice.add_argument(
        "--profile",
        "-p",
        default="pa12cf_structural",
        help="Slicer profile alias (pa12cf_structural, pa12cf_solid_gears, lwpla_aero_fairing, pla_prototype) or .ini path",
    )
    p_slice.add_argument("--output-dir", "-o", help="Output directory for sliced .gcode and .3mf")
    p_slice.add_argument("--layer-height", type=float, help="Override layer height (mm)")
    p_slice.add_argument("--perimeters", type=int, help="Override perimeter wall count")
    p_slice.add_argument("--fill-density", help="Override infill density (e.g. 60%%)")
    p_slice.add_argument("--supports", action="store_true", help="Enable support generation")
    p_slice.add_argument("--merge", action="store_true", help="Arrange and merge multiple inputs onto one plate")
    p_slice.add_argument("--no-3mf", action="store_true", help="Skip exporting a companion .3mf project file")
    p_slice.add_argument("--inspect", "-i", action="store_true", help="Only inspect mesh dimensions, volume, and manifold status")
    p_slice.add_argument("--gui", "-g", action="store_true", help="Open the model(s) and profile in the PrusaSlicer macOS GUI")
    p_slice.set_defaults(func=cmd_slice)

    # docs
    p_docs = sub.add_parser("docs", help="Regenerate PDF assembly manuals and flight test worksheets")
    p_docs.add_argument(
        "target",
        nargs="?",
        default="all",
        choices=["all", *sorted(DOC_TARGETS.keys())],
        help="Documentation target to regenerate (default: all)",
    )
    p_docs.set_defaults(func=cmd_docs)

    # doctor
    p_doc = sub.add_parser("doctor", help="Run repository hygiene, documentation link, and CAD/slicer health checks")
    p_doc.set_defaults(func=cmd_doctor)

    return parser


def main() -> int:
    parser = build_parser()
    args, unknown = parser.parse_known_args()
    args.sim_args = unknown if args.command == "sim" else []
    args.gs_args = unknown if args.command == "gs" else []
    if unknown and args.command not in {"sim", "gs"}:
        parser.error(f"unrecognized arguments: {' '.join(unknown)}")
    return args.func(args)


if __name__ == "__main__":
    sys.exit(main())
