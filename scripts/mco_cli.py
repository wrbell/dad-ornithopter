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
    ".gitignore",
    ".mcp.json",
    "AGENTS.md",
    "CONTRIBUTING.md",
    "INDEX.md",
    "Makefile",
    "PROJECT_STATE.md",
    "README.md",
    "mco",
    "pytest.ini",
    "run_sim.sh",
    "start_gs.sh",
    "view_bom.sh",
}

ALLOWED_ROOT_DIRS = {
    ".agents",
    ".git",
    ".github",
    ".pytest_cache",
    "__pycache__",
    "cad",
    "docs",
    "firmware",
    "knowledge",
    "logs",
    "ornithopter-cad",
    "scripts",
    "tests",
    "ui",
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


def cmd_doctor(_args: argparse.Namespace) -> int:
    from scripts.mcp import prusaslicer_mcp_server as ps

    print("=" * 78)
    print("  MCO-1D REPOSITORY DOCTOR — HYGIENE, LINK & CAD/SLICER AUDIT")
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
        print(f"[FAIL] Root Directory Cleanliness: {len(stray_root)} stray item(s)")
        for s in stray_root:
            print(f"       - {s}")
    else:
        print("[PASS] Root Directory Cleanliness: 0 stray files or directories")

    # 2. Markdown link verification
    md_files_to_check = [
        REPO_ROOT / "README.md",
        REPO_ROOT / "INDEX.md",
        REPO_ROOT / "PROJECT_STATE.md",
        REPO_ROOT / "docs" / "README.md",
    ]
    if (REPO_ROOT / "AGENTS.md").exists():
        md_files_to_check.append(REPO_ROOT / "AGENTS.md")

    broken_links: list[str] = []
    for md in md_files_to_check:
        broken_links.extend(_check_markdown_links(md))
    if broken_links:
        issues.extend(broken_links)
        print(f"[FAIL] Documentation Link Audit  : {len(broken_links)} broken link(s)")
        for b in broken_links:
            print(f"       - {b}")
    else:
        print(f"[PASS] Documentation Link Audit  : All links verified across {len(md_files_to_check)} index documents")

    # 3. PrusaSlicer & MCP readiness
    try:
        ps_status = ps.prusaslicer_version_and_status()
        print(f"[PASS] PrusaSlicer MCP Backend   : {ps_status['version_banner']} ({len(ps_status['mco_repo_profiles'])} repo profiles)")
    except Exception as exc:
        issues.append(f"PrusaSlicer MCP error: {exc}")
        print(f"[FAIL] PrusaSlicer MCP Backend   : {exc}")

    # 4. CAD Print Package Manifold & Bed-Fit Check
    try:
        pkg_audit = ps.validate_mco_print_package("cad/print_package_wing_root")
        if pkg_audit["all_manifold"] and pkg_audit["all_fit_bed"]:
            print(f"[PASS] CDR 3D Print Package      : {pkg_audit['total_stl_files']}/{pkg_audit['total_stl_files']} STLs manifold & fit CORE One bed")
        else:
            issues.append("One or more STLs in cad/print_package_wing_root failed manifold or bed-fit check")
            print("[FAIL] CDR 3D Print Package      : Manifold or bed-fit failure")
    except Exception as exc:
        issues.append(f"Print package check error: {exc}")
        print(f"[FAIL] CDR 3D Print Package      : {exc}")

    print("=" * 78)
    if issues:
        print(f"DOCTOR VERDICT: {len(issues)} ISSUE(S) FOUND")
        return 1
    print("DOCTOR VERDICT: ALL CHECKS PASSED (100% HEALTHY)")
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
    p_sim.add_argument("sim_args", nargs=argparse.REMAINDER, help="Arguments passed to firmware/mco_main.py --sim")
    p_sim.set_defaults(func=cmd_sim)

    # gs
    p_gs = sub.add_parser("gs", help="Launch Ground Station server & telemetry bridge")
    p_gs.add_argument("gs_args", nargs=argparse.REMAINDER, help="Arguments passed to ground station launcher")
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
    args = parser.parse_args()
    return args.func(args)


if __name__ == "__main__":
    sys.exit(main())
