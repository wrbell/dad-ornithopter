# MCO-1D Engineering Repository — Agent Rules & Workflow Contract (`AGENTS.md`)

You are working in the **MCO-1D Metachronal Cascade Ornithopter** flight software, parametric CAD, and additive manufacturing repository. Always follow these rules to preserve repository organization, physical/software synchronization, and engineering verification.

---

## 1. Repository Layout & Strict No-Root-Clutter Policy

**NEVER place temporary scripts, scratch `.py` files, `.stl`/`.step`/`.3mf` models, `.png` renders, or `.pdf` reports in the repository root.**

| Directory | Purpose & Placement Rules |
| :--- | :--- |
| [`firmware/`](firmware/) | Active 50 Hz flight executive (`mco_main.py`), metachronal wave kinematics (`mco_wave_library.py`), safety governors (`mco_flight_governor.py`), intent control (`mco_intent_control.py`), and hardware drivers. Must remain runnable in `--sim` mode with zero hardware dependencies. |
| [`tests/`](tests/) | Automated verification suites (`test_*.py`). Every new firmware feature, safety governor, or MCP tool must have unit test coverage here. |
| [`cad/print_package_wing_root/`](cad/print_package_wing_root/) | Authoritative CDR 3D print package for one complete Wing Root Assembly (`01`–`09` + `PLATE_01` + [`README_3D_PRINT_GUIDE.md`](cad/print_package_wing_root/README_3D_PRINT_GUIDE.md)). |
| [`cad/slicer_profiles/`](cad/slicer_profiles/) | Version-controlled PrusaSlicer 2.9.6 `.ini` profiles for **Prusa CORE One** (`pa12cf_structural`, `pa12cf_solid_gears`, `lwpla_aero_fairing`, `pla_prototype`). |
| [`cad/manufactured/`](cad/manufactured/) | Active generated `.step`, `.stl`, and `.3mf` deliverables. Historical `v1`–`v8` iterations belong in `cad/manufactured/archive/`. |
| [`scripts/`](scripts/) | Top-level operational entrypoints (`mco_cli.py`, `mco_bom_tool.py`, `run_tc01_bench_checkout.py`, `export_test_card_telemetry.py`) plus organized subdirectories:<ul><li>`scripts/mcp/` — MCP servers ([`prusaslicer_mcp_server.py`](scripts/mcp/prusaslicer_mcp_server.py))</li><li>`scripts/cad/` — Active `build123d` / CadQuery parametric CAD generators</li><li>`scripts/renders/` — Multi-view PNG & VTK/matplotlib rendering scripts</li><li>`scripts/docs/` — ReportLab PDF assembly & calibration guide generators</li><li>`scripts/audits/` — CFD, FEA, and full-envelope mission simulation audits</li><li>`scripts/archive/` — Legacy `v1`–`v8` scripts and scratch experiments</li></ul> |
| [`docs/mark1/`](docs/mark1/) | Authoritative **MCO-1D Mark 1** specifications, BOM, manufacturing/commissioning/operator manuals, assembly PDFs, and CFD/FEA audit reports. |

---

## 2. Python Runtime & Environment Selection

This macOS workspace has two Python environments with distinct roles:
1. **CAD, 3D Printing, ReportLab PDFs & MCP Servers**:
   - **MUST** use `ornithopter-cad/.venv-agentcad/bin/python` (Python 3.12 with `build123d`, `cadquery`, `agentcad`, `mcp`, `vtk`, `lib3mf`, `scipy`, `matplotlib`).
   - Or invoke via `./mco cad ...`, `./mco slice ...`, `./mco docs ...` which automatically select `.venv-agentcad/bin/python`.
2. **Flight Firmware & Unit Tests (`firmware/`, `tests/`)**:
   - Compatible with both system `python3` and `ornithopter-cad/.venv-agentcad/bin/python`.

---

## 3. Mandatory Workflows via `./mco` and `Makefile`

Always prefer `./mco <subcommand>` (or `make <target>`) over ad-hoc one-liner commands:

```bash
# 1. Repository Hygiene, Markdown Link & CAD/Slicer Health Check
./mco doctor                  # or: make doctor

# 2. Run Unit Verification Suites & TC-01 Bench Qualification
./mco test                    # Full test suite (or: make test)
./mco test --suite <name>     # Targeted suite (e.g., ./mco test --suite prusaslicer)
./mco test --tc01             # TC-01 bench qualification (or: make tc01)

# 3. Run 50 Hz SITL Flight Simulation
./mco sim                     # or: ./run_sim.sh / make sim

# 4. Update & Verify Bill of Materials (BOM) and MTOW Budget
./mco bom --view              # or: ./view_bom.sh / make bom
./mco bom --json

# 5. Parametric CAD Generation & Mesh Manifold Verification
./mco cad verify              # Verify all STLs in cad/print_package_wing_root/ (or: make cad-verify)
./mco cad package             # Regenerate & package Wing Root CDR 3D print files (or: make cad)

# 6. Slice & Inspect 3D Models with PrusaSlicer 2.9.6
./mco slice <file.stl> --inspect
./mco slice <file.stl> --profile pa12cf_structural
./mco slice <file.stl> --profile pa12cf_solid_gears --gui

# 7. Regenerate Publication-Grade PDF Manuals
./mco docs all                # or: make docs
```

---

## 4. CAD & 3D Printing Workflow Rules (PrusaSlicer MCP)

Whenever you create or modify any 3D printable part (`.stl`, `.step`, `.3mf`):
1. **Manifold & Bed-Fit Gate**:
   - Call the **`prusaslicer` MCP** tool `inspect_model` (or `validate_mco_print_package` / `./mco cad verify`) to verify:
     - `is_manifold: true` (`manifold = yes`)
     - Positive volume (`volume_cm3 > 0`)
     - Fits on the **Prusa CORE One** build plate ($250 \times 220 \times 250\text{ mm}$).
2. **Slicing & Mass Budget Verification**:
   - Use the `prusaslicer` MCP tool `slice_model` (or `./mco slice`) with the appropriate versioned profile in [`cad/slicer_profiles/`](cad/slicer_profiles/):
     - `pa12cf_structural` — 6 walls, 60% Gyroid infill, 0.16mm layer height (Collar `01`, Powertrain Stand `02`, Stagecoach Drum `04`, `PLATE_01`).
     - `pa12cf_solid_gears` — 6 walls, 100% solid rectilinear infill, 0.15mm layer height (Top Brace `03`, Capstan Pinion `05`, Gimbal Yoke `06`, Bevel Gears `07`/`08`).
     - `lwpla_aero_fairing` — 2 walls, 15% Gyroid infill, 0.55 extrusion multiplier foaming profile (NACA 0018 Fairing `09`, S1223/SD7062 wing ribs & wingtips).
     - `pla_prototype` — Prusament PLA rapid fit-check profile.
   - Compare the G-code `filament_used_g` against the mass budget in [`cad/print_package_wing_root/README_3D_PRINT_GUIDE.md`](cad/print_package_wing_root/README_3D_PRINT_GUIDE.md) and [`scripts/mco_bom_tool.py`](scripts/mco_bom_tool.py).
3. **Visual Inspection**:
   - Use `open_in_prusaslicer_gui` (or `./mco slice <file> --gui`) when the user asks to preview or plate parts in the PrusaSlicer macOS app.

---

## 5. Strict No-GitHub-Actions Policy

**DO NOT use GitHub Actions or create `.github/workflows/` files in this repository under any circumstances.**
- All continuous integration, unit testing (1,020+ tests), 50 Hz SITL flight simulation, CAD manifold verification, and doctor health audits **MUST** run locally on macOS via `./mco test`, `./mco doctor`, `./mco sim`, and `Makefile`.
- Never create, commit, or push `.github/workflows` or any remote CI pipelines. Verification is 100% local.

---

## 6. Documentation & Pre-Completion Checklist

Before concluding any task that adds files, changes hardware/mass parameters, or modifies flight laws:
1. If component masses or part counts changed, update [`scripts/mco_bom_tool.py`](scripts/mco_bom_tool.py) and run `./mco bom --view` to refresh [`docs/mark1/MCO-1D_Mark1_Bill_of_Materials.md`](docs/mark1/MCO-1D_Mark1_Bill_of_Materials.md).
2. If files were added, moved, or archived, update [`INDEX.md`](INDEX.md), [`README.md`](README.md), and [`docs/README.md`](docs/README.md).
3. Run `./mco doctor` and verify it exits with `DOCTOR VERDICT: ALL CHECKS PASSED (100% HEALTHY)`.
