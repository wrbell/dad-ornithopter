# Contributing to MCO-1D Metachronal Cascade Ornithopter

Thank you for contributing to the **MCO-1D** bio-inspired flight software, parametric CAD, and additive manufacturing repository.

To preserve flight safety, aerodynamic fidelity, and CAD/firmware synchronization across all 8 flapping wings, all contributors and automated agents must adhere to the workflows and quality gates outlined below.

---

## 1. Development Environment & Python Runtimes

This repository utilizes two dedicated Python runtime layers:

1. **CAD, 3D Printing, ReportLab PDFs & MCP Servers**:
   - **Environment**: `ornithopter-cad/.venv-agentcad/bin/python` (Python 3.12 with `build123d`, `cadquery`, `agentcad`, `mcp`, `vtk`, `lib3mf`, `scipy`, `matplotlib`).
   - Automatically selected when invoking CLI commands via `./mco cad ...`, `./mco slice ...`, and `./mco docs ...`.

2. **Flight Firmware & Unit Tests (`firmware/`, `tests/`)**:
   - Compatible with both standard system Python 3.12 and `.venv-agentcad/bin/python`.
   - Zero external binary or C-extension requirements for standard simulation execution (`mco_main.py --sim`).

---

## 2. Mandatory Workflows via `./mco` CLI

Always use the `./mco` unified command-line tool (or equivalent `Makefile` targets) for operations:

```bash
# 1. Repository Hygiene, Markdown Link & CAD/Slicer Health Check
./mco doctor                  # or: make doctor

# 2. Automated Test Suite (1,020 Tests) & Bench Qualification
./mco test                    # Full test suite (or: make test)
./mco test --suite prusaslicer # Targeted test suite
./mco test --tc01             # TC-01 rigid bench qualification

# 3. 50 Hz Hardware-in-the-Loop SITL Flight Simulation
./mco sim                     # 50 Hz flight executive simulation (or: ./run_sim.sh)

# 4. Bill of Materials (BOM) & Mass Budget Audit
./mco bom --view              # Refresh & inspect terminal BOM summary
./mco bom --json              # Output machine-readable JSON mass breakdown

# 5. Parametric CAD Generation & Mesh Manifold Verification
./mco cad verify              # Audit all print package STLs (manifold & bed fit)
./mco cad package             # Regenerate Wing Root CDR 3D print package

# 6. Model Slicing & G-Code Inspection via PrusaSlicer MCP
./mco slice <model.stl> --inspect
./mco slice <model.stl> --profile pa12cf_structural
./mco slice <model.stl> --profile pa12cf_solid_gears --gui

# 7. Documentation & PDF Manual Generation
./mco docs all                # Regenerate all 5 publication-grade assembly guides
```

---

## 3. Strict Repository Organization & No-Clutter Policy

To maintain repository cleanliness:
- **NEVER** place temporary scripts, scratch `.py` files, `.stl`/`.step`/`.3mf` models, `.png` renders, or `.pdf` reports in the repository root directory.
- All temporary experiments belong in `<appDataDir>/brain/<conversation-id>/scratch/` or `scripts/archive/`.
- Slicing profiles must reside in [`cad/slicer_profiles/`](cad/slicer_profiles/).
- Manufactured deliverables belong in [`cad/manufactured/`](cad/manufactured/).
- 3D printable production packages belong in [`cad/print_package_wing_root/`](cad/print_package_wing_root/).

---

## 4. CAD & 3D Printing Verification Gate

Before submitting any modification to a 3D printable part:
1. **Manifold & Bed Fit**:
   - Run `./mco cad verify` to verify all STLs are 100% manifold, possess positive volume, and fit within the **Prusa CORE One** build volume ($250 \times 220 \times 250\text{ mm}$).
2. **Slicing & Mass Budget**:
   - Run `./mco slice <part.stl> --profile <profile>` to verify toolpath feasibility, layer count, and filament consumption against the budget in [`cad/print_package_wing_root/README_3D_PRINT_GUIDE.md`](cad/print_package_wing_root/README_3D_PRINT_GUIDE.md).
3. **Pairwise Interference**:
   - Ensure CAD solid interference across assembled components evaluates to `0.0000 mm³`.

---

## 5. Pre-Commit & Pre-Push Checklist

Before pushing changes to GitHub (`origin/main`):

- [ ] **Run Doctor**: `./mco doctor` exits with `DOCTOR VERDICT: ALL CHECKS PASSED (100% HEALTHY)`.
- [ ] **Run Verification Suite**: `./mco test` passes all 1,020 tests.
- [ ] **Run SITL Simulation**: `./mco sim` executes cleanly without NaN or actuator fault warnings.
- [ ] **Refresh BOM**: If part counts or masses changed, update [`scripts/mco_bom_tool.py`](scripts/mco_bom_tool.py) and run `./mco bom --view`.
- [ ] **Update Documentation Catalogs**: Ensure any newly created or moved files are referenced in [`INDEX.md`](INDEX.md), [`README.md`](README.md), and [`docs/README.md`](docs/README.md).
