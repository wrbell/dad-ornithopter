# Contributing to MCO-1D Metachronal Cascade Ornithopter

All contributors and AI coding agents must follow the single-source-of-truth architecture and anti-slop quality gates defined in [`AGENTS.md`](AGENTS.md).

---

## 1. Single Source of Truth

Never hardcode physical constants, mass budgets, CAN node IDs, drivetrain ratios, or GPIO pin numbers in individual files:

1. **Airframe & Hardware Specification**: Edit [`config/airframe_spec.json`](config/airframe_spec.json) and load via [`firmware/mco_config.py`](firmware/mco_config.py) (`MCOConfig`).
2. **Bill of Materials & Mass Budget**: Edit [`scripts/mco_bom_tool.py`](scripts/mco_bom_tool.py) and run `./mco bom --view` to regenerate [`bom/bom.csv`](bom/bom.csv), sync `installed_total_mass_g` in [`config/airframe_spec.json`](config/airframe_spec.json), and update [`docs/mark1/MCO-1D_Mark1_Bill_of_Materials.md`](docs/mark1/MCO-1D_Mark1_Bill_of_Materials.md).
3. **Unmeasured Hardware Values**: Mark any quantity requiring physical bench or flight measurement as `"TBD (measure on hardware)"` until measured on physical instruments.

---

## 2. Mandatory Verification Commands

Run these commands from the repository root before committing:

```bash
# 1. Anti-slop repository health & single-source-of-truth gate
./mco doctor

# 2. Automated unit & configuration test suite
./mco test

# 3. 50 Hz Software-in-the-Loop (SITL) flight simulation
./mco sim --duration 2.0

# 4. Bill of Materials (BOM) regeneration & audit
./mco bom --view

# 5. Collection standards linters
python3 tools/agents_md_lint.py
python3 tools/ai_file_lint.py .
python3 tools/bom_lint.py --root .
python3 tools/check_docs.py .
python3 tools/check_prints.py .
```

---

## 3. Repository Hygiene & Anti-Slop Policy

- **No duplicate function definitions**: Every Python file must define each top-level function or class name at most once. `./mco doctor` parses ASTs across the repository and blocks duplicate definitions.
- **No fake FEA/CFD or synthetic test reports**: Do not write scripts that print `CERTIFIED` from hardcoded arithmetic, and never commit `--mock` bench checkout output as a physical hardware test report.
- **No machine-specific paths**: Never commit `/Users/...` or local scratch paths in scripts, configs, or docs.
- **No binary bloat in git**: Do not track `.bgcode`, `.gcode`, `.zip`, or STL-to-STEP mesh conversions.
