# Test Suite (`tests/`)

Deterministic unit, configuration, and closed-loop simulation tests for the
MCO-1D. Read [`../AGENTS.md`](../AGENTS.md) first.

## Active test suites

- [`test_single_source_of_truth.py`](test_single_source_of_truth.py): verifies that [`../config/airframe_spec.json`](../config/airframe_spec.json),
  [`../firmware/mco_config.py`](../firmware/mco_config.py), [`../bom/bom.csv`](../bom/bom.csv),
  and firmware modules agree on mass, geometry, gear ratio, CAN nodes, and
  disjoint GPIO pins.
- [`test_anti_slop_gates.py`](test_anti_slop_gates.py): enforces zero duplicate AST function definitions in
  active code, zero hardcoded machine home paths, and valid reciprocating
  capstan kinematics.
- [`test_mco_kinematics.py`](test_mco_kinematics.py): verifies 4-station
  metachronal phase progression, 90°-phase-advanced cosine pitch feathering,
  control allocation mixer clamps, and `MCOHardware.apply_commands()` wiring.
- [`test_mco_wave_library.py`](test_mco_wave_library.py),
  [`test_safety_and_performance_upgrades.py`](test_safety_and_performance_upgrades.py):
  core 8-wing phase progression, governor clamps, and fault-handling tests.
- [`test_aero_safety_mitigations.py`](test_aero_safety_mitigations.py),
  [`test_autopilot_bank_governor.py`](test_autopilot_bank_governor.py),
  [`test_sprint_thermal_duty_clamp.py`](test_sprint_thermal_duty_clamp.py),
  [`test_symmetrical_rth_controller.py`](test_symmetrical_rth_controller.py),
  [`test_tie_breaking.py`](test_tie_breaking.py),
  [`test_wig_floor_governor.py`](test_wig_floor_governor.py),
  [`test_wrist_gear_damage_mitigation.py`](test_wrist_gear_damage_mitigation.py),
  [`test_prusaslicer_mcp.py`](test_prusaslicer_mcp.py): subsystem and MCP tests.

## Rules for `tests/`

1. **Test behavior, not dictionary literals**: every test must exercise real
   control-loop execution, kinematics math, or structural/configuration
   invariants. Never write tautological tests that only check whether a key
   exists in a static dictionary (47 such suites are quarantined in
   `../archive/tests_tautological/`).
2. **Never hardcode a pass return**: do not add special cases in `firmware/` to
   make a test pass.

<!-- standards:begin -->
Collection standards (presentations, sources, git hygiene) live in
`/Users/willem/Code/standards/STANDARDS.md`; decks are built from
`standards/powerpoint template/` and gated with its `deck_checks.py`. Managed
block: edit `standards/ai-files/BLOCK-nested.md`, not this copy.
AI use in school work: no AI images, adviser pre-clearance for AI-written
text, never edit graded text; see
`standards/standards/ai-use-disclosure/ai-use-disclosure.md`.
AI files: one `AGENTS.md` per repo; `CLAUDE.md` is `@AGENTS.md`.
<!-- standards:end -->
