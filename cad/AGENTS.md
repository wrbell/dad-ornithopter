# CAD & 3D Printing (`cad/`)

Parametric geometry, airfoil coordinates, slicer profiles, and manufactured
models for the MCO-1D airframe. Read [`../AGENTS.md`](../AGENTS.md) first.

## Directory layout

- [`coordinates/`](coordinates/): canonical Selig airfoil coordinate files
  ([`MCO_S1223_120mm.txt`](coordinates/MCO_S1223_120mm.txt),
  [`MCO_S1223_MOD_165mm.txt`](coordinates/MCO_S1223_MOD_165mm.txt),
  [`SD7062_165mm.txt`](coordinates/SD7062_165mm.txt)).
- [`slicer_profiles/`](slicer_profiles/): PrusaSlicer `.ini` profiles for PA12-CF
  structural parts and foaming LW-PLA aerodynamic skins.
- [`manufactured/`](manufactured/): generated `.step`, `.stl`, `.3mf`, and `.dxf`
  parts plus rendered views (`renders/`, `renders_v9/`, `renders_v10/`,
  `renders_exploded/`, `renders_package/`).
- [`print_package_wing_root/`](print_package_wing_root/): consolidated build-plate
  meshes and [`README_3D_PRINT_GUIDE.md`](print_package_wing_root/README_3D_PRINT_GUIDE.md).
- [`drawings/`](drawings/) and [`wrist/`](wrist/): 2D schematics and legacy wrist
  reference geometry.

## Rules for CAD & print files

1. **Parametric source of truth**: every `.step` and `.stl` in [`manufactured/`](manufactured/)
   must be generated deterministically by a script in [`../scripts/cad/`](../scripts/cad/)
   using dimensions from [`../config/airframe_spec.json`](../config/airframe_spec.json).
2. **Real B-Rep STEP files only**: export `.step` directly from CadQuery B-Rep
   solids (`cq.exporters.export(solid, path)`). Never convert a tessellated
   `.stl` mesh into a faceted `.step` file.
3. **Boolean operation order**: always cut internal clearance pockets and servo
   bays *before* fusing precision bearing bosses (`6800-2RS` `Ø19.0 mm` seats)
   so bearing counterbores are never obliterated by subsequent cuts.
4. **No raw G-code or archives in git**: do not commit `.bgcode`, `.gcode`, or
   `.zip` bundles. Run `python3 tools/check_prints.py .` from the repo
   root to verify print documentation hygiene.

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
