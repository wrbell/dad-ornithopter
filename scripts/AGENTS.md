# Scripts & Tooling (`scripts/`)

CLI entrypoints, BOM generators, parametric CadQuery builders, and document
generators. Read [`../AGENTS.md`](../AGENTS.md) first.

## Subdirectories

- [`mco_cli.py`](mco_cli.py): backing implementation for `./mco` (`doctor`,
  `test`, `sim`, `cad`, `bom`, `preflight`, `wizard`, `gs`).
- [`mco_bom_tool.py`](mco_bom_tool.py): single-source-of-truth BOM generator.
  Exports [`../bom/bom.csv`](../bom/bom.csv), updates
  [`../config/airframe_spec.json`](../config/airframe_spec.json), and writes
  [`../docs/mark1/MCO-1D_Mark1_Bill_of_Materials.md`](../docs/mark1/MCO-1D_Mark1_Bill_of_Materials.md).
- [`cad/`](cad/): parametric CadQuery and DXF generators.
- [`docs/`](docs/): ReportLab PDF assembly guide and worksheet generators.
- [`renders/`](renders/): PyVista off-screen rendering scripts.
- [`audits/`](audits/): closed-loop flight envelope simulation scripts.
- [`mcp/`](mcp/): local MCP server (`prusaslicer_mcp_server.py`).

## Rules for `scripts/`

1. **Path resolution**: compute `REPO_ROOT` relative to `__file__` (two levels
   up for `scripts/*.py`, three levels up for `scripts/<subdir>/*.py`). Never
   hardcode absolute machine home directories or external scratch paths.
2. **Dynamic BOM math**: [`mco_bom_tool.py`](mco_bom_tool.py) must compute all
   category and airframe mass/cost totals dynamically from the item rows. Never
   hardcode summary numbers in Markdown strings.
3. **No fake FEA/CFD scripts**: every script in [`audits/`](audits/) or
   [`cad/`](cad/) must execute real numerical integration or B-Rep solid checks
   and exit non-zero on failure. Fake analytical scripts that print `CERTIFIED`
   without a real solver are quarantined in `archive/fake_audits/`.

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
