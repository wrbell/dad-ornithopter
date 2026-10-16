# Documentation (`docs/`)

Engineering specifications, manufacturing manuals, commissioning guides, and
field test-card worksheets for the MCO-1D. Read [`../AGENTS.md`](../AGENTS.md)
first.

## Structure

- [`README.md`](README.md): documentation index.
- [`mark1/`](mark1/): active Mark 1 hardware manuals, generated BOM
  ([`MCO-1D_Mark1_Bill_of_Materials.md`](mark1/MCO-1D_Mark1_Bill_of_Materials.md)),
  and printable field worksheets (`mark1/worksheets/`).
- [`archive/`](archive/): quarantined legacy drafts, unverified CFD/FEA reports,
  and future Mark 2 conceptual notes.

## Rules for `docs/`

1. **Match `config/airframe_spec.json`**: every physical parameter cited in
   [`mark1/`](mark1/) must match [`../config/airframe_spec.json`](../config/airframe_spec.json)
   and [`../bom/bom.csv`](../bom/bom.csv):
   - Installed BOM mass: `3.642 kg` (`3,642 g`)
   - Effective wingspan: `2.20 m` (`1.00 m` per wing panel + `0.20 m` root spacing)
   - Root chord: `0.28 m` (`280 mm`); total wing area: `2.24 m²`
   - Capstan reduction ratio: `10.0:1` (`Ø154.0 mm` sector wheel / `Ø15.4 mm` pinion)
   - Main battery: `6S 5000 mAh 50C LiPo` (`22.2 V` nominal) + `2x 20A HV SBEC (7.4 V)`
2. **Never invent measurements or certifications**: mark unmeasured hardware
   values as `TBD (measure on hardware)`. Do not claim DO-178C, MIL-STD, or CFD
   certification unless backed by physical test logs or external solver data.
3. **Relative links only**: use repository-relative links in Markdown files.
   Never commit absolute local `file://` URLs pointing to a user home directory.

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
