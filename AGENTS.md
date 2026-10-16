# AGENTS.md

## Project

This repository holds the engineering design, parametric CAD, bill of materials,
and Raspberry Pi 5 flight-control software for the **MCO-1D Metachronal Cascade
Ornithopter** (8-wing tandem-staggered flapping-wing research airframe).

## Single source of truth

Never hardcode physical constants, mass budgets, pin assignments, CAN node IDs,
or drivetrain ratios in scripts, firmware modules, or documentation prose.

- **Airframe & hardware constants**: [`config/airframe_spec.json`](config/airframe_spec.json),
  loaded in Python via [`firmware/mco_config.py`](firmware/mco_config.py) (`MCOConfig`).
- **Bill of materials & mass budget**: [`bom/bom.csv`](bom/bom.csv), generated
  and verified by [`scripts/mco_bom_tool.py`](scripts/mco_bom_tool.py).
- **Unmeasured physical values**: any quantity that requires physical bench or
  flight measurement (as-built weighed mass, static thrust, actual center of
  gravity, flight endurance) must be marked `"TBD (measure on hardware)"` until
  recorded from an instrument or scale.

Domain-specific rules live in:

- `firmware/AGENTS.md` — flight control loop, CAN bus, and GPIO rules
- `cad/AGENTS.md` — parametric CadQuery scripts and print artifacts
- `scripts/AGENTS.md` — CLI tools, BOM generator, and verification scripts
- `docs/AGENTS.md` — technical manuals and zero-invention documentation rules
- `tests/AGENTS.md` — unit and hardware-in-the-loop simulation test rules

## Commands

Run these checks from the repository root before finishing any task:

```sh
./mco doctor
./mco test
./mco bom --view
python3 tools/agents_md_lint.py
python3 tools/ai_file_lint.py .
python3 tools/bom_lint.py --root .
python3 tools/check_docs.py .
python3 tools/check_prints.py .
```

## Anti-slop engineering rules

1. **No duplicated constants**: read `MCOConfig` from `firmware/mco_config.py`
   or `config/airframe_spec.json`. Do not redefine mass (`3.642 kg`), wingspan
   (`2.20 m`), capstan ratio (`10.0:1`), or GPIO pins in individual files.
2. **No append-only function redefinitions**: every Python file must have
   exactly one definition per function or class name at module scope.
   `./mco doctor` parses every Python AST and fails on duplicate `def` names.
3. **No fake physics or fabricated reports**: never write scripts that multiply
   arbitrary numbers and print `CERTIFIED` or `0.0000 mm³` without running a
   real solver or hardware measurement. Never commit synthetic bench reports as
   completed hardware test results.
4. **No machine-specific paths**: never commit absolute user paths or local
   scratch directories in code, configs, or docs. Use `REPO_ROOT`-relative paths.
5. **No tracked binary bloat**: keep `.bgcode`, `.gcode`, `.zip`, and duplicate
   STL/STEP aliases out of git.

## Clarity

Write explanations to Willem in Simplified Technical English. Use the same style
for a pull request description, a commit message body, and prose in a README or
another doc. Aim for about 80 percent of the rules. Full compliance with
ASD-STE100 is not the goal.

The rules are in the standards repository at `standards/writing-ste/ste.md`.
The upstream skill is
[simplified-technical-english](https://github.com/0xpili/simplified-technical-english/tree/1e148d670cba46685ad2b4c3f2354a637a7fdbbe)
(MIT, commit `1e148d670cba46685ad2b4c3f2354a637a7fdbbe`). Link to that skill.
Do not copy the skill into this repository again.

Code, identifiers, math, command-line output, and quoted error text are exempt.

When structure, flow, or architecture is the point, use a mermaid diagram.
For a complex result, offer a self-contained HTML page.
That page is a throwaway file.
Do not commit it unless Willem asks.

Make a video only when Willem asks for a video.
Do not add an API key or a secret.

`scripts/ste_check.py` in the upstream skill is an optional check on docs.
Do not use it as a CI gate.

<!-- standards:begin -->
## Collection standards

Every project under `/Users/willem/Code` follows the shared standards in
`/Users/willem/Code/standards/` (index: `standards/STANDARDS.md`; future
standards: `standards/ROADMAP.md`).

- **Presentations:** build every deck from
  `standards/powerpoint template/Willem-Default.potx` (theme "Helena": Neue Haas
  Grotesk Text Pro, 16:9, black on white with a gray ramp, template v2). Spec:
  `standards/powerpoint template/STANDARD.md`. Generate with
  `standards/powerpoint template/house_style.py` (open
  `Willem-Default-Base.pptx`, never the `.potx`) and gate with
  `standards/powerpoint template/deck_checks.py` before calling a deck done.
- **Deck rules:** no speaker notes in submitted decks; editable shapes, not
  chart images; numbered, linked superscript citations with a final References
  slide; no bottom rules, citation strips, or page counters; footer text only
  when a course or client requires it (for example `ME460 HWx`), which overrides
  the default of no footer; export the deliverable PDF with native PowerPoint
  and use LibreOffice renders only for QA.
- **Everything else:** do not invent facts, dates, or numbers; mark unknowns TBD
  and point at the source. Keep copyrighted course material out of git. This
  block is managed by `standards/tools/apply_standards.py`; edit
  `standards/ai-files/BLOCK-root.md`, not this copy.
- **AI use (school work):** no AI-generated or AI-modified images in any school
  deliverable; AI-written deliverable text only with written adviser
  pre-clearance (`docs/ai-clearances/`); never cite an AI tool as a source;
  never edit graded report text (the repo's `protected-paths.txt`; example:
  `standards/enforcement/senior-design-repo/sd-protected-paths.txt`). AI-use
  logging and attestation are opt-in per repo via `ai-attestation-roots.txt`;
  see `standards/standards/ai-use-disclosure/ai-use-disclosure.md`.
- **AI files:** one `AGENTS.md` (≤ 200 lines, Clarity verbatim); `CLAUDE.md` is
  `@AGENTS.md`. Gates: `standards/tools/agents_md_lint.py`, `ai_file_lint.py`.
<!-- standards:end -->
