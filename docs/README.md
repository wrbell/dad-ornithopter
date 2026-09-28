# MCO-1D Documentation Architecture

The documentation in this repository is organized into three tiers:

## 1. MCO-1D Mark 1 Authoritative Suite (`docs/mark1/`)
**[PRIMARY AUTHORITATIVE BASELINE]**
All physical manufacturing, component sourcing, electronics commissioning, flight test execution, and piloting instructions are consolidated into this unified suite:
- [`MCO-1D_Mark1_Master_Specification.md`](mark1/MCO-1D_Mark1_Master_Specification.md) — Master aerodynamic, kinematic, and power specification.
- [`MCO-1D_Mark1_Bill_of_Materials.md`](mark1/MCO-1D_Mark1_Bill_of_Materials.md) — Live 39-component BOM ($3,757.58 USD, 6.150 kg MTOW with spares), auto-updated whenever viewed via `../view_bom.sh`.
- [`MCO-1D_Mark1_Manufacturing_Guide.md`](mark1/MCO-1D_Mark1_Manufacturing_Guide.md) — 216-rib additive print run (LW-PLA, 1.5g), CF machining, spar layup, and 180 N Dyneema rigging.
- [`MCO-1D_Mark1_Commissioning_Manual.md`](mark1/MCO-1D_Mark1_Commissioning_Manual.md) — Hardware assembly, 8x ODrive S1 CAN-FD configuration, 16x AGF-RC servo calibration, ADC zeroing.
- [`MCO-1D_Mark1_Operator_Manual.md`](mark1/MCO-1D_Mark1_Operator_Manual.md) — Pilot Operating Handbook (POH), Unified Lever zones (100% Sprint to 0% Max Airbrake), Gemini copilot integration, failsafes.
- [`MCO-1D_Mark1_Flight_Test_Protocol.md`](mark1/MCO-1D_Mark1_Flight_Test_Protocol.md) — 4-phase test cards: TC-01 (Bench), TC-02 (Tethered lift), TC-03 (Truck sweep), TC-04 (First free flight STOL).
- [`MCO-1D_Mark1_Software_Reference.md`](mark1/MCO-1D_Mark1_Software_Reference.md) — Real-time 50 Hz executive loop, Boyd $P_s$ energy management, CAN-FD frame dictionary, 368 command catalogue.

## 2. Pre-Consolidation Snapshots (`docs/latest/`)
**[SUPERSEDED ARCHIVE]**
Historical PDF and DOCX documents representing Revisions v18–v25. These are preserved for design history and DO-178C engineering audit trails.

## 3. Engineering Provenance Archive (`docs/archive/`)
**[HISTORICAL ARCHIVE]**
Contains 66 early design revisions (v1–v17 BOMs, v1–v24 manuals) dating back to prototype inception.
