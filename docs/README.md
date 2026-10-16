# MCO-1D Documentation Architecture

The documentation in this repository is organized into three tiers:

## 1. MCO-1D Mark 1 Authoritative Suite (`docs/mark1/`)
**[PRIMARY AUTHORITATIVE BASELINE]**
All physical manufacturing, component sourcing, electronics commissioning, flight test execution, and piloting instructions are consolidated into this unified suite:
- [`MCO-1D_Mark1_Master_Specification.md`](mark1/MCO-1D_Mark1_Master_Specification.md) — Master aerodynamic, kinematic, and power specification.
- [`MCO-1D_Mark1_Builder_Calibration_Sheet.pdf`](mark1/MCO-1D_Mark1_Builder_Calibration_Sheet.pdf) ([`Markdown`](mark1/MCO-1D_Mark1_Builder_Calibration_Sheet.md)) — Primary post-build hardware handoff manifest (`FIG-01` to `FIG-08`), as-built gross weight calibration gate, and pre-flight signoff checklist.
- [`MCO-1D_Mark1_Physical_Readiness_Audit.md`](mark1/MCO-1D_Mark1_Physical_Readiness_Audit.md) — Uncompromising adversarial hardware audit, 10:1 capstan drivetrain verification, sprint thermal duty clamp, and WIG floor safety governance.
- [`MCO-1D_Mark1_Flight_Envelope_Specification.md`](mark1/MCO-1D_Mark1_Flight_Envelope_Specification.md) — Flight operating corridors, $V$-$n$ diagram ($\pm 2.20g$ symmetric structural limits), dynamic stall scaling, and service ceilings.
- [`MCO-1D_Mark1_Bill_of_Materials.md`](mark1/MCO-1D_Mark1_Bill_of_Materials.md) — Live 39-component BOM ($3,757.58 USD, 5.367 kg baseline with GoolRC 5010-14 motors), auto-updated whenever viewed via `../view_bom.sh`.
- [`MCO-1D_Mark1_Manufacturing_Guide.md`](mark1/MCO-1D_Mark1_Manufacturing_Guide.md) — 216-rib additive print run (LW-PLA, 1.5g), CF machining, spar layup, reinforced battery tray mount, and 180 N Dyneema rigging.
- [`MCO-1D_Mark1_Commissioning_Manual.md`](mark1/MCO-1D_Mark1_Commissioning_Manual.md) — Hardware assembly, 8x ODrive S1 CAN-FD configuration (GoolRC 360KV & MN5008 upgrade), 16x AGF-RC servo calibration, ADC zeroing.
- [`MCO-1D_Mark1_Operator_Manual.md`](mark1/MCO-1D_Mark1_Operator_Manual.md) — Pilot Operating Handbook (POH), Unified Lever zones, Hard WIG Floor governor, PPL/CPL & aerobatic operations, Gemini copilot integration, failsafes.
- [`MCO-1D_Mark1_Flight_Test_Protocol.md`](mark1/MCO-1D_Mark1_Flight_Test_Protocol.md) — 4-phase test cards: TC-01 (Bench), TC-02 (Tethered lift), TC-03 (Truck sweep), TC-04 (First free flight STOL).
- [`MCO-1D_Mark1_Test_Card_Worksheets.md`](mark1/MCO-1D_Mark1_Test_Card_Worksheets.md) — Field data acquisition worksheets and printable PDF binder.
- [`MCO-1D_Mark1_Software_Reference.md`](mark1/MCO-1D_Mark1_Software_Reference.md) — Real-time 50 Hz executive loop, HardWIGFloorGovernor, SymmetricLoadFactorGovernor, Boyd $P_s$ energy management, CAN-FD frame dictionary.
- [`MCO-1D_Hierarchical_Exploded_Installation_Guide.pdf`](mark1/MCO-1D_Hierarchical_Exploded_Installation_Guide.pdf) — Complete 2-page hierarchical installation manual (Bevel Wrist -> Stagecoach Shoulder -> Fuselage Collar).
- [`MCO-1D_V10_Powertrain_Mount_Installation_Guide.pdf`](mark1/MCO-1D_V10_Powertrain_Mount_Installation_Guide.pdf) — V10 Powertrain Stand, Dual-Shear Top Brace, and ODrive Micro mounting guide.
- [`MCO-1D_V10_Stagecoach_Wrist_Assembly_Guide.pdf`](mark1/MCO-1D_V10_Stagecoach_Wrist_Assembly_Guide.pdf) — V10 7-spoke Stagecoach sector drum and bevel differential wrist guide.
- [`MCO-1D_Electronics_and_Systems_Guide.pdf`](mark1/MCO-1D_Electronics_and_Systems_Guide.pdf) — Avionics, CAN-FD bus topology, and dual-zone 7.4V servo power distribution guide.

## 2. Pre-Consolidation Snapshots (`docs/latest/`)
**[SUPERSEDED ARCHIVE]**
Historical PDF and DOCX documents representing Revisions v18–v25. These are preserved for design history and DO-178C engineering audit trails.

## 3. Engineering Provenance Archive (`docs/archive/`)
**[HISTORICAL ARCHIVE]**
Contains 66 early design revisions (v1–v17 BOMs, v1–v24 manuals) dating back to prototype inception.
