# MCO-1D Master Catalog & File Index

Comprehensive index of all assets in the MCO-1D repository grouped into 10 functional engineering domains.

> **Authoritative vs Archive Legend**:  
> - **[ACTIVE]**: Current authoritative document or production code to be used for the physical build and flight operations.  
> - **[ARCHIVE]**: Historical design iteration or earlier revision retained for engineering provenance and audit trace.

## 0. MCO-1D Mark 1 Authoritative Engineering Suite (`docs/mark1/`)

> **Consolidated Engineering Baseline**: The project documentation suite has been rewritten and consolidated from legacy revisions (v1–v25) and multiple appendices into a fresh, authoritative **MCO-1D Mark 1** baseline. All physical fabrication, component procurement, firmware commissioning, flight testing, and piloting reference these 7 core documents.

| File | Status | Description |
| :--- | :---: | :--- |
| [MCO-1D_Mark1_Master_Specification.md](docs/mark1/MCO-1D_Mark1_Master_Specification.md) | **[ACTIVE]** | Authoritative Master Design Specification: cascade geometry, S1223 aerodynamics, STOL performance envelope ($V_{cruise} = 9.0\text{ m/s}, L \ge 37\text{ N}$), 7:1 capstan drivetrain, mass balance, power budget. |
| [MCO-1D_Mark1_Bill_of_Materials.md](docs/mark1/MCO-1D_Mark1_Bill_of_Materials.md) | **[ACTIVE]** | Live 39-component procurement registry ($3,757.58 USD, 6.150 kg MTOW with spares). Automatically updates and verifies pricing when viewed via `./view_bom.sh`. |
| [MCO-1D_Mark1_Manufacturing_Guide.md](docs/mark1/MCO-1D_Mark1_Manufacturing_Guide.md) | **[ACTIVE]** | Complete fabrication manual: 216 universal rib print run (LW-PLA, 1.5g), CNC cutting of 2mm CF sector wheels, spar assembly, 180 N Dyneema pre-tensioning, slide tray. |
| [MCO-1D_Mark1_Commissioning_Manual.md](docs/mark1/MCO-1D_Mark1_Commissioning_Manual.md) | **[ACTIVE]** | Step-by-step airframe assembly, 8x ODrive S1 CAN-FD configuration, 16x AGF-RC servo 170° travel programming, MCP3208 SPI ADC calibration, pre-flight BIT. |
| [MCO-1D_Mark1_Operator_Manual.md](docs/mark1/MCO-1D_Mark1_Operator_Manual.md) | **[ACTIVE]** | Pilot Operating Handbook (POH): Unified Lever zones (100% Sprint to 0% Max Airbrake, unified 50% Trim Detent for cruise & hover lock), STOL takeoff/landing profiles, helicopter hover transition ($V \le 3\text{ m/s}$), Gemini copilot integration (port 5759), emergency checklists. |
| [MCO-1D_Mark1_Flight_Test_Protocol.md](docs/mark1/MCO-1D_Mark1_Flight_Test_Protocol.md) | **[ACTIVE]** | 4-Phase progressive test clearance: TC-01 (Ground bench static thrust), TC-02 (Tethered lift hunt), TC-03 (Truck advance-ratio sweep), TC-04 (First free flight STOL). |
| [MCO-1D_Mark1_Software_Reference.md](docs/mark1/MCO-1D_Mark1_Software_Reference.md) | **[ACTIVE]** | Real-time software architecture: 50 Hz executive loop breakdown, Boyd $P_s$ PI controller, 232 biomimetic hypotheses (70 advanced suites), software-only reversal recoil engine, CAN-FD frame dictionary, and 368 ground station command catalogue. |
| [MCO-1D_Mark1_Adversarial_Audit_Report.md](docs/mark1/MCO-1D_Mark1_Adversarial_Audit_Report.md) | **[ACTIVE]** | Comprehensive adversarial engineering audit: aerodynamic lift deficit, torque limits at 3.5 Hz, CAD spar bore correction, runtime crash remediations, and safety matrix. |
| [view_bom.sh](view_bom.sh) | **[ACTIVE]** | Root executable convenience script: updates pricing from verified vendors, regenerates BOM Markdown, and displays formatted terminal report. |
| [mco_bom_tool.py](scripts/mco_bom_tool.py) | **[ACTIVE]** | Standalone zero-dependency Python tool driving the live BOM pricing and MTOW calculation engine. |

---

## 1. Core Flight Software & Executive (firmware/)

| File | Status | Description |
| :--- | :---: | :--- |
| [mco_main.py](firmware/mco_main.py) | **[ACTIVE]** | Main 50 Hz flight computer executive loop, sensor polling, command dispatch, failsafe handling. |
| [mco_wave_library.py](firmware/mco_wave_library.py) | **[ACTIVE]** | Authoritative 90k+ line metachronal kinematics library containing all 232 biomimetic hypotheses (H-01 to H-108, H-88A dynamic vectoring, H-88B sweep bias trim, etc.), 5-modality Adaptive Learning Classifier (ALC), and ALSS adaptive state engine. |
| [mco_flight_governor.py](firmware/mco_flight_governor.py) | **[ACTIVE]** | Flight governor & pilot intent arbitration: software-only reversal recoil engine (`compute_soft_recoil_trajectory`), aerodynamic spring pitch advance (`compute_aero_spring_pitch_advance`), and salvaged hypothesis governor (H-91, H-101, H-108). |
| [mco_advanced_physics.py](firmware/mco_advanced_physics.py) | **[ACTIVE]** | Advanced aerodynamic physics suite containing 70 advanced hypotheses (H-37..H-108, H-LSB-1, H-DSH-1, H-AC-SHIFT-1, H-SHED-1, H-VEX-1, H-88A, H-88B), ALC evolutionary genome optimization, and bounded parameter safety clamps. |
| [mco_intent_control.py](firmware/mco_intent_control.py) | **[ACTIVE]** | Three-layer intent arbitration stack, unified lever parsing (100% Sprint to 0% Airbrake), Boyd energy-maneuvering controller. |
| [mco_wave_library_clean.py](firmware/mco_wave_library_clean.py) | **[ACTIVE]** | Reference clean baseline of wave kinematics library prior to patch integration. |
| [mco_wave_library_pre_audit.py](firmware/mco_wave_library_pre_audit.py) | **[ACTIVE]** | Pre-audit snapshot of wave kinematics library retained for DO-178C traceability. |
| [wing_drone_firmware.py](firmware/wing_drone_firmware.py) | **[ACTIVE]** | Consolidated prototype firmware reference and self-contained flight model. |

---

## 2. Hardware Interfaces & Drivetrain Drivers (firmware/, scripts/)

| File | Status | Description |
| :--- | :---: | :--- |
| [mco_odrive_interface.py](firmware/mco_odrive_interface.py) | **[ACTIVE]** | High-speed CAN-FD interface for 8x ODrive S1 motor controllers (nodes 0–7). |
| [mco_odrive_setup.py](firmware/mco_odrive_setup.py) | **[ACTIVE]** | Automated provisioner and calibration utility for ODrive S1 controllers (pole pairs, current limits, vel_limit). |
| [mco_servo_interface.py](firmware/mco_servo_interface.py) | **[ACTIVE]** | Hardware PWM controller via pigpio daemon for 16x AGFRC IB53BHU pitch and sweep servos. |
| [mco_mavlink_bridge.py](firmware/mco_mavlink_bridge.py) | **[ACTIVE]** | Serial MAVLink bridge interfacing Pixhawk 4 Mini (telemetry, GPS, pitot airspeed, RC receiver). |
| [mco_mavlink.py](firmware/mco_mavlink.py) | **[ACTIVE]** | MAVLink message definition parser and serial packet handler. |
| [mco_loadcell.py](firmware/mco_loadcell.py) | **[ACTIVE]** | HX711 24-bit ADC interface for 5 kg S-type load cell static thrust measurements. |
| [mco_can_setup.sh](scripts/mco_can_setup.sh) | **[ACTIVE]** | Linux socketCAN bringup script configuring CAN-FD at 1 Mbps nominal / 8 Mbps data. |
| [mco-flight.service](scripts/mco-flight.service) | **[ACTIVE]** | Systemd service unit managing autostart of mco_main.py on boot. |

---

## 3. Flight Operations, Pre-Flight BIT & Test Cards (firmware/, docs/latest/)

| File | Status | Description |
| :--- | :---: | :--- |
| [mco_preflight.py](firmware/mco_preflight.py) | **[ACTIVE]** | Three-tier Built-In Test (BIT) framework: P-BIT (Power-up), Pre-Flight BIT, and C-BIT (Continuous). |
| [mco_preflight_checks.py](firmware/mco_preflight_checks.py) | **[ACTIVE]** | Automated pre-flight verification script checking bus voltages, sensors, ADCs, and SD space. |
| [mco_test_cards.py](firmware/mco_test_cards.py) | **[ACTIVE]** | Automated test card engine for TC-01 (Bench), TC-02 (Hover hunt), TC-03 (Truck test), TC-04 (Free flight). |
| [mco_first_run_wizard.py](firmware/mco_first_run_wizard.py) | **[ACTIVE]** | Interactive commissioning wizard guiding new hardware bringup and calibration. |
| [mco_rc_verify.py](firmware/mco_rc_verify.py) | **[ACTIVE]** | Transmitter channel mapping and lever detent verification tool. |
| [mco_vibration_check.py](firmware/mco_vibration_check.py) | **[ACTIVE]** | MPU-6050 accelerometer spectrum analyzer checking structural resonance and rope slip. |
| [mco_taxi_park.py](firmware/mco_taxi_park.py) | **[ACTIVE]** | Taxi, ground handling, and wing folding/parking kinematics coordinator. |
| [MCO-1D_Test_Cards_v25.pdf](docs/latest/MCO-1D_Test_Cards_v25.pdf) | **[ACTIVE]** | Complete operational test card cards and test protocols for flight clearance. |
| [MCO-1D_Field_Cards_v25.pdf](docs/latest/MCO-1D_Field_Cards_v25.pdf) | **[ACTIVE]** | Quick-reference laminated field checklist for pilot and flight engineer. |
| [MCO-1D_Operator_Manual_v25.pdf](docs/latest/MCO-1D_Operator_Manual_v25.pdf) | **[ACTIVE]** | Pilot operating handbook, normal/abnormal procedures, lever zones, and emergency checklists. |
| [MCO-1D_Commissioning_Manual_v25.pdf](docs/latest/MCO-1D_Commissioning_Manual_v25.pdf) | **[ACTIVE]** | Step-by-step mechanical assembly, electrical wiring, and first-flight commissioning manual. |
| [MCO-1D_Build_Measurement_Sheet.pdf](docs/latest/MCO-1D_Build_Measurement_Sheet.pdf) | **[ACTIVE]** | Physical measurement sheet recording actual as-built weights, spans, and sector arcs. |
| [MCO-1D_Preflight_Information_Required.md](docs/latest/MCO-1D_Preflight_Information_Required.md) | **[ACTIVE]** | Markdown guide to required physical measurements feeding mco_build_measurements.py. |

---

## 4. Adaptive Learning, Optimizers & Aerodynamic Modeling (firmware/)

| File | Status | Description |
| :--- | :---: | :--- |
| [mco_adaptive_optimiser.py](firmware/mco_adaptive_optimiser.py) | **[ACTIVE]** | Biomimetic hypothesis explorer adjusting metachronal phase, dihedral, and AoA dynamically. |
| [mco_optimiser.py](firmware/mco_optimiser.py) | **[ACTIVE]** | Offline / online numerical optimizer for flap stroke waveforms. |
| [mco_performance_model.py](firmware/mco_performance_model.py) | **[ACTIVE]** | First-principles aerodynamic model calculating thrust, power, lift, and stall speed. |
| [mco_cg_estimator.py](firmware/mco_cg_estimator.py) | **[ACTIVE]** | Static and in-flight Center of Gravity estimator implementing FAA-H-8083-1B moment arm equations. |
| [mco_build_measurements.py](firmware/mco_build_measurements.py) | **[ACTIVE]** | Interactive tool recording physical aircraft measurements to knowledge/build_measurements.json. |
| [mco_hypothesis_tool.py](firmware/mco_hypothesis_tool.py) | **[ACTIVE]** | Ground station CLI for inspecting and mutating biomimetic hypotheses across all 232 hypotheses (H-01 to H-108 + advanced variants). |
| [mco_sync.py](firmware/mco_sync.py) | **[ACTIVE]** | Two-way synchronizer syncing flight logs and learning state between Pi and ground station. |

---

## 5. Telemetry, Blackbox Logging & Analysis (firmware/)

| File | Status | Description |
| :--- | :---: | :--- |
| [mco_logger.py](firmware/mco_logger.py) | **[ACTIVE]** | High-rate CSV blackbox flight recorder logging 50 Hz telemetry to /home/pi/logs/. |
| [mco_flight_recorder.py](firmware/mco_flight_recorder.py) | **[ACTIVE]** | Structured flight summary recorder saving post-flight debriefs to flight_logs/. |
| [mco_log_analysis.py](firmware/mco_log_analysis.py) | **[ACTIVE]** | Post-flight telemetry log parser extracting handling qualities and power metrics. |
| [mco_flight_test_protocol.py](firmware/mco_flight_test_protocol.py) | **[ACTIVE]** | Automated flight test executor recording structured aerodynamic sorties. |

---

## 6. Certification, FMEA & Safety Tools (firmware/, docs/latest/)

| File | Status | Description |
| :--- | :---: | :--- |
| [mco_certification_tools.py](firmware/mco_certification_tools.py) | **[ACTIVE]** | Automated DO-178C requirements tracer, FMEA generator, and MIL-STD-1797B analyzer. |
| [MCO_1D_Technical_Handover_v2.docx](docs/latest/MCO_1D_Technical_Handover_v2.docx) | **[ACTIVE]** | Complete Technical Handover Document Revision 2 detailing all engineering decisions. |
| [MCO_Master_Specification_v2.pdf](docs/latest/MCO_Master_Specification_v2.pdf) | **[ACTIVE]** | Authoritative master design specification for MCO-1D and MCO-1A programmes. |

---

## 7. Ground Station Servers, Bridges & Proxies (firmware/, scripts/)

| File | Status | Description |
| :--- | :---: | :--- |
| [mco_ground_station.py](firmware/mco_ground_station.py) | **[ACTIVE]** | Standalone terminal Ground Station with interactive telemetry and command CLI. |
| [mco_gs_server.py](firmware/mco_gs_server.py) | **[ACTIVE]** | HTTP and WebSocket server hosting browser-based Ground Station UI. |
| [mco_ws_bridge.py](firmware/mco_ws_bridge.py) | **[ACTIVE]** | Bi-directional WebSocket bridge streaming 50 Hz telemetry to web clients. |
| [mco_api_proxy.py](firmware/mco_api_proxy.py) | **[ACTIVE]** | Ground station AI assistant proxy routing queries to Anthropic Claude Sonnet. |
| [mco_panel.py](firmware/mco_panel.py) | **[ACTIVE]** | Waveshare 3.5 inch SPI touchscreen panel interface for onboard field diagnostics. |
| [mco-panel.service](scripts/mco-panel.service) | **[ACTIVE]** | Systemd service managing onboard touchscreen UI. |

---

## 8. User Interface, Dashboards & Visualizers (ui/)

| File | Status | Description |
| :--- | :---: | :--- |
| [mco_ai_ground_station.jsx](ui/mco_ai_ground_station.jsx) | **[ACTIVE]** | Mission Control dashboard with live telemetry meters, status alerts, and AI copilot window. |
| [performance_dashboard.jsx](ui/performance_dashboard.jsx) | **[ACTIVE]** | Real-time aerodynamic performance visualizer (lift curves, power curves, Ps tracking). |
| [wing_drone_visualizer.jsx](ui/wing_drone_visualizer.jsx) | **[ACTIVE]** | 3D canvas visualizer rendering live 8-wing flapping, sweeping, and pitching motions. |
| [airfoil_maneuver_guide.jsx](ui/airfoil_maneuver_guide.jsx) | **[ACTIVE]** | Interactive aerodynamic reference guide illustrating vortex generation and S1223 flow. |
| [mco_gs_assistant.jsx](ui/mco_gs_assistant.jsx) | **[ACTIVE]** | Chat interface component for Ground Station AI assistant. |

---

## 9. Mechanical Drawings & CAD Specifications (cad/drawings/)

| File | Status | Description |
| :--- | :---: | :--- |
| [MCO_1D_Rib_Drawing.pdf](cad/drawings/MCO_1D_Rib_Drawing.pdf) | **[ACTIVE]** | Dimensioned engineering drawing of universal S1223 rib with 2.2mm spar holes and D-box. |
| [MCO_1D_Wing_Construction.pdf](cad/drawings/MCO_1D_Wing_Construction.pdf) | **[ACTIVE]** | Rev H2 wing construction drawing showing spar stations, rib spacing, and membrane. |
| [MCO_1D_wing_design.html](cad/drawings/MCO_1D_wing_design.html) | **[ACTIVE]** | Interactive HTML schematic of S1223 airfoil cross-section and structural stations. |
| [MCO_1D_wing_schematic.html](cad/drawings/MCO_1D_wing_schematic.html) | **[ACTIVE]** | Detailed internal structural schematic of wing ribs, CF spars, and hinge plate. |
| [MCO_1D_VG_Design.pdf](cad/drawings/MCO_1D_VG_Design.pdf) | **[ACTIVE]** | Vortex generator placement, height, and spacing spec along 17%c chord line. |
| [MCO_1D_wing_design.pdf](cad/drawings/MCO_1D_wing_design.pdf) | **[ACTIVE]** | Static PDF export of wing design specification. |
| [MCO_1D_wing_manufacturing.pdf](cad/drawings/MCO_1D_wing_manufacturing.pdf) | **[ACTIVE]** | Early wing manufacturing instructions (superseded by v20). |
| [MCO_fuselage_spec.html](cad/drawings/MCO_fuselage_spec.html) | **[ACTIVE]** | Interactive schematic of fuselage keel line and cross-sections. |
| [MCO_1A_schematic.html](cad/drawings/MCO_1A_schematic.html) | **[ACTIVE]** | MCO-1A full-scale piloted aircraft tadpole fuselage and wing cascade schematic. |
| [MCO_wave_lever.html](cad/drawings/MCO_wave_lever.html) | **[ACTIVE]** | Interactive schematic of unified pilot wave lever mechanism. |

---

## 10. Pre-Consolidation Document Archive & Snapshots (`docs/latest/`)

> **Note on Pre-Consolidation Files**: The PDF documents in `docs/latest/` reflect pre-consolidation design milestones (Revisions v18–v25) prior to the **MCO-1D Mark 1** unification. For current active construction, commissioning, operations, and flight clearance, refer to [Section 0: MCO-1D Mark 1 Authoritative Engineering Suite (`docs/mark1/`)](#0-mco-1d-mark-1-authoritative-engineering-suite-docsmark1).

| File | Status | Description |
| :--- | :---: | :--- |
| [MCO-1D_Bill_of_Materials_v18.pdf](docs/latest/MCO-1D_Bill_of_Materials_v18.pdf) | **[SUPERSEDED]** | Pre-consolidation BOM (Rev 18, superseded by `docs/mark1/MCO-1D_Mark1_Bill_of_Materials.md`). |
| [MCO_1D_BOM_USD.pdf](docs/latest/MCO_1D_BOM_USD.pdf) | **[ACTIVE]** | USD pricing breakdown and line-item budget summary. |
| [MCO_1D_Electronics_BOM.pdf](docs/latest/MCO_1D_Electronics_BOM.pdf) | **[ACTIVE]** | Focused electrical BOM specifying wiring, connectors, and passive components. |
| [MCO-1D_Software_Reference_v25.pdf](docs/latest/MCO-1D_Software_Reference_v25.pdf) | **[ACTIVE]** | Comprehensive software architecture reference manual (Rev 25). |
| [MCO-1D_Wing_Manufacturing_v20.pdf](docs/latest/MCO-1D_Wing_Manufacturing_v20.pdf) | **[ACTIVE]** | Authoritative wing fabrication manual (universal 120mm rib, 216-rib print run, spar cuts). |
| [MCO-1D_Wing_Manufacturing_Addendum_R21.pdf](docs/latest/MCO-1D_Wing_Manufacturing_Addendum_R21.pdf) | **[ACTIVE]** | Addendum R21 detailing TPU feather winglet sleeve manufacture and commissioning. |
| [MCO1D_ODrive_Settings.pdf](docs/latest/MCO1D_ODrive_Settings.pdf) | **[ACTIVE]** | First-principles ODrive S1 motor parameter and velocity tuning manual. |
| [MCO1D_ODrive_VelLimit_Diagnostic.pdf](docs/latest/MCO1D_ODrive_VelLimit_Diagnostic.pdf) | **[ACTIVE]** | Root cause fault analysis resolving 2 Hz motor stalling via vel_limit adjustment. |
| [MCO1D_RattanRod_StressTest.pdf](docs/latest/MCO1D_RattanRod_StressTest.pdf) | **[ACTIVE]** | Static bending and torsional stress test data on flexible wing rod materials. |
| [MCO_1D_Control_Map.pdf](docs/latest/MCO_1D_Control_Map.pdf) | **[ACTIVE]** | Full transmitter stick and switch allocation map across all modes. |
| [MCO_1D_Preflight_Card.pdf](docs/latest/MCO_1D_Preflight_Card.pdf) | **[ACTIVE]** | Single-page laminated pre-flight inspection checklist. |
| [MCO_1D_addendum_gear_stability.pdf](docs/latest/MCO_1D_addendum_gear_stability.pdf) | **[ACTIVE]** | Engineering analysis on landing gear stance and ground roll stability. |
| [MCO_1D_engineering_proposal.pdf](docs/latest/MCO_1D_engineering_proposal.pdf) | **[ACTIVE]** | Original engineering design proposal for the MCO-1D prototype. |
| [drone_engineering_proposal.pdf](docs/latest/drone_engineering_proposal.pdf) | **[ACTIVE]** | Programme proposal and research justification. |
| [MCO_Flight_Test_Protocol.txt](docs/latest/MCO_Flight_Test_Protocol.txt) | **[ACTIVE]** | Textual flight test cards and sortie profiles. |
| [MCO_performance.pdf](docs/latest/MCO_performance.pdf) | **[ACTIVE]** | Detailed flight performance calculations (L/D, sink rate, sprint speed). |
| [MCO_wave_guide_v25.pdf](docs/latest/MCO_wave_guide_v25.pdf) | **[ACTIVE]** | Complete guide to metachronal wave kinematics, phase angles, and wake timing. |
| [cascade_flapper_performance.pdf](docs/latest/cascade_flapper_performance.pdf) | **[ACTIVE]** | Aerodynamic wind tunnel and computational fluid dynamics analysis of wing cascades. |
| [gs_command_catalogue.txt](docs/latest/gs_command_catalogue.txt) | **[ACTIVE]** | Exhaustive catalog of all Ground Station prompt commands. |
| [project_state_template.txt](docs/latest/project_state_template.txt) | **[ACTIVE]** | Legacy project state template. |

---

### Historical Archive (`docs/archive/`)

The archive directory contains **66 historical revisions** of BOMs (v1 through v17), Commissioning Manuals (v1 through v24), Operator Manuals (v20 through v24), and early wave guides, preserved for DO-178C engineering provenance.
