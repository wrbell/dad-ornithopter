# MCO-1D Living Project State & Engineering Dashboard

**Airframe**: MCO-1D Metachronal Cascade Ornithopter Prototype (8 Wings)  
**Single Source of Truth**: [`config/airframe_spec.json`](config/airframe_spec.json) & [`bom/bom.csv`](bom/bom.csv)  
**Engineering Audit & Roadmap**: [`willem-suggestions.md`](willem-suggestions.md)  

---

## 1. Active Firmware & Hardware Baseline

| Subsystem | Baseline Specification | Status / Notes |
| :--- | :--- | :--- |
| **Flap Motors** | **8x GoolRC 5010-14 360KV** | `95 g` each (`760 g` total), 7 pole pairs, `10.0:1` stepped capstan reduction (`Ø154.0 mm` sector wheel / `Ø15.4 mm` pinion drum). |
| **Flapping Travel** | **200° Sector Wheel (±50° Wing Flap)** | `±100°` sector wheel rotation (`10.0:1` ratio -> `±50°` wing flap amplitude). Frequency ceiling `3.5 Hz`. |
| **Motor Controllers** | **8x ODrive S1 / Micro (`can0`)** | CAN nodes `0..7` at `1 Mbps`. Soft current limit `25.0 A`, hard clamp `35.0 A`. Reciprocating velocity oscillation enforced in [`mco_odrive_interface.py`](firmware/mco_odrive_interface.py). |
| **Servo Actuators** | **16x AGFRC High-Torque Servos** | 8 pitch (`GPIO [4, 5, 6, 12, 13, 16, 17, 18]`) and 8 sweep (`GPIO [19, 20, 21, 22, 23, 24, 25, 26]`), 333 Hz PWM via `pigpio`. |
| **Servo Power Bus** | **Dual 20A HV SBECs (7.4 V)** | Powered from 6S main bus + 2x `4,700 µF` low-ESR buffer capacitors (`PWR-02`, `PWR-04`). |
| **Main Battery** | **6S 5000 mAh 50C LiPo (22.2 V)** | `725 g` flight pack (`PWR-01`) mounted on sliding keel tray (`230–340 mm` arm, nominal `250 mm`). |
| **Flight Computer** | **Raspberry Pi 5 (8GB) + Pixhawk 4 Mini** | 50 Hz synchronous loop in [`firmware/mco_main.py`](firmware/mco_main.py) with Waveshare MCP2515 CAN Hat (`ELEC-03`). |
| **Configuration Loader** | [`firmware/mco_config.py`](firmware/mco_config.py) | Loads all constants from [`config/airframe_spec.json`](config/airframe_spec.json) (`MCOConfig`). |

---

## 2. Mass & Geometry Reality (BOM vs. As-Built Hardware)

- **Installed Airframe BOM Mass**: **`3.642 kg (3,642 g / 35.72 N)`** across 39 line items in [`bom/bom.csv`](bom/bom.csv) (`6.150 kg` total shipped procurement mass including workshop tools and spares; `$3,757.58 USD`).
- **As-Built Weighed Mass**: **`TBD (measure on hardware)`** — must be weighed on a calibrated scale prior to TC-01/TC-02 and entered into [`MCO-1D_Mark1_Builder_Calibration_Sheet.md`](docs/mark1/MCO-1D_Mark1_Builder_Calibration_Sheet.md) (`FIG-01`).
- **Wing Cascade Geometry**:
  - **Effective Wingspan**: `2.20 m` (`1.00 m` per wing panel + `0.20 m` root spacing).
  - **Root Chord**: `0.28 m` (`280 mm`); **Total Wing Area**: `2.24 m²` (`0.28 m²` per wing across 8 wings).
  - **Longitudinal Stagger**: `0.138 m` (`138 mm`) between stations (`X = 120, 258, 396, 534 mm`).
- **Longitudinal Center of Gravity (CG)**:
  - **Analytical Estimate**: `278.5 mm` aft of nose datum with 6S battery at `250 mm` (`IN ENVELOPE: 265–292 mm` per [`firmware/mco_cg_estimator.py`](firmware/mco_cg_estimator.py)).
  - **As-Built Measured CG**: **`TBD (measure on hardware)`** (`FIG-02`).

---

## 3. Flight Test & Clearance Matrix

All hardware test cards require physical instrumentation and signed field worksheets in [`docs/mark1/worksheets/`](docs/mark1/worksheets/). Synthetic bench outputs from `--mock` mode are for software harness testing only and must never be recorded as hardware qualification.

| Test ID | Description | Hardware Status | Governing Script / Module |
| :---: | :--- | :---: | :--- |
| **TC-01** | Ground Bench Test (Rigid post + load cell, 0 to 3.5 Hz sweep) | **TBD (measure on hardware)** | [`scripts/run_tc01_bench_checkout.py`](scripts/run_tc01_bench_checkout.py) |
| **TC-02** | Tethered Lift Characterization (Hover freq hunt, lift curve) | **PENDING TC-01** | [`firmware/mco_test_cards.py`](firmware/mco_test_cards.py) |
| **TC-03** | Truck Advance-Ratio Sweep ($C_T$ vs $J$, 0–14 m/s airflow) | **PENDING TC-02** | [`firmware/mco_test_cards.py`](firmware/mco_test_cards.py) |
| **TC-04** | First Unrestrained Free Flight (STOL takeoff & flare) | **LOCKED (Requires TC-01..03)** | [`firmware/mco_test_cards.py`](firmware/mco_test_cards.py) |

---

## 4. Engineering Remediation Tracker

See [`willem-suggestions.md`](willem-suggestions.md) for the complete technical post-mortem and step-by-step remediation guide.

| Item ID | Priority | Description | Status |
| :---: | :---: | :--- | :---: |
| **REM-01** | **CRITICAL** | **Single Source of Truth (`config/airframe_spec.json` & `bom/bom.csv`)**: Unified all firmware, CAD, and BOM scripts around `MCOConfig` (`3.642 kg` installed mass, `2.20 m` span, `10.0:1` capstan ratio, 6S + 2x SBEC power bus). | **RESOLVED** |
| **REM-02** | **CRITICAL** | **Capstan Unidirectional Crash Bug (`firmware/mco_odrive_interface.py`)**: Fixed `set_flap_freq()` and `set_cascade_freq()` so they command zero-mean reciprocating velocity instead of continuous rotation into mechanical stops. | **RESOLVED** |
| **REM-03** | **CRITICAL** | **GPIO Servo Pin Collision (`firmware/mco_main.py`)**: Removed local override of `SWEEP_PINS` that collided on 5 GPIO pins (`4, 5, 6, 13, 17`) with `PITCH_PINS`. | **RESOLVED** |
| **REM-04** | **HIGH** | **Repository De-Bloating & Quarantine**: Removed ~710 MB of fake STL-to-STEP duplicates, `.bgcode`, bread stamps, and LEGO files; quarantined duplicate firmware modules, fake FEA/CFD scripts, and 47 tautological test suites into `archive/`. | **RESOLVED** |
| **REM-05** | **HIGH** | **CAD Upper Bearing Seat & Bevel Gear Tooth Geometry**: Fix boolean order in [`scripts/cad/generate_stagecoach_wrist_v10.py`](scripts/cad/generate_stagecoach_wrist_v10.py) (servo bay cut obliterates `Ø19 mm` upper bearing seat) and replace trapezoidal slots in [`scripts/cad/generate_printable_bevel_cartridge_v9.py`](scripts/cad/generate_printable_bevel_cartridge_v9.py) with true octoidal bevel teeth. | **OPEN (See `willem-suggestions.md`)** |
| **REM-06** | **HIGH** | **Replace 102k-Line `mco_wave_library.py` with Clean 500-Line Kinematics Engine**: Extract core 4-pair metachronal phase oscillator and wire per-wing trajectories directly into `MCOHardware.apply_commands()`. | **OPEN (See `willem-suggestions.md`)** |

---

## 5. Flight Hours & Airframe History

- **Airframe Serial**: MCO-1D-001
- **Total Flights**: 0 (`TBD — awaiting physical build and TC-01..TC-04`)
- **Total Flight Hours**: 0.00 h
- **Rope Drive Cycles**: 0
