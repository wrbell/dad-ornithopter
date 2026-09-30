# MCO-1D Metachronal Cascade Ornithopter Drone

**Autonomous 8-Wing Bio-Inspired Technology Demonstrator**  
*Flight Software & Hardware Engineering Repository*

---

## 1. System Overview

The **MCO-1D** is an advanced unmanned aerial vehicle employing four cascade-arranged wing pairs (8 independent wings: $W_1$ to $W_4$, Port & Starboard) executing a metachronal flapping wave — sequential phase offsets analogous to crustacean and insect propulsion. 

Designed as the sub-scale flight test validator for the crewed **MCO-1A Search-and-Rescue** aircraft, MCO-1D replaces traditional rotary or fixed-wing flight with continuous lift-and-thrust wave kinematics, variable wing sweep (0° to 170°), dynamic pitch morphing, and an intent-driven flight control architecture.

```mermaid
graph TD
    subgraph Pilot & Ground Station
        RC[2.4 GHz SBUS RC Transmitter] -->|Unified Lever + Sticks| PH[Pixhawk 4 Mini Autopilot]
        GS[Ground Station UI / Dashboard] <-->|Telemetry & Commands 50Hz| WSB[mco_ws_bridge / mco_gs_server]
        AI[Gemini 2.5 Flash Copilot Proxy] <-->|Port 5759| GS
    end

    subgraph Flight Control Computer [Raspberry Pi 4B]
        PH -->|MAVLink Serial 921600 baud| MLB[mco_mavlink_bridge.py]
        MLB --> EXE[mco_main.py Executive Loop 50 Hz]
        WSB <--> EXE
        EXE --> INT[mco_intent_control.py Intent Engine]
        INT --> GOV[mco_flight_governor.py Safety & Recoil Engine]
        GOV --> WL[mco_wave_library.py ~99k-Line Wave Kinematics 258 Hyps / 74 Suites]
        WL --> ODRV[mco_odrive_interface.py CAN-FD Bus]
        WL --> SRV[mco_servo_interface.py pigpiod 333Hz PWM]
        
        ADC[MCP3208 12-bit SPI ADCs] -->|Servo Feedback 300-3000mV| EXE
        SENS[MPU-6050 IMU / BMP280 Baro / TF-Luna Lidar] -->|I2C 0x68, 0x76| EXE
        PITOT[MS4525DO Pitot Tube] -->|Airspeed| PH
    end

    subgraph Actuation Drivetrain [8 Wings]
        ODRV -->|CAN 1 Mbps / FD| S1[8× ODrive S1 Controllers]
        S1 -->|Capstan Rope Drive 10:1| MTR[8× T-Motor MN5008 Motors (Option A)]
        MTR -->|±100° Stroke Flapping (200° Arc)| WINGS[8× S1223 Carbon/LW-PLA Wings]
        SRV -->|Pitch 0-45° & Sweep 0-170°| SERVOS[16× AGFRC IB53BHU Servos]
        SERVOS --> WINGS
    end
```

---

## 2. Technical Specifications

| Parameter | Value / Specification | Notes / Governing Document |
| :--- | :--- | :--- |
| **Designation** | MCO-1D Drone Prototype | Flight Test Demonstrator for MCO-1A |
| **Operational MTOW** | 5.687 kg (5,687 g) | With T-Motor MN5008 & 2S pack; balanced at 26.5% MAC |
| **Primary Flight Regime**| STOL Forward-Flight SAR (`H-SAR-FF`) | Catapult/runway launch; hover as bench characterization gate |
| **Wing Configuration** | 4 cascade pairs (8 wings) | Longitudinal stagger: 138 mm between roots |
| **Airfoil** | UIUC S1223 ($C_{L,\max} = 2.40$) | With VGs at 17%c, Gurney flaps at 1%c |
| **Semi-Spans (Root to Tip)** | $W_1$: 772 mm, $W_2$: 749 mm, $W_3$: 735 mm, $W_4$: 712 mm | Rev 20 / Addendum R21 specification |
| **Flap Stroke Amplitude** | $\pm 100^\circ$ ($200^\circ$ circular sector arc) | Full stroke authority at $f \le 2.2\text{ Hz}$, smootherstep profile |
| **Flap Frequency Ceiling** | 3.5 Hz | Capstan 10:1 reduction, ODrive `vel_limit \ge 150` rad/s |
| **Sweep Authority** | $0^\circ$ (full forward) to $170^\circ$ (stowed) | Programmed via AGF-SPV3 programmer |
| **Design Cruise Speed** | 9.0 m/s (17.5 kt) | Clean metachronal cascade ($L > 34\text{ N}$) |
| **Sprint Speed** | 14.0 m/s (27.0 kt) | Forward swept dart configuration |
| **Stall Speed ($V_s$)** | ~6.2 m/s at 5.69 kg MTOW | Dynamic stall protection in `mco_intent_control.py` |
| **Flap Motors** | 8× T-Motor MN5008 ~200KV | 2.2 N·m peak torque, EZO high-load bearings |
| **Main Propulsion Battery** | 6S LiPo 5000 mAh (22.2V nominal) | Quick-pin sliding tray (230–340 mm arm along keel) |
| **Servo Power Bus** | 2S LiPo 3000 mAh 45C (7.4V) | Isolated from motor bus; 16-port PDB + 4,700 µF capacitor |
| **CAN Interface** | Waveshare MCP2518FD Hat | 1 Mbps nominal / 8 Mbps data CAN-FD |

---

## 3. Unified Control Lever Architecture

The MCO-1D introduces an **Intent Control Layer** that maps a single physical lever on the pilot's transmitter (CH3) across all flight regimes, following fixed-wing aviation standard (**100% Full Forward = Sprint / Max Thrust**, **0% Full Aft = Max Airbrake / Emergency Stop**).

Crucially, the **Center Trim Detent (45%–55%)** acts as an intuitive altitude-neutral hold in both flight regimes:
* **In Fixed-Wing Flight ($V \ge 5\text{ m/s}$)**: Level flight cruise equilibrium ($P_s = 0$, zero VSI).
* **In Helicopter / Hover Mode ($V \le 3\text{ m/s}$)**: Stationary hover altitude lock ($0\text{ fpm}$ vertical speed, $1.0\text{g}$ lift).

| Lever Range | Mode | Aerodynamic State & Wing Kinematics |
| :---: | :--- | :--- |
| **88% – 100%** | **Sprint / Max Thrust** | Wings swept forward ($30^\circ$), reduced stroke amplitude, peak flapping frequency ($3.5 - 4.0\text{ Hz}$). CF-1 clap burst below 10 m/s (`H-91`). Peak forward thrust or maximum vertical breakout. |
| **78% – 88%** | **High Thrust / Climb** | Steep climb envelope (`H-92`, `H-71`), metachronal cascade, high specific excess power ($P_s > 0$). |
| **68% – 78%** | **Powered Cruise** | Fast transit, clean metachronal cascade ($90^\circ$ inter-pair phase lag, `H-86`, `H-104`), rapid ground coverage. |
| **55% – 68%** | **Economy Cruise** | Optimized Strouhal number ($St \approx 0.30$), minimum energy consumption per km ($P_s = 0$, `H-94`, `H-107`, `H-49`). |
| **45% – 55%** | **Trim Detent** | **Dual Altitude Equilibrium**: Level cruise in fixed-wing ($P_s = 0$); rock-steady stationary hover hold in helicopter mode ($0\text{ fpm}$). |
| **28% – 45%** | **Glide & Soar** | Flapping frequency throttled ($0.5 - 1.5\text{ Hz}$) or thermal soaring lock (`H-102`, `H-90`, `H-95`, `H-LSB-1`); low sink rate. In hover: controlled vertical descent. |
| **16% – 28%** | **Pigeon Descent** | High-dihedral V-shape ($+40^\circ$ to $+60^\circ$), steep glide slope without airspeed buildup. In hover: landing flare. |
| **6% – 16%** | **Active Brake** | Active upstroke negative AoA reverse thrust (`H-87`), turtle pivot brake (`H-103`), clamshell drag, rapid deceleration. |
| **0% – 6%** | **Max Airbrake / Stop** | Full cascade reversal ($180^\circ$), maximum aerodynamic braking flare (`H-55`, `H-108`, `LB3_EMERG`), motor idle/stop. |

---

## 4. Repository Structure

```
flight/
├── README.md                      # Mission Control & engineering overview
├── INDEX.md                       # Comprehensive 10-domain master file catalog
├── PROJECT_STATE.md               # Real-time build state, firmware revisions, open items
├── view_bom.sh                    # Live BOM pricing & MTOW auto-updater / inspector
├── run_sim.sh                     # Launch simulation mode in one command
├── start_gs.sh                    # Launch Ground Station & telemetry bridge
│
├── docs/                          # Engineering specifications & manuals
│   ├── mark1/                     # MCO-1D Mark 1 Authoritative Engineering Suite
│   │   ├── MCO-1D_Mark1_Master_Specification.md   # Flight physics, kinematics, mass budget
│   │   ├── MCO-1D_Mark1_Bill_of_Materials.md      # Auto-updating 39-part procurement BOM
│   │   ├── MCO-1D_Mark1_Manufacturing_Guide.md    # 216-rib print, CF machining, rigging
│   │   ├── MCO-1D_Mark1_Commissioning_Manual.md   # Assembly, CAN-FD setup, sensor zeroing
│   │   ├── MCO-1D_Mark1_Operator_Manual.md        # POH, lever map, Gemini copilot, failsafes
│   │   ├── MCO-1D_Mark1_Flight_Test_Protocol.md   # TC-01 to TC-04 flight clearance cards
│   │   ├── MCO-1D_Mark1_Software_Reference.md     # 50 Hz loop, Boyd Ps, CAN-FD dictionary
│   │   └── MCO-1D_Mark1_Adversarial_Audit_Report.md # Full-scope physics, kinematics & software audit
│   ├── latest/                    # Pre-consolidation snapshots (v18 BOM, v20 Wing, v25 Manuals)
│   ├── archive/                   # Historical design revisions (v1–v17 BOMs, v1–v24 manuals)
│   └── handover/                  # Complete Technical Handover Document v2
│
├── firmware/                      # Active flight software modules
│   ├── mco_main.py                # Flight computer executive loop (50 Hz)
│   ├── mco_wave_library.py        # Authoritative ~95k-line kinematics library (232 hypotheses)
│   ├── mco_flight_governor.py     # Deterministic safety governor & Software Recoil Engine
│   ├── mco_intent_control.py      # Unified lever & intent arbitration
│   ├── mco_odrive_interface.py    # ODrive S1 CAN motor interface
│   ├── mco_servo_interface.py     # pigpio 16-servo controller
│   ├── mco_mavlink_bridge.py      # Pixhawk telemetry & pitot bridge
│   ├── mco_cg_estimator.py        # Weight and balance estimator (FAA-H-8083-1B)
│   ├── mco_test_cards.py          # Automated TC-01 to TC-04 test engine
│   └── ...                        # Supporting adaptive & safety modules
│
├── cad/                           # Mechanical drawings & manufacturing specs
│   ├── drawings/                  # Rib profiles, wing construction, fuselage spec
│   └── manufactured/              # Generated printable STL and CNC cut DXF files
│
├── ui/                            # Ground station dashboards & visualizers
│   ├── mco_ai_ground_station.jsx  # Ground station interface & telemetry monitor
│   ├── performance_dashboard.jsx  # Real-time flight envelope analytics
│   └── wing_drone_visualizer.jsx  # 3D 8-wing kinematic renderer
│
└── scripts/                       # Deployment, bringup, and systemd services
    ├── mco_bom_tool.py            # Zero-dependency live BOM pricing engine
    ├── run_tc01_bench_checkout.py # Automated laboratory bench qualification runner (TC-01)
    ├── mco_install.sh             # Raspberry Pi automated installer
    ├── mco_can_setup.sh           # CAN-FD network bringup script
    ├── mco-flight.service         # Systemd flight computer autostart service
    └── mco-panel.service          # Systemd touchscreen panel service
```

---

## 5. Quick Start Guide

### 1. Run in Simulation Mode (No Hardware Required)
Test the 50 Hz control loop, wave library initialization, and intent arbitration on macOS or Linux:
```bash
./run_sim.sh
# or directly:
python3 firmware/mco_main.py --sim
```

### 2. Execute Full Verification Test Suite
Run the comprehensive 698-test verification suite spanning all 40 test suites and safety governors:
```bash
python3 -m unittest discover -s tests -p "test_*.py"
```

### 3. Run TC-01 Ground Bench Qualification
Execute the automated 10-step laboratory test harness for rigid post load cell qualification:
```bash
python3 scripts/run_tc01_bench_checkout.py
```

### 4. Launch the Ground Station
Start the WebSocket telemetry server, HTTP ground station bridge, and Gemini copilot proxy:
```bash
./start_gs.sh
```
Open your browser at `http://localhost:8080` to access the interactive Mission Control dashboard with Gemini 2.5 Flash copilot on port 5759.

### 5. Inspect & Auto-Update Bill of Materials (BOM)
Query verified vendors, check current market pricing, calculate total MTOW, and automatically refresh `docs/mark1/MCO-1D_Mark1_Bill_of_Materials.md`:
```bash
./view_bom.sh
# or query as JSON:
python3 scripts/mco_bom_tool.py --json
```

---

## 6. Applicable Aviation Standards
- **DO-178C**: Software Considerations in Airborne Systems and Equipment Certification
- **DO-160G**: Environmental Conditions and Test Procedures for Airborne Equipment
- **MIL-STD-1797B**: Flying Qualities of Piloted Aircraft
- **FAA-H-8083-1B / 25B**: Aircraft Weight and Balance / Pilot's Handbook of Aeronautical Knowledge
