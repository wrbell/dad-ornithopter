# MCO-1D Mark 1 — Flight Software & Kinematics Engine Reference

**Aircraft Title**: MCO-1D Mark 1 Metachronal Cascade Ornithopter  
**Document Designation**: MCO-1D-SW-MK1  
**Classification**: Authoritative Flight Software Baseline — Revision Mark 1 (September 2026)  

---

## 1. Executive Loop Architecture (`mco_main.py`)

The **MCO-1D Mark 1** flight computer executive operates deterministically at **50 Hz** ($20.0\text{ ms}$ tick period) on the Raspberry Pi 4B under Linux real-time scheduling.

```
20 ms Flight Control Loop Timeline:
[0ms] ──[ 1. Sensor Polling (3ms) ]──────────> IMU, Pitot, Lidar, SPI ADCs, Pixhawk
[3ms] ──[ 2. Intent Arbitration (2ms) ]──────> Unified Lever & Sticks -> Target Ps / VSI
[5ms] ──[ 3. Wave Kinematics Core (5ms) ]────> 8-Wing Phase, Sweep, Pitch Calculation
[10ms] ─[ 4. Actuator Bus Dispatch (4ms) ]───> CAN-FD to 8x S1s, pigpio 333Hz to 16 Servos
[14ms] ─[ 5. Telemetry & GS Dispatch (2ms) ]─> Line-delimited JSON on port 5760 / 5759
[16ms] ─[ 6. Timing Margin / Sleep (4ms) ]───> Jitter buffer guarantees strict 50.0 Hz
```

### 1.1 Failsafe State Machine
The core loop monitors system heartbeat and transitions states autonomously:
1. `INIT`: Hardware bringup, sensor zeroing, CAN bus discovery.
2. `STANDBY`: Ground idle; motors disarmed; servos held at rest sweep ($60^\circ$).
3. `TAXI`: Ground maneuvering; low-frequency cycling ($1.0\text{ Hz}$); differential sweep steering.
4. `FLIGHT`: Active airborne control; full metachronal wave kinematics and Boyd $P_s$ optimization.
5. `RTH`: Automated Return-to-Home triggered on RC loss or critical battery voltage.
6. `EMERGENCY`: Instantaneous aerodynamic brake flare ($0\%$ lever / full aft) followed by motor shutdown.

---

## 2. Metachronal Kinematics Engine (`mco_wave_library.py`)

The kinematics engine represents the single authoritative source of all flight waveforms, morphing envelopes, and evolutionary aerodynamic hypotheses (**258 total hypotheses** in `AERO_HYPOTHESES`, including **74 advanced biomimetic suites** in `_ADV_NEW_HYPS` spanning H-37 through H-108, H-LSB-1, H-DSH-1, H-AC-SHIFT-1, H-SHED-1, H-VEX-1, H-88A, and H-88B).

### 2.1 Boyd Specific Excess Power ($P_s$) PI Controller
Vertical climb and acceleration are governed by Colonel John Boyd's energy-maneuverability formulation:

$$P_s = \frac{V_\infty}{W} (T - D) = \frac{dh}{dt} + \frac{V_\infty}{g}\frac{dV_\infty}{dt}$$

The controller dynamically adjusts flapping frequency $f$ and stroke amplitude $\theta_0$ to track target $P_s$:

$$f(t) = f_{\text{base}} + K_p \cdot e_{P_s}(t) + K_i \int e_{P_s}(\tau) d\tau$$

With anti-windup clamping to enforce the strict physical boundaries of the drivetrain:

$$1.0\text{ Hz} \le f \le 3.5\text{ Hz}, \quad \pm 45.0^\circ \le \theta_0 \le \pm 80.0^\circ$$

### 2.2 Dynamic Sweep & Stroke Phase Morphing
During high-speed forward flight ($V_\infty \ge 10.0\text{ m/s}$), wing sweep is dynamically modulated within each individual flap cycle:
* **Downstroke ($\phi \in [0, \pi]$)**: Wings sweep forward to $30^\circ$ to tilt the stroke plane forward and generate maximum propulsive thrust.
* **Upstroke ($\phi \in [\pi, 2\pi]$)**: Wings sweep aft toward $85^\circ$ to streamline the span, minimizing parasite and profile drag during wing recovery.

### 2.3 Flight Safety Governors & Dynamic Airframe Protection (`mco_flight_governor.py`)
Hardware safety boundaries, ground protection, and turnaround dynamics are enforced deterministically:
1. **Hard Wing-In-Ground (WIG) Safety Floor Governor (`HardWIGFloorGovernor`)**:
   * Enforces an impassable **$0.35\text{ m}$ virtual floor** above runway level to prevent Controlled Flight Into Terrain (CFIT) while skimming.
   * Progressively resists forward stick pushing in the buffer zone ($0.35\text{--}0.65\text{ m}$) and locks out forward stick completely at $\le 0.35\text{ m}$ while pulsing front canard lift ($+35\text{ N}$) and pitch up ($+6.0^\circ$) on sink.
   * Automatically dissolves the virtual floor from $0.35\text{ m}$ to $0.00\text{ m}$ over **$0.25\text{ s}$** upon pilot initiation of a landing flare (`Intent.FLARE`, `ROOST_SOMERSAULT`, or deep aft stick).
2. **Symmetric Structural Load Factor Governor (`SymmetricLoadFactorGovernor`)**:
   * Strictly clamps normal load factor to **$N_z \in [-2.20g, +2.20g]$** across all maneuvers.
   * With the bidirectional capstan drive and reinforced battery tray, limits are completely symmetric, protecting carbon spars from overload during inside and outside push loops.
3. **Dynamic As-Built Mass Scaling (`scale_flight_laws_to_mass`)**:
   * Accepts live as-built gross weight from [`MCO-1D_Mark1_Builder_Calibration_Sheet.pdf`](MCO-1D_Mark1_Builder_Calibration_Sheet.pdf) (`FIG-01`).
   * Dynamically rescales stall speed floor $V_s \propto \sqrt{m}$, required lift $L = mg$, Boyd specific excess power $P_s$, and hover thrust-to-weight ratio.
4. **Software-Only Reversal Recoil Engine (`compute_soft_recoil_trajectory`)**: 
   * Trajectory shaping with a tuned 3rd-harmonic counter-acceleration component:
     $$\theta(t) = \theta_0 (1.02 \sin \omega t + 0.02 \sin 3\omega t)$$
   * Flattens turnaround acceleration spikes by up to $42\%$, eliminating motor torque saturation at stroke reversal.
5. **Aerodynamic Air-Spring Pitch Advance (`compute_aero_spring_pitch_advance`)**: 
   * Rotates wing chord into oncoming airflow $15\text{--}25\text{ ms}$ prior to stroke reversal ($+8^\circ$ at top turnaround, $-8^\circ$ at bottom turnaround), using dynamic pressure as an aerodynamic deceleration cushion.
6. **Hard Boundary Guards**:
   * Glide speed floor ($11.0\text{ m/s}$) with automated power injection below $10.5\text{ m/s}$.
   * Leaky-bucket motor stator thermal governor ($25.0\text{ s}$ hover budget).
   * Sprint thermal clamp: 20s burst ceiling, 45s cooldown lockout, 18.0A servo current trip.
   * Multi-wing blade strike collision avoidance ($94\text{ mm}$ root clearance for H-91 clap-fling).
   * 200° sector wheel travel with $\pm 96.0^\circ$ dynamic reciprocating stroke ceiling (reserving $4^\circ$ overshoot cushion).

### 2.4 Dual H-88 Stroke-Plane Vectoring Architecture
* **`H-88A` (Dynamic Harmonic Vectoring)**: Coordinates in-stroke flapping and sinusoidal sweep oscillation to tilt the physical stroke plane by $\beta = \arctan(\Theta/\Phi) = \pm 15^\circ\text{ to }\pm 18^\circ$, delivering rapid pure yaw couples ($>5.2\text{ N}\cdot\text{m}$, $>60^\circ/\text{s}$) with zero roll asymmetry.
* **`H-88B` (Quasi-Steady Differential Sweep Bias Trim)**: Applies static differential sweep angles ($\pm 12^\circ$) between port and starboard wings to tilt the aerodynamic drag vector, creating continuous yaw trim ($\sim 3.4\text{ N}\cdot\text{m}$) for crosswind decrab with zero cyclic servo fatigue.
* All hypotheses map strictly to pilot inceptors (Wave Lever, Stick, Pedals) and operate invisibly; autonomous autopilot navigation and position-hold modes are deferred post-first-flight.

### 2.5 Physical Calibration Models & Backlog Remediation (`Path 2`)
Under the Path 2 engineering baseline, physical grounding functions reconcile aeromodeling with test stand dynamics:
1. **Flapping Motor Power Factor Reconciliation (`PF_FLAPPING = 0.127`)**:
   * Apparent electrical power ($I_{\text{RMS}} \cdot V_{\text{bus}} \approx 1600\,\text{W}$) is reconciled with actual battery active power draw ($P_{\text{active}} \le 280\,\text{W}$) via back-EMF mechanical regeneration into the DC bus / dynamic brake resistors during stroke turnaround.
   * Governs $P_s$ steady climb rate ($V_z \le 2.2\,\text{m/s}$) and climb angle ($\gamma \le 15.5^\circ$), tagging transient surges as kinetic zooms with energy depletion tracking.
2. **Barometric Atmospheric Density Model (`atmospheric_density(h_m)`)**:
   * Models standard barometric lapse rate $\rho(h) = \rho_0 (1 - L \cdot h / T_0)^{\frac{g M}{R_0 L}}$ with temperature offset support ($T_{\text{offset\_k}}$).
   * Dynamically couples air density $\rho(h)$ into dynamic pressure $q_\infty = \frac{1}{2} \rho V_\infty^2$ and true stall speed $V_s(h) = V_{s,0} \sqrt{\rho_0 / \rho(h)}$.
3. **Loaded Servo Slew Derating (`compute_loaded_servo_slew_ceiling`) & Dynamic Pitch Lead (`calculate_dynamic_pitch_servo_lead`)**:
   * Derates effective servo slew rate under aerodynamic hinge moments: $\Omega_{\text{eff}} = \Omega_0 \cdot \max(0.35, 1.0 - M_h / M_{\text{stall}})$.
   * Computes dynamic pitch lead phase angle $\Delta \phi_{\text{lead}} = K_{\text{lead}} \cdot (2\pi f) \cdot \tau_{\text{servo}} \cdot (1 + M_h / M_{\text{stall}})$ to prevent phase lag breakdown at flapping frequencies up to $3.5\,\text{Hz}$.
4. **CF-1 Clap-Fling Dispatcher (`run_cf1_clap_fling`) [O-C1 Resolved]**:
   * Authoritative dispatcher for Weis-Fogh clap-and-fling aerodynamic bursts (`H-91`), enforcing maximum duration ($1.5\,\text{s}$), forward airspeed ceiling ($V_\infty \le 10.0\,\text{m/s}$), and recovery phase alignment.
5. **GPS Mountain Terrain Trigger (`detect_mountain_terrain_trigger`) [O-D7 Resolved]**:
   * Evaluates barometric/GPS altitude, vertical climb rate, terrain slope, and wind gust intensity to trigger high-altitude turbulent mountain flight profiles (`H-61`) with status reporting.
6. **Tie-Break Exploration Balancing**:
   * Hypothesis arbitration in `_select_winning_candidate` incorporates `eval_counts` usage tracking, selecting the least-evaluated candidate when performance scores tie within tolerance, ensuring balanced exploration across all 258 hypotheses.

### 2.6 Fanned Bi-Delta Tailplane Fast Cruise & Level Dash (`H-FALCON-TRIM-1`)
Under `H-FALCON-TRIM-1` ("Falcon Dart"), high-speed cruise ($V = 20.80\,\text{m/s}$, $46.5\,\text{mph}$, $365\,\text{W}$) and level dash ($V = 26.35\,\text{m/s}$, $58.9\,\text{mph}$, $580\,\text{W}$) are achieved on the Selig S1223 tandem airframe while eliminating uncompensated longitudinal body porpoising and cyclic dynamic stall:
1. **Kinematic Architecture**:
   * **Active Propulsive Cascade (Stations 1, 2, 3)**: Flap symmetrically at $f = 2.4 - 3.0\,\text{Hz}$ ($\pm 18.0^\circ$ stroke amplitude) in a fanned stagger sweep schedule ($+5.0^\circ, +15.0^\circ, +28.0^\circ$) with progressive washout trim ($+2.5^\circ, +1.5^\circ, +1.0^\circ$).
   * **Stationary FBW Tailplane (Station 4)**: Locked stationary at flapping stroke $\theta = 0.0^\circ$, swept aft to $+50.0^\circ$, and pitched to $\alpha = -5.5^\circ$.
2. **Longitudinal Pitch Equilibrium**:
   * Active flapping wings W1–W3 generate net nose-down pitching moment $M_{\text{pitch}} \approx -21.4\,\text{N}\cdot\text{m}$.
   * Stationary W4 at $\alpha = -5.5^\circ$ generates a calm $+21.4\,\text{N}\cdot\text{m}$ nose-up trim moment, zeroing residual porpoising ($<0.1\,\text{N}\cdot\text{m}$ vs baseline $\pm 24.4^\circ$).
3. **Actuator & Aeroelastic Margins**:
   * Stationary lock prevents cyclic upstroke dynamic stall ($\alpha = -13.2^\circ$).
   * Pitch servo hinge moment on Station 4 is bounded to $M_h \le 0.45\,\text{N}\cdot\text{m}$, preserving a $>34.8\%$ torque margin below the AGFRC $0.69\,\text{N}\cdot\text{m}$ stall torque limit.
   * Inter-wing dynamic 3D clearance exceeds $43.4\,\text{mm}$ across all stations ($57.8\,\text{mm}$ W1–W2, $95.8\,\text{mm}$ W3–W4).
4. **Safety Envelopes & Aborts**:
   * Instantaneous stick abort: $|\text{pitch}| > 0.35$, $|\text{roll}| > 0.35$, $|\text{yaw}| > 0.15$.
   * Low-airspeed stall abort ($V < 12.0\,\text{m/s}$) and ground lockout (`on_ground = True`).
   * High-speed pitot saturation guard ($V \ge 39.4\,\text{m/s}$ / $950\,\text{Pa}$) with automatic Tactical EKF / GPS airspeed fusion.
   * Dispatched automatically via `Intent.FALCON_DART` (`mco_intent_control.py`) and inspected via ground station command `falcon_dart_status` (`firmware/mco_wave_library.py`).

### 2.7 Locked Orthogonal Carrier Wave & Intra-Cascade Maneuvering
The flight kinematics core eliminates inter-station phase drifting, beat vibrations, and clearance clashing by locking all 4 stations to a permanent orthogonal carrier clock:

$$\phi_n(t) = 2\pi f_0 t - (n - 1) \cdot 90^\circ, \quad n \in \{1, 2, 3, 4\}$$

1. **Square-Root Conserving Amplitude Mapping (Zero-Heave Pitch/Roll Engine)**:
   * **Physics Rationale**: Flapping lift scales with the square of stroke amplitude ($\overline{L} \propto A^2$). Linear amplitude modulation ($A_1 = A_0(1+\delta)$, $A_4 = A_0(1-\delta)$) creates a $+6.25\text{--}12.5\%$ lift ballooning and breaks heave cancellation ($0.35 A_0$ residual heave couple).
   * **Governing Control Law**: The kinematics core commands fore-aft and lateral moments via square-root mapped amplitude modulation:
     $$A_1 = A_0 \sqrt{1 + 2\delta_{\text{pitch}}}, \quad A_4 = A_0 \sqrt{1 - 2\delta_{\text{pitch}}}$$
     $$A_{\text{port}} = A_0 \sqrt{1 + 2\delta_{\text{roll}}}, \quad A_{\text{stbd}} = A_0 \sqrt{1 - 2\delta_{\text{roll}}}$$
   * **Invariance**: This guarantees exact constant mean lift ($A_1^2 + A_4^2 = 2 A_0^2$), keeping cycle-averaged vertical acceleration below $0.04g$ during aggressive pitch and roll maneuvers.
2. **Hummingbird Idle Dynamic Glide Governor**:
   * **Stator Protection**: Holding wings stationary against $4.5\text{ N}$ aerodynamic lift draws $8.6\text{ A}$ DC into static motor coils ($1,530\text{ W}$ across 8 motors), causing immediate stator insulation failure within $12\text{ s}$.
   * **Dynamic Dither Implementation**: When glide or zero-thrust is commanded, the governor engages an ultra-low-amplitude micro-dither:
     $$\theta_n(t) = \theta_{\text{glide}} + 3.5^\circ \sin(2\pi \cdot 1.1 t - (n-1)\cdot 90^\circ)$$
   * Rotates motor electrical fields continuously ($I_q \approx 6.1\text{ A RMS}$), dissipating only $35.7\text{ W}$ motor copper losses and maintaining a total electrical budget of **$92.0\text{ W}$** while providing $12:1$ glide ratio and $0.85\text{ m/s}$ sink rate.
3. **VMS Quadrature Amplitude Cross-Fade Engine**:
   * Shifts between hovering, cruise flapping, and glide regimes using smooth amplitude homotopy through zero ($A(t) \to 0 \to A_{\text{target}}$) rather than phase jumps.
   * Couples a feedforward pitch reflex of $-4.0^\circ$ onto Station 1 during amplitude reductions to counteract the natural nose-up pitch surge ($+7.2\text{ rad/s}^2$).
4. **Dual-Zone Servo Bus Inrush Governor**:
   * Coordinates the AGFRC IB53BHU servos across Zone A (Fore, Stations 1 & 2) and Zone B (Aft, Stations 3 & 4) by introducing an intentional $4.0\text{ ms}$ command dispatch stagger between zones.
   * Caps aggregate inrush current below $18.0\text{ A}$ per BEC zone, eliminating multi-servo step transient droop.

---

## 3. CAN-FD Telemetry Frame Specification

Communication between the Raspberry Pi FCC (via Waveshare MCP2518FD Hat) and the 8× ODrive S1 controllers uses CAN-FD protocol at **1 Mbps nominal / 8 Mbps payload**:

### 3.1 Outbound Setpoint Frame (Pi $\to$ ODrive S1)
* **CAN Arbitration ID**: `(node_id << 5) | 0x00C` (Command: `Set_Input_Pos`)
* **Payload (8 Bytes)**:
  * Byte 0–3: `float32` Target Position (electrical revolutions).
  * Byte 4–5: `int16` Feedforward Velocity ($0.1\text{ rev/s}$ LSB).
  * Byte 6–7: `int16` Feedforward Torque ($0.01\text{ N}\cdot\text{m}$ LSB).

### 3.2 Inbound Feedback Frame (ODrive S1 $\to$ Pi)
* **CAN Arbitration ID**: `(node_id << 5) | 0x009` (Response: `Get_Encoder_Estimates`)
* **Payload (8 Bytes)**:
  * Byte 0–3: `float32` Actual Motor Position (revolutions).
  * Byte 4–7: `float32` Actual Motor Velocity (rev/s).

---

## 4. Ground Station Command Catalogue

The ground station interface engine (`_execute_gs_base_cmd`) supports **368 distinct remote commands** registered in `_GS_ALLOWED_CMDS`. All commands are accepted over line-delimited TCP JSON (port 5760 or through the Gemini proxy on port 5759):

```json
// Command Request Schema
{"cmd": "status"}

// Command Response Schema
{"type": "ack", "cmd": "status", "result": "ok"}
```

### Primary Command Subsystems

| Command Group | Representative Commands | Purpose |
| :--- | :--- | :--- |
| **System & Status** | `status`, `help`, `hyp_status`, `advanced_physics_status`, `gs_status`, `airborne_status` | Returns overall system state, active focus, calibration flags, and telemetry for all 74 advanced hypotheses. |
| **Aero Calibration** | `aero_cal_status`, `aero_cal_reset`, `margin_status` | Inspects $C_{L,\max}$, glide polar confidence, and stall margins. |
| **Test Cards** | `tc01_start`, `tc01_status`, `tc02_start`, `tc03_start`, `tc02_abort` | Triggers and monitors automated test cards TC-01 through TC-04. |
| **Sensing & Hardware**| `stroke_position_status`, `fault_summary`, `fault_status` | Displays ODrive sector velocities, ADC count mappings, and faults. |
| **Loop 9 & Evolution** | `l9_status`, `l9_freeze`, `l9_unfreeze`, `l9_reset` | Manages autonomous mode creation and lever map evolution. |
| **Optimization** | `freeze_optimizers`, `restore_optimizers`, `polar_survey_start` | Freezes or restores online parameter adaptation algorithms. |
| **Parameter Tuning** | `set`, `param_set`, `confirm_remap` | Updates whitelisted runtime parameters (e.g. `CASCADE_PHASE_DEFAULT_DEG`). |

---

## 5. Automated Verification & Simulation Mode

The complete software suite can be validated in simulated hardware mode without physical bench connections:

```bash
# Execute standalone 50 Hz simulation run
./run_sim.sh
```

### Full Repository Regression Test Suite
All flight software modules, kinematic allocators, governor guards, and test suites are verified via automated unittest discovery:
```bash
python3 -m unittest discover tests -p "test_*.py"
```
Pass criteria: **698 / 698 tests pass across 40 test suites (100% green, 0 failures, 0 errors)** covering the full envelope from zero-speed hover to high-speed dash, STOL takeoff, conventional flare/rollout, biological go-around aborts, physical atmospheric coupling, loaded servo derating, and Boyd $P_s$ power factor calibration.
