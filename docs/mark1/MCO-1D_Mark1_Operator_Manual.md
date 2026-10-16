# MCO-1D Mark 1 — Operator & Pilot Flight Operations Manual

**Aircraft Title**: MCO-1D Mark 1 Metachronal Cascade Ornithopter  
**Document Designation**: MCO-1D-OPS-MK1  
**Classification**: Authoritative Operations Baseline — Revision Mark 1 (September 2026)  

---

## 1. Flight Control Architecture & Transmitter Mapping

The **MCO-1D Mark 1** employs an **Intent Arbitration Layer** that converts conventional pilot stick and lever inputs into complex 8-wing kinematic commands. The pilot does not manually fly individual wing surfaces; the flight computer executive (`mco_main.py`) continuously translates pilot intent into optimal metachronal phase, sweep, and pitch commands at 50 Hz.

```
Transmitter Mapping (Mode 2 Standard):
   LEFT STICK (Vertical)                   RIGHT STICK (Vertical)
   +-----------------------+              +-----------------------+
   |   UNIFIED LEVER       |              |   PITCH COMMAND       |
   |   100% (Sprint/Max Thrust)           |   (Elevator Feel)     |
   |   50% (Trim Detent)   |              |                       |
   |   0% (Max Airbrake)   |              |                       |
   +-----------------------+              +-----------------------+
   LEFT STICK (Horizontal)                 RIGHT STICK (Horizontal)
   +-----------------------+              +-----------------------+
   |   YAW / SIDESLIP      |              |   ROLL / BANK         |
   |   Differential Sweep  |              |   Differential Amp    |
   +-----------------------+              +-----------------------+
```

---

## 2. The Unified Control Lever (0% to 100%)

The primary propulsion control is the **Unified Lever**, assigned to the transmitter throttle axis (CH3). Following fixed-wing aviation standard: **100% (Full Forward) = Maximum Power / Sprint**, and **0% (Full Aft) = Maximum Airbrake / Emergency Stop**. It governs total specific excess power ($P_s$), airspeed, and descent rate along a continuous response curve:

| Lever Range | Mode Name | Wing Kinematics & Aerodynamic State | Typical Airspeed | Operational Use |
| :---: | :--- | :--- | :---: | :--- |
| **88% – 100%** | **SPRINT / MAX THRUST** | Wings swept forward ($30^\circ$), reduced stroke amplitude, peak flapping frequency ($3.5 - 4.0\text{ Hz}$). CF-1 clap burst below 10 m/s (`H-91`) governed by `run_cf1_clap_fling` envelope protection ($V_\infty \le 10.0\text{ m/s}$, duration $\le 1.5\text{ s}$). **Fix 4 Protected**: 20.0s max continuous burst ceiling, 45.0s cooldown lockout, 18.0A servo current trip derates to $f \le 2.2\text{ Hz}$. | $12.0 - 15.0\text{ m/s}$ | Dash, headwind penetration, max climb / vertical breakout. |
| **78% – 88%** | **HIGH THRUST / CLIMB** | High-thrust climb modes (`CLB2_ANGLE`, `H-92`, `H-71`), stroke $\pm 80^\circ$, frequency $2.5 - 3.0\text{ Hz}$. Governed by calibrated $P_s$ steady climb limits ($V_z \le 2.2\text{ m/s}$, climb angle $\gamma \le 15.5^\circ$, active motor power $P \le 280\text{ W}$). Transient surges $> 2.2\text{ m/s}$ tagged as kinetic zooms with energy depletion tracking. | $10.0 - 12.5\text{ m/s}$ | STOL takeoff climbout, steep obstacle clearance. |
| **68% – 78%** | **POWERED CRUISE** | Clean $90^\circ$ metachronal cascade (`H-86`, `H-104`), stroke $\pm 80^\circ$, frequency $2.0 - 2.5\text{ Hz}$. Fast transit. | $9.5 - 11.0\text{ m/s}$ | Fast transit, search sweeps. |
| **55% – 68%** | **CRUISE** | Standard cruise (`H-94`, `H-107`, `H-49`), frequency $1.87 - 2.0\text{ Hz}$, dihedral $+6^\circ$, cascade phase $28^\circ$. | $9.0 - 9.5\text{ m/s}$ | Efficient search patrol, loiter. |
| **45% – 55%** | **TRIM DETENT** | **Fixed-Wing Mode ($V \ge 5\text{ m/s}$)**: Balanced level cruise ($P_s = 0$, zero VSI).<br>**Helicopter Mode ($V \le 3\text{ m/s}$)**: Stationary hover altitude lock ($0\text{ fpm}$ vertical rate, $1.0\text{g}$ lift equilibrium). | $9.0\text{ m/s}$ (FW)<br>$0.0\text{ m/s}$ (Heli) | Hands-off cruise level flight, or rock-steady stationary hover hold. |
| **38% – 45%** | **APPROACH / LOW POWER** | Whisper mode (`AQ1_WHISPER`, `H-102`), reduced power, frequency throttled ($1.0 - 1.5\text{ Hz}$). In hover: gentle vertical descent. | $8.0 - 9.0\text{ m/s}$ (FW)<br>$-50\text{ to } -150\text{ fpm}$ (Heli) | Approach corridor, acoustic stealth, or hover descent. |
| **28% – 38%** | **GLIDE & SOAR** | Unpowered/slow-flap best glide (`H-102`, `H-90`, `H-95`, `H-LSB-1`), dihedral $+6^\circ$, minimal sink rate. Thermal soaring active. | $7.0 - 8.5\text{ m/s}$ | Power conservation, glide descent. |
| **16% – 28%** | **PIGEON DESCENT** | High-dihedral V-shape ($+40^\circ$ to $+60^\circ$), steep glide slope without airspeed buildup (constant $-3.0\text{ m/s}$ sink). In hover: landing flare. | $6.5 - 7.5\text{ m/s}$ | Steep approach over obstacles, ground approach. |
| **6% – 16%** | **ACTIVE BRAKE** | Negative AoA active upstroke reverse thrust (`H-87`), turtle pivot brake (`H-103`), clamshell drag. Rapid deceleration. | $5.0 - 6.5\text{ m/s}$ | Rapid speed reduction, entry to hover, ground rollout. |
| **0% – 6%** | **MAX AIRBRAKE / STOP** | Full cascade reversal ($180^\circ$), maximum aerodynamic braking flare (`H-55`, `H-108`, `LB3_EMERG`), motor idle/stop. | $< 5.0\text{ m/s}$ | Touchdown flare arrest, emergency stop, motor cut. |

---

### 2.1 Transition to Helicopter / Hover Mode ($V \le 3\text{ m/s}$)

The transition between fixed-wing airplane flight and helicopter hover is fully seamless and requires **zero switch-flipping or menu interaction**. It is governed strictly by airspeed crossfading:

1. **Entering Hover at Will**:
   * While cruising in fixed-wing flight, retard the Unified Lever into the **Active Brake detent ($6\%\text{--}16\%$)** or apply gentle aft stick to flare.
   * As forward airspeed drops through the $5.0\text{ m/s} \to 3.0\text{ m/s}$ window, the FCC automatically crossfades aerodynamic lift into Weis-Fogh flapping hover lift (`H-64` / `H-66`).
   * When nearly at rest ($V \le 3.0\text{ m/s}$), **click the Unified Lever into the center 50% Trim Detent**.
   * The aircraft immediately locks into a stationary hover, commanding $0\text{ fpm}$ vertical speed and holding current radar altitude ($1.0\text{g}$ equilibrium).
2. **Helicopter Flight Handling**:
   * **Unified Lever**: Acts as helicopter collective with a center-hold detent:
     * *Centered at 50%*: Holds current hover altitude rock-steady ($0\text{ fpm}$).
     * *Forward (> 55%)*: Proportional vertical ascent up to $+400\text{ fpm}$.
     * *Aft (< 45%)*: Proportional vertical descent down to $-400\text{ fpm}$.
     * *Below 16%*: Ground contact and settling on landing skids.
   * **Control Stick**: Controls horizontal translation (cyclic):
     * Centered: Attitude hold (stabilizes level in place).
     * Nudge Forward/Aft/Lateral: Gently drifts the aircraft in that direction without changing hover altitude.
   * **Rudder Pedals**: Controls pure heading rotation (yaw):
     * Deflecting pedals uses decoupled stroke-plane vectoring (`H-88A` / `H-88B`) or inertial tail-flick (`H-81`) to rotate the aircraft in place ($30^\circ/\text{s}$) with zero roll coupling or lateral drift.
3. **Resuming Fixed-Wing Flight**:
   * Push the control stick forward to tilt the thrust vector forward and advance the Unified Lever above $60\%$.
   * As the aircraft accelerates past $3.0\text{ m/s}$ up through $5.0\text{ m/s}$, the FCC automatically crossfades back into fixed-wing airplane handling.

---

### 2.2 Pilot Inceptor Mapping & Invisible Hypothesis Optimization
All 258 aerodynamic hypotheses operate **completely invisibly** beneath the controls:
* **The Pilot Flies an Airplane**: The pilot commands pitch, roll, yaw, and thrust using standard fixed-wing controls (plus helicopter hover when slow).
* **Autonomous Learning Classifier (ALC)**: The ALC and Context Champions (`_CMTS_CHAMPIONS`) continuously test and select the optimal hypothesis variant for the current flight regime, minimizing battery power and vibration in the background.
* **No Autonomous Autopilot Active**: Autopilot waypoint navigation and automated 3D trajectory steering are strictly deferred post-first-flight. The human pilot maintains 100% direct command authority at all times.

---

### 2.3 Sprint Detent Duty Cycle & Pilot Operating Instructions (Fix 4)
* **Burst Limit Enforcement**: Moving the Unified Lever into the **Sprint Detent (88%–100%)** initiates high-frequency ($3.5\text{--}4.0\,\text{Hz}$) low-stroke dash kinematics. The flight governor enforces a strict **20.0-second maximum sprint burst timer**.
* **Automatic Throttle Kickdown**: Upon expiration of the 20-second timer (or immediately if total servo bus current trips the $18.0\,\text{A}$ threshold), the governor automatically kicks down kinematics to safe derated cruise ($f \le 2.2\,\text{Hz}$, $\theta_{\text{stroke}} \le \pm 18.0^\circ$), regardless of lever position.
* **45-Second Cooldown Lockout**: The ground station HUD and copilot will annunciate `SPRINT COOLDOWN (45s)`. To reset the timer and re-arm sprint capability:
  1. The pilot must pull the Unified Lever aft below $88\%$ (recommended: return to $55\%\text{--}68\%$ Cruise).
  2. Maintain sub-sprint flight for $45.0\,\text{seconds}$ continuously.
  3. Attempting to hold the lever at $100\%$ will **not** restore sprint power; the governor maintains derated cruise limits until the full 45s cooldown elapses at sub-sprint throttle.
* **Copilot Audio Annunciation**: Gemini Copilot will call out *"Sprint burst active: 10 seconds remaining"*, *"Sprint timeout, cooling down"*, and *"Sprint re-armed"*.
* **Prohibition of Open-Loop Phase Lead (Fix 3)**: Servos operate under closed-loop position feedback only; no manual or automated open-loop phase overdrive is active, protecting the IB53BHU titanium-steel gear trains from whiplash shock.

---

## 3. Standard Operating Procedures (SOP)

### 3.1 Pre-Flight Inspection & Calibration Handoff
1. **Physical Calibration Sheet Verification (Step 0 - MANDATORY)**:
   * Verify that [`MCO-1D_Mark1_Builder_Calibration_Sheet.pdf`](file:///Users/drbell/Code/flight/docs/mark1/MCO-1D_Mark1_Builder_Calibration_Sheet.pdf) has all 8 fields (`FIG-01` through `FIG-08`) completed and signed.
   * Confirm the as-built flying mass (`FIG-01`, target $< 5,600\text{ g}$, e.g. $3,670\text{--}4,800\text{ g}$) is loaded into firmware.
   * Verify sector wheel travel checks out to full $200.0^\circ$ physical arc ($\pm 100.0^\circ$), with dynamic stroke ceiling clamped to $\pm 24.0^\circ$ sprint / $\pm 18.0^\circ\text{--}22.0^\circ$ cruise.
2. **Mechanical Walkaround**:
   * Inspect all 8 wings for covering tears or rib delamination.
   * Verify $1.5\,\text{g}$ brass pterostigma slugs are bonded securely at $85\%$ span in leading edge bays.
   * Verify trailing-edge lightening cutouts are clear with unpunctured lightweight film (5.4:1 Aft Subtraction balance).
   * Check Dyneema capstan cables: verify no fraying, tension within strictly **$150\text{--}170\text{ N}$** ($160\text{ N} \pm 10\text{ N}$, verified by acoustic tap pitch $\approx 210\text{ Hz}$ across 150 mm span).
   * Verify 3D-printed capstan line keeper shrouds (`STRUCT-17`) are securely snapped in place over all 8 motor pinions with $<0.4\text{ mm}$ clearance.
   * Verify zero-slop CNC ball links and titanium turnbuckles on all 16 servos have $<0.1\text{ mm}$ axial play.
   * Verify forward canopy NACA ram-air intake ($25\text{ mm} \times 15\text{ mm}$) and belly louvers are unobstructed.
3. **Longitudinal Balance Verification**:
   * Inspect the sliding battery tray on the fuselage keel rail.
   * Verify the 4.0 mm ball-lock pin is securely engaged in **Hole 3 ($250\text{ mm}$ aft of datum)** for nominal STOL cruise ($26.5\%$ MAC).
   * Confirm 6S main battery and dual 20A BEC power leads are secure.
4. **Avionics & Ground Station Bringup**:
   * Power transmitter; ensure Unified Lever is at **$0\%$ (Stick Down / Safe Stop)**.
   * Power dual-zone BEC rails: verify Zone A (Fore) and Zone B (Aft) both read $7.4\text{ V} \pm 0.1\text{ V}$ on telemetry.
   * Power 6S motor main bus.
   * On the ground station laptop, launch the telemetry bridge and Gemini copilot:
     ```bash
     ./start_gs.sh
     ```
   * Open the browser ground station (`http://localhost:5759`). Verify solid telemetry streaming at 50 Hz and all 8 ODrive nodes active.

### 3.2 Taxi & STOL Takeoff
1. **Ground Taxi**:
   * Advance Unified Lever forward into the $55\% - 65\%$ band until wings begin gentle forward taxi cycling.
   * Steer along the runway using the Yaw stick (differential wing sweep).
   * Align aircraft directly into the wind; pull lever back to $50\%$ to idle/stop.
2. **STOL Takeoff Run**:
   * Advance Unified Lever smoothly forward to **$85\% - 90\%$ (High Thrust / Takeoff)**.
   * As airspeed passes $4.0\text{ m/s}$ on the ground roll (after $\approx 8\text{ m}$), apply slight aft pitch stick ($+5^\circ$ AoA).
   * Aircraft will unstick smoothly at **$4.6\text{ m/s}$** ($10.3\text{ mph}$).
   * Establish initial climbout at **$9.0\text{ m/s}$**; return lever to $55\% - 65\%$ (Cruise) as altitude passes $15\text{ m}$ AGL.
3. **Alternative: Vault Takeoff (`H-98`)**:
   * For zero or ultrashort runway launch ($< 3.0\text{ m}$ ground roll), apply full aft stick and rapid lever advance to $95\%$.
   * Wings execute a synchronized ground-coupled downward compression stroke, popping the aircraft $1.5\text{ m}$ vertically into the air in $1.4\text{ s}$ at $8.5\text{ m/s}$.

### 3.3 Approach, Hummingbird Idle Glide & STOL Landing Flare
1. **Downwind / Base Leg**:
   * Set Unified Lever to $60\%$ ($V \approx 9.0\text{ m/s}$); fly standard rectangular pattern.
2. **Hummingbird Idle Glide (Final Approach)**:
   * Retard Unified Lever to **$28\% - 35\%$ (Glide / Approach)**. The FCC automatically engages **Hummingbird Idle** ($\pm 3.5^\circ$ dither at $1.1\text{ Hz}$, $92.0\text{ W}$ total power), protecting motor stators from stationary burnout while establishing a calm, stable $-0.85\text{ m/s}$ descent at $7.5\text{--}7.7\text{ m/s}$ airspeed ($12:1$ glide ratio).
   * If high on glidepath, retard lever to **$20\%$ (Pigeon Descent)** for a steep descent without airspeed gain.
3. **Hard WIG Floor Skimming**:
   * When flying low in ground effect ($h < 0.85\text{ m}$), the **Hard WIG Floor Governor** actively protects the airframe from runway impact, enforcing a virtual floor at **$0.35\text{ m}$**. Forward stick is progressively resisted, and full forward push is locked out.
4. **Touchdown Flare & Automatic WIG Floor Dissolution**:
   * At $1.5\text{ m}$ AGL (monitored via TF-Luna LiDAR), pull pitch stick firmly aft into a landing flare gesture (`Intent.FLARE`).
   * The WIG floor automatically dissolves to $0.00\text{ m}$ over **$0.25\text{ s}$**, permitting touchdown.
   * The wings flare into maximum camber and dihedral, arresting sink rate to $< 0.2\text{ m/s}$ as the wheels touch down.
   * For emergency STOL / Roost Perch landing: pull lever to $0\%$ and full aft stick. Caridoid reverse pulse arrests airspeed to $1.1\text{ m/s}$, stopping rollout in **$0.23\text{ m}$**.

### 3.4 Advanced Aerobatic & Combat Maneuvers
The Mark 1 fly-by-wire flight laws enable the pilot to fly standard aerobatics just like a high-performance fixed-wing aircraft:
* **Inside & Outside Loops**: Fly-by-wire `SymmetricLoadFactorGovernor` transparently clamps normal acceleration to $\pm 2.20g$. Outside push loops are executed with equal structural safety as inside pull loops thanks to the reinforced battery tray and bidirectional capstan drive.
* **Level Knife-Edge Flight**: Apply full rudder in knife-edge attitude. The 8 wings transition into an active vertical flapping rotor (`H-KNIFE-ROTOR-1`), producing $53.2\text{ N}$ of sustained lateral lift to maintain level flight without altitude loss.
* **Snap Rolls & Autorotations**: Full simultaneous deflection of stick and rudder commands an accelerated snap roll. The FCC decouples stall preemption, permitting dynamic wingtip stall while retaining yaw authority.
* **Hammerheads & Tailslides**: During vertical apex pullups, the tail-first apex handshake automatically zeroes servo hinge moments ($M_h < 0.05\text{ N}\cdot\text{m}$), protecting AGFRC servos from reverse airflow damage.
* **Spin Recovery**: If an inadvertent spin develops, neutralize stick and rudder. The FCC immediately injects an active anti-spin couple ($18.5\text{ N}\cdot\text{m}$), stopping rotation within $0.25\text{ s}$.

---

## 4. Ground Station Operations with Gemini 2.5 Flash Copilot

The Mark 1 integrates an autonomous ground station copilot powered by **Google Gemini 2.5 Flash** running locally on port 5759 (`mco_gemini_proxy.py`).

### Key Capabilities
* **Live Telemetry Monitoring**: Analyzes 154 real-time telemetry variables at 50 Hz, detecting sensor drift, temperature anomalies, or abnormal energy consumption before the pilot notices.
* **Natural Language Queries**: The pilot or flight engineer can ask questions directly via the browser ground station console:
  * *"What is our current specific excess power and battery margin?"*
  * *"Report wing symmetry and Dyneema rope health on Pair 3."*
  * *"Analyze our landing approach and give pitch stability score."*
* **Automatic Post-Flight Debrief**: Generates an instantaneous flight envelope summary and maintenance alert log upon touchdown.

### 4.2 Test Card Commands & Automated Telemetry Export
The Ground Station console directly controls automated test card execution and telemetry extraction:

* **Triggering Test Cards**:
  ```bash
  tc01_start       # Executes TC-01 ground bench frequency sweep (1.0 to 3.5 Hz)
  tc02_start       # Executes TC-02 tethered hover unstick discovery
  tc03_start       # Executes TC-03 advance-ratio truck sweep
  tc04_start       # Initiates TC-04 guided free flight STOL sequence
  ```
* **Status & Abort**:
  ```bash
  tc01_status      # Queries live step progress and pass/fail gates
  tc01_confirm     # Operator confirmation for semi-automatic physical steps
  tc01_abort       # Emergency test abort: idles all 8 motors immediately
  ```
* **Data Export & Worksheets**:
  * Run `tc_export` to flush all test card results to `/home/pi/knowledge/tc_results.json`.
  * From the ground station terminal, execute:
    ```bash
    python3 scripts/export_test_card_telemetry.py [--tc 1|2|3|4|all] [--json]
    ```
    This outputs the formatted table corresponding to printable worksheets [`docs/mark1/worksheets/`](file:///Users/drbell/Code/flight/docs/mark1/worksheets/), providing immediate verification without manual pen recording during test sequences.

---

## 5. Emergency Procedures & Autonomous Failsafes

### 5.1 Remote Control Signal Loss (RC Failsafe)
* **Trigger**: SBUS packet loss for $> 1.0\text{ s}$.
* **Action**:
  1. The executive immediately overrides pilot inputs and enters **Autonomous Return-to-Home (RTH)**.
  2. The flight controller commands $50\%$ Unified Lever ($9.0\text{ m/s}$ cruise), climbs or descends to $17\text{ m}$ AGL safety altitude, and navigates via Pixhawk GPS toward the launch coordinates.
  3. Upon reaching home coordinates, aircraft holds a $25\text{ m}$ radius loiter circle for 60 seconds before executing an automated LiDAR-guided STOL landing flare.

### 5.2 Pitot Tube Obstruction / Ice (`PITOT_STUCK`)
* **Trigger**: Airspeed unchanged for $> 50$ consecutive samples ($1.0\text{ s}$) while inertial accelerations are present.
* **Action**:
  1. The flight computer flags `PITOT_STUCK` in telemetry.
  2. Airspeed estimation switches seamlessly to the **Synthetic Airspeed Estimator** ($V_{\text{synth}} = |\vec{V}_{\text{GPS}} - \vec{W}_{\text{est}}|$ fused with IMU forward integration).
  3. Dynamic stall protection margins increase by $1.5\text{ m/s}$ to prevent stall in gusting conditions.

### 5.3 Motor Thermal Overload Protection
* **Trigger**: ODrive S1 FET temperature exceeds **$75^\circ\text{C}$**.
* **Action**:
  1. Executive initiates dynamic velocity limiting: `vel_limit` is scaled back from $150\text{ rad/s}$ to $110\text{ rad/s}$ over 5 seconds.
  2. Telemetry alerts ground station: `FET_OVERTEMP: Throttling flapping frequency to 2.2 Hz`.
  3. Pilot is directed to maintain $\ge 35\%$ lever position (cruise) to ensure cooling airflow over the motor bays.

### 5.4 Mountain Terrain & High-Altitude Ridge Turbulence (`MOUNTAIN_TERRAIN_TRIGGER`)
* **Trigger**: GPS/barometric altitude $> 1500\,\text{m}$, terrain slope $> 12^\circ$, or vertical gust rate $> 2.0\,\text{m/s}$ detected by `detect_mountain_terrain_trigger()`.
* **Action**:
  1. Telemetry annunciates `TERRAIN_ADVISORY: Mountain turbulence profile active`.
  2. The flight controller biases hypothesis selection toward mountain ridge soaring kinematics (`H-61`), increasing pitch lead compensation and dynamic dihedral stability margins.
  3. Copilot reports barometric density correction $\rho(h)$ and updated stall speed $V_s(h)$ to the operator display.

