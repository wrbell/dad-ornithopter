# MCO-1D Mark 1 — Physical Build & Flight Performance Readiness Audit

**Aircraft Title**: MCO-1D Mark 1 Metachronal Cascade Ornithopter  
**Demonstrator Program**: MCO-1A Search-and-Rescue (SAR) Subscale Flight Demonstrator  
**Document Designation**: MCO-1D-AUD-PHYS-MK1  
**Classification**: Engineering Audit & Flight Readiness Certification — Revision Mark 1 (September 2026)  
**Target Hardware Baseline**: 8-Wing Cascade, 10:1 Capstan Drivetrain, 5.687 kg MTOW  

---

## 1. Executive Summary & Flight-Readiness Risk Matrix

As the physical mechanism of the **MCO-1D Mark 1** reaches mechanical completion and bench assembly, this audit assesses all physical, mechanical, thermal, aerodynamic, and electrical factors that could directly degrade or impair flight performance.

Each identified risk is categorized by severity, flight impact, and shop-floor remediation required prior to first motor spin and flight testing.

```
Severity Definitions:
  [CRITICAL]  Direct airframe loss, structural failure, or in-flight loss of control. Must be resolved before power-up.
  [HIGH]      Severe performance degradation, motor thermal throttling, or control instability.
  [MODERATE]  Trim bias, sensor noise, or operational envelope restriction.
```

| ID | Engineering Domain | Primary Risk / Impairment Mechanism | Severity | Flight Performance Impact | Required Action / Remediation |
| :---: | :--- | :--- | :---: | :--- | :--- |
| **PHY-01** | **Drivetrain / Rigging** | Dyneema SK99 primary creep & tension relaxation | **HIGH** | Wing zero-point drifts $2^\circ\text{--}5^\circ$, causing asymmetric lift and roll bias. | 48-hour pre-stretch at $200\,\text{N}$, re-tensioning to $180\,\text{N}$, Loctite 480 witness-marking. |
| **PHY-02** | **Structure / Spar** | Carbon root tube crushing under sector clamp load | **CRITICAL** | Micro-delamination and catastrophic root spar shear failure under $2.2g$ flapping. | Insert $\varnothing 8.0\,\text{mm} \times 40\,\text{mm}$ 6061-T6 aluminum internal crush ferrules at root. |
| **PHY-03** | **Aerodynamics / Trim** | Passive glide lift deficit ($18.6\,\text{N}$ deficit at $9\,\text{m/s}$) | **HIGH** | Aircraft cannot maintain unpowered level flight at nominal cruise speed ($9.0\,\text{m/s}$). | Enforce active flapping $f \ge 1.87\,\text{Hz}$ in cruise; clamp dead-stick glide speed to $V_{\text{glide}} \ge 11.0\,\text{m/s}$. |
| **PHY-04** | **Thermal / Materials** | LW-PLA motor pod softening ($T_g = 55^\circ\text{C}$) | **CRITICAL** | Motor mounts soften under $35\,\text{W}$ heat, causing shaft tilt and capstan derailment. | Install $0.5\,\text{mm}$ thermal isolator gaskets and aluminum standoffs; confirm ram-air NACA scoops. |
| **PHY-05** | **Kinematics / CAD** | Vertical staircase & sweep clash boundary | **HIGH** | Wingtip clashing during high-rate differential sweep ($>15^\circ$ split) during opposing strokes. | Software sweep-differential limiter ($\le 15^\circ$) and enforce $41.25\,\text{mm}$ staircase check in governor. |
| **PHY-06** | **Avionics / Sensing** | MS4525DO Pitot resolution at low airspeeds ($6.2\,\text{m/s}$) | **MODERATE** | Sensor noise ($\pm 4\,\text{counts}$) induces $\pm 0.35\,\text{m/s}$ speed jitter, risking false failovers. | Enable rolling median filter ($N=5$) and synthetic airspeed blending below $7.5\,\text{m/s}$. |
| **PHY-07** | **Actuation / Servos** | Dynamic pitch servo torque derating under aerodynamic loads | **HIGH** | Pitch phase lag during rapid pre-rotation (H-89), reducing metachronal thrust by up to $18\%$. | Optimize pitch tie-rod mechanical advantage to $1.2:1$; verify 7.4V rail maintains $>7.1\text{V}$ under load. |
| **PHY-08** | **Electrical / Power** | Inductive kickback on 7.4V servo bus | **HIGH** | Digital brownout resets on MCP3208 ADCs during 16-servo simultaneous reversals. | Verify $4,700\,\mu\text{F}$ ultra-low ESR electrolytic capacitor is soldered directly across bus terminals. |

---

## 2. Mechanical & Drivetrain Performance

### 2.1 Dyneema SK99 Cable Creep & Asymmetric Wing Zero-Drift (PHY-01)
* **Physical Mechanism**: The 10:1 capstan drivetrain utilizes $0.8\,\text{mm}$ 12-strand Dyneema SK99 rope wrapped 3 full turns on a helical aluminum pinion drum ($d_{\text{pitch}} = 15.4\,\text{mm}$ or $18.0\,\text{mm}$) and anchored to a $2.0\,\text{mm}$ carbon fiber sector wheel. While SK99 has ultra-high tensile strength ($>1100\,\text{N}$ breaking load), all Ultra-High-Molecular-Weight Polyethylene (UHMWPE) exhibits **viscoelastic primary creep** during the initial 24–48 hours under sustained tension ($180\,\text{N}$).
* **Flight Impairment**: A relaxation of only $0.6\,\text{mm}$ of cable travel translates across the $77\,\text{mm}$ sector radius to an angular zero-point error of:
  $$\Delta \theta = \frac{0.6\,\text{mm}}{77.0\,\text{mm}} \times \frac{180^\circ}{\pi} \approx 0.45^\circ \text{ on the sector, and up to } 2.5^\circ\text{--}4.5^\circ \text{ at the wing chord line!}$$
  If wings creep asymmetrically, the aircraft experiences persistent trim rolling moments, unequal lift distribution, and high bias current on the ODrive position controllers.
* **Mandatory Shop-Floor Fix**:
  1. **Pre-Stretching Procedure**: Rig all 8 cables to $200\,\text{N}$ using an inline tension scale. Allow to settle under static tension for **48 hours** before final calibration.
  2. **Secondary Re-Tensioning**: Re-tension cables to the nominal **$180\,\text{N} \pm 5\,\text{N}$**.
  3. **Witness Marks**: Apply high-contrast paint witness marks across the Dyneema clamp screw and carbon sector wheel.
  4. **Adhesive Lock**: Apply a drop of Loctite 480 flexible cyanoacrylate to the clamp anchor groove to eliminate micro-slippage.

```
Dyneema Capstan Drive Tensioning Schematic:
     +-------------------------------------------------+
     |           10:1 CARBON SECTOR WHEEL             |
     |         (R = 77.0 mm, 100° Stroke Arc)          |
     +---------o-----------------------------o---------+
              /                               \
     Down    /                                 \   Up
     Run    /   180 N Continuous Pre-Tension    \  Run
           /                                     \
          v                                       v
        [========= HELICAL CAPSTAN DRUM =========]
        [  15.4 mm Pitch Dia, 3 Full Turns Wrap  ]
        [      T-Motor MN5008 5mm D-Shaft        ]
```

### 2.2 Root Carbon Spar Clamping Crush & Ferrule Reinforcement (PHY-02)
* **Physical Mechanism**: The main wing spar is a $10.0\,\text{mm OD} \times 8.0\,\text{mm ID}$ pultruded unidirectional carbon fiber tube. At the root, this tube is clamped into the aluminum sector yoke using split-collar clamp bolts torqued to $2.2\,\text{N}\cdot\text{m}$.
* **Structural Failure Risk**: Pultruded carbon tubes have exceptional axial tensile strength ($>1200\,\text{MPa}$), but very low transverse hoop strength (typically $<60\,\text{MPa}$). Clamping pressure without internal support causes micro-cracking and crushing of the composite wall. Under $2.2g$ flapping limit load, the root bending moment reaches $M_b = 5.36\,\text{N}\cdot\text{m}$, creating severe localized stress concentrations where the tube exits the yoke collar.
* **Mandatory Shop-Floor Fix**:
  - Machined **6061-T6 aluminum internal ferrules** ($\varnothing 8.0\,\text{mm} \times 40.0\,\text{mm}$ with a $0.5\,\text{mm}$ retention shoulder) must be coated in 30-minute structural epoxy and pressed into the root of each 10 mm carbon spar.
  - This ferrule supports $100\%$ of the radial clamping force, preventing composite wall collapse while transferring bending loads uniformly into the sector yoke.

---

## 3. Kinematic Alignment & Collision Envelopes

### 3.1 Vertical Staircase ($41.25\,\text{mm}$ vs $100.0\,\text{mm}$) Geometry Alignment (PHY-05)
* **Specification Reconciliation**:
  - The Mark 1 Master Specification (`MCO-1D-SPEC-MK1` Section 2.2) originally defined a theoretical staircase of $+150, +50, -50, -150\,\text{mm}$ ($100.0\,\text{mm}$ steps).
  - The actual physical CAD fuselage assembly (`cad/manufactured/mco_fuselage_assembly.step`) and governor firmware (`firmware/mco_flight_governor.py` line 213) implement the **$41.25\,\text{mm}$ drop per station** ($123.75\,\text{mm}$ total cascade drop), matching the $9.37^\circ$ spine incline.
* **Physical Clearance Analysis**:
  - Longitudinal Station Stagger: $s = 250.0\,\text{mm}$.
  - Mean Aerodynamic Chord: $\bar{c} = 120.0\text{--}143.7\,\text{mm}$.
  - Diagonal Root Clearance: $\sqrt{250^2 + 41.25^2} - 143.7 \approx 109.7\,\text{mm}$ (safe static separation).
  - **Dynamic Wingtip Clash Risk**: At high dihedral angles ($>40^\circ$) and large sweep differentials, the wingtips swing toward adjacent stations. If Pair $N$ sweeps aft to $65^\circ$ while Pair $N+1$ sweeps forward to $30^\circ$, the dynamic clearance bubble drops below the $35.0\,\text{mm}$ aeroelastic buffer.
* **Mandatory Protection**:
  - Ensure `MIN_AEROELASTIC_CLEARANCE_MM = 35.0` in `firmware/mco_flight_governor.py` is actively checking `vert_staircase_mm = 41.25`.
  - In firmware, enforce a hard clamp on inter-pair sweep differential:
    $$\Delta \Lambda = |\Lambda_n - \Lambda_{n+1}| \le 15.0^\circ \quad \text{when } |\theta_n - \theta_{n+1}| > 40.0^\circ$$

---

## 4. Thermal & Materials Safety Under Flight Loads

### 4.1 LW-PLA Motor Pod Glass Transition Softening (PHY-04)
* **Physical Mechanism**: The 8 motor nacelles are 3D-printed in active-foaming ColorFabb LW-PLA to achieve minimal mass ($1.5\,\text{g}$ per rib, lightweight pods).
* **Thermal Failure Boundary**: Standard PLA and foaming LW-PLA have a low glass transition temperature:
  $$T_g \approx 55.0^\circ\text{C} \quad (131.0^\circ\text{F})$$
  During high-load climbs or hover operations, each T-Motor MN5008 generates up to **$35.0\,\text{W}$ of stator resistive and eddy current heat**. Motor case temperatures routinely reach $60^\circ\text{C}\text{--}70^\circ\text{C}$.
* **Flight Impairment**: Direct contact between the hot motor baseplate and the LW-PLA nacelle will cause the plastic to exceed $T_g$. Under the $180\,\text{N}$ tension of the Dyneema capstan cable, the motor mount will soften, yielding plastically. This tilts the motor shaft, causing capstan cable binding, severe friction, and catastrophic cable derailment.
* **Mandatory Shop-Floor Fix**:
  1. **Thermal Isolator Gaskets**: Install a $0.5\,\text{mm}$ silicone/fiberglass or FR4 composite thermal isolation gasket between each motor base and the airframe bracket.
  2. **Mounting Plate Material**: The motor faceplate itself must be CNC 6061-T6 aluminum ($2.0\,\text{mm}$ thickness), never raw LW-PLA.
  3. **Ram-Air Scoops**: Verify that all 8 3D-printed ram-air NACA scoops (`MFG-MK1` Section 5.3) are unblocked, with clear line-of-sight to ambient freestream air.

```
Motor Thermal Isolation Stackup:
    [ T-Motor MN5008 Baseplate (65°C) ]
                   ||
    [ 0.5 mm Silicone/FR4 Thermal Isolator Gasket ]  <-- Thermal Barrier (ΔT ~ 20°C)
                   ||
    [ 2.0 mm 6061-T6 Aluminum Station Bracket ]     <-- Structural Load Path
                   ||
    [ LW-PLA Fuselage Skin / Pod (< 42°C) ]          <-- Stays safely below Tg (55°C)
```

### 4.2 ODrive S1 Inverter Power FET Dissipation
* **Thermal Load**: Each ODrive S1 switches up to $25\,\text{A}$ continuous ($35\,\text{A}$ peak) at $22.2\,\text{V}$. The internal MOSFETs and current shunt resistors dissipate $12\text{--}18\,\text{W}$ per board under peak flapping.
* **Internal Pod Airflow**: In an enclosed fuselage pod without forced ventilation, ambient temperature around the inverters will rise rapidly toward the ODrive thermal trip point ($85^\circ\text{C}$).
* **Action**: Ensure the ODrive aluminum heatsink plates are thermally coupled to the external fuselage airflow louvers using $1.5\,\text{mm}$ silicone thermal pads (thermal conductivity $k \ge 6.0\,\text{W/m}\cdot\text{K}$).

---

## 5. Aerodynamics, Downwash Interaction & Longitudinal Trim

### 5.1 Passive Glide Lift Deficit & Cruise Flapping Requirement (PHY-03)
* **Aerodynamic Calculation**:
  - Maximum Takeoff Weight: $m = 5.687\,\text{kg} \implies W = m \cdot g = 55.77\,\text{N}$.
  - Total Wing Area (8 wings): $S = 0.7124\,\text{m}^2$.
  - Air density at sea level: $\rho = 1.225\,\text{kg/m}^3$.
  - Design Cruise Airspeed: $V_{\text{cruise}} = 9.0\,\text{m/s}$.
  - Clean Maximum Cruise Lift Coefficient (S1223 airfoil): $C_L \approx 1.05$.
* **Passive Aerodynamic Lift at $9.0\,\text{m/s}$**:
  $$L_{\text{passive}} = \frac{1}{2} \rho V^2 S C_L = 0.5 \times 1.225 \times 9.0^2 \times 0.7124 \times 1.05 = \mathbf{37.13\,\text{N}}$$
* **Lift Deficit**:
  $$\Delta L = W - L_{\text{passive}} = 55.77\,\text{N} - 37.13\,\text{N} = \mathbf{18.64\,\text{N}} \quad (33.4\% \text{ deficit!})$$
* **Flight Performance Impact**:
  - **The aircraft cannot sustain level flight at $9.0\,\text{m/s}$ in pure unpowered glide.**
  - To produce the missing $18.64\,\text{N}$ of vertical force, the aircraft relies on active metachronal flapping to generate stroke-averaged excess lift.
  - **Minimum Sustained Flapping Frequency**: Flapping frequency in cruise must not drop below **$f_{\text{cruise,min}} = 1.87\,\text{Hz}$** (nominal $2.20\text{--}2.50\,\text{Hz}$).
  - **Dead-Stick Glide Speed**: If all 8 flapping motors experience power loss, the autopilot must immediately pitch down to establish an unpowered glide speed of:
    $$V_{\text{glide}} \ge \sqrt{\frac{2 W}{\rho S C_{L,\max}}} = \sqrt{\frac{2 \times 55.77}{1.225 \times 0.7124 \times 1.45}} \approx \mathbf{9.38\,\text{m/s}} \quad (\text{with safety margin: } \ge \mathbf{11.0\,\text{m/s}})$$

```
Lift vs Airspeed Operational Envelope:
  Lift (N)
   60 |--------------------------------------------- [MTOW Level Flight = 55.77 N]
      |                                      . - '
   50 |                              . - '
      |                      . - '   <--- Total Lift with Active Flapping (f = 2.4 Hz)
   40 |              . - ' 
      |      . - ' [Passive Lift at 9 m/s = 37.13 N (18.64 N Deficit)]
   30 | . - ' 
      |_____________________________________________
      0      3      6      9      11     14   Airspeed (m/s)
                          ^              ^
                     Cruise (9 m/s)   Safe Dead-Stick Glide (11 m/s)
```

### 5.2 Cascade Downwash Induced De-Cambering
* **Aerodynamic Phenomenon**: In an 8-wing tandem cascade, Station 1 ($W_1$) sheds a strong vortex wake and downwash field that washes over Stations 2, 3, and 4.
* **Induced AoA Reduction**: The downwash angle $\epsilon$ decreases the effective angle of attack seen by downstream wings:
  $$\alpha_{\text{eff}, n} = \alpha_{\text{geom}, n} - \epsilon_n, \quad \text{where } \epsilon_n \approx \frac{2 C_L}{\pi AR} \approx 2.5^\circ\text{--}4.0^\circ$$
* **Impairment**: If all wings are commanded to identical pitch angles, Stations 2, 3, and 4 will operate below their optimal $L/D$, causing aft lift loss and nose-up pitch trim drift.
* **Firmware Fix**: Ensure `mco_wave_library.py` downwash wash-in trim offsets are applied:
  - Station 1: $+0.0^\circ$
  - Station 2: $+1.8^\circ$ pitch bias
  - Station 3: $+2.8^\circ$ pitch bias
  - Station 4: $+3.2^\circ$ pitch bias

### 5.3 Center of Gravity (CG) Envelope Verification
* **Geometric Reference**: Datum $0.0\,\text{mm}$ at nose tip. Mean Aerodynamic Chord (MAC) = $120.0\,\text{mm}$, with LEMAC at $280.0\,\text{mm}$. Tandem neutral point $x_{\text{NP}} = 370.5\,\text{mm}$.
* **Static Margin Requirements**:
  - Nominal Design CG: **$26.5\%$ MAC** ($x_{\text{CG}} = 280.0 + 0.265 \times 120.0 = \mathbf{311.8\,\text{mm}}$ aft of datum).
  - Forward Limit: $20.5\%$ MAC ($304.6\,\text{mm}$) — high pitch stability, high elevator authority required.
  - Aft Stability Limit: $29.5\%$ MAC ($315.4\,\text{mm}$) — neutral stability threshold.
* **Shop-Floor Calibration**:
  - The battery sled quick-pin rail (`MFG-MK1` Section 7) has $10\,\text{mm}$ indexing holes.
  - **Hole 3 ($250.0\,\text{mm}$ sled position)** corresponds strictly to the $26.5\%$ MAC nominal trim.
  - Sled positions beyond Hole 5 ($270\,\text{mm}$) push CG aft of $29.5\%$ MAC and **must be mechanically locked out** during initial flight testing.

---

## 6. Actuation, Electrical Power & Servo Bus Dynamics

### 6.1 AGFRC IB53BHU Dynamic Slew Rate Derating Under Load (PHY-07)
* **Actuator Specification**: The 16 AGFRC IB53BHU brushless servos are rated for $53.0\,\text{kg}\cdot\text{cm}$ ($5.2\,\text{N}\cdot\text{m}$) stall torque and $0.070\,\text{s}/60^\circ$ ($857^\circ/\text{s}$) no-load speed at $7.4\,\text{V}$.
* **Loaded Dynamic Slew Derating**: During high-frequency flapping ($3.0\text{--}3.5\,\text{Hz}$), oscillating aerodynamic lift and inertial pitch moments exert up to $0.85\,\text{N}\cdot\text{m}$ opposing torque on the pitch tie-rods. Under this load, brushless servo coreless motors slow down according to the linear torque-speed curve:
  $$\omega_{\text{loaded}} = \omega_0 \left(1 - \frac{\tau_{\text{load}}}{\tau_{\text{stall}}}\right) = 857^\circ/\text{s} \times \left(1 - \frac{0.85}{5.2}\right) \approx 717^\circ/\text{s} \implies 0.084\,\text{s}/60^\circ$$
* **Impact on Insect Biomechanics (H-89 Pre-Rotation)**:
  - Hypothesis H-89 requires a rapid trapezoidal pre-rotation ($\Delta \alpha = 40^\circ$ in $\Delta t = 25\,\text{ms}$, requiring $\approx 444^\circ/\text{s}$ effective slew rate).
  - While $717^\circ/\text{s} > 444^\circ/\text{s}$ provides a theoretical $61\%$ dynamic speed margin, any voltage drop on the $7.4\,\text{V}$ rail below $6.8\,\text{V}$ will cause the servo to lag behind the command, degrading stroke-reversal rotational lift.
* **Shop-Floor Verification**:
  - Measure servo voltage on an oscilloscope during simultaneous 16-servo step actuation. Ensure voltage droop is $\le 0.3\,\text{V}$ (rail maintains $\ge 7.1\,\text{V}$).

### 6.2 7.4V Servo Bus Inductive Kickback & Decoupling (PHY-08)
* **Transient Current Demands**: 16 servos undergoing simultaneous stroke-reversal draw up to $16 \times 2.8\,\text{A} = \mathbf{44.8\,\text{A}}$ instantaneous surge current.
* **Inductive Spikes**: The inductance of the 2S LiPo battery wiring harness ($L \approx 1.2\,\mu\text{H}$) creates large $L \frac{di}{dt}$ voltage spikes and dips ($\pm 2.5\,\text{V}$) on the 7.4V rail.
* **Risk to Flight Computer**:
  - The MCP3208 12-bit SPI ADCs (monitoring analog servo position feedback) and Pixhawk 4 Mini sensor rails can suffer from ground bounce and brownout resets if transient ripple exceeds $500\,\text{mV}$.
* **Mandatory Fix**:
  - A **$4,700\,\mu\text{F} \times 16\,\text{V}$ ultra-low ESR ($<0.02\,\Omega$) electrolytic capacitor** must be soldered with leads $<20\,\text{mm}$ directly across the 7.4V servo power distribution board terminals.

### 6.3 Sprint Thermal Duty Cycle & AGFRC IB53BHU Protection (PHY-09)
* **High-Speed Dash / Sprint Thermal Vulnerability**: During sustained dash or burst flapping ($f = 3.5\text{--}4.0\,\text{Hz}$ with stroke amplitudes up to $\pm 34^\circ$), 16 pitch and sweep servos operate near continuous peak torque against dynamic hinge moments and inertia. Continuous current on the 7.4V rail can reach $18.0\text{--}24.0\,\text{A}$, causing coreless motor winding temperatures to climb toward the $95^\circ\text{C}$ demagnetization threshold within 30 seconds.
* **Firmware Fix 4 Enforcement**:
  - Implemented in `firmware/mco_flight_governor.py` (`SprintThermalDutyTracker`) and integrated into the 50 Hz flight loop (`mco_main.py`).
  - **Burst Duration Ceiling**: Maximum continuous sprint duration is hard-capped at **$20.0\,\text{s}$**.
  - **Thermal Cooldown Lockout**: Requires a mandatory **$45.0\,\text{s}$** continuous sub-sprint cooldown ($f \le 2.5\,\text{Hz}$, Wave Lever $< 88\%$) before sprint mode can re-engage.
  - **Servo Bus Current Clamp**: If total 7.4V servo bus current exceeds **$18.0\,\text{A}$** during sprint, the governor trips an immediate overcurrent lockout, derating flapping dynamics to safe cruise ceiling:
    $$f_{\text{derated}} \le 2.2\,\text{Hz}, \quad \theta_{\text{stroke}} \le \pm 18.0^\circ$$
* **Rejection of Open-Loop Phase-Lead Feedforward (Fix 3)**:
  - Feedforwarding dynamic pitch lead ($\Delta t \approx 58\,\text{ms}$) open-loop based purely on nominal servo transfer functions was analyzed and rejected.
  - *Failure Mode*: When battery bus voltage droops under heavy flapping or aero hinge moments fluctuate unpredictably, open-loop feedforward causes command saturation ("whiplash"), 100% duty stall torque spikes, and gear tooth micro-stripping on the AGFRC IB53BHU steel gear train, introducing phase overshoot that risks explosive aeroelastic divergence. Closed-loop phase tracking and duty cycle clamping (Fix 4) are strictly mandated instead.
* **Biomimetic Pterostigma Slug Mass & Aft Subtraction Balance**:
  - To prevent high-speed flutter divergence above $15\,\text{m/s}$ without penalizing flapping inertia, all 8 wings feature a $1.5\,\text{g} \pm 0.1\,\text{g}$ brass slug ($\varnothing 3.0\,\text{mm} \times 24\,\text{mm}$) installed at $85\%$ semispan ($y = 0.85b$) and $x/c \approx 0.08$ (forward of the $20\% c$ main spar).
  - Paired with the **5.4:1 Aft Subtraction Rule**: every $1.0\,\text{g}$ of mass lightened from the trailing edge ($65\% c$ aft of spar) provides the same flutter-suppression forward CG shift as adding $5.4\,\text{g}$ of dead nose ballast, preserving wing lightness while pushing flutter divergence speed beyond $28\,\text{m/s}$.

---

## 7. Avionics & Low-Dynamic-Pressure Sensing

### 7.1 MS4525DO Pitot-Static Dynamic Pressure Resolution (PHY-06)
* **Sensor Physics**: The digital pitot sensor is an MS4525DO with a $1\,\text{PSI}$ ($6,895\,\text{Pa}$) differential pressure range and a 14-bit ADC ($16,384$ total counts, resolution $\Delta P \approx 0.421\,\text{Pa/count}$).
* **Dynamic Pressure at Low Airspeeds**:
  - Takeoff / Unstick Speed ($V = 6.2\,\text{m/s}$):
    $$q_{\text{takeoff}} = \frac{1}{2} \rho V^2 = 0.5 \times 1.225 \times 6.2^2 = \mathbf{23.54\,\text{Pa}} \implies \mathbf{56\,\text{counts}}$$
  - Nominal Cruise Speed ($V = 9.0\,\text{m/s}$):
    $$q_{\text{cruise}} = 0.5 \times 1.225 \times 9.0^2 = \mathbf{49.61\,\text{Pa}} \implies \mathbf{118\,\text{counts}}$$
* **Signal-to-Noise Ratio (SNR) Analysis**:
  - At $6.2\,\text{m/s}$, raw sensor noise of $\pm 4\,\text{counts}$ represents a dynamic pressure uncertainty of $\pm 1.68\,\text{Pa}$.
  - In airspeed terms:
    $$V = \sqrt{\frac{2 q}{\rho}} \implies \frac{dV}{dq} = \frac{1}{\rho V} \implies \Delta V \approx \frac{1.68}{1.225 \times 6.2} \approx \pm \mathbf{0.22\text{--}0.35\,\text{m/s}}$$
* **Impairment**: Unfiltered pitot noise can falsely trigger the autopilot's synthetic airspeed failover state or stall warnings during STOL rotation and ground-effect rollouts.
* **Firmware Mitigation**:
  - Configure the Pixhawk EKF3 airspeed fusion to apply a **5-point rolling median filter** on raw dynamic pressure readings.
  - Implement synthetic airspeed blending (using GPS velocity and motor RPM / flapping frequency correlation) whenever indicated airspeed is below $7.5\,\text{m/s}$.

---

## 8. Actionable Pre-Flight Shop-Floor Checklist

Before mounting the airframe onto the outdoor test rig or initiating low-speed taxi runs, the build team must sign off on every item below:

```
[ ] 0. As-Built Gross Weight & Mass Budget Handoff (PRIMARY CALIBRATION GATE)
       - Measure complete flight-ready airframe (with 6S + 2S LiPos, avionics, canopy) on calibrated digital scale.
       - As-built target: < 5,600 g (baseline reference: 5,687 g MTOW).
       - Record 3-point wheel reactions (nose gear, port main, stbd main) to confirm transverse symmetry and empty CG.
       - Hand off exact gross weight figure to firmware: dynamically updates stall floor, Boyd Ps, and hover thrust margin.

[ ] 1. Dyneema 48-Hour Creep Settlement
       - Verify cables rested at 200 N for 48 hours.
       - Re-tensioned to 180 N ± 5 N across all 8 capstans.
       - Loctite 480 applied to clamping cams; witness marks aligned.

[ ] 2. Carbon Spar Internal Ferrules
       - Aluminum 6061-T6 ferrules (Ø8.0 mm x 40 mm) epoxied into all 8 spar roots.
       - Spar clamp bolts torqued to 2.2 N*m with Loctite 243.

[ ] 3. Motor Mount Thermal Isolation & Drive Model
       - GoolRC 5010-14 360KV brushless motors verified on all 8 flapping stations (95g ea).
       - 0.5 mm silicone/FR4 thermal barrier gaskets installed between motors and mounts.
       - 2.0 mm 6061-T6 aluminum faceplates verified under all 8 motor mounts.
       - Ram-air NACA scoops verified clear with unobstructed internal airflow.

[ ] 4. Center of Gravity (CG) Position
       - Sled quick-release pin locked into Hole 3 (250 mm mark).
       - Complete airframe balanced on knife-edge: CG falls at 311.8 mm ± 3.0 mm (26.5% MAC).

[ ] 5. Servo Bus Electrical Integrity & Sprint Protection
       - 4,700 µF 16V low-ESR capacitor verified on 7.4V servo rail (leads < 20 mm).
       - Full 16-servo simultaneous sweep test: 7.4V rail voltage remains >= 7.1V.
       - Sprint thermal governor verified: 20s burst limit, 45s cooldown lockout, 18.0A trip.

[ ] 6. Biomimetic Pterostigma Slug & Wing Mass Balance
       - 1.5 g ± 0.1 g brass slug verified at y = 0.85b, x/c ≈ 0.08 in all 8 wings.
       - Trailing-edge mass verified skeletonized (Aft Subtraction Rule: 5.4:1 ratio).
       - Chordwise CG verified forward of elastic axis (<= 24% chord).

[ ] 7. CAN-FD Bus Resistance
       - DMM resistance across CAN_H and CAN_L measures exactly 60.0 Ω ± 1.5 Ω.

[ ] 8. Dynamic Clearance & Sweep Lock
       - Software sweep differential clamped to <= 15° during high-stroke flapping.
       - Station spacing verified at 250 mm stagger and 41.25 mm vertical staircase.
       - All 8 wingtips clear adjacent structures by >= 35.0 mm across all stroke phases.
       - 200° sector wheel travel verified: mechanical stops at ±100.0°, dynamic flap bounded to ±96.0°.

[ ] 9. Pitot Low-Speed Calibration
       - Pitot tube zero-offset calibrated with cover attached at ambient temperature.
       - Low-speed rolling median filter enabled in flight controller EKF3.
```

---

## 9. Builder As-Built Calibration Figures Manifest

Once the Mark 1 physical airframe is constructed, the builder must provide the following measured parameters to calibrate the flight controller firmware before first flight:

| Param # | Parameter Name | Symbol | Baseline / Target | As-Built Measured Entry | Firmware Coupling & Impact |
| :---: | :--- | :---: | :---: | :---: | :--- |
| **FIG-01** | **All-Up Gross Weight** | $m_{\text{gross}}$ | **$< 5,600\text{ g}$** ($5,687\text{ g}$ max) | `__________ g` | Scales stall speed $V_s \propto \sqrt{m}$, required lift $L = mg$, Boyd $P_s$, climb ceiling, and hover thrust-to-weight ratio. Lighter mass expands all margins. |
| **FIG-02** | **Wheel Weight Distribution** | $m_N, m_L, m_R$ | $15\% / 42.5\% / 42.5\%$ | `N:___ g, L:___ g, R:___ g` | Direct ground balance confirmation of longitudinal CG and lateral symmetry. |
| **FIG-03** | **Longitudinal CG Station** | $x_{\text{CG}}$ | $311.8 \pm 3.0\text{ mm}$ (Hole 3) | `__________ mm` | Trim elevator bias, pitch stability derivative $C_{m\alpha}$, and neutral point static margin. |
| **FIG-04** | **Sector Wheel Total Travel** | $\theta_{\text{sector\_total}}$ | **$200.0^\circ$** ($\pm 100.0^\circ$) | `__________ deg` | Mechanical travel envelope. Firmware sets reciprocating flapping ceiling to $\pm 96.0^\circ$ ($4.0^\circ$ soft margin) and macro maneuvers to $200.0^\circ$. |
| **FIG-05** | **Flapping Motor Baseline** | Drive Model | **GoolRC 5010-14 360KV** | `Confirmed / Other` | $K_t \approx 0.0265\text{ N}\cdot\text{m/A}$, $10:1$ capstan, root torque $7.95\text{ N}\cdot\text{m}$ cont / $10.6\text{ N}\cdot\text{m}$ burst. |
| **FIG-06** | **Dyneema Resting Cable Tension** | $T_{\text{cable}}$ | $180 \pm 5\text{ N}$ | `W1:___, W2:___, W3:___, W4:___ N` | Ensures zero cable backlash across stroke reversals without bearing preload overload. |
| **FIG-07** | **Servo Neutral Centers (16)** | $\theta_{\text{sweep0}}, \theta_{\text{pitch0}}$ | $0.0^\circ \pm 0.5^\circ$ | `Recorded in EKF3 table` | AGFRC IB53BHU PWM microsecond neutral offsets for zero-incidence trim. |
| **FIG-08** | **Pitot Static Offset** | $\Delta P_{\text{offset}}$ | $0.00 \pm 0.02\text{ hPa}$ | `__________ Pa` | Calibrates low-speed airspeed sensor zero before synthetic EKF3 fusion kicks in. |

---
*Document certified into Mark 1 Technical Baseline.*
