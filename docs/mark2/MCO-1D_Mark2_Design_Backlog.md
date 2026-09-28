# MCO-1D Mark 2 Advanced Design Backlog & Hardware Repository

**Aircraft**: MCO-1D Metachronal Cascade Ornithopter  
**Baseline Distinction**:
- **Mark 1 (Current Hardware)**: Fixed 10mm OD solid-wall carbon spars, standard S1223 ribs, Oracover film skin, 8-motor / 16-servo kinematic layout. Focus: Software-only algorithmic envelope expansion, ground bench characterization (TC-01..TC-03), and initial flight clearance (TC-04).
- **Mark 2 (Next-Iteration Airframe)**: Structural redesign, advanced biomimetic composites, articulated high-lift surfaces, and generative osteological mechanics.

---

## 1. Pterosaur Hardware Evolution Suite (Deferred from Mark 1)

These hypotheses require physical tooling, specialized composite manufacturing, or articulated mechanism fabrication that cannot be deployed via pure firmware on the existing Mark 1 airframe.

### H-96: Actino-Fibril Anisotropic Membrane Anti-Ballooning Covering
- **Biological Origin**: Pterosaur *Brachiopatagium* (dense sub-millimeter actinofibrils embedded chordwise within dermis).
- **Mechanical Architecture**:
  - Replace isotropic heat-shrink Oracover polyester film with an anisotropic pre-tensioned composite skin.
  - Pultruded $0.28\text{ mm}$ carbon fiber micro-battens spaced at $12\text{ mm}$ chordwise intervals along upper camber between ribs 1–4.
  - Encapsulated in a $25\text{ g/m}^2$ Japanese paper / TPU elastomeric film matrix.
- **Problem Solved**:
  - Eliminates aerodynamic suction ballooning between ribs ($3\text{--}5\text{ mm}$ billow sag under $\Delta p \approx 300\text{ Pa}$ at $V > 18\text{ m/s}$).
  - Locks designed laminar bucket of the S1223 section across the full speed envelope ($0\text{ to }30\text{ m/s}$).
- **Target Performance**: $28\%$ reduction in high-speed profile drag ($C_{D0}$); dash speed increased to $28.5\text{ m/s}$.
- **Mark 2 Fabrication Requirements**: Vacuum-bagged heated layup tooling; tensioning jig for axial pre-load during curing.

---

### H-97: Pteroid Dynamic Leading-Edge Droop Flap (Propatagium Krueger Slat)
- **Biological Origin**: Pterosaur *Pteroid* bone (wrist-articulated forward bone driving dynamic propatagium droop).
- **Mechanical Architecture**:
  - Articulated leading-edge nose section ($0\text{--}15\%$ of chord) on Stations 1 and 2 (front 4 wings).
  - Continuous carbon micro-hinge ($0.8\text{ mm}$ CF rod) driven by an internal slave bellcrank linkage coupled to the pitch actuator horn.
  - Deflects downward by $20^\circ\text{--}25^\circ$ during slow flight, landing flares, and steep obstacle climbs; retracts flush into a low-drag knife-edge nose for clean cruise.
- **Problem Solved**:
  - Suppresses severe leading-edge suction spikes and laminar separation bubbles that limit the standard S1223 to $\alpha \le 18^\circ$.
- **Target Performance**: Delays stall to $\alpha_{\text{stall}} \ge 27^\circ$; transient $C_{L,\text{max}} \ge 2.42$ ($+31\%$ lift headroom for obstacle climb and short-field flare).
- **Mark 2 Fabrication Requirements**: 2-piece split CNC rib molds; micro-ball-bearing bellcranks; flexible silicone nose transition fairings.

---

### H-99: Thin-Walled Trabecular Pneumatized Tubular Spars
- **Biological Origin**: Pterosaur Wing Digit IV osteology (thin-walled hollow bone tubes with $R/t \approx 20\text{--}40$ reinforced by internal Warren-truss bone trabeculae).
- **Mechanical Architecture**:
  - Replace current $10.0\text{ mm}$ OD $\times 1.0\text{ mm}$ wall solid carbon tube ($R/t = 5.0$, $138\text{ g}$ per wing) with thin-walled $14.0\text{ mm}$ OD $\times 0.45\text{ mm}$ high-modulus carbon tube ($R/t = 15.5$).
  - Internal spatial Warren truss core: 3D-printed ultralight ColorFabb LW-PLA star-trabecular inserts ($0.15\text{ g}$ each) spaced at $40\text{ mm}$ intervals, bonded with foaming polyurethane.
- **Problem Solved**:
  - Thin-walled tubes under cyclic bending suffer catastrophic Brazier ovalization buckling (cross-sectional flattening under bending). The internal trabeculae brace the compressive cortex against the tensile cortex, completely suppressing ovalization.
- **Target Performance**:
  - Second moment of area ($I$) increased by $+82\%$.
  - Torsional polar rigidity ($J$) increased by $+74\%$.
  - Spar mass reduced from $138\text{ g}$ to $76\text{ g}$ per wing—**saving $496\text{ g}$ of flapping mass across all 8 wings** ($8.7\%$ of total MTOW!).
- **Mark 2 Fabrication Requirements**: Mandrel-wound ultra-thin HM carbon tubes; automated trabecular insert bonding fixtures.

---

## 2. Additional Mark 2 Structural & Hardware Aspirations

| Design Item | Origin / Inspiration | Mark 2 Implementation | Benefit |
| :--- | :--- | :--- | :--- |
| **MK2-HW-01: Integrated Motor-Hinge Hub** | Odonata basalar fulcrum | Custom frameless brushless motors built directly inside the titanium wing root gimbal | Eliminates sector gear rope drive; cuts 320g nacelle weight |
| **MK2-HW-02: Active Wingtip Pterostigma Pocket** | Pterosaur wingtip mass | CNC milled micro-cavities with remote micro-solenoids for variable tip inertia damping | Adaptive aeroelastic flutter suppression across wide speed bands |
| **MK2-HW-03: Quick-Release Folding Spar Joints** | Pterosaur wing folding | Spring-loaded collet locks on Station 2 & 3 spars | 45-second pack-down to 800mm backpack footprint |

---

## 3. Cargo Drone & eVTOL Hardware Requirements (Mark 2 Expansion)

These hardware systems are required for the Mark 2 airframe to graduate from short-takeoff research into fully autonomous, certified commercial **Cargo Drone** and urban **eVTOL** operations.

### MK2-HW-04: High-Torque eVTOL Hover Motors & 8S LiPo High-Voltage Propulsion Bus
- **Requirement Type**: eVTOL Pure Hover Propulsion
- **Problem Solved**:
  - Mark 1 operational as-built MTOW is $5.687\text{ kg}$ ($55.8\text{ N}$). Current Option A flap motors produce $\approx 28\text{--}32\text{ N}$ static thrust, necessitating STOVL or H-98 vault ground takeoff.
  - Mark 2 requires true pure vertical VTOL ($T/W \ge 1.20$, $> 67\text{ N}$ static lift) from confined vertiport pads at full cargo MTOW ($6.5\text{ kg}$).
- **Hardware Specification**:
  - 8× high-pole-count ($28\text{P}/24\text{S}$) custom pancake outrunner flap motors, $160\text{ KV}$, delivering $3.8\text{ N}\cdot\text{m}$ continuous torque at $100\text{ Hz}$ bandwidth.
  - High-voltage **8S (33.6V) 6000 mAh 75C LiPo pack** to halve bus current ($I_{\text{peak}} < 35\text{ A}$ per motor) and eliminate thermal saturation in sustained hover.

### MK2-HW-05: Precision Downward LiDAR & Optical Flow Vertiport Sensor Suite
- **Requirement Type**: eVTOL Precision Vertiport Guidance & In-Ground Effect (IGE)
- **Problem Solved**:
  - Barometric altimetry has $\pm 0.5\text{ m}$ latency and ground-bounce errors within $1.0\text{ m}$ of the vertiport pad surface, destabilizing the In-Ground-Effect (IGE) cushion law.
- **Hardware Specification**:
  - **Down-Facing Micro-LiDAR**: Benewake TFmini Plus / Garmin LIDAR-Lite v4 ($0.04\text{--}30\text{ m}$ range, $100\text{ Hz}$, $\pm 10\text{ mm}$ resolution) over CAN / I2C.
  - **High-Speed Optical Flow Sensor**: PMW3901 optical tracking sensor ($80\text{ fps}$) with integrated micro-lens for sub-centimeter lateral station-keeping over vertiport landing markers under GPS-denied urban canyon conditions.

### MK2-HW-06: Electro-Permanent Magnetic Payload Release Hook with Integrated Strain Gauge
- **Requirement Type**: Autonomous Cargo Delivery & Air-Drop Kinematics
- **Problem Solved**:
  - Mark 1 lacks real-time cargo mass telemetry and requires manual payload attachment. Air-drops must coordinate with H-CR-2 de-ballooning pitch steps within $< 30\text{ ms}$.
- **Hardware Specification**:
  - Zero-standby-power **Electro-Permanent Magnet (EPM) / Mechanical Latch Hook** (fails safe in clamped position upon power loss).
  - Integrated **Full-Bridge Foil Strain Gauge Load Cell** ($0\text{--}60\text{ N}$ capacity) directly in the hook mount. Provides real-time hook cable tension to `mco_cg_estimator.py`, auto-detects payload pickup, and triggers the H-CR-2 de-balloon pitch step simultaneously with hook release.

### MK2-HW-07: Ballistic Emergency Parachute Recovery System (BRS)
- **Requirement Type**: eVTOL Airworthiness & Urban Flight Safety (FAA/EASA Part 21/23)
- **Problem Solved**:
  - Flying over people or in urban vertiport terminal airspace requires deterministic mitigation against dual-bus failure or mid-air structural collision.
- **Hardware Specification**:
  - **Pneumatic / Mechanical Spring Ejection Canister** ($180\text{ g}$ complete system mass) carrying a $1.8\text{ m}$ high-drag annular parachute.
  - Autonomous deployment triggered directly by FCC safety monitor on total power loss or unrecoverable pitch divergence below $30\text{ m}$ AGL, limiting ground impact velocity to $< 4.5\text{ m/s}$.

### MK2-HW-08: Suspended Cargo Micro-Winch with Active Cable Spooler
- **Requirement Type**: Cargo Drone Hover Delivery
- **Problem Solved**:
  - Delivering cargo to uneven terrain, boats, or confined spaces where landing is prohibited.
- **Hardware Specification**:
  - Carbon-composite spool driven by an internal miniature planetary gear motor ($12\text{ V}$, $1.5\text{ m/s}$ line speed, $15\text{ m}$ Dyneema braided cable).
  - Optical line encoder for millimeter cable depth telemetry, feeding cable length and swing angle directly into the **H-HL-3 Cargo Pendulum Damping** controller.

---

## 4. Amphibious Seaplane & Seafoil Hydrodynamic Hardware Suite (Mark 2 Expansion)

These systems are deferred from Mark 1 to allow the flight demonstrator to validate airframe kinematics and flight laws without early mechanical commitment to hulls, floats, or hydrofoils.

### MK2-HW-09: Retractable Lisa Akoya Surface-Piercing Seafoils (Composite Hydrofoils)
- **Requirement Type**: Hydrofoil Assisted Water Operations & Takeoff
- **Biological Origin**: Black Skimmer (*Rynchops niger*) mandibular hydrofoil knife-blade hydrodynamic slicing; Swan (*Cygnus*) foot hydroplaning.
- **Problem Solved**:
  - Traditional displacement hulls suffer severe bow-wave "hump drag" peaking at $35\%\text{--}50\%$ stall speed, demanding massive burst power to break out onto the step.
  - Seafoils exploit the $816\times$ density ratio ($\rho_{\text{water}} = 1000\text{ kg/m}^3$ vs $\rho_{\text{air}} = 1.225\text{ kg/m}^3$). A tiny foil area of $A_{\text{sub}} \approx 0.035\text{ m}^2$ generates over $56\text{ N}$ of hydrodynamic lift at just $2.6\text{ m/s}$ ($5.0\text{ knots}$), lifting the fuselage clear of water long before hull hump drag develops.
- **Hardware Specification**:
  - Twin carbon/epoxy surface-piercing dihedral V-hydrofoils mounted on main gear pivot struts, with secondary tip stabilization foils.
  - High-torque brushless retraction servo with mechanical over-center lock for seamless runway/water amphibious operation.
  - Anti-ventilation fences on the strut legs to prevent atmospheric air suction from collapsing foil lift.

### MK2-HW-10: Stepped Hull & Modular Twin Sponson Floats
- **Requirement Type**: Displacement Water Taxi & Secondary Flotation
- **Problem Solved**:
  - Low-speed displacement taxi and static flotation in sea states up to Douglas Sea State 2 (0.3m chop).
- **Hardware Specification**:
  - Modular twin carbon/Kevlar sponson floats or single blended flying-boat hull with a $6^\circ$ transverse ventilated step.
  - Longitudinal spray rails to deflect spray clear of wing root mechanisms and propeller/flapper arcs.

### MK2-HW-11: Dual-Frequency Downward Water Surface & Wave Profiler
- **Requirement Type**: Seafoil Immersion Depth Regulation & Wave Height Estimation
- **Problem Solved**:
  - Seafoils require real-time knowledge of water surface ride height ($h_{\text{water}}$) and wave orbital velocity to prevent foil ventilation or hull slamming.
- **Hardware Specification**:
  - Downward-facing dual-frequency waterproof acoustic sonar ($200\text{ kHz}$, $50\text{ Hz}$ update, $0.05\text{--}5.0\text{ m}$ range) paired with a high-speed pulsed time-of-flight waterproof micro-LiDAR ($905\text{ nm}$, IP67, $100\text{ Hz}$).
  - Feeds ride height $h(t)$ and wave variance $\sigma^2_{\text{wave}}$ into the firmware foil heave regulation loop.

### MK2-HW-12: IP67 Marine Watertight Avionics Bay & Motor Ingress Protection
- **Requirement Type**: Marine Environmental Hardening
- **Problem Solved**:
  - Saltwater spray and moisture corrosion on electronics, CAN-FD buses, and motor bearings.
- **Hardware Specification**:
  - O-ring sealed CNC aluminum central avionics tub with Gore-Tex hydrophobic pressure-equalization vents.
  - Flap motors with ceramic hybrid bearings and conformal vacuum parylene coating on stators; IP67 rated brushless servos.

### MK2-HW-13: Hashimoto Clap-and-Fling Transient Water Unstick Actuators
- **Requirement Type**: High-Synergy Water Departure Burst (H-WD-1)
- **Biological Origin**: Seagull and goose water takeoff dynamics (*Hashimoto et al., JRM Oct 2024*).
- **Problem Solved**:
  - Surface tension and suction adhesion during the final millimeter of water separation.
- **Hardware Specification**:
  - Transient motor driver current headroom (+25% burst for 0.40s) enabling symmetric wingtip clap at $h/b < 0.25$ to trap ground mirror vortex rings, producing a $+22\%$ lift spike that unhooks foils from water adhesion instantly.

---

## 5. Amphibious Operations Flight Control Architecture & State Machine

The Mark 2 software stack coordinates the dual-fluid lift handshake:
$$\gamma_{\text{lift}}(t) = \frac{L_{\text{aero}}(t)}{W_{\text{MTOW}}}$$

```
+-------------------------------------------------------------------------------+
|                       MARK 2 AMPHIBIOUS STATE MACHINE                         |
+-------------------------------------------------------------------------------+
| 1. WATER_DISPLACEMENT: Buoyant float taxi, wings low-drag idle               |
|            │ (Thrust applied, V > 1.2 m/s)                                    |
|            ▼                                                                  |
| 2. FOIL_PLANING: Seafoils lift fuselage out of water (V = 2.6 m/s)            |
|            │ (Airspeed accelerates on foils, γ_lift reaches 0.85)             |
|            ▼                                                                  |
| 3. LIFT_HANDSHAKE: Aerodynamic lift takes over vertical load                  |
|            │ (γ_lift ≥ 0.95, unstick window reached)                          |
|            ▼                                                                  |
| 4. UNSTICK_BURST: Hashimoto clap-and-fling pulse (+22% lift), foils exit     |
|            │ (Clear of water, h_water > 0.4m)                                 |
|            ▼                                                                  |
| 5. WIG_CRUISE: Pelican Wing-in-Ground low-drag wave skimming                  |
+-------------------------------------------------------------------------------+
```

