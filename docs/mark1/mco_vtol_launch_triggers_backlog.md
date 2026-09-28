# MCO-1D VTOL & High-Impulse Launch Triggers — Technical Backlog & Discussion Guide

**Document Status**: Open Technical Backlog for Operator Alignment  
**Applies to**: MCO-1D 8-Wing Ornithopter Prototype & Mark 2 Amphibious Demonstrator  
**Purpose**: Document all candidate biological launch modes, their physical trigger options, and safety interlocks so they remain cleanly isolated from standard fixed-wing runway operations.

---

## 1. Executive Summary

During initial flight software development, explosive biological launch modes were formulated to provide zero-ground-roll vertical breakout and high-angle climb. However, **standard fixed-wing runway takeoff (CTOL/STOL)** must remain the default operation when a pilot advances the throttle lever with neutral stick.

To prevent inadvertent high-impulse bursts during a conventional runway takeoff roll, all VTOL and explosive launch modes are strictly sequestered behind **explicit, deliberate pilot triggers**.

This document files these launch modes and trigger options away for dedicated review and operator alignment.

---

## 2. Inventory of High-Impulse Biological Launch Modes

| Mode ID | Biological Inspiration | Aerodynamic / Biomechanical Mechanism | Peak Impulse / Acceleration | Target Operational Envelope |
| :--- | :--- | :--- | :--- | :--- |
| **`H-WD-1`** | **Gull Clap-and-Fling Surface Synergy** | Dorsal clap synchronized with water mirror vortex trapping; explosive $+22\%$ lift spike snaps surface suction. | $+22\%$ lift ($>12.3\text{ N}$ transient boost) | Water departures at rotation speed ($V \ge 8.5\text{ m/s}$, $h \le 0.54\text{ m}$). |
| **`H-98`** | **Pterosaur Quadrupedal Vault** | Ground-coupled catapult impulse; front gear extension + rear wing downstroke generates instant liftoff. | $+1.85g$ vertical launch impulse | Confined terrain, zero-ground-roll land departure. |
| **`H-LA-1`** | **Dragonfly $0^\circ$ In-Phase Super-Lift** | Collapse metachronal cascade to $0^\circ$ in-phase synchronized downstroke across all 8 wings; constructive LEV superposition. | **$2.42\times$ MTOW ($>135.0\text{ N}$)** in $<20\text{ ms}$ | Confined vertical takeoff pads, obstacle escape from standstill. |
| **`H-LA-2`** | **Vampire Bat Forearm Catapult** | High-force landing strut spring extension $35\text{ ms}$ prior to stroke 1; vaults airframe $0.4\text{ m}$ before wings stroke. | $+3.5\text{--}4.5g$ in $30\text{ ms}$ ($V_0 \approx 2.4\text{ m/s}$) | Rough terrain, tall grass, obstacle clearance without wingtip strike. |
| **`H-LA-3`** | **Pigeon Clap-and-Peel Vortex Ring** | Asymmetric dorsal peel ($18\text{ ms}$ lead) creates suction wedge; downstroke sheds toroidal starting vortex ring downward. | Vectors force at $75^\circ\text{--}85^\circ$ climb angle | Low-speed ballistic obstacle clearance corridor ($V \le 7\text{ m/s}$). |
| **`H-LA-4`** | **Butterfly Cupped Pneumatic Jet** | Compliant trailing-edge cuff closes dorsal air pocket, squeezing pneumatic downward air jet at stroke reversal. | $+1.5\text{--}2.2g$ impulse | Low-acoustic obstacle jump. |

---

## 3. Candidate Trigger Architectures for Operator Discussion

The operator and pilot team will select from among the following trigger mechanisms:

### Option A: Dedicated Physical Cockpit Switch (Hardware Gate)
* **Description**: A 2-position spring-loaded momentary toggle switch on the throttle quadrant (labeled `LAUNCH BURST`).
* **Behavior**:
  - Advancing throttle to 100% executes a **standard rolling takeoff**.
  - Holding the `LAUNCH BURST` switch while advancing throttle engages the armed launch mode (e.g. Dragonfly Super-Lift or Gull Clap).
  - Releasing the switch or reaching climb breakout immediately returns the flight controller to standard metachronal flight.
* **Pros**: Physically impossible to trigger accidentally; tactile and familiar to military/test pilots.
* **Cons**: Requires dedicated wiring and an extra channel on the RC transmitter/flight deck.

### Option B: Pilot Multi-Axis Stick Gesture (Software Gate)
* **Description**: A distinctive, deliberate combination of stick, throttle, and pedal controls that cannot occur during ordinary taxi or takeoff roll.
* **Example Gestures**:
  - *Pterosaur Vault*: Dual rudder pedals depressed (brakes locked) + full forward throttle + full aft stick, held for $0.2\text{ s}$, then rapid stick centering.
  - *Dragonfly Super-Lift*: Throttle detent double-tap (snap to idle and push full forward within $0.3\text{ s}$) with neutral pitch/roll.
  - *Gull Clap Water Unstick*: Pitch stick quick forward-and-back "nod" at rotation speed on water.
* **Pros**: Zero additional hardware or switches required; fully implemented in software.
* **Cons**: Requires pilot memorization and precise timing.

### Option C: Flight Deck / Ground Station Armed Button (Software Armed + Stick Fired)
* **Description**: The pilot or copilot arms the launch mode via the touchscreen / MAVLink ground station (e.g. `ARM_VTOL_LAUNCH` or `ARM_WATER_CLAP`).
* **Behavior**:
  - Once armed, a visual and audio annunciator alerts the crew: `"LAUNCH BURST ARMED — READY ON RUNWAY"`.
  - The next throttle stroke to 100% executes the launch burst.
  - After 1 execution (or 30 seconds timeout), the system auto-disarms back to standard CTOL mode.
* **Pros**: High situational awareness; clean integration with automated flight test cards.
* **Cons**: Requires copilot or GCS interaction before takeoff.

### Option D: Fully Autonomous Sensor Envelope Trigger (State-Machine Automated)
* **Description**: The flight computer auto-arms the burst only when specific hydrodynamic or physical boundary conditions are satisfied simultaneously.
* **Example**:
  - *Gull Clap*: Only auto-arms when `surface_is_water == True`, airspeed $V \ge 8.5\text{ m/s}$ (unstick speed), and radar altimeter $h \le 0.54\text{ m}$.
* **Pros**: Zero pilot workload; fires exactly at the optimal microsecond for hydrodynamic unstick.
* **Cons**: Must be tested extensively to guarantee it never fires in error over shallow water or spray.

---

## 4. Discussion Action Items for Future Launch Alignment

1. **Pilot Preference**: Determine whether the pilot prefers a dedicated hardware switch (Option A) or software arming (Option C) for land-based VTOL bursts.
2. **Gull Clap Envelope**: Decide whether the Gull Clap on water should be semi-autonomous (Option D with master arm switch) or purely pilot-commanded.
3. **Pterosaur vs Dragonfly Selection**: Determine whether the default high-impulse launch for confined pads should be the mechanical landing-gear vault (`H-98`) or the aerodynamic in-phase super-lift (`H-LA-1`).
4. **Failsafe Reversion**: Confirm that in all cases, any pilot manual stick deflection $> \pm 0.15$ immediately aborts the automated burst and hands full manual authority back to the pilot.
