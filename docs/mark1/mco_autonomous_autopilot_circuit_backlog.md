# MCO-1D Autonomous Learning Circuit & Autopilot Architecture — Technical Backlog

**Document Status**: Autopilot & Mission Architecture Backlog  
**Applies to**: MCO-1D Mark 1 Prototype & MCO-1A Demonstrator  
**Purpose**: Document the design, state machine, battery hot-swap persistence workflow, and `H-SOAR-CIR` thermal detour logic for autonomous circuit learning, sequestered cleanly from core flight-governor kinematics.

---

## 1. Concept & Vision

The operational goal is **"Endless Autonomous Learning Circuits"**:
1. The operator places the aircraft on a runway or open field, inserts a freshly charged 6S battery, and arms the system.
2. The aircraft executes an autonomous takeoff roll, accelerates past stall speed ($10.5\text{ m/s}$), and climbs at $V_y$ ($11.2\text{ m/s}$) to circuit altitude ($40\text{ m}$ AGL).
3. It flies a predetermined pylon/box circuit ($400\text{ m} \times 150\text{ m}$), executing candidate wave kinematic variations (micro-strokes, phase offsets, pulse bounding).
4. At the end of each lap, the Autonomous Learning Curriculum (ALC) calculates the specific cost of transport ($Wh/\text{km}$), RMS vibration, and thermal headroom.
5. The Pareto optimizer mutates kinematic parameters for the subsequent lap.
6. Upon reaching battery reserve ($21.8\text{ V}$), the aircraft enters a stabilized approach, flares, and stops via aerodynamic rollout braking (`CONV-BRK`).
7. **Hot-Swap Workflow**: The operator swaps the battery pack. All genome mutations, CMTS champions, and learned parameter distributions are loaded seamlessly from non-volatile flash storage (`/home/pi/knowledge/`), resuming immediately at Lap $N+1$ with zero forgotten learning.

---

## 2. Circuit State Machine

```
                   AUTONOMOUS LEARNING CIRCUIT STATE MACHINE

    +-------------------+       Takeoff Clearance        +-------------------+
    |   0. GROUND_IDLE  | ----------------------------> | 1. RUNWAY_TAKEOFF |
    | (Check Telemetry) |                               | (H-CTOL-1 to 11m) |
    +-------------------+                               +-------------------+
              ^                                                   |
              |                                                   v
    +-------------------+                               +-------------------+
    | 5. TOUCHDOWN_STOP |                               |  2. VY_CLIMB_LEG  |
    | (CONV-BRK Airbrak)|                               | (H-71 to 40m AGL) |
    +-------------------+                               +-------------------+
              ^                                                   |
              | Battery Reserve / End Sortie                      v
    +-------------------+                               +-------------------+
    |  4. BASE_FINAL    | <---------------------------- | 3. DOWNWIND_TRIAL |
    | (Glide Floor Sync)|        Circuit Completion     | (Test Hypotheses) |
    +-------------------+                               +-------------------+
                                                                  |
                                              Updraft Detected    | (w > 0.5 m/s)
                                                                  v
                                                        +-------------------+
                                                        |   H-SOAR-CIR      |
                                                        | (Thermal Detour)  |
                                                        | (0W Climb Loiter) |
                                                        +-------------------+
```

### Leg Breakdown
* **Leg 1: Runway Acceleration ($0 \to 10.5\text{ m/s}$)**: Rolling acceleration on Du-Bro foam wheels, preventing high-amp vertical hover burnout.
* **Leg 2: Strouhal Climb ($10.5 \to 11.5\text{ m/s}$, $0 \to 40\text{ m}$ AGL)**: High-efficiency climb at $V_y$ maximizing altitude gain per Watt-hour.
* **Leg 3: Downwind Hypothesis Trial**: The ALC assigns a test hypothesis (`H-MICRO-1`, `H-PULSE-1`, or `H-SHUTTLE-1`) to evaluate energy consumption along a steady $400\text{ m}$ leg.
* **Leg 4: Base / Final & Energy Evaluation**: The lap is scored. If battery capacity $>25\%$, the autopilot initiates a touch-and-go or continuous circuit climb into Lap $N+1$.
* **Leg 5: Aerodynamic Rollout Stop**: Full-stop flare utilizing shuttlecock drag (`CONV-BRK`), disarming motors and waiting for battery change.

---

## 3. `H-SOAR-CIR`: Netto-Variometer Autonomous Circuit Loiter

* **Biological Inspiration**: Frigatebirds and storks patrolling migratory corridors.
* **Mechanism**:
  - The MS5611 barometric altimeter and EKF produce true vertical speed ($w_{\text{z}}$).
  - The netto-variometer subtracts the polar sink rate of the clean airframe at current airspeed ($w_{\text{polar}} \approx 0.85\text{ m/s}$ at $11\text{ m/s}$):
    $$w_{\text{air}} = w_z - w_{\text{polar}}$$
  - If $w_{\text{air}} > +0.45\text{ m/s}$ is sustained for $>2.0\text{ seconds}$, the autopilot interrupts circuit waypoints:
    1. Wings lock in high-camber S1223 dihedral glide ($L/D \approx 10.5:1$).
    2. Flapping motors idle ($P_{\text{motor}} = 0\text{ W}$).
    3. Autopilot enters a thermalling spiral ($18^\circ$ bank, radius $\sim 14\text{ m}$) until $w_{\text{air}} < 0.2\text{ m/s}$ or ceiling ($120\text{ m}$ AGL) is reached.
  - Returns to the circuit at waypoints with positive energy gained for free.

---

## 4. Hot-Swap State Persistence

* **Flash Artifact**: `/home/pi/knowledge/genome_state.json`
* **Persisted Keys**:
  - `generation_count`: Total circuits flown.
  - `active_mechanism_weights`: Current `_ALL_MW_AXES` values.
  - `cmts_champions`: Winning wave parameters for each flight regime.
  - `pareto_frontier`: Optimal trade-offs between $Wh/\text{km}$ and airspeed.
* **Disarm & Arm Hooks**:
  - `disarm`: Flushes memory cache to disk atomically with checksum verification.
  - `arm`: Ingests `genome_state.json`, initializes the wave library with the latest champion parameters, and increments session sortie count.
