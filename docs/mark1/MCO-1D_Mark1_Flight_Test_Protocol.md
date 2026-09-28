# MCO-1D Mark 1 — Flight Test Protocol & Test Cards Manual

**Aircraft Title**: MCO-1D Mark 1 Metachronal Cascade Ornithopter  
**Document Designation**: MCO-1D-TEST-MK1  
**Classification**: Authoritative Flight Test Baseline — Revision Mark 1 (September 2026)  

---

## 1. Flight Test Philosophy & Phased Safety Gates

The **MCO-1D Mark 1** flight test campaign follows a strict four-stage progressive clearance program. Progression to subsequent test phases is locked until all objective criteria of the preceding test card are certified by the test director.

```mermaid
flowchart LR
    TC1["TC-01: Ground Bench Test (Rigid Post & Load Cell)"] -->|Static Thrust >= 35N| TC2["TC-02: Tethered Lift Characterization"]
    TC2 -->|Aero Stability Certified| TC3["TC-03: Advance-Ratio Truck Sweep"]
    TC3 -->|CT vs J Polar Mapped| TC4["TC-04: Free Flight STOL Runway Expansion"]
```

### Safety Abort Criteria (All Phases)
1. **Dyneema Cable Slippage**: Any sector wheel angular slip $> 2.0^\circ$ relative to motor position aborts test immediately.
2. **Thermal Limit**: ODrive S1 FET temperature $> 75^\circ\text{C}$ or motor winding temperature $> 85^\circ\text{C}$.
3. **CAN Communication Dropout**: Loss of more than 2 consecutive 50 Hz frames on any ODrive node.
4. **Vibration Spike**: RMS airframe accelerometer vibration $> 2.5\text{ g}$ in the $10 - 200\text{ Hz}$ band.

---

## 2. Test Card TC-01: Ground Bench Test

### 2.1 Objective & Test Setup
Measure static thrust, electrical power draw, capstan cable behavior, and thermal equilibrium across the flapping frequency range ($1.0\text{ Hz}$ to $3.5\text{ Hz}$).

```
TC-01 Mechanical Fixture:
   +=============================================+  Ceiling / Rigid Anchor
   |                    LOAD CELL (5 kg)         |
   +====================o========================+
                        |  Vertical Tension Link
                        |
               +--------o--------+
               |  MCO-1D AIRFRAME|  (Wings Flapping Horizontally)
               |  (Rigid Mount)  |
               +-----------------+
```

### 2.2 Test Point Matrix

| Point | Commanded Frequency | Target Amplitude | Dwell Time | Pass Criteria (Minimum Thrust) | Max Allowed Current |
| :---: | :---: | :---: | :---: | :---: | :---: |
| **P1** | $1.0\text{ Hz}$ | $\pm 60.0^\circ$ | $30\text{ s}$ | $T_{\text{static}} \ge 8.0\text{ N}$ | $< 6.0\text{ A}$ |
| **P2** | $1.5\text{ Hz}$ | $\pm 70.0^\circ$ | $30\text{ s}$ | $T_{\text{static}} \ge 16.0\text{ N}$ | $< 12.0\text{ A}$ |
| **P3** | $2.0\text{ Hz}$ | $\pm 80.0^\circ$ | $30\text{ s}$ | $T_{\text{static}} \ge 24.0\text{ N}$ | $< 18.0\text{ A}$ |
| **P4** | $2.5\text{ Hz}$ | $\pm 80.0^\circ$ | $45\text{ s}$ | $T_{\text{static}} \ge 31.0\text{ N}$ | $< 25.0\text{ A}$ |
| **P5** | $3.0\text{ Hz}$ | $\pm 80.0^\circ$ | $60\text{ s}$ | $T_{\text{static}} \ge 37.0\text{ N}$ | $< 32.0\text{ A}$ |
| **P6** | $3.5\text{ Hz}$ | $\pm 70.0^\circ$ | $20\text{ s}$ | $T_{\text{static}} \ge 42.0\text{ N}$ | $< 38.0\text{ A}$ |

### 2.3 Automated Execution
Run the automated TC-01 test sequencer:
```bash
python3 firmware/mco_test_cards.py --tc 1
```
Or execute remotely via Ground Station:
```json
{"cmd": "tc01_start"}
```

---

## 3. Test Card TC-02: Tethered Lift & Hover Characterization

### 3.1 Objective & Setup
Establish vertical force balance and attitude stabilization authority while secured by a low-friction vertical guide cable.

* **Guide Wire**: 4 mm steel cable anchored vertically between ground stake and overhead gantry with slip-ring collar attached to aircraft CG.
* **Top Limit Bungee**: 1.5 m elastic arrestor cord preventing uncontrolled climb above 2.5 m AGL.

### 3.2 Evaluation Criteria
* **Equilibrium Discovery**: Identify the minimum frequency where aircraft becomes weightless on the tether ($T / W = 1.00$).
* **Roll / Pitch Control Authority**: Inject $\pm 5^\circ$ step stick commands; verify pitch servo differential produces $\ge 15^\circ/\text{s}$ pitch rate and roll amplitude differential produces $\ge 20^\circ/\text{s}$ roll rate.

---

## 4. Test Card TC-03: Moving Vehicle Advance-Ratio Sweep

### 4.1 Objective
Map the aerodynamic thrust and lift polars ($C_T$ and $C_L$) as a function of forward airspeed $V_\infty$ ($0$ to $14\text{ m/s}$) and advance ratio $J$:

$$J = \frac{V_\infty}{2 \cdot f \cdot b}$$

Where $b = 1.584\text{ m}$ (total span) and $f$ is flapping frequency.

### 4.2 Test Vehicle Rigging
1. Mount the aircraft on an elevated 1.5 m rigid pylon above the cab of an open-bed pickup truck in zero-wind atmospheric conditions ($< 1.0\text{ m/s}$ ambient wind).
2. The pylon incorporates a 6-axis load cell measuring lift ($L$), drag/thrust ($F_x$), and pitching moment ($M_y$).
3. The truck accelerates along a straight runway in steady $2.0\text{ m/s}$ increments:
   * **Run 1 ($4.0\text{ m/s}$)**: Pre-stall ground roll aerodynamics.
   * **Run 2 ($6.2\text{ m/s}$)**: Takeoff rotation speed; verify unstick lift $L \ge 56.0\text{ N}$.
   * **Run 3 ($9.0\text{ m/s}$)**: Design cruise speed; verify $L \ge 37\text{ N}$ and positive thrust margin.
   * **Run 4 ($12.0\text{ m/s}$)**: Fast cruise speed.
   * **Run 5 ($14.0\text{ m/s}$)**: Forward-sweep sprint dart configuration.

---

## 5. Test Card TC-04: First Free Flight STOL Envelope Expansion

### 5.1 Pre-Requisites
1. TC-01, TC-02, and TC-03 certified complete with zero open anomalies.
2. Weight and balance certified with battery sled locked at **250 mm (26.5% MAC)**.
3. Ambient surface winds $< 3.0\text{ m/s}$ aligned within $15^\circ$ of runway heading.

### 5.2 Flight Profile & Maneuver Sequence

```
Profile Timeline:
  [0:00] Line up on runway; advance lever forward to 85%–90% (Takeoff Thrust)
  [0:04] Unstick at 6.2 m/s; climb at 9.0 m/s to 15 m AGL
  [0:15] Level off at 15 m; retard lever to 45%–55% (Trim Detent / Cruise)
  [0:30] Evaluate pitch/roll handling qualities; hold level figure-8
  [1:30] Enter downwind; retard lever to 30%–35% (Glide Approach)
  [1:50] Final approach at 7.5 m/s; flare at 1.5 m AGL by retarding lever to 5%–10% (Airbrake Flare)
  [2:00] Touchdown on skids; full airbrake / stop (Lever 0%)
```

### 5.3 Post-Flight De-Brief & Data Archival
Immediately upon aircraft retrieval:
1. Run Ground Station debrief:
   ```bash
   python3 -c "import mco_wave_library as w; w._gs_execute_cmd({'cmd':'alc_debrief'})"
   ```
2. Inspect Dyneema drive lines for fuzzing or micro-wear using 10× optical loupe.
3. Download 50 Hz binary flight log from `/var/log/mco_flight.log` for aerodynamic parameter estimation.
