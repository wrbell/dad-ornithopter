# MCO-1D Mark 1 — Flight Envelope Specification & Performance Corridors

**Aircraft Title**: MCO-1D Mark 1 Metachronal Cascade Ornithopter  
**Demonstrator Program**: MCO-1A Search-and-Rescue (SAR) Subscale Flight Demonstrator  
**Document Designation**: MCO-1D-ENV-MK1  
**Classification**: Aerodynamics & Flight Mechanics Engineering Specification — Revision Mark 1 (September 2026)  
**Target Hardware Baseline**: 8-Wing Cascade, 10:1 Capstan Drivetrain, 5.687 kg MTOW  

---

## 1. What is a Flight Envelope?

In aeronautical engineering, a **Flight Envelope** (also termed a *flight operating envelope*, *service envelope*, or *$V$-$n$ diagram*) defines the precise operational boundaries within which an aircraft can be operated safely, stably, and effectively without:
1. **Aerodynamic Stall**: Exceeding the maximum usable lift coefficient ($C_{L,\max}$).
2. **Structural Damage or Failure**: Exceeding positive or negative limit load factors ($n_{\text{limit}}$).
3. **Control Loss or Control Reversal**: Exceeding control actuator torque, hinge moment ceilings, or dynamic slew limits.
4. **Thermal or Electrical Overload**: Exceeding continuous inverter FET or motor winding temperature clamps.
5. **Aeroelastic Divergence or Flutter**: Sparing twisting or flapping divergence before the never-exceed speed ($V_{NE}$).

### Standard Forms of Presentation in Aeronautics
A complete aircraft flight envelope is traditionally presented in **two complementary engineering diagrams**:

1. **The $V$-$n$ Diagram (Velocity–Load Factor Envelope)**:
   * **Axes**: True Airspeed ($V$ on the horizontal axis) vs Load Factor ($n = L/W$ on the vertical axis).
   * **Stall Boundaries**: Quadratic parabolic curves ($n \propto V^2$) defined by $n_{\max} = \frac{\frac{1}{2}\rho V^2 S C_{L,\max}}{W}$.
   * **Structural Boundaries**: Horizontal limit load factor cutoffs ($+n_{\text{limit}}$ and $-n_{\text{limit}}$).
   * **Maneuver / Corner Speed ($V_A$)**: The intersection point where the maximum lift curve meets the positive structural limit load.
   * **Dive / Never-Exceed Boundary ($V_{NE}$)**: Vertical high-speed clamp beyond which aeroelastic flutter or motor velocity limits occur.

2. **The Altitude vs Airspeed Operational Corridor (Service Envelope)**:
   * **Axes**: True Airspeed ($V_\infty$ on the horizontal axis) vs Flight Altitude ($h$ MSL on the vertical axis).
   * **Low-Speed Boundary**: Leftward stall boundary $V_s(h)$, which expands toward higher speeds as air density $\rho(h)$ decreases with altitude.
   * **High-Speed Boundary**: Rightward thrust/power-available ceiling, where maximum motor shaft output balances total aerodynamic drag.
   * **Ceiling Boundaries**: Absolute ceiling (zero climb rate) and Service ceiling ($V_z = 0.5\,\text{m/s}$ or $100\,\text{fpm}$).

---

## 2. Why the MCO-1D Ornithopter Envelope is Unique

A conventional fixed-wing airplane possesses a closed $V$-$n$ envelope that terminates at its stall speed (e.g. $V_s = 9.38\,\text{m/s}$), below which it **cannot maintain level flight ($n=1.0g$)**.

The **MCO-1D Mark 1**, however, possesses a revolutionary **Expanded Multi-Modal Flight Envelope**:

```
      Load Factor (n)
       3.0 |
           |                                             +2.2g Limit Load Line
       2.2 |-------------------------------------------+-------------------+
           |                                          /                    |
           |                                         /                     |
       1.35|-------+                                /                      |
           | Hover |                               /                       |
       1.0 |===+===|==============================*========================|  1.0g Cruise
           |   |   |        Active Stall Curve   /   Passive Stall Curve   |
       0.0 +---+---+----------------------------+--------------------------+  0.0g Free-Fall
           0   1   2   3   4   5   6   7   8   9   10  11  12  13  14  15  16  Airspeed (m/s)
           |<-- Hover / VTOL -->|<-- STOL & Cruise -->|<-- Fast Transit -->|
```

1. **Zero-Speed Active Hover Capability ($0 \le V \le 3.0\,\text{m/s}$)**:
   * By employing high-stroke Weis-Fogh clap-and-fling flapping (`H-64`, `H-66`), the aircraft generates $1.0g$ equilibrium ($55.77\,\text{N}$) at **zero forward airspeed**!
   * The envelope extends all the way to the origin ($V = 0.0\,\text{m/s}$), providing VTOL and deep hover authority.
2. **Dynamic Unsteady Lift Enhancement ($C_{L,\text{eff}} = 1.85$)**:
   * Flapping induces spanwise dynamic stall delay and leading-edge vortex (LEV) attachment, expanding the active lift boundary far beyond the passive clean wing limit ($C_L = 1.45$).
3. **Passive Glide Sub-Corridor ($V \ge 11.0\,\text{m/s}$)**:
   * If all 8 motors cease flapping, the aircraft transitions into a conventional fixed-wing glider, requiring $V \ge 11.0\,\text{m/s}$ for unpowered 1g level flight.

---

## 3. Authoritative Flight Envelope Limits & Corner Speeds

The operational limits are derived from the grounded physical parameters ($5.367\,\text{kg}$ MTOW with 8x GoolRC 5010-14 360KV motors, $S = 0.7124\,\text{m}^2$, 10:1 capstan reduction, $10\,\text{mm}$ carbon spar):

| Envelope Parameter | Symbol | Metric Value | Imperial / Standard | Engineering Derivation & Governing Limit |
| :--- | :---: | :---: | :---: | :--- |
| **Maximum Takeoff Weight** | MTOW | **$5.367\,\text{kg}$** | $11.83\,\text{lb}$ | Flight-ready airframe with 8x GoolRC 5010 motors (320g mass saving vs heavy commercial option). |
| **All-Up Weight** | $W$ | **$52.63\,\text{N}$** | $11.83\,\text{lbf}$ | $W = m \cdot g_0 = 5.367 \times 9.80665\,\text{m/s}^2$ (down from $55.77\,\text{N}$). |
| **Total Projected Wing Area** | $S$ | $0.7124\,\text{m}^2$ | $7.67\,\text{ft}^2$ | 8 tandem wings ($120\,\text{mm}$ chord, spans $772\text{--}712\,\text{mm}$). |
| **Wing Loading** | $W/S$ | **$73.88\,\text{N/m}^2$** | **$1.54\,\text{lb/ft}^2$** | Light wing loading ($7.53\,\text{kg/m}^2$, reduced from $7.98\,\text{kg/m}^2$). |
| **Zero-Airspeed Hover** | $V_{\text{hover}}$ | **$0.0\,\text{m/s}$** | $0.0\,\text{mph}$ | Weis-Fogh active flapping ($f = 1.95\,\text{Hz}$, stroke $\pm 75^\circ$, $8.3\%$ lower power required). |
| **STOL Unstick Rotation Speed** | $V_{\text{unstick}}$ | **$6.02\,\text{m/s}$** | $13.47\,\text{mph}$ | Active flapping with Ground Effect image vortex ($C_{L,\text{eff}} \approx 2.15$, down from $6.20\,\text{m/s}$). |
| **Active Flapping 1g Stall** | $V_{s,\text{active}}$ | **$8.07\,\text{m/s}$** | $18.05\,\text{mph}$ | Minimum steady airspeed supported in free-stream active flapping ($C_{L,\text{eff}} = 1.85$, down from $8.30\,\text{m/s}$). |
| **Passive Glide 1g Stall** | $V_{s,\text{passive}}$ | **$9.12\,\text{m/s}$** | $20.40\,\text{mph}$ | Unpowered fixed-wing clean aerodynamic stall ($C_{L,\max} = 1.45$, down from $9.38\,\text{m/s}$). |
| **Design Cruise Airspeed** | $V_{\text{cruise}}$ | **$8.75\,\text{m/s}$** | $19.57\,\text{mph}$ | Primary STOL min-power cruise at $f = 2.10\text{--}2.35\,\text{Hz}$ (+10% battery endurance). |
| **Passive 1g Level Glide Speed** | $V_{\text{glide}}$ | **$10.72\,\text{m/s}$** | $23.98\,\text{mph}$ | Airspeed required to sustain level flight in unpowered fixed glide ($C_L = 1.05$, down from $11.02\,\text{m/s}$). |
| **Fast Cruise / Transit Speed** | $V_{\text{fast}}$ | **$12.00\,\text{m/s}$** | $26.84\,\text{mph}$ | Fast patrol speed at $f = 2.60\,\text{Hz}$ with forward wing sweep ($35^\circ$). |
| **Active Maneuvering Speed** | $V_{A,\text{active}}$ | **$11.97\,\text{m/s}$** | $26.78\,\text{mph}$ | Intersection of active stall curve ($C_{L,\text{eff}}=1.85$) and $+2.2g$ structural limit. |
| **Passive Maneuvering Speed** | $V_{A,\text{passive}}$ | **$13.53\,\text{m/s}$** | $30.27\,\text{mph}$ | Intersection of clean stall curve ($C_L=1.45$) and $+2.2g$ structural limit. |
| **Never-Exceed Airspeed** | $V_{NE}$ | **$16.00\,\text{m/s}$** | $35.79\,\text{mph}$ | ODrive $40.0\,\text{rps}$ velocity ceiling on 10:1 capstan & spar aeroelastic divergence. |
| **High-Sweep Sprint Dart (`H-33`)**| $V_{\text{sprint,dart}}$ | **$22.0\text{--}26.0\,\text{m/s}$** | **$49.2\text{--}58.2\,\text{mph}$** | Non-flapping stowed delta dive mode (unloaded motors, reduced profile drag). |
| **Positive Limit Load Factor** | $n_{\text{limit,pos}}$ | **$+2.20g$** | $+2.20g$ | Carbon spar flexural limit ($M_b = 5.06\,\text{N}\cdot\text{m}$, bending stress $87.3\,\text{MPa}$, $SF = 9.1$). |
| **Positive Ultimate Load Factor**| $n_{\text{ult,pos}}$ | **$+3.30g$** | $+3.30g$ | FAR Part 23 $1.5\times$ safety factor multiplier on limit load. |
| **Negative Limit Load Factor** | $n_{\text{limit,neg}}$ | **$-0.80g$** | $-0.80g$ | S1223 reflex inverted flight and push-over limit ($C_{L,\min} = -0.60$). |
| **Service Ceiling ($V_z = 0.5\,\text{m/s}$)**| $h_{\text{service}}$ | **$2,750\,\text{m}$** | $9,020\,\text{ft}$ MSL | Expanded from $2,500\,\text{m}$ due to lower wing loading and reduced hover/climb power. |
| **Absolute Aerodynamic Ceiling** | $h_{\text{absolute}}$ | **$3,450\,\text{m}$** | $11,320\,\text{ft}$ MSL | Maximum altitude where active flapping thrust equals total profile drag. |

---

## 4. Flight Envelope Diagrams & Visual Plots

The high-resolution vector PDF and raster PNG plots generated by [`scripts/plot_flight_envelope.py`](file:///Users/drbell/Code/flight/scripts/plot_flight_envelope.py) are stored in [`docs/mark1/worksheets/`](file:///Users/drbell/Code/flight/docs/mark1/worksheets/):

```
Vector & Image Assets:
  * Complete Master Envelope: docs/mark1/worksheets/MCO-1D_Mark1_Flight_Envelope.png
  * V-n Structural Envelope:  docs/mark1/worksheets/MCO-1D_V_n_Diagram.png
  * Service Corridor:         docs/mark1/worksheets/MCO-1D_Altitude_Airspeed_Envelope.png
```

### 4.1 Master 2-Panel Envelope Diagram
![MCO-1D Mark 1 Master Flight Performance Envelope](file:///Users/drbell/Code/flight/docs/mark1/worksheets/MCO-1D_Mark1_Flight_Envelope.png)

### 4.2 Structural Load Factor ($V$-$n$) Diagram
![MCO-1D Mark 1 V-n Diagram](file:///Users/drbell/Code/flight/docs/mark1/worksheets/MCO-1D_V_n_Diagram.png)

* **Green Box (0 to 3 m/s)**: Active Hover & Weis-Fogh Regime. Enables full $n = 1.0g$ level hold down to zero airspeed.
* **Blue Solid Curve**: Dynamic active flapping stall line ($C_{L,\text{eff}} = 1.85$).
* **Blue Dashed Curve**: Clean passive glide stall line ($C_L = 1.45$).
* **Purple Solid Curve**: Negative stall line ($C_{L,\min} = -0.60$).
* **Red Solid Lines**: $+2.2g$ and $-0.8g$ Structural limit load factors.
* **Red Dotted Line**: $+3.3g$ Ultimate load failure boundary.
* **Dark Red Vertical Line**: $V_{NE} = 16.0\,\text{m/s}$ Never-Exceed Airspeed.

### 4.3 Altitude vs Airspeed Operational Corridor
![MCO-1D Mark 1 Altitude Airspeed Service Corridor](file:///Users/drbell/Code/flight/docs/mark1/worksheets/MCO-1D_Altitude_Airspeed_Envelope.png)

* **Light Blue Shaded Zone**: Primary operational flight corridor from sea level to $2,500\,\text{m}$ ($8,200\,\text{ft}$) MSL.
* **Light Green Zone (0 to 3 m/s, up to 1,500 m)**: Hover and VTOL authority window.
* **Yellow Shaded Zone ($V \ge 11.0$ m/s)**: Dead-stick passive glide safe operating corridor.
* **Red Solid Line**: Service ceiling at $2,500\,\text{m}$ ($8,200\,\text{ft}$) MSL.

---

## 5. Operational Flight Envelope Boundaries for Autopilot & Pilot

To ensure the physical aircraft operates safely within these certified limits, the flight executive software (`firmware/mco_flight_governor.py`) actively enforces the following hard clamps:

```python
# MCO-1D Mark 1 Certified Flight Envelope Clamps
V_MIN_HOVER_MS      = 0.0     # Zero-speed hover capable
V_MIN_CRUISE_MS     = 6.2     # Takeoff unstick floor
V_MAX_CRUISE_MS     = 12.0    # Nominal cruise ceiling
V_NE_AIRSPEED_MS    = 16.0    # Never-Exceed speed (ODrive 40 rps clamp)

N_LIMIT_POSITIVE_G  = 2.20    # Structural limit load (+2.2g)
N_LIMIT_NEGATIVE_G  = -0.80   # Inverted push-over limit (-0.8g)

ALTITUDE_FLOOR_M    = 0.30    # AGL minimum for altitude hold
ALTITUDE_CEILING_M  = 2500.0  # Service ceiling MSL
```

---
*Certified into the Mark 1 Technical Baseline.*
