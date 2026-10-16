# MCO-1D Mark 1 — Builder As-Built Measurement & Calibration Sheet

**Document Designation**: `MCO-1D-CALIB-MK1-R2`  
**Classification**: Formal Production Handoff Manifest — Mark 1 Technical Baseline  
**PDF Printable Companion**: [`MCO-1D_Mark1_Builder_Calibration_Sheet.pdf`](MCO-1D_Mark1_Builder_Calibration_Sheet.pdf)  
**Latest Mirror**: [`../latest/MCO-1D_Mark1_Builder_Calibration_Sheet.pdf`](../latest/MCO-1D_Mark1_Builder_Calibration_Sheet.pdf)  

---

## 1. Airframe Identification & Shop-Floor Metadata

| Airframe S/N | Build Date | Shop Location | Lead Airframe Builder | Scale S/N & Cal Date | Caliper / Rule S/N |
| :---: | :---: | :---: | :---: | :---: | :---: |
| **MCO-1D-001** | `____ / ____ / 2026` | `____________________` | `____________________` | `____________________` | `____________________` |

---

## 2. Airframe Mass, Wheel Reactions & Longitudinal Center of Gravity

All items below must be measured on the completed, flight-ready airframe with 6S 2200mAh LiPo and all 8 wings installed. Flight software dynamically recalculates aerodynamic lift demands, stall speed floors ($V_s \propto \sqrt{m}$), and Boyd $P_s$ climb envelopes directly from these inputs.

| Param ID | Parameter Name | Baseline / Nominal Target | Tolerance / Limits | As-Built Measured Value | Firmware Coupling & Physical Action |
| :---: | :--- | :---: | :---: | :---: | :--- |
| **M-01** | **All-Up Flying AUW**<br/>*All 8 wings + 6S pack installed* | **$3,642.0\text{ g}$ ($3.642\text{ kg}$)**<br/>*(8.03 lb certified baseline)* | Max MTOW: **$< 4,000\text{ g}$**<br/>Target: $3,550\text{--}3,750\text{ g}$ | `[                    ] g` | **PRIMARY CALIBRATION GATE**: Recalculates stall speed floor $V_s \propto \sqrt{m}$, required lift $L = mg$, Boyd $P_s$, and hover $T/W$ ratio ($1.44:1$ nominal). |
| **M-02** | **3-Point Landing Gear Tare**<br/>*Calibrated scales under wheels* | Tailskid: $\approx 10\%$ ($\approx 360\text{ g}$)<br/>Mains: $\approx 45\%$ ea ($\approx 1,640\text{ g}$ ea) | Port vs Stbd balance<br/>within **$\pm 20\text{ g}$** | Tail: `[        ] g`<br/>Port: `[        ] g`<br/>Stbd: `[        ] g` | Confirms ground stability against tail tipping; verifies transverse lateral mass symmetry across gear saddle. |
| **M-03** | **Longitudinal CG Station**<br/>*Spine knife-edge balance* | **$352.6\text{ mm}$ aft of datum**<br/>*(Sled Hole 3: 320mm arm)* | $352.6 \pm 5.0\text{ mm}$<br/>*(Safe: 338–385 mm)* | `[                    ] mm` | Verifies static margin ($+48.3\text{ mm}$ / $+33.6\%$ MAC forward of neutral point $X_{\text{NP}} = 400.9\text{ mm}$) for positive pitch stability. |

---

## 3. Wing Root Collar Geometry & Longitudinal Cascade Stagger (Central Spine)

Measure collar bearing centers along the 700 mm carbon spine tube from the fuselage nose tip (Datum 0.0 mm). These spatial relationships govern dynamic wingtip clashing, inter-wing sweep clearances, and wake slipstream capture.

| Param ID | Geometric Feature | Nominal Design Target | Tolerance | As-Built Measured Value | Governing Physics & Impact |
| :---: | :--- | :---: | :---: | :---: | :--- |
| **M-04** | **Root Lateral Spacing**<br/>*($2 \cdot Y_{\text{root}}$ across spine)* | **$70.0\text{ mm}$ bearing centers**<br/>*($Y = \pm 35.0\text{ mm}$ from spine)* | $\pm 1.5\text{ mm}$ | `[                    ] mm` | **CRITICAL TIP CONTACT GEOMETRY**: Governs tip separation $\Delta Y(\theta) = 2(Y_{\text{root}} + (b/2)\cos\theta)$. Sets software stroke clamp $\theta_{\max} \le 85^\circ$. |
| **M-05** | **Station 1 Arm (W1 Fore)**<br/>*Pivot center from nose datum* | **$X_1 = 95.0\text{ mm}$ aft datum**<br/>Waterline: **$Z_1 = +48.0\text{ mm}$** | $X: \pm 1.5\text{ mm}$<br/>$Z: \pm 1.0\text{ mm}$ | $X_1$: `[        ] mm`<br/>$Z_1$: `[        ] mm` | Leading lifting pair arm; governs nose pitching moment and canard dynamic upwash shed onto downstream stations. |
| **M-06** | **Station 2 Arm (W2 Mid-Fore)**<br/>*Pivot center from nose datum* | **$X_2 = 275.0\text{ mm}$ ($\Delta X = 180\text{ mm}$)**<br/>Waterline: **$Z_2 = +16.0\text{ mm}$** | $X: \pm 1.5\text{ mm}$<br/>$Z: \pm 1.0\text{ mm}$ | $X_2$: `[        ] mm`<br/>$Z_2$: `[        ] mm` | Second cascade station; establishes $90^\circ$ metachronal wake capture timing $\tau = \Delta X / V_\infty$ for induced drag reduction. |
| **M-07** | **Station 3 Arm (W3 Mid-Aft)**<br/>*Pivot center from nose datum* | **$X_3 = 455.0\text{ mm}$ ($\Delta X = 180\text{ mm}$)**<br/>Waterline: **$Z_3 = -16.0\text{ mm}$** | $X: \pm 1.5\text{ mm}$<br/>$Z: \pm 1.0\text{ mm}$ | $X_3$: `[        ] mm`<br/>$Z_3$: `[        ] mm` | Third cascade station; vertical drop ($-32\text{ mm}$ step) prevents chord-plane clash during differential sweep maneuvers. |
| **M-08** | **Station 4 Arm (W4 Aft)**<br/>*Pivot center from nose datum* | **$X_4 = 635.0\text{ mm}$ ($\Delta X = 180\text{ mm}$)**<br/>Waterline: **$Z_4 = -48.0\text{ mm}$** | $X: \pm 1.5\text{ mm}$<br/>$Z: \pm 1.0\text{ mm}$ | $X_4$: `[        ] mm`<br/>$Z_4$: `[        ] mm` | Trailing station; downstream slipstream ejector. Governs pitch damping and overall tandem neutral point $X_{\text{NP}} = 400.9\text{ mm}$. |
| **M-09** | **Spar Quarter-Chord Offset**<br/>*Root rib LE to spar center* | **$36.0\text{ mm}$ aft of root LE**<br/>*(Strict 25.0% MAC target)* | $\pm 0.5\text{ mm}$<br/>*(24.5–25.5% MAC)* | `[                    ] mm` | **AERO HINGE MOMENT GATE**: Eliminates aerodynamic moment arms on spar pivot; prevents pitching blowback on B24CLM pitch servos. |

---

## 4. Individual Wing Panel Matrix (Semi-Span, Mass, Neutral Pitch & Twist)

Measure each of the 8 assembled wing panels independently prior to airframe mounting. Ingested into ODrive FOC inertia matrices, roll trim schedules, and aerodynamic drag polar estimators.

| Wing Station | Side | Semi-Span ($b/2$)<br/>*(Pivot to Tip)* | Panel Mass ($m$)<br/>*(Covered Panel)* | Neutral Pitch ($\alpha_0$)<br/>*(@ 1500µs PWM)* | Tip Washout ($\Delta\alpha$)<br/>*(Root vs Tip Twist)* | Tap Frequency<br/>*(Dyneema Line)* |
| :---: | :---: | :---: | :---: | :---: | :---: | :---: |
| **W1 Fore** | Port (L) | `[        ] mm` | `[        ] g` | `[        ] °` | `[        ] °` | `[        ] Hz` |
| **W1 Fore** | Stbd (R) | `[        ] mm` | `[        ] g` | `[        ] °` | `[        ] °` | `[        ] Hz` |
| **W2 Mid-Fore** | Port (L) | `[        ] mm` | `[        ] g` | `[        ] °` | `[        ] °` | `[        ] Hz` |
| **W2 Mid-Fore** | Stbd (R) | `[        ] mm` | `[        ] g` | `[        ] °` | `[        ] °` | `[        ] Hz` |
| **W3 Mid-Aft** | Port (L) | `[        ] mm` | `[        ] g` | `[        ] °` | `[        ] °` | `[        ] Hz` |
| **W3 Mid-Aft** | Stbd (R) | `[        ] mm` | `[        ] g` | `[        ] °` | `[        ] °` | `[        ] Hz` |
| **W4 Aft** | Port (L) | `[        ] mm` | `[        ] g` | `[        ] °` | `[        ] °` | `[        ] Hz` |
| **W4 Aft** | Stbd (R) | `[        ] mm` | `[        ] g` | `[        ] °` | `[        ] °` | `[        ] Hz` |
| **NOMINAL TARGETS** | **Both** | **$1,150.0\text{ mm} \pm 2.0$** | **$69.0\text{ g} \pm 2.5$** | **$0.0^\circ \pm 0.5^\circ$** | **$0.0^\circ\text{ to } -1.5^\circ$** | **$210\text{ Hz}$ ($160\text{ N}$)** |

---

## 5. Drivetrain, Tension, Kinematic Stops & Sensor Zero Calibrations

| Param ID | Subsystem / Mechanism | Baseline Target | Tolerance | As-Built Measured Value | Firmware Safety Action |
| :---: | :--- | :---: | :---: | :---: | :--- |
| **M-10** | **Dyneema Rest Pre-Tension**<br/>*Acoustic tap pitch (150mm span)* | **$160.0\text{ N}$ ($210\text{ Hz}$)**<br/>*post-48h creep settlement* | $160 \pm 10\text{ N}$<br/>($210 \pm 8\text{ Hz}$) | `[                    ] N`<br/>Pitch: `[          ] Hz` | Eliminates slack-side cable derailment across reversals while protecting motor bearing radial life. |
| **M-11** | **Wingtip Dynamic Backlash**<br/>*Mechanical slop at 1.15m tip* | **$< 1.0\text{ mm}$ play at tip**<br/>*(< 0.05° angular slop)* | Max: $1.5\text{ mm}$<br/>Target: $< 0.8\text{ mm}$ | `[                    ] mm` | Ensures precision aerodynamic angle tracking; eliminates aeroelastic flutter trigger at high speed. |
| **M-12** | **Sector Wheel Travel Arc**<br/>*Mechanical stop-to-stop arc* | **$200.0^\circ$ Total Arc**<br/>*($\pm 100.0^\circ$ from neutral)* | $198.0^\circ\text{ to } 202.0^\circ$<br/>Stop sym: $\pm 1.0^\circ$ | `[                    ] deg` | Sets physical stops at $\pm 100^\circ$; flight governor clamps dynamic stroke to $f\cdot\theta_0 \le 91.7^\circ\cdot\text{Hz}$ ($\le 1,800\text{ RPM}$). |
| **M-13** | **Pitot-Static Zero Offset**<br/>*MS4525DO indoor zero reading* | **$0.00\text{ Pa}$ (covered tube)**<br/>*Calm room ambient* | $\pm 2.0\text{ Pa}$<br/>*($\pm 0.02\text{ hPa}$)* | `[                    ] Pa` | Calibrates low-speed rolling median filter before synthetic EKF3 airspeed blending takes over. |

---

## 6. Mandatory Pre-Flight Hardware Assembly Signoff Checklist

```
[ ] 0. Sub-4 kg Flight AUW Verification
       - Weigh complete flight-ready airframe with 6S 2200mAh pack. Confirm < 4,000 g (nominal 3,642 g). Record M-01.

[ ] 1. Dyneema 48-Hour Creep Settlement
       - Cables rested at 200 N for 48h; re-tensioned to 160 N ± 10 N (210 Hz tap pitch); witness marks aligned; shrouds installed.

[ ] 2. Carbon Spar Internal Ferrules & Clamps
       - 6061-T6 ferrules (Ø8.0x40mm) epoxied into spar roots; spar clamp bolts torqued to 2.2 N·m with Loctite 243.

[ ] 3. Motor Thermal Mounts & NACA Duct Cooling
       - 0.5mm FR4/silicone isolator gaskets under all 8 GoolRC motors torqued to 1.8 N·m; canopy NACA ram-air scoop unobstructed.

[ ] 4. Center of Gravity Knife-Edge Balance
       - Battery tray sled pin locked into Hole 3 (320mm arm); airframe balances on knife-edge at 352.6 ± 5.0 mm (Record M-03).

[ ] 5. Dual-Zone BEC Bus Electrical Droop Test
       - Full 16-servo step deflection: Zone A & B rails hold ≥ 7.1V (droop ≤ 0.3V); 4,700µF buffer capacitors verified intact.

[ ] 6. Biomimetic Pterostigma Flutter Balance
       - 1.5g brass slugs verified at y=0.85b, x/c≈0.08 in all 8 wings; chordwise CG forward of elastic axis (≤ 24% chord).

[ ] 7. CAN-FD Termination Bus Resistance
       - DMM measurement across CAN_H and CAN_L reads exactly 60.0 Ω ± 1.5 Ω with two 120-Ohm split termination resistors.

[ ] 8. Dynamic Kinematic Governor Floor Clamping
       - FCC confirms f·θ0 ≤ 91.7°·Hz universal governor clamp active; ODrive velocity trips prevented below 1,800 RPM ceiling.
```

---

## 7. Production Handoff & TC-01 Bench Authorization

```
GATE CERTIFICATION: Mark 1 Physical Calibration & Sensor Zero Handoff
STATUS:             [  ] READY FOR FIRMWARE INJECTION    [  ] RE-WEIGH / ADJUST

LEAD AIRFRAME BUILDER:    ___________________________   DATE: ____ / ____ / 2026
AVIONICS & FIRMWARE LEAD: ___________________________   FIRMWARE COMMIT: _________
FLIGHT TEST DIRECTOR:     ___________________________   TC-01 APPROVAL: [  ] APPROVED   [  ] HOLD
```
