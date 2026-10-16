# MCO-1D Adversarial Flight Envelope & Performance Audit: S1223 vs SD7062

This document provides a **skeptical, unvarnished adversarial engineering audit** comparing the flight envelope, power requirements, battery endurance, cross-country range, and Boyd energy-maneuverability of the **lightened MCO-1D airframe** equipped with:
1. **Lightened 3D-Printed Selig S1223 Wings** ($m = 3.731\text{ kg}$, $W = 36.59\text{ N}$)
2. **Lightened 3D-Printed Selig-Donovan SD7062 Wings** ($m = 4.087\text{ kg}$, $W = 40.08\text{ N}$)

---

## 1. Adversarial Performance Comparison Register

All figures below are computed with **zero optimism**, incorporating low-Reynolds laminar separation bubbles, aeroelastic spar washout under pitching moment ($C_{m0}$), trim drag penalties, off-design flapping propulsive efficiency rolloff ($\eta_{\text{sys}} \approx 0.38\text{--}0.58$), hotel electronics power ($18.5\text{ W}$), and conservative $80\%$ depth-of-discharge on the single 6S 2200mAh LiPo battery ($E_{\text{usable}} = 39.07\text{ Wh} = 140,659\text{ J}$).

| Performance Dimension | Lightened S1223 ($3.731\text{ kg}$) | Lightened SD7062 ($4.087\text{ kg}$) | Adversarial Delta / Reality Check |
| :--- | :---: | :---: | :--- |
| **Clean Stall Speed ($V_{\text{stall}}$)** | **$5.28\text{ m/s}$ ($10.3\text{ kt}$)** | $6.22\text{ m/s}$ ($12.1\text{ kt}$) | **S1223 wins by $0.94\text{ m/s}$ ($-15\%$ lower stall speed)**. Allows $3.2\text{ m}$ grass takeoff roll vs $4.5\text{ m}$. |
| **Best Loiter / Endurance Speed** | $6.23\text{ m/s}$ ($12.1\text{ kt}$) | $7.18\text{ m/s}$ ($14.0\text{ kt}$) | S1223 loiters slower, keeping turning orbits compact. |
| **Max Flight Time (80% DoD)** | $31.78\text{ min}$ | **$35.51\text{ min}$** | **SD7062 wins by $+3.73\text{ min}$ ($+11.7\%$)**. Low profile drag beats S1223 despite $356\text{ g}$ mass penalty. |
| **Best Range Speed ($V_{\text{range}}$)** | $7.41\text{ m/s}$ ($14.4\text{ kt}$) | $8.96\text{ m/s}$ ($17.4\text{ kt}$) | SD7062 cruises $21\%$ faster at optimal range. |
| **Maximum Cross-Country Range** | $13.08\text{ km}$ ($7.06\text{ NM}$) | **$16.72\text{ km}$ ($9.03\text{ NM}$)** | **SD7062 wins by $+3.64\text{ km}$ ($+27.8\%$ range)**. S1223 suffers high parasite & trim drag. |
| **Peak Aerodynamic Glide Ratio ($L/D$)**| $7.77 : 1$ | **$11.00 : 1$** | **SD7062 glides $41\%$ farther dead-stick**. S1223 separation bubble kills unpowered glide. |
| **Max Continuous Cruise ($350\text{ W}$ Limit)**| $14.03\text{ m/s}$ ($27.3\text{ kt}$) | **$18.07\text{ m/s}$ ($35.1\text{ kt}$)** | **SD7062 wins by $+4.04\text{ m/s}$ ($+28.8\%$)** before hitting motor continuous thermal limit. |
| **Sprint / Never-Exceed Speed ($V_{ne}$)**| $18.00\text{ m/s}$ ($35.0\text{ kt}$) | **$28.00\text{ m/s}$ ($54.4\text{ kt}$)** | S1223 hits a flutter wall at $18\text{ m/s}$; SD7062 can safely dive and dash to $28\text{ m/s}$. |
| **Peak Specific Excess Power ($P_s$)** | **$+4.69\text{ m/s}$ ($923\text{ fpm}$)** | $+4.39\text{ m/s}$ ($864\text{ fpm}$) | **S1223 climbs faster at low speed** ($7\text{ m/s}$) due to lighter weight, but $P_s$ collapses above $14\text{ m/s}$. |
| **Minimum Turn Radius ($2.0g$)** | **$2.36\text{ m}$ ($7.7\text{ ft}$)** | $3.28\text{ m}$ ($10.8\text{ ft}$)** | **S1223 turns $39\%$ tighter**. Incredible low-speed bird agility. |
| **Sustained Turn Rate @ $9\text{ m/s}$ ($2.0g$)**| $108.1^\circ/\text{s}$ | $108.1^\circ/\text{s}$ | Tied (kinematically governed by $2.0g$ bank load factor). |

---

## 2. Graphical Performance Audit

The high-resolution 6-panel performance audit is visualized below:

![MCO-1D Flight Envelope Comparison](file:///Users/drbell/.gemini/antigravity/brain/6213fcb6-5e21-4842-8745-8b4c5b294586/s1223_vs_sd7062_flight_envelope_audit.png)

### Key Graphical Insights:
1. **Panel 1 (Power Demand Curve)**: S1223 has a steeper "U-shaped" power curve. Beyond $10\text{ m/s}$, S1223 profile drag skyrockets, demanding $350\text{ W}$ at just $14.0\text{ m/s}$. SD7062 has a broad, flat power bucket from $7\text{--}18\text{ m/s}$.
2. **Panel 2 & 3 (Range & Endurance)**: Despite being $356\text{ g}$ heavier, SD7062 outperforms S1223 in both endurance ($35.5\text{ min}$ vs $31.8\text{ min}$) and cross-country distance ($16.7\text{ km}$ vs $13.1\text{ km}$).
3. **Panel 4 (L/D Polar)**: S1223 peaks at a modest $7.8 : 1$ around $7.5\text{ m/s}$ and degrades rapidly. SD7062 achieves a clean $11.0 : 1$ glide ratio.
4. **Panel 5 (Boyd Energy-Maneuverability $P_s$)**: S1223 has superior zoom-climb authority below $10\text{ m/s}$, but goes energy-negative ($P_s < 0$) above $14\text{ m/s}$. SD7062 maintains positive specific energy up to $18\text{ m/s}$.
5. **Panel 6 (V-n Maneuvering Diagram)**: S1223 corners within a tiny $2.36\text{ m}$ radius but is red-lined at $18\text{ m/s}$ ($V_{ne}$). SD7062 unlocks a high-speed flight corridor from $18\text{--}28\text{ m/s}$.

---

## 3. Adversarial Engineering Breakdown

### Why S1223 Wins in the Low-Speed & STOL Domain
1. **$15\%$ Lower Stall Speed ($5.28\text{ m/s}$ vs $6.22\text{ m/s}$)**: S1223's deep camber ($8.1\%$) creates massive circulation even at gentle flapping amplitudes.
2. **Micro-Radius Cornering ($2.36\text{ m}$ Turn Radius)**: S1223 can bank at $60^\circ$ and complete a $360^\circ$ circle inside a $15\text{ ft}$ cylinder.
3. **Low Flapping Inertia ($J_{\text{wing}}$)**: S1223's lighter printed weight ($80.5\text{ g}$ vs $102.5\text{ g}$) reduces motor reversal current spikes by $\sim 22\%$.

### Why S1223 Suffers in High-Speed Cruise & Range
1. **Severe Pitching Moment Tuck ($C_{m0} = -0.145$)**: S1223 exerts a strong nose-down twisting couple on the pultruded ⌀10mm carbon spar. The spar twists by $-3^\circ\dots -6^\circ$ at the wingtip, destroying spanwise lift distribution and adding trim drag.
2. **Low-Reynolds Drag Divergence**: S1223's high camber traps a thick laminar separation bubble on the undercambered lower surface at $Re < 100,000$. Above $10\text{ m/s}$, drag diverges exponentially.
3. **$18\text{ m/s}$ Structural Flutter Ceiling**: S1223 cannot safely execute high-speed dive-brakes or airshow demonstrations.

### Why SD7062 Wins in Cruise, Range & High-Speed Dash
1. **Reflexed Profile with Zero Pitching Moment ($C_{m0} = -0.018$)**: The wing does not twist the spar under aerodynamic speed, eliminating trim drag and control cross-coupling.
2. **$27.8\%$ Greater Range ($16.7\text{ km}$)**: SD7062's low profile drag ($C_{D0} = 0.0185$) preserves battery energy across all cruise speeds.
3. **$28\text{ m/s}$ High-Speed Envelope**: SD7062 can sprint at $24\text{ m/s}$ ($53.7\text{ mph}$) and dive up to $28\text{ m/s}$ ($62.6\text{ mph}$), fulfilling the high-speed airshow demonstration role requested in the maneuverability discussions.

---

## 4. Final Recommendation

* **For First Flights, Hover/STOL Validation, and Low-Speed Soaring**: Build the **Lightened 3D-Printed S1223** ($3.731\text{ kg}$). It gives the safest margin against stall, the lowest takeoff ground roll ($3.2\text{ m}$), and the gentlest touchdown velocities.
* **For Cross-Country Patrol, High-Speed Dash, and Aerobatic Airshow Demonstrations**: Transition to the **SD7062** ($4.087\text{ kg}$). It delivers $+28\%$ range, $+12\%$ endurance, and expands top speed from $31\text{ mph}$ to over $54\text{ mph}$.
