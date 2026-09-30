# MCO-1D Mark 1 — Master Technical Specification & Architecture

**Aircraft Title**: MCO-1D Mark 1 Metachronal Cascade Ornithopter  
**Demonstrator Program**: MCO-1A Search-and-Rescue (SAR) Subscale Flight Demonstrator  
**Document Designation**: MCO-1D-SPEC-MK1  
**Classification**: Authoritative Engineering Baseline — Revision Mark 1 (September 2026)  

---

## 1. Executive Summary & Mission Profile

The **MCO-1D Mark 1** is an autonomous bio-inspired unmanned aerial vehicle utilizing an eight-wing cascade architecture ($W_1$ to $W_4$ Port and Starboard). By executing a continuous metachronal flapping wave with dynamic wing sweep ($0^\circ$ to $170^\circ$) and independent servo angle-of-attack (AoA) morphing, the Mark 1 eliminates traditional rotary propellers and fixed-wing control surfaces.

The Mark 1 serves as the validated subscale testbed for the crewed **MCO-1A Search-and-Rescue (SAR)** aircraft. It establishes aerodynamic performance, dynamic stability, and structural durability across the flight envelope, with primary mission parameters optimized for **Short Take-Off & Landing (STOL)** forward flight, high-density altitude loiter, and gust-tolerant terrain following.

```mermaid
graph TD
    subgraph Actuation & Kinematics [8 Wings]
        ODRV[8x ODrive S1 CAN-FD] -->|10:1 Capstan Cable Drive| MTR[8x T-Motor MN5008 200KV]
        MTR -->|Stroke Flap ±100 deg (200° Arc)| WING[8x S1223 Carbon/LW-PLA Wings]
        SRV[16x AGFRC IB53BHU Servos] -->|Pitch -35° to +35°, Sweep 0° to 170°| WING
    end

    subgraph Avionics & Computing [FCC]
        RPI[Raspberry Pi 4B FCC] -->|50 Hz Real-Time Executive| INT[Intent Engine / Boyd PI]
        INT --> GOV[mco_flight_governor.py & Soft Recoil Engine]
        GOV --> WL[mco_wave_library.py 232 Biomimetic Hypotheses]
        WL --> ODRV
        WL --> SRV
        HAT[MCP2518FD CAN-FD Hat] <-->|1 Mbps / 8 Mbps| ODRV
        ADC[2x MCP3208 12-Bit ADCs] -->|Servo Feedback 300-3000mV| RPI
        PX4[Pixhawk 4 Mini Autopilot] -->|MAVLink Serial 921600| RPI
    end

    subgraph Mission Sensing
        PITOT[MS4525DO Pitot-Static] --> PX4
        LIDAR[TF-Luna 8m Lidar] --> RPI
        IMU[MPU-6050 + PX4 IMU] --> RPI
        BARO[BMP280 Barometer] --> RPI
    end
```

---

## 2. Airframe Geometry & Cascade Stagger

The airframe comprises four cascade-arranged wing pairs mounted sequentially along a 600 mm rigid 3K carbon-fiber central spine tube ($20\text{ mm OD} \times 18\text{ mm ID}$).

### 2.1 Wing Station Dimensions

All longitudinal arms are measured aft positive from the **fuselage nose tip (Datum 0.0 mm)**. The Mean Aerodynamic Chord (MAC) length for the rectangular wings is **120.0 mm** with LEMAC reference at **280.0 mm aft of datum** (tandem cascade neutral point at $370.5\text{ mm}$).

| Wing Pair | Longitudinal Arm | Root Chord | Semi-Span (b/2) | Total Projected Area | Aspect Ratio | Ribs / Wing |
| :---: | :---: | :---: | :---: | :---: | :---: | :---: |
| **$W_1$ (Fore)** | $138.0\text{ mm}$ | $120.0\text{ mm}$ | $772.0\text{ mm}$ | $0.1853\text{ m}^2$ | 6.43 | 28 port / 28 stbd |
| **$W_2$ (Mid-Fore)** | $276.0\text{ mm}$ | $120.0\text{ mm}$ | $749.0\text{ mm}$ | $0.1798\text{ m}^2$ | 6.24 | 27 port / 27 stbd |
| **$W_3$ (Mid-Aft)** | $414.0\text{ mm}$ | $120.0\text{ mm}$ | $735.0\text{ mm}$ | $0.1764\text{ m}^2$ | 6.13 | 27 port / 27 stbd |
| **$W_4$ (Aft)** | $552.0\text{ mm}$ | $120.0\text{ mm}$ | $712.0\text{ mm}$ | $0.1709\text{ m}^2$ | 5.93 | 26 port / 26 stbd |
| **TOTAL** | — | — | — | **$0.7124\text{ m}^2$** | — | **216 Universal Ribs** |

$$\text{Total Wing Span (Tip-to-Tip, } W_1\text{)} = 2 \times 772\text{ mm} + 40\text{ mm (Fuselage Width)} = 1.584\text{ m}$$

### 2.2 Vertical Staircase Mounting Geometry
To prevent chord-plane collision between adjacent wing pairs during differential sweeping and pitch changes, the 4 wing pairs are mounted on stepped riser brackets providing a **$100.0\text{ mm}$ vertical offset** per pair:
* **$W_1$ Pivot Height**: $+150.0\text{ mm}$ above centerline
* **$W_2$ Pivot Height**: $+50.0\text{ mm}$ above centerline
* **$W_3$ Pivot Height**: $-50.0\text{ mm}$ below centerline
* **$W_4$ Pivot Height**: $-150.0\text{ mm}$ below centerline

---

## 3. Aerodynamics & Wave Kinematics

### 3.1 UIUC S1223 High-Lift Airfoil Profile
The Mark 1 utilizes the **UIUC S1223 high-camber reflexed airfoil** across all 8 wings. 
* Maximum Thickness: $12.1\%$ at $19.8\%$ chord.
* Maximum Camber: $8.1\%$ at $49.0\%$ chord.
* Maximum Clean 3D Lift Coefficient: $C_{L,\max} \approx 1.60$ at $\alpha = 14.0^\circ$ ($Re \approx 73,000$).
* Effective $C_{L,\max}$ with Turbulator Trip Tape ($x/c = 0.15$): **$1.80$**.
* Theoretical 2D Wind Tunnel Maximum (with Gurney flap, $Re > 2.5 \times 10^5$): $2.40$.

```
S1223 Airfoil Profile (120 mm Chord)
   +--***----------------------------------------------------------------+
   |      ***                                                            |
   |         ***                                                         |
   |            *****                                                    |
   |   LE            *****************                                   |
   |  O===============================********************               |
   |   *                                                  ****************  TE
   |    *                                                 |
   +------------------------------------------------------+
   0 mm                                                 120 mm
```

### 3.2 Metachronal Wave Propulsion Physics
The 4 wing pairs are driven with an inter-pair cascade phase delay $\Delta \phi$:

$$\phi_n(t) = 2\pi f t - (n - 1)\Delta \phi, \quad n \in \{1, 2, 3, 4\}$$

* **Nominal Cruise Phase Lag**: $\Delta \phi = 90^\circ$ (quarter-cycle metachronal lag).
* **Flapping Frequency Range**: $f = 1.0\text{ Hz}$ (loiter) to $3.5\text{ Hz}$ (sprint/takeoff).
* **Flap Stroke Amplitude**: $\theta_0 = \pm 100^\circ$ (full $200^\circ$ circular arc) at $f \le 2.2\text{ Hz}$; dynamically derated to $\pm 34^\circ$ at $3.5\text{ Hz}$ to avoid motor current saturation.
* **Wake Capture**: $W_2, W_3, W_4$ intercept the downwash vortex sheet generated by the upstream wing pair, recovering dynamic energy and lowering specific power consumption:

$$\eta_{\text{cascade}} = \frac{T_{\text{net}} \cdot V_\infty}{P_{\text{mech}}} \approx 0.68 - 0.74$$

### 3.3 Aerodynamic Equilibrium & Lift Balance
* **Unpowered Passive Glide**: Equilibrium glide requires aerodynamic lift equal to aircraft weight ($W = 55.77\text{ N}$). At $C_L = 1.05$:

$$V_{\text{glide}} = \sqrt{\frac{2 W}{\rho S_{\text{total}} C_L}} = \sqrt{\frac{2 \times 55.77}{1.225 \times 0.7124 \times 1.05}} = \mathbf{11.03\text{ m/s} \quad (21.4\text{ kt})}$$

* **Powered Cruise ($9.0\text{ m/s}$)**: At $V_\infty = 9.0\text{ m/s}$ ($q_\infty = 49.61\text{ Pa}$), static wing lift is $L_{\text{aero}} = 37.13\text{ N}$ ($66.58\%$ of weight). To maintain level flight, active metachronal flapping must supply the remaining vertical force:

$$F_{z,\text{flap}} = W - L_{\text{aero}} = 55.77 - 37.13 = \mathbf{18.64\text{ N}}$$

via downstroke pitch bias and stroke plane inclination, maintaining positive specific excess power ($P_s > 0$).
* **Stall Speed**: Clean aerodynamic stall occurs at:

$$V_s = \sqrt{\frac{2 W}{\rho S_{\text{total}} C_{L,\max}}} = \sqrt{\frac{2 \times 55.77}{1.225 \times 0.7124 \times 1.60}} = \mathbf{8.94\text{ m/s} \quad (17.4\text{ kt})}$$

### 3.4 Flapping Motor Electrical Power Model (PF = 0.127)
In reciprocating flapping drives, the peak apparent electrical voltamperes calculated from peak phase current:

$$P_{\text{apparent}} = \sum_{i=1}^8 V_{\text{bus}} \cdot I_{\text{peak},i} \approx 8 \times 22.2\text{ V} \times 12.5\text{ A} \approx 2,220\text{ VA}$$

represents oscillating reactive energy exchange rather than continuous heat dissipation. In the MCO-1D, metachronal inter-pair phase staggering (`H-SHUTTLE-1`) recovers back-EMF across the DC bus with $85\%$ shuttle efficiency. Net active electrical work is governed by the grounded flapping power factor:

$$\text{PF}_{\text{flapping}} = 0.127$$
$$\overline{P}_{\text{elec,active}} = \text{PF}_{\text{flapping}} \cdot P_{\text{apparent}} \approx 280.0\text{ W (at 2.5 Hz cruise)}$$

Nominal continuous motor current per phase is $I_{\text{cont}} \approx 3.0\text{ A}$ (well within the $25.0\text{ A}$ continuous thermal rating of the T-Motor MN5008).

### 3.5 $P_s$-Constrained Climb Envelope
To preserve positive battery reserves and prevent motor over-temperature shutdown, unaccelerated steady climb modes enforce strict physical energy bounds:
* **Best Climb Rate (`CLB1_RATE`)**: $V_z \le 2.2\text{ m/s}$ (target $1.85\text{ m/s}$), bounded by continuous electrical draw $P_{\text{cont}} \le 280.0\text{ W}$.
* **Best Climb Gradient (`CLB2_ANGLE`)**: Steady flight-path climb angle $\gamma \le 15.5^\circ$, vertical velocity ceiling $V_z \le 2.1\text{ m/s}$, continuous draw $P_{\text{cont}} \le 280.0\text{ W}$.
* **Kinetic Zoom (`CLB3_ZOOM`)**: Climb angles $> 25.0^\circ$ are tagged strictly as transient kinetic zoom maneuvers converting forward kinetic energy into altitude ($\tau \le 1.5\text{ s}$, $\Delta V = -\sqrt{2g \Delta h}$) with non-continuous power injection.

### 3.6 Standard Barometric Atmospheric Density $\rho(h)$
Dynamic pressure, aerodynamic lift, parasite drag, and Carson's best-range speed dynamically couple with barometric altitude:

$$\rho(h) = \rho_0 \left(1 - \frac{0.0065 \cdot h}{288.15}\right)^{4.256}$$

As density decreases with altitude ($\rho \approx 1.056\text{ kg/m}^3$ at $1500\text{ m}$ MSL), stall speed increases ($V_s(h) = V_{s0}\sqrt{\rho_0/\rho}$), and cruise flap frequency automatically shifts higher via ALC density-altitude compensation (`H-DA-1`..`7`).

### 3.7 Aero Hinge-Moment Derating on Servo Slew Rate
Control surface panel hinge moments scale with dynamic pressure: $M_h(q) = c_h \cdot q \cdot S_{\text{panel}} \cdot c_{\text{mean}}$. The pitch servo effective slew rate ceiling ($\Omega_0 = 1034^\circ/\text{s}$ no-load at 7.4V) is derated aerodynamically:

$$\Omega_{\text{servo\_eff}} = \Omega_0 \cdot \max\left(0.35, 1.0 - \frac{M_h(q)}{M_{\text{stall}}}\right)$$

Predictive lead phase feedforward (`calculate_dynamic_pitch_servo_lead`) adjusts transit timing $\tau_{\text{transit}} = \Delta \theta / \Omega_{\text{servo\_eff}}$ to eliminate aero lag without demanding physically unattainable slew rates at high dynamic pressure.

---

## 4. Weight, Balance & Mass Budget

### 4.1 Certified Mass Breakdown

| Subsystem Component | Quantity | Unit Mass | Subtotal Mass | % of MTOW |
| :--- | :---: | :---: | :---: | :---: |
| **T-Motor MN5008 Flap Motors (Option A)** | 8 | $135.0\text{ g}$ | $1,080.0\text{ g}$ | $19.0\%$ |
| **ODrive S1 CAN-FD Motor Controllers** | 8 | $80.0\text{ g}$ | $640.0\text{ g}$ | $11.3\%$ |
| **AGFRC IB53BHU Servos (Sweep & Pitch)** | 16 | $69.5\text{ g}$ | $1,112.0\text{ g}$ | $19.6\%$ |
| **Tattu 6S 5000 mAh 50C Main LiPo** | 1 | $540.0\text{ g}$ | $540.0\text{ g}$ | $9.5\%$ |
| **Tattu 2S 3000 mAh 45C Servo LiPo** | 1 | $175.0\text{ g}$ | $175.0\text{ g}$ | $3.1\%$ |
| **Airframe Carbon Structure & Wing Assembly** | 8 | — | $1,165.0\text{ g}$ | $20.5\%$ |
| **Avionics, Pi 4B, CAN Hat, Pixhawk, Sensors**| 1 set | — | $202.0\text{ g}$ | $3.6\%$ |
| **Power Distribution, 4700µF Buffer, Wiring** | 1 set | — | $365.0\text{ g}$ | $6.4\%$ |
| **Fasteners, Bearings, Brake Resistors, Misc** | 1 set | — | $408.0\text{ g}$ | $7.0\%$ |
| **TOTAL OPERATIONAL MTOW** | — | — | **$5,687.0\text{ g}$ (5.687 kg)** | **100.0%** |

### 4.2 Longitudinal Center of Gravity & Slide Rail

$$\text{CG Position} = \frac{\sum (m_i \cdot x_i)}{\sum m_i}$$

* **Aerodynamic Center / Safe CG Envelope**: **$15.0\%$ to $35.0\%$ MAC** ($304.8\text{ mm}$ to $337.8\text{ mm}$ aft of datum).
* **Sliding Battery Tray**: Mounts both the 6S main battery and 2S servo battery on an indexed track with quick-pins:
  * **Arm Range**: $230.0\text{ mm}$ to $340.0\text{ mm}$ aft of datum.
  * **Nominal Trim Setting**: **$250.0\text{ mm}$** battery arm.
  * **Resulting CG**: **$323.8\text{ mm}$ aft of datum ($26.5\%$ MAC)**.
  * **Status**: **IN ENVELOPE (SAFE STABILITY MARGIN: $+8.5\%$ MAC forward of neutral point)**.

---

## 5. Drivetrain & Kinematic Transmission

### 5.1 Capstan 10:1 Cable Reduction Ratio (Upgraded Baseline)
* **Design Directive**: Upgraded from $7.0:1$ to **$10.0:1$** to optimize motor torque loading for the 1-meter semispan wings ($J_{\text{wing}} = 0.027\text{ kg}\cdot\text{m}^2$), cutting motor current by $30\%$ and stator copper heating ($I^2 R$) by $51\%$.
* **Motor Pinion Drum Diameter**: $d_{\text{drum}} = 15.4\text{ mm}$ (helical groove, 6061-T6 aluminum) for $R_{\text{sector}} = 77.0\text{ mm}$, or $d_{\text{drum}} = 18.0\text{ mm}$ for $R_{\text{sector}} = 90.0\text{ mm}$.
* **Wing Sector Wheel Pitch Arc Radius**: $R_{\text{sector}} = 77.0\text{ mm}$ (Option A) or $90.0\text{ mm}$ (Option B) (Full $200^\circ$ circular arc providing $\pm 100^\circ$ stroke authority: $100^\circ$ dorsal/up, $100^\circ$ ventral/down, CNC milled from 2.0 mm quasi-isotropic carbon fiber plate).
* **Effective Transmission Ratio**:

$$G = \frac{2 \cdot R_{\text{sector}}}{d_{\text{drum}}} = \frac{154.0}{15.4} = 10.00 : 1 \quad \left(\text{or } \frac{180.0}{18.0} = 10.00 : 1\right)$$

* **Cable Specification**: Dyneema SK99 12-strand braided line ($0.8\text{ mm}$ diameter, 210 kg break load).
* **Cable Rigging**: Figure-eight double-wrapped capstan with dead-end clamping screw; pre-tensioned to $150 - 200\text{ N}$.


### 5.2 Dynamic Flapping Limits
* **ODrive S1 Velocity Limit Setting**: $\text{vel\_limit} = 40.0\text{ turns/s}$ ($251.3\text{ rad/s} = 2,400\text{ RPM}$, ODrive S1 native unit).
* **Motor Phase Current Limits**: Soft limit $25.0\text{ A}$, Hard clamp $35.0\text{ A}$ peak per phase.
* **Dynamic Amplitude Envelope Derating**: Peak inertial torque scales quadratically with frequency ($\tau_{\text{inertial}} = I_{\text{wing}} \theta_0 (2\pi f)^2$). To prevent current saturation and motor stalling above $2.2\text{ Hz}$, the FCC dynamically scales commanded stroke amplitude:
  * $f \le 2.2\text{ Hz}$: Full nominal stroke up to $\theta_0 = \pm 100.0^\circ$ ($200^\circ$ travel)
  * $f = 2.5\text{ Hz}$: Derated to $\theta_0 = \pm 57.0^\circ$ ($I_q \le 25\text{ A}$)
  * $f = 3.5\text{ Hz}$ (Sprint): Derated to $\theta_0 = \pm 34.0^\circ$ ($I_q \le 30\text{ A}$)
* **Dyneema Tension Safety Margin**: Amplitude derating prevents slack-side cable de-tensioning, guaranteeing $T_{\text{slack}} \ge 50\text{ N}$ and preventing cable derailment from capstan drums.

---

## 6. Electrical Bus Architecture & Failsafe Integrity

The Mark 1 architecture enforces strict physical and electrical separation between propulsion and avionics:

```
[6S 5000 mAh 22.2V LiPo] ----> [High-Current PDB Rail] ----> 8x ODrive S1 (30A Fused)
                                                                 |
                                                                 v
                                                            8x MN5008 Motors

[2S 3000 mAh 7.4V LiPo]  ----> [16-Port Servo Bus Rail] ----> 16x AGFRC IB53BHU Servos
                                      |
                               [4,700 µF Low-ESR Buffer]

[Isolated 5V 3A BEC]     ----> [Raspberry Pi 4B & Pixhawk 4 Mini Avionics]
```

1. **Galvanic Isolation**: The 7.4V servo bus and 22.2V motor bus share common logic ground exclusively at the opto-isolated FCC interface, preventing motor inductive kickback from causing servo jitter or flight computer brownouts.
2. **Capacitive Buffering**: A 4,700 µF low-ESR electrolytic capacitor across the 7.4V servo rail absorbs transient current spikes (>40A) during rapid pitch/sweep reversals.
3. **CAN-FD Network**: Two 120-Ohm termination resistors at the bus extremes (Waveshare Hat and ODrive Node #7). Communication runs at 1 Mbps nominal arbitration / 8 Mbps payload bitrate.
4. **Physical Clean-Air Motor Cooling Pathing**: Each of the 8 T-Motor MN5008 stator pods features an isolated, forward-facing ram-air duct radially staggered (+35mm / +55mm) outboard of the fuselage boundary layer. Downstream stations (W2, W3, W4) are physically isolated from upstream motor heat plumes, ensuring continuous clean ambient freestream cooling ($T_\infty$) with rear Bernoulli negative-pressure scavenge extraction.

---

## 7. Wheeled Landing Gear & Amphibious / Human-Rated Lineage

### 7.1 Wheeled Landing Gear Architecture
The Mark 1 is equipped with a 4-strut wide-stance wheeled landing gear designed for robust rough-field ground operations, taxi maneuvers, and leap takeoffs:
* **Wheels**: Du-Bro 2.00" (⌀50.8 mm) low-bounce lightweight foam wheels (BOM `STRUCT-11`).
* **Axles**: ⌀4.0 mm precision stainless steel axles with dual external E-clips (BOM `STRUCT-12`).
* **Struts**: ⌀6.0 mm pultruded carbon-fiber splayed struts bonded into 3D-printed chassis hardpoint sockets (BOM `STRUCT-10`).
* **Ground Clearance**: 115.0 mm fuselage-to-ground clearance, providing a Static Stability Factor (SSF) of 1.00 with tip clearance margins conforming to taxi constraints.
* **Operational Modes**: Supports cracked-throttle conventional rolling taxi, differential steering, rollout braking, and quadrupedal/wheeled ground-coupled vault takeoff (`H-98`).

### 7.2 Scaled Human-Rated Amphibious Aircraft Roadmap
The MCO-1D Mark 1 uncrewed flight testbed functions as the active technology demonstrator for a larger, crewed, amphibious Search-and-Rescue (SAR) aircraft:
* **Passenger Ride Comfort & Heave Damping**: Even on this subscale testbed, passenger comfort modes (`comfort_mode`, `RideComfortBehaviour`, and vertical acceleration limiters) are actively hosted and flight-qualified to validate vibration suppression and heave damping algorithms ahead of human-piloted operations.
* **Sealed Amphibious Hull & Winter Operations**: While the Mark 1 testbed features open motor pods for ground-effect testing, the downstream human-rated production aircraft features a fully hermetic, sealed hull and integrated ski/wheel landing gear. Amphibious, water, and snow takeoff/landing hypotheses (`H-BS-1` through `H-BS-5`, `H-WD-1` through `H-WD-3`, and `H-PREC-3`) are maintained as core active research algorithms to mature autonomous water and snow landing guidance before human fabrication.


