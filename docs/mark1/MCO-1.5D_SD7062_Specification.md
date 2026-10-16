# MCO-1.5D Design Option: SD7062 High-Speed Tandem Wing Specification

**Document Designation**: MCO-1.5D-SPEC-SD7062  
**Authoritative Status**: Active Design Variant Baseline  
**Date**: October 2026  
**Program**: MCO-1D Metachronal Cascade Ornithopter Upgrade Program  

---

## 1. Executive Summary & Purpose

The **MCO-1.5D SD7062 Option** is an authorized high-speed aerodynamic upgrade variant to the baseline MCO-1D Mark 1 airframe. It replaces the high-camber UIUC S1223 airfoil with the **Selig-Donovan SD7062 reflexed airfoil** across all eight wings ($W_1$ to $W_4$ Port and Starboard), directly resolving the high-speed drag divergence and severe pitch-down pitching moment ($C_m = -0.14$) of the original high-lift airfoil.

To balance high-speed dash efficiency with Short Take-Off & Landing (STOL) climb and 3D print durability, the MCO-1.5D option locks in the **102.5 g per-wing panel mass target** ($820.0\text{ g}$ total 3D-printed airframe) utilizing **2.0 mm solid rib bulkheads with the certified four filleted oval lightening holes**.

```mermaid
graph TD
    subgraph MCO-1.5D SD7062 Architecture
        A[Selig-Donovan SD7062 Profile: 165mm Chord] --> B[Monocoque D-Box Front 30% Chord]
        B --> C[Robust 2.0mm Bulkheads with 4x Oval Lightening Holes]
        C --> D[Continuous 3K CF Spar Array: ⌀10mm Main + 2x ⌀2.0mm Secondary]
        D --> E[Oralight Mylar Skin: 25µm Ironed at 135°C]
        E --> F[Flying AUW: 4.306 kg | Single Wing: 102.5g | 8 Wings: 820g]
    end
```

---

## 2. Mass Budget Comparison: S1223 Baseline vs MCO-1.5D SD7062

The table below records the authoritative aircraft all-up flying weight (AUW) transition:

| Aircraft Subsystem | MCO-1D Baseline (Built-Up S1223) | MCO-1.5D Initial (187.5g SD7062) | MCO-1.5D Locked Baseline (102.5g SD7062) |
| :--- | :---: | :---: | :---: |
| **Fuselage, Avionics, Motors, Servos, Battery** | $3,090.0\text{ g}$ | $3,090.0\text{ g}$ | **$3,090.0\text{ g}$** |
| **8x 3D-Printed Wings (ColorFabb LW-PLA)** | $324.0\text{ g}$ *(fragile ribs)* | $1,499.5\text{ g}$ *(6mm bulkheads)* | **$820.0\text{ g}$** *(2mm robust oval bulkheads)* |
| **Carbon Spar Array (8x ⌀10mm + 16x ⌀2mm)** | $296.8\text{ g}$ | $296.8\text{ g}$ | **$296.8\text{ g}$** |
| **Oralight Mylar Covering Film + Ballast** | $100.1\text{ g}$ | $100.1\text{ g}$ | **$100.1\text{ g}$** |
| **TOTAL FLYING ALL-UP WEIGHT (AUW)** | **$3,642.0\text{ g} \ (3.64\text{ kg})$** | **$5,004.0\text{ g} \ (5.00\text{ kg})$** | **$\mathbf{4,306.9\text{ g} \ (4.31\text{ kg} / 9.49\text{ lb})}$** |
| **Structural MTOW Safety Margin** | $+2.508\text{ kg} \ (+40.8\%)$ | $+1.146\text{ kg} \ (+18.6\%)$ | **$+1.843\text{ kg} \ (+30.0\%)$** |

### Per-Wing Panel Mass Breakdown ($102.5\text{ g}$ Target):
- **Root Airfoil Mount (20 mm)**: $12.0\text{ g}$ ($21.8\text{ cm}^3$)
- **4x 25 cm Modular Wing Segments**: $81.0\text{ g}$ ($20.26\text{ g}$ / $36.8\text{ cm}^3$ each)
- **Aerodynamic Smooth Wingtip (125 mm)**: $9.5\text{ g}$ ($17.3\text{ cm}^3$)
- **Single Wing Panel Total**: **$102.5\text{ g}$ in ColorFabb LW-PLA** ($\rho = 0.55\text{ g/cm}^3$)

---

## 3. Flight Envelope Simulation & Aerodynamic Performance

```
========================================================================================
MCO-1.5D SD7062 AERODYNAMIC & STRUCTURAL SIMULATION REPORT
========================================================================================
Aircraft Flying AUW:                  4.306 kg (9.49 lb)
Total Wing Projected Area (S):        1.224 m² (8 wings)
Mean Aerodynamic Chord (MAC):         143.7 mm (165.0 mm root chord)
----------------------------------------------------------------------------------------
1. Low-Speed STOL Envelope:
   • Minimum Clean Stall Speed:       6.13 m/s (11.9 kt)
   • Takeoff Ground Roll (Grass):     4.18 m (True STOL compliance)
   • Minimum Sink Rate (Glide):       0.72 m/s (Glide ratio L/D = 12.5 : 1)

2. High-Speed Dash Envelope:
   • Sustainable Sprint Speed:        24.0 m/s (46.6 kt / 53.7 mph)
   • Never-Exceed Speed (V_ne):       28.0 m/s (54.4 kt / 62.6 mph)
   • Sprint Profile Drag (24 m/s):    9.15 N (vs 23.81 N on S1223 -> 61.6% DRAG REDUCTION)
   • Sprint Mechanical Shaft Power:   219.7 W (vs 571.5 W on S1223 -> Saves 352 W!)
   • Sprint Thermal Duty Cycle:       > 90 seconds continuous (vs < 20s on S1223)

3. Structural Load Factors & Stress Margins:
   • Maneuvering G-Limits:            +2.20g Limit / +3.30g Ultimate
   • Peak Flapping Root Bending:      18.20 N·m (Limit n=2.2g @ 3.5 Hz stroke reversal)
   • ⌀10mm CF Main Spar Bending:      314.0 MPa (CF Yield = 1,500 MPa -> FOS = 4.78x)
   • Rib Bulkhead Compressive Stress: 0.43 MPa (LW-PLA Yield = 12.0 MPa -> FOS = 27.9x)
   • Rib Web Euler Buckling Limit:    96.5 N (Applied load = 10 N -> Buckling FOS = 9.65x)
   • Mylar Membrane Billowing (24m/s):0.88 mm (0.53% chord, within 1.0% aero tolerance)

4. Flapping Drivetrain & Kinematic Dynamics:
   • Flapping Rotational Inertia:     J_wing = 0.0615 kg·m² (40.3% REDUCTION vs 187g build)
   • Peak Motor Current at Reversal:  24.6 A (Down from 41.1 A)
   • Motor Stator I²R Copper Heat:    64.1% REDUCTION in resistive motor heating!
   • 1st Cantilever Bending Mode:     f_h = 7.29 Hz (Clear of 3.5 Hz flapping harmonic)
   • 1st Cantilever Torsion Mode:     f_alpha = 36.10 Hz (f_alpha / f_h = 4.95 > 3.5: IMMUNE TO FLUTTER)
   • Static Aeroelastic Divergence:   V_div = 40.7 m/s (91.0 mph) (Safe margin above V_ne = 28 m/s)
========================================================================================
```

---

## 4. Manufacturing & CAD Architecture

The CAD architecture enforces structural safety and print repeatability:

1. **Monocoque D-Box Leading Edge ($X = 0\text{ to }49.5\text{ mm}$)**:
   - Sliced with a continuous $0.40\text{ mm}$ outer shell and integral vertical shear web at $X = 33.0\text{ mm}$.
   - Welded spar sleeves house the $\varnothing 10.0\text{ mm}$ main carbon tube and $\varnothing 2.0\text{ mm}$ forward alignment rod with a precision $0.25\text{ mm}$ radial slip tolerance.
2. **Robust Rib Bulkheads ($Z = 0, -50, -100, -150, -200, -250\text{ mm}$)**:
   - Spanwise bulkhead thickness: **$2.0\text{ mm}$ solid material** (4 continuous extrusion perimeters).
   - Four filleted oval lightening holes eliminate non-functional core mass while maintaining continuous caps ($\ge 3.7\text{ mm}$) to upper and lower outer contours.
3. **Interlocking Stepped Joint Sockets**:
   - Inboard male lip ($+4.5\text{ mm}$) engages into the outboard female socket ($5.0\text{ mm}$ depth) with $0.30\text{ mm}$ radial slip tolerance and $0.50\text{ mm}$ axial expansion bottoming gap.
4. **Bilateral Symmetry**:
   - Both Port and Starboard components are exported as dedicated mirrored models in [`../../cad/manufactured/`](../../cad/manufactured/), ensuring identical flight qualities and center of gravity symmetry.

---

## 5. Authoritative Engineering Reference Files

- **Specification**: [`MCO-1.5D_SD7062_Specification.md`](MCO-1.5D_SD7062_Specification.md)
- **CAD Generator**: [`../../scripts/cad/generate_sd7062_wing_system.py`](../../scripts/cad/generate_sd7062_wing_system.py)
- **Manufactured 3D Models**: [`../../cad/manufactured/`](../../cad/manufactured/)
- **Visual Previews**: [`../../cad/manufactured/sd7062_bilateral_wings_assembly_preview.png`](../../cad/manufactured/sd7062_bilateral_wings_assembly_preview.png)
