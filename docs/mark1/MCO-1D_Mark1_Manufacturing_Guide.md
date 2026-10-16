# MCO-1D Mark 1 — Airframe Manufacturing & Assembly Manual

**Aircraft Title**: MCO-1D Mark 1 Metachronal Cascade Ornithopter  
**Document Designation**: MCO-1D-MFG-MK1  
**Classification**: Authoritative Manufacturing Baseline — Revision Mark 1 (September 2026)  

---

## 1. Manufacturing Scope & Tooling Requirements

This manual provides the step-by-step procedures for fabricating, assembling, and rigging the **MCO-1D Mark 1** eight-wing airframe. All fabrication must adhere to aerospace composite and rapid-prototyping standards to meet the verified **5.687 kg MTOW** and **26.5% MAC** Center of Gravity specification.

### Required Workshop Equipment & Tooling
* **3D Printer**: Bambu Lab, Prusa, or Voron FDM printer with direct-drive extruder, 0.4 mm hardened steel nozzle, capable of printing active-foaming LW-PLA.
* **Rotary Lathe**: Precision mini-lathe for turning 6061-T6 helical capstan motor drums ($22.0\text{ mm}$ pitch diameter).
* **CNC Router / Waterjet**: For cutting 2.0 mm carbon fiber sector wheels and station brackets (or order via SendCutSend / Xometry using provided DXF assets).
* **Covering Iron**: Temperature-controlled model aviation iron ($100^\circ\text{C}$ to $150^\circ\text{C}$) for heat-shrink Mylar covering film.
* **Tension Gauge**: Inline spring fish scale or digital hanging scale ($0 - 250\text{ N}$) for rigging Dyneema drive cables.
* **Adhesives & Compounds**:
  * Medium-cure 30-minute structural epoxy (West System or Bob Smith Industries).
  * Loctite 243 (medium-strength removable threadlocker for M2/M3 fasteners).
  * CA Glue (Thin and Medium cyanoacrylate with accelerator).

---

## 2. Fabrication of 216 Universal S1223 Wing Ribs

The Mark 1 achieves manufacturing simplicity by standardizing on a **single universal rib design** at a constant $120\text{ mm}$ chord across all 8 wings (28 ribs on $W_1$, 27 on $W_2$, 27 on $W_3$, 26 on $W_4$, Port and Starboard).

```
Universal Rib Features (cad/manufactured/s1223_universal_rib_120mm.stl)
  LE                                                                   TE
   (o)===========#======================================================>
   Main Spar    Secondary Notch                               Trailing Edge
   (10mm Bore)  (3mm Spar)                                    (0.8mm Lip)
```

### 2.1 3D Printing in ColorFabb LW-PLA
1. **Material**: ColorFabb LW-PLA (Lightweight Foaming PLA).
2. **Slicer Configuration**:
   * Extrusion Temperature: $235^\circ\text{C}$ (initiates $2.2\times$ active foaming expansion).
   * Flow Rate: $45\% - 50\%$ (compensates for volumetric foaming).
   * Layer Height: $0.20\text{ mm}$.
   * Perimeters / Walls: 1 perimeter (yields a clean $0.4\text{ mm}$ outer shell).
   * Infill: $0\%$ (hollow structure; rigidity provided by internal spar bores and shear web).
   * Bed Temperature: $55^\circ\text{C}$ on textured PEI sheet.
3. **Weight Inspection**:
   * Target Mass: **$1.50\text{ g} \pm 0.10\text{ g}$** per rib.
   * Total Mass for 216 ribs: $\approx 324\text{ g}$.
   * Reject any rib exceeding $1.70\text{ g}$ due to over-extrusion or inadequate foaming.

### 2.2 Optional CNC / Laser Cutting Profile
For high-volume CNC waterjet or laser cutting from 1.5 mm balsa or 1.0 mm carbon/ply laminate, use [`cad/manufactured/s1223_universal_rib_120mm.dxf`](file:///Users/drbell/Code/flight/cad/manufactured/s1223_universal_rib_120mm.dxf).

---

## 3. Drivetrain Component Fabrication (10:1 Capstan Reduction Upgrade)

> [!IMPORTANT]
> **Engineering Upgrade Directive: 10:1 Drive Ratio Baseline**:
> With the transition to the **1-meter semispan wings** ($2.35\text{ m}$ wingspan, $J_{\text{wing}} \approx 0.027\text{ kg}\cdot\text{m}^2$, $f \approx 3.5\text{ Hz}$), the capstan reduction ratio is upgraded from the original $7.0:1$ to **$10.0:1$**.
> * **Thermal Benefit**: Reduces motor peak current by $30\%$, cutting stator resistive copper heating ($I^2 R$) by **$51\%$**.
> * **Inertia Matching**: Drops reflected wing inertia from $5.5\times$ rotor inertia down to $2.7\times$, dramatically improving ODrive FOC current-loop response and stroke reversal crispness.
> * **Back-EMF Headroom**: At $3.5\text{ Hz}$ flapping, peak motor speed rises to $733\text{ RPM}$ ($12.2\text{ rps}$), utilizing only $3.7\text{ V}$ of back-EMF and leaving $>83\%$ voltage headroom on the 6S $22.2\text{ V}$ bus (well within the ODrive $40\text{ rps}$ velocity clamp).

### 3.1 10:1 Carbon Fiber Sector Wheels
1. **Material**: 2.0 mm 3K quasi-isotropic solid carbon fiber plate.
2. **Manufacturing Options**:
   * **Baseline Option A (Direct Conversion with 15.4 mm Drum)**: Pitch Arc Radius $R = 77.0\text{ mm}$ over $100^\circ$ continuous sweep arc. Cut 8 units from [`cad/manufactured/sector_wheel_7to1_100deg.dxf`](file:///Users/drbell/Code/flight/cad/manufactured/sector_wheel_7to1_100deg.dxf). Paired with $15.4\text{ mm}$ pitch diameter motor drum yields $77.0 / 7.7 = \mathbf{10.0 : 1}$.
   * **Balanced Option B (90 mm Sector + 18.0 mm Drum)**: Pitch Arc Radius $R = 90.0\text{ mm}$ over $100^\circ$ arc ($+13\text{ mm}$ radial expansion, clears all adjacent fuselage stations). Paired with $18.0\text{ mm}$ pitch diameter drum yields $90.0 / 9.0 = \mathbf{10.0 : 1}$.
3. **Features**:
   * Perimeter Cable Channel: 1.0 mm semicircular groove along the rim to guide the 0.8 mm Dyneema line.
   * Central Axle Bore: 5.0 mm reamed bore for EZO F685ZZ flanged ball bearings.
   * Weight: Target $30.0\text{ g}$ (Option A) to $35.0\text{ g}$ (Option B) each.

### 3.2 10:1 Helical Capstan Motor Pinion Drums
1. **Material**: 6061-T6 Aluminum bar stock.
2. **Dimensions**:
   * **For Option A ($R = 77.0\text{ mm}$ Sector)**:
     * Pitch Diameter: **$d_{\text{pitch}} = 15.4\text{ mm}$** ($r = 7.7\text{ mm}$); Outside Flange Diameter: $18.0\text{ mm}$.
     * Radial Hub Wall: $5.2\text{ mm}$ wall thickness around 5.0 mm D-shaft bore (ample meat for M3 set screws).
     * Dyneema Bend Ratio: $15.4\text{ mm} / 0.8\text{ mm} = 19.25 : 1$ (well above the $8:1$ minimum fatigue bend threshold).
   * **For Option B ($R = 90.0\text{ mm}$ Sector)**:
     * Pitch Diameter: **$d_{\text{pitch}} = 18.0\text{ mm}$** ($r = 9.0\text{ mm}$); Outside Flange Diameter: $21.0\text{ mm}$.
     * Radial Hub Wall: $6.5\text{ mm}$ wall thickness around 5.0 mm D-shaft bore.
   * Length: $28.0\text{ mm}$ with 5.0 mm D-shaft motor bore and dual M3 grub screw clamping collars.
   * Helical Groove: Single-start helical groove with 1.2 mm pitch and 0.5 mm radius to seat the Dyneema line without overlap.

---

## 4. Wing Spar Layup & Structural Assembly

### 4.1 Carbon Spar Dimensioning
Cut the 10 mm OD pultruded carbon fiber leading-edge tubes and 2.0 mm / 3.0 mm secondary carbon rods to the exact lengths:

* **$W_1$ Pair (Fore)**: Leading edge spar cut to **$772\text{ mm}$**. 28 ribs spaced at $27.0\text{ mm}$.
* **$W_2$ Pair (Mid-Fore)**: Leading edge spar cut to **$749\text{ mm}$**. 27 ribs spaced at $27.0\text{ mm}$.
* **$W_3$ Pair (Mid-Aft)**: Leading edge spar cut to **$735\text{ mm}$**. 27 ribs spaced at $26.5\text{ mm}$.
* **$W_4$ Pair (Aft)**: Leading edge spar cut to **$712\text{ mm}$**. 26 ribs spaced at $26.5\text{ mm}$.

### 4.2 Assembly Procedure
1. **Spar Sliding & Jigging**: Thread the universal ribs onto the 10 mm leading edge tube and 3.0 mm mid-chord spar. Mount the assembly in a rigid alignment jig to enforce $0.0^\circ$ wash-in / wash-out across the span.
2. **Bonding**: Apply a drop of medium CA to tack each rib in place. Reinforce each rib-to-spar junction with a microballoon-thickened epoxy fillet.
3. **Trailing Edge Strip**: Bond a $0.5\text{ mm} \times 3.0\text{ mm}$ carbon flat strip along the trailing edge notches to maintain razor-sharp trailing-edge definition under flapping aerodynamic loads.
4. **Covering Film Application**:
   * Cut Oralight heat-shrink film to panel span $+ 50\text{ mm}$ allowance.
   * Tack the film to the carbon leading edge tube at $105^\circ\text{C}$.
   * Wrap smoothly over upper camber, tack to the trailing edge strip, and fold under the lower reflex surface.
   * Heat iron to $135^\circ\text{C}$ and shrink evenly from center to tips until drum-tight with no wrinkles.

### 4.3 Biomimetic Pterostigma Leading-Edge Mass Balancing & High-Speed Aft Skeletonization
To suppress aeroelastic torsional flutter at high flapping frequencies ($f > 3.0\text{ Hz}$) and sprint airspeeds ($V > 12\text{ m/s}$), while avoiding heavy rotational inertia ($J_{\text{wing}}$) penalties, each of the 8 wing panels follows a strict mass-balancing and skeletonization protocol:

1. **Biomimetic Pterostigma Tip Slug Ballast (Dragonfly Principle)**:
   * **Ballast Specification**: Use a **$1.5\text{ g} \pm 0.1\text{ g}$ cylindrical brass slug** ($\varnothing 3.0\text{ mm} \times 24\text{ mm}$) or tungsten putty.
   * **Placement**: Insert into the dedicated ballast cavity at $85\%$ span ($y = 0.85b$, approximately $106\text{ mm}$ outboard on the $125\text{ mm}$ `Balanced-125` wingtip), located at $x/c \approx 0.08$ immediately behind the forward carbon spar pin socket.
   * **Retention**: Secure with a drop of medium cyanoacrylate (CA) before sliding the 3D-printed wingtip shell onto the spar pins.
   * **Aeroelastic Function**: Modeled on dragonfly wing morphology (*Norberg 1972*), this shifts the localized wingtip center of gravity forward of the torsional elastic axis ($x_{\text{cg}} < x_{\text{ea}}$). Under upward aeroelastic deflection, the slug exerts a passive nose-down restoring torque, increasing the critical flutter boundary ($V_{\text{flutter}}$) from $16\text{ m/s}$ to well beyond $28\text{ m/s}$ ($V_{ne}$).
   * **Low Inertia Penalty**: At $r = 0.65\text{ m}$, the $1.5\text{ g}$ slug adds only $J_{\text{slug}} = m r^2 \approx 0.00063\text{ kg}\cdot\text{m}^2$ ($<2.5\%$ of total wing inertia $J_{\text{wing}}$), costing less than $0.08\text{ Hz}$ of flapping frequency.

2. **The "Aft Subtraction" Principle (The 5.4 : 1 Lever Arm Rule)**:
   * **Design Directive**: Do **NOT** attempt whole-chord mass balancing by adding heavy lead weights at the wing root or mid-span. Adding $12\text{--}15\text{ g}$ of dead nose ballast per wing would add over $100\text{ g}$ to MTOW and severely penalize flapping motor current ($I_q \propto J_{\text{wing}}$).
   * **The Lever Arm Geometry**:
     * Forward ballast pocket: $x/c \approx 0.08$ (arm to spar = $12\% c$).
     * Trailing-edge structure: $x/c \approx 0.85$ (arm to spar = $65\% c$).
     * Mechanical ratio: $\frac{65\% c}{12\% c} \approx \mathbf{5.4 : 1}$.
   * **Manufacturing Rule**: Removing **$1.0\text{ g}$** of aft plastic or adhesive behind the main spar shifts the chordwise CG forward by the exact same amount as adding **$5.4\text{ g}$** of dead nose ballast—while reducing total airframe weight and flapping inertia!
   * **Shop-Floor Skeletonization Actions**:
     * Print all 216 universal ribs with $0\%$ infill aft of the main spar ($x/c \ge 0.20$), retaining only the single-perimeter truss ribs.
     * Sand down the trailing-edge composite/balsa flat strip to a knife-edge ($<0.4\text{ mm}$ trailing-edge thickness).
     * Apply lightweight Oralight film ($36\text{ g/m}^2$) over the aft $60\%$ of the wing cord; avoid thick adhesives or reinforcement tape on the trailing edge.

3. **High-Speed Sprint Thermal & Battery Duty Cycle Operational Limits (Expanded Faired Baseline)**:
   * **Continuous Fast Cruise**: $V = 20.80\text{ m/s}$ ($46.5\text{ mph}$) operates **$100\%$ unconstrained** with zero time clamp ($P_{\text{shaft}} \approx 400\text{ W}$, motor current $<3.0\text{ A}$ per phase).
   * **Max Sprint Burst Duration**: High-speed dash ($V \ge 24\text{ m/s}$, $f = 3.2\text{ Hz}$, $792\text{ W}$ burst) is clamped to **$\tau_{\text{sprint}} \le 90.0\text{ seconds}$** (expanded from legacy $20.0\text{ s}$ due to $49\%$ drag reduction, 10:1 capstans, and $20\text{ A}$ switching BEC).
   * **Rapid Cooldown Lockout**: Requires an accelerated **$10.0\text{ seconds}$** cruise/loiter cooling period ($f \le 2.2\text{ Hz}$, $\theta_0 \le 18.0^\circ$) before sprint can be re-engaged (reduced from legacy $45.0\text{ s}$ thanks to fresh-air NACA scoops).
   * **Servo Current Ceiling**: Total 7.4V servo bus current is bounded at **$18.0\text{ A}$** as a hardware fail-safe against mechanical cable binding.
   * **Prohibition of Open-Loop Phase-Lead Feedforward**: Software-driven artificial phase-lead advances (Fix 3) are strictly prohibited on stock geared servos to prevent gear tooth stripping, command whiplash, and aeroelastic negative-damping resonance.

### 4.4 Insect Biomechanics Airframe Enhancements (H-89 to H-95 Physical Build Requirements)
To enable the full aerodynamic performance of the Insect Flight Biomechanics Suite (H-89 to H-95), the physical build must address seven critical mechanical, electrical, and structural requirements:

1. **AGFRC IB53BHU Pitch Servo Linkage & Current Buffer (H-89 Pre-Rotation)**:
   - *Slew Rate & Mechanical Backlash*: Rapid trapezoidal pitch pre-rotation ($\Delta t_{\text{lead}} \approx 25\text{ ms}$, slew rate up to $444^\circ/\text{s}$) demands zero-slop mechanical linkage. All carbon tie-rods and ball-links between the AGFRC IB53BHU pitch servos and wing root yokes must be inspected for backlash $< 0.1\text{ mm}$.
   - *Decoupling Capacitor*: Ensure the $4,700\ \mu\text{F}$ 16V low-ESR electrolytic reservoir capacitor is soldered with leads $< 20\text{ mm}$ directly across the 7.4V servo bus board input (`PWR-03`) to absorb inductive kickback during rapid multi-servo reversal.
   - *Travel Limit*: Confirm with `AGF-SPV3` servo programmer that pitch travel limits are clamped to $[-30^\circ, +30^\circ]$ to prevent servo stall against physical root hardstops.

2. **Universal S1223 Rib Micro-Cavity Trapped-Vortex Profiling (H-90 Meganeura Steps)**:
   - *Slicing Specification*: When printing the 216 universal ribs in ColorFabb LW-PLA, use the stepped CAM profile featuring 3 spanwise micro-step cavities ($1.2\text{ mm}$ deep $\times 6.0\text{ mm}$ chord) along the upper surface forward of the main spar ($x/c \approx 0.08\text{--}0.28$).
   - *Covering Application*: The Oralight heat-shrink film must be tacked to the step ridges without bridging the cavity valleys, allowing the stationary recirculation bubble to establish.
   - *Weight Benefit*: Verify that the cavitied rib weighs $\le 2.9\text{ g}$ (saving $\approx 1.4\text{ g}$ per rib compared to solid ribs).

3. **Inboard Trailing-Edge Flexible Pneumatic Jet Skirt (H-91 Butterfly Clap Jet)**:
   - *Location*: Apply exclusively to the inboard 3 rib bays ($y = 0\text{ to }0.35b$) of Stations 1 ($W_1/W_2$) and 2 ($W_3/W_4$).
   - *Material & Bonding*: Bond a $15.0\text{ mm}$ wide strip of $0.2\text{ mm}$ flexible silicone membrane or ripstop nylon extending past the carbon trailing-edge flat.
   - *Pneumatic Sealing*: When dihedral is commanded to $+70^\circ$ (dorsal clap), verify that the silicone skirts make compliant contact, forming an expulsion nozzle pocket directed downward/rearward.

4. **Station 1 Elytron Canard Pivot Holding Rigidity (H-92 Beetle Canard)**:
   - *Station 1 Mechanical Locking*: When Station 1 ($W_1/W_2$) locks stationary at $+35^\circ$ dihedral under aerodynamic climb loads, the ODrive S1 capstan must hold holding position with $< 0.5^\circ$ angular deflection.
   - *Holding Current Test*: Verify on bench that Station 1 holding current does not exceed $12.0\text{ A}$ continuous ($T_{\text{motor}} < 65^\circ\text{C}$) under simulated $15\text{ N}$ aerodynamic lift load.

5. **Pixhawk 4 Mini Vibration Gel Isolation & CAN-FD Shielding (H-93 Coriolis Demodulation)**:
   - *IMU Damping*: Mount the Pixhawk 4 Mini flight controller on 4× Shore 20A silicone gel damping grommets ($9.5\text{ mm} \times 4.0\text{ mm}$) with $1.5\text{ mm}$ uncompressed clearance. This prevents $2.5\text{ Hz}$ and $5.0\text{ Hz}$ airframe flapping harmonics from clipping the internal MEMS accelerometers.
   - *CAN-FD Bus Termination*: Verify with digital multimeter that the CAN-FD twisted pair measures $60.0\ \Omega \pm 1.5\ \Omega$ line-to-line resistance (confirming $120\ \Omega$ termination at both the MCP2518FD Hat and ODrive Node 7).

6. **Outboard Wing Spar Torsional Compliance (H-94 Locust Torsional Wave)**:
   - *Torsional Freedom*: The $10\text{ mm OD} \times 8\text{ mm ID}$ carbon spar must allow the outer $30\%$ span to twist elastically by $\pm 4^\circ$ under dynamic pitch servo excitation without puckering or tearing the Oralight covering film.
   - *Heat-Shrink Relieving*: Apply silicone lubricant dry film to the rib spar-sleeves on the outer 4 ribs before covering shrinking.

7. **Tandem Wing Station Spacing & Slot Convergence Clearance (H-95 Slotted Flap)**:
   - *Station Spacing Tolerance*: Verify distance between Station 1 pivot axis and Station 2 pivot axis is precisely $138.0\text{ mm} \pm 0.5\text{ mm}$ ($s/c = 0.836$).
   - *Convergent Slot Clearance*: At neutral sweep ($60^\circ$), measure the gap between Station 1 trailing-edge carbon flat and Station 2 leading-edge carbon tube. Clearance must be $25.0\text{ mm} \pm 2.0\text{ mm}$ across the entire span to ensure aerodynamically optimal boundary layer blowing without physical interference during $+20^\circ$ phase-lead flapping.

---

## 5. Fuselage Spine & Actuator Station Assembly

### 5.1 Carbon Spine Preparation
1. Take the $20\text{ mm OD} \times 18\text{ mm ID} \times 600\text{ mm}$ carbon tube.
2. Mark Datum 0.0 mm at the forward edge.
3. Precision-drill the four station pivot bracket bolt holes along the spine centerline:
   * **Station 1 ($W_1$)**: $138.0\text{ mm}$ aft of datum.
   * **Station 2 ($W_2$)**: $276.0\text{ mm}$ aft of datum.
   * **Station 3 ($W_3$)**: $414.0\text{ mm}$ aft of datum.
   * **Station 4 ($W_4$)**: $552.0\text{ mm}$ aft of datum.
4. Install aluminum station brackets using M3 Grade 5 titanium bolts and Nyloc nuts.

### 5.2 Motor & Servo Mounting
1. **Motor Installation & Thermal Isolation (HW-08 / PROP-01)**: Mount the 8× **GoolRC 5010-14 360KV** motors to their respective station mounting plates. Between each motor baseplate and the CNC 6061-T6 aluminum mounting plate, install one precision-cut **0.5 mm FR4 / ceramic-filled silicone thermal isolator gasket (`HW-08`)**. Fasten using M3 $\times$ 8 mm Grade 12.9 socket head cap screws with Loctite 243, torqued to $1.8\text{ N}\cdot\text{m}$. This gasket physically interrupts metallic heat conduction from the stator into the carbon spine and electronics pod, forcing dissipated stator heat into the NACA forced-air cooling stream.
2. Mount the 8× AGFRC IB53BHU pitch servos directly to the wing root pivot yokes.
3. Mount the 8× AGFRC IB53BHU sweep servos to the station chassis with carbon tie-rods linking to the wing root collars.
4. **Reinforced Battery Sled Mount**: Mount the sliding carbon battery tray with double shear gussets to support symmetric normal load factor ($\pm 2.20g$ limit, $\pm 3.30g$ ultimate), ensuring zero tray deflection under inverted flight and outside push loops.

### 5.3 Dedicated Fresh-Air Motor Cooling Architecture & Ducting
To ensure that all 8 motors maintain continuous stator temperatures below the $65^\circ\text{C}$ advisory threshold ($80^\circ\text{C}$ critical clamp) during sustained cruise and high-power hovering, the physical installation must enforce dedicated, uncontaminated fresh-air airflow across every motor:

1. **Clean Air Ingestion Principle**:
   - In an 8-wing tandem configuration, downstream motors (Stations 2, 3, and 4) are naturally exposed to the pre-heated downwash and thermal exhaust plumes of upstream motors.
   - *Physical Rule*: Under no circumstances may a downstream motor draw cooling air from the internal fuselage cavity or the turbulent exhaust wake of an upstream motor. Every motor must ingest undisturbed, ambient freestream air.

2. **Ram-Air NACA Inlets & Staggered Outboard Scoops**:
   - Each of the 8 motor station brackets is equipped with a dedicated 3D-printed forward-facing ram-air scoop (printed in lightweight PETG or foaming LW-PLA, mass $\le 3.8\text{ g}$).
   - *Outboard Radial Stagger*: The scoops are offset laterally by $+35\text{ mm}$ (Stations 1 & 3) and $+55\text{ mm}$ (Stations 2 & 4) relative to the spine centerline, staggering their capture areas outside the upstream thermal boundary layer.
   - *Direct Stator Flow*: The internal duct geometry directs incoming ram air directly through the stator winding vents and across the internal rotor magnets.

3. **Negative-Pressure Scavenge Louvers & Thermal Isolation**:
   - Each motor nacelle incorporates rearward-facing $45^\circ$ negative-pressure exhaust louvers on its outboard face.
   - *Bernoulli Scavenging*: Freestream flight airspeed generates localized low static pressure across the louvers, actively drawing heated stator air out of the pod and expelling it cleanly outboard into the wing slipstream.
   - *Chassis Thermal Break*: Install a $0.5\text{ mm}$ silicone/fiberglass thermal isolator gasket between each motor baseplate and the aluminum station bracket, preventing motor stator heat from conducting into the central carbon spine or heating adjacent avionics and CAN-FD wiring.

4. **Integration with Software Balancing**:
   - This physical fresh-air ducting establishes the baseline passive thermal equilibrium ($\Delta T_{\text{fore-aft}} < 3.0^\circ\text{C}$ in forward flight). Firmware engines `H-BUS-3` and `H-CMP-1` serve as active software governors only when ambient temperatures exceed $35^\circ\text{C}$ or during prolonged zero-airspeed hovering.

---

## 6. Dyneema Capstan Rigging & Pre-Tensioning

The Dyneema rope drive transfers reciprocating motor torque to the wing sector wheel with zero backlash.

```
Rigging Schematic:
         +-------------------------------------+
         |         10:1 SECTOR WHEEL           |
         |         (100° Stroke Arc)           |
         +-------o---------------------o-------+
                  \                   /
             Down  \                 /  Up
             Cable  \               /   Cable
                     \             /
                      v           v
                    [ HELICAL CAPSTAN ]
                    [   MOTOR DRUM    ]
```

### 6.1 Cable Rigging Steps
1. **Anchor Pin Setup**: Terminate a 1.2 m length of 0.8 mm Dyneema SK99 line at the top-left anchor hole of the sector wheel using a figure-eight dead-end knot.
2. **Capstan Wrap**: Route the cable down to the helical motor drum. Wrap **3 full turns** clockwise around the drum grooves.
3. **Sector Return**: Continue routing up to the opposite side of the sector wheel rim and pass through the cable tensioning cam.
4. **Pre-Tensioning**:
   * Attach a digital hanging scale to the free cable end.
### 6.1 Cable Rigging Steps
1. **Anchor Pin Setup**: Terminate a 1.2 m length of 0.8 mm Dyneema SK99 line at the top-left anchor hole of the sector wheel using a figure-eight dead-end knot.
2. **Capstan Wrap**: Route the cable down to the helical motor drum. Wrap **3 full turns** clockwise around the drum grooves.
3. **Sector Return**: Continue routing up to the opposite side of the sector wheel rim and pass through the cable tensioning cam.
4. **Pre-Tensioning**:
   * Attach a digital hanging scale to the free cable end.
   * Draw tension to **$160\text{ N} \pm 10\text{ N}$** (verified on scale and by acoustic tap pitch of **$210\text{ Hz}$** across a 150 mm free cable span).
   * Tighten the M3 brass clamping screw on the sector wheel to lock the cable in place.
   * Apply a drop of Loctite 480 flexible cyanoacrylate to the clamp to eliminate micro-slippage.
5. **Install 3D Line Keeper Shroud**: Snap the 3D-printed capstan line keeper shroud (`STRUCT-17`) over the motor pinion drum, verifying $<0.4\text{ mm}$ radial clearance over the drum grooves to physically trap the Dyneema line.
6. **Repeat for all 8 wings**. Verify by manually deflecting each wing $\pm 45^\circ$: there must be zero perceptible backlash, zero slackening of the opposite run, and smooth bearing rotation.

---

## 7. Quick-Pin Sliding Battery Tray Installation

To maintain longitudinal trim across flight regimes, the 6S main battery and 2S servo battery are carried on a single indexed sled.

```
Fuselage Keel Track (230 mm to 340 mm aft of datum):
   Datum 0.0mm (Nose)
   |=====[ Fore Avionics ]====[ 230mm  *  *  *  *  *  *  *  *  *  *  * 340mm ]====> Tail
                              |<- 10mm Pin Pitch ->|
                              Position 250mm = 26.5% MAC (NOMINAL TRIM)
```

1. **Keel Guide Rail**: Mount a $15\text{ mm} \times 15\text{ mm}$ aluminum U-channel along the bottom of the carbon spine from $220\text{ mm}$ to $350\text{ mm}$ aft of datum.
2. **Indexing Holes**: Drill 4.0 mm reamed indexing holes along the rail side wall at **$10.0\text{ mm}$ center-to-center pitch**:
   * Hole 1: $230\text{ mm}$ (Forward penetration setting — $20.5\%$ MAC).
   * **Hole 3: $250\text{ mm}$ (Nominal STOL cruise setting — $26.5\%$ MAC)**.
   * Hole 8: $300\text{ mm}$ (Aft high-lift loiter setting — $30.3\%$ MAC).
   * Hole 12: $340\text{ mm}$ (Max aft flare setting — $33.5\%$ MAC).
3. **Engraving**: Laser-engrave the corresponding `% MAC` labels adjacent to each hole.
4. **Battery Sled**: Mount the 4.0 mm stainless-steel ball-lock quick-release pin through the 3D-printed PETG sled body. Verify that depressing the release button allows smooth sliding and releasing instantly locks into the detent.

### 7.3 Aerodynamic Fairing Package Manufacturing & Installation

The aerodynamic fairing package slashes parasite drag by $49.1\%$ ($f_{\text{par}}$ drops from $0.0550\text{ m}^2$ to $0.0280\text{ m}^2$), unlocking $20.80\text{ m/s}$ ($46.5\text{ mph}$) continuous fast cruise and $26.35\text{ m/s}$ ($58.9\text{ mph}$) dash bursts:

1. **Fuselage Delta Truss Infill Fairing Panels**:
   * **Material**: ColorFabb LW-PLA (Lightweight Foaming PLA), foamed at $235^\circ\text{C}$ (density $\approx 0.45\text{ g/cm}^3$).
   * **Print Settings**: 1 perimeter ($0.4\text{ mm}$ wall), $0.20\text{ mm}$ layer height, $3\%$ gyroid infill for panel shear stability. Total printed mass: **$95.0\text{ g}$**.
   * **Mounting**: Snaps onto the upper spine (⌀20 mm) and lower keel (⌀12 mm) using integrated semi-circular clip channels with 2.5 mm silicone cushioning tape.
   * **Cooling Pathing**: Forward canopy integrates an active $25\text{ mm} \times 15\text{ mm}$ NACA ram-air scoop directing freestream air over the ODrive S1 heatsinks and dual 20A BEC units; aft $40\text{ mm}$ negative-pressure exhaust louvers on the belly tub provide continuous suction scavenging.
   * **Service Access**: Removable canopy hatch secured via 4× ⌀6mm neodymium rare-earth magnets and forward tongue-in-groove tab.

2. **Wing Root Sector & Apparatus Teardrop Nacelles (8×)**:
   * **Material**: ColorFabb LW-PLA shell ($6.0\text{ g}$ each $\times 8 = \mathbf{48.0\text{ g}}$ total).
   * **Internal Geometry**: Encloses the 10:1 capstan sector wheel, Dyneema lines, AGFRC IB53BHU pitch/sweep servos, and root pivot collars.
   * **Spar Flapping Slit & Compliant Wiper Seals**:
     * A vertical chordwise slot allows full dynamic stroke authority ($\pm 24^\circ$ sprint / $\pm 18^\circ\text{--}22^\circ$ cruise, accommodating macro stops up to $\pm 100^\circ$).
     * Dual flexible $0.1\text{ mm}$ clear mylar / silicone wiper blades line both edges of the slit, maintaining aerodynamic streamlining around the oscillating ⌀10 mm spar without binding or friction.
   * **Fastening**: Secured to the modular split-clamp collars via 2× countersunk M2.5 titanium machine screws into brass heat-set inserts.

---

## 8. Physical Build Quality Verification Gate & Actionable Assembly To-Do List

Before proceeding to electrical commissioning and initial motor spin, the mechanical build team must execute and verify each item on this authoritative assembly checklist, incorporating all findings from the Physical Readiness Audit (`MCO-1D-AUD-PHYS-MK1`):

### 8.1 Mechanical, Drivetrain & Structural Integrity
* [ ] **Total Airframe Dry Mass**: Structural airframe without batteries must weigh $\le 4,972\text{ g}$ (Target MTOW with 6S 4500mAh + 2S 3000mAh: $5,687\text{ g}$; target lightweight flight AUW: $3,670\text{--}4,489\text{ g}$).
* [ ] **Wing Alignment & Zero-Twist**: All 4 pairs show $0.0^\circ \pm 0.2^\circ$ dihedral at rest and zero geometric wash-in/wash-out across span.
* [ ] **Strict 25.0% MAC Spar Pivot Alignment**: Main spar centerline verified at exactly **$36.0\text{ mm} \pm 0.5\text{ mm}$ aft of the wing leading edge** (25.0% MAC) across all 8 panels, eliminating aerodynamic lift pitch couples on AGFRC servos.
* [ ] **Carbon Spar Internal Root Ferrules (PHY-02)**: CNC 6061-T6 aluminum internal crush sleeves ($\varnothing 8.0\,\text{mm} \times 40.0\,\text{mm}$ with $0.5\,\text{mm}$ collar) epoxied into the root bore of all 8 main spars. Clamp split-collars torqued to $2.2\,\text{N}\cdot\text{m}$ with Loctite 243. Prevents composite crush under $2.2g$ flapping ($M_b = 5.36\,\text{N}\cdot\text{m}$).
* [ ] **PC-CF Corner Clamps Torquing (STRUCT-14)**: Frame truss corner clamps printed in 100% infill PC-CF torqued to **$2.2\,\text{N}\cdot\text{m}$** with Grade 12.9 M3 bolts and split lock washers.
* [ ] **Zero-Slop CNC Linkages (HW-07)**: Zero-slop CNC aluminum ball links with M3 titanium turnbuckles installed on all 16 servos, verified for $<0.1\,\text{mm}$ axial play ($<0.2^\circ$ angular play).
* [ ] **Dyneema SK99 48-Hour Creep Settlement (PHY-01)**: All 8 capstan drive cables pre-stretched at $200\,\text{N}$ on an inline tension scale for a minimum of 48 hours to exhaust viscoelastic primary creep.
* [ ] **Final Capstan Pre-Tensioning & Witness Marks (PHY-01)**: Cables re-tensioned to strictly **$160\,\text{N} \pm 10\,\text{N}$**, verified by acoustic tap pitch of **$210\,\text{Hz}$** across a 150 mm free span. Clamping cam screws locked with Loctite 480 flexible CA. High-contrast paint witness marks applied across cable, screw, and sector rim.
* [ ] **Line Keeper Shroud Clearance (STRUCT-17)**: 3D-printed capstan shrouds installed over all 8 motor pinions, verifying radial drum clearance $<0.4\,\text{mm}$ to prevent line derailment.
* [ ] **10:1 Transmission Ratio & Groove Seating**: Motor pinion drum ($d_{\text{pitch}} = 15.4\text{ mm}$ or $18.0\text{ mm}$) and sector wheel ($R = 77.0\text{ mm}$ or $90.0\text{ mm}$) verify exact $10.0:1$ transmission ratio with 3 full Dyneema wraps seated in the helical groove without crossover.
* [ ] **Bearing Freedom**: Sector wheels and motor drums rotate smoothly with zero binding across full $100^\circ$ stroke arc.
* [ ] **Fastener Security**: Every M2/M3 fastener has Loctite 243 visible on threads.

### 8.2 Thermal Isolation & Cooling Architecture
* [ ] **Motor Mount Thermal Isolator Gaskets (HW-08 / PHY-04)**: Precision 0.5 mm high-temp FR4 / silicone thermal barrier gaskets (`HW-08`) installed between all 8 GoolRC motor baseplates and aluminum station brackets. Verified torqued to $1.8\text{ N}\cdot\text{m}$ with Loctite 243. Prevents stator heat from conducting into the primary spine structure.
* [ ] **Ram-Air NACA Inlets & Exhaust Louvers (STRUCT-09 / STRUCT-15 / H-BUS-3)**: Forward canopy $25\text{ mm} \times 15\text{ mm}$ NACA ram-air intake scoop (`STRUCT-09`), delta fairing secondary scoops (`STRUCT-15`), and dual $40\text{ mm}$ belly negative-pressure exhaust louvers verified unobstructed. Individual motor station scoops ($+35\,\text{mm} / +55\,\text{mm}$ lateral offset) and $45^\circ$ scavenge louvers verified clear. Prevents bay and pod temperatures from exceeding $55^\circ\text{C}$ even during continuous 92W Hummingbird Idle glide.
* [ ] **ODrive S1 Inverter Thermal Coupling**: Aluminum heatsinks on all 8 ODrive S1 boards coupled to airflow via $1.5\,\text{mm}$ silicone thermal pads ($k \ge 6.0\,\text{W/m}\cdot\text{K}$).

### 8.3 Aerodynamics, Kinematics & Stability Envelopes
* [ ] **Longitudinal CG & Battery Sled Calibration (PHY-03 / PHY-05)**: Sled quick-release pin locked into **Hole 3 ($250.0\,\text{mm}$ setting)**. Complete aircraft balanced on knife-edge: center of gravity verified at **$311.8\,\text{mm} \pm 3.0\,\text{mm}$ aft of datum ($26.5\%$ MAC)**.
* [ ] **Aft CG Mechanical Lockout**: Physical roll pin installed in Hole 6 of the battery keel track to mechanically prevent sled insertion beyond Hole 5 ($270\,\text{mm}$ / $29.5\%$ MAC aft stability boundary).
* [ ] **Vertical Staircase & Tandem Clearance (PHY-05)**: Vertical drop of $32.0\,\text{mm}$ per station ($96.0\,\text{mm}$ total) and longitudinal stagger of $180.0\text{--}250.0\,\text{mm}$ verified. Minimum dynamic clearance between adjacent wingtips $\ge 35.0\,\text{mm}$ across all stroke phases.
* [ ] **Software Sweep-Differential Limit Enforcement**: Firmware governor clamped to $\Delta \Lambda \le 15.0^\circ$ during high-rate opposing strokes to eliminate inter-station blade clashing.
* [ ] **Active Lift Surplus & Glide Speed Calibration**: With verified physical wing area ($S = 1.224\,\text{m}^2$), passive aerodynamic lift at $9.0\,\text{m/s}$ provides **$63.76\,\text{N}$** ($+16.56\text{ N}$ to $+19.74\text{ N}$ surplus over weight). Minimum unpowered $1g$ clean glide speed calibrated to $V_{\text{glide}} = 7.48\text{--}7.74\,\text{m/s}$ ($16.7\text{--}17.3\,\text{mph}$).
* [ ] **Aerodynamic Fairing Package Installation**: Delta truss infill panels ($95\,\text{g}$) and 8× teardrop wing root nacelles ($48\,\text{g}$) installed; compliant wiper seals verified on spar flapping slits; NACA intake scoops and aft exhaust louvers verified unobstructed. Continuous cruise envelope verified up to $16.0\,\text{m/s}$ (clamped to prevent high-camber S1223 lower-surface separation bubble stall).

### 8.4 Actuation, Servos & Electrical Protection (Dual-Zone Architecture)
* [ ] **Dual-Zone Split Servo Rail Wiring (PWR-02)**: Servo power rail split into Zone A (Stations 1 & 2, Fore) and Zone B (Stations 3 & 4, Aft), each independently powered by a 20A synchronous switching BEC. Verified zero shared positive wiring between Zone A and Zone B.
* [ ] **Dual Low-ESR Buffer Capacitors (PWR-04)**: Two $4,700\,\mu\text{F} \times 16\,\text{V}$ low-ESR electrolytic capacitors soldered directly across Zone A and Zone B bus terminals with leads $<20\,\text{mm}$ (9,400 µF total buffering).
* [ ] **Servo Linkage Backlash (H-89 / PHY-07)**: All 16 AGFRC IB53BHU carbon tie-rods and CNC ball-links verify $< 0.1\text{ mm}$ mechanical play. Pitch horn mechanical advantage verified at $1.2:1$ for dynamic load headroom.
* [ ] **Servo Travel Clamping**: All 16 servos programmed via `AGF-SPV3` to $170^\circ$ max travel ($333\,\text{Hz}$ PWM, $1.0\,\mu\text{s}$ deadband) to prevent MCP3208 ADC rail saturation.
* [ ] **Servo Rail Voltage Droop Test**: Simultaneous 16-servo step deflection under bench test verifies both Zone A and Zone B rail voltages remain $\ge 7.1\,\text{V}$ (droop $\le 0.3\,\text{V}$ with zero brownout).

### 8.5 Avionics & Sensing Systems
* [ ] **CAN-FD Bus Termination & Resistance**: Total line-to-line resistance across CAN_H and CAN_L measures exactly $60.0\,\Omega \pm 1.5\,\Omega$ with power off.
* [ ] **Flight Controller Gel Isolation (H-93)**: Pixhawk 4 Mini mounted on 4× Shore 20A silicone gel grommets with $1.5\,\text{mm}$ uncompressed gap to decouple $2.5\,\text{Hz}$ airframe flapping harmonics.
* [ ] **Pitot-Static Low-$q$ Filtering (PHY-06)**: MS4525DO digital pitot zero-calibrated at ambient temperature with probe cover on. 5-point rolling median filter and low-speed synthetic airspeed fusion ($< 7.5\,\text{m/s}$) verified in Pixhawk EKF3.
* [ ] **TF-Luna LiDAR Ground Zero**: Ground elevation reads $0.02\,\text{m}$ (ground taxi height) at rest on wheels.

### 8.6 Biomimetic Wing Features
* [ ] **Pterostigma Tip Mass Balancing**: All 8 wingtips have $1.5\text{ g}$ brass/tungsten slug potted at $y = 0.85b$ forward of the pitch axis.
* [ ] **Micro-Cavity Step Definition (H-90)**: Universal S1223 ribs verify 3 micro-step cavities ($1.2\text{ mm}$ deep) with tight Mylar covering without bridging or sagging.
* [ ] **Pneumatic Clap Skirt (H-91)**: Inboard trailing edges of Stations 1 & 2 have $15\text{ mm}$ silicone skirts sealing cleanly at $+70^\circ$ dihedral.

### 8.7 Formal Builder Calibration Sheet & Physical Handoff
* [ ] **Builder Calibration Sheet Completion**: Complete all 8 measurement fields (`FIG-01` through `FIG-08`) on [`MCO-1D_Mark1_Builder_Calibration_Sheet.pdf`](file:///Users/drbell/Code/flight/docs/mark1/MCO-1D_Mark1_Builder_Calibration_Sheet.pdf).
* [ ] **As-Built Gross Weight Recorded (`FIG-01`)**: All-up weight verified on calibrated scale (target $< 5,600\text{ g}$, e.g. $4,800\text{--}5,300\text{ g}$). Handed off to firmware lead for dynamic $V_s$ and $P_s$ scaling.
* [ ] **200° Sector Stop Alignment (`FIG-04`)**: Sector wheel travel verified across full $200.0^\circ$ physical arc ($\pm 100.0^\circ$ stops). Software dynamic reciprocating flapping ceiling verified at $\pm 24.0^\circ$ sprint / $\pm 18.0^\circ\text{--}22.0^\circ$ cruise.
* [ ] **Formal Signoff**: Chief Builder and Firmware Lead signatures recorded prior to mounting on outdoor test rig or proceeding to TC-01 bench testing.



