# MCO-1D Wing Root 3D Print Package — Build Guide & BOM

**Configuration**: One Complete Robotic Flapping Wing Root Device (Wrist + Shoulder + Fuselage Scaffolding Collar)  
**Status**: Critical Design Review (CDR) Baseline — Approved for Production  
**Target Printer**: Any 3D printer with ≥ 220 × 220 × 200 mm build volume (Bambu Lab X1C / P1S, Voron 2.4, Prusa MK4, Creality K1)  

---

## 1. Quick-Start Slicing Recommendation

- **Option A (Fast Single-Bed Print — 100% Structural Parts)**:
  Import `PLATE_01_PA12CF_ALL_STRUCTURAL_PARTS.stl` directly into OrcaSlicer / Bambu Studio / PrusaSlicer.
  Every structural part is pre-oriented flat on the build plate for optimal layer bonding and zero/minimal support.
  - **Estimated Print Time**: ~5 hrs 30 mins (at 0.16mm layer height).
  - **Estimated Filament Usage**: ~108g of PA12-CF.

- **Option B (Individual Part Printing)**:
  Import individual `.stl` files from this folder according to the slicing parameters in the table below.

---

## 2. Complete 3D Printable Parts Manifest & Slicing Specifications

| File Name (STL & STEP) | Subsystem & Function | Recommended Material | Walls / Perimeters | Infill % & Pattern | Mass (g) | Qty |
| :--- | :--- | :--- | :---: | :---: | :---: | :---: |
| `01_mco_bilateral_fuselage_collar_v1` | **Fuselage Collar**: Clamps Ø20mm spine tube, mounts both wings | PA12-CF | 6 | 60% Gyroid | 69.5 g | 1 |
| `02_v10_powertrain_mount_stand` | **Shoulder Chassis**: Slotted motor deck, ODrive pocket, tower | PA12-CF | 6 | 60% Gyroid | 36.0 g | 1 |
| `03_v10_powertrain_dual_shear_top_brace` | **Shoulder Tie-Bar**: Locks drum axle to 688 top pinion bearing | PA12-CF | 6 | 100% Solid | 5.8 g | 1 |
| `04_v10_stagecoach_sector_drum` | **Shoulder Drum**: 7 spokes, dual helical grooves, tunnels, M3 clamps | PA12-CF | 4 | 35% Gyroid | 34.2 g | 1 |
| `05_v10_stepped_capstan_pinion` | **Capstan Pinion**: 10:1 reduction, D-shaft bore, 688 top journal | PA12-CF | 6 | 100% Solid | 1.8 g | 1 |
| `06_v9_differential_gimbal_yoke` | **Wrist Gimbal**: Holds Ø10mm spar socket & Ø4mm cross-axle | PA12-CF | 6 | 100% Solid | 3.1 g | 1 |
| `07_v9_quickswap_bevel_gear_x3_plate` | **Bevel Gearset (3x)**: Left input, Right input, Spar spider | PA12-CF | 6 | 100% Solid | 4.5 g | 1 (set of 3) |
| `08_v9_quickswap_bevel_gear_single` | **Spare Bevel Gear**: Quick-swap field replacement spare | PA12-CF | 6 | 100% Solid | 1.5 g | 1 (spare) |
| `PLATE_01_PA12CF_ALL_STRUCTURAL_PARTS` | **Single-Bed Layout**: All PA12-CF parts pre-arranged on 220x220mm | PA12-CF | 6 | 50% Gyroid | ~150 g | 1 |

---

## 3. Recommended Slicer Profiles (PA12-CF Carbon Nylon)

- **Nozzle Diameter**: 0.40 mm hardened steel (0.60 mm optional).
- **Layer Height**: 0.16 mm (Optimal compromise between gear tooth fidelity and print speed).
- **First Layer Height**: 0.20 mm.
- **Nozzle Temperature**: 285°C – 295°C.
- **Bed Temperature**: 100°C – 110°C (Engineering textured PEI plate with glue stick / Magigoo PA).
- **Chamber Temperature**: Enclosed chamber recommended (heated to 50°C–60°C or passively enclosed).
- **Drying Condition**: Filament MUST be dried at 80°C for ≥ 8 hours before printing (< 15% RH).
- **Seam Placement**: 'Nearest' or 'Aligned' on back facets. Avoid seams on bevel gear teeth or capstan grooves.
- **Supports**: 
  - `PLATE_01` is pre-oriented flat to require **no supports** on functional faces.
  - Tree supports (organic) are only needed for the small underside clevis gap on the collar.

---

## 4. Hardware Checklist to Complete 1 Full Wing Root Assembly

| Component | Specification | Quantity | Mechanical Location |
| :--- | :--- | :---: | :--- |
| **Main Motor** | GoolRC 5010 360KV BLDC (Ø50mm, 4mm shaft) | 1 | Powertrain Stand (slotted mount) |
| **Flapping Controller** | ODrive S1 CAN-FD Controller | 1 | Stand Undercarriage Pocket |
| **Magnetic Encoder** | Diametral Neodymium Magnet Ø6.35 × 2.5 mm | 1 | Motor Shaft Tail (1.50mm air gap) |
| **Wrist Servos** | AGFRC B24CLM / B44BLS Servos | 2 | Stagecoach Sector Drum Side Bays |
| **Wing Spar Tube** | 3K Woven Carbon Fiber Tube Ø10.0 × 8.0 mm | 1 | Wrist Gimbal Socket |
| **Fuselage Spine** | 3K Roll-Wrapped Carbon Fiber Tube Ø20.0 × 17.0 mm | 1 | Central Fuselage Spine |
| **Flapping Bearings** | 6800-2RS Deep-Groove Bearings (10 × 19 × 5 mm) | 2 | Stagecoach Drum Hub & Tower Seat |
| **Top Pinion Bearing** | 688-2RS Deep-Groove Bearing (8 × 16 × 5 mm) | 1 | Top Brace Pinion Pocket |
| **Gimbal Bearings** | MR74-2RS Miniature Bearings (4 × 7 × 2.5 mm) | 2 | Differential Yoke Pivot |
| **Flapping Axle** | Grade 5 Titanium Dead Axle Pin Ø10.0 × 44.0 mm | 1 | Main Pivot Centerline |
| **Wrist Cross-Axle** | Ground Stainless Steel Pin Ø4.0 × 26.0 mm | 1 | Differential Gimbal Pitch Axis |
| **Capstan Cable** | Braided Dyneema SK99 Line Ø0.80 mm | 1.2 m | 10:1 Helical Capstan Drive Wrap |
| **Fasteners (Clamping)** | M3 × 16 mm Socket Cap Screws (Grade 12.9) | 4 | Fuselage Collar Split Pinch Ears |
| **Fasteners (Motor)** | M3 × 8 mm Button Head Screws (Grade 12.9) | 4 | GoolRC 5010 Motor Mount Slots |
| **Fasteners (Top Brace)**| M3 × 12 mm Socket Cap Screws | 2 | Stand Tie-Down Struts |
| **Fasteners (Cable Clamp)**| M3 × 6 mm Low-Profile Screws & Washers | 2 | Sector Drum Dyneema Clamps |

---

## 5. Companion Installation Manuals

- Complete 2-page PDF assembly manual:  
  `docs/mark1/MCO-1D_Hierarchical_Exploded_Installation_Guide.pdf`
- V10 Powertrain & ODrive installation guide:  
  `docs/mark1/MCO-1D_V10_Powertrain_Mount_Installation_Guide.pdf`
