#!/usr/bin/env python3
"""
scripts/generate_spine_mating_collar.py
========================================
Generates the Lightweight Topologically Optimized Bilateral Fuselage Spine Collar:
  - Clamps to the Ø20.0mm carbon fiber spine longeron tube.
  - Features bilateral vertical mounting bulkheads at X = ±26.0mm.
  - Matches the V10 Powertrain Mount (Stand) bolt pattern:
      * Central Ø6.00mm titanium shear pin bore at (Y = -21.0mm, Z = 24.5mm).
      * 4x M3 clearance bores on 20mm (Y) x 40mm (Z) pitch.
  - Bilateral mounting: Active wing on one side, Opposing wing on the other side.
  - Warren-truss webs linking bulkheads to the central tube.
  - Dual split-collar pinch ears with 2x M3 Grade 12.9 clamping bolts.
"""

import os
import build123d as b3d

OUT_DIR = "cad/manufactured"
os.makedirs(OUT_DIR, exist_ok=True)

# Datums matching V10 Powertrain Mount
FLANGE_X = 26.0           # mm: Flange face offset from spine centerline
FLANGE_THICK = 4.0        # mm: Mounting flange thickness
D_SPINE = 20.0            # mm: Spine tube OD
R_SPINE = D_SPINE / 2.0   # 10.0 mm
COLLAR_LEN = 44.0         # mm: Total sleeve length along Y (covers Y = -43mm to +1mm)
Y_CENTER = -21.0          # mm: Center of mounting pattern
Z_PIN = 24.5              # mm: Centerline of shear pin
PITCH_Y = 20.0            # mm: M3 hole pitch along Y
PITCH_Z = 40.0            # mm: M3 hole pitch along Z
PIN_D = 6.05              # mm: Ø6mm shear pin bore
BOLT_D = 3.3              # mm: M3 clearance bore

print("=" * 76)
print("  GENERATING LIGHTWEIGHT BILATERAL FUSELAGE SPINE COLLAR")
print("=" * 76)

with b3d.BuildPart() as spine_collar:
    # 1. Central Sleeve along Y-axis (Ø20mm ID, Ø24mm OD -> 2mm wall)
    with b3d.BuildSketch(b3d.Plane(origin=(0, Y_CENTER, 0), x_dir=(1, 0, 0), z_dir=(0, 0, 1))):
        b3d.Circle(radius=R_SPINE + 2.0)
    b3d.extrude(amount=COLLAR_LEN / 2.0, both=True)

    # 2. Bilateral Vertical Mounting Plates at X = ±FLANGE_X
    # Height covers Z = Z_PIN - 24mm (0.5mm) to Z_PIN + 24mm (48.5mm)
    # Length covers Y = Y_CENTER - 16mm (-37mm) to Y_CENTER + 16mm (-5mm)
    for sx in [-1, 1]:
        # Vertical Flange Plate
        with b3d.BuildSketch(b3d.Plane(origin=(sx * (FLANGE_X - FLANGE_THICK / 2.0), Y_CENTER, Z_PIN), x_dir=(0, 1, 0), z_dir=(0, 0, 1))):
            b3d.Rectangle(width=34.0, height=PITCH_Z + 10.0)
        b3d.extrude(amount=FLANGE_THICK, both=True)

        # Transverse gusset struts tying vertical plate to central sleeve
        # Upper strut at Z = Z_PIN + 15mm
        with b3d.BuildSketch(b3d.Plane.XY.offset(Z_PIN + 15.0)):
            b3d.Polygon([
                (0.0, Y_CENTER - 12.0),
                (sx * FLANGE_X, Y_CENTER - 12.0),
                (sx * FLANGE_X, Y_CENTER + 12.0),
                (0.0, Y_CENTER + 12.0)
            ])
        b3d.extrude(amount=4.0)

        # Lower strut at Z = Z_PIN - 15mm
        with b3d.BuildSketch(b3d.Plane.XY.offset(Z_PIN - 15.0)):
            b3d.Polygon([
                (0.0, Y_CENTER - 12.0),
                (sx * FLANGE_X, Y_CENTER - 12.0),
                (sx * FLANGE_X, Y_CENTER + 12.0),
                (0.0, Y_CENTER + 12.0)
            ])
        b3d.extrude(amount=4.0)

        # Diagonal Warren shear gusset linking center of plate to sleeve
        with b3d.BuildSketch(b3d.Plane(origin=(sx * FLANGE_X / 2.0, Y_CENTER, Z_PIN / 2.0), x_dir=(1, 0, 0), z_dir=(0, 0, 1))):
            b3d.Rectangle(width=FLANGE_X, height=4.0)
        b3d.extrude(amount=10.0, both=True)

    # Central vertical rib tying upper struts down to sleeve (Z = 10 to Z = 40mm)
    with b3d.BuildSketch(b3d.Plane(origin=(0.0, Y_CENTER, 25.0), x_dir=(0, 1, 0), z_dir=(0, 0, 1))):
        b3d.Rectangle(width=22.0, height=28.0)
    b3d.extrude(amount=3.0, both=True)

    # 3. Central Spine Bore (Ø20.05mm ID through-bore along Y)
    with b3d.BuildSketch(b3d.Plane(origin=(0, Y_CENTER, 0), x_dir=(1, 0, 0), z_dir=(0, 0, 1))):
        b3d.Circle(radius=D_SPINE / 2.0 + 0.05)
    b3d.extrude(amount=COLLAR_LEN + 10.0, both=True, mode=b3d.Mode.SUBTRACT)

    # 4. Bilateral Flange Mounting Bores (Port side +X and Opposing side -X)
    for sx in [-1, 1]:
        # Center Ø6.00mm shear pin bore
        with b3d.BuildSketch(b3d.Plane.YZ.offset(sx * (FLANGE_X + 2.0))):
            with b3d.Locations((Y_CENTER, Z_PIN)):
                b3d.Circle(radius=PIN_D / 2.0)
        b3d.extrude(amount=FLANGE_THICK + 6.0, both=True, mode=b3d.Mode.SUBTRACT)

        # 4x M3 clearance bores (on 20mm Y x 40mm Z pitch)
        for dy in [-PITCH_Y / 2.0, PITCH_Y / 2.0]:
            for dz in [-PITCH_Z / 2.0, PITCH_Z / 2.0]:
                with b3d.BuildSketch(b3d.Plane.YZ.offset(sx * (FLANGE_X + 2.0))):
                    with b3d.Locations((Y_CENTER + dy, Z_PIN + dz)):
                        b3d.Circle(radius=BOLT_D / 2.0)
                b3d.extrude(amount=FLANGE_THICK + 6.0, both=True, mode=b3d.Mode.SUBTRACT)

        # Flange window lightening cutout (between upper and lower M3 bolts)
        with b3d.BuildSketch(b3d.Plane.YZ.offset(sx * (FLANGE_X + 2.0))):
            with b3d.Locations((Y_CENTER, Z_PIN)):
                b3d.Rectangle(width=12.0, height=18.0)
        b3d.extrude(amount=FLANGE_THICK + 6.0, both=True, mode=b3d.Mode.SUBTRACT)

    # 5. Bottom Split-Collar Clamping Pinch Ears
    with b3d.BuildSketch(b3d.Plane.XY.offset(-R_SPINE - 6.0)):
        with b3d.Locations((0.0, Y_CENTER)):
            b3d.Rectangle(width=16.0, height=28.0)
    b3d.extrude(amount=7.0)

    # Clamping pinch slot (1.5mm along XZ plane)
    with b3d.BuildSketch(b3d.Plane(origin=(0, Y_CENTER, -R_SPINE - 3.0), x_dir=(1, 0, 0), z_dir=(0, 0, 1))):
        b3d.Rectangle(width=1.5, height=12.0)
    b3d.extrude(amount=16.0, both=True, mode=b3d.Mode.SUBTRACT)

    # 2x M3 Pinch Bolt Clearance Bores across clamp ears (X-axis)
    for py in [Y_CENTER - 7.0, Y_CENTER + 7.0]:
        with b3d.Locations(b3d.Location((0.0, py, -R_SPINE - 3.0), (0, 90, 0))):
            b3d.Cylinder(radius=BOLT_D / 2.0, height=24.0, mode=b3d.Mode.SUBTRACT)

collar_solid = spine_collar.part
vol_collar = collar_solid.volume / 1000.0
mass_pa12 = vol_collar * 1.25

b3d.export_stl(collar_solid, os.path.join(OUT_DIR, "mco_spine_bilateral_mating_collar.stl"))
b3d.export_step(collar_solid, os.path.join(OUT_DIR, "mco_spine_bilateral_mating_collar.step"))

print(f"Lightweight Spine Collar Solid Volume : {vol_collar:.2f} cm³")
print(f"Lightweight Spine Collar Mass PA12-CF : {mass_pa12:.2f} g")
print(f"Collar shape type: {type(collar_solid)}")
if isinstance(collar_solid, b3d.Compound):
    print("Number of solids in collar compound:", len(collar_solid.solids()))
else:
    print("Collar is a single solid!")
print(f"Exported: mco_spine_bilateral_mating_collar.stl & .step")
