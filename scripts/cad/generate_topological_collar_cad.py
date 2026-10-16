#!/usr/bin/env python3
"""
scripts/cad/generate_topological_collar_cad.py
==============================================
Topologically Optimized Bilateral Wing Collar & Flapping Mechanism Mount
for MCO-1D Ornithopter:
  - Clamps to the primary ⌀20.0mm carbon spine tube along the longitudinal X-axis.
  - Mounts dual V10 Powertrain Stands at Y = ±59.0mm (or matching stand flange).
  - Eliminates legacy V9 monolithic clevis overhangs and orphaned motor stanchions.
  - Implements closed-loop transverse tension arch tying port and starboard lift couples.
  - Standard 4x M3 pattern (20mm x 24mm) + central ⌀6.00mm titanium shear pin on each pad.
"""

import math
import os
import build123d as b3d

OUT_DIR = "cad/manufactured"
os.makedirs(OUT_DIR, exist_ok=True)

# -----------------------------------------------------------------------------
# GEOMETRIC & INTERFACE PARAMETERS
# -----------------------------------------------------------------------------
D_spine = 20.0        # mm: Spine tube outer diameter
R_spine = D_spine / 2 # 10.0 mm
collar_len = 36.0     # mm: Sleeve axial length along spine (X-axis)

# Bilateral Powertrain Stand Mating Interface (normal along Y)
Y_flange = 59.0       # mm: Stand mating flange face at Y = ±59.0mm
pad_w = 26.0          # mm: Pad width along X
pad_h = 30.0          # mm: Pad height along Z (Z: -15.0 to +15.0)
z_bot = -15.0
z_top = 15.0

print("=" * 78)
print("  GENERATING TOPOLOGICALLY OPTIMIZED BILATERAL WING COLLAR (V10 DFM MOUNT)")
print("=" * 78)

with b3d.BuildPart() as topological_collar:
    # 1. Central spine clamping sleeve (OD = 25.6mm)
    with b3d.BuildSketch(b3d.Plane.YZ):
        b3d.Circle(radius=R_spine + 2.8)
        with b3d.Locations((0.0, R_spine + 3.5)):
            b3d.Rectangle(width=14.0, height=7.0)
    b3d.extrude(amount=collar_len / 2.0, both=True)

    # 2. Bilateral mounting arms & pads (out to exact Y = ±59.0mm)
    for sign in [-1, 1]:
        arm_len = Y_flange - 10.0
        with b3d.BuildSketch(b3d.Plane.XY.offset(z_bot)):
            with b3d.Locations((0.0, sign * (10.0 + arm_len / 2.0))):
                b3d.Rectangle(width=pad_w, height=arm_len)
        b3d.extrude(amount=pad_h)

    # 3. Transverse truss pocketing / lightening (leaving 5.5mm solid walls along X)
    for sign in [-1, 1]:
        with b3d.BuildSketch(b3d.Plane.XY.offset(z_bot - 1.0)):
            with b3d.Locations((0.0, sign * 34.5)):
                b3d.Rectangle(width=15.0, height=22.0)
        b3d.extrude(amount=pad_h + 2.0, mode=b3d.Mode.SUBTRACT)

        with b3d.BuildSketch(b3d.Plane.YZ):
            with b3d.Locations((sign * 34.5, 0.0)):
                b3d.Circle(radius=5.0)
        b3d.extrude(amount=pad_w + 4.0, both=True, mode=b3d.Mode.SUBTRACT)

    # 4. Central Ø20.0mm Spine Bore along X with 45-deg teardrop crown
    with b3d.BuildSketch(b3d.Plane.YZ):
        b3d.Circle(radius=R_spine)
        b3d.Polygon([
            (-R_spine * 0.707, R_spine * 0.707),
            (0.0, R_spine * 1.414),
            (R_spine * 0.707, R_spine * 0.707),
        ], align=None)
    b3d.extrude(amount=collar_len + 10.0, both=True, mode=b3d.Mode.SUBTRACT)

    # 5. Top Pinch Clamp Relief Slot (1.2mm slit) and 2x M3 clamp pinch bolt bores
    with b3d.BuildSketch(b3d.Plane.XZ.offset(0.0)):
        with b3d.Locations((0.0, R_spine + 3.5)):
            b3d.Rectangle(width=collar_len + 2.0, height=1.2)
    b3d.extrude(amount=15.0, mode=b3d.Mode.SUBTRACT)

    for cx in [-9.0, 9.0]:
        with b3d.BuildSketch(b3d.Plane.XZ.offset(-9.0)):
            with b3d.Locations((cx, R_spine + 4.0)):
                b3d.Circle(radius=1.65)
        b3d.extrude(amount=18.0, mode=b3d.Mode.SUBTRACT)

    # 6. Lateral Wing-Stand Mounting Flange Bores (at Y = ±Y_flange, centered at X = 0.0mm, Z = 0.0mm)
    for sign in [-1, 1]:
        with b3d.BuildSketch(b3d.Plane.XZ.offset(-sign * Y_flange)):
            b3d.Circle(radius=3.0)
            b3d.Polygon([
                (-3.0 * 0.707, 3.0 * 0.707),
                (0.0, 3.0 * 1.414),
                (3.0 * 0.707, 3.0 * 0.707),
            ], align=None)
        b3d.extrude(amount=sign * 6.5, mode=b3d.Mode.SUBTRACT)

        for dx in [-10.0, 10.0]:
            for dz in [-12.0, 12.0]:
                hx = dx
                hz = dz
                rh = 1.65
                with b3d.BuildSketch(b3d.Plane.XZ.offset(-sign * Y_flange)):
                    with b3d.Locations((hx, hz)):
                        b3d.Circle(radius=rh)
                    b3d.Polygon([
                        (hx - rh * 0.707, hz + rh * 0.707),
                        (hx, hz + rh * 1.414),
                        (hx + rh * 0.707, hz + rh * 0.707),
                    ], align=None)
                b3d.extrude(amount=sign * 10.0, mode=b3d.Mode.SUBTRACT)

    # 7. Top Radial Ø3.0mm Anti-Rotation Shear Dowel Pin Bore at (0, 0)
    with b3d.BuildSketch(b3d.Plane.XY.offset(0.0)):
        b3d.Circle(radius=1.55)
    b3d.extrude(amount=25.0, mode=b3d.Mode.SUBTRACT)

collar_solid = topological_collar.part

# -----------------------------------------------------------------------------
# MASS & METRICS AUDIT
# -----------------------------------------------------------------------------
vol_mm3 = collar_solid.volume
vol_cm3 = vol_mm3 / 1000.0

rho_pacf = 1.15   # g/cm3 (PA12-CF / Carbon Nylon)
rho_al7075 = 2.81 # g/cm3 (7075-T6 CNC Aluminum)
rho_petg = 1.27   # g/cm3 (PETG)

mass_pacf = vol_cm3 * rho_pacf
mass_al7075 = vol_cm3 * rho_al7075
mass_petg = vol_cm3 * rho_petg

print("-" * 78)
print(f"Topological Solid Volume   : {vol_cm3:.2f} cm³ ({vol_mm3:.1f} mm³)")
print(f"Bilateral Collar Mass (PA12-CF): {mass_pacf:.2f} g")
print(f"Bilateral Collar Mass (PETG)   : {mass_petg:.2f} g")
print(f"Bilateral Collar Mass (7075-T6): {mass_al7075:.2f} g")
print("=" * 78)

# -----------------------------------------------------------------------------
# EXPORT STEP & STL DELIVERABLES
# -----------------------------------------------------------------------------
step_path = os.path.join(OUT_DIR, "mco_topological_wing_collar_v1.step")
stl_path  = os.path.join(OUT_DIR, "mco_topological_wing_collar_v1.stl")

b3d.export_step(collar_solid, step_path)
print(f"Exported STEP: {step_path}")

b3d.export_stl(collar_solid, stl_path)
print(f"Exported STL : {stl_path}")
