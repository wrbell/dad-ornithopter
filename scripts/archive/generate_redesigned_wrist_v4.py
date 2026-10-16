#!/usr/bin/env python3
"""
scripts/generate_redesigned_wrist_v4.py
=======================================
Generates the fully redesigned 2-DOF wing root wrist joint & sector wheel:
  1. 0.6mm Dyneema SK99 cable (D:d = 11.67 : 1 on 7mm motor pinion).
  2. Exact 10.0 : 1 capstan reduction (R_sector = 35.0 mm, r_pinion = 3.5 mm).
  3. Wing spar extends out from the FLAT FACE (+Y direction) into the wing.
  4. Core reduced from 40mm to a compact 10.0mm hollow bore for wiring (dual 6800-2RS bearings).
  5. Dual AGFRC B24CLM servos pulled as far away from the flat face as possible (nested aft into -Y).
  6. Includes GoolRC 5010 motor and small capstan pinion drum model tangent to curved arc (-Y).
"""

import math
import os
import build123d as b3d

OUT_DIR = "cad/manufactured"
os.makedirs(OUT_DIR, exist_ok=True)

# ------------------------------------------------------------------------------
# PARAMETERS
# ------------------------------------------------------------------------------
# 10:1 Capstan Gear Ratio
R_pitch = 35.0        # mm (Sector pitch radius)
R_flange = 37.5       # mm (Outer cable retention flange)
arc_deg = 150.0       # deg (Sweep arc for flapping travel)
rim_thickness = 8.0   # mm (Dyneema cable track height)

# 0.6mm Dyneema Pinion Drum
r_pinion = 3.5        # mm (Pinion pitch radius: dia 7.0mm, bend ratio 11.67:1)
r_pinion_flange = 4.5 # mm (Pinion outer flange: dia 9.0mm)
pinion_height = 12.0  # mm

# Central Hub & Wiring Bore (Replaces 40mm spring bore)
r_wire_bore = 5.0     # mm (10.0mm ID hollow core for servo wire harness)
r_bearing_od = 9.5    # mm (19.0mm OD for 6800-2RS bearings: 10x19x5mm)
h_bearing = 5.0       # mm (bearing width)
r_hub_outer = 11.5    # mm (23.0mm OD bearing housing boss)
drum_height = 24.0    # mm (compacted from 32mm)

# Servo Integration (AGFRC B24CLM: 23 x 12 x 27.5mm)
# Pulled as far away from the flat face (Y >= 0) as possible -> into -Y
servo_x_offset = 14.5 # mm (tight to the 23mm OD hub boss)
servo_y_offset = -17.0 # mm (deep toward the curved arc, away from flat face)
servo_w = 12.6        # mm
servo_l = 23.6        # mm
servo_d = 20.0        # mm (depth into drum)

# Wing Spar & Collar (Extends from Flat Face in +Y direction)
spar_od = 10.05       # mm (10mm Carbon Fiber spar tube)
z_pivot = drum_height + 10.0 # mm (34.0mm pivot elevation)

print("Redesigned Wrist & Sector V4 Parameters:")
print(f"  Sector Pitch Radius R_pitch : {R_pitch:.1f} mm")
print(f"  Motor Pinion Radius r_pin   : {r_pinion:.1f} mm (dia {2*r_pinion:.1f} mm)")
print(f"  Gear Ratio                  : {R_pitch / r_pinion:.1f} : 1")
print(f"  Dyneema Diameter            : 0.6 mm (Bend Ratio = {2*r_pinion/0.6:.1f} : 1)")
print(f"  Hollow Wire Pass Bore       : {2*r_wire_bore:.1f} mm ID (Bearings: 6800-2RS)")
print(f"  Pivot Elevation             : {z_pivot:.1f} mm")

# ------------------------------------------------------------------------------
# 1. REDESIGNED SECTOR DRUM WITH FLAT FACE AT +Y, CURVED ARC AT -Y
# ------------------------------------------------------------------------------
steps = 60
half_arc = math.radians(arc_deg / 2.0)
st_ang = -math.pi / 2.0 - half_arc
en_ang = -math.pi / 2.0 + half_arc
d_ang = (en_ang - st_ang) / steps

# Arc points along the curved rim (at -Y)
arc_pts = []
for s in range(steps + 1):
    ang = st_ang + s * d_ang
    arc_pts.append((R_flange * math.cos(ang), R_flange * math.sin(ang)))

# Flat face at +Y side (connecting across Y = 8.0mm)
flat_face_y = 8.0 # mm
p_end = arc_pts[-1]
p_start = arc_pts[0]
sector_poly_pts = arc_pts + [(p_end[0], flat_face_y), (p_start[0], flat_face_y)]

with b3d.BuildPart() as sector_drum:
    # Main Body Extrusion
    with b3d.BuildSketch(b3d.Plane.XY):
        b3d.Polygon(sector_poly_pts)
    b3d.extrude(amount=drum_height)

    # Capstan Cable Tracking Groove (Pitch R = 35mm, width 2.4mm, depth 2.5mm)
    groove_pts = []
    for s in range(steps + 1):
        ang = st_ang + s * d_ang
        groove_pts.append(((R_flange + 1.0) * math.cos(ang), (R_flange + 1.0) * math.sin(ang)))
    for s in range(steps, -1, -1):
        ang = st_ang + s * d_ang
        groove_pts.append((R_pitch * math.cos(ang), R_pitch * math.sin(ang)))
    with b3d.BuildSketch(b3d.Plane.XY.offset(2.8)):
        b3d.Polygon(groove_pts)
    b3d.extrude(amount=2.4, mode=b3d.Mode.SUBTRACT)

    # Central 10mm Wire Pass-Through Bore
    with b3d.BuildSketch(b3d.Plane.XY):
        b3d.Circle(radius=r_wire_bore)
    b3d.extrude(amount=drum_height + 10.0, mode=b3d.Mode.SUBTRACT, both=True)

    # Precision 6800-2RS Bearing Seats (19mm OD, 5mm depth top & bottom)
    with b3d.BuildSketch(b3d.Plane.XY):
        b3d.Circle(radius=r_bearing_od)
    b3d.extrude(amount=h_bearing + 0.2, mode=b3d.Mode.SUBTRACT)

    with b3d.BuildSketch(b3d.Plane.XY.offset(drum_height - h_bearing - 0.2)):
        b3d.Circle(radius=r_bearing_od)
    b3d.extrude(amount=h_bearing + 0.5, mode=b3d.Mode.SUBTRACT)

    # Dual Aft Servo Bays (Pulled away from flat face into -Y)
    for x_pos in [-servo_x_offset, servo_x_offset]:
        with b3d.BuildSketch(b3d.Plane.XY.offset(drum_height - servo_d)):
            with b3d.Locations((x_pos, servo_y_offset)):
                b3d.Rectangle(width=servo_w, height=servo_l)
        b3d.extrude(amount=servo_d + 1.0, mode=b3d.Mode.SUBTRACT)
        # Mounting screw holes (M2)
        for dy in [-servo_l / 2.0 + 2.0, servo_l / 2.0 - 2.0]:
            with b3d.BuildSketch(b3d.Plane.XY.offset(drum_height - servo_d - 3.0)):
                with b3d.Locations((x_pos, servo_y_offset + dy)):
                    b3d.Circle(radius=1.1)
            b3d.extrude(amount=6.0, mode=b3d.Mode.SUBTRACT)

    # Topological Lightening Pockets
    # Aft central pocket between servos
    with b3d.BuildSketch(b3d.Plane.XY.offset(-1.0)):
        with b3d.Locations((0.0, servo_y_offset - 4.0)):
            b3d.Circle(radius=3.5)
    b3d.extrude(amount=drum_height + 2.0, mode=b3d.Mode.SUBTRACT)

    # Flange lightening cutouts
    for ang_deg in [-135, -110, -70, -45]:
        rad = math.radians(ang_deg)
        cx = 26.0 * math.cos(rad)
        cy = 26.0 * math.sin(rad)
        with b3d.BuildSketch(b3d.Plane.XY.offset(-1.0)):
            with b3d.Locations((cx, cy)):
                b3d.Circle(radius=3.0)
        b3d.extrude(amount=rim_thickness + 2.0, mode=b3d.Mode.SUBTRACT)

drum_solid = sector_drum.part

# ------------------------------------------------------------------------------
# 2. 2-DOF WRIST YOKE WITH SPAR EXTENDING OUT OF FLAT FACE (+Y)
# ------------------------------------------------------------------------------
with b3d.BuildPart() as wrist_assembly:
    # Upright Pylons on Drum Top Face
    for x_pylon in [-12.0, 12.0]:
        with b3d.BuildSketch(b3d.Plane.YZ.offset(x_pylon)):
            with b3d.Locations((0, drum_height)):
                b3d.Polygon([
                    (-7.0, 0.0), (7.0, 0.0),
                    (5.0, 15.0), (-5.0, 15.0)
                ])
        b3d.extrude(amount=3.0 if x_pylon < 0 else -3.0)

    # Transverse Gimbal Cross-Pin (along X-axis at Y=0, Z=z_pivot)
    with b3d.Locations(b3d.Location((0, 0, z_pivot), (0, 90, 0))):
        b3d.Cylinder(radius=3.5, height=22.0)

    # Bevel Gear Differentials
    for sign in [-1, 1]:
        with b3d.Locations(b3d.Location((sign * 8.5, 0, z_pivot), (0, 90, 0))):
            b3d.Cylinder(radius=7.0, height=2.5)

    # Wing Spar Clamp Collar Body EXTENDING OUT OF THE FLAT FACE (+Y direction)
    # Centered at Y = 13.0, Z = z_pivot, length = 26.0mm (Y from 0 to 26)
    with b3d.Locations(b3d.Location((0, 13.0, z_pivot), (90, 0, 0))):
        b3d.Cylinder(radius=7.0, height=26.0)

    # Spar Bore (10.05mm ID) through collar (Y from -1 to 27)
    with b3d.Locations(b3d.Location((0, 13.0, z_pivot), (90, 0, 0))):
        b3d.Cylinder(radius=spar_od / 2.0, height=28.0, mode=b3d.Mode.SUBTRACT)

    # Spar clamping slit (1.0mm slit along top)
    with b3d.BuildSketch(b3d.Plane.XY.offset(z_pivot - 1.0)):
        with b3d.Locations((0.0, 13.0)):
            b3d.Rectangle(width=1.0, height=26.0)
    b3d.extrude(amount=9.0, mode=b3d.Mode.SUBTRACT)

    # Clamping screw holes (M2.5 cross screws at Y=7 and Y=19)
    for y_pinch in [7.0, 19.0]:
        with b3d.Locations(b3d.Location((0, y_pinch, z_pivot + 4.2), (0, 90, 0))):
            b3d.Cylinder(radius=1.3, height=16.0, mode=b3d.Mode.SUBTRACT)

wrist_solid = wrist_assembly.part

# ------------------------------------------------------------------------------
# 3. CARBON FIBER WING SPAR TUBE (Emerging from Flat Face into +Y)
# ------------------------------------------------------------------------------
# Extends from Y = 10.0mm out to Y = 110.0mm (100mm span segment)
with b3d.BuildPart() as spar_tube:
    with b3d.Locations(b3d.Location((0, 60.0, z_pivot), (90, 0, 0))):
        b3d.Cylinder(radius=spar_od / 2.0, height=100.0)
    # Hollow interior (8.0mm ID)
    with b3d.Locations(b3d.Location((0, 60.0, z_pivot), (90, 0, 0))):
        b3d.Cylinder(radius=4.0, height=102.0, mode=b3d.Mode.SUBTRACT)

spar_solid = spar_tube.part

# ------------------------------------------------------------------------------
# 4. GOOLRC 5010 FLAP MOTOR & SMALL CAPSTAN PINION DRUM MODEL
# ------------------------------------------------------------------------------
# Placed tangent to the curved sector arc at -Y
motor_y = -R_pitch - r_pinion - 1.0 # mm (Y = -39.5mm)

with b3d.BuildPart() as motor_pinion:
    # Pinion Drum (Pitch dia 7.0mm, Flange dia 9.0mm)
    with b3d.BuildSketch(b3d.Plane.XY.offset(2.0)):
        with b3d.Locations((0.0, motor_y)):
            b3d.Circle(radius=r_pinion_flange)
    b3d.extrude(amount=pinion_height)

    # Cable groove on pinion
    with b3d.BuildSketch(b3d.Plane.XY.offset(2.8)):
        with b3d.Locations((0.0, motor_y)):
            b3d.Circle(radius=r_pinion)
    b3d.extrude(amount=2.4)

    # Motor Shaft Bore (4.0mm)
    with b3d.BuildSketch(b3d.Plane.XY.offset(-1.0)):
        with b3d.Locations((0.0, motor_y)):
            b3d.Circle(radius=2.0)
    b3d.extrude(amount=pinion_height + 5.0, mode=b3d.Mode.SUBTRACT)

    # GoolRC 5010 Brushless Motor Body (Dia 50.0mm, Height 15.0mm)
    # Positioned underneath the pinion
    with b3d.BuildSketch(b3d.Plane.XY.offset(-15.0)):
        with b3d.Locations((0.0, motor_y)):
            b3d.Circle(radius=25.0) # 50mm OD stator housing
    b3d.extrude(amount=15.0)

    # Motor central hub
    with b3d.BuildSketch(b3d.Plane.XY.offset(-16.0)):
        with b3d.Locations((0.0, motor_y)):
            b3d.Circle(radius=6.0)
    b3d.extrude(amount=18.0)

motor_solid = motor_pinion.part

# ------------------------------------------------------------------------------
# 5. STREAMLINED NACA 0018 FAIRING (Oriented with Flight Flow)
# ------------------------------------------------------------------------------
def get_fairing_pts(chord=160.0, num_points=60):
    t = 0.18
    pts_up = []
    pts_dn = []
    for i in range(num_points + 1):
        beta = math.pi * i / num_points
        x = chord * 0.5 * (1.0 - math.cos(beta))
        xn = x / chord
        if xn > 1.0: xn = 1.0
        yt = 5.0 * t * chord * (
            0.2969 * math.sqrt(xn)
            - 0.1260 * xn
            - 0.3516 * (xn ** 2)
            + 0.2843 * (xn ** 3)
            - 0.1015 * (xn ** 4)
        )
        pts_up.append((yt, 20.0 - x))
        pts_dn.append((-yt, 20.0 - x))
    return pts_up + pts_dn[::-1][1:]

with b3d.BuildPart() as fairing_assembly:
    f_pts = get_fairing_pts(chord=160.0)
    with b3d.BuildSketch(b3d.Plane.XY.offset(-5.0)):
        b3d.Polygon(f_pts)
    b3d.extrude(amount=drum_height + 18.0)

    # Hollow internal mechanism shell
    f_inner_pts = get_fairing_pts(chord=155.0)
    with b3d.BuildSketch(b3d.Plane.XY.offset(-3.5)):
        b3d.Polygon(f_inner_pts)
    b3d.extrude(amount=drum_height + 17.0, mode=b3d.Mode.SUBTRACT)

    # Flat face wing spar forward emergence slot (+Y)
    with b3d.Locations(b3d.Location((0, 20.0, z_pivot), (90, 0, 0))):
        b3d.Cylinder(radius=9.0, height=40.0, mode=b3d.Mode.SUBTRACT)

fairing_solid = fairing_assembly.part

# ------------------------------------------------------------------------------
# EXPORT GEOMETRIES
# ------------------------------------------------------------------------------
b3d.export_stl(drum_solid, os.path.join(OUT_DIR, "v4_sector_wheel_flatface_spar.stl"))
b3d.export_step(drum_solid, os.path.join(OUT_DIR, "v4_sector_wheel_flatface_spar.step"))

b3d.export_stl(wrist_solid, os.path.join(OUT_DIR, "v4_2dof_wrist_yoke.stl"))
b3d.export_step(wrist_solid, os.path.join(OUT_DIR, "v4_2dof_wrist_yoke.step"))

b3d.export_stl(spar_solid, os.path.join(OUT_DIR, "v4_spar_tube.stl"))
b3d.export_stl(motor_solid, os.path.join(OUT_DIR, "v4_goolrc5010_pinion_assembly.stl"))
b3d.export_step(motor_solid, os.path.join(OUT_DIR, "v4_goolrc5010_pinion_assembly.step"))

b3d.export_stl(fairing_solid, os.path.join(OUT_DIR, "v4_naca0018_fairing.stl"))
b3d.export_step(fairing_solid, os.path.join(OUT_DIR, "v4_naca0018_fairing.step"))

merged_core = drum_solid + wrist_solid
b3d.export_stl(merged_core, os.path.join(OUT_DIR, "v4_merged_core.stl"))
b3d.export_step(merged_core, os.path.join(OUT_DIR, "v4_merged_core.step"))

print("All V4 geometries successfully regenerated!")
