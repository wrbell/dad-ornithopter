#!/usr/bin/env python3
"""
scripts/generate_embedded_drum_wrist_v5.py
==========================================
Generates V5 Monolithic Embedded 2-DOF Drum-Face Spar Architecture:
  1. Drum height = 26.0 mm.
  2. Spar pivot axis dropped to Z = 13.0 mm (mid-plane of drum body!).
  3. Spar collar & differential bevel yoke housed directly INSIDE the drum cavity.
  4. Spar emerges directly out of the vertical flat face of the drum.
  5. Dual 6800-2RS bearings at Z=0..5mm and Z=21..26mm, directly clamping the spar loads.
  6. Zero top pylons: maximum bending strength, ultra-low 26mm profile.
"""

import math
import os
import build123d as b3d

OUT_DIR = "cad/manufactured"
os.makedirs(OUT_DIR, exist_ok=True)

R_pitch = 35.0
R_flange = 37.5
arc_deg = 150.0
rim_thickness = 8.0

r_pinion = 3.5
r_pinion_flange = 4.5
pinion_height = 12.0

r_wire_bore = 5.0
r_bearing_od = 9.5
h_bearing = 5.0
r_hub_outer = 11.5
drum_height = 26.0

# Embedded Spar Mid-Plane Pivot
z_pivot = 13.0 # mm (dead center of drum height!)
spar_od = 10.05
collar_od = 13.8

# Servos in Aft Pockets (-Y)
servo_x_offset = 14.5
servo_y_offset = -17.0
servo_w = 12.6
servo_l = 23.6
servo_d = 20.0

steps = 60
half_arc = math.radians(arc_deg / 2.0)
st_ang = -math.pi / 2.0 - half_arc
en_ang = -math.pi / 2.0 + half_arc
d_ang = (en_ang - st_ang) / steps

arc_pts = []
for s in range(steps + 1):
    ang = st_ang + s * d_ang
    arc_pts.append((R_flange * math.cos(ang), R_flange * math.sin(ang)))

flat_face_y = 10.0 # mm (flat vertical emergence face)
p_end = arc_pts[-1]
p_start = arc_pts[0]
sector_poly_pts = arc_pts + [(p_end[0], flat_face_y), (p_start[0], flat_face_y)]

with b3d.BuildPart() as monolithic_drum:
    # 1. Main Drum Solid
    with b3d.BuildSketch(b3d.Plane.XY):
        b3d.Polygon(sector_poly_pts)
    b3d.extrude(amount=drum_height)

    # 2. Capstan Cable Tracking Groove (Pitch R = 35mm, width 2.4mm, at Z=2.8mm)
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

    # 3. Central 10mm Wire Pass-Through Bore
    with b3d.BuildSketch(b3d.Plane.XY):
        b3d.Circle(radius=r_wire_bore)
    b3d.extrude(amount=drum_height + 10.0, mode=b3d.Mode.SUBTRACT, both=True)

    # 4. Precision 6800-2RS Bearing Seats (19mm OD, 5mm depth at top and bottom)
    with b3d.BuildSketch(b3d.Plane.XY):
        b3d.Circle(radius=r_bearing_od)
    b3d.extrude(amount=h_bearing + 0.2, mode=b3d.Mode.SUBTRACT)

    with b3d.BuildSketch(b3d.Plane.XY.offset(drum_height - h_bearing - 0.2)):
        b3d.Circle(radius=r_bearing_od)
    b3d.extrude(amount=h_bearing + 0.5, mode=b3d.Mode.SUBTRACT)

    # 5. INTERNAL RECESSED WRIST GIMBAL CAVITY (Inside drum body at Z=z_pivot)
    # Houses the bevel differential gears right between the bearing bosses
    with b3d.BuildSketch(b3d.Plane.XY.offset(5.5)):
        with b3d.Locations((0.0, 0.0)):
            b3d.Rectangle(width=22.0, height=18.0)
    b3d.extrude(amount=15.0, mode=b3d.Mode.SUBTRACT)

    # 6. FLAT FACE SPAR EMERGENCE SOCKET (+Y face, Z=z_pivot)
    # A conical/slotted aperture allowing spar emergence & pitch/sweep deflection
    with b3d.Locations(b3d.Location((0, flat_face_y, z_pivot), (90, 0, 0))):
        b3d.Cylinder(radius=7.5, height=12.0, mode=b3d.Mode.SUBTRACT)

    # 7. Dual Aft Servo Bays (-Y)
    for x_pos in [-servo_x_offset, servo_x_offset]:
        with b3d.BuildSketch(b3d.Plane.XY.offset(drum_height - servo_d)):
            with b3d.Locations((x_pos, servo_y_offset)):
                b3d.Rectangle(width=servo_w, height=servo_l)
        b3d.extrude(amount=servo_d + 1.0, mode=b3d.Mode.SUBTRACT)
        for dy in [-servo_l / 2.0 + 2.0, servo_l / 2.0 - 2.0]:
            with b3d.BuildSketch(b3d.Plane.XY.offset(drum_height - servo_d - 3.0)):
                with b3d.Locations((x_pos, servo_y_offset + dy)):
                    b3d.Circle(radius=1.1)
            b3d.extrude(amount=6.0, mode=b3d.Mode.SUBTRACT)

drum_solid = monolithic_drum.part

# ------------------------------------------------------------------------------
# 2. EMBEDDED DIFFERENTIAL YOKE & SPAR COLLAR (Centered at Z=13mm)
# ------------------------------------------------------------------------------
with b3d.BuildPart() as embedded_yoke:
    # Transverse Gimbal Cross-Pin (along X-axis inside cavity at Y=0, Z=z_pivot)
    with b3d.Locations(b3d.Location((0, 0, z_pivot), (0, 90, 0))):
        b3d.Cylinder(radius=3.0, height=21.0)

    # Bevel Gear Differentials
    for sign in [-1, 1]:
        with b3d.Locations(b3d.Location((sign * 8.5, 0, z_pivot), (0, 90, 0))):
            b3d.Cylinder(radius=6.5, height=2.5)

    # Spar Clamp Collar Body EXTENDING DIRECTLY OUT OF FLAT FACE (+Y)
    # Y from 0 to 28mm (protruding 18mm past the flat face at Y=10)
    with b3d.Locations(b3d.Location((0, 14.0, z_pivot), (90, 0, 0))):
        b3d.Cylinder(radius=6.8, height=28.0)

    # Spar Bore (10.05mm ID)
    with b3d.Locations(b3d.Location((0, 14.0, z_pivot), (90, 0, 0))):
        b3d.Cylinder(radius=spar_od / 2.0, height=30.0, mode=b3d.Mode.SUBTRACT)

    # Spar clamping slit
    with b3d.BuildSketch(b3d.Plane.XY.offset(z_pivot - 1.0)):
        with b3d.Locations((0.0, 14.0)):
            b3d.Rectangle(width=1.0, height=26.0)
    b3d.extrude(amount=8.0, mode=b3d.Mode.SUBTRACT)

yoke_solid = embedded_yoke.part

# ------------------------------------------------------------------------------
# 3. CARBON FIBER WING SPAR TUBE (Emerging from Drum Face into Wing)
# ------------------------------------------------------------------------------
with b3d.BuildPart() as spar_tube:
    # Spar from Y = 12mm out to Y = 112mm
    with b3d.Locations(b3d.Location((0, 62.0, z_pivot), (90, 0, 0))):
        b3d.Cylinder(radius=spar_od / 2.0, height=100.0)
    with b3d.Locations(b3d.Location((0, 62.0, z_pivot), (90, 0, 0))):
        b3d.Cylinder(radius=4.0, height=102.0, mode=b3d.Mode.SUBTRACT)

spar_solid = spar_tube.part

# ------------------------------------------------------------------------------
# 4. COMPACT NACA 0018 FAIRING (Ultra-Low 30mm Profile!)
# ------------------------------------------------------------------------------
def get_fairing_pts(chord=150.0, num_points=60):
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
        pts_up.append((yt, 18.0 - x))
        pts_dn.append((-yt, 18.0 - x))
    return pts_up + pts_dn[::-1][1:]

with b3d.BuildPart() as fairing_body:
    f_pts = get_fairing_pts(chord=150.0)
    with b3d.BuildSketch(b3d.Plane.XY.offset(-2.0)):
        b3d.Polygon(f_pts)
    b3d.extrude(amount=drum_height + 4.0) # Only 30mm total fairing height!

    f_inner_pts = get_fairing_pts(chord=146.0)
    with b3d.BuildSketch(b3d.Plane.XY.offset(-1.0)):
        b3d.Polygon(f_inner_pts)
    b3d.extrude(amount=drum_height + 3.0, mode=b3d.Mode.SUBTRACT)

    # Flat face spar emergence aperture
    with b3d.Locations(b3d.Location((0, 18.0, z_pivot), (90, 0, 0))):
        b3d.Cylinder(radius=8.5, height=35.0, mode=b3d.Mode.SUBTRACT)

fairing_solid = fairing_body.part

# Export
b3d.export_stl(drum_solid, os.path.join(OUT_DIR, "v5_embedded_sector_drum.stl"))
b3d.export_step(drum_solid, os.path.join(OUT_DIR, "v5_embedded_sector_drum.step"))

b3d.export_stl(yoke_solid, os.path.join(OUT_DIR, "v5_embedded_wrist_yoke.stl"))
b3d.export_step(yoke_solid, os.path.join(OUT_DIR, "v5_embedded_wrist_yoke.step"))

b3d.export_stl(spar_solid, os.path.join(OUT_DIR, "v5_embedded_spar.stl"))
b3d.export_stl(fairing_solid, os.path.join(OUT_DIR, "v5_embedded_fairing.stl"))
b3d.export_step(fairing_solid, os.path.join(OUT_DIR, "v5_embedded_fairing.step"))

merged_v5 = drum_solid + yoke_solid
b3d.export_stl(merged_v5, os.path.join(OUT_DIR, "v5_monolithic_merged_core.stl"))
b3d.export_step(merged_v5, os.path.join(OUT_DIR, "v5_monolithic_merged_core.step"))

print(f"V5 Monolithic Embedded Drum-Face Spar Core generated successfully!")
print(f"Drum Height: {drum_height:.1f} mm | Spar Elevation: {z_pivot:.1f} mm")
print(f"Total Structural Core Volume: {merged_v5.volume/1000.0:.2f} cm³ (Mass in PA12-CF: {merged_v5.volume/1000.0*1.25:.1f} g)")
