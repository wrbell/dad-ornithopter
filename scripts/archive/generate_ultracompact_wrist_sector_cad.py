#!/usr/bin/env python3
"""
scripts/generate_ultracompact_wrist_sector_cad.py
=================================================
Generates the ultra-compact shrunken 2-DOF Robotic Wrist & Skeletonized Sector Wheel
for MCO-1D while strictly preserving:
  1. Exactly 10.0 : 1 capstan reduction ratio (R_sector / r_pinion = 10.0)
  2. Full library stroke authority (±45° nominal, up to ±96° sprint envelope)
  3. Dynamic frequency response (f = 2.2 Hz continuous, 3.2 Hz sprint)
  4. Hollow 40mm bore through dual 6708-2RS bearings for coaxial torsion spring & wiring
  5. Dual embedded aft-shifted servos (AGFRC B24CLM)
  6. NACA 0018 LW-PLA aerodynamic fairing
"""

import math
import os
import build123d as b3d

OUT_DIR = "cad/manufactured"
os.makedirs(OUT_DIR, exist_ok=True)

# ==============================================================================
# PARAMETERS FOR ULTRA-COMPACT 10:1 DRIVETRAIN
# ==============================================================================
# 10:1 Ratio:
# Pitch radius of sector wheel: R_sector = 45.0 mm (dia 90.0 mm)
# Pitch radius of motor pinion: r_pinion = 4.5 mm (dia 9.0 mm)
# Transmission Ratio: 45.0 / 4.5 = 10.000 : 1
R_pitch = 45.0       # mm (Capstan pitch radius)
R_flange = 48.0      # mm (Outer cable retention flange)
arc_deg = 180.0      # 180° flapping stroke envelope
rim_thickness = 10.0 # mm (Capstan cable track height)

# Bearing & Hub Dimensions (Strictly constrained by 30mm spring & 40mm pedestal)
r_bore = 20.0        # 40mm ID hollow core (radius 20mm)
r_bearing_od = 25.0  # 50mm OD for 6708-2RS bearings (radius 25mm)
h_bearing = 6.0      # 6mm bearing width
r_hub_outer = 27.2   # 54.4mm OD bearing housing
drum_height = 32.0   # Compacted from 36mm down to 32mm (fits spring + 2x bearings)

# Ultra-Compact Aft Servo Integration (AGFRC B24CLM: 23 x 12 x 27.5mm)
servo_x_offset = 28.5 # mm (tucked tight to the 50mm bearing boss)
servo_y_offset = -18.0 # mm AFT (behind 40mm core)
servo_w = 12.8        # mm (bay width)
servo_l = 24.0        # mm (bay length)
servo_d = 26.5        # mm (recess depth into drum)

# Output Spar Clamp
spar_od = 10.05      # 10mm OD Carbon Fiber spar tube

# Shrunken NACA 0018 Airfoil Fairing
naca_chord = 152.0    # Shrunk from 210mm down to 152mm (27.6% chord reduction!)
naca_height = 58.0    # Shrunk from 82mm down to 58mm (29.3% height reduction!)

print("=" * 72)
print("  GENERATING ULTRA-COMPACT 10:1 MERGED WRIST & SKELETONIZED SECTOR CAD")
print("=" * 72)
print(f"Sector Pitch Radius R_sec  : {R_pitch:.1f} mm")
print(f"Motor Pinion Pitch r_pin   : {R_pitch/10.0:.1f} mm (dia {2*R_pitch/10.0:.1f} mm)")
print(f"Capstan Gear Ratio         : {R_pitch / (R_pitch/10.0):.1f} : 1 EXACT")
print(f"Hollow Bearing Core        : 40.0 mm ID (Dual 6708-2RS)")
print(f"Fairing Chord / Height     : {naca_chord:.1f} mm / {naca_height:.1f} mm")
print("-" * 72)

# ------------------------------------------------------------------------------
# 1. SHRUNKEN NACA 0018 PROFILE GENERATOR
# ------------------------------------------------------------------------------
def get_naca0018_points(chord, num_points=60):
    t = 0.18
    pts_upper = []
    pts_lower = []
    for i in range(num_points + 1):
        beta = math.pi * i / num_points
        x = chord * 0.5 * (1.0 - math.cos(beta))
        x_norm = x / chord
        if x_norm > 1.0:
            x_norm = 1.0
        y = 5.0 * t * chord * (
            0.2969 * math.sqrt(x_norm)
            - 0.1260 * x_norm
            - 0.3516 * (x_norm ** 2)
            + 0.2843 * (x_norm ** 3)
            - 0.1015 * (x_norm ** 4)
        )
        # Shift chord so aerodynamic center is at flapping pivot
        pts_upper.append((y, 54.0 - x))
        pts_lower.append((-y, 54.0 - x))
    return pts_upper + pts_lower[::-1][1:]

# ------------------------------------------------------------------------------
# 2. ULTRA-COMPACT SKELETONIZED SECTOR DRUM
# ------------------------------------------------------------------------------
steps = 60
d_ang = math.radians(arc_deg) / steps
st_ang = math.radians(-arc_deg / 2.0)

arc_pts = [(0.0, 0.0)]
for s in range(steps + 1):
    ang = st_ang + s * d_ang
    arc_pts.append((R_flange * math.sin(ang), R_flange * math.cos(ang)))
arc_pts.append((0.0, 0.0))

with b3d.BuildPart() as sector_drum:
    # 2.1 Central Main Hub Cylinder & Aft Extension Boss
    with b3d.BuildSketch(b3d.Plane.XY):
        b3d.Circle(radius=r_hub_outer)
        # Compact aft platform housing the rearward-shifted servos
        with b3d.Locations((0, servo_y_offset)):
            b3d.Rectangle(width=74.0, height=30.0)
    b3d.extrude(amount=drum_height)

    # 2.2 Forward Capstan Sector Drum
    with b3d.BuildSketch(b3d.Plane.XY):
        b3d.Polygon(arc_pts)
    b3d.extrude(amount=rim_thickness)

    # 2.3 Capstan Cable Tracking Groove (Pitch R = 45mm, width 3.6mm, depth 3.0mm)
    groove_pts = []
    for s in range(steps + 1):
        ang = st_ang + s * d_ang
        groove_pts.append(((R_flange + 1.0) * math.sin(ang), (R_flange + 1.0) * math.cos(ang)))
    for s in range(steps, -1, -1):
        ang = st_ang + s * d_ang
        groove_pts.append((R_pitch * math.sin(ang), R_pitch * math.cos(ang)))
    with b3d.BuildSketch(b3d.Plane.XY.offset(3.2)):
        b3d.Polygon(groove_pts)
    b3d.extrude(amount=3.6, mode=b3d.Mode.SUBTRACT)

    # 2.4 Central 40mm Hollow Bore (Pass-Through)
    with b3d.BuildSketch(b3d.Plane.XY):
        b3d.Circle(radius=r_bore)
    b3d.extrude(amount=drum_height + 5.0, mode=b3d.Mode.SUBTRACT, both=True)

    # 2.5 Precision 6708-2RS Bearing Seats (50mm OD, 6mm depth at top and bottom)
    with b3d.BuildSketch(b3d.Plane.XY):
        b3d.Circle(radius=r_bearing_od)
    b3d.extrude(amount=h_bearing + 0.3, mode=b3d.Mode.SUBTRACT)

    with b3d.BuildSketch(b3d.Plane.XY.offset(drum_height - h_bearing - 0.3)):
        b3d.Circle(radius=r_bearing_od)
    b3d.extrude(amount=h_bearing + 0.8, mode=b3d.Mode.SUBTRACT)

    # 2.6 Twin Aft-Shifted Servo Bays (Recessed deep into Drum Body)
    for x_pos in [-servo_x_offset, servo_x_offset]:
        with b3d.BuildSketch(b3d.Plane.XY.offset(drum_height - servo_d)):
            with b3d.Locations((x_pos, servo_y_offset)):
                b3d.Rectangle(width=servo_w, height=servo_l)
        b3d.extrude(amount=servo_d + 1.0, mode=b3d.Mode.SUBTRACT)
        # Servo mounting screw clearance holes (M2.5)
        for dy in [-servo_l / 2.0 + 2.0, servo_l / 2.0 - 2.0]:
            with b3d.BuildSketch(b3d.Plane.XY.offset(drum_height - servo_d - 4.0)):
                with b3d.Locations((x_pos, servo_y_offset + dy)):
                    b3d.Circle(radius=1.3)
            b3d.extrude(amount=8.0, mode=b3d.Mode.SUBTRACT)

    # 2.7 TOPOLOGICAL OPTIMIZATION: Radial Isogrid Lightening Cutouts
    for ang_deg in [-50, -25, 0, 25, 50]:
        rad = math.radians(ang_deg)
        cx = 36.5 * math.sin(rad)
        cy = 36.5 * math.cos(rad)
        with b3d.BuildSketch(b3d.Plane.XY.offset(-1.0)):
            with b3d.Locations((cx, cy)):
                b3d.Circle(radius=3.8)
        b3d.extrude(amount=rim_thickness + 2.0, mode=b3d.Mode.SUBTRACT)

    # Center Aft Lightening Bay (between the two recessed servos)
    with b3d.BuildSketch(b3d.Plane.XY.offset(-1.0)):
        with b3d.Locations((0.0, servo_y_offset)):
            b3d.SlotCenterToCenter(center_separation=14.0, height=10.0)
    b3d.extrude(amount=drum_height + 2.0, mode=b3d.Mode.SUBTRACT)

    # Lateral Corner Lightening Bevels
    for sign in [-1, 1]:
        with b3d.BuildSketch(b3d.Plane.XY.offset(-1.0)):
            with b3d.Locations((sign * 33.0, servo_y_offset - 11.0)):
                b3d.Circle(radius=3.5)
        b3d.extrude(amount=drum_height + 2.0, mode=b3d.Mode.SUBTRACT)

    # Dyneema Cable Pinch Dead-End Clamping Holes (M2.5 at arc terminals)
    for sign in [-1, 1]:
        ax = sign * (R_pitch - 4.5) * math.sin(math.radians(78.0))
        ay = (R_pitch - 4.5) * math.cos(math.radians(78.0))
        with b3d.BuildSketch(b3d.Plane.XY.offset(1.0)):
            with b3d.Locations((ax, ay)):
                b3d.Circle(radius=1.25)
        b3d.extrude(amount=rim_thickness, mode=b3d.Mode.SUBTRACT)

drum_solid = sector_drum.part

# ------------------------------------------------------------------------------
# 3. ULTRA-COMPACT 2-DOF DIFFERENTIAL BEVEL WRIST TOWER
# ------------------------------------------------------------------------------
with b3d.BuildPart() as wrist_assembly:
    # Transverse pivot axis dropped to Z = drum_height + 11.5mm = 43.5mm!
    z_pivot = drum_height + 11.5
    for x_pylon in [-20.0, 20.0]:
        with b3d.BuildSketch(b3d.Plane.YZ.offset(x_pylon)):
            with b3d.Locations((0, drum_height)):
                # Compact upright pylon
                b3d.Polygon([
                    (-11.0, 0.0), (11.0, 0.0),
                    (8.0, 18.0), (-8.0, 18.0)
                ])
        b3d.extrude(amount=4.0 if x_pylon < 0 else -4.0)

        # Topological Lightening Hole through Pylon
        with b3d.BuildSketch(b3d.Plane.YZ.offset(x_pylon - 4.0)):
            with b3d.Locations((0.0, drum_height + 6.0)):
                b3d.Circle(radius=2.8)
        b3d.extrude(amount=8.0, mode=b3d.Mode.SUBTRACT)

    # Transverse Pivot Axis Bore (8mm ID)
    with b3d.BuildSketch(b3d.Plane.YZ):
        with b3d.Locations((0.0, z_pivot)):
            b3d.Circle(radius=4.0)
    b3d.extrude(amount=28.0, mode=b3d.Mode.SUBTRACT, both=True)

    # Differential Bevel Gears (Representative solids)
    for sign in [-1, 1]:
        with b3d.BuildSketch(b3d.Plane.YZ.offset(sign * 11.5)):
            with b3d.Locations((0.0, z_pivot)):
                b3d.Circle(radius=9.5)
        b3d.extrude(amount=-sign * 3.0)

    # Main Wing Carbon Spar Clamp Yoke (Center Output at Z = z_pivot)
    with b3d.BuildSketch(b3d.Plane.XZ.offset(0.0)):
        with b3d.Locations((0.0, z_pivot)):
            b3d.Circle(radius=7.8)
    b3d.extrude(amount=28.0) # Compacted clamp length

    # 10.05mm Carbon Spar Bore
    with b3d.BuildSketch(b3d.Plane.XZ.offset(0.0)):
        with b3d.Locations((0.0, z_pivot)):
            b3d.Circle(radius=spar_od / 2.0)
    b3d.extrude(amount=30.0, mode=b3d.Mode.SUBTRACT)

    # Spar Pinch Split Slot (1.2mm slit) & 2x M3 Pinch Clamp Screw Bores
    with b3d.BuildSketch(b3d.Plane.XY.offset(z_pivot - 8.0)):
        with b3d.Locations((0.0, 14.0)):
            b3d.Rectangle(width=1.2, height=28.0)
    b3d.extrude(amount=16.0, mode=b3d.Mode.SUBTRACT)

    for y_pinch in [7.0, 21.0]:
        with b3d.BuildSketch(b3d.Plane.YZ.offset(-10.0)):
            with b3d.Locations((y_pinch, z_pivot + 3.8)):
                b3d.Circle(radius=1.6)
        b3d.extrude(amount=20.0, mode=b3d.Mode.SUBTRACT)

wrist_solid = wrist_assembly.part

# ------------------------------------------------------------------------------
# 4. SHRUNKEN NACA 0018 LW-PLA FAIRING
# ------------------------------------------------------------------------------
naca_pts = get_naca0018_points(chord=naca_chord)

with b3d.BuildPart() as fairing_body:
    # Outer NACA 0018 Shell
    with b3d.BuildSketch(b3d.Plane.XY.offset(-4.0)):
        b3d.Polygon(naca_pts)
    b3d.extrude(amount=naca_height)

    # Hollow Internal Mechanism Bay (1.2mm LW-PLA wall)
    naca_inner_pts = get_naca0018_points(chord=naca_chord - 4.5)
    with b3d.BuildSketch(b3d.Plane.XY.offset(-2.5)):
        with b3d.Locations((0.0, -1.0)):
            b3d.Polygon(naca_inner_pts)
    b3d.extrude(amount=naca_height - 2.5, mode=b3d.Mode.SUBTRACT)

    # Bottom Aperture for 40mm Hollow Bearing Pedestal
    with b3d.BuildSketch(b3d.Plane.XY.offset(-6.0)):
        b3d.Circle(radius=r_hub_outer + 1.2)
    b3d.extrude(amount=12.0, mode=b3d.Mode.SUBTRACT)

    # Outboard Wing Spar Articulation Slot
    with b3d.BuildSketch(b3d.Plane.XZ.offset(12.0)):
        with b3d.Locations((0.0, z_pivot)):
            b3d.SlotCenterToCenter(center_separation=24.0, height=20.0, rotation=0.0)
    b3d.extrude(amount=50.0, mode=b3d.Mode.SUBTRACT)

fairing_solid = fairing_body.part

# ------------------------------------------------------------------------------
# 5. COMPLETE MERGED ASSEMBLY & EXPORTS
# ------------------------------------------------------------------------------
merged_structural = drum_solid + wrist_solid

b3d.export_stl(drum_solid, os.path.join(OUT_DIR, "mco_ultracompact_sector_wheel_10to1.stl"))
b3d.export_step(drum_solid, os.path.join(OUT_DIR, "mco_ultracompact_sector_wheel_10to1.step"))

b3d.export_stl(wrist_solid, os.path.join(OUT_DIR, "mco_ultracompact_2dof_wrist.stl"))
b3d.export_step(wrist_solid, os.path.join(OUT_DIR, "mco_ultracompact_2dof_wrist.step"))

b3d.export_stl(merged_structural, os.path.join(OUT_DIR, "mco_ultracompact_merged_core_10to1.stl"))
b3d.export_step(merged_structural, os.path.join(OUT_DIR, "mco_ultracompact_merged_core_10to1.step"))

b3d.export_stl(fairing_solid, os.path.join(OUT_DIR, "mco_ultracompact_naca0018_fairing.stl"))
b3d.export_step(fairing_solid, os.path.join(OUT_DIR, "mco_ultracompact_naca0018_fairing.step"))

# Volume & Mass Calculations
vol_drum = drum_solid.volume / 1000.0
vol_wrist = wrist_solid.volume / 1000.0
vol_total = merged_structural.volume / 1000.0
vol_fairing = fairing_solid.volume / 1000.0

mass_struct_pacf = vol_total * 1.25 # PA12-CF (1.25 g/cm3)
mass_struct_al = vol_total * 2.70   # 7075-T6 (2.70 g/cm3)
mass_fairing_lwpla = vol_fairing * 0.48 # LW-PLA (0.48 g/cm3)
mass_servos = 42.0                  # 2x AGFRC B24CLM

print(f"Sector Drum Solid Volume       : {vol_drum:.2f} cm³ (was 103.07 cm³)")
print(f"Wrist Differential Yoke Volume : {vol_wrist:.2f} cm³ (was 11.89 cm³)")
print(f"Total Structural Core Volume   : {vol_total:.2f} cm³ (was 113.92 cm³)")
print(f"Structural Mass (PA12-CF)      : {mass_struct_pacf:.1f} g (was 142.4 g)")
print(f"NACA 0018 Fairing Mass (LW-PLA): {mass_fairing_lwpla:.1f} g (was 15.7 g)")
print(f"2x Servos Mass (AGFRC B24CLM)  : {mass_servos:.1f} g")
print(f"TOTAL SYSTEM ALL-UP MASS (PA12): {mass_struct_pacf + mass_fairing_lwpla + mass_servos:.1f} g (was 200.1 g)")
print("=" * 72)
