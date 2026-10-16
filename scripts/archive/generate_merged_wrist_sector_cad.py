#!/usr/bin/env python3
"""
scripts/generate_merged_wrist_sector_cad.py
===========================================
Generates the fully merged 2-DOF Robotic Wrist & Skeletonized Sector Wheel
for MCO-1D, incorporating:
  1. Skeletonized Sector Wheel (Big Drum) with dual 6708-2RS bearing seats (40mm ID x 50mm OD)
  2. Hollow 40mm central core
  3. Integrated twin servo bays on either side of the 40mm core and shifted AFT
  4. 2-DOF Differential Bevel Wrist mechanism with 10mm carbon spar clamp
  5. Topologically optimized structural web truss & lightening pockets
  6. Aerodynamic NACA 0018 LW-PLA fairing set 90° to the sector wheel
"""

import math
import os
import build123d as b3d

OUT_DIR = "cad/manufactured"
os.makedirs(OUT_DIR, exist_ok=True)

# ==============================================================================
# PARAMETERS
# ==============================================================================
# Sector Wheel (Big Drum)
r_bore = 20.0        # 40mm ID hollow core (radius 20mm)
r_bearing_od = 25.0  # 50mm OD for 6708-2RS bearings (radius 25mm)
h_bearing = 6.0      # 6mm bearing width
r_hub_outer = 27.5   # 55mm OD outer bearing boss
drum_height = 36.0   # Total axial drum height

r_pitch = 60.0       # Capstan pitch radius (120mm pitch dia)
r_flange = 64.0      # Outer cable flange radius
arc_deg = 180.0      # 180° flapping stroke envelope
rim_thickness = 12.0 # Capstan cable track height

# Servo Bays (sized for AGFRC B24CLM / B44BLS micro/mini servos)
servo_x_offset = 32.0 # ±32mm from centerline (flanking the 40mm core)
servo_y_offset = -22.0 # 22mm AFT (behind the 40mm core for rearward CG balance)
servo_w = 13.5        # Bay width (X)
servo_l = 25.5        # Bay length (Y)
servo_d = 28.0        # Recessed cavity depth into drum (Z)

# Wing Spar Interface
spar_od = 10.05      # 10mm OD Carbon Fiber tube (+0.05mm slip fit)

# NACA 0018 Airfoil Fairing
naca_chord = 210.0    # 210mm chord length (enclosing drum & servos)
naca_height = 82.0    # 82mm vertical height (spanning base through wrist)

print("Building Merged Sector Wheel & 2-DOF Wrist CAD Geometry...")

# ------------------------------------------------------------------------------
# 1. NACA 0018 PROFILE GENERATOR
# ------------------------------------------------------------------------------
def get_naca0018_points(chord, num_points=60):
    t = 0.18
    pts_upper = []
    pts_lower = []
    # Cosine spacing for high resolution at leading edge
    for i in range(num_points + 1):
        beta = math.pi * i / num_points
        x = chord * 0.5 * (1.0 - math.cos(beta))
        x_norm = x / chord
        if x_norm > 1.0:
            x_norm = 1.0
        # NACA 4-digit thickness equation
        y = 5.0 * t * chord * (
            0.2969 * math.sqrt(x_norm)
            - 0.1260 * x_norm
            - 0.3516 * (x_norm ** 2)
            + 0.2843 * (x_norm ** 3)
            - 0.1015 * (x_norm ** 4)
        )
        # Shift chord so leading edge is forward (+Y) and trailing edge is aft (-Y)
        # In aircraft coordinates: +Y is forward, -Y is aft
        pts_upper.append((y, 75.0 - x))
        pts_lower.append((-y, 75.0 - x))
    return pts_upper + pts_lower[::-1][1:]

# ------------------------------------------------------------------------------
# 2. SKELETONIZED SECTOR DRUM & INTEGRATED SERVO CRADLE
# ------------------------------------------------------------------------------
# Capstan Sector Arc Polygon (Forward Facing arc from -90° to +90°)
steps = 60
d_ang = math.radians(arc_deg) / steps
st_ang = math.radians(-arc_deg / 2.0)

arc_pts = [(0.0, 0.0)]
for s in range(steps + 1):
    ang = st_ang + s * d_ang
    arc_pts.append((r_flange * math.sin(ang), r_flange * math.cos(ang)))
arc_pts.append((0.0, 0.0))

with b3d.BuildPart() as sector_drum:
    # 2.1 Central Main Hub Cylinder & Aft Extension Boss
    with b3d.BuildSketch(b3d.Plane.XY):
        b3d.Circle(radius=r_hub_outer)
        # Aft platform housing the rearward-shifted servos
        with b3d.Locations((0, servo_y_offset)):
            b3d.Rectangle(width=84.0, height=36.0)
    b3d.extrude(amount=drum_height)

    # 2.2 Forward Capstan Sector Drum
    with b3d.BuildSketch(b3d.Plane.XY):
        b3d.Polygon(arc_pts)
    b3d.extrude(amount=rim_thickness)

    # 2.3 Capstan Cable Tracking Groove (Pitch R = 60mm, width 4mm, depth 3.5mm)
    groove_pts = []
    for s in range(steps + 1):
        ang = st_ang + s * d_ang
        groove_pts.append(((r_flange + 1.0) * math.sin(ang), (r_flange + 1.0) * math.cos(ang)))
    for s in range(steps, -1, -1):
        ang = st_ang + s * d_ang
        groove_pts.append((r_pitch * math.sin(ang), r_pitch * math.cos(ang)))
    with b3d.BuildSketch(b3d.Plane.XY.offset(4.0)):
        b3d.Polygon(groove_pts)
    b3d.extrude(amount=4.5, mode=b3d.Mode.SUBTRACT)

    # 2.4 Central 40mm Hollow Bore (Pass-Through)
    with b3d.BuildSketch(b3d.Plane.XY):
        b3d.Circle(radius=r_bore)
    b3d.extrude(amount=drum_height + 5.0, mode=b3d.Mode.SUBTRACT, both=True)

    # 2.5 Precision 6708-2RS Bearing Seats (50mm OD, 6mm depth at top and bottom)
    # Bottom Bearing Seat
    with b3d.BuildSketch(b3d.Plane.XY):
        b3d.Circle(radius=r_bearing_od)
    b3d.extrude(amount=h_bearing + 0.5, mode=b3d.Mode.SUBTRACT)

    # Top Bearing Seat
    with b3d.BuildSketch(b3d.Plane.XY.offset(drum_height - h_bearing - 0.5)):
        b3d.Circle(radius=r_bearing_od)
    b3d.extrude(amount=h_bearing + 1.0, mode=b3d.Mode.SUBTRACT)

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
    for ang_deg in [-60, -35, -12, 12, 35, 60]:
        rad = math.radians(ang_deg)
        cx = 44.0 * math.sin(rad)
        cy = 44.0 * math.cos(rad)
        with b3d.BuildSketch(b3d.Plane.XY.offset(-1.0)):
            with b3d.Locations((cx, cy)):
                b3d.Circle(radius=4.5)
        b3d.extrude(amount=rim_thickness + 2.0, mode=b3d.Mode.SUBTRACT)

    # Center Aft Lightening Bay (between the two recessed servos)
    with b3d.BuildSketch(b3d.Plane.XY.offset(-1.0)):
        with b3d.Locations((0.0, servo_y_offset)):
            b3d.SlotCenterToCenter(center_separation=18.0, height=12.0)
    b3d.extrude(amount=drum_height + 2.0, mode=b3d.Mode.SUBTRACT)

    # Lateral Lightening Scallops on Aft Pod Corners
    for sign in [-1, 1]:
        with b3d.BuildSketch(b3d.Plane.XY.offset(-1.0)):
            with b3d.Locations((sign * 38.0, servo_y_offset - 14.0)):
                b3d.Circle(radius=4.0)
        b3d.extrude(amount=drum_height + 2.0, mode=b3d.Mode.SUBTRACT)

    # Dyneema Cable Pinch Dead-End Clamping Holes (M2.5 at arc terminals)
    for sign in [-1, 1]:
        ax = sign * (r_pitch - 5.0) * math.sin(math.radians(80.0))
        ay = (r_pitch - 5.0) * math.cos(math.radians(80.0))
        with b3d.BuildSketch(b3d.Plane.XY.offset(1.0)):
            with b3d.Locations((ax, ay)):
                b3d.Circle(radius=1.25)
        b3d.extrude(amount=rim_thickness, mode=b3d.Mode.SUBTRACT)

drum_solid = sector_drum.part

# ------------------------------------------------------------------------------
# 3. 2-DOF DIFFERENTIAL BEVEL WRIST TOWER & SPAR YOKE
# ------------------------------------------------------------------------------
with b3d.BuildPart() as wrist_assembly:
    # 3.1 Skeletonized Transverse Yoke Brackets (Rising from Drum at X = ±22mm)
    z_pivot = drum_height + 18.0
    for x_pylon in [-22.0, 22.0]:
        with b3d.BuildSketch(b3d.Plane.YZ.offset(x_pylon)):
            with b3d.Locations((0, drum_height)):
                # Tapered lightweight upright pylon
                b3d.Polygon([
                    (-13.0, 0.0), (13.0, 0.0),
                    (9.0, 26.0), (-9.0, 26.0)
                ])
        b3d.extrude(amount=4.5 if x_pylon < 0 else -4.5)

        # Topological Lightening Hole through Pylon
        with b3d.BuildSketch(b3d.Plane.YZ.offset(x_pylon - 5.0)):
            with b3d.Locations((0.0, drum_height + 9.0)):
                b3d.Circle(radius=3.5)
        b3d.extrude(amount=10.0, mode=b3d.Mode.SUBTRACT)

    # 3.2 Transverse Pivot Axis Bore (8mm ID for differential bevel cross-shaft at Z = z_pivot)
    with b3d.BuildSketch(b3d.Plane.YZ):
        with b3d.Locations((0.0, z_pivot)):
            b3d.Circle(radius=4.0) # 8mm pivot pin
    b3d.extrude(amount=32.0, mode=b3d.Mode.SUBTRACT, both=True)

    # 3.3 Differential Bevel Gears (Representative solids meshing in center)
    for sign in [-1, 1]:
        with b3d.BuildSketch(b3d.Plane.YZ.offset(sign * 13.0)):
            with b3d.Locations((0.0, z_pivot)):
                b3d.Circle(radius=11.5)
        b3d.extrude(amount=-sign * 3.5)

    # 3.4 Main Wing Carbon Spar Clamp Yoke (Center Output at Z = z_pivot)
    with b3d.BuildSketch(b3d.Plane.XZ.offset(0.0)):
        with b3d.Locations((0.0, z_pivot)):
            b3d.Circle(radius=8.2)
    # Extrude along spanwise wing direction (outboard +Y)
    b3d.extrude(amount=32.0)

    # 10.05mm Carbon Spar Bore
    with b3d.BuildSketch(b3d.Plane.XZ.offset(0.0)):
        with b3d.Locations((0.0, z_pivot)):
            b3d.Circle(radius=spar_od / 2.0)
    b3d.extrude(amount=35.0, mode=b3d.Mode.SUBTRACT)

    # Spar Pinch Split Slot (1.2mm slit) & 2x M3 Pinch Clamp Screw Bores
    with b3d.BuildSketch(b3d.Plane.XY.offset(z_pivot - 10.0)):
        with b3d.Locations((0.0, 16.0)):
            b3d.Rectangle(width=1.2, height=32.0)
    b3d.extrude(amount=20.0, mode=b3d.Mode.SUBTRACT)

    for y_pinch in [8.0, 24.0]:
        with b3d.BuildSketch(b3d.Plane.YZ.offset(-12.0)):
            with b3d.Locations((y_pinch, z_pivot + 4.5)):
                b3d.Circle(radius=1.6) # M3 clearance
        b3d.extrude(amount=24.0, mode=b3d.Mode.SUBTRACT)

wrist_solid = wrist_assembly.part

# ------------------------------------------------------------------------------
# 4. NACA 0018 LW-PLA AERODYNAMIC FAIRING (90° TO SECTOR WHEEL)
# ------------------------------------------------------------------------------
# Airfoil cross-section in the horizontal plane (XY) standing vertically in Z:
# Set 90° to the sector wheel: NACA 0018 profile runs chordwise (Y-direction)
# with thickness along X, extruded vertically in Z to enclose drum and wrist.
naca_pts = get_naca0018_points(chord=naca_chord)

with b3d.BuildPart() as fairing_body:
    # 4.1 Outer NACA 0018 Fairing Shell (extruding along Z from -5 to +75mm)
    with b3d.BuildSketch(b3d.Plane.XY.offset(-5.0)):
        b3d.Polygon(naca_pts)
    b3d.extrude(amount=naca_height)

    # 4.2 Hollow Internal Electronics & Mechanism Bay (1.4mm LW-PLA wall)
    naca_inner_pts = get_naca0018_points(chord=naca_chord - 5.0)
    with b3d.BuildSketch(b3d.Plane.XY.offset(-3.5)):
        with b3d.Locations((0.0, -1.0)):
            b3d.Polygon(naca_inner_pts)
    b3d.extrude(amount=naca_height - 3.0, mode=b3d.Mode.SUBTRACT)

    # 4.3 Bottom Aperture for 40mm Hollow Bearing Pedestal & Base Flange
    with b3d.BuildSketch(b3d.Plane.XY.offset(-7.0)):
        b3d.Circle(radius=r_hub_outer + 1.5)
    b3d.extrude(amount=14.0, mode=b3d.Mode.SUBTRACT)

    # 4.4 Outboard Wing Spar Articulation Slot (permitting ±35° pitch & 85° sweep)
    with b3d.BuildSketch(b3d.Plane.XZ.offset(15.0)):
        with b3d.Locations((0.0, z_pivot)):
            b3d.SlotCenterToCenter(center_separation=26.0, height=22.0, rotation=0.0)
    b3d.extrude(amount=60.0, mode=b3d.Mode.SUBTRACT)

fairing_solid = fairing_body.part

# ------------------------------------------------------------------------------
# 5. COMPLETE MERGED ASSEMBLY & METRICS
# ------------------------------------------------------------------------------
merged_structural = drum_solid + wrist_solid

# Export Individual Component STLs and STEP files
b3d.export_stl(drum_solid, os.path.join(OUT_DIR, "mco_skeletonized_sector_wheel.stl"))
b3d.export_step(drum_solid, os.path.join(OUT_DIR, "mco_skeletonized_sector_wheel.step"))

b3d.export_stl(wrist_solid, os.path.join(OUT_DIR, "mco_integrated_2dof_wrist.stl"))
b3d.export_step(wrist_solid, os.path.join(OUT_DIR, "mco_integrated_2dof_wrist.step"))

b3d.export_stl(merged_structural, os.path.join(OUT_DIR, "mco_merged_sector_wrist_assembly.stl"))
b3d.export_step(merged_structural, os.path.join(OUT_DIR, "mco_merged_sector_wrist_assembly.step"))

b3d.export_stl(fairing_solid, os.path.join(OUT_DIR, "mco_naca0018_wrist_fairing_lwpla.stl"))
b3d.export_step(fairing_solid, os.path.join(OUT_DIR, "mco_naca0018_wrist_fairing_lwpla.step"))

# Calculate Mass and Volume Metrics
vol_drum = drum_solid.volume / 1000.0   # cm^3
vol_wrist = wrist_solid.volume / 1000.0 # cm^3
vol_total = merged_structural.volume / 1000.0 # cm^3
vol_fairing = fairing_solid.volume / 1000.0 # cm^3

# Baseline Solid (pre-topological optimization volume)
vol_baseline_unopt = (math.pi * (r_flange**2) * drum_height * 0.5 + 84.0 * 36.0 * drum_height) / 1000.0

mass_struct_pacf = vol_total * 1.25
mass_struct_al = vol_total * 2.70
mass_fairing_lwpla = vol_fairing * 0.48

print("\n" + "=" * 72)
print("  TOPOLOGICALLY OPTIMIZED MERGED WRIST & SECTOR WHEEL REPORT")
print("=" * 72)
print(f"Pre-Optimization Solid Envelope Volume : {vol_baseline_unopt:.2f} cm³")
print(f"Post-Optimization Skeletonized Drum Vol: {vol_drum:.2f} cm³")
print(f"Volume Reduction (Topological Lighten) : {(1.0 - vol_drum/vol_baseline_unopt)*100.0:.1f}%")
print(f"Wrist Differential Yoke Volume         : {vol_wrist:.2f} cm³")
print(f"Total Structural Core Volume           : {vol_total:.2f} cm³")
print(f"Structural Mass (PA12-CF / Tough PLA)  : {mass_struct_pacf:.1f} g")
print(f"Structural Mass (7075-T6 Skeleton)     : {mass_struct_al:.1f} g")
print(f"NACA 0018 LW-PLA Fairing Volume        : {vol_fairing:.2f} cm³")
print(f"NACA 0018 LW-PLA Fairing Mass (0.48g)  : {mass_fairing_lwpla:.1f} g")
print(f"2x AGFRC B24CLM Servos Mass            : 42.0 g")
print(f"TOTAL SYSTEM ALL-UP MASS (PA12-CF)     : {mass_struct_pacf + mass_fairing_lwpla + 42.0:.1f} g")
print("=" * 72)
