#!/usr/bin/env python3
"""
scripts/render_hierarchical_exploded_assembly.py
================================================
Renders the comprehensive 3-tier exploded engineering assembly:
  Tier 1: Wrist Differential Bevel Gear System (Servos, Gears, Axle, Yoke, Spar, Fairing)
  Tier 2: Flapping Shoulder Mechanism (Stagecoach Wheel, Bearings, Axle, Pinion, Motor, ODrive, Brace)
  Tier 3: Fuselage & Scaffolding Brace (Spine Tube, Topological Clamp Collar, Clevises, Stanchions)
"""

import os
import pyvista as pv

OUT_DIR = "cad/manufactured/renders_exploded"
os.makedirs(OUT_DIR, exist_ok=True)

pv.OFF_SCREEN = True
pv.set_plot_theme("document")

# 1. Load base CAD meshes
mesh_collar = pv.read("cad/manufactured/mco_topological_wing_collar_v1.stl")
mesh_drum = pv.read("cad/manufactured/v10_stagecoach_sector_drum.stl")
mesh_yoke = pv.read("cad/manufactured/v9_differential_gimbal_yoke.stl")
mesh_gear = pv.read("cad/manufactured/v9_quickswap_bevel_gear.stl")
mesh_spar = pv.read("cad/manufactured/v9_topological_spar.stl")
mesh_pinion = pv.read("cad/manufactured/v9_stepped_pinion_assembly.stl")
mesh_brace = pv.read("cad/manufactured/v10_powertrain_top_brace.stl")
mesh_fairing = pv.read("cad/manufactured/v9_topological_fairing.stl")

# -----------------------------------------------------------------------------
# 2. POSITIONING & EXPLODED TRAJECTORIES
# -----------------------------------------------------------------------------
# Datum: Spine centerline along X at Y = 0, Z = 0
# Scaffolding Collar is anchored at origin (0, 0, 0)
# Port clevis center is at (0, 35.0, 0)
# Motor stanchion is at (-41.25, 35.0, 0)

# A. Fuselage Spine Tube (Ø20mm OD, 220mm length)
fuselage_spine = pv.Cylinder(center=(0, 0, 0), direction=(1, 0, 0), radius=10.0, height=220.0)

# B. Flapping Shoulder Drum: Exploded outboard along +Y by +38mm (Y = 73.0mm)
drum_exp = mesh_drum.copy().translate([0.0, 73.0, -14.0])

# C. Flapping Pivot Hardware (Axle & Bearings) exploded vertically at Y = 73.0mm
pivot_pin = pv.Cylinder(center=(0.0, 73.0, 42.0), direction=(0, 0, 1), radius=5.0, height=48.0)
bearing_top = pv.Cylinder(center=(0.0, 73.0, 22.0), direction=(0, 0, 1), radius=9.5, height=5.0)
bearing_bot = pv.Cylinder(center=(0.0, 73.0, -22.0), direction=(0, 0, 1), radius=9.5, height=5.0)

# D. Motor, Pinion, and ODrive: Exploded aft along -X from -41.25 to -85.0mm
motor_exp = pv.Cylinder(center=(-85.0, 73.0, -14.0), direction=(0, 0, 1), radius=25.0, height=20.0)
pinion_exp = mesh_pinion.copy().translate([-43.75, 73.0 - 42.45, -14.0])
odrive_exp = pv.Cube(center=(-85.0, 73.0, -32.0), x_length=32.0, y_length=32.0, z_length=2.0)
magnet_exp = pv.Cylinder(center=(-85.0, 73.0, -27.0), direction=(0, 0, 1), radius=3.175, height=2.5)

# E. Dual-Shear Top Brace: Exploded upwards (+Z by +38mm)
brace_exp = mesh_brace.copy().rotate_z(90).translate([-42.0, 73.0, 38.0])

# F. Wrist Servos: Exploded upwards (+Z by +44mm) from drum
servo_left = pv.Cube(center=(-13.2, 73.0 - 14.5, 44.0), x_length=23.5, y_length=12.4, z_length=26.0)
servo_right = pv.Cube(center=(13.2, 73.0 - 14.5, 44.0), x_length=23.5, y_length=12.4, z_length=26.0)

# G. Wrist Differential Gimbal Yoke: Exploded forward (+Y by +52mm -> Y = 125.0mm)
yoke_exp = mesh_yoke.copy().translate([0.0, 125.0, 0.0])
cross_axle = pv.Cylinder(center=(0.0, 125.0, 0.0), direction=(1, 0, 0), radius=2.0, height=32.0)

# H. 3x Bevel Gears: Exploded from yoke
gear_left = mesh_gear.copy().rotate_y(90).translate([-22.0, 125.0, 0.0])
gear_right = mesh_gear.copy().rotate_y(-90).translate([22.0, 125.0, 0.0])
gear_spider = mesh_gear.copy().rotate_x(90).translate([0.0, 147.0, 0.0])

# I. Carbon Wing Spar Tube: Exploded forward (+Y by +110mm -> Y = 183.0mm)
spar_exp = mesh_spar.copy().translate([0.0, 183.0, -14.0])

# J. NACA 0018 Airfoil Fairing: Exploded upwards (+Z by +62mm)
fairing_exp = mesh_fairing.copy().translate([0.0, 73.0, 62.0 - 14.0])

# -----------------------------------------------------------------------------
# 3. RENDER SCENES & CAMERA ANGLES
# -----------------------------------------------------------------------------
views = [
    {
        "filename": "mco_01_hierarchical_exploded_isometric.png",
        "cam": [(170, -150, 135), (-10, 60, 5), (0, 0, 1)],
        "title": "MCO-1D 3-Tier Exploded Assembly: Wrist -> Shoulder -> Scaffolding Brace -> Fuselage",
    },
    {
        "filename": "mco_02_wrist_to_shoulder_closeup.png",
        "cam": [(90, 45, 75), (0, 105, 5), (0, 0, 1)],
        "title": "Tier 1 Wrist Cartridge Installing into Tier 2 Stagecoach Sector Wheel",
    },
    {
        "filename": "mco_03_shoulder_to_scaffolding_brace.png",
        "cam": [(-120, -50, 95), (-25, 45, -5), (0, 0, 1)],
        "title": "Tier 2 Flapping Shoulder Mechanism Docking to Tier 3 Scaffolding Brace",
    },
    {
        "filename": "mco_04_side_elevation_exploded.png",
        "cam": [(230, 70, 15), (-10, 70, 10), (0, 0, 1)],
        "title": "Side Elevation View: Clear Assembly Trajectories along +Y, -X, and +Z",
    }
]

print("=" * 78)
print("  RENDERING HIGH-PRECISION 3-TIER EXPLODED ASSEMBLY VIEWS")
print("=" * 78)

for v in views:
    pl = pv.Plotter(window_size=[2400, 1350])
    pl.camera_position = v["cam"]
    pl.add_text(v["title"], position="upper_left", font_size=12, color="#0f172a")

    # Tier 3: Fuselage & Scaffolding Brace
    pl.add_mesh(fuselage_spine, color="#0f172a", opacity=0.92, smooth_shading=True)
    pl.add_mesh(mesh_collar, color="#0284c7", opacity=0.95, smooth_shading=True, specular=0.45)

    # Tier 2: Flapping Shoulder Mechanism
    pl.add_mesh(drum_exp, color="#38bdf8", opacity=0.85, smooth_shading=True, specular=0.5)
    pl.add_mesh(pivot_pin, color="#cbd5e1", opacity=1.0, smooth_shading=True, specular=0.8)
    pl.add_mesh(bearing_top, color="#64748b", opacity=1.0, smooth_shading=True)
    pl.add_mesh(bearing_bot, color="#64748b", opacity=1.0, smooth_shading=True)
    pl.add_mesh(motor_exp, color="#1e293b", opacity=0.9, smooth_shading=True)
    pl.add_mesh(pinion_exp, color="#94a3b8", opacity=1.0, smooth_shading=True)
    pl.add_mesh(odrive_exp, color="#10b981", opacity=1.0, smooth_shading=True)
    pl.add_mesh(magnet_exp, color="#cbd5e1", opacity=1.0, smooth_shading=True)
    pl.add_mesh(brace_exp, color="#f59e0b", opacity=0.9, smooth_shading=True)

    # Tier 1: Wrist Differential System
    pl.add_mesh(servo_left, color="#ef4444", opacity=0.9, smooth_shading=True)
    pl.add_mesh(servo_right, color="#ef4444", opacity=0.9, smooth_shading=True)
    pl.add_mesh(yoke_exp, color="#f59e0b", opacity=1.0, smooth_shading=True, specular=0.4)
    pl.add_mesh(cross_axle, color="#cbd5e1", opacity=1.0, smooth_shading=True)
    pl.add_mesh(gear_left, color="#e2e8f0", opacity=1.0, smooth_shading=True, specular=0.6)
    pl.add_mesh(gear_right, color="#e2e8f0", opacity=1.0, smooth_shading=True, specular=0.6)
    pl.add_mesh(gear_spider, color="#e2e8f0", opacity=1.0, smooth_shading=True, specular=0.6)
    pl.add_mesh(spar_exp, color="#0f172a", opacity=0.95, smooth_shading=True, specular=0.5)
    pl.add_mesh(fairing_exp, color="#f8fafc", opacity=0.32, smooth_shading=True, specular=0.3)

    out_file = os.path.join(OUT_DIR, v["filename"])
    pl.screenshot(out_file)
    pl.close()
    print(f"  ✓ Rendered: {v['filename']}")

print("=" * 78)
