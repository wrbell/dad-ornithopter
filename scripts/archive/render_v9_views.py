#!/usr/bin/env python3
"""
scripts/render_v9_views.py
==========================
Renders V9 "Topological Truss & Asymmetric Trumpet" 10:1 Merged Wrist & Sector Assembly:
  - 10:1 ratio with R=37.5mm sector drum & r=3.75mm stepped pinion
  - Locked 0.75mm Dyneema SK75/SK99 cable path (D:d = 10.0:1)
  - Full 210.0° unbroken continuous sector rim (supports 120° stroke with ±45° margin)
  - Asymmetric teardrop trumpet emergence socket (-15° upstroke to +36° downstroke)
  - Topologically skeletonized Warren-truss space frame and flow-through cooling chimneys
  - Gearless carbon pushrod actuation horns on embedded gimbal yoke
  - NACA 0018 aerodynamic fairing with integrated cooling scoops
"""

import os
import math
import numpy as np
import pyvista as pv

pv.OFF_SCREEN = True

RENDER_DIR = "cad/manufactured/renders_v9"
ARTIFACT_DIR = "/Users/drbell/.gemini/antigravity/brain/a5a482f2-2cea-48ab-945b-be0d2fd94a84/renders"
os.makedirs(RENDER_DIR, exist_ok=True)
os.makedirs(ARTIFACT_DIR, exist_ok=True)

# Load V9 STL meshes
mesh_drum = pv.read("cad/manufactured/v9_topological_sector_drum.stl")
mesh_yoke = pv.read("cad/manufactured/v9_actuation_yoke.stl")
mesh_spar = pv.read("cad/manufactured/v9_topological_spar.stl")
mesh_pinion = pv.read("cad/manufactured/v9_stepped_pinion_assembly.stl")
mesh_fairing = pv.read("cad/manufactured/v9_topological_fairing.stl")

# Dual 6800-2RS Bearings (OD 19mm, ID 10mm, H 5mm)
bearing_top = pv.Cylinder(center=(0, 0, 25.5), direction=(0, 0, 1), radius=9.5, height=5.0)
bearing_bot = pv.Cylinder(center=(0, 0, 2.5), direction=(0, 0, 1), radius=9.5, height=5.0)

# Dual AGFRC B24CLM Servos nested internally
servo_left = pv.Cube(center=(-13.2, -14.0, 18.0), x_length=23.0, y_length=12.0, z_length=19.5)
servo_right = pv.Cube(center=(13.2, -14.0, 18.0), x_length=23.0, y_length=12.0, z_length=19.5)

# 0.75mm Dyneema Cable Path (wraps around 210° sector rim and stepped pinion)
R_pitch = 37.5
r_pin = 3.75
pin_y = -R_pitch - r_pin - 1.2  # -42.45 mm
z_cable = 4.1                   # Mid-plane of cable groove

cable_pts = []
# Sector rim wrap from -190° to 10°
for a in np.linspace(-190, 10, 84):
    rad = math.radians(a)
    cable_pts.append([R_pitch * math.cos(rad), R_pitch * math.sin(rad), z_cable])

# Lead to pinion
cable_pts.append([r_pin, pin_y, z_cable])
# Wrap around pinion
for a in np.linspace(0, -180, 25):
    rad = math.radians(a)
    cable_pts.append([r_pin * math.cos(rad), pin_y + r_pin * math.sin(rad), z_cable])
# Return to sector rim
cable_pts.append([-r_pin, pin_y, z_cable])
cable_pts.append([cable_pts[0][0], cable_pts[0][1], z_cable])

cable_path = pv.MultipleLines(points=np.array(cable_pts))
cable_tube = cable_path.tube(radius=0.45) # 0.75-0.9mm visual tube

# Direct Carbon Pushrods from Servos to Yoke Horns (Option A demonstration)
pushrod_left = pv.Cylinder(center=(-10.6, -11.0, 14.0), direction=(-0.35, 0.93, 0.0), radius=0.8, height=12.0)
pushrod_right = pv.Cylinder(center=(10.6, -11.0, 14.0), direction=(0.35, 0.93, 0.0), radius=0.8, height=12.0)

# Colors
C_BG = "#0b0f19"        # Deep aerospace dark background
C_DRUM = "#0284c7"      # Sky blue PA12-CF drum
C_YOKE = "#d97706"      # Anodized gold CNC 7075-T6 yoke
C_SPAR = "#1e293b"      # Matte carbon twill spar
C_SERVO = "#ef4444"     # Crimson red micro-servos
C_PINION = "#64748b"    # Slate grey stepped pinion & motor
C_BEARING = "#e2e8f0"   # Chrome bearing steel
C_CABLE = "#a855f7"     # Vivid purple Dyneema line
C_ROD = "#38bdf8"       # Cyan carbon pushrods
C_FAIRING = "#f8fafc"   # White LW-PLA fairing

views = [
    (
        "v9_01_isometric_without_fairing",
        False,
        None,
        [(120, -115, 90), (0, -10, 14), (0, 0, 1)],
    ),
    (
        "v9_02_isometric_with_fairing_translucent",
        True,
        0.28,
        [(120, -115, 90), (0, -10, 14), (0, 0, 1)],
    ),
    (
        "v9_03_top_planform_210deg_wrap",
        False,
        None,
        [(0, -10, 185), (0, -10, 0), (0, 1, 0)],
    ),
    (
        "v9_04_bottom_perspective_isogrid_truss",
        False,
        None,
        [(115, -110, -80), (0, -10, 14), (0, 0, 1)],
    ),
    (
        "v9_05_front_elevation_asymmetric_trumpet",
        False,
        None,
        [(65, 85, 45), (0, 15, 14), (0, 0, 1)],
    ),
    (
        "v9_06_rear_elevation_stepped_pinion",
        False,
        None,
        [(0, -160, 14), (0, -25, 14), (0, 0, 1)],
    ),
    (
        "v9_07_side_profile_unbroken_rim",
        False,
        None,
        [(155, -15, 14), (0, -15, 14), (0, 0, 1)],
    ),
]

for name, inc_fairing, fairing_opac, cam_pos in views:
    plotter = pv.Plotter(off_screen=True, window_size=[1920, 1080])
    plotter.set_background(C_BG)
    plotter.enable_anti_aliasing("ssaa")

    plotter.add_mesh(mesh_drum, color=C_DRUM, smooth_shading=True, specular=0.5)
    plotter.add_mesh(mesh_yoke, color=C_YOKE, smooth_shading=True, specular=0.6)
    plotter.add_mesh(mesh_spar, color=C_SPAR, smooth_shading=True, specular=0.3)
    plotter.add_mesh(mesh_pinion, color=C_PINION, smooth_shading=True, specular=0.5)
    plotter.add_mesh(servo_left, color=C_SERVO, specular=0.4)
    plotter.add_mesh(servo_right, color=C_SERVO, specular=0.4)
    plotter.add_mesh(bearing_top, color=C_BEARING, specular=0.8)
    plotter.add_mesh(bearing_bot, color=C_BEARING, specular=0.8)
    plotter.add_mesh(cable_tube, color=C_CABLE, specular=0.5)
    plotter.add_mesh(pushrod_left, color=C_ROD, specular=0.7)
    plotter.add_mesh(pushrod_right, color=C_ROD, specular=0.7)

    if inc_fairing:
        plotter.add_mesh(
            mesh_fairing,
            color=C_FAIRING,
            opacity=fairing_opac,
            smooth_shading=True,
            specular=0.4,
        )

    plotter.camera_position = cam_pos
    out_file = os.path.join(RENDER_DIR, f"{name}.png")
    plotter.screenshot(out_file)
    art_file = os.path.join(ARTIFACT_DIR, f"{name}.png")
    plotter.screenshot(art_file)
    plotter.close()
    print(f"Rendered: {name}.png")

print("All V9 renders generated successfully!")
