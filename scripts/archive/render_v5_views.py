#!/usr/bin/env python3
"""
scripts/render_v5_views.py
==========================
Renders V5 Monolithic Embedded Drum-Face Spar views:
  - Spar emerging directly out of drum vertical face at mid-height (Z=13mm)
  - Zero top towers or pylons: ultra-low 26mm profile
  - Flanked by dual 6800 bearings at top and bottom
  - Aft servos and GoolRC 5010 motor tangent to curved arc
"""

import os
import math
import numpy as np
import pyvista as pv

pv.OFF_SCREEN = True

RENDER_DIR = "cad/manufactured/renders_v5"
ARTIFACT_DIR = "/Users/drbell/.gemini/antigravity/brain/a5a482f2-2cea-48ab-945b-be0d2fd94a84/renders"
os.makedirs(RENDER_DIR, exist_ok=True)
os.makedirs(ARTIFACT_DIR, exist_ok=True)

mesh_drum = pv.read("cad/manufactured/v5_embedded_sector_drum.stl")
mesh_yoke = pv.read("cad/manufactured/v5_embedded_wrist_yoke.stl")
mesh_spar = pv.read("cad/manufactured/v5_embedded_spar.stl")
mesh_motor = pv.read("cad/manufactured/v4_goolrc5010_pinion_assembly.stl")
mesh_fairing = pv.read("cad/manufactured/v5_embedded_fairing.stl")

# Bearings top (Z=26) and bottom (Z=0)
bearing_top = pv.Cylinder(center=(0, 0, 26.0), direction=(0, 0, 1), radius=9.5, height=5.0)
bearing_bot = pv.Cylinder(center=(0, 0, 0.0), direction=(0, 0, 1), radius=9.5, height=5.0)

# Servos in aft body (-Y)
servo_left = pv.Cube(center=(-14.5, -17.0, 15.0), x_length=12.0, y_length=23.0, z_length=24.0)
servo_right = pv.Cube(center=(14.5, -17.0, 15.0), x_length=12.0, y_length=23.0, z_length=24.0)

# Dyneema Cable Loop
cable_pts = []
for a in np.linspace(-150, -30, 40):
    rad = math.radians(a)
    cable_pts.append([35.0 * math.cos(rad), 35.0 * math.sin(rad), 4.0])
cable_pts.append([3.5, -39.5, 4.0])
for a in np.linspace(0, -180, 20):
    rad = math.radians(a)
    cable_pts.append([3.5 * math.cos(rad), -39.5 + 3.5 * math.sin(rad), 4.0])
cable_pts.append([-3.5, -39.5, 4.0])
cable_pts.append([cable_pts[0][0], cable_pts[0][1], 4.0])

cable_path = pv.MultipleLines(points=np.array(cable_pts))
cable_tube = cable_path.tube(radius=0.5)

C_BG = "#0b0f19"
C_DRUM = "#0284c7"       # Vibrant Cyan
C_YOKE = "#d97706"       # Gold / Bronze
C_SPAR = "#1e293b"       # Graphite dark
C_SERVO = "#ef4444"      # Red
C_MOTOR = "#334155"      # Gunmetal motor
C_BEARING = "#e2e8f0"    # Silver bearing
C_CABLE = "#c084fc"      # Purple Dyneema
C_FAIRING = "#f8fafc"    # White LW-PLA

views = [
    ("v5_01_isometric_without_fairing", False, None, [(110, -110, 80), (0, 0, 13), (0, 0, 1)]),
    ("v5_02_isometric_with_fairing_translucent", True, 0.28, [(110, -110, 80), (0, 0, 13), (0, 0, 1)]),
    ("v5_03_isometric_with_fairing_solid", True, 0.95, [(110, -110, 80), (0, 0, 13), (0, 0, 1)]),
    ("v5_04_side_profile_showing_midplane_spar", False, None, [(140, 10, 13), (0, 10, 13), (0, 0, 1)]),
    ("v5_05_side_profile_with_fairing", True, 0.35, [(140, 10, 13), (0, 10, 13), (0, 0, 1)]),
    ("v5_06_top_planform_without_fairing", False, None, [(0, 0, 160), (0, 0, 0), (0, 1, 0)]),
    ("v5_07_front_elevation_flat_face_emergence", False, None, [(0, 150, 13), (0, 0, 13), (0, 0, 1)]),
]

for name, inc_fairing, fairing_opac, cam_pos in views:
    plotter = pv.Plotter(off_screen=True, window_size=[1920, 1080])
    plotter.set_background(C_BG)
    plotter.enable_anti_aliasing("ssaa")

    plotter.add_mesh(mesh_drum, color=C_DRUM, smooth_shading=True, specular=0.5)
    plotter.add_mesh(mesh_yoke, color=C_YOKE, smooth_shading=True, specular=0.6)
    plotter.add_mesh(mesh_spar, color=C_SPAR, smooth_shading=True, specular=0.3)
    plotter.add_mesh(mesh_motor, color=C_MOTOR, smooth_shading=True, specular=0.4)
    plotter.add_mesh(servo_left, color=C_SERVO, specular=0.4)
    plotter.add_mesh(servo_right, color=C_SERVO, specular=0.4)
    plotter.add_mesh(bearing_top, color=C_BEARING, specular=0.8)
    plotter.add_mesh(bearing_bot, color=C_BEARING, specular=0.8)
    plotter.add_mesh(cable_tube, color=C_CABLE, specular=0.5)

    if inc_fairing:
        plotter.add_mesh(mesh_fairing, color=C_FAIRING, opacity=fairing_opac, smooth_shading=True, specular=0.4)

    plotter.camera_position = cam_pos
    out_file = os.path.join(RENDER_DIR, f"{name}.png")
    plotter.screenshot(out_file)
    art_file = os.path.join(ARTIFACT_DIR, f"{name}.png")
    plotter.screenshot(art_file)
    plotter.close()
    print(f"Rendered: {name}.png")

print("All V5 renders generated successfully!")
