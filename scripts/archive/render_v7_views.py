#!/usr/bin/env python3
"""
scripts/render_v7_views.py
==========================
Renders V7 200° Skeletonized Sector Wheel views:
  - Full 200° continuous arc
  - Thinned 1.8mm rim wall with topological isogrid lightening pockets
  - Mid-plane spar emerging from flat face
  - Tangent GoolRC 5010 motor & 0.6mm Dyneema loop
"""

import os
import math
import numpy as np
import pyvista as pv

pv.OFF_SCREEN = True

RENDER_DIR = "cad/manufactured/renders_v7"
ARTIFACT_DIR = "/Users/drbell/.gemini/antigravity/brain/a5a482f2-2cea-48ab-945b-be0d2fd94a84/renders"
os.makedirs(RENDER_DIR, exist_ok=True)
os.makedirs(ARTIFACT_DIR, exist_ok=True)

mesh_drum = pv.read("cad/manufactured/v7_skeleton_sector_drum.stl")
mesh_yoke = pv.read("cad/manufactured/v7_skeleton_wrist_yoke.stl")
mesh_spar = pv.read("cad/manufactured/v7_skeleton_spar.stl")
mesh_motor = pv.read("cad/manufactured/v4_goolrc5010_pinion_assembly.stl")
mesh_fairing = pv.read("cad/manufactured/v7_skeleton_fairing.stl")

bearing_top = pv.Cylinder(center=(0, 0, 26.0), direction=(0, 0, 1), radius=9.5, height=5.0)
bearing_bot = pv.Cylinder(center=(0, 0, 0.0), direction=(0, 0, 1), radius=9.5, height=5.0)

servo_left = pv.Cube(center=(-12.5, -13.0, 16.25), x_length=23.0, y_length=12.0, z_length=19.5)
servo_right = pv.Cube(center=(12.5, -13.0, 16.25), x_length=23.0, y_length=12.0, z_length=19.5)

# 200° Continuous Dyneema Cable Loop
cable_pts = []
for a in np.linspace(-180, 0, 60):
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
C_DRUM = "#0284c7"
C_YOKE = "#d97706"
C_SPAR = "#1e293b"
C_SERVO = "#ef4444"
C_MOTOR = "#334155"
C_BEARING = "#e2e8f0"
C_CABLE = "#c084fc"
C_FAIRING = "#f8fafc"

views = [
    ("v7_01_isometric_200deg_skeleton", False, None, [(115, -110, 85), (0, 0, 13), (0, 0, 1)]),
    ("v7_02_isometric_with_fairing_translucent", True, 0.28, [(115, -110, 85), (0, 0, 13), (0, 0, 1)]),
    ("v7_03_top_planform_200deg_wrap", False, None, [(0, 0, 165), (0, 0, 0), (0, 1, 0)]),
    ("v7_04_bottom_perspective_isogrid_skeleton", False, None, [(110, -110, -75), (0, 0, 13), (0, 0, 1)]),
    ("v7_05_side_elevation_unbroken_rim", False, None, [(140, 0, 13), (0, 0, 13), (0, 0, 1)]),
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

print("All V7 renders generated successfully!")
