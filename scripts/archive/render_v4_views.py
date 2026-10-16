#!/usr/bin/env python3
"""
scripts/render_v4_views.py
==========================
Renders multi-angle high-resolution visuals of the V4 redesigned 2-DOF wing root
mechanism:
  - Spar extending out of the FLAT FACE
  - Compact 10mm hollow wiring core
  - Servos pulled far away from the flat face (aft toward curved arc)
  - Flap motor (GoolRC 5010) & small capstan drum positioned tangent to curved arc
  - 0.6mm Dyneema drive cable loop (10:1 ratio)
  - With and without NACA 0018 fairing
"""

import os
import math
import numpy as np
import pyvista as pv

pv.OFF_SCREEN = True

RENDER_DIR = "cad/manufactured/renders_v4"
ARTIFACT_DIR = "/Users/drbell/.gemini/antigravity/brain/a5a482f2-2cea-48ab-945b-be0d2fd94a84/renders"
os.makedirs(RENDER_DIR, exist_ok=True)
os.makedirs(ARTIFACT_DIR, exist_ok=True)

# Load meshes
mesh_drum = pv.read("cad/manufactured/v4_sector_wheel_flatface_spar.stl")
mesh_wrist = pv.read("cad/manufactured/v4_2dof_wrist_yoke.stl")
mesh_spar = pv.read("cad/manufactured/v4_spar_tube.stl")
mesh_motor = pv.read("cad/manufactured/v4_goolrc5010_pinion_assembly.stl")
mesh_fairing = pv.read("cad/manufactured/v4_naca0018_fairing.stl")

# Create servo blocks (AGFRC B24CLM: 12.0 x 23.0 x 26.0mm)
servo_left = pv.Cube(center=(-14.5, -17.0, 14.0), x_length=12.0, y_length=23.0, z_length=24.0)
servo_right = pv.Cube(center=(14.5, -17.0, 14.0), x_length=12.0, y_length=23.0, z_length=24.0)

# Create 6800 bearing ring (OD 19mm, ID 10mm, H 5mm) at top of hub
bearing_top = pv.Cylinder(center=(0, 0, 24.0), direction=(0, 0, 1), radius=9.5, height=5.0)

# Create 0.6mm Dyneema cable path (pitch R=35mm, pinion r=3.5mm at Y=-39.5mm)
# Arc from -140° to -40°
cable_pts = []
# Sector arc
for a in np.linspace(-150, -30, 40):
    rad = math.radians(a)
    cable_pts.append([35.0 * math.cos(rad), 35.0 * math.sin(rad), 4.0])
# Cross over to motor pinion at Y = -39.5
cable_pts.append([3.5, -39.5, 4.0])
for a in np.linspace(0, -180, 20):
    rad = math.radians(a)
    cable_pts.append([3.5 * math.cos(rad), -39.5 + 3.5 * math.sin(rad), 4.0])
cable_pts.append([-3.5, -39.5, 4.0])
cable_pts.append([cable_pts[0][0], cable_pts[0][1], 4.0])

cable_path = pv.MultipleLines(points=np.array(cable_pts))
cable_tube = cable_path.tube(radius=0.5)

# Color Scheme
C_BG = "#0b0f19"
C_DRUM = "#0284c7"       # Vibrant Cyan/Teal
C_WRIST = "#d97706"      # Gold / Bronze
C_SPAR = "#1e293b"       # Carbon graphite dark
C_SERVO = "#ef4444"      # Red
C_MOTOR = "#334155"      # Dark gunmetal motor stator
C_PINION = "#38bdf8"     # Light sky blue pinion
C_BEARING = "#e2e8f0"    # Silver bearing
C_CABLE = "#c084fc"      # Purple Dyneema SK99 0.6mm
C_FAIRING = "#f8fafc"    # Light ivory LW-PLA

views = [
    ("v4_01_isometric_without_fairing", False, None, [(110, -120, 95), (0, 0, 15), (0, 0, 1)]),
    ("v4_02_isometric_with_fairing_translucent", True, 0.28, [(110, -120, 95), (0, 0, 15), (0, 0, 1)]),
    ("v4_03_isometric_with_fairing_solid", True, 0.95, [(110, -120, 95), (0, 0, 15), (0, 0, 1)]),
    ("v4_04_top_without_fairing", False, None, [(0, 0, 175), (0, 0, 0), (0, 1, 0)]),
    ("v4_05_top_with_fairing_translucent", True, 0.30, [(0, 0, 175), (0, 0, 0), (0, 1, 0)]),
    ("v4_06_top_with_fairing_solid", True, 0.95, [(0, 0, 175), (0, 0, 0), (0, 1, 0)]),
    ("v4_07_side_without_fairing", False, None, [(160, 0, 15), (0, 0, 15), (0, 0, 1)]),
    ("v4_08_side_with_fairing_translucent", True, 0.30, [(160, 0, 15), (0, 0, 15), (0, 0, 1)]),
    ("v4_09_front_looking_at_flat_face", False, None, [(0, 170, 25), (0, 0, 20), (0, 0, 1)]),
    ("v4_10_rear_looking_at_motor_and_servos", False, None, [(0, -170, 25), (0, 0, 15), (0, 0, 1)]),
]

for name, inc_fairing, fairing_opac, cam_pos in views:
    plotter = pv.Plotter(off_screen=True, window_size=[1920, 1080])
    plotter.set_background(C_BG)
    plotter.enable_anti_aliasing("ssaa")

    # Add components
    plotter.add_mesh(mesh_drum, color=C_DRUM, smooth_shading=True, specular=0.5)
    plotter.add_mesh(mesh_wrist, color=C_WRIST, smooth_shading=True, specular=0.6)
    plotter.add_mesh(mesh_spar, color=C_SPAR, smooth_shading=True, specular=0.3)
    plotter.add_mesh(mesh_motor, color=C_MOTOR, smooth_shading=True, specular=0.4)
    plotter.add_mesh(servo_left, color=C_SERVO, specular=0.4)
    plotter.add_mesh(servo_right, color=C_SERVO, specular=0.4)
    plotter.add_mesh(bearing_top, color=C_BEARING, specular=0.8)
    plotter.add_mesh(cable_tube, color=C_CABLE, specular=0.5)

    if inc_fairing:
        plotter.add_mesh(mesh_fairing, color=C_FAIRING, opacity=fairing_opac, smooth_shading=True, specular=0.4)

    plotter.camera_position = cam_pos
    out_file = os.path.join(RENDER_DIR, f"{name}.png")
    plotter.screenshot(out_file)
    # Also copy to artifact dir
    art_file = os.path.join(ARTIFACT_DIR, f"{name}.png")
    plotter.screenshot(art_file)
    plotter.close()
    print(f"Rendered: {name}.png")

print("All V4 multi-angle renders completed successfully!")
