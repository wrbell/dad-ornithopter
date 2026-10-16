#!/usr/bin/env python3
"""
scripts/render_exploded_assembly_v9.py
======================================
Renders a 3D exploded assembly view of the V9 Beveled Gear Wing Root Wrist:
  - Exploded along vertical, lateral, and axial axes
  - Top and bottom 6800-2RS bearings displaced axially (+Z and -Z)
  - Dual micro-servos pulled out upwards and outwards
  - 3x Quick-swap bevel gears, 4mm cross-axle, and gimbal yoke exploded
  - Carbon fiber wing spar tube pulled forward (+Y)
  - Stepped capstan pinion and GoolRC 5010 motor pulled aft (-Y)
  - NACA 0018 fairing shell floating above (+Z)
"""

import os
import pyvista as pv

pv.OFF_SCREEN = True

OUT_DIR = "cad/manufactured/renders_v9"
ARTIFACT_DIR = "/Users/drbell/.gemini/antigravity/brain/a5a482f2-2cea-48ab-945b-be0d2fd94a84/renders"
os.makedirs(OUT_DIR, exist_ok=True)
os.makedirs(ARTIFACT_DIR, exist_ok=True)

# Base meshes
mesh_drum = pv.read("cad/manufactured/v9_topological_sector_drum.stl")
mesh_yoke = pv.read("cad/manufactured/v9_differential_gimbal_yoke.stl")
mesh_spar = pv.read("cad/manufactured/v9_topological_spar.stl")
mesh_pinion = pv.read("cad/manufactured/v9_stepped_pinion_assembly.stl")
mesh_fairing = pv.read("cad/manufactured/v9_topological_fairing.stl")
mesh_gear = pv.read("cad/manufactured/v9_quickswap_bevel_gear.stl")

# Exploded offsets
# Drum stays at origin (0, 0, 0)
# Top bearing exploded +Z by +35mm
bearing_top = pv.Cylinder(center=(0, 0, 25.5 + 35.0), direction=(0, 0, 1), radius=9.5, height=5.0)
# Bottom bearing exploded -Z by -30mm
bearing_bot = pv.Cylinder(center=(0, 0, 2.5 - 30.0), direction=(0, 0, 1), radius=9.5, height=5.0)

# Servos exploded upwards and outwards (+Z by +45mm, +/-X by +/-8mm)
servo_left = pv.Cube(center=(-13.2 - 10.0, -14.0, 18.0 + 45.0), x_length=23.0, y_length=12.0, z_length=19.5)
servo_right = pv.Cube(center=(13.2 + 10.0, -14.0, 18.0 + 45.0), x_length=23.0, y_length=12.0, z_length=19.5)

# Yoke and spar pulled forward (+Y by +40mm)
mesh_yoke_exp = mesh_yoke.copy().translate([0, 40.0, 0])
mesh_spar_exp = mesh_spar.copy().translate([0, 75.0, 0])

# Central cross axle (in yoke position)
axle_exp = pv.Cylinder(center=(0, 40.0, 14.0), direction=(1, 0, 0), radius=2.0, height=32.0)

# 3x Bevel Gears exploded from yoke:
# Left input gear pulled -X by -22mm
gear_left = mesh_gear.copy().rotate_y(90).translate([-22.0, 40.0, 14.0])
# Right input gear pulled +X by +22mm
gear_right = mesh_gear.copy().rotate_y(-90).translate([22.0, 40.0, 14.0])
# Spider gear pulled aft -Y by -20mm
gear_spider = mesh_gear.copy().rotate_x(90).translate([0, 40.0 - 20.0, 14.0])

# Motor and stepped pinion pulled aft (-Y by -45mm)
mesh_pinion_exp = mesh_pinion.copy().translate([0, -45.0, 0])

# Fairing floating high above (+Z by +95mm)
mesh_fairing_exp = mesh_fairing.copy().translate([0, 0, 95.0])

# Colors
C_BG = "#0b0f19"
C_DRUM = "#0284c7"      # Blue chassis
C_YOKE = "#d97706"      # Gold gimbal yoke
C_SPAR = "#1e293b"      # Carbon spar
C_SERVO = "#ef4444"     # Crimson red servos
C_PINION = "#64748b"    # Slate grey motor pinion
C_BEARING = "#e2e8f0"   # Chrome bearings
C_GEAR = "#f8fafc"      # White printable gears
C_AXLE = "#94a3b8"      # Steel axle
C_FAIRING = "#f1f5f9"   # Translucent fairing

plotter = pv.Plotter(off_screen=True, window_size=[1920, 1080])
plotter.set_background(C_BG)
plotter.enable_anti_aliasing("ssaa")

plotter.add_mesh(mesh_drum, color=C_DRUM, smooth_shading=True, specular=0.5)
plotter.add_mesh(mesh_yoke_exp, color=C_YOKE, smooth_shading=True, specular=0.6)
plotter.add_mesh(mesh_spar_exp, color=C_SPAR, smooth_shading=True, specular=0.3)
plotter.add_mesh(mesh_pinion_exp, color=C_PINION, smooth_shading=True, specular=0.4)
plotter.add_mesh(servo_left, color=C_SERVO, specular=0.4)
plotter.add_mesh(servo_right, color=C_SERVO, specular=0.4)
plotter.add_mesh(bearing_top, color=C_BEARING, specular=0.8)
plotter.add_mesh(bearing_bot, color=C_BEARING, specular=0.8)
plotter.add_mesh(axle_exp, color=C_AXLE, specular=0.8)
plotter.add_mesh(gear_left, color=C_GEAR, smooth_shading=True, specular=0.6)
plotter.add_mesh(gear_right, color=C_GEAR, smooth_shading=True, specular=0.6)
plotter.add_mesh(gear_spider, color=C_GEAR, smooth_shading=True, specular=0.6)
plotter.add_mesh(mesh_fairing_exp, color=C_FAIRING, opacity=0.35, smooth_shading=True, specular=0.4)

plotter.camera_position = [(180, -160, 160), (0, 15, 30), (0, 0, 1)]

out_img = os.path.join(OUT_DIR, "v9_10_exploded_assembly_view.png")
plotter.screenshot(out_img)
plotter.screenshot(os.path.join(ARTIFACT_DIR, "v9_10_exploded_assembly_view.png"))
plotter.close()

print(f"Exploded assembly view rendered: {out_img}")
