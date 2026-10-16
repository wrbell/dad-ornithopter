#!/usr/bin/env python3
"""
scripts/render_v10_powertrain_chassis.py
========================================
Renders high-resolution technical CAD views of the V10 Powertrain Mount (Stand),
Dual-Shear Top Brace, ODrive Micro integration, and Fuselage interface.
"""

import os
import pyvista as pv

OUT_DIR = "cad/manufactured/renders_v10"
os.makedirs(OUT_DIR, exist_ok=True)

pv.OFF_SCREEN = True
pv.set_plot_theme("document")

# 1. Load component meshes
mesh_stand = pv.read("cad/manufactured/v10_powertrain_stand.stl")
mesh_brace = pv.read("cad/manufactured/v10_powertrain_top_brace.stl")
mesh_drum = pv.read("cad/manufactured/v10_stagecoach_sector_drum.stl")
mesh_pinion = pv.read("cad/manufactured/v9_stepped_pinion_assembly.stl")

# Reposition parts into complete assembled powertrain
# Drum sits at (0, 0, 16.5) resting on the tower pedestal
mesh_drum_asm = mesh_drum.copy().translate([0.0, 0.0, 16.5])

# Brace sits on top at Z = 44.5mm
mesh_brace_asm = mesh_brace.copy().translate([0.0, 0.0, 44.5])

# Motor representation (GoolRC 5010 OD 50mm, height 20mm) under pinion at (0, -42, -5.0)
mesh_motor = pv.Cylinder(center=(0.0, -42.0, 5.0), direction=(0, 0, 1), radius=25.0, height=20.0)

# ODrive Micro PCB representation (32x32x1.6mm green PCB) inside pocket at (0, -42, -5.0)
mesh_odrive = pv.Cube(center=(0.0, -42.0, -5.5), x_length=32.0, y_length=32.0, z_length=1.6)

# Magnetic Counter chip (MPS MA702 3x3x0.9mm black IC)
mesh_ic = pv.Cube(center=(0.0, -42.0, -4.25), x_length=3.0, y_length=3.0, z_length=0.9)

# Diametral magnet (Ø6.35x2.5mm silver cylinder on shaft)
mesh_magnet = pv.Cylinder(center=(0.0, -42.0, -2.75), direction=(0, 0, 1), radius=3.175, height=2.5)

# Fuselage Spine Carbon Tube representation (Ø20mm longeron at X = -35mm)
mesh_spine = pv.Cylinder(center=(-35.0, -21.0, 24.5), direction=(0, 1, 0), radius=10.0, height=120.0)
mesh_fuse_flange = pv.Cube(center=(-26.0, -21.0, 24.5), x_length=6.0, y_length=50.0, z_length=56.0)

# Pinion drum at (0, -42.0, 16.5)
mesh_pinion_asm = mesh_pinion.copy().translate([0.0, 0.45, 16.5])

views = [
    {
        "filename": "v10_powertrain_01_full_assembly_isometric.png",
        "cam": [(120, -130, 110), (0, -21, 20), (0, 0, 1)],
        "title": "V10 Powertrain Complete Assembly (Stand + Brace + Drum + Motor)",
        "elements": [
            (mesh_stand, "#38bdf8", 1.0),      # Stand in cyan
            (mesh_brace_asm, "#f59e0b", 1.0),  # Top brace in amber
            (mesh_drum_asm, "#64748b", 0.85),  # Drum in slate
            (mesh_motor, "#1e293b", 0.9),      # Motor in dark charcoal
            (mesh_pinion_asm, "#94a3b8", 1.0), # Stepped pinion in silver
            (mesh_odrive, "#10b981", 1.0),     # ODrive PCB in green
        ]
    },
    {
        "filename": "v10_powertrain_02_odrive_encoder_undercarriage.png",
        "cam": [(60, -90, -85), (0, -42, -5), (0, 1, 0)],
        "title": "ODrive Micro Integration & 1.50mm Magnetic Counter Air Gap",
        "elements": [
            (mesh_stand, "#38bdf8", 0.65),     # Semi-transparent stand
            (mesh_motor, "#1e293b", 0.5),
            (mesh_odrive, "#10b981", 1.0),     # ODrive Micro PCB
            (mesh_ic, "#dc2626", 1.0),         # MA702 IC package in red
            (mesh_magnet, "#cbd5e1", 1.0),     # Magnet in bright chrome
        ]
    },
    {
        "filename": "v10_powertrain_03_sector_wheel_groove_and_bumpers.png",
        "cam": [(95, -75, 45), (0, 0, 20), (0, 0, 1)],
        "title": "V10 Sector Wheel: Helical Tracking Groove, End Bumpers & Dyneema Bolts",
        "elements": [
            (mesh_drum_asm, "#38bdf8", 1.0),   # Stagecoach drum in cyan
            (mesh_stand, "#64748b", 0.5),      # Stand background
            (mesh_pinion_asm, "#f59e0b", 0.9), # Pinion in amber
        ]
    },
    {
        "filename": "v10_powertrain_04_fuselage_spine_interface.png",
        "cam": [(-125, -21, 24.5), (0, -21, 24.5), (0, 0, 1)],
        "title": "Fuselage Mounting Interface (4x M3 + Ø6.0mm Pin @ Y = ±35.0mm)",
        "elements": [
            (mesh_stand, "#38bdf8", 1.0),
            (mesh_brace_asm, "#f59e0b", 0.8),
            (mesh_fuse_flange, "#475569", 0.6),
            (mesh_spine, "#0f172a", 0.85),
        ]
    },
    {
        "filename": "v10_powertrain_05_dual_shear_box_truss_elevation.png",
        "cam": [(120, -21, 20), (0, -21, 20), (0, 0, 1)],
        "title": "Dual-Shear Box Truss: 220N Tension Reacted in Pure Plane Truss",
        "elements": [
            (mesh_stand, "#38bdf8", 1.0),
            (mesh_brace_asm, "#f59e0b", 1.0),
            (mesh_drum_asm, "#64748b", 0.5),
            (mesh_motor, "#1e293b", 0.7),
            (mesh_pinion_asm, "#94a3b8", 0.9),
        ]
    }
]

print("=" * 76)
print("  RENDERING V10 POWERTRAIN CHASSIS CAD VIEWS")
print("=" * 76)

for v in views:
    pl = pv.Plotter(window_size=[1920, 1080])
    pl.camera_position = v["cam"]
    pl.add_text(v["title"], position="upper_left", font_size=12, color="#0f172a")

    for elem, col, op in v["elements"]:
        pl.add_mesh(
            elem,
            color=col,
            opacity=op,
            smooth_shading=True,
            specular=0.5,
            specular_power=20,
            show_edges=False
        )

    out_file = os.path.join(OUT_DIR, v["filename"])
    pl.screenshot(out_file)
    pl.close()
    print(f"  ✓ Rendered: {v['filename']}")

print("=" * 76)
