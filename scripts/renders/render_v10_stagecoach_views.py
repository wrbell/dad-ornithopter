#!/usr/bin/env python3
"""
scripts/render_v10_stagecoach_views.py
======================================
Renders high-resolution technical CAD views of the V10 Stage-Coach Hollow
Skeletonized Sector Drum and Differential Assembly.
"""

import os
import pyvista as pv

OUT_DIR = "cad/manufactured/renders_v10"
ART_DIR = "/Users/drbell/.gemini/antigravity/brain/a5a482f2-2cea-48ab-945b-be0d2fd94a84/renders"
os.makedirs(OUT_DIR, exist_ok=True)
os.makedirs(ART_DIR, exist_ok=True)

pv.set_plot_theme("document")

drum_stl = "cad/manufactured/v10_stagecoach_sector_drum.stl"
merged_stl = "cad/manufactured/v10_stagecoach_merged_core.stl"

mesh_drum = pv.read(drum_stl)
mesh_merged = pv.read(merged_stl) if os.path.exists(merged_stl) else mesh_drum

views = [
    (
        "v10_01_isometric_stagecoach_drum.png",
        mesh_drum,
        [(85, -95, 75), (0, -10, 14), (0, 0, 1)],
        "#38bdf8" # Cyan
    ),
    (
        "v10_02_side_profile_servo_cooling_gates.png",
        mesh_drum,
        [(115, -14, 14), (0, -14, 14), (0, 0, 1)],
        "#38bdf8"
    ),
    (
        "v10_03_top_planform_stagecoach_spokes.png",
        mesh_drum,
        [(0, -10, 125), (0, -10, 14), (0, 1, 0)],
        "#38bdf8"
    ),
    (
        "v10_04_front_elevation_ram_air_vents.png",
        mesh_drum,
        [(0, 115, 14), (0, 0, 14), (0, 0, 1)],
        "#38bdf8"
    ),
    (
        "v10_05_bottom_perspective_chimney_draft.png",
        mesh_drum,
        [(75, -85, -60), (0, -10, 14), (0, 0, 1)],
        "#38bdf8"
    ),
    (
        "v10_06_differential_assembly_integration.png",
        mesh_merged,
        [(85, -90, 70), (0, 0, 14), (0, 0, 1)],
        "#0284c7" # Sky blue
    )
]

print("Rendering V10 Stage-Coach technical views...")
for fname, mesh, cam, color in views:
    plotter = pv.Plotter(off_screen=True, window_size=[1920, 1080])
    plotter.set_background("#0f172a") # Slate 900
    plotter.add_mesh(mesh, color=color, specular=0.6, specular_power=25, smooth_shading=True)
    plotter.camera_position = cam
    
    local_path = os.path.join(OUT_DIR, fname)
    art_path = os.path.join(ART_DIR, fname)
    
    plotter.screenshot(local_path)
    plotter.screenshot(art_path)
    plotter.close()
    print(f"  Rendered: {fname}")

print("All V10 renders generated successfully!")
