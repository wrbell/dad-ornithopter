#!/usr/bin/env python3
"""
scripts/render_plate_01_bed.py
==============================
Renders an isometric and top-down view of the single-plate slicer build layout
(PLATE_01_PA12CF_ALL_STRUCTURAL_PARTS.stl) on a standard 220x220mm 3D printer bed.
"""

import os
import pyvista as pv

OUT_DIR = "cad/manufactured/renders_package"
os.makedirs(OUT_DIR, exist_ok=True)

pv.OFF_SCREEN = True
pv.set_plot_theme("document")

mesh_plate = pv.read("cad/print_package_wing_root/PLATE_01_PA12CF_ALL_STRUCTURAL_PARTS.stl")

# Create 220x220mm build bed plate representation
bed_plane = pv.Plane(center=(0, 0, -0.5), direction=(0, 0, 1), i_size=220, j_size=220)

views = [
    {
        "filename": "plate_01_bed_layout_isometric.png",
        "cam": [(200, -220, 180), (0, 0, 10), (0, 0, 1)],
        "title": "PLATE 01: Full Wing Root Structural Parts (PA12-CF on 220x220mm Bed)"
    },
    {
        "filename": "plate_01_bed_layout_top_down.png",
        "cam": [(0, 0, 320), (0, 0, 0), (0, 1, 0)],
        "title": "PLATE 01: Slicer Bed Layout Plan View (Collar, Drum, Brace, Yoke, Pinion, Gears)"
    }
]

for v in views:
    plotter = pv.Plotter(window_size=[1600, 1000])
    plotter.set_background("#0f172a") # dark slate
    
    # Bed surface
    plotter.add_mesh(bed_plane, color="#1e293b", opacity=0.9, show_edges=True, edge_color="#334155")
    
    # Parts mesh (PA12-CF carbon look)
    plotter.add_mesh(
        mesh_plate,
        color="#38bdf8", # Cyan
        pbr=True,
        metallic=0.4,
        roughness=0.3,
        show_edges=True,
        edge_color="#0284c7"
    )
    
    plotter.camera_position = v["cam"]
    plotter.add_text(v["title"], position="upper_left", font_size=13, color="white")
    
    out_file = os.path.join(OUT_DIR, v["filename"])
    plotter.screenshot(out_file)
    plotter.close()
    print(f"Rendered: {out_file}")

print("Plate rendering complete.")
