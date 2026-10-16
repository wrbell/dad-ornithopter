#!/usr/bin/env python3
import os
import pyvista as pv

OUT_DIR = 'cad/manufactured/renders'
os.makedirs(OUT_DIR, exist_ok=True)

pv.set_plot_theme("document")

# 1. Exploded Perspective Render (showing internal tray and canopy hatch)
plotter_exp = pv.Plotter(off_screen=True, window_size=[1600, 1000])
plotter_exp.set_background("#1e222b", top="#111318")

tub_mesh = pv.read('cad/manufactured/mco_electronics_bay_tub.stl')
hatch_mesh = pv.read('cad/manufactured/mco_electronics_bay_canopy_hatch.stl')

# Shift hatch upward along Z by 45mm to show internal battery tray, cinch slots, and keel saddles
hatch_exploded = hatch_mesh.copy()
hatch_exploded.translate([0, 0, 45])

plotter_exp.add_mesh(tub_mesh, color="#38bdf8", metallic=0.5, roughness=0.35, specular=0.8, show_edges=True, edge_color="#0284c7")
plotter_exp.add_mesh(hatch_exploded, color="#34d399", metallic=0.6, roughness=0.3, specular=0.85, show_edges=True, edge_color="#059669")

plotter_exp.add_text("MCO-1 Streamlined Teardrop Electronics Pod & Quick-Release Canopy Hatch", position="upper_edge", font_size=13, color="#f8fafc")
plotter_exp.camera_position = [(450, -220, 150), (280, 0, 45), (0, 0, 1)]
plotter_exp.screenshot(os.path.join(OUT_DIR, 'mco_electronics_bay_exploded_detailed.png'))

# 2. Side Profile / Aero Fairing Render
plotter_side = pv.Plotter(off_screen=True, window_size=[1600, 700])
plotter_side.set_background("#1e222b", top="#111318")
asm_mesh = pv.read('cad/manufactured/mco_electronics_bay_assembly.stl')
plotter_side.add_mesh(asm_mesh, color="#38bdf8", metallic=0.5, roughness=0.35, specular=0.8, show_edges=True, edge_color="#0284c7")
plotter_side.add_text("MCO-1 Teardrop Fairing Side Profile (L = 370mm, NACA Airflow Profile)", position="upper_edge", font_size=12, color="#f8fafc")
plotter_side.camera_position = [(295, -280, 40), (295, 0, 40), (0, 0, 1)]
plotter_side.screenshot(os.path.join(OUT_DIR, 'mco_electronics_bay_side_detailed.png'))

# 3. Top-Down Internal Cockpit / Battery Cradle View
plotter_top = pv.Plotter(off_screen=True, window_size=[1600, 800])
plotter_top.set_background("#1e222b", top="#111318")
plotter_top.add_mesh(tub_mesh, color="#38bdf8", metallic=0.5, roughness=0.35, specular=0.8, show_edges=True, edge_color="#0284c7")
plotter_top.add_text("MCO-1 Lower Tub Interior: 6S LiPo Battery Cradle, Velcro Strap Slots & Keel Saddles", position="upper_edge", font_size=12, color="#f8fafc")
plotter_top.camera_position = [(295, 0, 260), (295, 0, 30), (1, 0, 0)]
plotter_top.screenshot(os.path.join(OUT_DIR, 'mco_electronics_bay_top_interior.png'))

print("Rendered all detailed electronics bay views successfully!")
