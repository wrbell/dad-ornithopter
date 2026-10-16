#!/usr/bin/env python3
import os
import pyvista as pv

OUT_DIR = '/Users/drbell/.gemini/antigravity/brain/6213fcb6-5e21-4842-8745-8b4c5b294586'

pv.set_plot_theme("document")

# 1. Representative Tail Skid Zoom
plotter_skid = pv.Plotter(off_screen=True, window_size=[1400, 900])
plotter_skid.set_background("#1e222b", top="#111318")
skid_mesh = pv.read('cad/manufactured/mco_representative_tailskid.stl')
plotter_skid.add_mesh(skid_mesh, color="#38bdf8", metallic=0.5, roughness=0.3, specular=0.8, show_edges=True, edge_color="#0284c7")
plotter_skid.add_text("MCO-1 Full-Scale Tail Skid (Scale-Model & Reference-Photo Matched)", position="upper_edge", font_size=12, color="#f8fafc")
plotter_skid.camera_position = [(60, -140, 40), (34, 0, -18), (0, 0, 1)]
plotter_skid.screenshot(os.path.join(OUT_DIR, 'mco_representative_tailskid_render.png'))

# 2. Main Gear Saddle Clamp Zoom
plotter_gear = pv.Plotter(off_screen=True, window_size=[1400, 900])
plotter_gear.set_background("#1e222b", top="#111318")
gear_mesh = pv.read('cad/manufactured/mco_main_gear_saddle_clamp.stl')
plotter_gear.add_mesh(gear_mesh, color="#a855f7", metallic=0.5, roughness=0.3, specular=0.8, show_edges=True, edge_color="#7e22ce")
plotter_gear.add_text("MCO-1 Ultralight Main Landing Gear Saddle Clamp (X=400mm on ⌀12mm Keel)", position="upper_edge", font_size=12, color="#f8fafc")
plotter_gear.camera_position = [(40, -60, 50), (0, 0, 0), (0, 0, 1)]
plotter_gear.screenshot(os.path.join(OUT_DIR, 'mco_main_gear_clamp_render.png'))

# 3. Corner 1 Zoom
plotter_c1 = pv.Plotter(off_screen=True, window_size=[1200, 800])
plotter_c1.set_background("#1e222b", top="#111318")
c1_mesh = pv.read('cad/manufactured/mco_corner1_top_fwd_clamp.stl')
plotter_c1.add_mesh(c1_mesh, color="#38bdf8", metallic=0.6, roughness=0.35, specular=0.8, show_edges=True, edge_color="#0284c7")
plotter_c1.add_text("Corner 1 Skeletal Clamp (⌀20mm Spine + ⌀14mm Post)", position="upper_edge", font_size=12, color="#f8fafc")
plotter_c1.camera_position = [(60, -90, 50), (0, 0, 10), (0, 0, 1)]
plotter_c1.screenshot(os.path.join(OUT_DIR, 'mco_corner1_skeletal_clamp_render.png'))

# 4. Complete Kit Overview
plotter_kit = pv.Plotter(off_screen=True, window_size=[1600, 1000])
plotter_kit.set_background("#1e222b", top="#111318")
kit_mesh = pv.read('cad/manufactured/mco_clamps_and_skid_complete_kit.stl')
plotter_kit.add_mesh(kit_mesh, color="#38bdf8", metallic=0.6, roughness=0.35, specular=0.8, show_edges=True, edge_color="#0284c7")
plotter_kit.add_text("MCO-1 Complete Airframe Clamp & Tail Skid Kit (Print Bed Layout)", position="upper_edge", font_size=13, color="#f8fafc")
plotter_kit.camera_position = [(0, -180, 100), (0, 0, 0), (0, 0, 1)]
plotter_kit.screenshot(os.path.join(OUT_DIR, 'mco_clamps_and_skid_kit_overview.png'))

print("Rendered all preview images successfully!")
