#!/usr/bin/env python3
"""
scripts/render_quickswap_cartridge.py
=====================================
Renders high-resolution technical CAD views of the V9 3D-printable
quick-swap bevel gear and differential wrist cartridge.
"""

import os
import pyvista as pv

pv.OFF_SCREEN = True

RENDER_DIR = "cad/manufactured/renders_v9"
os.makedirs(RENDER_DIR, exist_ok=True)

mesh_gear = pv.read("cad/manufactured/v9_quickswap_bevel_gear.stl")
mesh_cartridge = pv.read("cad/manufactured/v9_differential_bevel_wrist_cartridge.stl")

C_BG = "#0b0f19"
C_GEAR = "#e2e8f0"    # Off-white / light grey tough polymer
C_DIFF = "#38bdf8"    # Sky cyan cartridge assembly

# Render 1: Standalone Quick-Swap Bevel Gear (Isometric zoom showing teeth, hex bore & M2 hole)
p1 = pv.Plotter(off_screen=True, window_size=[1920, 1080])
p1.set_background(C_BG)
p1.enable_anti_aliasing("ssaa")
p1.add_mesh(mesh_gear, color=C_GEAR, smooth_shading=True, specular=0.6)
p1.camera_position = [(25, -25, 22), (0, 0, 5), (0, 0, 1)]
p1.screenshot(os.path.join(RENDER_DIR, "v9_08_quickswap_bevel_gear_zoom.png"))
p1.close()

# Render 2: Complete Differential Bevel Cartridge Assembly
p2 = pv.Plotter(off_screen=True, window_size=[1920, 1080])
p2.set_background(C_BG)
p2.enable_anti_aliasing("ssaa")
p2.add_mesh(mesh_cartridge, color=C_DIFF, smooth_shading=True, specular=0.5)
p2.camera_position = [(45, -40, 35), (0, 5, 14), (0, 0, 1)]
p2.screenshot(os.path.join(RENDER_DIR, "v9_09_differential_cartridge_assembly.png"))
p2.close()

print("Cartridge renders completed successfully!")
