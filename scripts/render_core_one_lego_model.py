#!/usr/bin/env python3
"""
scripts/render_core_one_lego_model.py
=====================================
Renders the special Lego Edition of the MCO-1 8-Wing Tandem Aircraft,
focusing on the forward-extended keel cockpit, Lego seat with studs,
control panel, joysticks, and front bumper.
"""

import os
import shutil
import numpy as np
import pyvista as pv

OUT_DIR = 'cad/manufactured'
ARTIFACT_DIR = '/Users/drbell/.gemini/antigravity/brain/fcfca30b-4f75-4837-ba9b-50a1b0b3caba'
AIRCRAFT_STL = os.path.join(OUT_DIR, 'mco_core_one_lego_edition.stl')

def render_views():
    if not os.path.exists(AIRCRAFT_STL):
        print(f"Error: {AIRCRAFT_STL} not found.")
        return

    print("Rendering Prusa CORE One Lego Edition model...")
    reader = pv.get_reader(AIRCRAFT_STL)
    mesh = reader.read()

    bb = mesh.bounds  # xmin, xmax, ymin, ymax, zmin, zmax
    plate_x = 250.0
    plate_y = 220.0
    plate_center_x = plate_x / 2.0
    plate_center_y = 0.0

    # Center mesh on the build plate
    mesh_center_x = (bb[0] + bb[1]) / 2.0
    mesh_center_y = (bb[2] + bb[3]) / 2.0
    mesh_centered = mesh.translate((plate_center_x - mesh_center_x, plate_center_y - mesh_center_y, -bb[4]))

    # Plate mesh
    plate = pv.Cube(center=(plate_center_x, plate_center_y, -1.0), x_length=plate_x, y_length=plate_y, z_length=2.0)

    # 1. Perspective View on Build Plate (Isometric Overview)
    p1 = pv.Plotter(off_screen=True, window_size=[1800, 1100])
    p1.set_background('#f8fafc')
    p1.add_mesh(plate, color='#1e293b', specular=0.2, ambient=0.3, diffuse=0.7)
    # Vibrant Lego-style Yellow-Gold (#f59e0b) or Classic Red/Orange
    p1.add_mesh(mesh_centered, color='#ea580c', specular=0.4, ambient=0.25, diffuse=0.8, smooth_shading=True)
    p1.camera_position = [(380, -320, 240), (plate_center_x, plate_center_y, 25), (0, 0, 1)]
    p1.camera.zoom(1.15)
    img1 = os.path.join(OUT_DIR, 'mco_core_one_lego_bed_overview.png')
    p1.screenshot(img1)
    print(f"  ✓ Rendered: {img1}")

    # 2. Close-up on the Lego Cockpit (Seat with 2 Studs, Dual Joysticks & Console)
    # The nose is at the front of the aircraft (lower X in centered space)
    cockpit_x = plate_center_x - (mesh_center_x - bb[0]) + 18.0
    p2 = pv.Plotter(off_screen=True, window_size=[1800, 1200])
    p2.set_background('#ffffff')
    p2.add_mesh(mesh_centered, color='#ea580c', specular=0.45, ambient=0.3, diffuse=0.8, smooth_shading=True)
    # Camera looking directly down-forward into the open cockpit
    p2.camera_position = [(cockpit_x - 45, -55, 55), (cockpit_x + 5, 0, 16), (0, 0, 1)]
    p2.camera.zoom(1.4)
    img2 = os.path.join(OUT_DIR, 'mco_core_one_lego_cockpit_zoom.png')
    p2.screenshot(img2)
    print(f"  ✓ Rendered: {img2}")

    # 3. Side Profile (Highlighting Extended Forward Keel & Cockpit Lines)
    p3 = pv.Plotter(off_screen=True, window_size=[1800, 1000])
    p3.set_background('#ffffff')
    p3.add_mesh(mesh_centered, color='#ea580c', specular=0.4, ambient=0.25, diffuse=0.8, smooth_shading=True)
    p3.camera_position = [(plate_center_x, -360, 30), (plate_center_x, 0, 30), (0, 0, 1)]
    p3.camera.zoom(1.2)
    img3 = os.path.join(OUT_DIR, 'mco_core_one_lego_side_profile.png')
    p3.screenshot(img3)
    print(f"  ✓ Rendered: {img3}")

    # 4. Front View Highlighting the Branded Bumper & Cockpit Framing
    p4 = pv.Plotter(off_screen=True, window_size=[1800, 1100])
    p4.set_background('#ffffff')
    p4.add_mesh(mesh_centered, color='#ea580c', specular=0.4, ambient=0.25, diffuse=0.8, smooth_shading=True)
    p4.camera_position = [(cockpit_x - 90, 0, 26), (cockpit_x + 20, 0, 20), (0, 0, 1)]
    p4.camera.zoom(1.3)
    img4 = os.path.join(OUT_DIR, 'mco_core_one_lego_front_view.png')
    p4.screenshot(img4)
    print(f"  ✓ Rendered: {img4}")

    # Copy images to artifact directory
    if os.path.exists(ARTIFACT_DIR):
        for img in [img1, img2, img3, img4]:
            shutil.copy(img, ARTIFACT_DIR)
        print("  ✓ Synced all Lego Edition renders to artifacts directory.")

if __name__ == '__main__':
    render_views()
