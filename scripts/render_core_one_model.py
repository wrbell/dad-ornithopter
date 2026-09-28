#!/usr/bin/env python3
"""
scripts/render_core_one_model.py
================================
Renders the dedicated Prusa CORE One 3D-printable MCO-1 aircraft model,
capturing the dynamic metachronal flapping wave across multiple viewpoints.
"""

import os
import shutil
import numpy as np
import pyvista as pv

OUT_DIR = 'cad/manufactured'
ARTIFACT_DIR = '/Users/drbell/.gemini/antigravity/brain/fcfca30b-4f75-4837-ba9b-50a1b0b3caba'
AIRCRAFT_STL = os.path.join(OUT_DIR, 'mco_core_one_complete_aircraft.stl')

def render_views():
    if not os.path.exists(AIRCRAFT_STL):
        print(f"Error: {AIRCRAFT_STL} not found.")
        return

    print("Rendering Prusa CORE One metachronal wave model...")
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

    # 1. Perspective View on Build Plate (Isometric)
    p1 = pv.Plotter(off_screen=True, window_size=[1800, 1100])
    p1.set_background('#f8fafc')
    p1.add_mesh(plate, color='#1e293b', specular=0.2, ambient=0.3, diffuse=0.7)
    p1.add_mesh(mesh_centered, color='#ea580c', specular=0.4, ambient=0.25, diffuse=0.8, smooth_shading=True)
    p1.camera_position = [(380, -320, 240), (plate_center_x, plate_center_y, 25), (0, 0, 1)]
    p1.camera.zoom(1.15)
    img1 = os.path.join(OUT_DIR, 'mco_core_one_bed_overview.png')
    p1.screenshot(img1)
    print(f"  ✓ Rendered: {img1}")

    # 2. Side Elevation Profile (Highlighting the Metachronal Wave along the Fuselage)
    p2 = pv.Plotter(off_screen=True, window_size=[1800, 1000])
    p2.set_background('#ffffff')
    p2.add_mesh(mesh_centered, color='#ea580c', specular=0.4, ambient=0.25, diffuse=0.8, smooth_shading=True)
    p2.camera_position = [(plate_center_x, -360, 30), (plate_center_x, 0, 30), (0, 0, 1)]
    p2.camera.zoom(1.2)
    img2 = os.path.join(OUT_DIR, 'mco_core_one_side_profile.png')
    p2.screenshot(img2)
    print(f"  ✓ Rendered: {img2}")

    # 3. Head-On Front View (Highlighting Bilateral Flapping Wave Crests & Troughs)
    p3 = pv.Plotter(off_screen=True, window_size=[1800, 1100])
    p3.set_background('#ffffff')
    p3.add_mesh(mesh_centered, color='#ea580c', specular=0.4, ambient=0.25, diffuse=0.8, smooth_shading=True)
    p3.camera_position = [(-160, 0, 45), (plate_center_x, 0, 35), (0, 0, 1)]
    p3.camera.zoom(1.2)
    img3 = os.path.join(OUT_DIR, 'mco_core_one_front_wave.png')
    p3.screenshot(img3)
    print(f"  ✓ Rendered: {img3}")

    # 4. Dynamic Front 3/4 Perspective View
    p4 = pv.Plotter(off_screen=True, window_size=[1800, 1100])
    p4.set_background('#ffffff')
    p4.add_mesh(mesh_centered, color='#ea580c', specular=0.4, ambient=0.25, diffuse=0.8, smooth_shading=True)
    p4.camera_position = [(-80, -260, 160), (plate_center_x, 0, 30), (0, 0, 1)]
    p4.camera.zoom(1.15)
    img4 = os.path.join(OUT_DIR, 'mco_core_one_iso_dynamic.png')
    p4.screenshot(img4)
    print(f"  ✓ Rendered: {img4}")

    # 5. Close-up on Fuselage Truss, Landing Gear, and Teardrop Pod
    p5 = pv.Plotter(off_screen=True, window_size=[1800, 1100])
    p5.set_background('#ffffff')
    p5.add_mesh(mesh_centered, color='#ea580c', specular=0.4, ambient=0.25, diffuse=0.8, smooth_shading=True)
    p5.camera_position = [(plate_center_x - 30, -120, 60), (plate_center_x - 10, 0, 10), (0, 0, 1)]
    p5.camera.zoom(1.4)
    img5 = os.path.join(OUT_DIR, 'mco_core_one_gear_truss_zoom.png')
    p5.screenshot(img5)
    print(f"  ✓ Rendered: {img5}")

    # Copy images to artifact directory
    if os.path.exists(ARTIFACT_DIR):
        for img in [img1, img2, img3, img4, img5]:
            shutil.copy(img, ARTIFACT_DIR)
        print("  ✓ Synced all 5 renders to artifacts directory.")

if __name__ == '__main__':
    render_views()
