#!/usr/bin/env python3
"""
scripts/render_core_one_lego_model.py
=====================================
Renders the special Lego Edition of the MCO-1 8-Wing Tandem Aircraft,
focusing on the forward-extended keel cockpit, open cockpit floor (seat removed),
LEGO connection studs, control panel, joysticks, and front bumper.
"""

import os
import shutil
import numpy as np
import pyvista as pv
from PIL import Image, ImageDraw, ImageFont

OUT_DIR = 'cad/manufactured'
ARTIFACT_DIR = '/Users/drbell/.gemini/antigravity/brain/6bbf0df4-3e68-4851-9322-6154657ee0b5'
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

    # Extract clean CAD feature edges (sharp outlines without internal triangulation clutter)
    edges = mesh.extract_feature_edges(boundary_edges=True, feature_edges=True, manifold_edges=False, feature_angle=30)
    edges_centered = edges.translate((plate_center_x - mesh_center_x, plate_center_y - mesh_center_y, -bb[4]))

    # Calculate exact cockpit coordinates in centered space
    x_nose_centered = bb[0] - mesh_center_x + plate_center_x
    cockpit_center_x = x_nose_centered + 24.0

    # Plate mesh
    plate = pv.Cube(center=(plate_center_x, plate_center_y, -1.0), x_length=plate_x, y_length=plate_y, z_length=2.0)

    # 1. Perspective View on Build Plate (Isometric Overview)
    p1 = pv.Plotter(off_screen=True, window_size=[1800, 1100])
    p1.set_background('#f8fafc')
    p1.add_mesh(plate, color='#1e293b', specular=0.2, ambient=0.3, diffuse=0.7)
    p1.add_mesh(mesh_centered, color='#ea580c', specular=0.4, ambient=0.25, diffuse=0.8, smooth_shading=True)
    p1.add_mesh(edges_centered, color='#0f172a', line_width=1.0)
    p1.camera_position = [(380, -320, 240), (plate_center_x, plate_center_y, 25), (0, 0, 1)]
    p1.camera.zoom(1.15)
    img1 = os.path.join(OUT_DIR, 'mco_core_one_lego_bed_overview.png')
    p1.screenshot(img1)
    print(f"  ✓ Rendered: {img1}")

    # 2. Elevated Perspective Close-Up on Cockpit Floor (Seat Removed, Studs on Floor)
    p2 = pv.Plotter(off_screen=True, window_size=[1800, 1200])
    p2.set_background('#ffffff')
    p2.add_mesh(mesh_centered, color='#ea580c', specular=0.45, ambient=0.35, diffuse=0.75, smooth_shading=True)
    p2.add_mesh(edges_centered, color='#0f172a', line_width=1.5)
    p2.camera_position = [(cockpit_center_x - 38, -50, 42), (cockpit_center_x + 5, 0, 18), (0, 0, 1)]
    p2.camera.zoom(1.25)
    img2 = os.path.join(OUT_DIR, 'mco_core_one_lego_cockpit_zoom.png')
    p2.screenshot(img2)
    print(f"  ✓ Rendered: {img2}")

    # 2B. Direct Top-Down (Plan) View of Open Cockpit Floor
    p_top = pv.Plotter(off_screen=True, window_size=[1800, 1200])
    p_top.set_background('#ffffff')
    p_top.add_mesh(mesh_centered, color='#ea580c', specular=0.4, ambient=0.4, diffuse=0.75, smooth_shading=True)
    p_top.add_mesh(edges_centered, color='#0f172a', line_width=1.5)
    p_top.camera_position = [(cockpit_center_x + 2, 0, 130), (cockpit_center_x + 2, 0, 15), (1, 0, 0)]
    p_top.camera.zoom(1.25)
    img_top = os.path.join(OUT_DIR, 'mco_core_one_lego_cockpit_top_down.png')
    p_top.screenshot(img_top)
    print(f"  ✓ Rendered: {img_top}")

    # 2C. Direct Side Profile of Cockpit (Showing flat floor and stud profile)
    p_side_c = pv.Plotter(off_screen=True, window_size=[1800, 1000])
    p_side_c.set_background('#ffffff')
    p_side_c.add_mesh(mesh_centered, color='#ea580c', specular=0.4, ambient=0.35, diffuse=0.75, smooth_shading=True)
    p_side_c.add_mesh(edges_centered, color='#0f172a', line_width=1.5)
    p_side_c.camera_position = [(cockpit_center_x, -110, 20), (cockpit_center_x, 0, 20), (0, 0, 1)]
    p_side_c.camera.zoom(1.4)
    img_side_c = os.path.join(OUT_DIR, 'mco_core_one_lego_cockpit_side.png')
    p_side_c.screenshot(img_side_c)
    print(f"  ✓ Rendered: {img_side_c}")

    # 3. Full Aircraft Side Profile
    p3 = pv.Plotter(off_screen=True, window_size=[1800, 1000])
    p3.set_background('#ffffff')
    p3.add_mesh(mesh_centered, color='#ea580c', specular=0.4, ambient=0.25, diffuse=0.8, smooth_shading=True)
    p3.add_mesh(edges_centered, color='#0f172a', line_width=1.0)
    p3.camera_position = [(plate_center_x, -360, 30), (plate_center_x, 0, 30), (0, 0, 1)]
    p3.camera.zoom(1.2)
    img3 = os.path.join(OUT_DIR, 'mco_core_one_lego_side_profile.png')
    p3.screenshot(img3)
    print(f"  ✓ Rendered: {img3}")

    # 4. Front View Highlighting Bumper & Cockpit Framing
    p4 = pv.Plotter(off_screen=True, window_size=[1800, 1100])
    p4.set_background('#ffffff')
    p4.add_mesh(mesh_centered, color='#ea580c', specular=0.4, ambient=0.25, diffuse=0.8, smooth_shading=True)
    p4.add_mesh(edges_centered, color='#0f172a', line_width=1.0)
    p4.camera_position = [(cockpit_center_x - 90, 0, 26), (cockpit_center_x + 20, 0, 20), (0, 0, 1)]
    p4.camera.zoom(1.3)
    img4 = os.path.join(OUT_DIR, 'mco_core_one_lego_front_view.png')
    p4.screenshot(img4)
    print(f"  ✓ Rendered: {img4}")

    # 5. Close-up on Stout Aircraft Tail Skid Bumper
    p5 = pv.Plotter(off_screen=True, window_size=[1800, 1100])
    p5.set_background('#ffffff')
    p5.add_mesh(mesh_centered, color='#ea580c', specular=0.4, ambient=0.25, diffuse=0.8, smooth_shading=True)
    p5.add_mesh(edges_centered, color='#0f172a', line_width=1.0)
    p5.camera_position = [(200, -85, 25), (165, 0, 8), (0, 0, 1)]
    p5.camera.zoom(1.3)
    img5 = os.path.join(OUT_DIR, 'mco_core_one_lego_gear_truss_zoom.png')
    p5.screenshot(img5)
    print(f"  ✓ Rendered: {img5}")

    # 6. Generate Annotated Diagram with Direct Callouts
    img_annotated_path = os.path.join(OUT_DIR, 'mco_core_one_lego_cockpit_annotated.png')
    annotated_img = Image.open(img2)
    draw = ImageDraw.Draw(annotated_img)

    try:
        font = ImageFont.truetype('/System/Library/Fonts/Helvetica.ttc', 28)
        title_font = ImageFont.truetype('/System/Library/Fonts/Helvetica.ttc', 36)
    except Exception:
        font = ImageFont.load_default()
        title_font = font

    # Title header banner
    draw.rectangle([(40, 40), (1150, 115)], fill='#0f172a')
    draw.text((60, 58), 'MCO-1 LEGO COCKPIT: SEAT REMOVED & STUDS ON FLOOR', fill='#ffffff', font=title_font)

    def draw_callout(box_xy, text, target_xy, bg='#1e293b', fg='#ffffff'):
        bx0, by0, bx1, by1 = box_xy
        draw.rectangle([bx0, by0, bx1, by1], fill=bg, outline='#38bdf8', width=2)
        draw.text((bx0 + 15, by0 + 12), text, fill=fg, font=font)
        draw.line([(bx0 + bx1)/2, by1, target_xy[0], target_xy[1]], fill='#38bdf8', width=3)
        draw.ellipse([target_xy[0]-5, target_xy[1]-5, target_xy[0]+5, target_xy[1]+5], fill='#38bdf8')

    draw_callout((700, 180, 1280, 245), 'DUAL LEGO STUDS (ON FLAT FLOOR)\nØ4.8mm x 1.7mm (8.0mm pitch)', (920, 520))
    draw_callout((1150, 720, 1720, 785), 'OPEN COCKPIT FLOOR (SEAT REMOVED)\n100% unrestricted clearance for figure', (1200, 560))
    draw_callout((60, 720, 580, 785), 'DASHBOARD & JOYSTICKS\nFlight instruments & controls', (320, 450))

    annotated_img.save(img_annotated_path)
    print(f"  ✓ Rendered: {img_annotated_path}")

    # Copy images to artifact directory
    all_imgs = [img1, img2, img_top, img_side_c, img3, img4, img5, img_annotated_path]
    if os.path.exists(ARTIFACT_DIR):
        for img in all_imgs:
            shutil.copy(img, ARTIFACT_DIR)
        print("  ✓ Synced all Lego Edition renders to artifacts directory.")

if __name__ == '__main__':
    render_views()
