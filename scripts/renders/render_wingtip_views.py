#!/usr/bin/env python3
import os
import shutil
import pyvista as pv

OUT_DIR = 'cad/manufactured'
STL_PATH = os.path.join(OUT_DIR, 's1223_wingtip_smooth_starboard_2.4mm.stl')

def render_wingtip():
    mesh = pv.read(STL_PATH)
    
    # -------------------------------------------------------------------------
    # 1. Full Wingtip Isometric Overview
    # -------------------------------------------------------------------------
    m_iso = mesh.copy()
    m_iso.translate((-71.8, -9.9, 62.6), inplace=True)
    m_iso.rotate_x(-90, inplace=True)
    m_iso.rotate_z(-90, inplace=True)
    
    p_iso = pv.Plotter(off_screen=True, window_size=[1920, 1080])
    p_iso.set_background('white')
    p_iso.add_mesh(m_iso, color='#1d4ed8', specular=0.5, ambient=0.35, diffuse=0.8, smooth_shading=True)
    p_iso.camera_position = [(-110, -160, 140), (0, 0, 0), (0, 0, 1)]
    p_iso.reset_camera()
    p_iso.camera.zoom(1.25)
    iso_file = os.path.join(OUT_DIR, 's1223_wingtip_skeleton_iso.png')
    p_iso.screenshot(iso_file)
    print(f"Rendered: {iso_file}")

    # -------------------------------------------------------------------------
    # 2. Inboard Root Connector Close-Up (⌀10.5mm spar, 4 oval lightening holes, male lip)
    # -------------------------------------------------------------------------
    p_root = pv.Plotter(off_screen=True, window_size=[1920, 1080])
    p_root.set_background('white')
    p_root.add_mesh(mesh, color='#1d4ed8', specular=0.5, ambient=0.35, diffuse=0.8, smooth_shading=True)
    p_root.camera_position = [(68, 15, 140), (68, 10, 0), (0, 1, 0)]
    p_root.reset_camera()
    p_root.camera.zoom(1.2)
    root_file = os.path.join(OUT_DIR, 's1223_wingtip_root_connector.png')
    p_root.screenshot(root_file)
    print(f"Rendered: {root_file}")

    # -------------------------------------------------------------------------
    # 3. Planform Tilted View (Solid D-box, Rib ovals, 3mm TE, intact distal apex)
    # -------------------------------------------------------------------------
    p_top = pv.Plotter(off_screen=True, window_size=[1920, 1080])
    p_top.set_background('white')
    p_top.add_mesh(mesh, color='#1d4ed8', specular=0.4, ambient=0.35, diffuse=0.8, smooth_shading=True)
    p_top.camera_position = [(72, 220, 30), (72, 10, -63), (-1, 0, 0)]
    p_top.reset_camera()
    p_top.camera.zoom(1.25)
    top_file = os.path.join(OUT_DIR, 's1223_wingtip_skeleton_top.png')
    p_top.screenshot(top_file)
    print(f"Rendered: {top_file}")

    # -------------------------------------------------------------------------
    # 4. Trailing Edge Zoom (Solid 3.0mm TE rail)
    # -------------------------------------------------------------------------
    p_te = pv.Plotter(off_screen=True, window_size=[1920, 1080])
    p_te.set_background('white')
    p_te.add_mesh(mesh, color='#1d4ed8', specular=0.4, ambient=0.3, diffuse=0.8, smooth_shading=True)
    p_te.camera_position = [(150, 45, -30), (105, 10, -65), (0, 1, 0)]
    p_te.reset_camera()
    p_te.camera.zoom(1.3)
    te_file = os.path.join(OUT_DIR, 's1223_wingtip_skeleton_te_zoom.png')
    p_te.screenshot(te_file)
    print(f"Rendered: {te_file}")

if __name__ == '__main__':
    render_wingtip()
