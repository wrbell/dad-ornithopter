#!/usr/bin/env python3
"""
MCO-1 Visualization Script: Multi-Angle High-Resolution Renders
Renders both the parametric fuselage frame and the complete 8-wing tandem aircraft,
matching the camera viewpoints of the user's physical wooden sculpture photos.
"""

import os
import pyvista as pv

OUT_DIR = 'cad/manufactured'
FUSELAGE_STL = os.path.join(OUT_DIR, 'mco_fuselage_assembly.stl')
AIRCRAFT_STL = os.path.join(OUT_DIR, 'mco_complete_8wing_aircraft.stl')

def render_views():
    print("=" * 70)
    print("RENDERING MCO-1 FUSELAGE & COMPLETE 8-WING AIRCRAFT")
    print("=" * 70)

    # -------------------------------------------------------------------------
    # 1. Renders of Parametric Fuselage Frame Alone
    # -------------------------------------------------------------------------
    if os.path.exists(FUSELAGE_STL):
        print("\n[1/2] Rendering Fuselage Frame...")
        reader_fus = pv.get_reader(FUSELAGE_STL)
        mesh_fus = reader_fus.read()

        # 1A. Fuselage Side View (matching Photo 3)
        p_side = pv.Plotter(off_screen=True, window_size=[1600, 900])
        p_side.set_background('white')
        p_side.add_mesh(mesh_fus, color='#0284c7', specular=0.3, ambient=0.2, diffuse=0.8, smooth_shading=True)
        p_side.camera_position = [(350, -1400, 25), (350, 0, 25), (0, 0, 1)]
        p_side.camera.zoom(1.2)
        p_side.screenshot(os.path.join(OUT_DIR, 'mco_fuselage_side.png'))
        print("  ✓ Rendered: mco_fuselage_side.png")

        # 1B. Fuselage Perspective 3/4 View (matching Photo 1)
        p_iso = pv.Plotter(off_screen=True, window_size=[1600, 1000])
        p_iso.set_background('white')
        p_iso.add_mesh(mesh_fus, color='#0284c7', specular=0.4, ambient=0.2, diffuse=0.8, smooth_shading=True)
        p_iso.camera_position = [(-400, -800, 600), (350, 0, 25), (0, 0, 1)]
        p_iso.camera.zoom(1.2)
        p_iso.screenshot(os.path.join(OUT_DIR, 'mco_fuselage_iso.png'))
        print("  ✓ Rendered: mco_fuselage_iso.png")

        # 1C. Fuselage Front View (matching Photo 4)
        p_front = pv.Plotter(off_screen=True, window_size=[1400, 1000])
        p_front.set_background('white')
        p_front.add_mesh(mesh_fus, color='#0284c7', specular=0.3, ambient=0.2, diffuse=0.8, smooth_shading=True)
        p_front.camera_position = [(-900, 0, 40), (350, 0, 25), (0, 0, 1)]
        p_front.camera.zoom(1.3)
        p_front.screenshot(os.path.join(OUT_DIR, 'mco_fuselage_front.png'))
        print("  ✓ Rendered: mco_fuselage_front.png")

    # -------------------------------------------------------------------------
    # 2. Renders of Complete 8-Wing Tandem Aircraft
    # -------------------------------------------------------------------------
    if os.path.exists(AIRCRAFT_STL):
        print("\n[2/2] Rendering Complete 8-Wing Tandem Aircraft...")
        reader_air = pv.get_reader(AIRCRAFT_STL)
        mesh_air = reader_air.read()

        # 2A. Perspective View Matching User Photo 1 (front 3/4 high angle)
        p1 = pv.Plotter(off_screen=True, window_size=[1920, 1200])
        p1.set_background('white')
        p1.add_mesh(mesh_air, color='#d97706', specular=0.4, ambient=0.2, diffuse=0.8, smooth_shading=True)
        p1.camera_position = [(-800, -2600, 1500), (500, 0, 0), (0, 0, 1)]
        p1.camera.zoom(1.05)
        p1.screenshot(os.path.join(OUT_DIR, 'mco_aircraft_photo1_match.png'))
        print("  ✓ Rendered: mco_aircraft_photo1_match.png")

        # 2B. Perspective View Matching User Photo 2 (rear 3/4 angle)
        p2 = pv.Plotter(off_screen=True, window_size=[1920, 1200])
        p2.set_background('white')
        p2.add_mesh(mesh_air, color='#d97706', specular=0.4, ambient=0.2, diffuse=0.8, smooth_shading=True)
        p2.camera_position = [(2000, -2600, 1100), (450, 0, 30), (0, 0, 1)]
        p2.camera.zoom(1.05)
        p2.screenshot(os.path.join(OUT_DIR, 'mco_aircraft_photo2_match.png'))
        print("  ✓ Rendered: mco_aircraft_photo2_match.png")

        # 2C. Direct Side Elevation Matching User Photo 3
        p3 = pv.Plotter(off_screen=True, window_size=[1920, 1080])
        p3.set_background('white')
        p3.add_mesh(mesh_air, color='#d97706', specular=0.3, ambient=0.2, diffuse=0.8, smooth_shading=True)
        p3.camera_position = [(500, -3200, 20), (500, 0, 20), (0, 0, 1)]
        p3.camera.zoom(1.0)
        p3.screenshot(os.path.join(OUT_DIR, 'mco_aircraft_photo3_match.png'))
        print("  ✓ Rendered: mco_aircraft_photo3_match.png")

        # 2D. Front View Down Centerline Matching User Photo 4
        p4 = pv.Plotter(off_screen=True, window_size=[1920, 1200])
        p4.set_background('white')
        p4.add_mesh(mesh_air, color='#d97706', specular=0.3, ambient=0.2, diffuse=0.8, smooth_shading=True)
        p4.camera_position = [(-3400, 0, 120), (500, 0, 20), (0, 0, 1)]
        p4.camera.zoom(1.1)
        p4.screenshot(os.path.join(OUT_DIR, 'mco_aircraft_photo4_match.png'))
        print("  ✓ Rendered: mco_aircraft_photo4_match.png")

    # -------------------------------------------------------------------------
    # 3. Renders of 1-Meter Wing Skeleton & Rib Lightening Holes
    # -------------------------------------------------------------------------
    print("\n[3/3] Rendering 1-Meter Wing Skeleton & Rib Lightening Holes...")
    import cadquery as cq
    import shutil

    MOUNT_STEP = os.path.join(OUT_DIR, 's1223_airfoil_mount_starboard_2.4mm.step')
    SEG_STEP   = os.path.join(OUT_DIR, 's1223_wing_segment_25cm_2.4mm.step')
    TIP_STEP   = os.path.join(OUT_DIR, 's1223_wingtip_smooth_starboard_2.4mm.step')

    if os.path.exists(MOUNT_STEP) and os.path.exists(SEG_STEP) and os.path.exists(TIP_STEP):
        mount = cq.importers.importStep(MOUNT_STEP).val()
        seg   = cq.importers.importStep(SEG_STEP).val()
        tip   = cq.importers.importStep(TIP_STEP).val()

        wing_parts = [mount]
        for idx in range(4):
            wing_parts.append(seg.translate(cq.Vector(0, 0, -idx * 250.0)))
        wing_parts.append(tip.translate(cq.Vector(0, 0, -1000.0)))
        wing_compound = cq.Compound.makeCompound(wing_parts)

        wing_stl = os.path.join(OUT_DIR, 's1223_full_wing_skeleton_assembly.stl')
        cq.exporters.export(cq.Workplane().add(wing_compound), wing_stl, tolerance=0.04, angularTolerance=0.2)

        reader_wing = pv.get_reader(wing_stl)
        mesh_wing = reader_wing.read()

        # 3A. Full Wing Skeleton Isometric Overview
        pw = pv.Plotter(off_screen=True, window_size=[1920, 1080])
        pw.set_background('white')
        pw.add_mesh(mesh_wing, color='#2563eb', specular=0.4, ambient=0.2, diffuse=0.8, smooth_shading=True)
        pw.camera_position = [(250, -800, -500), (70, 10, -500), (0, 1, 0)]
        pw.camera.zoom(1.15)
        pw.screenshot(os.path.join(OUT_DIR, 's1223_full_wing_skeleton_assembly.png'))
        print("  ✓ Rendered: s1223_full_wing_skeleton_assembly.png")

        # 3B. Close-up Zoom on Rib Lightening Holes (2/3 vertical rib height)
        pz = pv.Plotter(off_screen=True, window_size=[1600, 1200])
        pz.set_background('white')
        pz.add_mesh(mesh_wing, color='#2563eb', specular=0.4, ambient=0.2, diffuse=0.8, smooth_shading=True)
        pz.camera_position = [(200, 120, -130), (85, 12, -100), (0, 1, 0)]
        pz.camera.zoom(1.3)
        pz.screenshot(os.path.join(OUT_DIR, 's1223_wing_rib_lightening_holes_zoom.png'))
        print("  ✓ Rendered: s1223_wing_rib_lightening_holes_zoom.png")

        # 3C. Dedicated Wingtip Skeleton Renders (Oval Holes & 3.0mm TE)
        tip_stl_path = os.path.join(OUT_DIR, 's1223_wingtip_smooth_starboard_2.4mm.stl')
        if os.path.exists(tip_stl_path):
            mesh_tip = pv.get_reader(tip_stl_path).read()

            # Isometric view of wingtip
            ptip_iso = pv.Plotter(off_screen=True, window_size=[1600, 1200])
            ptip_iso.set_background('white')
            ptip_iso.add_mesh(mesh_tip, color='#2563eb', specular=0.4, ambient=0.2, diffuse=0.8, smooth_shading=True)
            ptip_iso.camera_position = [(180, 100, 40), (70, 10, -65), (0, 1, 0)]
            ptip_iso.camera.zoom(1.15)
            ptip_iso.screenshot(os.path.join(OUT_DIR, 's1223_wingtip_skeleton_iso.png'))
            print("  ✓ Rendered: s1223_wingtip_skeleton_iso.png")

            # Top-down view showing oval lightening holes clearly
            ptip_top = pv.Plotter(off_screen=True, window_size=[1600, 1200])
            ptip_top.set_background('white')
            ptip_top.add_mesh(mesh_tip, color='#2563eb', specular=0.4, ambient=0.2, diffuse=0.8, smooth_shading=True)
            ptip_top.camera_position = [(70, 220, -65), (70, 10, -65), (0, 0, -1)]
            ptip_top.camera.zoom(1.1)
            ptip_top.screenshot(os.path.join(OUT_DIR, 's1223_wingtip_skeleton_top.png'))
            print("  ✓ Rendered: s1223_wingtip_skeleton_top.png")

            # Trailing edge close-up zoom
            ptip_te = pv.Plotter(off_screen=True, window_size=[1600, 1200])
            ptip_te.set_background('white')
            ptip_te.add_mesh(mesh_tip, color='#2563eb', specular=0.4, ambient=0.2, diffuse=0.8, smooth_shading=True)
            ptip_te.camera_position = [(140, 40, -50), (105, 10, -70), (0, 1, 0)]
            ptip_te.camera.zoom(1.4)
            ptip_te.screenshot(os.path.join(OUT_DIR, 's1223_wingtip_skeleton_te_zoom.png'))
            print("  ✓ Rendered: s1223_wingtip_skeleton_te_zoom.png")

    print("\nAll renders completed successfully!")

if __name__ == '__main__':
    render_views()
