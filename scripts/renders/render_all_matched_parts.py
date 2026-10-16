#!/usr/bin/env python3
import os
import shutil
import pyvista as pv
import cadquery as cq

OUT_DIR = 'cad/manufactured'

MOUNT_STL = os.path.join(OUT_DIR, 's1223_airfoil_mount_starboard_2.4mm.stl')
SEG_STL   = os.path.join(OUT_DIR, 's1223_wing_segment_25cm_2.4mm.stl')
TIP_STL   = os.path.join(OUT_DIR, 's1223_wingtip_smooth_starboard_2.4mm.stl')

def render_matched_system():
    print("Rendering matched wing system components...")
    
    # -------------------------------------------------------------------------
    # 1. Mount Close-Up (Z=0 mating face showing through-cut ovals & ⌀10.5mm spar)
    # -------------------------------------------------------------------------
    m_mount = pv.read(MOUNT_STL)
    p_mount = pv.Plotter(off_screen=True, window_size=[1920, 1080])
    p_mount.set_background('white')
    p_mount.add_mesh(m_mount, color='#1d4ed8', specular=0.5, ambient=0.35, diffuse=0.8, smooth_shading=True)
    p_mount.camera_position = [(68, 15, -120), (68, 10, 0), (0, 1, 0)]
    p_mount.reset_camera()
    p_mount.camera.zoom(1.2)
    mount_png = os.path.join(OUT_DIR, 's1223_matched_mount.png')
    p_mount.screenshot(mount_png)
    print(f"Rendered: {mount_png}")

    # -------------------------------------------------------------------------
    # 2. 25cm Segment Isometric View (Warren truss, D-box, connector ovals, 3mm TE)
    # -------------------------------------------------------------------------
    m_seg = pv.read(SEG_STL)
    p_seg = pv.Plotter(off_screen=True, window_size=[1920, 1080])
    p_seg.set_background('white')
    p_seg.add_mesh(m_seg, color='#1d4ed8', specular=0.5, ambient=0.35, diffuse=0.8, smooth_shading=True)
    p_seg.camera_position = [(-160, -220, 90), (70, 10, -125), (0, 1, 0)]
    p_seg.reset_camera()
    p_seg.camera.zoom(1.3)
    seg_png = os.path.join(OUT_DIR, 's1223_matched_segment.png')
    p_seg.screenshot(seg_png)
    print(f"Rendered: {seg_png}")

    # -------------------------------------------------------------------------
    # 3. Full Wing Assembly (Mount + 4 Segments + Wingtip) in True 1-Meter Span
    # -------------------------------------------------------------------------
    mount_cq = cq.importers.importStep(os.path.join(OUT_DIR, 's1223_airfoil_mount_starboard_2.4mm.step')).val()
    seg_cq   = cq.importers.importStep(os.path.join(OUT_DIR, 's1223_wing_segment_25cm_2.4mm.step')).val()
    tip_cq   = cq.importers.importStep(os.path.join(OUT_DIR, 's1223_wingtip_smooth_starboard_2.4mm.step')).val()

    parts = [mount_cq]
    for i in range(4):
        parts.append(seg_cq.translate(cq.Vector(0, 0, -i * 250.0)))
    parts.append(tip_cq.translate(cq.Vector(0, 0, -1000.0)))
    wing_assy = cq.Compound.makeCompound(parts)

    assy_stl = os.path.join(OUT_DIR, 's1223_matched_full_wing.stl')
    cq.exporters.export(cq.Workplane().add(wing_assy), assy_stl, tolerance=0.03, angularTolerance=0.15)
    
    m_assy = pv.read(assy_stl)
    p_assy = pv.Plotter(off_screen=True, window_size=[1920, 1080])
    p_assy.set_background('white')
    p_assy.add_mesh(m_assy, color='#1d4ed8', specular=0.5, ambient=0.35, diffuse=0.8, smooth_shading=True)
    p_assy.camera_position = [(260, -850, -500), (70, 10, -500), (0, 1, 0)]
    p_assy.reset_camera()
    p_assy.camera.zoom(1.25)
    assy_png = os.path.join(OUT_DIR, 's1223_matched_full_wing.png')
    p_assy.screenshot(assy_png)
    print(f"Rendered: {assy_png}")

    # -------------------------------------------------------------------------
    # 4. Zoom on Joint Interface (Mount to Segment 1 Mating & Alignment)
    # -------------------------------------------------------------------------
    p_joint = pv.Plotter(off_screen=True, window_size=[1920, 1080])
    p_joint.set_background('white')
    p_joint.add_mesh(m_assy, color='#1d4ed8', specular=0.5, ambient=0.35, diffuse=0.8, smooth_shading=True)
    # Camera focused right on Z = 0 joint interface
    p_joint.camera_position = [(200, 140, 70), (70, 10, 0), (0, 1, 0)]
    p_joint.reset_camera()
    p_joint.camera.zoom(2.0)
    joint_png = os.path.join(OUT_DIR, 's1223_matched_joint_zoom.png')
    p_joint.screenshot(joint_png)
    print(f"Rendered: {joint_png}")

if __name__ == '__main__':
    render_matched_system()
