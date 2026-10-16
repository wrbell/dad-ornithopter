import os
import pyvista as pv
import numpy as np

pv.set_plot_theme("document")

OUT_DIR = "cad/manufactured"

stl_drum = os.path.join(OUT_DIR, "mco_skeletonized_sector_wheel.stl")
stl_wrist = os.path.join(OUT_DIR, "mco_integrated_2dof_wrist.stl")
stl_fairing = os.path.join(OUT_DIR, "mco_naca0018_wrist_fairing_lwpla.stl")

mesh_drum = pv.read(stl_drum)
mesh_wrist = pv.read(stl_wrist)
mesh_fairing = pv.read(stl_fairing)

# Add representative servos into the bays (AGFRC B24CLM micro servos at ±32mm, Y=-20mm)
servo_left = pv.Cube(center=[-32.0, -20.0, 20.0], x_length=12.0, y_length=23.0, z_length=27.5)
servo_right = pv.Cube(center=[32.0, -20.0, 20.0], x_length=12.0, y_length=23.0, z_length=27.5)

# Add 10mm Carbon Spar Tube (extending outboard)
spar_tube = pv.Cylinder(center=[0.0, 50.0, 54.0], direction=[0.0, 1.0, 0.0], radius=5.0, height=90.0, resolution=40)

# Colors
C_DRUM = "#0284c7"     # Precision Cyan/Blue (Skeletonized Sector Drum)
C_WRIST = "#f59e0b"    # Anodized Gold/Amber (2DOF Differential Bevel Wrist)
C_SERVO = "#ef4444"    # Precision Red (Embedded Servos)
C_SPAR = "#1e293b"     # Matte Carbon Fiber
C_FAIRING = "#e2e8f0"  # Matte Translucent Pearl White (LW-PLA Fairing)

def render_assembly(show_fairing=True, opacity_fairing=0.35, fname="merged_wrist_iso.png", cam_view="iso"):
    p = pv.Plotter(off_screen=True, window_size=[1920, 1080])
    p.set_background("#0f172a")
    
    p.add_mesh(mesh_drum, color=C_DRUM, specular=0.5, ambient=0.25, diffuse=0.75, smooth_shading=True)
    p.add_mesh(mesh_wrist, color=C_WRIST, specular=0.6, ambient=0.3, diffuse=0.7, smooth_shading=True)
    p.add_mesh(servo_left, color=C_SERVO, specular=0.4, ambient=0.3)
    p.add_mesh(servo_right, color=C_SERVO, specular=0.4, ambient=0.3)
    p.add_mesh(spar_tube, color=C_SPAR, specular=0.3, ambient=0.3)
    
    if show_fairing:
        p.add_mesh(mesh_fairing, color=C_FAIRING, opacity=opacity_fairing, specular=0.3, smooth_shading=True)
        
    if cam_view == "iso":
        p.view_isometric()
    elif cam_view == "top":
        p.view_xy()
    elif cam_view == "front":
        p.view_xz()
    elif cam_view == "side":
        p.view_yz()
        
    p.camera.zoom(1.2)
    out_path = os.path.join(OUT_DIR, fname)
    p.screenshot(out_path)
    p.close()
    print(f"Rendered {out_path}")

# 1. Isometric (Skeleton & Internal Servos Visible, translucent fairing)
render_assembly(show_fairing=True, opacity_fairing=0.30, fname="mco_merged_wrist_fairing_iso.png", cam_view="iso")

# 2. Isometric Mechanical Core (Fairing Hidden - Full Skeleton & Aft Servos Inspection)
render_assembly(show_fairing=False, fname="mco_merged_wrist_core_iso.png", cam_view="iso")

# 3. Top Down (Demonstrating 40mm Hollow Core & Aft-Shifted Servos)
render_assembly(show_fairing=False, fname="mco_merged_wrist_top_layout.png", cam_view="top")

# 4. Front View (Pitch/Feathering Axis & Capstan Rim Profile)
render_assembly(show_fairing=False, fname="mco_merged_wrist_front_profile.png", cam_view="front")

# 5. Side View (NACA 0018 Airfoil Chordwise Orientation at 90° to Sector Wheel)
render_assembly(show_fairing=True, opacity_fairing=0.40, fname="mco_merged_wrist_naca0018_side.png", cam_view="side")
