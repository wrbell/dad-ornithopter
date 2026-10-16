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

# Representative Servos (AGFRC B24CLM at ±32mm, Y=-22mm)
servo_left = pv.Cube(center=[-32.0, -22.0, 18.0], x_length=12.0, y_length=24.0, z_length=27.5)
servo_right = pv.Cube(center=[32.0, -22.0, 18.0], x_length=12.0, y_length=24.0, z_length=27.5)

# 10mm Carbon Fiber Spar Tube
spar_tube = pv.Cylinder(center=[0.0, 45.0, 54.0], direction=[0.0, 1.0, 0.0], radius=5.0, height=85.0, resolution=40)

# Colors
C_DRUM = "#0284c7"     # Precision Blue (Skeletonized Drum)
C_WRIST = "#f59e0b"    # Gold / Amber (2DOF Wrist & Spar Clamp)
C_SERVO = "#ef4444"    # Precision Red (Aft Embedded Servos)
C_SPAR = "#1e293b"     # Matte Carbon Fiber
C_FAIRING = "#e2e8f0"  # Pearl White Translucent (LW-PLA Fairing)

p = pv.Plotter(shape=(1, 4), off_screen=True, window_size=[2400, 700])
p.set_background("#0f172a")

titles = [
    "Isometric (NACA 0018 Fairing)",
    "Core Skeleton (Fairing Hidden)",
    "Top Layout (Aft Servos & 40mm Core)",
    "Front Elevation (Bevel Axis)"
]

for i in range(4):
    p.subplot(0, i)
    p.add_mesh(mesh_drum, color=C_DRUM, specular=0.5, ambient=0.25, diffuse=0.75, smooth_shading=True)
    p.add_mesh(mesh_wrist, color=C_WRIST, specular=0.6, ambient=0.3, diffuse=0.7, smooth_shading=True)
    p.add_mesh(servo_left, color=C_SERVO, specular=0.4, ambient=0.3)
    p.add_mesh(servo_right, color=C_SERVO, specular=0.4, ambient=0.3)
    p.add_mesh(spar_tube, color=C_SPAR, specular=0.3, ambient=0.3)
    
    if i == 0:
        p.add_mesh(mesh_fairing, color=C_FAIRING, opacity=0.30, specular=0.3, smooth_shading=True)
        p.view_isometric()
    elif i == 1:
        p.view_isometric()
    elif i == 2:
        p.view_xy()
    elif i == 3:
        p.view_xz()
        
    p.add_text(titles[i], position="upper_edge", font_size=12, color="#38bdf8")
    p.camera.zoom(1.25)

out_path = os.path.join(OUT_DIR, "mco_merged_wrist_final_4view.png")
p.screenshot(out_path)
p.close()
print(f"Rendered {out_path}")
