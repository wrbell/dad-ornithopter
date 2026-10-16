import os
import pyvista as pv
import numpy as np

pv.set_plot_theme("document")

OUT_DIR = "cad/manufactured"
os.makedirs(OUT_DIR, exist_ok=True)

parts = [
    ("cad/wrist/00_Bottom_Piece_v1_v1.stl", "#475569", "Base Chassis (Bottom Piece)"),
    ("cad/wrist/04_Top_piece_v1.stl", "#64748b", "Top Shell / Mounting Clamp"),
    ("cad/wrist/01_Motor__gear_v8.stl", "#0284c7", "Actuator 1: Neo 550 + UltraPlanetary"),
    ("cad/wrist/02_Motor__gear_v8.stl", "#06b6d4", "Actuator 2: Neo 550 + UltraPlanetary"),
    ("cad/wrist/03_New_update_bearings_v28.stl", "#f59e0b", "2DOF Differential Bevel Gearset & Axles"),
    ("cad/wrist/05_Thrifty_Bot_Absolute_Magnetic_Encoder_v4.stl", "#a855f7", "Thrifty Absolute Magnetic Encoder"),
    ("cad/wrist/06_Magnet_v1.stl", "#ef4444", "Target Sensing Magnet")
]

meshes = []
for p, col, name in parts:
    if os.path.exists(p):
        m = pv.read(p)
        meshes.append((m, col, name))

def setup_plotter():
    p = pv.Plotter(off_screen=True, window_size=[1920, 1080])
    p.set_background("#0f172a") # dark slate
    for m, col, _ in meshes:
        p.add_mesh(m, color=col, specular=0.5, specular_power=20, ambient=0.25, diffuse=0.75, smooth_shading=True)
    return p

# 1. Isometric Render
p_iso = setup_plotter()
p_iso.view_isometric()
p_iso.camera.zoom(1.2)
iso_path = os.path.join(OUT_DIR, "robotic_wrist_v1_iso.png")
p_iso.screenshot(iso_path)
p_iso.close()
print(f"Rendered {iso_path}")

# 2. Front Render
p_front = setup_plotter()
p_front.view_xz()
p_front.camera.zoom(1.2)
front_path = os.path.join(OUT_DIR, "robotic_wrist_v1_front.png")
p_front.screenshot(front_path)
p_front.close()
print(f"Rendered {front_path}")

# 3. Top Render
p_top = setup_plotter()
p_top.view_xy()
p_top.camera.zoom(1.2)
top_path = os.path.join(OUT_DIR, "robotic_wrist_v1_top.png")
p_top.screenshot(top_path)
p_top.close()
print(f"Rendered {top_path}")

# 4. Side Render
p_side = setup_plotter()
p_side.view_yz()
p_side.camera.zoom(1.2)
side_path = os.path.join(OUT_DIR, "robotic_wrist_v1_side.png")
p_side.screenshot(side_path)
p_side.close()
print(f"Rendered {side_path}")
