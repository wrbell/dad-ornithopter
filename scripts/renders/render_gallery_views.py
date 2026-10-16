import os
import pyvista as pv
import numpy as np

pv.set_plot_theme("document")

OUT_DIR = "cad/manufactured/renders"
os.makedirs(OUT_DIR, exist_ok=True)

# Meshes
stl_drum = "cad/manufactured/mco_ultracompact_sector_wheel_10to1.stl"
stl_wrist = "cad/manufactured/mco_ultracompact_2dof_wrist.stl"
stl_fairing = "cad/manufactured/mco_ultracompact_naca0018_fairing_enclosing.stl"
stl_bearing = "cad/manufactured/bearing_6708_annular.stl"

mesh_drum = pv.read(stl_drum)
mesh_wrist = pv.read(stl_wrist)
mesh_fairing = pv.read(stl_fairing)
mesh_bearing_base = pv.read(stl_bearing)

# Annular Bearings (6708-2RS: 40mm ID, 50mm OD, 6mm H)
bearing_bot = mesh_bearing_base.copy().translate([0.0, 0.0, 0.0])
bearing_top = mesh_bearing_base.copy().translate([0.0, 0.0, 26.0])

# Servos (AGFRC B24CLM at X=±28.5mm, Y=-18.0mm, Z=14.0mm)
servo_l = pv.Cube(center=[-28.5, -18.0, 14.0], x_length=12.0, y_length=23.0, z_length=27.5)
servo_r = pv.Cube(center=[28.5, -18.0, 14.0], x_length=12.0, y_length=23.0, z_length=27.5)

# Main 10mm Carbon Fiber Spar Tube (pivot at Z=43.5mm, extending +Y outboard)
spar_tube = pv.Cylinder(center=[0.0, 38.0, 43.5], direction=[0.0, 1.0, 0.0], radius=5.0, height=75.0, resolution=50)

# 0.8mm Dyneema SK99 Cable tracking in capstan groove (pitch R=45mm, Z=5.0mm)
thetas = np.linspace(np.radians(-78), np.radians(78), 80)
cable_pts = np.column_stack([45.0 * np.sin(thetas), 45.0 * np.cos(thetas), np.full_like(thetas, 5.0)])
cable_lines = pv.lines_from_points(cable_pts)

# Colors
C_DRUM = "#0284c7"     # Precision Sky Blue (Skeletonized Drum)
C_WRIST = "#f59e0b"    # Gold / Amber (2DOF Wrist Yoke)
C_SERVO = "#ef4444"    # Precision Red (Aft Servos)
C_BEARING = "#cbd5e1"  # Polished Steel Bearings
C_SPAR = "#1e293b"     # Matte Carbon Fiber
C_CABLE = "#a855f7"    # Purple Dyneema Cable
C_FAIRING = "#f8fafc"  # Pearl White Fairing
BG_COLOR = "#0b1120"   # Dark Slate Studio Background

def render_image(show_fairing=False, fairing_opacity=1.0, view="iso", zoom=1.25, fname="render.png"):
    p = pv.Plotter(off_screen=True, window_size=[1920, 1080])
    p.set_background(BG_COLOR)
    
    # Core Mechanics
    p.add_mesh(mesh_drum, color=C_DRUM, specular=0.5, ambient=0.25, diffuse=0.75, smooth_shading=True)
    p.add_mesh(mesh_wrist, color=C_WRIST, specular=0.6, ambient=0.30, diffuse=0.70, smooth_shading=True)
    p.add_mesh(servo_l, color=C_SERVO, specular=0.4, ambient=0.25)
    p.add_mesh(servo_r, color=C_SERVO, specular=0.4, ambient=0.25)
    p.add_mesh(bearing_bot, color=C_BEARING, specular=0.8, ambient=0.2)
    p.add_mesh(bearing_top, color=C_BEARING, specular=0.8, ambient=0.2)
    p.add_mesh(spar_tube, color=C_SPAR, specular=0.3, ambient=0.3)
    p.add_mesh(cable_lines, color=C_CABLE, line_width=4)
    
    if show_fairing:
        p.add_mesh(mesh_fairing, color=C_FAIRING, opacity=fairing_opacity, specular=0.4, ambient=0.25, diffuse=0.75, smooth_shading=True)
        
    if view == "iso":
        p.view_isometric()
    elif view == "iso_rear":
        p.camera_position = [(-110, -130, 95), (0, -5, 25), (0, 0, 1)]
    elif view == "top":
        p.view_xy()
    elif view == "front":
        p.view_xz()
    elif view == "side":
        p.view_yz()
        
    p.camera.zoom(zoom)
    out_file = os.path.join(OUT_DIR, fname)
    p.screenshot(out_file)
    p.close()
    print(f"Generated {out_file}")

# 1. Isometric
render_image(show_fairing=False, view="iso", fname="01_isometric_without_fairing.png")
render_image(show_fairing=True, fairing_opacity=0.35, view="iso", fname="02_isometric_with_fairing_translucent.png")
render_image(show_fairing=True, fairing_opacity=1.0, view="iso", fname="03_isometric_with_fairing_solid.png")

# 2. Top-Down Planform
render_image(show_fairing=False, view="top", fname="04_top_without_fairing.png")
render_image(show_fairing=True, fairing_opacity=0.40, view="top", fname="05_top_with_fairing_translucent.png")
render_image(show_fairing=True, fairing_opacity=1.0, view="top", fname="06_top_with_fairing_solid.png")

# 3. Side Profile (NACA 0018 Airfoil Cross-Section)
render_image(show_fairing=False, view="side", fname="07_side_without_fairing.png")
render_image(show_fairing=True, fairing_opacity=0.45, view="side", fname="08_side_with_fairing_translucent.png")
render_image(show_fairing=True, fairing_opacity=1.0, view="side", fname="09_side_with_fairing_solid.png")

# 4. Front Elevation
render_image(show_fairing=False, view="front", fname="10_front_without_fairing.png")
render_image(show_fairing=True, fairing_opacity=0.40, view="front", fname="11_front_with_fairing.png")

# 5. Rear 3/4 Perspective (Aft Servos)
render_image(show_fairing=False, view="iso_rear", fname="12_rear_quarter_without_fairing.png")
render_image(show_fairing=True, fairing_opacity=0.35, view="iso_rear", fname="13_rear_quarter_with_fairing.png")
