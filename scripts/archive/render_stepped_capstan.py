import os
import pyvista as pv
import numpy as np

pv.set_plot_theme("document")

OUT_DIR = "cad/manufactured"
os.makedirs(OUT_DIR, exist_ok=True)

MOTOR_STL = os.path.join(OUT_DIR, "stepped_capstan_motor_pinion.stl")
INTER_STL = os.path.join(OUT_DIR, "stepped_capstan_intermediate_pulley.stl")
WING_STL  = os.path.join(OUT_DIR, "stepped_capstan_wing_sector_drum.stl")

mesh_motor = pv.read(MOTOR_STL)
mesh_inter = pv.read(INTER_STL)
mesh_wing  = pv.read(WING_STL)

# Position meshes in operational assembly
# Stage 1: Motor at (0, 0, 0)
# Stage 2: Intermediate at (35, 0, 0)
# Stage 3: Wing at (75, 0, 0)
mesh_inter_op = mesh_inter.copy().translate([35.0, 0.0, 0.0])
mesh_wing_op  = mesh_wing.copy().translate([75.0, 0.0, 0.0])

# Add 10mm Carbon Fiber Spar cylinder to the wing sector
spar_center = [75.0, 45.0, 5.0]
spar_geom = pv.Cylinder(center=spar_center, direction=[0, 1, 0], radius=5.0, height=90.0, resolution=50)

# Colors
C_MOTOR = "#0284c7"   # Precision Electric Blue
C_INTER = "#f97316"   # Anodized Copper/Orange
C_WING  = "#10b981"   # Emerald Green
C_SPAR  = "#1f2937"   # Matte Carbon Fiber
C_CABLE = "#a855f7"   # Dyneema Purple/Violet

# -----------------------------------------------------------------------------
# 1. ISOMETRIC 3D RENDER (Operational Assembly)
# -----------------------------------------------------------------------------
p_iso = pv.Plotter(off_screen=True, window_size=[1920, 1080])
p_iso.set_background("#0f172a") # Slate dark background

p_iso.add_mesh(mesh_motor, color=C_MOTOR, specular=0.6, specular_power=25, ambient=0.2, diffuse=0.8, smooth_shading=True)
p_iso.add_mesh(mesh_inter_op, color=C_INTER, specular=0.6, specular_power=25, ambient=0.2, diffuse=0.8, smooth_shading=True)
p_iso.add_mesh(mesh_wing_op, color=C_WING, specular=0.6, specular_power=25, ambient=0.2, diffuse=0.8, smooth_shading=True)
p_iso.add_mesh(spar_geom, color=C_SPAR, specular=0.3, ambient=0.3, diffuse=0.7, smooth_shading=True)

# Add cable lines
cable1 = pv.lines_from_points(np.array([
    [0, 6, 7], [35, 24, 7], [35, -24, 7], [0, -6, 7], [0, 6, 7]
]))
cable2 = pv.lines_from_points(np.array([
    [35, 6, 12], [75, 24, 12], [75, -24, 12], [35, -6, 12], [35, 6, 12]
]))
p_iso.add_mesh(cable1, color=C_CABLE, line_width=4)
p_iso.add_mesh(cable2, color=C_CABLE, line_width=4)

# Grid floor
grid = pv.Plane(center=[45, 0, -6], direction=[0, 0, 1], i_size=160, j_size=160, i_resolution=16, j_resolution=16)
p_iso.add_mesh(grid, style='wireframe', color='#334155', line_width=1)

# Camera angle
p_iso.camera_position = [(45, -120, 85), (45, 5, 5), (0, 0, 1)]
p_iso.camera.zoom(1.15)
iso_out = os.path.join(OUT_DIR, "stepped_capstan_iso_render.png")
p_iso.screenshot(iso_out)
p_iso.close()
print(f"Rendered: {iso_out}")

# -----------------------------------------------------------------------------
# 2. TOP PLAN VIEW RENDER (Engineering Layout)
# -----------------------------------------------------------------------------
p_top = pv.Plotter(off_screen=True, window_size=[1920, 1080])
p_top.set_background("#0f172a")

p_top.add_mesh(mesh_motor, color=C_MOTOR, specular=0.5, ambient=0.2, diffuse=0.8, smooth_shading=True)
p_top.add_mesh(mesh_inter_op, color=C_INTER, specular=0.5, ambient=0.2, diffuse=0.8, smooth_shading=True)
p_top.add_mesh(mesh_wing_op, color=C_WING, specular=0.5, ambient=0.2, diffuse=0.8, smooth_shading=True)
p_top.add_mesh(spar_geom, color=C_SPAR, specular=0.3, ambient=0.3, diffuse=0.7, smooth_shading=True)
p_top.add_mesh(cable1, color=C_CABLE, line_width=4)
p_top.add_mesh(cable2, color=C_CABLE, line_width=4)
p_top.add_mesh(grid, style='wireframe', color='#334155', line_width=1)

p_top.camera_position = [(45, 10, 160), (45, 10, 5), (0, 1, 0)]
p_top.camera.zoom(1.1)
top_out = os.path.join(OUT_DIR, "stepped_capstan_top_render.png")
p_top.screenshot(top_out)
p_top.close()
print(f"Rendered: {top_out}")

# -----------------------------------------------------------------------------
# 3. EXPLODED 3D RENDER (Component Breakdown)
# -----------------------------------------------------------------------------
mesh_motor_exp = mesh_motor.copy().translate([-25.0, 0.0, -15.0])
mesh_inter_exp = mesh_inter.copy().translate([35.0, 0.0, 0.0])
mesh_wing_exp  = mesh_wing.copy().translate([100.0, 0.0, 20.0])
spar_exp       = spar_geom.copy().translate([25.0, 0.0, 20.0])

p_exp = pv.Plotter(off_screen=True, window_size=[1920, 1080])
p_exp.set_background("#0f172a")

p_exp.add_mesh(mesh_motor_exp, color=C_MOTOR, specular=0.6, specular_power=25, ambient=0.2, diffuse=0.8, smooth_shading=True)
p_exp.add_mesh(mesh_inter_exp, color=C_INTER, specular=0.6, specular_power=25, ambient=0.2, diffuse=0.8, smooth_shading=True)
p_exp.add_mesh(mesh_wing_exp, color=C_WING, specular=0.6, specular_power=25, ambient=0.2, diffuse=0.8, smooth_shading=True)
p_exp.add_mesh(spar_exp, color=C_SPAR, specular=0.3, ambient=0.3, diffuse=0.7, smooth_shading=True)

# Add dashed alignment line through axes
align_line = pv.lines_from_points(np.array([
    [-25, 0, -15], [35, 0, 0], [100, 0, 20]
]))
p_exp.add_mesh(align_line, color="#64748b", line_width=2)

p_exp.camera_position = [(50, -140, 95), (45, 0, 5), (0, 0, 1)]
p_exp.camera.zoom(1.1)
exp_out = os.path.join(OUT_DIR, "stepped_capstan_exploded_render.png")
p_exp.screenshot(exp_out)
p_exp.close()
print(f"Rendered: {exp_out}")
