#!/usr/bin/env python3
"""
scripts/generate_generic_airfoil_wing.py
========================================
Generates robust, solid generic aerodynamic airfoil wings for the MCO-1 8-Wing Tandem Aircraft.
Specifically engineered for 3D printing the entire aircraft build:
  1. Solid generic cambered airfoil loft (NACA 2412 profile, 165mm root chord) without delicate,
     spindly open-bay ribs that trap support material and tear during post-processing.
  2. Thickened trailing edge (vertical thickness >= 3.0 mm) preventing curl, notch failures,
     and support tear-out.
  3. Continuous internal spar bores:
     - Main Spar (#2, 20% chord): ⌀10.40 mm bore (radius 5.20 mm) for ⌀10.0 mm carbon fiber tube
     - Forward Alignment Spar (#1, 10% chord): ⌀3.20 mm bore (radius 1.60 mm) for ⌀3.0 mm carbon rod
     - Aft Alignment Spar (#3, 55% chord): ⌀3.20 mm bore (radius 1.60 mm) for ⌀3.0 mm carbon rod
     (Thinnest spar in the build is 3.0 mm, fully meeting the >=3mm specification).
  4. Smooth aerodynamic rounded wingtip (Z = -1000 to -1130 mm).
  5. Inboard 3-DOF root mounting bracket (Z = 0 to 20 mm) with ⌀6.0mm titanium pivot bore and
     M3 collar flange bolt pattern, mating directly with the fuselage split-clamp collars.
"""

import os
import sys
import math
import numpy as np
import cadquery as cq

OUT_DIR = 'cad/manufactured'
os.makedirs(OUT_DIR, exist_ok=True)

CHORD_ROOT = 165.0      # mm: Root chord
TE_THICKNESS = 3.20     # mm: Minimum trailing edge thickness (>= 3.0 mm)
SPAN_MAIN = 1000.0      # mm: Main constant-chord lifting span (Z = 0 to -1000)
SPAN_TIP = 130.0        # mm: Aerodynamic rounded tip extension (Z = -1000 to -1130)
MOUNT_DEPTH = 20.0      # mm: Inboard mounting bracket (Z = 0 to +20)

def make_naca_wire(chord=CHORD_ROOT, te_thick=TE_THICKNESS, scale=1.0, z=0.0, x_shift=0.0, n_pts=80):
    """Generate a clean NACA 2412 airfoil wire with a calibrated minimum trailing edge thickness."""
    m = 0.02  # Maximum camber
    p = 0.40  # Location of maximum camber
    t = 0.12  # Thickness ratio (12%)
    
    x = np.linspace(0, 1, n_pts)
    yt = 5 * t * (0.2969 * np.sqrt(x) - 0.1260 * x - 0.3516 * (x**2) + 0.2843 * (x**3) - 0.1015 * (x**4))
    
    yc = np.zeros_like(x)
    dyc_dx = np.zeros_like(x)
    for i in range(len(x)):
        if x[i] < p:
            yc[i] = (m / (p**2)) * (2 * p * x[i] - x[i]**2) if p > 0 else 0
            dyc_dx[i] = (2 * m / (p**2)) * (p - x[i]) if p > 0 else 0
        else:
            yc[i] = (m / ((1 - p)**2)) * ((1 - 2 * p) + 2 * p * x[i] - x[i]**2) if p < 1 else 0
            dyc_dx[i] = (2 * m / ((1 - p)**2)) * (p - x[i]) if p < 1 else 0
            
    theta = np.arctan(dyc_dx)
    xu = (x - yt * np.sin(theta)) * chord * scale + x_shift
    yu = (yc + yt * np.cos(theta)) * chord * scale
    xl = (x + yt * np.sin(theta)) * chord * scale + x_shift
    yl = (yc - yt * np.cos(theta)) * chord * scale
    
    # Smooth trailing edge thickening over aft 25% of chord
    te_w = (te_thick * scale) / 2.0
    x_blend = chord * scale * 0.75 + x_shift
    for i in range(len(x)):
        if xu[i] > x_blend:
            f = (xu[i] - x_blend) / ((chord * scale + x_shift) - x_blend)
            yu[i] += te_w * f
        if xl[i] > x_blend:
            f = (xl[i] - x_blend) / ((chord * scale + x_shift) - x_blend)
            yl[i] -= te_w * f
            
    pts_upper = list(zip(xu[::-1], yu[::-1]))
    pts_lower = list(zip(xl[1:], yl[1:]))
    all_pts = pts_upper + pts_lower
    return cq.Wire.makePolygon([cq.Vector(pt[0], pt[1], z) for pt in all_pts] + [cq.Vector(all_pts[0][0], all_pts[0][1], z)])

def generate_generic_wings():
    print("=" * 75)
    print("GENERATING ROBUST GENERIC AIRFOIL WINGS (STARBOARD & PORT)")
    print("=" * 75)

    # 1. Main Lifting Wing Loft (Z = 0 to -1000 mm)
    print("\n[1/3] Lofting Solid Generic Wing Body (NACA 2412, 165mm Chord)...")
    w_root = make_naca_wire(z=0.0)
    w_mid  = make_naca_wire(z=-SPAN_MAIN)
    main_body = cq.Solid.makeLoft([w_root, w_mid], ruled=True)

    # 2. Aerodynamic Rounded Wingtip (Z = -1000 to -1130 mm)
    tip_wires = [w_mid]
    for frac, scale, x_sh in [
        (0.35, 0.85, 4.0),
        (0.70, 0.55, 10.0),
        (0.92, 0.25, 14.0),
        (1.00, 0.08, 16.0),
    ]:
        z_curr = -SPAN_MAIN - frac * SPAN_TIP
        tip_wires.append(make_naca_wire(scale=scale, z=z_curr, x_shift=x_sh))

    tip_solid = cq.Solid.makeLoft(tip_wires, ruled=False)
    wing_outer = main_body.fuse(tip_solid)

    # 3. Inboard Root Mounting Bracket (Z = 0 to +20 mm)
    w_mount = make_naca_wire(z=MOUNT_DEPTH)
    mount_solid = cq.Solid.makeLoft([w_root, w_mount], ruled=True)
    wing_solid = mount_solid.fuse(wing_outer)

    # 4. Spar Bores & Pivot Holes:
    # All spars >= 3.0 mm per user instruction:
    #   - Mid Spar (Main): ⌀10.40 mm bore (radius 5.20 mm) at X = 33.0 mm (20% chord)
    #   - Fwd Spar (Alignment): ⌀3.20 mm bore (radius 1.60 mm) at X = 16.5 mm (10% chord)
    #   - Aft Spar (Alignment): ⌀3.20 mm bore (radius 1.60 mm) at X = 90.75 mm (55% chord)
    print("\n[2/3] Boring Continuous Carbon Fiber Spar Bores (⌀10.4mm & ⌀3.2mm)...")
    wp = cq.Workplane().add(wing_solid)
    wp = wp.cut(cq.Workplane('XY').transformed(offset=(33.0, 7.5, 25.0)).circle(5.20).extrude(-1050.0))
    wp = wp.cut(cq.Workplane('XY').transformed(offset=(16.5, 5.0, 25.0)).circle(1.60).extrude(-1050.0))
    wp = wp.cut(cq.Workplane('XY').transformed(offset=(90.75, 5.5, 25.0)).circle(1.60).extrude(-1050.0))

    # Inboard Collar Pivot Bore (⌀6.0 mm titanium pivot pin, center at X = 33.0, Y = 7.5, Z = 10.0):
    wp = wp.cut(cq.Workplane('YZ').transformed(offset=(7.5, 10.0, 33.0)).circle(3.0).extrude(30.0))

    # Collar Flange M3 mounting holes at root (4x M3 clearance holes ⌀3.2 mm):
    wp = wp.cut(cq.Workplane('YZ').transformed(offset=(7.5 - 25.0, 10.0, 33.0 - 15.0)).circle(1.60).extrude(30.0))
    wp = wp.cut(cq.Workplane('YZ').transformed(offset=(7.5 - 25.0, 10.0, 33.0 + 15.0)).circle(1.60).extrude(30.0))
    wp = wp.cut(cq.Workplane('YZ').transformed(offset=(7.5 + 25.0, 10.0, 33.0 - 15.0)).circle(1.60).extrude(30.0))
    wp = wp.cut(cq.Workplane('YZ').transformed(offset=(7.5 + 25.0, 10.0, 33.0 + 15.0)).circle(1.60).extrude(30.0))

    wing_stbd = wp.val()
    if not wing_stbd.isValid():
        raise RuntimeError("Starboard generic airfoil wing solid is invalid!")

    # Port wing is mirrored across the root chord plane (XY) or native symmetry:
    # In aircraft coordinates, mirroring across XZ creates the left/port wing:
    wing_port = cq.Workplane().add(wing_stbd).mirror('XY').val()
    
    vol_stbd = wing_stbd.Volume() / 1000.0
    bb = wing_stbd.BoundingBox()
    print(f"  • Starboard Wing Volume: {vol_stbd:.1f} cm³")
    print(f"  • Bounding Box: X=[{bb.xmin:.1f}, {bb.xmax:.1f}], Y=[{bb.ymin:.1f}, {bb.ymax:.1f}], Z=[{bb.zmin:.1f}, {bb.zmax:.1f}] mm")
    print(f"  • Total Span: {bb.zlen:.1f} mm ({bb.zlen/1000.0:.2f} m)")
    print(f"  • Trailing Edge Thickness: {TE_THICKNESS:.2f} mm (>= 3.0 mm)")

    # Export Starboard & Port CAD Deliverables
    print("\n[3/3] Exporting CAD Deliverables (STEP, STL, 3MF)...")
    stbd_step = os.path.join(OUT_DIR, 'generic_airfoil_wing_starboard.step')
    stbd_stl  = os.path.join(OUT_DIR, 'generic_airfoil_wing_starboard.stl')
    stbd_3mf  = os.path.join(OUT_DIR, 'generic_airfoil_wing_starboard.3mf')

    port_step = os.path.join(OUT_DIR, 'generic_airfoil_wing_port.step')
    port_stl  = os.path.join(OUT_DIR, 'generic_airfoil_wing_port.stl')
    port_3mf  = os.path.join(OUT_DIR, 'generic_airfoil_wing_port.3mf')

    wp_stbd = cq.Workplane().add(wing_stbd)
    wp_port = cq.Workplane().add(wing_port)

    cq.exporters.export(wp_stbd, stbd_stl, tolerance=0.03, angularTolerance=0.2)
    cq.exporters.export(wp_stbd, stbd_step)
    cq.exporters.export(wp_stbd, stbd_3mf, tolerance=0.03, angularTolerance=0.2)

    cq.exporters.export(wp_port, port_stl, tolerance=0.03, angularTolerance=0.2)
    cq.exporters.export(wp_port, port_step)
    cq.exporters.export(wp_port, port_3mf, tolerance=0.03, angularTolerance=0.2)

    print(f"  ✓ Exported Starboard STEP: {stbd_step}")
    print(f"  ✓ Exported Starboard STL:  {stbd_stl}")
    print(f"  ✓ Exported Starboard 3MF:  {stbd_3mf}")
    print(f"  ✓ Exported Port STEP:      {port_step}")
    print(f"  ✓ Exported Port STL:       {port_stl}")
    print(f"  ✓ Exported Port 3MF:       {port_3mf}")

    return wing_stbd, wing_port

if __name__ == '__main__':
    generate_generic_wings()
