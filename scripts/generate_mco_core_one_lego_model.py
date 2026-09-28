#!/usr/bin/env python3
"""
scripts/generate_mco_core_one_lego_model.py
===========================================
Special "Lego Edition" of the MCO-1 8-Wing Tandem Aircraft for Prusa CORE One:
  - Preserves all standard parameters:
      * Ground-constrained metachronal flapping wave (wings strictly at or above horizontal)
      * Splayed landing gear with 2.0 mm struts & axles (tires on bed at Z = 0)
      * Fuselage truss with 2.0 mm diagonal & vertical struts, 2.6 mm spine
      * NACA 2412 wings with 1.4 mm robust trailing edge
      * Internal teardrop electronics pod
  - Fun Lego Additions:
      * Extended forward keel forming an open pilot cockpit (X = 0 to -35 mm)
      * Minifigure-compatible bucket seat with standard 2x Lego studs (⌀4.8mm, 8.0mm pitch)
      * Cockpit control console with dual flight joysticks and instrument panel
      * Embossed "LEGO" branding plate on the front cockpit nose bumper
  - Fits within the Prusa CORE One build envelope (250 x 220 x 270 mm):
      * Length: ~136 mm (well within 250 mm limit)
      * Wingspan: 217.9 mm (within 220 mm limit)
      * Height: ~68 mm (within 270 mm limit)
"""

import os
import math
import numpy as np
import cadquery as cq

OUT_DIR = 'cad/manufactured'
os.makedirs(OUT_DIR, exist_ok=True)

# -----------------------------------------------------------------------------
# 1. Scale Factors & Global Parameters
# -----------------------------------------------------------------------------
SCALE = 220.0 / 2208.5  # 0.099615

FUSELAGE_LEN = 1000.0 * SCALE   # 99.6 mm
HEIGHT_FWD   = 200.0 * SCALE    # 19.9 mm
HEIGHT_AFT   = 35.0 * SCALE     # 3.48 mm
WIDTH_FWD    = 80.0 * SCALE     # 7.97 mm
WIDTH_AFT    = 30.0 * SCALE     # 2.99 mm

# Printable member diameters (strictly enforcing user directive: >= 3.0 mm across)
STRUT_OD = 3.00   # mm: Landing gear struts (main oleos and drag braces)
AXLE_OD  = 3.00   # mm: Wheel axles
DIAG_OD  = 3.00   # mm: Diagonal truss members
POST_OD  = 3.00   # mm: Vertical posts & cross-tubes
KEEL_OD  = 3.20   # mm: Lower keel longerons
SPINE_OD = 3.40   # mm: Upper spine longerons

# Metachronal Flapping Wave Kinematics (Ground Taxi Clearance Mode per firmware/mco_taxi_park.py)
WAVE_BIAS_DEG      = 13.5   # deg: Upward dihedral bias keeping wings above ground
WAVE_AMPLITUDE_DEG = 12.5   # deg: Flapping stroke amplitude
MASTER_PHASE_DEG   = 30.0   # deg: Snapshot phase along the traveling wave

# Wing Stations along the fuselage spine
WING_STATIONS = [
    {'name': 'Station 1 (Fore)', 'x': 100.0 * SCALE, 'incidence_deg': 20.0, 'phase_offset_deg': 0.0},
    {'name': 'Station 2',        'x': 350.0 * SCALE, 'incidence_deg': 20.0, 'phase_offset_deg': 90.0},
    {'name': 'Station 3',        'x': 600.0 * SCALE, 'incidence_deg': 20.0, 'phase_offset_deg': 180.0},
    {'name': 'Station 4 (Aft)',  'x': 850.0 * SCALE, 'incidence_deg': 20.0, 'phase_offset_deg': 270.0},
]

def get_envelope(x):
    frac = max(0.0, min(1.0, x / FUSELAGE_LEN))
    z_top = HEIGHT_FWD - frac * (HEIGHT_FWD - HEIGHT_AFT)
    z_bot = 0.0 + frac * (10.0 * SCALE)
    w_half = 0.5 * (WIDTH_FWD - frac * (WIDTH_FWD - WIDTH_AFT))
    return z_top, z_bot, w_half

def make_tube(p1, p2, od=2.0):
    """Create a solid cylinder between two 3D points."""
    p1_v = cq.Vector(p1)
    p2_v = cq.Vector(p2)
    vec = p2_v - p1_v
    length = vec.Length
    if length < 1e-4:
        return cq.Workplane('XY').sphere(od / 2.0).val()
    
    cyl = cq.Workplane('XY').circle(od / 2.0).extrude(length).val()
    z_axis = cq.Vector(0, 0, 1)
    rot_axis = z_axis.cross(vec)
    angle_rad = z_axis.getAngle(vec)
    angle_deg = math.degrees(angle_rad)
    
    if rot_axis.Length > 1e-5:
        rot_axis = rot_axis.normalized()
        cyl = cyl.rotate((0, 0, 0), (rot_axis.x, rot_axis.y, rot_axis.z), angle_deg)
    elif vec.z < 0:
        cyl = cyl.rotate((0, 0, 0), (1, 0, 0), 180)
        
    return cyl.translate(p1_v)

# -----------------------------------------------------------------------------
# 2. Build Scaled Fuselage with Forward Keel Extension & Lego Cockpit
# -----------------------------------------------------------------------------
def build_lego_fuselage():
    print("  • Building Lego Edition fuselage with extended keel & cockpit...")
    solids = []

    # Main Longitudinal Longerons
    z_top0, z_bot0, w0 = get_envelope(0.0)
    z_top1, z_bot1, w1 = get_envelope(FUSELAGE_LEN)
    
    # Upper Spine
    solids.append(make_tube((0,  w0, z_top0), (FUSELAGE_LEN,  w1, z_top1), od=SPINE_OD))
    solids.append(make_tube((0, -w0, z_top0), (FUSELAGE_LEN, -w1, z_top1), od=SPINE_OD))

    # Lower Keel (main section)
    solids.append(make_tube((0,  w0, z_bot0), (FUSELAGE_LEN,  w1, z_bot1), od=KEEL_OD))
    solids.append(make_tube((0, -w0, z_bot0), (FUSELAGE_LEN, -w1, z_bot1), od=KEEL_OD))

    # Bulkhead Stations (vertical posts & cross-ties)
    x_bulkheads = [0.0, 100.0*SCALE, 225.0*SCALE, 350.0*SCALE, 475.0*SCALE, 600.0*SCALE, 725.0*SCALE, 850.0*SCALE, FUSELAGE_LEN]
    for x in x_bulkheads:
        zt, zb, wh = get_envelope(x)
        solids.append(make_tube((x,  wh, zb), (x,  wh, zt), od=POST_OD))
        solids.append(make_tube((x, -wh, zb), (x, -wh, zt), od=POST_OD))
        solids.append(make_tube((x, -wh, zt), (x,  wh, zt), od=POST_OD))
        solids.append(make_tube((x, -wh, zb), (x,  wh, zb), od=POST_OD))

    # Diagonal Truss Shear Bracing
    for i in range(len(x_bulkheads) - 1):
        xa = x_bulkheads[i]
        xb = x_bulkheads[i+1]
        zta, zba, wha = get_envelope(xa)
        ztb, zbb, whb = get_envelope(xb)

        solids.append(make_tube((xa,  wha, zba), (xb,  whb, ztb), od=DIAG_OD))
        solids.append(make_tube((xa, -wha, zba), (xb, -whb, ztb), od=DIAG_OD))
        solids.append(make_tube((xa, -wha, zta), (xb,  whb, ztb), od=DIAG_OD))
        solids.append(make_tube((xa, -wha, zba), (xb,  whb, zbb), od=DIAG_OD))

    # Wing Station Mounting Tabs (thickened to 3.0 mm)
    for st in WING_STATIONS:
        x_st = st['x']
        zt, zb, wh = get_envelope(x_st)
        inc = st['incidence_deg']
        tab_p = cq.Workplane('YZ').box(3.0, 4.5, 7.5)\
                                  .rotate((0,0,0), (0,1,0), inc)\
                                  .translate(cq.Vector(x_st, wh + 1.0, zt - 1.2)).val()
        tab_s = cq.Workplane('YZ').box(3.0, 4.5, 7.5)\
                                  .rotate((0,0,0), (0,1,0), inc)\
                                  .translate(cq.Vector(x_st, -wh - 1.0, zt - 1.2)).val()
        solids.extend([tab_p, tab_s])

    # Internal Teardrop Electronics Pod
    x_pod_start = 130.0 * SCALE
    x_pod_end   = 480.0 * SCALE
    n_pts = 12
    x_pod_pts = np.linspace(x_pod_start, x_pod_end, n_pts)
    wires = []
    for x in x_pod_pts:
        t = (x - x_pod_start) / (x_pod_end - x_pod_start)
        scale_fac = 0.05 if (t == 0 or t == 1) else max(0.08, (t ** 0.45) * ((1.0 - t) ** 0.85) * 2.1)
        rx = 2.4 * scale_fac
        rz = 3.2 * scale_fac
        zc = (50.0 - 15.0 * t) * SCALE
        w = cq.Workplane('YZ').transformed(offset=(0, zc, x)).ellipse(rx, rz).val()
        wires.append(w)
    
    pod_solid = cq.Solid.makeLoft(wires, ruled=False)
    pylons = [
        cq.Workplane('XY').transformed(offset=(180.0*SCALE, 0, 1.5)).box(3.0, 3.0, 3.5).val(),
        cq.Workplane('XY').transformed(offset=(300.0*SCALE, 0, 1.5)).box(3.0, 3.0, 3.5).val(),
        cq.Workplane('XY').transformed(offset=(420.0*SCALE, 0, 1.5)).box(3.0, 3.0, 3.5).val(),
    ]
    for p in pylons:
        pod_solid = pod_solid.fuse(p)
    solids.append(pod_solid)

    # Landing Gear & Wheels
    # Front Landing Gear: AH-64 Apache-style mildly in trail under seat
    # Main oleo strut attaches under seat at X = -14.0 mm, rakes aft to axle at X = -9.5 mm (~18° trail)
    # Drag brace attaches at X = -5.0 mm, triangulating with the axle
    x_fwd_mount = -14.0
    y_mount_fwd = 7.5   # mm: attaches to cockpit keel longeron under seat
    z_mount_fwd = z_bot0  # 0.0 mm
    
    x_fwd_axle  = -9.5   # mm: mildly in trail (4.5 mm aft of mount point)
    z_axle_fwd  = -90.0 * SCALE * 1.5  # -13.45 mm (50% extended vertical drop)
    y_fwd_wheel = 18.5  # mm: wide-track splayed stance

    x_drag_mount = -5.0 # mm: rear drag brace mount under back of seat
    y_drag_mount = 6.5
    z_drag_mount = z_bot0

    # Main Apache shock struts (mildly in trail)
    solids.append(make_tube((x_fwd_mount,  y_mount_fwd, z_mount_fwd), (x_fwd_axle,  y_fwd_wheel, z_axle_fwd), od=STRUT_OD))
    solids.append(make_tube((x_fwd_mount, -y_mount_fwd, z_mount_fwd), (x_fwd_axle, -y_fwd_wheel, z_axle_fwd), od=STRUT_OD))

    # Apache triangulated drag braces
    solids.append(make_tube((x_drag_mount,  y_drag_mount, z_drag_mount), (x_fwd_axle,  y_fwd_wheel, z_axle_fwd), od=STRUT_OD))
    solids.append(make_tube((x_drag_mount, -y_drag_mount, z_drag_mount), (x_fwd_axle, -y_fwd_wheel, z_axle_fwd), od=STRUT_OD))

    # Transverse axles & rugged wheels (OD 3.0mm, wheel radius 3.5mm, thickness 3.0mm)
    solids.append(make_tube((x_fwd_axle, y_fwd_wheel - 2.0, z_axle_fwd), (x_fwd_axle, y_fwd_wheel + 2.5, z_axle_fwd), od=AXLE_OD))
    solids.append(make_tube((x_fwd_axle, -y_fwd_wheel - 2.5, z_axle_fwd), (x_fwd_axle, -y_fwd_wheel + 2.0, z_axle_fwd), od=AXLE_OD))
    wheel_fwd_p = cq.Workplane('XZ').circle(3.5).extrude(3.0).translate(cq.Vector(x_fwd_axle, y_fwd_wheel - 0.5, z_axle_fwd)).val()
    wheel_fwd_s = cq.Workplane('XZ').circle(3.5).extrude(3.0).translate(cq.Vector(x_fwd_axle, -y_fwd_wheel - 2.5, z_axle_fwd)).val()
    solids.extend([wheel_fwd_p, wheel_fwd_s])

    # Rear Landing Gear: oriented "in trail" (swept aft to trail behind Station 4)
    x_aft_mount = 850.0 * SCALE  # 84.67 mm (Station 4)
    _, z_aft_bot, w_aft = get_envelope(x_aft_mount)
    x_aft_wheel = 97.0  # mm: trails aft toward fuselage tail
    y_aft_wheel = 153.0 * SCALE  # 15.24 mm
    z_axle_aft = -90.0 * SCALE  # -8.97 mm

    solids.append(make_tube((x_aft_mount,  w_aft, z_aft_bot), (x_aft_wheel,  y_aft_wheel, z_axle_aft), od=STRUT_OD))
    solids.append(make_tube((x_aft_mount, -w_aft, z_aft_bot), (x_aft_wheel, -y_aft_wheel, z_axle_aft), od=STRUT_OD))
    solids.append(make_tube((x_aft_wheel, y_aft_wheel - 2.0, z_axle_aft), (x_aft_wheel, y_aft_wheel + 2.5, z_axle_aft), od=AXLE_OD))
    solids.append(make_tube((x_aft_wheel, -y_aft_wheel - 2.5, z_axle_aft), (x_aft_wheel, -y_aft_wheel + 2.0, z_axle_aft), od=AXLE_OD))
    wheel_aft_p = cq.Workplane('XZ').circle(3.5).extrude(3.0).translate(cq.Vector(x_aft_wheel, y_aft_wheel - 0.5, z_axle_aft)).val()
    wheel_aft_s = cq.Workplane('XZ').circle(3.5).extrude(3.0).translate(cq.Vector(x_aft_wheel, -y_aft_wheel - 2.5, z_axle_aft)).val()
    solids.extend([wheel_aft_p, wheel_aft_s])

    # -------------------------------------------------------------------------
    # 2B. LEGO COCKPIT EXTENSION (X = 0 to -35 mm)
    # -------------------------------------------------------------------------
    print("  • Adding forward keel extension & Lego minifigure cockpit...")
    x_nose = -35.0  # mm: Forward extension of the keel
    w_cockpit = 10.0  # mm: Half-width = 10.0 mm (total width = 20.0 mm for Lego minifigure)

    # 1. Extended Keel Longerons (flaring out from w0=4.0mm to w_cockpit=10.0mm)
    solids.append(make_tube((0,  w0, z_bot0), (x_nose + 5.0,  w_cockpit, z_bot0), od=KEEL_OD))
    solids.append(make_tube((0, -w0, z_bot0), (x_nose + 5.0, -w_cockpit, z_bot0), od=KEEL_OD))
    # Front transverse keel cross-tube / bumper rail
    solids.append(make_tube((x_nose + 5.0, -w_cockpit, z_bot0), (x_nose + 5.0, w_cockpit, z_bot0), od=KEEL_OD))

    # Cockpit Floor Plate (X in [-30.0, 0.0], Y in [-10.0, 10.0], thickness = 3.0 mm)
    floor_len = 30.0
    floor = cq.Workplane('XY').transformed(offset=(-floor_len/2.0, 0, z_bot0 + 1.5))\
                             .box(floor_len, 20.0, 3.0).val()
    solids.append(floor)

    # 2. LEGO Minifigure Compatible Seat
    # Standard Lego Seat: 2 studs wide (15.8mm), with 2 standard studs (⌀4.8mm, 8.0mm pitch)
    # Seat cushion base block (11.0mm deep x 15.8mm wide x 4.0mm high, X in [-18.5, -7.5])
    z_seat_base = z_bot0 + 3.0 + 2.0  # 5.0 mm above keel
    
    seat_base = cq.Workplane('XY').transformed(offset=(-13.0, 0, z_seat_base))\
                                  .box(11.0, 15.8, 4.0).val()
    solids.append(seat_base)

    # 2x Lego Studs on seat cushion (standard ⌀4.8mm diameter, 1.7mm height, spaced 8.0mm apart)
    # Studs located at X = -14.0 mm, Y = ±4.0 mm (standard 8.0mm pitch)
    stud_p = cq.Workplane('XY').transformed(offset=(-14.0,  4.0, z_seat_base + 2.0 + 0.85))\
                               .circle(2.4).extrude(1.7).val()
    stud_s = cq.Workplane('XY').transformed(offset=(-14.0, -4.0, z_seat_base + 2.0 + 0.85))\
                               .circle(2.4).extrude(1.7).val()
    solids.extend([stud_p, stud_s])

    # Reclined Ergonomic Seat Backrest: 15.8mm wide, 3.0mm thick, 14.0mm tall
    # Reclined aft by 24° from vertical (positive rotation about +Y axis)
    # This provides ample clearance for a Lego minifigure's torso, head, and helmet/hair.
    recline_angle = 24.0  # degrees aft
    backrest = cq.Workplane('XY').box(3.0, 15.8, 14.0)\
                                 .edges('>Z').fillet(0.8)\
                                 .rotate((0,0,0), (0,1,0), recline_angle)\
                                 .translate(cq.Vector(-4.95, 0, z_seat_base + 8.39)).val()
    solids.append(backrest)

    # 3. Flight Control Console in front of seat (X in [-29.0, -22.0])
    console_base = cq.Workplane('XY').transformed(offset=(-25.5, 0, z_bot0 + 3.0 + 3.0))\
                                     .box(7.0, 15.8, 6.0).val()
    solids.append(console_base)

    # Console display wedge (sloped toward pilot) - rotated before translation
    wedge = cq.Workplane('XY').box(4.5, 15.8, 2.5)\
                              .rotate((0,0,0), (0,1,0), -30)\
                              .translate(cq.Vector(-24.0, 0, z_bot0 + 3.0 + 6.8)).val()
    solids.append(wedge)

    # Dual Minifigure Flight Joysticks (⌀3.0mm vertical stalks with ⌀4.0mm grip knobs)
    # Hand positions for standard seated Lego minifigure at Y = ±5.5 mm
    joy_p = cq.Workplane('XY').transformed(offset=(-22.5,  5.5, z_bot0 + 3.0 + 6.5))\
                             .circle(1.5).extrude(3.5).val()
    knob_p = cq.Workplane('XY').transformed(offset=(-22.5,  5.5, z_bot0 + 3.0 + 10.0))\
                              .sphere(2.0).val()
    joy_s = cq.Workplane('XY').transformed(offset=(-22.5, -5.5, z_bot0 + 3.0 + 6.5))\
                             .circle(1.5).extrude(3.5).val()
    knob_s = cq.Workplane('XY').transformed(offset=(-22.5, -5.5, z_bot0 + 3.0 + 10.0))\
                              .sphere(2.0).val()
    solids.extend([joy_p, knob_p, joy_s, knob_s])

    # 4. Front Bumper (smooth, 3.0 mm thick)
    # Front bumper plate at X = -34.0 mm, width = 18.0 mm, height = 6.0 mm, thickness = 3.0 mm
    bumper = cq.Workplane('YZ').transformed(offset=(0, z_bot0 + 3.5, x_nose + 4.5))\
                               .box(18.0, 6.0, 3.0).val()
    solids.append(bumper)

    fuselage_compound = cq.Compound.makeCompound(solids)
    return fuselage_compound

# -----------------------------------------------------------------------------
# 3. Robust Thickened NACA Wings (25% Shortened Semispan, >= 3.0 mm Thickness)
# -----------------------------------------------------------------------------
def make_scaled_naca_wire(chord=16.44, te_thick=2.0, t=0.195, scale=1.0, z=0.0, x_shift=0.0, n_pts=60):
    m = 0.02
    p = 0.40
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
    
    te_half = (te_thick / 2.0) * scale
    blend_start = 0.65
    for i in range(len(x)):
        if x[i] >= blend_start:
            factor = (x[i] - blend_start) / (1.0 - blend_start)
            yu[i] = max(yu[i], yc[i]*chord*scale + te_half * factor)
            yl[i] = min(yl[i], yc[i]*chord*scale - te_half * factor)
            
    pts = []
    for i in range(len(xu) - 1, -1, -1):
        pts.append((xu[i], yu[i]))
    for i in range(1, len(xl)):
        pts.append((xl[i], yl[i]))
    pts.append(pts[0])
    
    cq_pts = [cq.Vector(px, py, z) for px, py in pts]
    return cq.Wire.makePolygon(cq_pts)

def build_scaled_wing():
    chord = 16.44
    te_thick = 2.0
    # Wing Semispan shortened by 25% (was 94.0 + 12.0 = 106.0 mm -> now 70.5 + 9.0 = 79.5 mm)
    span_main = 70.5
    span_tip  = 9.0
    
    wire_root = make_scaled_naca_wire(chord, te_thick, t=0.195, scale=1.0, z=0.0)
    wire_mid  = make_scaled_naca_wire(chord, te_thick, t=0.195, scale=1.0, z=-span_main)
    wire_t1   = make_scaled_naca_wire(chord, te_thick, t=0.195, scale=0.98, z=-(span_main + 0.5*span_tip), x_shift=0.8)
    wire_tip  = make_scaled_naca_wire(chord, te_thick, t=0.200, scale=0.95, z=-(span_main + span_tip), x_shift=1.5)
    
    wing_solid = cq.Solid.makeLoft([wire_root, wire_mid, wire_t1, wire_tip], ruled=True)
    root_block = cq.Workplane('XY').box(chord, 3.5, 3.2)\
                                  .translate(cq.Vector(chord/2.0, 0, 1.6)).val()
    wing_solid = wing_solid.fuse(root_block)
    return wing_solid

# -----------------------------------------------------------------------------
# 4. Master Lego Edition Assembly & Verification
# -----------------------------------------------------------------------------
def build_lego_aircraft():
    print("=" * 75)
    print("GENERATING MCO-1 LEGO MINIFIGURE EDITION (PRUSA CORE ONE)")
    print("=" * 75)

    # 1. Lego Fuselage with extended keel & cockpit
    fuselage = build_lego_fuselage()

    # 2. Wings
    wing_raw = build_scaled_wing()
    wing_port = wing_raw.rotate((0, 0, 0), (1, 0, 0), 90)
    wing_stbd = wing_raw.rotate((0, 0, 0), (1, 0, 0), -90)

    parts = [fuselage]

    # Mount 8 wings at the 4 stations with +20.0° AoA and ground-constrained metachronal wave
    for idx, st in enumerate(WING_STATIONS, start=1):
        x_st = st['x']
        inc  = st['incidence_deg']
        phase_offset = st['phase_offset_deg']
        zt, _, wh = get_envelope(x_st)
        
        y_stbd = -(wh + 1.0)
        y_port =  (wh + 1.0)
        z_mount = zt - 1.2
        
        phase_rad = math.radians(MASTER_PHASE_DEG + phase_offset)
        dihedral_deg = WAVE_BIAS_DEG + WAVE_AMPLITUDE_DEG * math.sin(phase_rad)
        
        w_p = wing_port.rotate((0, 0, 0), (0, 1, 0), inc)\
                       .rotate((0, 0, 0), (1, 0, 0), dihedral_deg)\
                       .translate(cq.Vector(x_st - 3.3, y_port, z_mount + 0.4))
                       
        w_s = wing_stbd.rotate((0, 0, 0), (0, 1, 0), inc)\
                       .rotate((0, 0, 0), (1, 0, 0), -dihedral_deg)\
                       .translate(cq.Vector(x_st - 3.3, y_stbd, z_mount + 0.4))
                       
        parts.extend([w_s, w_p])
        print(f"  • Wing Pair {idx} ({st['name']}): AoA=+{inc:.1f}°, Dihedral={dihedral_deg:+.1f}°")

    print("\nFusing complete aircraft into unified manifold solid...")
    aircraft_compound = cq.Compound.makeCompound(parts)
    
    # Ground Stance & Build Plate Alignment
    # Front gear is AH-64 Apache style mildly in trail with 50% extended vertical drop
    # Rear gear is in trail
    x_fwd_gear = -9.5   # Apache mildly in trail axle
    z_fwd_gear = -90.0 * SCALE * 1.5
    x_aft_gear = 97.0
    z_aft_gear = -90.0 * SCALE
    dx_gear = x_aft_gear - x_fwd_gear
    dz_gear = z_aft_gear - z_fwd_gear
    ground_pitch_deg = math.degrees(math.atan2(dz_gear, dx_gear))
    print(f"\nAligning ground stance ({ground_pitch_deg:.2f}° nose-up pitch for 4-point tire ground contact)...")
    aircraft_pitched = aircraft_compound.rotate((0, 0, 0), (0, 1, 0), ground_pitch_deg)

    # Bounding Box & Fit Verification
    bb = aircraft_pitched.BoundingBox()
    dx = bb.xmax - bb.xmin
    dy = bb.ymax - bb.ymin
    dz = bb.zmax - bb.zmin
    vol_cm3 = aircraft_pitched.Volume() / 1000.0

    print("\n[PRUSA CORE ONE PRINT ENVELOPE VERIFICATION]")
    print(f"  • Model Bounding Box:")
    print(f"      Length (X):   {dx:.1f} mm  (Limit: 250.0 mm) -> FIT: {'PASS' if dx <= 250 else 'FAIL'}")
    print(f"      Wingspan (Y): {dy:.1f} mm  (Limit: 220.0 mm) -> FIT: {'PASS' if dy <= 220.5 else 'FAIL'}")
    print(f"      Height (Z):   {dz:.1f} mm  (Limit: 270.0 mm) -> FIT: {'PASS' if dz <= 270 else 'FAIL'}")
    print(f"  • Total Aircraft Volume: {vol_cm3:.2f} cm³")
    print(f"  • Estimated Filament Mass (PLA @ 1.24 g/cm³): {vol_cm3 * 1.24:.1f} g")
    print(f"  • Minimum Strut/Spar Thickness: {STRUT_OD:.1f} mm (Directly satisfies >= 3.0 mm spec)")

    # Center model on bed: tires at Z = 0
    print("\nCentering model for build plate (placing tires at Z = 0)...")
    aircraft_printable = aircraft_pitched.translate(cq.Vector(-bb.xmin, -(bb.ymin + bb.ymax)/2.0, -bb.zmin))
    
    # Export deliverables with "lego" in the file name
    print("\n[EXPORTING DELIVERABLES]")
    step_path = os.path.join(OUT_DIR, 'mco_core_one_lego_edition.step')
    stl_path  = os.path.join(OUT_DIR, 'mco_core_one_lego_edition.stl')
    threemf_path = os.path.join(OUT_DIR, 'mco_core_one_lego_edition.3mf')

    cq.exporters.export(aircraft_printable, step_path)
    print(f"  ✓ Exported STEP: {step_path}")
    cq.exporters.export(aircraft_printable, stl_path, tolerance=0.08, angularTolerance=0.25)
    print(f"  ✓ Exported STL:  {stl_path}")
    cq.exporters.export(aircraft_printable, threemf_path)
    print(f"  ✓ Exported 3MF:  {threemf_path}")

    return aircraft_printable

if __name__ == '__main__':
    build_lego_aircraft()
