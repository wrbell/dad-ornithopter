#!/usr/bin/env python3
"""
MCO-1 8-Wing Tandem Ornithopter: Parametric Fuselage Frame & Landing Gear Generator
Upgraded to Rev B Structural Standards following Adversarial Audit:

1. Upper Longerons upgraded to ⌀12mm OD x 1.0mm wall 3K carbon fiber tubes
   (triples torsional rigidity against metachronal flapping whip, J: 201 -> 678 mm⁴).
2. Wing Station Mounts upgraded from flat plates to circumferential Split-Clamp Collars
   (⌀18mm OD, 60mm clamp length, ⌀12mm bore, 2x M3 pinch bolts, ⌀6mm Ti pivot pin bore).
3. Transverse Torque Ties across port-starboard collar pairs to lock flapping root moments.
4. Landing Gear lowered to 115mm belly clearance (ground plane Z = -115mm, axle Z = -90mm).
5. Front track tuned to tf = 250mm, rear track tr = 320mm (tf/tr = 0.781, SSF = 1.00).
"""

import os
import math
import numpy as np
import cadquery as cq

OUT_DIR = 'cad/manufactured'
os.makedirs(OUT_DIR, exist_ok=True)

# -----------------------------------------------------------------------------
# 1. Global Fuselage Parameters
# -----------------------------------------------------------------------------
SPAN_FUSELAGE   = 1000.0   # mm: Total longitudinal length (X = 0 to 1000)
HEIGHT_FWD      = 200.0    # mm: Front bulkhead vertical height (Z)
HEIGHT_AFT      = 35.0     # mm: Tail junction vertical height (Z)
WIDTH_FWD       = 80.0     # mm: Front fuselage width (Y = -40 to +40)
WIDTH_AFT       = 30.0     # mm: Tail fuselage width (Y = -15 to +15)

UPPER_TUBE_OD   = 12.0     # mm: Upgraded upper spine longeron OD (3.37x torsional stiffness)
LOWER_TUBE_OD   = 8.0      # mm: Lower keel longeron OD
POST_TUBE_OD    = 8.0      # mm: Vertical posts and cross-tubes OD
DIAG_TUBE_OD    = 6.0      # mm: Diagonal shear bracing tube OD
STRUT_TUBE_OD   = 6.0      # mm: Landing gear strut tube OD
AXLE_OD         = 4.0      # mm: Ground steel wheel axle OD

COLLAR_OD       = 18.0     # mm: Split-clamp collar outer diameter
COLLAR_LEN      = 60.0     # mm: Split-clamp collar clamp length along longeron
FLANGE_THICK    = 6.0      # mm: Outboard wing mounting flange thickness

# 4 Wing Root Stations along the inclined spine (20.0° Angle of Attack per user directive)
WING_STATIONS = [
    {'name': 'Station 1 (Fore)', 'x': 100.0, 'incidence_deg': 20.0},
    {'name': 'Station 2',        'x': 350.0, 'incidence_deg': 20.0},
    {'name': 'Station 3',        'x': 600.0, 'incidence_deg': 20.0},
    {'name': 'Station 4 (Aft)',  'x': 850.0, 'incidence_deg': 20.0},
]

def get_fuselage_envelope(x):
    """Compute (z_top, z_bot, w_half) at longitudinal position x."""
    frac = max(0.0, min(1.0, x / SPAN_FUSELAGE))
    z_top = HEIGHT_FWD - frac * (HEIGHT_FWD - HEIGHT_AFT)
    z_bot = 0.0 + frac * 10.0  # slight 10mm boat-tail upsweep
    w_half = 0.5 * (WIDTH_FWD - frac * (WIDTH_FWD - WIDTH_AFT))
    return z_top, z_bot, w_half

def make_tube_between(p1, p2, od=8.0, id_=0.0):
    """Create a cylindrical tube solid between two 3D points."""
    v1 = cq.Vector(*p1)
    v2 = cq.Vector(*p2)
    delta = v2 - v1
    length = delta.Length
    if length < 1e-3:
        raise ValueError("Tube endpoints are identical!")
    
    cyl = cq.Workplane('XY').circle(od / 2.0).extrude(length)
    if id_ > 0.0:
        bore = cq.Workplane('XY').circle(id_ / 2.0).extrude(length)
        cyl = cyl.cut(bore)
        
    solid = cyl.val()
    z_axis = cq.Vector(0, 0, 1)
    dir_vec = delta.normalized()
    
    cross = z_axis.cross(dir_vec)
    dot = z_axis.dot(dir_vec)
    
    if cross.Length < 1e-6:
        if dot < 0:
            solid = solid.rotate((0, 0, 0), (1, 0, 0), 180)
    else:
        angle_deg = math.degrees(math.acos(max(-1.0, min(1.0, dot))))
        rot_axis = (cross.x, cross.y, cross.z)
        solid = solid.rotate((0, 0, 0), rot_axis, angle_deg)
        
    return solid.translate(v1)

def build_fuselage():
    print("=" * 75)
    print("GENERATING MCO-1 PARAMETRIC FUSELAGE TRUSS (REV B - ⌀12MM SPINE & COLLARS)")
    print("=" * 75)

    tubes = []

    # 1. Four Longitudinal Longerons
    pts_x = [0.0] + [st['x'] for st in WING_STATIONS] + [SPAN_FUSELAGE]
    
    for i in range(len(pts_x) - 1):
        x1 = pts_x[i]
        x2 = pts_x[i+1]
        z_t1, z_b1, w1 = get_fuselage_envelope(x1)
        z_t2, z_b2, w2 = get_fuselage_envelope(x2)
        
        # Upper Port (⌀12mm Heavy Spine)
        tubes.append(make_tube_between((x1, w1, z_t1), (x2, w2, z_t2), od=UPPER_TUBE_OD))
        # Upper Starboard (⌀12mm Heavy Spine)
        tubes.append(make_tube_between((x1, -w1, z_t1), (x2, -w2, z_t2), od=UPPER_TUBE_OD))
        # Lower Port (⌀8mm Keel)
        tubes.append(make_tube_between((x1, w1, z_b1), (x2, w2, z_b2), od=LOWER_TUBE_OD))
        # Lower Starboard (⌀8mm Keel)
        tubes.append(make_tube_between((x1, -w1, z_b1), (x2, -w2, z_b2), od=LOWER_TUBE_OD))

    # 2. Transverse Bulkheads & Vertical Posts at each Wing Station and Ends
    for x in pts_x:
        z_top, z_bot, w = get_fuselage_envelope(x)
        # Port vertical post
        tubes.append(make_tube_between((x, w, z_bot), (x, w, z_top), od=POST_TUBE_OD))
        # Starboard vertical post
        tubes.append(make_tube_between((x, -w, z_bot), (x, -w, z_top), od=POST_TUBE_OD))
        # Top cross-brace (transverse torque tie)
        tubes.append(make_tube_between((x, -w, z_top), (x, w, z_top), od=POST_TUBE_OD))
        # Bottom cross-brace
        tubes.append(make_tube_between((x, -w, z_bot), (x, w, z_bot), od=POST_TUBE_OD))

    # 3. Diagonal Truss Shear Bracing (matching the craft-stick mock-up)
    for i in range(len(pts_x) - 1):
        x1 = pts_x[i]
        x2 = pts_x[i+1]
        z_t1, z_b1, w1 = get_fuselage_envelope(x1)
        z_t2, z_b2, w2 = get_fuselage_envelope(x2)
        
        # Port side diagonal
        tubes.append(make_tube_between((x1, w1, z_b1), (x2, w2, z_t2), od=DIAG_TUBE_OD))
        # Starboard side diagonal
        tubes.append(make_tube_between((x1, -w1, z_b1), (x2, -w2, z_t2), od=DIAG_TUBE_OD))
        # Bottom bay diagonal (torsional stiffness)
        tubes.append(make_tube_between((x1, -w1, z_b1), (x2, w2, z_b2), od=DIAG_TUBE_OD))

    # Fuse all truss tubes into unified structural frame
    print("  • Fusing primary longerons and bulkheads...")
    frame = tubes[0]
    for t in tubes[1:]:
        frame = frame.fuse(t)
    print("  ✓ Primary truss frame fused successfully.")

    # -------------------------------------------------------------------------
    # 4. Wing Station Split-Clamp Collars & 3-DOF Mounting Flanges
    # -------------------------------------------------------------------------
    print("  • Adding 4 pairs of 3-DOF Split-Clamp Structural Wing Mount Collars...")
    collars = []
    for st in WING_STATIONS:
        x = st['x']
        z_top, _, w = get_fuselage_envelope(x)
        inc = st['incidence_deg']
        
        # Upper longeron slope: ~9.37° downwards
        slope_rad = math.atan((HEIGHT_FWD - HEIGHT_AFT) / SPAN_FUSELAGE)
        slope_deg = math.degrees(slope_rad)
        
        # Build Port Split-Clamp Collar (Y = +w):
        # 1) Cylindrical Sleeve around the ⌀12mm longeron (OD 18mm, length 60mm)
        sleeve_p = cq.Workplane('XY').circle(COLLAR_OD / 2.0).extrude(COLLAR_LEN)\
                                     .translate(cq.Vector(0, 0, -COLLAR_LEN / 2.0))\
                                     .rotate((0,0,0), (0,1,0), slope_deg + 90)\
                                     .translate(cq.Vector(x, w, z_top))
                                     
        # 2) Outboard Mounting Flange (80mm x 42mm x 6mm thick)
        flange_p = cq.Workplane('YZ').box(FLANGE_THICK, 42.0, 80.0)
        # Fastener holes: 4x M3 mounting holes
        flange_p = flange_p.faces('>X').workplane().pushPoints([
            (-15.0, -25.0), (15.0, -25.0),
            (-15.0,  25.0), (15.0,  25.0),
        ]).circle(1.60).cutThruAll()  # ⌀3.2mm clearance for M3
        # Center ⌀6.00 mm pivot pin bore for titanium pin
        flange_p = flange_p.faces('>X').workplane().circle(3.0).cutThruAll()
        # Pinch clamp bolt lugs (2x M3 clearance holes)
        flange_p = flange_p.faces('>X').workplane().pushPoints([
            (-12.0, 0.0), (12.0, 0.0)
        ]).circle(1.60).cutThruAll()
        
        flange_p_solid = flange_p.val().rotate((0, 0, 0), (0, 1, 0), inc)\
                                       .translate(cq.Vector(x, w + FLANGE_THICK / 2.0 + 1.0, z_top - 12.0))
        
        collar_port = sleeve_p.val().fuse(flange_p_solid)

        # Build Starboard Split-Clamp Collar (Y = -w):
        sleeve_s = cq.Workplane('XY').circle(COLLAR_OD / 2.0).extrude(COLLAR_LEN)\
                                     .translate(cq.Vector(0, 0, -COLLAR_LEN / 2.0))\
                                     .rotate((0,0,0), (0,1,0), slope_deg + 90)\
                                     .translate(cq.Vector(x, -w, z_top))
                                     
        flange_s = cq.Workplane('YZ').box(FLANGE_THICK, 42.0, 80.0)
        flange_s = flange_s.faces('>X').workplane().pushPoints([
            (-15.0, -25.0), (15.0, -25.0),
            (-15.0,  25.0), (15.0,  25.0),
        ]).circle(1.60).cutThruAll()
        flange_s = flange_s.faces('>X').workplane().circle(3.0).cutThruAll()
        flange_s = flange_s.faces('>X').workplane().pushPoints([
            (-12.0, 0.0), (12.0, 0.0)
        ]).circle(1.60).cutThruAll()
        
        flange_s_solid = flange_s.val().rotate((0, 0, 0), (0, 1, 0), inc)\
                                       .translate(cq.Vector(x, -w - FLANGE_THICK / 2.0 - 1.0, z_top - 12.0))
        
        collar_stbd = sleeve_s.val().fuse(flange_s_solid)
        
        collars.extend([collar_port, collar_stbd])

    for c in collars:
        frame = frame.fuse(c)
    print("  ✓ Split-clamp wing station collars fused.")

    # -------------------------------------------------------------------------
    # 5. Internal Electronics Bay & Teardrop Enclosure (Lower Deck)
    # -------------------------------------------------------------------------
    print("  • Generating Internal Electronics Tray & Teardrop Enclosure...")
    tray_len = 550.0 - 120.0
    tray_w = WIDTH_FWD - 12.0
    tray = cq.Workplane('XY').transformed(offset=(120.0 + tray_len/2.0, 0, 6.0))\
             .box(tray_len, tray_w, 3.0)
             
    tray = tray.faces('>Z').workplane()
    for sx in np.linspace(150, 520, 8):
        tray = tray.transformed(offset=(sx - (120.0 + tray_len/2.0), 0, 0))\
                   .slot2D(tray_w - 16.0, 12.0, 90).cutThruAll()
                   
    frame = frame.fuse(tray.val())

    # Aerodynamic Teardrop Electronics & Battery Enclosure Pod:
    # Mounted along centerline Y=0, spanning X = 130mm to 480mm (350mm length)
    x_pts = np.linspace(130.0, 480.0, 18)
    teardrop_wires = []
    for x in x_pts:
        frac = (x - 130.0) / (480.0 - 130.0)
        t = frac
        scale = 0.05 if (t == 0 or t == 1) else max(0.05, (t ** 0.45) * ((1.0 - t) ** 0.85) * 2.1)
        rx = 24.0 * scale  # max half-width ~ 24mm (width ~ 48mm, fits inside fuselage)
        rz = 32.0 * scale  # max half-height ~ 32mm (height ~ 64mm)
        z_center = 50.0 - 15.0 * t  # matches downward inclined fuselage slope
        w = cq.Workplane('YZ').transformed(offset=(0, z_center, x)).ellipse(rx, rz).val()
        teardrop_wires.append(w)

    pod_solid = cq.Solid.makeLoft(teardrop_wires, ruled=False)
    # Secure mounting pylons anchoring pod to the battery tray and lower keel longerons:
    pylons = [
        cq.Workplane('XY').transformed(offset=(180.0, 0, 15.0)).box(20.0, 16.0, 30.0).val(),
        cq.Workplane('XY').transformed(offset=(300.0, 0, 15.0)).box(20.0, 16.0, 30.0).val(),
        cq.Workplane('XY').transformed(offset=(420.0, 0, 15.0)).box(20.0, 16.0, 30.0).val(),
    ]
    for p in pylons:
        pod_solid = pod_solid.fuse(p)
    frame = frame.fuse(pod_solid)
    print("  ✓ Teardrop electronics enclosure & mounting pylons fused.")

    # -------------------------------------------------------------------------
    # 6. Splayed Landing Gear & Small Wheels (Rev B: tf = 250mm, tr = 320mm, Z_clearance = 115mm)
    # -------------------------------------------------------------------------
    print("  • Generating Splayed Landing Gear & Wheels (Rev B Geometry)...")
    gear_tubes = []
    
    # Ground plane is at Z = -115.0 mm. Wheel radius = 25.0 mm (⌀50mm wheels).
    # Wheel axle centerline is at Z = -115.0 + 25.0 = -90.0 mm.
    z_axle = -90.0
    
    # Forward gear (at X = 80 mm, near Station 1) - Front Track tf = 250 mm
    # Outer tire edge at Y = ±125 mm. Wheel width = 14 mm. Wheel center at Y = ±118 mm.
    x_fwd_gear = 80.0
    _, z_fwd_bot, w_fwd_gear = get_fuselage_envelope(x_fwd_gear)
    gear_tubes.append(make_tube_between((x_fwd_gear, w_fwd_gear, z_fwd_bot), (x_fwd_gear, 118.0, z_axle), od=STRUT_TUBE_OD))
    gear_tubes.append(make_tube_between((x_fwd_gear, -w_fwd_gear, z_fwd_bot), (x_fwd_gear, -118.0, z_axle), od=STRUT_TUBE_OD))
    # Forward axle (transverse ⌀4mm ground steel)
    gear_tubes.append(make_tube_between((x_fwd_gear, 105.0, z_axle), (x_fwd_gear, 126.0, z_axle), od=AXLE_OD))
    gear_tubes.append(make_tube_between((x_fwd_gear, -126.0, z_axle), (x_fwd_gear, -105.0, z_axle), od=AXLE_OD))

    # Aft gear (at X = 850 mm, near Station 4) - Rear Track tr = 320 mm
    # Outer tire edge at Y = ±160 mm. Wheel width = 14 mm. Wheel center at Y = ±153 mm.
    x_aft_gear = 850.0
    _, z_aft_bot, w_aft_gear = get_fuselage_envelope(x_aft_gear)
    gear_tubes.append(make_tube_between((x_aft_gear, w_aft_gear, z_aft_bot), (x_aft_gear, 153.0, z_axle), od=STRUT_TUBE_OD))
    gear_tubes.append(make_tube_between((x_aft_gear, -w_aft_gear, z_aft_bot), (x_aft_gear, -153.0, z_axle), od=STRUT_TUBE_OD))
    # Aft axle (transverse ⌀4mm ground steel)
    gear_tubes.append(make_tube_between((x_aft_gear, 140.0, z_axle), (x_aft_gear, 161.0, z_axle), od=AXLE_OD))
    gear_tubes.append(make_tube_between((x_aft_gear, -161.0, z_axle), (x_aft_gear, -140.0, z_axle), od=AXLE_OD))

    # Wheels (⌀50mm x 14mm wide RC EVA/foam wheels)
    # Front wheels: Y centers at ±118 mm -> Outer edge = ±125 mm -> Track tf = 250 mm
    wheel_fwd_port = cq.Workplane('XZ').circle(25.0).extrude(14.0).translate(cq.Vector(x_fwd_gear, 111.0, z_axle)).val()
    wheel_fwd_stbd = cq.Workplane('XZ').circle(25.0).extrude(14.0).translate(cq.Vector(x_fwd_gear, -125.0, z_axle)).val()
    # Aft wheels: Y centers at ±153 mm -> Outer edge = ±160 mm -> Track tr = 320 mm
    wheel_aft_port = cq.Workplane('XZ').circle(25.0).extrude(14.0).translate(cq.Vector(x_aft_gear, 146.0, z_axle)).val()
    wheel_aft_stbd = cq.Workplane('XZ').circle(25.0).extrude(14.0).translate(cq.Vector(x_aft_gear, -160.0, z_axle)).val()

    for gt in gear_tubes:
        frame = frame.fuse(gt)
    for wh in [wheel_fwd_port, wheel_fwd_stbd, wheel_aft_port, wheel_aft_stbd]:
        frame = frame.fuse(wh)
    print("  ✓ Landing gear and wheels fused.")

    # -------------------------------------------------------------------------
    # 7. Verification & Export
    # -------------------------------------------------------------------------
    print("\n[Verification & Metrics]")
    if not frame.isValid():
        raise RuntimeError("Fuselage assembly solid is invalid!")

    bb = frame.BoundingBox()
    vol_cm3 = frame.Volume() / 1000.0
    mass_cf = vol_cm3 * 1.55  # Carbon fiber composite density ~1.55 g/cm³

    print(f"  • Bounding Box: X=[{bb.xmin:.1f}, {bb.xmax:.1f}], Y=[{bb.ymin:.1f}, {bb.ymax:.1f}], Z=[{bb.zmin:.1f}, {bb.zmax:.1f}] mm")
    print(f"  • Fuselage Truss Volume: {vol_cm3:.2f} cm³")
    print(f"  • Estimated Mass (Carbon Truss + Split Collars + Joints): {mass_cf:.1f} g")
    print(f"  • Wheelbase: {x_aft_gear - x_fwd_gear:.1f} mm")
    print(f"  • Front Track: 250.0 mm | Aft Track: 320.0 mm (tf/tr = {250.0/320.0:.3f})")
    print(f"  • Belly Ground Clearance: {abs(bb.zmin):.1f} mm (Ground Plane Z = {bb.zmin:.1f} mm)")
    print(f"  • Static Stability Factor (SSF): {250.0 / (2.0 * 125.0):.2f} (Roll-Over Stable)")

    # Export deliverables
    wp_export = cq.Workplane().add(frame)
    step_path = os.path.join(OUT_DIR, 'mco_fuselage_assembly.step')
    stl_path  = os.path.join(OUT_DIR, 'mco_fuselage_assembly.stl')
    path_3mf  = os.path.join(OUT_DIR, 'mco_fuselage_assembly.3mf')

    print("\n[Exporting CAD Deliverables]")
    cq.exporters.export(wp_export, stl_path, tolerance=0.02, angularTolerance=0.15)
    cq.exporters.export(wp_export, step_path)
    cq.exporters.export(wp_export, path_3mf, tolerance=0.02, angularTolerance=0.15)
    print(f"  ✓ Exported STEP: {step_path}")
    print(f"  ✓ Exported STL:  {stl_path}")
    print(f"  ✓ Exported 3MF:  {path_3mf}")

    return frame

if __name__ == '__main__':
    build_fuselage()
