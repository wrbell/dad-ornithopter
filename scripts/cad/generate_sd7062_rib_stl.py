#!/usr/bin/env python3
"""
scripts/generate_sd7062_rib_stl.py
==================================
Parametrically models and exports 3D-printable wing ribs for the SD7062
(Selig-Donovan low Reynolds number) airfoil at chord c = 165.0 mm.

Spars configured per MCO-1 specification at identical chord percentages:
  - Fwd Spar (#1, 10% chord): X = 16.50 mm, ⌀2.40 mm bore (for ⌀2.0 mm carbon rod)
  - Mid Spar (#2, 20% chord): X = 33.00 mm, ⌀10.50 mm bore (for ⌀10.0 mm carbon tube)
  - Aft Spar (#3, 55% chord): X = 90.75 mm, ⌀2.40 mm bore (for ⌀2.0 mm carbon rod)

Exports:
  1. cad/manufactured/sd7062_wing_rib_165mm.stl (Primary 3D-printable rib, 2.4 mm thick)
  2. cad/manufactured/sd7062_wing_rib_chordline_165mm.stl (Chord-line datum reference)
  3. cad/manufactured/sd7062_wing_rib_lightened_165mm.stl (Lightened flight rib with oval weight-relief bays)
  4. cad/manufactured/sd7062_wing_rib_165mm.step (Master CAD STEP file)
  5. cad/coordinates/SD7062_165mm.txt (Calibrated coordinate manifest)
"""

import os
import sys
import math
import numpy as np
import cadquery as cq

OUT_DIR = 'cad/manufactured'
COORD_DIR = 'cad/coordinates'
os.makedirs(OUT_DIR, exist_ok=True)
os.makedirs(COORD_DIR, exist_ok=True)

CHORD = 165.0         # mm
RIB_THICKNESS = 2.40  # mm: Standard MCO wing rib extrusion thickness
TE_MIN_THICK = 1.00   # mm: Minimum trailing edge thickness for clean FDM slicing

def load_sd7062_points(chord=CHORD, te_min=TE_MIN_THICK):
    """Load SD7062 coordinates and scale to specified chord with robust TE."""
    dat_path = 'ornithopter-cad/data/airfoils/sd7062.dat'
    with open(dat_path, 'r') as f:
        lines = f.readlines()
    
    raw = []
    for l in lines[1:]:
        toks = l.strip().split()
        if len(toks) >= 2:
            try:
                raw.append([float(toks[0]), float(toks[1])])
            except ValueError:
                pass
    raw = np.array(raw)
    pts = raw * chord

    # Apply subtle trailing edge thickening if sharp TE would vanish in slicing
    x_max = pts[:, 0].max()
    le_idx = np.argmin(pts[:, 0])
    upper = pts[:le_idx+1].copy()
    lower = pts[le_idx:].copy()

    # Thickening over aft 15% chord (X > 140 mm)
    x_blend = chord * 0.85
    for i in range(len(pts)):
        px, py = pts[i]
        if px > x_blend:
            w = ((px - x_blend) / (chord - x_blend)) ** 1.2
            if i <= le_idx: # Upper
                pts[i, 1] += (te_min / 2.0) * w
            else:           # Lower
                pts[i, 1] -= (te_min / 2.0) * w

    return pts

def create_sd7062_wire(pts):
    """Create a closed CadQuery wire from points."""
    c_pts = [cq.Vector(p[0], p[1], 0.0) for p in pts]
    if np.linalg.norm(pts[0] - pts[-1]) > 1e-4:
        c_pts.append(cq.Vector(pts[0][0], pts[0][1], 0.0))
    return cq.Wire.makePolygon(c_pts)

def get_airfoil_metrics(pts, chord=CHORD):
    """Calculate thickness and camber across chord."""
    le_idx = np.argmin(pts[:, 0])
    upper = pts[:le_idx+1]
    lower = pts[le_idx:]
    
    metrics = {}
    for pct, name, bore_dia in [(0.10, 'Fwd', 2.40), (0.20, 'Mid', 10.50), (0.55, 'Aft', 2.40)]:
        x = pct * chord
        yu = float(np.interp(pct, upper[::-1, 0]/chord, upper[::-1, 1]))
        yl = float(np.interp(pct, lower[:, 0]/chord, lower[:, 1]))
        yc = 0.5 * (yu + yl)
        t = yu - yl
        metrics[name] = {
            'pct': pct, 'x': x, 'yu': yu, 'yl': yl, 'yc': yc, 't': t, 'bore_dia': bore_dia
        }
    return metrics

def build_rib(pts, spar_mode='camber', lightened=False, thickness=RIB_THICKNESS):
    """
    Build a solid 3D-printable rib.
    spar_mode:
      - 'camber': Spars placed along local mean camber line (optimal wall margin)
      - 'chord': Spars placed along geometric chord line Y = 0.0
    """
    wire = create_sd7062_wire(pts)
    face = cq.Face.makeFromWires(wire)
    solid = cq.Workplane('XY').add(face).extrude(thickness)
    
    metrics = get_airfoil_metrics(pts)
    
    # 1. Drill Spar Bores
    for name in ['Fwd', 'Mid', 'Aft']:
        m = metrics[name]
        x = m['x']
        bore_r = m['bore_dia'] / 2.0
        y = m['yc'] if spar_mode == 'camber' else 0.0
        cutter = cq.Workplane('XY').transformed(offset=(x, y, -2.0)).circle(bore_r).extrude(thickness + 4.0)
        solid = solid.cut(cutter)
        
    # 2. Add Lightening Holes if requested
    if lightened:
        # Aft of main spar, oval cutouts between X=50mm and X=130mm
        ovals = [
            (60.0, metrics['Mid']['yc'] + 0.5, 7.5, 5.0),
            (76.0, metrics['Mid']['yc'] + 0.5, 6.5, 4.2),
            (106.0, metrics['Aft']['yc'] - 0.5, 5.5, 2.8),
            (120.0, metrics['Aft']['yc'] - 1.2, 4.5, 1.8),
        ]
        for hx, hy, rx, ry in ovals:
            cutter = cq.Workplane('XY').transformed(offset=(hx, hy, -2.0)).ellipse(rx, ry).extrude(thickness + 4.0)
            solid = solid.cut(cutter)
            
    val = solid.val()
    if not val.isValid():
        raise RuntimeError(f"Solid is invalid for mode {spar_mode} (lightened={lightened})")
    return solid

def main():
    print("=" * 75)
    print("MCO-1.5D SD7062 WING RIB GENERATOR (CHORD = 165.0 mm)")
    print("=" * 75)
    
    pts = load_sd7062_points()
    metrics = get_airfoil_metrics(pts)
    
    print("\n[1/4] Calibrated Spar Stations (Chord = 165.0 mm):")
    for name, m in metrics.items():
        print(f"  • {name} Spar ({m['pct']*100:.0f}% chord): X = {m['x']:.2f} mm")
        print(f"      Section Thickness: {m['t']:.2f} mm (Upper: {m['yu']:.2f} mm, Lower: {m['yl']:.2f} mm)")
        print(f"      Mean Camber Height: Y = {m['yc']:.2f} mm | Chord Datum: Y = 0.00 mm")
        print(f"      Bore: ⌀{m['bore_dia']:.2f} mm (R = {m['bore_dia']/2:.2f} mm)")
        
    # Save coordinate manifest
    coord_txt_path = os.path.join(COORD_DIR, 'SD7062_165mm.txt')
    with open(coord_txt_path, 'w') as f:
        f.write("# SD7062 Calibrated Airfoil Coordinates (Chord = 165.0 mm)\n")
        f.write("# X (mm)      Y (mm)      Z (mm)\n")
        for p in pts:
            f.write(f"{p[0]:10.4f}  {p[1]:10.4f}  0.0000\n")
    print(f"\n[2/4] Wrote coordinate manifest: {coord_txt_path}")
    
    # Generate Rib 1: Camber-aligned Spars (Primary flight rib)
    print("\n[3/4] Generating Primary SD7062 Flight Rib (Camber-aligned Spar Bores)...")
    rib_primary = build_rib(pts, spar_mode='camber', lightened=False)
    stl_primary = os.path.join(OUT_DIR, 'sd7062_wing_rib_165mm.stl')
    step_primary = os.path.join(OUT_DIR, 'sd7062_wing_rib_165mm.step')
    cq.exporters.export(rib_primary, stl_primary, tolerance=0.005, angularTolerance=0.05)
    cq.exporters.export(rib_primary, step_primary)
    v1 = rib_primary.val().Volume()
    print(f"  ✓ Exported: {stl_primary} (Volume: {v1:.1f} mm³, PLA mass: {v1/1000*1.24:.2f} g, LW-PLA: {v1/1000*0.55:.2f} g)")
    print(f"  ✓ Exported: {step_primary}")
    
    # Generate Rib 2: Chordline-aligned Spars (Y = 0)
    print("\n[4/4] Generating Geometric Chordline-aligned Rib (Y = 0 Reference)...")
    rib_chord = build_rib(pts, spar_mode='chord', lightened=False)
    stl_chord = os.path.join(OUT_DIR, 'sd7062_wing_rib_chordline_165mm.stl')
    cq.exporters.export(rib_chord, stl_chord, tolerance=0.005, angularTolerance=0.05)
    print(f"  ✓ Exported: {stl_chord}")
    
    # Generate Rib 3: Lightened Flight Rib with Oval Cutouts
    print("      Generating Lightened Flight Rib with Calibrated Oval Weight-Relief Bays...")
    rib_light = build_rib(pts, spar_mode='camber', lightened=True)
    stl_light = os.path.join(OUT_DIR, 'sd7062_wing_rib_lightened_165mm.stl')
    cq.exporters.export(rib_light, stl_light, tolerance=0.005, angularTolerance=0.05)
    v_light = rib_light.val().Volume()
    print(f"  ✓ Exported: {stl_light} (Volume: {v_light:.1f} mm³, saved {(v1-v_light)/v1*100:.1f}% mass!)")
    
    print("\n" + "=" * 75)
    print("ALL SD7062 CAD ARTIFACTS SUCCESSFULLY GENERATED & VERIFIED!")
    print("=" * 75)

if __name__ == '__main__':
    main()
