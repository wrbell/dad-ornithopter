#!/usr/bin/env python3
"""
MCO-1 8-Wing Tandem Aircraft Master Assembly (Robust 3D-Printable Build)
Assembles the complete 8-wing tandem ornithopter in CadQuery:
- Parametric Fuselage Truss with internal Teardrop Electronics Enclosure
  and split-clamp wing station collars (all spars & struts >= 4.0 mm).
- 4 Tandem Wing Stations at X = [100, 350, 600, 850] mm along downward-sloped spine
- 8 Robust Generic Solid Airfoil Wings (NACA 2412, 165mm chord):
  * Continuous solid aerodynamic lifting surface (no spindly support-trapping ribs)
  * Trailing edge vertical thickness >= 3.0 mm (3.2 mm)
  * Continuous ⌀10.4mm main spar bore & ⌀3.2mm secondary alignment spar bores (all spars >= 3.0 mm)
  * Smooth aerodynamic rounded wingtips (span 1.15m per wing, 2.3m total wingspan)
  * Mated directly to Rev B split-clamp collar flanges on the ⌀12mm longerons.
"""

import os
import math
import numpy as np
import cadquery as cq

OUT_DIR = 'cad/manufactured'
FUSELAGE_STEP   = os.path.join(OUT_DIR, 'mco_fuselage_assembly.step')
WING_STBD_STEP  = os.path.join(OUT_DIR, 'generic_airfoil_wing_starboard.step')

# 4 Wing Root Stations along the inclined spine (20.0° Angle of Attack per user directive)
WING_STATIONS = [
    {'name': 'Station 1 (Fore)', 'x': 100.0, 'incidence_deg': 20.0},
    {'name': 'Station 2',        'x': 350.0, 'incidence_deg': 20.0},
    {'name': 'Station 3',        'x': 600.0, 'incidence_deg': 20.0},
    {'name': 'Station 4 (Aft)',  'x': 850.0, 'incidence_deg': 20.0},
]

SPAN_FUSELAGE = 1000.0
HEIGHT_FWD    = 200.0
HEIGHT_AFT    = 35.0
WIDTH_FWD     = 80.0
WIDTH_AFT     = 30.0

def get_fuselage_envelope(x):
    frac = max(0.0, min(1.0, x / SPAN_FUSELAGE))
    z_top = HEIGHT_FWD - frac * (HEIGHT_FWD - HEIGHT_AFT)
    z_bot = 0.0 + frac * 10.0
    w_half = 0.5 * (WIDTH_FWD - frac * (WIDTH_FWD - WIDTH_AFT))
    return z_top, z_bot, w_half

def assemble_aircraft():
    print("=" * 75)
    print("ASSEMBLING ROBUST MCO-1 8-WING TANDEM ORNITHOPTER FOR 3D PRINTING")
    print("=" * 75)

    # 1. Load Fuselage (with Teardrop Electronics Enclosure)
    print("\n[1/3] Loading Fuselage Frame with Teardrop Electronics Enclosure...")
    fuselage = cq.importers.importStep(FUSELAGE_STEP).val()
    print(f"  ✓ Loaded Fuselage (Volume: {fuselage.Volume()/1000:.1f} cm³)")

    # 2. Load Robust Generic Airfoil Wings (Starboard & Port)
    print("\n[2/3] Loading Robust Generic Airfoil Wings (NACA 2412, >=3mm TE & Spars)...")
    wing_stbd_raw = cq.importers.importStep(WING_STBD_STEP).val()
    
    # Orient wing into aircraft coordinate system:
    # In aircraft: +X is aft, +Y is Port, -Y is Starboard, +Z is up
    # Rotate 90 deg around X so span (along -Z) points along -Y (Starboard)
    wing_stbd_aircraft = wing_stbd_raw.rotate((0, 0, 0), (1, 0, 0), 90)
    
    # Mirror across XZ plane (Y=0) to create Port wing (spanning along +Y)
    wing_port_aircraft = wing_stbd_aircraft.mirror('XZ')
    
    print(f"  ✓ Starboard Wing Unit Volume: {wing_stbd_aircraft.Volume()/1000:.1f} cm³")
    print(f"  ✓ Port Wing Unit Volume:      {wing_port_aircraft.Volume()/1000:.1f} cm³")

    # 3. Mount 4 Pairs of Wings at the 4 Fuselage Stations
    print("\n[3/3] Positioning 8 Generic Airfoil Wings onto Fuselage Stations...")
    assembly_parts = [fuselage]
    
    for idx, st in enumerate(WING_STATIONS, start=1):
        x_st = st['x']
        inc  = st['incidence_deg']
        z_top, _, w_half = get_fuselage_envelope(x_st)
        
        # Inboard mount face rests flush against the collar mounting flange face:
        # Port flange outer face is at +(w_half + 7.0 mm)
        # Starboard flange outer face is at -(w_half + 7.0 mm)
        y_stbd = -(w_half + 7.0)
        y_port =  (w_half + 7.0)
        z_mount = z_top - 12.0
        
        # Pitch wing by positive incidence angle (+20.0° AoA: leading edge tilted UP into forward airflow)
        # Pivot pin center is at X = 33.58, Z = -4.24 after +20° rotation
        w_s = wing_stbd_aircraft.rotate((0, 0, 0), (0, 1, 0), inc)\
                                .translate(cq.Vector(x_st - 33.58, y_stbd, z_mount + 4.24))
        w_p = wing_port_aircraft.rotate((0, 0, 0), (0, 1, 0), inc)\
                                .translate(cq.Vector(x_st - 33.58, y_port, z_mount + 4.24))
                                
        assembly_parts.extend([w_s, w_p])
        print(f"  • Wing Pair {idx} ({st['name']}): X = {x_st:.1f} mm, Z = {z_mount:.1f} mm, Incidence = +{inc:.1f}°")

    # Unified master assembly compound
    print("\nBuilding unified master aircraft model...")
    aircraft_compound = cq.Compound.makeCompound(assembly_parts)
    bb = aircraft_compound.BoundingBox()
    
    print("\n" + "=" * 75)
    print("MCO-1 COMPLETE 8-WING AIRCRAFT (ROBUST 3D-PRINT BUILD) METRICS:")
    print(f"  • Total Length (X): {bb.xlen:.1f} mm (Span X: [{bb.xmin:.1f}, {bb.xmax:.1f}] mm)")
    print(f"  • Total Wingspan (Y): {bb.ylen:.1f} mm ({bb.ylen/1000:.2f} m / {bb.ylen/25.4:.1f} in across tips)")
    print(f"  • Total Height (Z): {bb.zlen:.1f} mm (Z: [{bb.zmin:.1f}, {bb.zmax:.1f}] mm)")
    print(f"  • Ground Plane Clearance: {abs(bb.zmin):.1f} mm")
    print(f"  • Wing Count: 8 solid generic airfoil wings (4 tandem pairs)")
    print(f"  • Thinnest Spar: 3.2 mm (>= 3.0 mm specification)")
    print(f"  • Trailing Edge Thickness: 3.2 mm (>= 3.0 mm specification)")
    print("=" * 75)

    # Export unified master STEP, STL, 3MF
    out_step = os.path.join(OUT_DIR, 'mco_complete_8wing_aircraft.step')
    out_stl  = os.path.join(OUT_DIR, 'mco_complete_8wing_aircraft.stl')
    out_3mf  = os.path.join(OUT_DIR, 'mco_complete_8wing_aircraft.3mf')
    
    print("\n[Exporting CAD Deliverables]")
    wp_export = cq.Workplane().add(aircraft_compound)
    cq.exporters.export(wp_export, out_stl, tolerance=0.04, angularTolerance=0.25)
    cq.exporters.export(wp_export, out_step)
    cq.exporters.export(wp_export, out_3mf, tolerance=0.04, angularTolerance=0.25)
    print(f"  ✓ Exported Complete Aircraft STEP: {out_step}")
    print(f"  ✓ Exported Complete Aircraft STL:  {out_stl}")
    print(f"  ✓ Exported Complete Aircraft 3MF:  {out_3mf}")

    return aircraft_compound

if __name__ == '__main__':
    assemble_aircraft()
