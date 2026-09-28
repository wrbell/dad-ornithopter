#!/usr/bin/env python3
"""
scripts/generate_cad_assets.py
==============================
Generates physical manufacturing files for MCO-1D:
  1. cad/coordinates/MCO_S1223_120mm.txt (SolidWorks Curve XYZ import)
  2. cad/manufactured/s1223_universal_rib_120mm.dxf (2D laser cut DXF)
  3. cad/manufactured/s1223_universal_rib_120mm.stl (3D printable LW-PLA rib mesh)
  4. cad/manufactured/sector_wheel_7to1_100deg.dxf (2D CNC cut sheet for 2mm CF sheet)
  5. cad/manufactured/sector_wheel_7to1_100deg.stl (3D printable sector wheel)
"""

import os, math, struct

COORD_POINTS = [
    # Upper Surface (LE to TE)
    (0.000, 0.000), (0.151, 1.001), (0.542, 1.958), (1.522, 3.416),
    (3.048, 4.914), (5.729, 6.872), (9.253, 8.771), (13.516, 10.522),
    (18.388, 12.047), (23.738, 13.297), (29.436, 14.244), (36.124, 14.861),
    (41.326, 15.108), (47.244, 14.996), (52.963, 14.544), (58.354, 13.791),
    (63.293, 12.791), (67.629, 11.597), (71.337, 10.273), (74.141, 8.896),
    (78.918, 7.291), (84.101, 5.616), (89.597, 4.049), (95.174, 2.686),
    (100.533, 1.622), (105.640, 0.878), (110.054, 0.415), (113.975, 0.137),
    (116.622, 0.002), (116.400, 0.000),
    # Trailing Edge Close
    (116.400, -0.800),
    # Lower Surface (TE back to LE)
    (116.622, -0.036), (113.975, -0.143), (110.054, -0.393), (105.640, -0.623),
    (100.533, -0.798), (95.174, -0.917), (89.597, -0.979), (84.101, -0.972),
    (78.918, -0.878), (74.141, -0.769), (71.337, -0.630), (67.629, -0.498),
    (63.293, -0.357), (58.354, -0.214), (52.963, -0.099), (47.244, -0.009),
    (41.326, 0.040), (36.124, 0.076), (29.436, 0.075), (23.738, 0.042),
    (18.388, -0.017), (13.516, -0.088), (9.253, -0.194), (5.729, -0.318),
    (3.048, -0.396), (1.522, -0.339), (0.542, -0.206), (0.151, -0.092),
    (0.000, 0.000)
]

def write_coordinates_txt():
    out_path = 'cad/coordinates/MCO_S1223_120mm.txt'
    with open(out_path, 'w') as f:
        for x, y in COORD_POINTS:
            f.write(f"{x:.3f} {y:.3f} 0.000\n")
    print(f"Generated {out_path} ({len(COORD_POINTS)} points)")

def write_rib_dxf():
    out_path = 'cad/manufactured/s1223_universal_rib_120mm.dxf'
    with open(out_path, 'w') as f:
        f.write("0\nSECTION\n2\nHEADER\n0\nENDSEC\n")
        f.write("0\nSECTION\n2\nENTITIES\n")
        
        # 1. Airfoil Outer Polyline
        f.write("0\nPOLYLINE\n8\nAIRFOIL_OUTLINE\n66\n1\n70\n1\n")
        for x, y in COORD_POINTS:
            f.write(f"0\nVERTEX\n8\nAIRFOIL_OUTLINE\n10\n{x:.3f}\n20\n{y:.3f}\n30\n0.0\n")
        f.write("0\nSEQEND\n")
        
        # 2. Main 10mm CF Spar Bore (x=15.0, y=5.5, dia=10.10 mm, radius=5.05 mm)
        f.write("0\nCIRCLE\n8\nMAIN_SPAR_BORE_10MM\n10\n15.000\n20\n5.500\n30\n0.0\n40\n5.050\n")
        
        # 3. Leading Edge Alignment Pin / Secondary Bore (x=1.80, y=1.85, dia=2.10 mm)
        f.write("0\nCIRCLE\n8\nLE_PIN_BORE_2MM\n10\n1.800\n20\n1.850\n30\n0.0\n40\n1.050\n")
        
        # 4. Secondary Spar Notch (x=72.0, y=9.8, dia=3.10 mm)
        f.write("0\nCIRCLE\n8\nSECONDARY_SPAR_3MM\n10\n72.000\n20\n9.800\n30\n0.0\n40\n1.550\n")
        
        f.write("0\nENDSEC\n0\nEOF\n")
    print(f"Generated {out_path}")

def write_binary_stl(filename, facets):
    """Write binary STL file from list of ((n_x,n_y,n_z), (v1), (v2), (v3))"""
    with open(filename, 'wb') as f:
        # 80-byte header
        header = f"MCO-1D CAD Model Export - {os.path.basename(filename)}".ljust(80)[:80].encode('ascii')
        f.write(header)
        # Number of triangles
        f.write(struct.pack('<I', len(facets)))
        for norm, v1, v2, v3 in facets:
            f.write(struct.pack('<3f', *norm))
            f.write(struct.pack('<3f', *v1))
            f.write(struct.pack('<3f', *v2))
            f.write(struct.pack('<3f', *v3))
            f.write(struct.pack('<H', 0)) # attribute byte count

def calc_normal(p1, p2, p3):
    u = (p2[0]-p1[0], p2[1]-p1[1], p2[2]-p1[2])
    v = (p3[0]-p1[0], p3[1]-p1[1], p3[2]-p1[2])
    nx = u[1]*v[2] - u[2]*v[1]
    ny = u[2]*v[0] - u[0]*v[2]
    nz = u[0]*v[1] - u[1]*v[0]
    l = math.sqrt(nx*nx + ny*ny + nz*nz)
    if l > 0:
        return (nx/l, ny/l, nz/l)
    return (0.0, 0.0, 1.0)

def write_rib_stl():
    out_path = 'cad/manufactured/s1223_universal_rib_120mm.stl'
    thickness = 1.20 # mm (standard rib thickness)
    pts = COORD_POINTS[:-1] # avoid duplicate LE close point
    N = len(pts)
    facets = []
    
    # Polygon triangulation for front face (z=0) and back face (z=thickness)
    # Simple fan triangulation from center of chord (40.0, 7.0)
    cx, cy = 40.0, 7.0
    c_bot = (cx, cy, 0.0)
    c_top = (cx, cy, thickness)
    
    for i in range(N):
        p1 = (pts[i][0], pts[i][1], 0.0)
        p2 = (pts[(i+1)%N][0], pts[(i+1)%N][1], 0.0)
        norm = (0.0, 0.0, -1.0)
        facets.append((norm, c_bot, p2, p1))
        
        p1_top = (pts[i][0], pts[i][1], thickness)
        p2_top = (pts[(i+1)%N][0], pts[(i+1)%N][1], thickness)
        norm_top = (0.0, 0.0, 1.0)
        facets.append((norm_top, c_top, p1_top, p2_top))
        
        # Side walls connecting z=0 to z=thickness
        n_side = calc_normal(p1, p2, p2_top)
        facets.append((n_side, p1, p2, p2_top))
        facets.append((n_side, p1, p2_top, p1_top))
        
    write_binary_stl(out_path, facets)
    print(f"Generated {out_path} ({len(facets)} triangles, thickness={thickness}mm)")

def write_sector_wheel():
    dxf_path = 'cad/manufactured/sector_wheel_7to1_100deg.dxf'
    stl_path = 'cad/manufactured/sector_wheel_7to1_100deg.stl'
    
    # Sector wheel parameters:
    # 7:1 ratio with motor drum diameter d_drum = 22mm (radius 11mm) -> R_sector = 77.0 mm
    # 100° arc (±50°)
    R_outer = 77.0  # mm
    R_inner = 63.0  # mm lightening cutout
    pivot_dia = 6.0 # mm center pivot
    thickness = 2.0 # mm CF sheet
    
    # Generate DXF
    with open(dxf_path, 'w') as f:
        f.write("0\nSECTION\n2\nHEADER\n0\nENDSEC\n")
        f.write("0\nSECTION\n2\nENTITIES\n")
        # Center pivot
        f.write(f"0\nCIRCLE\n8\nPIVOT\n10\n0.0\n20\n0.0\n30\n0.0\n40\n{pivot_dia/2:.3f}\n")
        # Bolt circle (4x M3 at PCD 20mm)
        for ang in [45, 135, 225, 315]:
            bx = 10.0 * math.cos(math.radians(ang))
            by = 10.0 * math.sin(math.radians(ang))
            f.write(f"0\nCIRCLE\n8\nBOLT_HOLES\n10\n{bx:.3f}\n20\n{by:.3f}\n30\n0.0\n40\n1.60\n")
        # Outer Arc (100° arc from -50° to +50°)
        f.write(f"0\nARC\n8\nOUTER_RIM\n10\n0.0\n20\n0.0\n30\n0.0\n40\n{R_outer:.3f}\n50\n-50.0\n51\n50.0\n")
        # End radials
        x1, y1 = R_outer * math.cos(math.radians(-50)), R_outer * math.sin(math.radians(-50))
        x2, y2 = R_outer * math.cos(math.radians(50)), R_outer * math.sin(math.radians(50))
        f.write(f"0\nLINE\n8\nBOUNDS\n10\n0.0\n20\n0.0\n30\n0.0\n11\n{x1:.3f}\n21\n{y1:.3f}\n31\n0.0\n")
        f.write(f"0\nLINE\n8\nBOUNDS\n10\n0.0\n20\n0.0\n30\n0.0\n11\n{x2:.3f}\n21\n{y2:.3f}\n31\n0.0\n")
        f.write("0\nENDSEC\n0\nEOF\n")
    print(f"Generated {dxf_path}")
    
    # Generate 3D printable Sector Wheel STL
    steps = 40
    d_theta = math.radians(100.0) / steps
    start_theta = math.radians(-50.0)
    rim_pts = []
    for s in range(steps + 1):
        th = start_theta + s * d_theta
        rim_pts.append((R_outer * math.cos(th), R_outer * math.sin(th)))
    
    facets = []
    c_bot = (0.0, 0.0, 0.0)
    c_top = (0.0, 0.0, thickness)
    for i in range(steps):
        p1_bot = (rim_pts[i][0], rim_pts[i][1], 0.0)
        p2_bot = (rim_pts[i+1][0], rim_pts[i+1][1], 0.0)
        facets.append(((0.0, 0.0, -1.0), c_bot, p2_bot, p1_bot))
        
        p1_top = (rim_pts[i][0], rim_pts[i][1], thickness)
        p2_top = (rim_pts[i+1][0], rim_pts[i+1][1], thickness)
        facets.append(((0.0, 0.0, 1.0), c_top, p1_top, p2_top))
        
        n_side = calc_normal(p1_bot, p2_bot, p2_top)
        facets.append((n_side, p1_bot, p2_bot, p2_top))
        facets.append((n_side, p1_bot, p2_top, p1_top))
        
    # Radial closing walls
    r1_b = (rim_pts[0][0], rim_pts[0][1], 0.0)
    r1_t = (rim_pts[0][0], rim_pts[0][1], thickness)
    facets.append(((0.0, 1.0, 0.0), c_bot, r1_b, r1_t))
    facets.append(((0.0, 1.0, 0.0), c_bot, r1_t, c_top))
    
    r2_b = (rim_pts[-1][0], rim_pts[-1][1], 0.0)
    r2_t = (rim_pts[-1][0], rim_pts[-1][1], thickness)
    facets.append(((0.0, -1.0, 0.0), c_bot, r2_t, r2_b))
    facets.append(((0.0, -1.0, 0.0), c_bot, c_top, r2_t))
    
    write_binary_stl(stl_path, facets)
    print(f"Generated {stl_path} ({len(facets)} triangles, thickness={thickness}mm)")

if __name__ == '__main__':
    write_coordinates_txt()
    write_rib_dxf()
    write_rib_stl()
    write_sector_wheel()
    print("CAD asset generation complete.")
