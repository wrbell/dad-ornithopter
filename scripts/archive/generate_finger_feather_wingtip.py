#!/usr/bin/env python3
"""
scripts/generate_finger_feather_wingtip.py
==========================================
Generates a truly organic "fingers from a hand" biomimetic wingtip where the
monolithic S1223 base body bifurcates directly and seamlessly into 5 distinct
splayed primary feathers like fingers growing out of a palm.
"""

import os, math, struct
import numpy as np

COORD_PATH = 'cad/coordinates/MCO_S1223_MOD_165mm.txt'
OUT_DIR = 'cad/manufactured'

SPAR_SOCKETS = [
    (18.71,  8.09, 1.10, 22.0),   # #1 Fwd LE
    (37.79, 12.53, 1.10, 22.0),   # #2 Mid
    (104.49, 16.33, 1.10, 22.0),  # #3 Aft
]
SOCKET_SEGS = 20

def load_contour():
    pts = []
    with open(COORD_PATH, 'r') as f:
        for l in f:
            parts = l.strip().split()
            if parts:
                pts.append((float(parts[0]), float(parts[1])))
    return pts

def tri_normal(p1, p2, p3):
    u = (p2[0]-p1[0], p2[1]-p1[1], p2[2]-p1[2])
    v = (p3[0]-p1[0], p3[1]-p1[1], p3[2]-p1[2])
    nx = u[1]*v[2] - u[2]*v[1]
    ny = u[2]*v[0] - u[0]*v[2]
    nz = u[0]*v[1] - u[1]*v[0]
    l = math.sqrt(nx*nx + ny*ny + nz*nz)
    if l > 1e-12:
        return (nx/l, ny/l, nz/l)
    return (0.0, 0.0, 1.0)

def write_binary_stl(filename, facets):
    os.makedirs(os.path.dirname(filename), exist_ok=True)
    with open(filename, 'wb') as f:
        hdr = f"MCO-1D Organic Finger-Feather Splayed Wingtip - {os.path.basename(filename)}".ljust(80)[:80].encode('ascii')
        f.write(hdr)
        f.write(struct.pack('<I', len(facets)))
        for norm, v1, v2, v3 in facets:
            f.write(struct.pack('<3f', *norm))
            f.write(struct.pack('<3f', *v1))
            f.write(struct.pack('<3f', *v2))
            f.write(struct.pack('<3f', *v3))
            f.write(struct.pack('<H', 0))
    kb = os.path.getsize(filename) / 1024.0
    print(f"  [OK] Generated {filename} ({len(facets):,} triangles, {kb:.1f} KB)")

def make_symmetrical_airfoil_ring(chord, thickness_ratio=0.12, n_pts=24):
    """Returns exactly n_pts evenly distributed along a closed NACA 00xx perimeter."""
    half = n_pts // 2
    upper, lower = [], []
    for i in range(half):
        x_frac = (1.0 - math.cos(math.pi * i / (half - 1))) / 2.0
        x = x_frac * chord
        yt = 5.0 * thickness_ratio * chord * (0.2969 * math.sqrt(max(0.0, x_frac))
                                              - 0.1260 * x_frac
                                              - 0.3516 * (x_frac**2)
                                              + 0.2843 * (x_frac**3)
                                              - 0.1015 * (x_frac**4))
        upper.append((x, yt))
    for i in range(1, half + 1):
        x_frac = (1.0 - math.cos(math.pi * i / half)) / 2.0
        x = (1.0 - x_frac) * chord
        yt = 5.0 * thickness_ratio * chord * (0.2969 * math.sqrt(max(0.0, 1.0 - x_frac))
                                              - 0.1260 * (1.0 - x_frac)
                                              - 0.3516 * ((1.0 - x_frac)**2)
                                              + 0.2843 * ((1.0 - x_frac)**3)
                                              - 0.1015 * ((1.0 - x_frac)**4))
        lower.append((x, -yt))
    res = upper + lower
    return res

def build_organic_finger_wingtip(is_port=True):
    base_pts = load_contour()
    N_base = len(base_pts)
    facets = []
    
    # ── 1. The Palm Base (Z = 0 to Z = 38mm) ────────────────────────
    H_palm = 38.0
    M_palms = 12
    palm_slices = []
    
    for s in range(M_palms + 1):
        u = s / float(M_palms)
        z = u * H_palm if is_port else -u * H_palm
        # Organic hand taper: slight chord taper (1.0 -> 0.92) and smooth thinning
        scale_x = 1.0 - 0.08 * u
        scale_y = 1.0 - 0.12 * u
        pts = [(p[0] * scale_x, p[1] * scale_y, z) for p in base_pts]
        palm_slices.append(pts)
        
    for s in range(M_palms):
        s0, s1 = palm_slices[s], palm_slices[s+1]
        for i in range(N_base):
            j = (i + 1) % N_base
            a, b, c, d = s0[i], s0[j], s1[j], s1[i]
            if is_port:
                facets.append((tri_normal(a, b, c), a, b, c))
                facets.append((tri_normal(a, c, d), a, c, d))
            else:
                facets.append((tri_normal(a, c, b), a, c, b))
                facets.append((tri_normal(a, d, c), a, d, c))
                
    # Root face at Z=0 with 3 spar bores
    root = palm_slices[0]
    rcx = sum(p[0] for p in root) / N_base
    rcy = sum(p[1] for p in root) / N_base
    rc = (rcx, rcy, 0.0)
    
    z_sign = 1.0 if is_port else -1.0
    bore_rings_top, bore_rings_bot, bore_centers = [], [], []
    for sx, sy, r, depth in SPAR_SOCKETS:
        rt, rb = [], []
        z_bot = depth * z_sign
        for k in range(SOCKET_SEGS):
            ang = 2.0 * math.pi * k / SOCKET_SEGS
            hx = sx + r * math.cos(ang)
            hy = sy + r * math.sin(ang)
            rt.append((hx, hy, 0.0))
            rb.append((hx, hy, z_bot))
        bore_rings_top.append(rt)
        bore_rings_bot.append(rb)
        bore_centers.append((sx, sy, r))
        
    def _contains_bore(ax, ay, bx, by, cx_, cy_):
        denom = (by - cy_) * (ax - cx_) + (cx_ - bx) * (ay - cy_)
        if abs(denom) < 1e-15:
            return False
        for bx_, by_, _ in bore_centers:
            l1 = ((by - cy_) * (bx_ - cx_) + (cx_ - bx) * (by_ - cy_)) / denom
            l2 = ((cy_ - ay) * (bx_ - cx_) + (ax - cx_) * (by_ - cy_)) / denom
            l3 = 1.0 - l1 - l2
            if l1 >= -1e-9 and l2 >= -1e-9 and l3 >= -1e-9:
                return True
        return False

    for i in range(N_base):
        j = (i + 1) % N_base
        if _contains_bore(rc[0], rc[1], root[i][0], root[i][1], root[j][0], root[j][1]):
            continue
        if is_port:
            facets.append((tri_normal(rc, root[j], root[i]), rc, root[j], root[i]))
        else:
            facets.append((tri_normal(rc, root[i], root[j]), rc, root[i], root[j]))
            
    for bi, (sx, sy, r, depth) in enumerate(SPAR_SOCKETS):
        rt, rb = bore_rings_top[bi], bore_rings_bot[bi]
        for k in range(SOCKET_SEGS):
            kn = (k + 1) % SOCKET_SEGS
            if is_port:
                facets.append((tri_normal(rc, rt[k], rt[kn]), rc, rt[k], rt[kn]))
                facets.append((tri_normal(rt[k], rb[kn], rt[kn]), rt[k], rb[kn], rt[kn]))
                facets.append((tri_normal(rt[k], rb[k], rb[kn]), rt[k], rb[k], rb[kn]))
            else:
                facets.append((tri_normal(rc, rt[kn], rt[k]), rc, rt[kn], rt[k]))
                facets.append((tri_normal(rt[k], rt[kn], rb[kn]), rt[k], rt[kn], rb[kn]))
                facets.append((tri_normal(rt[k], rb[kn], rb[k]), rt[k], rb[kn], rb[k]))
        z_bot = depth * z_sign
        bot_c = (sx, sy, z_bot)
        for k in range(SOCKET_SEGS):
            kn = (k + 1) % SOCKET_SEGS
            if is_port:
                facets.append((tri_normal(bot_c, rb[k], rb[kn]), bot_c, rb[k], rb[kn]))
            else:
                facets.append((tri_normal(bot_c, rb[kn], rb[k]), bot_c, rb[kn], rb[k]))

    # Knuckle cap at the end of the palm
    palm_end = palm_slices[-1]
    pcx = sum(p[0] for p in palm_end) / N_base
    pcy = sum(p[1] for p in palm_end) / N_base
    pcz = H_palm if is_port else -H_palm
    knuckle_c = (pcx, pcy, pcz)
    
    for i in range(N_base):
        j = (i + 1) % N_base
        if is_port:
            facets.append((tri_normal(knuckle_c, palm_end[i], palm_end[j]), knuckle_c, palm_end[i], palm_end[j]))
        else:
            facets.append((tri_normal(knuckle_c, palm_end[j], palm_end[i]), knuckle_c, palm_end[j], palm_end[i]))

    # ── 2. The 5 Fingers Extruding Directly From the Knuckles ────────
    # Each finger originates right on the palm's terminal chord contour,
    # radiating outward like fingers from a palm.
    finger_data = [
        # (name, x_root_center, root_chord, span_ext, sweep_deg, upward_curl_mm)
        ("Finger_1_LE",     12.0, 24.0, 140.0, 36.0, 65.0), # Thumb / leading finger
        ("Finger_2_Index",  38.0, 26.0, 130.0, 31.0, 52.0), # Index
        ("Finger_3_Middle", 68.0, 28.0, 120.0, 25.0, 40.0), # Middle
        ("Finger_4_Ring",  102.0, 26.0, 110.0, 19.0, 28.0), # Ring
        ("Finger_5_Pinky", 135.0, 22.0, 100.0, 14.0, 16.0), # Pinky / trailing finger
    ]
    
    N_pts = 24
    N_knuckle_slices = 8   # Extrusion blend slices directly growing out of palm face
    N_finger_slices = 22   # Finger shaft loft
    N_tip_slices = 8       # Hemispherical rounded fingertip dome
    
    palm_xs = [p[0] for p in palm_end]
    palm_ys = [p[1] for p in palm_end]
    
    for f_idx, (f_name, x_center, c_root, L_finger, sweep, z_curl) in enumerate(finger_data):
        nearby_ys = [palm_ys[k] for k in range(N_base) if abs(palm_xs[k] - x_center) < 18.0]
        y_center = sum(nearby_ys) / len(nearby_ys) if nearby_ys else 10.0
        
        finger_slices = []
        Z_origin = H_palm - 4.0  # 4mm inside the palm to ensure solid fusion
        Z_free = H_palm + 10.0
        
        # 1. Knuckle extrusion slices (growing seamlessly out of the palm)
        for s in range(N_knuckle_slices):
            t_k = s / float(N_knuckle_slices)
            z_curr = Z_origin + t_k * (Z_free - Z_origin)
            z = z_curr if is_port else -z_curr
            
            # Smooth flare at the knuckle base
            flare = 1.0 + 0.50 * math.cos(0.5 * math.pi * t_k)**2
            chord = c_root * flare
            thick = 0.16 * flare
            
            t_total = (z_curr - Z_origin) / L_finger
            dx = L_finger * math.tan(math.radians(sweep)) * (0.35 * t_total + 0.65 * t_total**2)
            dy = z_curl * (t_total ** 1.85)
            
            raw_sec = make_symmetrical_airfoil_ring(chord, thickness_ratio=thick, n_pts=N_pts)
            ring = []
            cx_pivot = chord * 0.50
            for px, py in raw_sec:
                gx = x_center + dx + (px - cx_pivot)
                gy = y_center + dy + py
                gz = z
                ring.append((gx, gy, gz))
            finger_slices.append(ring)
            
        # 2. Main Finger Shaft & 3. Rounded Tip Dome
        total_shaft = N_finger_slices + N_tip_slices
        for s in range(total_shaft + 1):
            if s <= N_finger_slices:
                t_shaft = (s / float(N_finger_slices)) * 0.90
                chord_f = 1.0 - 0.50 * (t_shaft / 0.90)
                thick_f = 1.0 - 0.20 * (t_shaft / 0.90)
            else:
                st = (s - N_finger_slices) / float(N_tip_slices)
                t_shaft = 0.90 + 0.10 * st
                dome = math.sqrt(max(0.001, 1.0 - st**2))
                chord_f = 0.50 * dome
                thick_f = 0.80 * dome

            chord = max(0.8, c_root * chord_f)
            thick = max(0.08, 0.13 * thick_f)
            
            z_curr = Z_free + t_shaft * (L_finger - (Z_free - Z_origin))
            z = z_curr if is_port else -z_curr
            
            t_total = (z_curr - Z_origin) / L_finger
            dx = L_finger * math.tan(math.radians(sweep)) * (0.35 * t_total + 0.65 * t_total**2)
            dy = z_curl * (t_total ** 1.85)
            
            raw_sec = make_symmetrical_airfoil_ring(chord, thickness_ratio=thick, n_pts=N_pts)
            ring = []
            cx_pivot = chord * 0.50
            for px, py in raw_sec:
                gx = x_center + dx + (px - cx_pivot)
                gy = y_center + dy + py
                gz = z
                ring.append((gx, gy, gz))
            finger_slices.append(ring)
            
        # Loft entire finger mesh
        n_ring_pts = len(finger_slices[0])
        for s in range(len(finger_slices) - 1):
            s0, s1 = finger_slices[s], finger_slices[s+1]
            for i in range(n_ring_pts):
                j = (i + 1) % n_ring_pts
                a, b, c, d = s0[i], s0[j], s1[j], s1[i]
                if is_port:
                    facets.append((tri_normal(a, b, c), a, b, c))
                    facets.append((tri_normal(a, c, d), a, c, d))
                else:
                    facets.append((tri_normal(a, c, b), a, c, b))
                    facets.append((tri_normal(a, d, c), a, d, c))
                    
        # Finger root base cap (embedded inside the palm body at Z=34mm)
        r_slice = finger_slices[0]
        rcx = sum(p[0] for p in r_slice) / n_ring_pts
        rcy = sum(p[1] for p in r_slice) / n_ring_pts
        rcz = r_slice[0][2]
        rc = (rcx, rcy, rcz)
        for i in range(n_ring_pts):
            j = (i + 1) % n_ring_pts
            if is_port:
                facets.append((tri_normal(rc, r_slice[j], r_slice[i]), rc, r_slice[j], r_slice[i]))
            else:
                facets.append((tri_normal(rc, r_slice[i], r_slice[j]), rc, r_slice[i], r_slice[j]))
                
        # Fingertip rounded dome apex cap
        t_slice = finger_slices[-1]
        tcx = sum(p[0] for p in t_slice) / n_ring_pts
        tcy = sum(p[1] for p in t_slice) / n_ring_pts
        z_apex = t_slice[0][2] + (1.2 if is_port else -1.2)
        tc = (tcx, tcy, z_apex)
        for i in range(n_ring_pts):
            j = (i + 1) % n_ring_pts
            if is_port:
                facets.append((tri_normal(tc, t_slice[i], t_slice[j]), tc, t_slice[i], t_slice[j]))
            else:
                facets.append((tri_normal(tc, t_slice[j], t_slice[i]), tc, t_slice[j], t_slice[i]))

    return facets

def main():
    os.makedirs(OUT_DIR, exist_ok=True)
    print("Generating Organic 'Fingers from a Hand' Splayed Wingtips...")
    
    port = build_organic_finger_wingtip(is_port=True)
    write_binary_stl(os.path.join(OUT_DIR, 's1223_bird_of_prey_fingers_port.stl'), port)
    
    stbd = build_organic_finger_wingtip(is_port=False)
    write_binary_stl(os.path.join(OUT_DIR, 's1223_bird_of_prey_fingers_starboard.stl'), stbd)
    
    print("\n[SUCCESS] Organic finger-feather STL models built successfully.")

if __name__ == '__main__':
    main()
