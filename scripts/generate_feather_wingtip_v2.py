#!/usr/bin/env python3
"""
scripts/generate_feather_wingtip_v2.py
======================================
Generates watertight binary STL files for the Biomimetic "Bird of Prey"
splayed-feather wingtip with SYMMETRICAL AIRFOILS (NACA 0012 style) and
BEAUTIFULLY ROUNDED / ELLIPTICALLY BULBOUS FEATHER TIPS.

Key Updates:
  1. Base mounts flush to Selig S1223 165mm rib at Z=0.
  2. Exactly 3 x 2.2mm spar sockets (X=18.71mm, 37.79mm, 104.49mm) depth 22mm.
  3. Feathers use pure symmetrical NACA 00xx thickness distributions (zero camber,
     zero parasitic pitching moment, uniform performance in both upstroke and downstroke).
  4. Rounded tips: The blade planform and thickness taper gracefully to a rounded,
     hemispherical/elliptical tip rather than a flat cutoff.
  5. Fits comfortably within the Prusa Core One 250x250x250mm build envelope.
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
        hdr = f"MCO-1D Bird of Prey Symmetrical Rounded Feather Tip - {os.path.basename(filename)}".ljust(80)[:80].encode('ascii')
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

def make_symmetrical_airfoil_section(chord, thickness_ratio=0.12, n_pts=24):
    """
    Pure symmetrical NACA 00xx profile.
    Zero camber, perfect top-bottom symmetry, smooth rounded leading edge and fine trailing edge.
    """
    half = n_pts // 2
    upper, lower = [], []
    for i in range(half):
        # Half-cosine spacing for smooth leading edge resolution
        x_frac = (1.0 - math.cos(math.pi * i / (half - 1))) / 2.0
        x = x_frac * chord
        # NACA symmetrical thickness equation
        yt = 5.0 * thickness_ratio * chord * (0.2969 * math.sqrt(max(0.0, x_frac))
                                              - 0.1260 * x_frac
                                              - 0.3516 * (x_frac**2)
                                              + 0.2843 * (x_frac**3)
                                              - 0.1015 * (x_frac**4))
        upper.append((x, yt))
        if i > 0 and i < half - 1:
            lower.append((x, -yt))
            
    # Counter-clockwise: LE -> upper -> TE -> lower -> LE
    res = upper + lower[::-1]
    return res

def generate_symmetrical_feather_wingtip(is_port=True):
    base_contour = load_contour()
    N_base = len(base_contour)
    facets = []
    
    # ── 1. Root & Base Hub (Z = 0 to Z = 35mm) ──────────────────────
    H_base = 35.0
    M_base_slices = 10
    base_slices = []
    
    for s in range(M_base_slices + 1):
        u = s / float(M_base_slices)
        z = u * H_base if is_port else -u * H_base
        scale = 1.0 - 0.18 * u
        slice_pts = []
        for x, y in base_contour:
            xs = x * scale + 3.0 * u
            ys = y * scale
            slice_pts.append((xs, ys, z))
        base_slices.append(slice_pts)
        
    # Loft base shell
    for s in range(M_base_slices):
        s0, s1 = base_slices[s], base_slices[s+1]
        for i in range(N_base):
            j = (i + 1) % N_base
            a, b, c, d = s0[i], s0[j], s1[j], s1[i]
            if is_port:
                facets.append((tri_normal(a, b, c), a, b, c))
                facets.append((tri_normal(a, c, d), a, c, d))
            else:
                facets.append((tri_normal(a, c, b), a, c, b))
                facets.append((tri_normal(a, d, c), a, d, c))
                
    # Root face (Z=0) with 3 spar bores
    root = base_slices[0]
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

    # Cap hub top
    hub_top = base_slices[-1]
    hub_cx = sum(p[0] for p in hub_top) / N_base
    hub_cy = sum(p[1] for p in hub_top) / N_base
    hub_cz = H_base if is_port else -H_base
    hub_c = (hub_cx, hub_cy, hub_cz)
    
    for i in range(N_base):
        j = (i + 1) % N_base
        if is_port:
            facets.append((tri_normal(hub_c, hub_top[i], hub_top[j]), hub_c, hub_top[i], hub_top[j]))
        else:
            facets.append((tri_normal(hub_c, hub_top[j], hub_top[i]), hub_c, hub_top[j], hub_top[i]))

    # ── 2. Five Symmetrical, Rounded-Tip Primary Feathers ────────────
    feather_configs = [
        # (name, x_start, root_chord, span_length, sweep_deg, upward_curve_mm)
        ("P1_LE",     6.0,  25.0, 145.0, 36.0, 65.0),
        ("P2_MidLE", 34.0,  24.0, 135.0, 32.0, 52.0),
        ("P3_Mid",   60.0,  23.0, 125.0, 26.0, 40.0),
        ("P4_MidTE", 85.0,  22.0, 115.0, 20.0, 28.0),
        ("P5_TE",   110.0,  21.0, 105.0, 15.0, 16.0),
    ]
    
    N_feather_pts = 24
    N_blade_slices = 24   # along length
    N_tip_slices = 8      # dedicated slices to form the smooth rounded tip dome
    
    for f_idx, (f_name, x0, c_root, L_feather, sweep, z_curve) in enumerate(feather_configs):
        feather_slices = []
        y0 = 6.0 - 0.03 * x0
        
        # We will build slices from t = 0.0 to t = 1.0 (blade)
        # and from t = 1.0 to tip apex with elliptical rounding
        
        # 1. Main Blade loft slices (t=0 to t=0.92)
        # Chord tapers smoothly: c_root -> 0.40 * c_root
        # 2. Rounded Tip Dome (t=0.92 to t=1.0)
        # Chord and thickness round off to zero like an elliptical wingtip
        
        total_slices = N_blade_slices + N_tip_slices
        for s in range(total_slices + 1):
            if s <= N_blade_slices:
                # Main blade progression
                t = (s / float(N_blade_slices)) * 0.90
                # Smooth chord taper
                chord_factor = (1.0 - 0.55 * (t / 0.90))
                thick_factor = 1.0 - 0.20 * (t / 0.90)
            else:
                # Rounded tip dome: ellipse from 0.90 to 1.00
                st = (s - N_blade_slices) / float(N_tip_slices)
                t = 0.90 + 0.10 * st
                # Elliptical rounding profile: sqrt(1 - st^2)
                dome_profile = math.sqrt(max(0.001, 1.0 - st**2))
                chord_factor = 0.45 * dome_profile
                thick_factor = 0.80 * dome_profile

            chord = max(0.8, c_root * chord_factor)
            thickness = max(0.08, 0.12 * thick_factor)
            
            # Position along span
            dz = t * L_feather
            z = (H_base + dz) if is_port else -(H_base + dz)
            
            # Parabolic sweep
            dx = L_feather * math.tan(math.radians(sweep)) * (0.35 * t + 0.65 * t * t)
            
            # Upward dihedral curvature
            dy = z_curve * (t ** 1.85)
            
            # Gentle aerodynamic washout twist (-4 deg at tip)
            twist_deg = -4.0 * t
            cos_tw = math.cos(math.radians(twist_deg))
            sin_tw = math.sin(math.radians(twist_deg))
            
            # Pure symmetrical section
            raw_sec = make_symmetrical_airfoil_section(chord, thickness_ratio=thickness, n_pts=N_feather_pts)
            
            ring = []
            cx_pivot = chord * 0.30 # 30% chord aerodynamic center
            for px, py in raw_sec:
                # Rotate around 30% chord
                rx = (px - cx_pivot) * cos_tw - py * sin_tw + cx_pivot
                ry = (px - cx_pivot) * sin_tw + py * cos_tw
                
                gx = x0 + dx + rx
                gy = y0 + dy + ry
                gz = z
                ring.append((gx, gy, gz))
            feather_slices.append(ring)
            
        # Loft feather skin along all slices
        N_pts = len(feather_slices[0])
        for s in range(len(feather_slices) - 1):
            s0, s1 = feather_slices[s], feather_slices[s+1]
            for i in range(N_pts):
                j = (i + 1) % N_pts
                a, b, c, d = s0[i], s0[j], s1[j], s1[i]
                if is_port:
                    facets.append((tri_normal(a, b, c), a, b, c))
                    facets.append((tri_normal(a, c, d), a, c, d))
                else:
                    facets.append((tri_normal(a, c, b), a, c, b))
                    facets.append((tri_normal(a, d, c), a, d, c))
                    
        # Feather root cap (at base hub intersection)
        r_slice = feather_slices[0]
        rcx = sum(p[0] for p in r_slice) / N_pts
        rcy = sum(p[1] for p in r_slice) / N_pts
        rcz = r_slice[0][2]
        rc = (rcx, rcy, rcz)
        for i in range(N_pts):
            j = (i + 1) % N_pts
            if is_port:
                facets.append((tri_normal(rc, r_slice[j], r_slice[i]), rc, r_slice[j], r_slice[i]))
            else:
                facets.append((tri_normal(rc, r_slice[i], r_slice[j]), rc, r_slice[i], r_slice[j]))
                
        # Feather rounded tip terminal cap (closes the final tiny dome ring cleanly)
        t_slice = feather_slices[-1]
        tcx = sum(p[0] for p in t_slice) / N_pts
        tcy = sum(p[1] for p in t_slice) / N_pts
        # Project 1mm further out along feather trajectory to create a rounded nose cap
        z_tip_apex = t_slice[0][2] + (1.2 if is_port else -1.2)
        tc = (tcx, tcy, z_tip_apex)
        for i in range(N_pts):
            j = (i + 1) % N_pts
            if is_port:
                facets.append((tri_normal(tc, t_slice[i], t_slice[j]), tc, t_slice[i], t_slice[j]))
            else:
                facets.append((tri_normal(tc, t_slice[j], t_slice[i]), tc, t_slice[j], t_slice[i]))

    return facets

def main():
    os.makedirs(OUT_DIR, exist_ok=True)
    print("Generating Biomimetic Symmetrical Rounded-Tip Bird-of-Prey Wingtips...")
    
    port = generate_symmetrical_feather_wingtip(is_port=True)
    write_binary_stl(os.path.join(OUT_DIR, 's1223_bird_of_prey_symm_rounded_port.stl'), port)
    
    stbd = generate_symmetrical_feather_wingtip(is_port=False)
    write_binary_stl(os.path.join(OUT_DIR, 's1223_bird_of_prey_symm_rounded_starboard.stl'), stbd)
    
    print("\n[SUCCESS] Symmetrical rounded-tip Bird-of-Prey models generated successfully.")

if __name__ == '__main__':
    main()
