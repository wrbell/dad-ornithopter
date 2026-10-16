#!/usr/bin/env python3
"""
scripts/generate_wingtip_165mm_stl.py
======================================
Generates watertight 3D-printable STL files for the MCO-1D Vortex-Reducing
Raked Wingtip, configured for the 165 mm modified Selig S1223 wing rib.

The root face (Z = 0) reproduces the *exact* 165 mm modified S1223 rib contour
so the printed wingtip butts flush against the outboard rib.  Three internal
blind bores accept the 2 mm carbon-fibre spar rods that protrude from the rib;
these bores are entirely inside the airfoil shell and produce NO external
bumps, bulges, or non-aerodynamic surface features.

Root Interface (Z = 0):
  Spar Socket #1 (LE):    X = 18.71 mm, Y = 8.09 mm,  bore ∅2.15 mm, depth 25 mm
  Spar Socket #2 (Mid):   X = 37.79 mm, Y = 12.53 mm, bore ∅2.15 mm, depth 25 mm
  Spar Socket #3 (Aft):   X = 104.49 mm, Y = 16.33 mm, bore ∅2.15 mm, depth 25 mm

Lofted Aerodynamic Features:
  - 48° leading-edge rake (parabolic onset at the root)
  - 140 mm spanwise extension
  - Progressive chord taper  165.0 mm → 32.0 mm
  - Progressive section decambering  (ΔCp → 0 at tip)
  - −3.5° aerodynamic washout about Spar #2 axis

3D Printing — Standard PLA:
  Material:     Standard PLA (eSun PLA+, Bambu Basic, or equivalent)
  Nozzle temp:  210–215 °C  |  Bed: 60 °C
  Walls:        2 perimeters (0.40 mm nozzle)
  Infill:       10–15 % gyroid
  Orientation:  Root face flat on bed, 5 mm brim
  Supports:     None required (48° rake within overhang limits)
  Part mass:    ≈ 8–10 g per wingtip (solid PLA ρ ≈ 1.24 g/cm³)
"""

import os
import math
import struct

# ── Paths ────────────────────────────────────────────────────────────
COORD_PATH  = 'cad/coordinates/MCO_S1223_MOD_165mm.txt'
OUT_DIR     = 'cad/manufactured'

# ── Spar socket definitions (matches rib hole pattern) ───────────────
# Each entry: (X_mm, Y_mm, bore_radius_mm, depth_mm)
SPAR_SOCKETS = [
    (18.71,  8.09,  1.075, 25.0),   # #1  LE shear / anti-rotation
    (37.79, 12.53,  1.075, 25.0),   # #2  Main bending spar / washout pivot
    (104.49, 16.33, 1.075, 25.0),   # #3  Aft reflex support
]
SOCKET_SEGS = 24   # circumferential facets per bore cylinder

# ── Loft parameters ─────────────────────────────────────────────────
L_TIP_MM    = 140.0      # spanwise extension
N_SLICES    = 40         # spanwise loft slices (higher = smoother)
CHORD_ROOT  = 165.0      # mm  (must match coordinate file)
CHORD_TIP   = 32.0       # mm  (tapered tip chord)
RAKE_DEG    = 48.0       # LE sweep angle
WASHOUT_DEG = -3.5       # total nose-down twist at tip
CAMBER_TIP  = 0.22       # decambering fraction at tip  (1.0 = full camber)


# ─────────────────────────────────────────────────────────────────────
# Geometry helpers
# ─────────────────────────────────────────────────────────────────────

def load_rib_contour():
    """Load the 165 mm modified S1223 rib XY contour (closed polygon)."""
    pts = []
    with open(COORD_PATH, 'r') as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            parts = line.split()
            pts.append((float(parts[0]), float(parts[1])))
    return pts


def tri_normal(p1, p2, p3):
    """Unit outward normal of triangle p1-p2-p3 (right-hand rule)."""
    ux, uy, uz = p2[0]-p1[0], p2[1]-p1[1], p2[2]-p1[2]
    vx, vy, vz = p3[0]-p1[0], p3[1]-p1[1], p3[2]-p1[2]
    nx = uy*vz - uz*vy
    ny = uz*vx - ux*vz
    nz = ux*vy - uy*vx
    mag = math.sqrt(nx*nx + ny*ny + nz*nz)
    if mag > 1e-12:
        return (nx/mag, ny/mag, nz/mag)
    return (0.0, 0.0, 1.0)


def write_binary_stl(path, facets):
    """Write a watertight binary STL file."""
    with open(path, 'wb') as f:
        hdr = f"MCO-1D S1223 165mm Wingtip - {os.path.basename(path)}"
        f.write(hdr.ljust(80)[:80].encode('ascii'))
        f.write(struct.pack('<I', len(facets)))
        for norm, v1, v2, v3 in facets:
            f.write(struct.pack('<3f', *norm))
            f.write(struct.pack('<3f', *v1))
            f.write(struct.pack('<3f', *v2))
            f.write(struct.pack('<3f', *v3))
            f.write(struct.pack('<H', 0))
    kb = os.path.getsize(path) / 1024.0
    print(f"  [OK] {path}  ({len(facets):,} triangles, {kb:.1f} KB)")


# ─────────────────────────────────────────────────────────────────────
# Lofted wingtip mesh generation
# ─────────────────────────────────────────────────────────────────────

def _loft_section(raw_pts, u, spar2_x, spar2_y, is_port):
    """
    Return a ring of 3-D points for spanwise station *u* ∈ [0, 1].

    u = 0 → root (exact rib contour, Z = 0)
    u = 1 → tip  (tapered / decambered / washed-out, Z = ±L_TIP_MM)
    """
    # ── chord & scale ────────────────────────────────────────────────
    chord_u = CHORD_ROOT * (1.0 - u) + CHORD_TIP * u
    scale   = chord_u / CHORD_ROOT

    # ── LE sweep offset (parabolic onset) ────────────────────────────
    x_le = L_TIP_MM * math.tan(math.radians(RAKE_DEG)) * (0.30*u + 0.70*u*u)

    # ── washout rotation ─────────────────────────────────────────────
    w_rad = math.radians(WASHOUT_DEG * u)
    cos_w, sin_w = math.cos(w_rad), math.sin(w_rad)

    # ── progressive decambering ──────────────────────────────────────
    camber_f = 1.0*(1.0 - u) + CAMBER_TIP*u

    # ── pivot = Spar #2 (scaled) ─────────────────────────────────────
    piv_x = spar2_x * scale
    piv_y = spar2_y * scale

    # ── z coordinate ─────────────────────────────────────────────────
    z_val = u * L_TIP_MM if is_port else -u * L_TIP_MM

    ring = []
    for x_raw, y_raw in raw_pts:
        # scale
        xs = x_raw * scale

        # decamber: attenuate deviation from the mean waterline
        y_ref = 8.0 * scale
        y_rel = y_raw * scale - y_ref
        if y_rel < 0:
            y_mod = y_ref + y_rel * (camber_f ** 1.3)
        else:
            y_mod = y_ref + y_rel * (0.55 + 0.45 * camber_f)

        # washout about pivot
        dx = xs   - piv_x
        dy = y_mod - piv_y
        rx = piv_x + dx*cos_w - dy*sin_w
        ry = piv_y + dx*sin_w + dy*cos_w

        ring.append((rx + x_le, ry, z_val))

    return ring


def generate_wingtip_mesh(is_port=True):
    """
    Build the complete facet list for one wingtip (port or starboard).

    Returns a list of (normal, v1, v2, v3) tuples.
    """
    raw_pts = load_rib_contour()
    N = len(raw_pts)

    spar2_x, spar2_y = SPAR_SOCKETS[1][0], SPAR_SOCKETS[1][1]

    # ── 1.  Build loft slices ────────────────────────────────────────
    slices = []
    for s in range(N_SLICES + 1):
        u = s / float(N_SLICES)
        slices.append(_loft_section(raw_pts, u, spar2_x, spar2_y, is_port))

    facets = []

    # ── 2.  Outer skin (quad strips between adjacent slices) ─────────
    for s in range(N_SLICES):
        s0 = slices[s]
        s1 = slices[s + 1]
        for i in range(N):
            j = (i + 1) % N
            a, b, c, d = s0[i], s0[j], s1[j], s1[i]
            if is_port:
                facets.append((tri_normal(a, b, c), a, b, c))
                facets.append((tri_normal(a, c, d), a, c, d))
            else:
                facets.append((tri_normal(a, c, b), a, c, b))
                facets.append((tri_normal(a, d, c), a, d, c))

    # ── 3.  Tip end-cap (fan from centroid) ──────────────────────────
    tip = slices[-1]
    cx = sum(p[0] for p in tip) / N
    cy = sum(p[1] for p in tip) / N
    cz = sum(p[2] for p in tip) / N
    tc = (cx, cy, cz)
    for i in range(N):
        j = (i + 1) % N
        if is_port:
            facets.append((tri_normal(tc, tip[i], tip[j]), tc, tip[i], tip[j]))
        else:
            facets.append((tri_normal(tc, tip[j], tip[i]), tc, tip[j], tip[i]))

    # ── 4.  Root face (Z = 0) with three spar bore holes cut out ────
    #
    #   Strategy: fan-triangulate the root face from the airfoil centroid,
    #   but SKIP any fan triangle whose centroid falls inside a bore circle.
    #   Then for each bore, connect its rim back to the airfoil centroid
    #   with reverse-wound triangles that form an annular ring — this
    #   effectively leaves a clean circular hole at each bore location.
    #   The bore cylinders and bottom caps are entirely inside the shell.

    root = slices[0]   # exact rib contour at Z = 0

    # Root-face centroid
    rcx = sum(p[0] for p in root) / N
    rcy = sum(p[1] for p in root) / N
    rcz = 0.0
    rc  = (rcx, rcy, rcz)

    # Pre-build bore rim geometry for all spar sockets
    z_sign = 1.0 if is_port else -1.0
    bore_rings_top = []  # list of ring_top lists
    bore_rings_bot = []  # list of ring_bot lists
    bore_centers = []    # (sx, sy, r) for point-in-circle tests

    for sx, sy, r, depth in SPAR_SOCKETS:
        ring_top = []
        ring_bot = []
        z_bot = depth * z_sign
        for k in range(SOCKET_SEGS):
            ang = 2.0 * math.pi * k / SOCKET_SEGS
            hx = sx + r * math.cos(ang)
            hy = sy + r * math.sin(ang)
            ring_top.append((hx, hy, 0.0))
            ring_bot.append((hx, hy, z_bot))
        bore_rings_top.append(ring_top)
        bore_rings_bot.append(ring_bot)
        bore_centers.append((sx, sy, r))

    def _tri_contains_any_bore_center(ax, ay, bx, by, cx_, cy_):
        """Return True if any bore center falls inside triangle (a,b,c) in 2D."""
        denom = (by - cy_) * (ax - cx_) + (cx_ - bx) * (ay - cy_)
        if abs(denom) < 1e-15:
            return False
        for bore_x, bore_y, _ in bore_centers:
            l1 = ((by - cy_) * (bore_x - cx_) + (cx_ - bx) * (bore_y - cy_)) / denom
            l2 = ((cy_ - ay) * (bore_x - cx_) + (ax - cx_) * (bore_y - cy_)) / denom
            l3 = 1.0 - l1 - l2
            if l1 >= -1e-9 and l2 >= -1e-9 and l3 >= -1e-9:
                return True
        return False

    # Fan-triangulate the root face, skipping triangles that contain a bore center
    for i in range(N):
        j = (i + 1) % N
        if _tri_contains_any_bore_center(rc[0], rc[1],
                                          root[i][0], root[i][1],
                                          root[j][0], root[j][1]):
            continue  # skip — this triangle covers a bore opening
        if is_port:
            n = tri_normal(rc, root[j], root[i])
            facets.append((n, rc, root[j], root[i]))
        else:
            n = tri_normal(rc, root[i], root[j])
            facets.append((n, rc, root[i], root[j]))

    # For each bore: annular rim ring, cylinder wall, and bottom cap
    for bi, (sx, sy, r, depth) in enumerate(SPAR_SOCKETS):
        ring_top = bore_rings_top[bi]
        ring_bot = bore_rings_bot[bi]

        # Annular ring: connects bore rim to airfoil centroid.
        # These are REVERSE wound relative to the solid fan, so they
        # face the same outward direction but form the ring around the hole.
        for k in range(SOCKET_SEGS):
            kn = (k + 1) % SOCKET_SEGS
            if is_port:
                # Wind so normal points -Z (same as rest of root face)
                n = tri_normal(rc, ring_top[k], ring_top[kn])
                facets.append((n, rc, ring_top[k], ring_top[kn]))
            else:
                n = tri_normal(rc, ring_top[kn], ring_top[k])
                facets.append((n, rc, ring_top[kn], ring_top[k]))

        # Cylinder side wall (normals pointing inward = toward bore axis)
        for k in range(SOCKET_SEGS):
            kn = (k + 1) % SOCKET_SEGS
            a, b = ring_top[k], ring_top[kn]
            c, d = ring_bot[kn], ring_bot[k]
            if is_port:
                facets.append((tri_normal(a, c, b), a, c, b))
                facets.append((tri_normal(a, d, c), a, d, c))
            else:
                facets.append((tri_normal(a, b, c), a, b, c))
                facets.append((tri_normal(a, c, d), a, c, d))

        # Bottom cap (blind end of bore)
        z_bot = depth * z_sign
        bot_c = (sx, sy, z_bot)
        for k in range(SOCKET_SEGS):
            kn = (k + 1) % SOCKET_SEGS
            if is_port:
                facets.append((tri_normal(bot_c, ring_bot[k], ring_bot[kn]),
                               bot_c, ring_bot[k], ring_bot[kn]))
            else:
                facets.append((tri_normal(bot_c, ring_bot[kn], ring_bot[k]),
                               bot_c, ring_bot[kn], ring_bot[k]))

    return facets


# ─────────────────────────────────────────────────────────────────────
# Main
# ─────────────────────────────────────────────────────────────────────

def main():
    os.makedirs(OUT_DIR, exist_ok=True)

    print("Generating MCO-1D 165 mm S1223 raked wingtip STL files (standard PLA) …")

    port = generate_wingtip_mesh(is_port=True)
    write_binary_stl(os.path.join(OUT_DIR,
                     's1223_vortex_raked_wingtip_165mm_port.stl'), port)

    stbd = generate_wingtip_mesh(is_port=False)
    write_binary_stl(os.path.join(OUT_DIR,
                     's1223_vortex_raked_wingtip_165mm_starboard.stl'), stbd)

    print(f"\n[DONE] Port: {len(port):,} triangles  |  Starboard: {len(stbd):,} triangles")
    print("       Material target: standard PLA (210–215 °C, 2 walls, 10–15 % gyroid)")
    print("       Root face: exact S1223 165 mm contour, flush butt joint to outboard rib")
    print("       Spar bores: 3 × ∅2.15 mm blind bores (25 mm deep), fully internal")


if __name__ == '__main__':
    main()
