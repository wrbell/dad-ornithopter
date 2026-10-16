#!/usr/bin/env python3
"""
scripts/test_v7_skeletonized_200deg.py
Tests:
  1. 200 degree sector arc (arc_deg = 200.0)
  2. Outer rim wall thinned to ~1.8mm
  3. Topological isogrid / truss skeletonization lightening pockets
"""

import math
import build123d as b3d

R_pitch = 35.0
R_flange = 37.5
arc_deg = 200.0
drum_height = 26.0
steps = 80

half_arc = math.radians(arc_deg / 2.0)
st_ang = -math.pi / 2.0 - half_arc # -190 deg
en_ang = -math.pi / 2.0 + half_arc # +10 deg
d_ang = (en_ang - st_ang) / steps

arc_pts = []
for s in range(steps + 1):
    ang = st_ang + s * d_ang
    arc_pts.append((R_flange * math.cos(ang), R_flange * math.sin(ang)))

flat_face_y = arc_pts[0][1] # Connect directly across endpoints!
print(f"Arc endpoints: ({arc_pts[0][0]:.1f}, {arc_pts[0][1]:.1f}) to ({arc_pts[-1][0]:.1f}, {arc_pts[-1][1]:.1f})")
print(f"Flat face Y: {flat_face_y:.1f} mm")

sector_poly_pts = arc_pts + [(arc_pts[0][0], flat_face_y)]

with b3d.BuildPart() as drum:
    with b3d.BuildSketch(b3d.Plane.XY):
        b3d.Polygon(sector_poly_pts, align=None)
    b3d.extrude(amount=drum_height)

bb = drum.part.bounding_box()
print("200° Drum bounds:")
print("X:", bb.min.X, bb.max.X)
print("Y:", bb.min.Y, bb.max.Y)
print("Z:", bb.min.Z, bb.max.Z)
print(f"Solid un-skeletonized volume: {drum.part.volume/1000.0:.2f} cm³")
