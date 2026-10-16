#!/usr/bin/env python3
"""
scripts/test_embedded_drum_face_spar.py
=======================================
Tests embedding the 2-DOF wrist joint directly inside the vertical face of the drum,
so the wing spar emerges directly out of the drum's mid-height (Z = 12mm),
nestled between the upper and lower 6800 bearings.
"""

import math
import build123d as b3d

R_pitch = 35.0
R_flange = 37.5
arc_deg = 150.0
drum_height = 24.0
z_pivot = 12.0 # Mid-height of the drum!

print(f"Testing Embedded Drum Face Spar at Z = {z_pivot} mm...")
# Verify clearances
h_bearing = 5.0 # 6800 bearing at bottom (0..5) and top (19..24)
clearance_window = (drum_height - h_bearing) - h_bearing
print(f"Vertical window between bearing seats: {clearance_window:.1f} mm (Z: {h_bearing} to {drum_height - h_bearing})")
spar_od = 10.05
collar_od = 13.6
print(f"Spar OD: {spar_od:.1f} mm, Collar OD: {collar_od:.1f} mm -> fits in {clearance_window:.1f} mm window with {(clearance_window - collar_od):.1f} mm vertical clearance!")

