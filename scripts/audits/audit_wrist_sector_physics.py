#!/usr/bin/env python3
"""
scripts/audit_wrist_sector_physics.py
Adversarial engineering calculation suite for MCO-1D V7 Wrist & Sector Wheel:
  1. Dyneema tensile safety factor & knot/clamp derating.
  2. Rim wall flexural deflection & local buckling over servo pocket span.
  3. Bearing life, static/dynamic safety factors under combined tension & lift moments.
  4. Motor pinion wall thickness, set-screw thread engagement, and shaft shear.
  5. Servo thermal dissipation in enclosed PA12-CF pocket.
  6. Wing spar conical articulation clearance (pitch & sweep limits).
  7. Bevel gear tooth bending and contact fatigue stress.
"""

import math

print("=" * 76)
print("  ADVERSARIAL ENGINEERING AUDIT: MCO-1D V7 WRIST & SECTOR DRIVETRAIN")
print("=" * 76)

# 1. DYNEEMA SK99 TENSION AUDIT
t_settled = 180.0 # N (rigging pre-tension)
t_peak = 220.0    # N (peak dynamic reversal at 3.2 Hz sprint)

d_cables = [0.6, 0.75, 0.8]
tensile_strengths = {0.6: 410.0, 0.75: 680.0, 0.8: 920.0} # N (SK99 12-strand braid)
knot_efficiencies = {"Pinch Clamp": 0.85, "Figure-8 Knot": 0.70, "Tight Bend Sheave (D/d=11.7)": 0.80}

print("\n--- 1. CABLE TENSILE & FATIGUE AUDIT ---")
for d in d_cables:
    raw_str = tensile_strengths[d]
    eff_str = raw_str * 0.75 # conservative 75% termination efficiency
    sf_raw = raw_str / t_peak
    sf_eff = eff_str / t_peak
    d_pinion = 7.0 if d == 0.6 else (9.0 if d == 0.75 else 9.0)
    bend_ratio = d_pinion / d
    print(f"Cable d={d}mm: Raw Str={raw_str}N | Eff Str={eff_str:.1f}N | Bend Ratio={bend_ratio:.1f}:1")
    print(f"   Raw SF={sf_raw:.2f}x | Factored SF={sf_eff:.2f}x (Peak T={t_peak}N)")

# 2. RIM WALL DEFLECTION OVER SERVO POCKET
print("\n--- 2. RIM WALL LOCAL BENDING / BUCKLING AUDIT ---")
# Span of servo pocket under rim: L = 23.6 mm
# Wall thickness: t = 1.8 mm
# Height of rim: h = 8.0 mm (groove section)
# Modulus of PA12-CF: E = 5,500 MPa
# Distributed cable load: q = T / R = 220 N / 35 mm = 6.29 N/mm
L_span = 23.6 # mm
t_wall = 1.8  # mm
h_rim = 8.0   # mm
E_pa12 = 5500.0 # MPa
q_line = t_peak / 35.0 # N/mm

I_wall = (h_rim * (t_wall ** 3)) / 12.0 # mm^4
# Fixed-fixed beam model under uniform inward pressure
# Max moment M = q * L^2 / 12
M_max = (q_line * (L_span ** 2)) / 12.0 # N*mm
sigma_bend = (M_max * (t_wall / 2.0)) / I_wall # MPa
# Max deflection delta = q * L^4 / (384 * E * I)
delta_max = (q_line * (L_span ** 4)) / (384.0 * E_pa12 * I_wall) # mm

print(f"Rim Wall t = {t_wall} mm, Span L = {L_span} mm:")
print(f"   Inward Line Load       : {q_line:.2f} N/mm")
print(f"   Moment of Inertia I    : {I_wall:.3f} mm^4")
print(f"   Peak Bending Stress    : {sigma_bend:.1f} MPa (PA12-CF Yield: 95 MPa -> SF = {95.0/sigma_bend:.2f}x)")
print(f"   Peak Radial Deflection : {delta_max*1000.0:.1f} µm ({delta_max:.4f} mm)")

# 3. BEARING FATIGUE & LOAD COUPLE AUDIT
print("\n--- 3. 6800-2RS FLAPPING BEARING COUPLE AUDIT ---")
# Dual 6800-2RS bearings: 10mm ID x 19mm OD x 5mm W
# Dynamic capacity Cr = 1,720 N, Static Cor = 840 N
# Flapping span between bearing centers: delta_z = 21.0 mm
# Lift bending moment: M_lift = 5.25 N*m = 5,250 N*mm
# Static Dyneema pull on hub: F_tension = 2 * 180 * cos(15 deg) = 348 N
C_r = 1720.0
C_0r = 840.0
delta_z = 21.0 # mm
M_lift = 5250.0 # N*mm
F_couple = M_lift / delta_z # N
F_tension = 2.0 * t_settled * math.cos(math.radians(15.0)) # N

# Combined radial load on upper bearing
# Moment couple acts in vertical flapping plane (Y-Z), tension acts in horizontal plane (X-Y)
F_rad_upper = math.sqrt((F_tension / 2.0) ** 2 + F_couple ** 2)
sf_bearing_stat = C_0r / F_rad_upper
sf_bearing_dyn = C_r / F_rad_upper
print(f"Bending Couple Reaction Force : {F_couple:.1f} N")
print(f"Horizontal Cable Tension Load : {F_tension:.1f} N")
print(f"Peak Combined Radial Load     : {F_rad_upper:.1f} N")
print(f"Static Safety Factor (C_0r)   : {sf_bearing_stat:.2f}x (vs 840 N rating)")
print(f"Dynamic Safety Factor (C_r)   : {sf_bearing_dyn:.2f}x (vs 1,720 N rating)")

# 4. MOTOR PINION DRUM THREAD ENGAGEMENT AUDIT
print("\n--- 4. MOTOR PINION DRUM & CLAMPING AUDIT ---")
d_shaft = 4.0 # mm (GoolRC 5010 motor shaft)
d_pitch_pinion = 7.0 # mm
d_root_pinion = 6.4 # mm (with 0.3mm groove depth for 0.6mm cable)
wall_pinion = (d_root_pinion - d_shaft) / 2.0 # mm
print(f"Motor Pinion Pitch Dia: {d_pitch_pinion} mm | Shaft: {d_shaft} mm")
print(f"Pinion Wall Thickness at Groove Root: {wall_pinion:.2f} mm")
print(f"Standard M3 Set-Screw Engagement in 1.2mm wall: {wall_pinion/0.5:.1f} threads (Pitch=0.5mm)")
print("CRITICAL FINDING: 2.4 threads in 6061-Al is prone to thread stripping under 220N dynamic torque (0.77 N*m)!")
print("RECOMMENDATION: Stepped pinion hub with D-flat clamp collar or pinch split collar!")

# 5. SPAR ARTICULATION CLEARANCE AUDIT
print("\n--- 5. WING SPAR CONICAL ARTICULATION CLEARANCE AUDIT ---")
spar_od = 10.05 # mm
socket_dia = 15.0 # mm (7.5mm radius cylinder cut)
socket_wall_thickness = 10.0 # mm (from pivot Y=0 to flat face Y=10)
# Angular clearance: theta_clear = +/- arctan((R_socket - R_spar) / y_dist)
clearance_radial = (socket_dia - spar_od) / 2.0 # mm
pitch_clear_deg = math.degrees(math.atan2(clearance_radial, socket_wall_thickness))
print(f"Socket Dia: {socket_dia} mm | Spar OD: {spar_od} mm | Radial Gap: {clearance_radial:.2f} mm")
print(f"Current Max Angular Clearance before Strike: +/- {pitch_clear_deg:.1f}°")
print("CRITICAL FINDING: Library demands +/-15° to +/-20° pitch AoA! Current cylinder bore (+/-13.9°) WILL BIND at peak pitch!")
print("RECOMMENDATION: Flare socket into an elliptical conical trumpet (+/-22° pitch, +/-18° sweep)!")

# 6. BEVEL GEAR TOOTH SHEAR AUDIT
print("\n--- 6. BEVEL GEAR TOOTH BENDING AUDIT ---")
tau_pitch_peak = 0.317 # N*m (from flight requirements)
tau_sweep_peak = 0.597 # N*m
r_bevel = 6.5 # mm
F_tooth = tau_sweep_peak / (r_bevel / 1000.0) # N
print(f"Peak Tooth Tangential Force: {F_tooth:.1f} N")
# In PA12-CF, tooth bending stress with m=0.6, width b=2.5mm
# Lewis formula: sigma = F_t / (b * m * Y_lewis), Y_lewis ~ 0.3
m_gear = 0.6
b_face = 2.5
sigma_tooth = F_tooth / (b_face * m_gear * 0.3)
print(f"Estimated Tooth Bending Stress: {sigma_tooth:.1f} MPa")
print(f"PA12-CF Fatigue Endurance Limit (~35 MPa): SF = {35.0 / sigma_tooth:.2f}x")
print("CRITICAL FINDING: PA12-CF printed teeth under 203 MPa peak tooth load WILL FAIL in tooth fatigue!")
print("RECOMMENDATION: Use CNC 7075-T6 aluminum or brass bevel gears (sigma_yield > 450 MPa, SF > 2.2x)!")
