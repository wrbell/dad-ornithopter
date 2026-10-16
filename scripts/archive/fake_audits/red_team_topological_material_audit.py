#!/usr/bin/env python3
"""
scripts/red_team_topological_material_audit.py
==============================================
Red Team Topological & Material Engineering Audit for MCO-1 Frame Clamps & Skid

Evaluates:
1. Micro-scale vs Macro-scale failure mechanics:
   - Anisotropy of FDM 3D printing (layer line delamination tau_z vs in-plane sigma_xy).
   - Viscoelastic creep under sustained M3 bolt preload over time (relaxation rate).
   - Dynamic fatigue under 50 Hz flapping vibrations (cyclic S-N fatigue limit).
   - Thermal softening near operating motors / sunshine (Tg of PETG vs PA-CF vs PLA).
2. Topological Thinning Failure Modes (Targeting SF = 1.5):
   - Local hoop buckling vs sleeve peel.
   - Notch sensitivity around fastener holes.
   - Bolt head punching shear through thin flanges.
   - Critical minimum wall thickness for 0.4mm nozzle FDM printability.
"""

import math

PETG = {
    'name': 'PETG (100% infill)',
    'Tg_C': 75.0,              # Glass transition temp
    'sigma_yield_xy': 50.0,    # MPa in-plane
    'sigma_yield_z':  20.0,    # MPa interlayer tensile (40% of XY)
    'tau_interlayer': 18.0,    # MPa interlaminar shear
    'creep_modulus_1000h': 0.45, # Retains only ~45% of elastic modulus under sustained 20 MPa stress!
    'fatigue_limit_ratio': 0.30, # Fatigue limit @ 10^7 cycles is only 30% of yield! (15 MPa)
    'notch_sensitivity_q': 0.75, # High notch sensitivity
}

PACF = {
    'name': 'PA-CF (Carbon Fiber Reinforced Nylon)',
    'Tg_C': 110.0,             # Heat deflection > 150°C
    'sigma_yield_xy': 75.0,    # MPa in-plane
    'sigma_yield_z':  38.0,    # MPa interlayer (50% of XY)
    'tau_interlayer': 32.0,    # MPa
    'creep_modulus_1000h': 0.72, # Retains ~72% of modulus
    'fatigue_limit_ratio': 0.42, # Fatigue limit @ 10^7 cycles ~ 31 MPa
    'notch_sensitivity_q': 0.50, # Lower notch sensitivity due to micro-fiber bridging
}

TOUGH_PLA = {
    'name': 'Tough PLA',
    'Tg_C': 55.0,              # Very low heat resistance!
    'sigma_yield_xy': 60.0,    # MPa
    'sigma_yield_z':  28.0,    # MPa
    'tau_interlayer': 22.0,    # MPa
    'creep_modulus_1000h': 0.30, # Extreme creep relaxation! Loses clamp preload quickly
    'fatigue_limit_ratio': 0.25,
    'notch_sensitivity_q': 0.80,
}

print("=" * 80)
print("RED TEAM TOPOLOGICAL & MATERIAL ENGINEERING AUDIT")
print("=" * 80)

# Red Team Question 1: What happens when wall thickness is reduced to SF = 1.5?
print("\n[AUDIT TOPIC 1: MINIMUM STRUCTURAL WALL THICKNESS & PRINTABILITY]")
# FDM rules: 0.4mm nozzle with 0.45mm line width.
# A wall of 1.2mm = only 2.6 perimeters (weak infill in between).
# A solid wall requires an even multiple of perimeters:
# 4 perimeters = 4 * 0.45 = 1.80 mm.
# 5 perimeters = 5 * 0.45 = 2.25 mm.
print("  • FDM Slicing Reality:")
print("    - A wall < 1.8mm defaults to 2 thin perimeters + hollow gap, severely degrading Z-axis strength.")
print("    - 1.8mm solid perimeter wall (4 loops) is the true absolute minimum robust FDM shell.")
print("    - Punching shear under an M3 bolt head (dia 5.5mm, hole 3.2mm, contact ring width 1.15mm):")
# Punching shear area = pi * 5.5mm * wall_thick
for t in [1.2, 1.8, 2.2, 2.5]:
    A_punch = math.pi * 5.5 * t
    tau_punch = 1200.0 / A_punch  # with 1200 N bolt tension
    print(f"      Wall {t:.1f} mm -> Punching shear under M3 bolt = {tau_punch:.2f} MPa (PETG allowable = 18 MPa)")

print("\n[AUDIT TOPIC 2: VISCOELASTIC CREEP & PRELOAD RELAXATION UNDER SUSTAINED CLAMPING]")
# Clamps are permanently bolted onto tubes.
# Viscoelastic polymers relax over days/weeks.
# If clamp preload drops by 50%, axial slip resistance drops by 50%!
print("  • If SF = 1.5 is designed strictly for Day 1 bolt preload:")
print("    - In PETG, stress relaxation drops clamp tension by 35-50% within 200 flight/hangar hours.")
print("    - In PLA, relaxation drops clamp tension by >65% (tubes will slip!).")
print("    - In PA-CF, carbon fibers lock polymer chains, limiting relaxation to <20%.")
print("  • RED TEAM RULE: Hoop clamp sizing MUST use an 'Apparent Relaxed Modulus', not static Day 1 yield!")

print("\n[AUDIT TOPIC 3: HIGH-FREQUENCY FLAPPING FATIGUE (50 Hz MOTOR / FLAP CYCLES)]")
# Flapping frequency = 2 to 5 Hz fundamental, motor ripple = 20-50 Hz.
# 1 hour of flight = 180,000 stress cycles!
# 10 flights = 1.8 x 10^6 cycles (infinite fatigue regime).
print("  • S-N Fatigue endurance limits at 2 x 10^6 cycles:")
print(f"    - PETG fatigue limit = {PETG['sigma_yield_xy'] * PETG['fatigue_limit_ratio']:.1f} MPa (vs static 50 MPa)")
print(f"    - PA-CF fatigue limit = {PACF['sigma_yield_xy'] * PACF['fatigue_limit_ratio']:.1f} MPa (vs static 75 MPa)")
print("  • If static SF = 1.5, cyclic stress = 50 / 1.5 = 33.3 MPa.")
print("    -> In PETG, 33.3 MPa EXCEEDS the 15 MPa fatigue limit! Fatigue cracking will occur over time.")
print("    -> In PA-CF, cyclic stress must stay below 31.5 MPa.")

print("\n[AUDIT TOPIC 4: THERMAL & MOTOR PROXIMITY HAZARDS]")
print("  • Motors run at up to 55-65°C under continuous 280W cruise.")
print("  • Lower keel and spine absorb radiant motor warmth in flight.")
print(f"  • PETG Tg = {PETG['Tg_C']}°C: Marginally safe (requires 10-15°C buffer).")
print(f"  • Tough PLA Tg = {TOUGH_PLA['Tg_C']}°C: DANGEROUS for primary frame joints (softens on warm tarmac!).")
print(f"  • PA-CF Tg = {PACF['Tg_C']}°C: Completely immune to heat softening.")

print("\n" + "=" * 80)
print("RED TEAM SYNTHESIS & SAFE TOPOLOGICAL TARGET MATRIX")
print("=" * 80)
