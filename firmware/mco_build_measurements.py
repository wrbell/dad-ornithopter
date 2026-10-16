#!/usr/bin/env python3
"""
mco_build_measurements.py — Build Measurement Sheet for MCO-1D

This script guides you through measuring the physical aircraft, validates
every measurement against expected ranges, and writes the confirmed values
into mco_wave_library.py so every aerodynamic calculation uses real data.

Run BEFORE TC-01. Takes approximately 30 minutes with the aircraft in front of you.

Usage:
    python3 mco_build_measurements.py

Results are saved to:
    /home/pi/knowledge/build_measurements.json
    (also patches mco_wave_library.py in the same directory)
"""

import json
import os
import sys
import time
import math
import ast

RESULTS_FILE    = '/home/pi/knowledge/build_measurements.json'
WAVE_LIB        = os.path.join(os.path.dirname(__file__), 'mco_wave_library.py')

# Expected ranges for sanity checking (min, max, unit, description)
EXPECTED = {
    'chord_mm':        (130,  170,  'mm',  'Wing chord (leading to trailing edge)'),
    'span_W1_mm':      (2200, 2500, 'mm',  'W1 pair full span (port tip to stbd tip)'),
    'span_W2_mm':      (2200, 2500, 'mm',  'W2 pair full span'),
    'span_W3_mm':      (2200, 2500, 'mm',  'W3 pair full span'),
    'span_W4_mm':      (2200, 2500, 'mm',  'W4 pair full span'),
    'mtow_g':          (5000, 7500, 'g',   'All-up weight with battery (grams)'),
    'cg_from_nose_mm': (430,  510,  'mm',  'CG distance aft of fuselage nose'),
    'fuselage_length_mm': (950, 1050, 'mm', 'Total fuselage length nose to tail'),
    'w1_pos_from_nose_mm': (50, 150, 'mm', 'W1 pair pivot aft of nose'),
    'w4_pos_from_nose_mm': (800, 900, 'mm', 'W4 pair pivot aft of nose'),
    'sector_arc_deg':  (180,  210,  '°',   'Sector wheel arc (mechanical stop to stop)'),
    'gear_ratio':      (6.5,  7.5,  ':1',  'Motor turns per sector wheel turn'),
    'sweep_range_deg': (40,   175,  '°',   'Full sweep servo travel (min to max position)'),
    'sweep_neutral_deg': (50,  70,  '°',   'Sweep servo neutral position'),
    'sweep_stow_deg':  (150,  180,  '°',   'Maximum aft sweep (fuselage stow position)'),
    'motor_to_sck_mm': (0,    100,  'mm',  'W1 motor shaft to W1 sector wheel axle distance'),
}

SECTIONS = [
    ('WING GEOMETRY', [
        ('chord_mm',          'Measure chord on one wing (leading to trailing edge) in mm'),
        ('span_W1_mm',        'W1: full span tip-to-tip in mm (both wings extended)'),
        ('span_W2_mm',        'W2: full span tip-to-tip in mm'),
        ('span_W3_mm',        'W3: full span tip-to-tip in mm'),
        ('span_W4_mm',        'W4: full span tip-to-tip in mm'),
    ]),
    ('FUSELAGE GEOMETRY', [
        ('fuselage_length_mm', 'Fuselage: nose to tail in mm'),
        ('w1_pos_from_nose_mm', 'W1 pair pivot point: distance aft of nose in mm'),
        ('w4_pos_from_nose_mm', 'W4 pair pivot point: distance aft of nose in mm'),
    ]),
    ('MASS AND CG', [
        ('mtow_g',             'Weigh the complete aircraft with flight battery installed (grams)'),
        ('cg_from_nose_mm',    'Balance point (CG): distance aft of nose in mm\n'
                               '    (balance under fuselage between W2 and W3 pairs)'),
    ]),
    ('MECHANICAL DRIVE', [
        ('sector_arc_deg',     'Sector wheel arc in degrees (mechanical stop to stop).\n'
                               '    Rotate by hand and measure with protractor'),
        ('gear_ratio',         'Gear ratio (motor turns : sector wheel turns).\n'
                               '    Turn motor shaft slowly, count motor turns per one full sector arc.\n'
                               '    Enter as a decimal, e.g. 7.0'),
        ('motor_to_sck_mm',   'Distance from motor shaft centre to sector wheel axle centre in mm'),
    ]),
    ('SWEEP SERVO RANGE', [
        ('sweep_range_deg',    'Total sweep servo travel in degrees (full forward to full aft).\n'
                               '    Command max forward, command max aft, measure angle change with protractor'),
        ('sweep_neutral_deg',  'Sweep angle at software neutral position (60° commanded).\n'
                               '    Measure actual angle at that position'),
        ('sweep_stow_deg',     'Sweep angle at maximum aft position (170° commanded).\n'
                               '    Measure whether wing actually reaches near-fuselage position'),
    ]),
]


AIRFOIL_OPTIONS = {
    '1': ('S1223', 'S1223 High-Lift STOL (143.7mm chord, 110.0g panel, CL_max 1.82)'),
    '2': ('SD7062', 'SD7062 High-Speed Dash (165.0mm chord, 109.2g panel, L/D 14.8)'),
}

def ask_airfoil():
    print(f"\n  Select Installed Wing Airfoil Option:")
    print("    1: S1223 (High-Lift STOL - 143.7mm chord, 1.15m span, certified baseline)")
    print("    2: SD7062 (High-Speed Dash - 165.0mm chord, 1.15m span, MCO-1.5D option)")
    while True:
        choice = input("  Enter option (1 or 2, default 1): ").strip()
        if choice == '' or choice == '1':
            return 'S1223'
        elif choice == '2':
            return 'SD7062'
        elif choice.upper() in ['S1223', 'SD7062']:
            return choice.upper()
        print("  Invalid selection. Please enter 1 (S1223) or 2 (SD7062).")

def ask(key, description, expected_range):
    lo, hi, unit, _ = expected_range
    while True:
        print(f"\n  {description}")
        raw = input(f"  Enter value ({unit}, expected {lo}–{hi}): ").strip()
        try:
            val = float(raw)
        except ValueError:
            print(f"  Invalid: '{raw}'. Enter a number.")
            continue
        if val < lo * 0.5 or val > hi * 2.0:
            print(f"  WARNING: {val} {unit} is very far outside expected range {lo}–{hi}.")
            print(f"  This may indicate a measurement error.")
            confirm = input("  Are you sure? (yes/no): ").strip().lower()
            if confirm != 'yes':
                continue
        elif val < lo or val > hi:
            print(f"  Note: {val} {unit} is outside normal range {lo}–{hi}.")
            confirm = input("  Confirm this measurement? (yes/no): ").strip().lower()
            if confirm != 'yes':
                continue
        return val


def compute_derived(m):
    """Compute derived quantities from measurements."""
    d = {}

    # Semi-spans (half of full span = one wing)
    d['semi_span_W1_mm'] = m['span_W1_mm'] / 2.0
    d['semi_span_W2_mm'] = m['span_W2_mm'] / 2.0
    d['semi_span_W3_mm'] = m['span_W3_mm'] / 2.0
    d['semi_span_W4_mm'] = m['span_W4_mm'] / 2.0

    # Wing areas
    chord_m = m['chord_mm'] / 1000.0
    d['area_W1_m2'] = 2 * (m['span_W1_mm'] / 2000.0) * chord_m
    d['area_W2_m2'] = 2 * (m['span_W2_mm'] / 2000.0) * chord_m
    d['area_W3_m2'] = 2 * (m['span_W3_mm'] / 2000.0) * chord_m
    d['area_W4_m2'] = 2 * (m['span_W4_mm'] / 2000.0) * chord_m
    d['area_total_m2'] = (d['area_W1_m2'] + d['area_W2_m2'] +
                          d['area_W3_m2'] + d['area_W4_m2'])

    # MTOW in kg and N
    d['mtow_kg'] = m['mtow_g'] / 1000.0
    d['mtow_N']  = d['mtow_kg'] * 9.807

    # Wing loading
    d['wing_loading_kg_m2'] = d['mtow_kg'] / d['area_total_m2']

    # Stall speed (approximate, from CL_max ~1.4 at Re 50k)
    CL_max = 1.4
    rho    = 1.225
    d['vs_oge_ms'] = math.sqrt(2 * d['mtow_N'] / (rho * CL_max * d['area_total_m2']))

    # CG as fraction of fuselage length
    d['cg_fraction'] = m['cg_from_nose_mm'] / m['fuselage_length_mm']

    # Pair positions as fraction of fuselage
    span_positions = (m['w1_pos_from_nose_mm'] +
                      (m['w4_pos_from_nose_mm'] - m['w1_pos_from_nose_mm']) * 0.33,
                      m['w1_pos_from_nose_mm'] +
                      (m['w4_pos_from_nose_mm'] - m['w1_pos_from_nose_mm']) * 0.66)
    d['w2_pos_est_mm'] = round(span_positions[0], 1)
    d['w3_pos_est_mm'] = round(span_positions[1], 1)

    # Reynolds number estimate at hover
    mean_semi_span_m = sum([d['semi_span_W1_mm'], d['semi_span_W2_mm'],
                            d['semi_span_W3_mm'], d['semi_span_W4_mm']]) / 4000.0
    tip_vel_hover = 2.0 * math.pi * d['vs_oge_ms'] * mean_semi_span_m / m['chord_mm'] * 1000
    nu = 1.5e-5  # kinematic viscosity of air
    d['re_hover_est'] = int(chord_m * tip_vel_hover / nu)

    # Free lift threshold speed (3 stopped pairs, CL=1.2)
    CL_ld  = 1.2
    S3     = d['area_W1_m2'] + d['area_W2_m2'] + d['area_W3_m2']
    d['v_full_coverage_ms'] = math.sqrt(
        2 * d['mtow_N'] / (rho * CL_ld * S3))

    # Sector arc software limit (4° safety margin from mechanical stop)
    d['flap_amp_deg'] = m['sector_arc_deg'] - 4.0

    # Stow feasibility
    d['stow_feasible'] = m['sweep_stow_deg'] >= 80.0

    return d


def patch_wave_library(m, d):
    """
    Patch mco_wave_library.py with real measured values.
    Replaces placeholder constants with measured ones.
    """
    if not os.path.exists(WAVE_LIB):
        print(f"  Wave library not found at {WAVE_LIB}")
        return False

    src = open(WAVE_LIB).read()
    patches = [
        # Airfoil
        ("AIRFOIL_TYPE",
         f"# Wing airfoil profile — MEASURED {time.strftime('%Y-%m-%d')}",
         f"AIRFOIL_TYPE             = '{m.get('airfoil_type', 'S1223')}'"),
        # Chord
        (f"CHORD_MM",
         f"# Wing chord mm — MEASURED {time.strftime('%Y-%m-%d')}",
         f"CHORD_MM                  = {m['chord_mm']:.1f}"),
        # MTOW
        (f"MTOW_KG",
         f"# All-up weight kg — MEASURED {time.strftime('%Y-%m-%d')}",
         f"MTOW_KG                   = {d['mtow_kg']:.4f}"),
        # Stall speed
        (f"VS_OGE_MS",
         f"# Stall speed m/s (computed from measurements)",
         f"VS_OGE_MS                 = {d['vs_oge_ms']:.4f}"),
        # Flap amplitude
        (f"FLAP_AMP_DEG",
         f"# Flap amplitude deg (sector arc - 4° safety) — MEASURED",
         f"FLAP_AMP_DEG              = {d['flap_amp_deg']:.1f}"),
    ]

    changed = 0
    for key, comment, new_line in patches:
        # Find and replace the constant definition
        lines = src.split('\n')
        for i, line in enumerate(lines):
            stripped = line.strip()
            if stripped.startswith(key) and '=' in stripped and not stripped.startswith('#'):
                old_line = lines[i]
                lines[i] = new_line + f"   # {comment}"
                src = '\n'.join(lines)
                changed += 1
                print(f"  Patched: {key} → {new_line.split('=')[1].strip()}")
                break

    # Patch SEMI_SPANS_MM dict — more complex, find and replace dict
    semi_spans_new = (
        f"SEMI_SPANS_MM             = {{"
        f"1: {d['semi_span_W1_mm']:.0f}, "
        f"2: {d['semi_span_W2_mm']:.0f}, "
        f"3: {d['semi_span_W3_mm']:.0f}, "
        f"4: {d['semi_span_W4_mm']:.0f}"
        f"}}   # MEASURED {time.strftime('%Y-%m-%d')}"
    )
    for line in src.split('\n'):
        if 'SEMI_SPANS_MM' in line and '{' in line and '}' in line:
            src = src.replace(line, semi_spans_new)
            print(f"  Patched: SEMI_SPANS_MM → {semi_spans_new[28:70]}...")
            changed += 1
            break

    if changed > 0:
        # Syntax check before writing
        try:
            ast.parse(src)
        except SyntaxError as e:
            print(f"  Patch would create syntax error at line {e.lineno}: {e.msg}")
            print(f"  Wave library NOT patched — check manually")
            return False

        with open(WAVE_LIB, 'w') as f:
            f.write(src)
        print(f"  Wave library patched with {changed} updates ✓")
        return True
    else:
        print(f"  No matching constants found to patch.")
        print(f"  Update mco_wave_library.py manually with these values:")
        for key, _, new_line in patches:
            print(f"    {new_line}")
        return False


def print_summary(m, d):
    print(f"\n{'═'*60}")
    print(f"  BUILD MEASUREMENT SUMMARY")
    print(f"{'═'*60}")
    print(f"\n  WING GEOMETRY")
    print(f"    Chord:              {m['chord_mm']:.0f} mm")
    print(f"    Span W1:            {m['span_W1_mm']:.0f} mm  (semi: {d['semi_span_W1_mm']:.0f})")
    print(f"    Span W2:            {m['span_W2_mm']:.0f} mm  (semi: {d['semi_span_W2_mm']:.0f})")
    print(f"    Span W3:            {m['span_W3_mm']:.0f} mm  (semi: {d['semi_span_W3_mm']:.0f})")
    print(f"    Span W4:            {m['span_W4_mm']:.0f} mm  (semi: {d['semi_span_W4_mm']:.0f})")
    print(f"    Total wing area:    {d['area_total_m2']:.4f} m²")
    print(f"\n  MASS")
    print(f"    MTOW:               {d['mtow_kg']:.3f} kg  ({m['mtow_g']:.0f} g)")
    print(f"    Wing loading:       {d['wing_loading_kg_m2']:.2f} kg/m²")
    print(f"    Stall speed (est):  {d['vs_oge_ms']:.2f} m/s")
    print(f"\n  CG")
    print(f"    CG from nose:       {m['cg_from_nose_mm']:.0f} mm  "
          f"({d['cg_fraction']*100:.0f}% of fuselage)")
    print(f"\n  MECHANICAL")
    print(f"    Sector arc:         {m['sector_arc_deg']:.1f}°  (software limit: {d['flap_amp_deg']:.1f}°)")
    print(f"    Gear ratio:         {m['gear_ratio']:.2f}:1")
    print(f"    Sweep range:        {m['sweep_range_deg']:.1f}°")
    print(f"    Sweep neutral:      {m['sweep_neutral_deg']:.1f}°")
    print(f"    Stow position:      {m['sweep_stow_deg']:.1f}°  "
          f"({'feasible ✓' if d['stow_feasible'] else 'INSUFFICIENT — H-33 State C limited'})")
    print(f"\n  DERIVED PERFORMANCE ESTIMATES")
    print(f"    Re at hover (est):  {d['re_hover_est']:,}")
    print(f"    3-pair full coverage at: {d['v_full_coverage_ms']:.1f} m/s pitot")
    print(f"      (above this speed, 3 stopped pairs cover MTOW from fixed-wing lift)")
    print(f"{'═'*60}\n")


def main():
    print("\nMCO-1D BUILD MEASUREMENT SHEET")
    print("="*50)
    print("Measure the physical aircraft with a tape measure and kitchen scale.")
    print("All measurements go into the wave library and replace estimates.")
    print("Take your time — accuracy here affects every aerodynamic calculation.\n")

    measurements = {}
    airfoil_type = ask_airfoil()
    measurements['airfoil_type'] = airfoil_type

    for section_name, fields in SECTIONS:
        print(f"\n{'─'*50}")
        print(f"  {section_name}")
        print(f"{'─'*50}")
        for key, description in fields:
            measurements[key] = ask(key, description, EXPECTED[key])

    # Compute derived values
    derived = compute_derived(measurements)

    # Print summary
    print(f"\n  AIRFOIL INSTALLED: {airfoil_type}")
    print_summary(measurements, derived)

    # Save
    os.makedirs(os.path.dirname(RESULTS_FILE), exist_ok=True)
    output = {
        'airfoil': airfoil_type,
        'measured': measurements,
        'derived':  derived,
        'measured_at': time.strftime('%Y-%m-%dT%H:%M:%S'),
    }
    with open(RESULTS_FILE, 'w') as f:
        json.dump(output, f, indent=2)
    print(f"  Measurements saved to {RESULTS_FILE}")

    # Patch wave library
    print(f"\n  Patching mco_wave_library.py with measured values...")
    patch_wave_library(measurements, derived)

    print(f"\n  BUILD MEASUREMENT SHEET COMPLETE")
    print(f"  Re-run TC-01 setup to pick up the updated constants.")


if __name__ == '__main__':
    main()
