#!/usr/bin/env python3
"""
MCO-1D Hypothesis Authoring Tool
==================================
Helps pilots, engineers, and researchers create hypothesis JSON files
for injection into the MCO-1D optimizer — without programming knowledge.

Three modes:
  1. Interactive:  python3 mco_hypothesis_tool.py
  2. From description: python3 mco_hypothesis_tool.py --describe
  3. Quick inject:  python3 mco_hypothesis_tool.py --inject --host mco-1d.local

The tool validates parameters against the MCO-1D safety envelope before
writing the file, so invalid configurations are caught at the laptop
rather than rejected by the aircraft in flight.

PARAMETER REFERENCE (all optional — only specify what you want to change):
  freq_hz           Wing frequency (0.3 to 3.5 Hz)
  global_aoa_deg    Angle of attack, degrees (0 to 30)
  global_amp_deg    Stroke amplitude, degrees (0 to 96)
  cascade_phase_deg Phase offset between wing pairs, degrees (0 to 360)
                    0 = all wings in phase (maximum force)
                    180 = alternating pairs antiphase (maximum stability)
                    45 = maximum thrust coefficient (dragonfly finding)
  dihedral_deg      Wing elevation angle, degrees (0 = horizontal)
  sweep_deg         Wing sweep angle, degrees (30 = full forward, 100 = full aft)
  spanwise_wave_deg Phase gradient root-to-tip within each pair (0 to 45)
  aoa_advance_deg   How early AoA starts transitioning before stroke reversal
  amp_tip_scale     Tip amplitude relative to root (0.8 to 1.2)
  reversal_burst_frac Fraction of stroke where ODrive bursts at max speed
  split_bias_W1     Port/stbd amplitude asymmetry for pair 1 (-0.5 to 0.5)
  split_bias_W2     Port/stbd amplitude asymmetry for pair 2
"""

import json
import sys
import os
import argparse
import socket

# Safety limits (mirrors wave library constants)
PARAM_LIMITS = {
    'freq_hz':             (0.3,  3.5,   'Hz',  'Wing frequency'),
    'global_aoa_deg':      (0.0,  30.0,  'deg', 'Angle of attack'),
    'global_amp_deg':      (0.0,  96.0,  'deg', 'Stroke amplitude'),
    'cascade_phase_deg':   (0.0,  360.0, 'deg', 'Inter-pair phase offset'),
    'dihedral_deg':        (-10., 45.0,  'deg', 'Wing elevation'),
    'sweep_deg':           (30.0, 100.0, 'deg', 'Wing sweep angle'),
    'spanwise_wave_deg':   (0.0,  45.0,  'deg', 'Spanwise wave angle'),
    'aoa_advance_deg':     (0.0,  30.0,  'deg', 'AoA advance angle'),
    'amp_tip_scale':       (0.5,  1.5,   'x',   'Tip/root amplitude ratio'),
    'reversal_burst_frac': (0.0,  0.15,  'frac','Reversal burst fraction'),
    'split_bias_W1':       (-0.5, 0.5,   'frac','Pair 1 port/stbd bias'),
    'split_bias_W2':       (-0.5, 0.5,   'frac','Pair 2 port/stbd bias'),
}

LEVER_ZONES = {
    'takeoff':  (90, 100, 'Maximum power — takeoff and sprint'),
    'climb':    (75,  90, 'Climb power'),
    'cruise':   (55,  70, 'Cruise — efficient level flight'),
    'trim':     (48,  55, 'Trim — minimal power level flight'),
    'glide':    (30,  48, 'Clean glide — descent with low power'),
    'approach': (15,  30, 'Approach power — controlled descent'),
    'hover':    (45,  55, 'Hover — minimum power altitude hold'),
}


def validate(params, lever_pct):
    errors = []
    if not (0 <= lever_pct <= 100):
        errors.append(f'lever_pct {lever_pct} must be 0-100')
    for k, v in params.items():
        if k in PARAM_LIMITS:
            lo, hi, unit, desc = PARAM_LIMITS[k]
            if not (lo <= v <= hi):
                errors.append(f'{k}={v} out of range [{lo}, {hi}] {unit}')
    return errors


def interactive_mode():
    """Step-by-step interactive hypothesis authoring."""
    print("\nMCO-1D Hypothesis Authoring Tool")
    print("=" * 40)
    print("Answer each question. Press Enter to skip optional fields.")
    print()

    # Required fields
    name   = input("Hypothesis name (e.g. 'Monarch butterfly glide'): ").strip()
    source = input("Source / inspiration (paper, observation, etc.): ").strip()

    print("\nLever position — where in the flight regime to test this?")
    print("Common positions:")
    for zone, (lo, hi, desc) in LEVER_ZONES.items():
        print(f"  {zone:<10} {lo}-{hi}%   {desc}")
    lever_str = input("\nLever percentage (0-100, or zone name): ").strip()
    try:
        lever_pct = float(lever_str)
    except ValueError:
        zone = lever_str.lower()
        if zone in LEVER_ZONES:
            lo, hi, _ = LEVER_ZONES[zone]
            lever_pct = (lo + hi) / 2.0
            print(f"  Using {lever_pct:.0f}% (centre of {zone} zone)")
        else:
            lever_pct = 50.0
            print(f"  Unrecognised zone, defaulting to 50% (trim)")

    print("\nParameters to set (press Enter to skip each):")
    print("Only specify values you want to change from defaults.")
    print()

    params = {}
    for param, (lo, hi, unit, desc) in PARAM_LIMITS.items():
        val_str = input(f"  {desc} ({param}, {lo}-{hi} {unit}): ").strip()
        if val_str:
            try:
                v = float(val_str)
                if lo <= v <= hi:
                    params[param] = round(v, 4)
                else:
                    print(f"    Out of range [{lo}, {hi}] — skipped")
            except ValueError:
                print(f"    Not a number — skipped")

    notes = input("\nNotes (optional — describe the idea): ").strip()

    print("\nSafety settings (press Enter for defaults):")
    min_h_str = input("  Minimum test altitude in metres (default 15): ").strip()
    min_h     = float(min_h_str) if min_h_str else 15.0

    max_t_str = input("  Maximum trials (1-5, default 3): ").strip()
    max_t     = int(max_t_str) if max_t_str else 3

    errors = validate(params, lever_pct)
    if errors:
        print(f"\nValidation errors:")
        for e in errors:
            print(f"  - {e}")
        print("Please correct and retry.")
        return None

    hyp = {
        'name':      name,
        'source':    source,
        'lever_pct': lever_pct,
        'params':    params,
        'notes':     notes,
        'safety':    {'min_h_m': max(15.0, min_h), 'max_trials': max_t},
    }

    print(f"\nHypothesis created:")
    print(json.dumps(hyp, indent=2))
    return hyp


def quick_mode(name, source, lever, params_str, notes=''):
    """
    Create hypothesis from command-line arguments.
    params_str: comma-separated key=value pairs
    e.g. "freq_hz=2.0,cascade_phase_deg=45,global_aoa_deg=28"
    """
    params = {}
    if params_str:
        for pair in params_str.split(','):
            if '=' in pair:
                k, v = pair.split('=', 1)
                try:
                    params[k.strip()] = float(v.strip())
                except ValueError:
                    pass

    errors = validate(params, lever)
    if errors:
        print("Validation errors:")
        for e in errors:
            print(f"  {e}")
        sys.exit(1)

    return {
        'name':      name,
        'source':    source,
        'lever_pct': lever,
        'params':    params,
        'notes':     notes,
        'safety':    {'min_h_m': 15.0, 'max_trials': 3},
    }


def send_to_aircraft(hyp, host='mco-1d.local', port=5757):
    """Send hypothesis directly to aircraft via ground station socket."""
    try:
        s = socket.create_connection((host, port), timeout=5)
        cmd = f'inject_hypothesis {json.dumps(hyp)}'
        msg = json.dumps({'type': 'cmd', 'cmd': cmd}) + '\n'
        s.sendall(msg.encode())
        # Read response
        response = s.recv(4096).decode()
        s.close()
        return True, response
    except Exception as e:
        return False, str(e)


def save_to_file(hyp, output_dir='.'):
    """Save hypothesis to a numbered JSON file."""
    os.makedirs(output_dir, exist_ok=True)
    # Find next number
    existing = [f for f in os.listdir(output_dir)
                if f.startswith('hypothesis_') and f.endswith('.json')]
    n = len(existing) + 1
    filename = f'hypothesis_{n:03d}.json'
    filepath = os.path.join(output_dir, filename)
    with open(filepath, 'w') as f:
        json.dump(hyp, f, indent=2)
    print(f"\nSaved: {filepath}")
    print("To inject: copy to USB stick root, connect to aircraft,")
    print("  then run: load_hypotheses_usb  (ground station command)")
    print("Or direct inject: python3 mco_hypothesis_tool.py --inject "
          f"--file {filepath} --host mco-1d.local")
    return filepath


def main():
    parser = argparse.ArgumentParser(
        description='MCO-1D Hypothesis Authoring Tool')
    parser.add_argument('--interactive', action='store_true',
                        help='Interactive step-by-step mode (default)')
    parser.add_argument('--name',    default=None)
    parser.add_argument('--source',  default=None)
    parser.add_argument('--lever',   type=float, default=50.0)
    parser.add_argument('--params',  default='',
                        help='key=val,key=val parameter string')
    parser.add_argument('--notes',   default='')
    parser.add_argument('--inject',  action='store_true',
                        help='Send directly to aircraft')
    parser.add_argument('--file',    default=None,
                        help='Load hypothesis from file and inject')
    parser.add_argument('--host',    default='mco-1d.local')
    parser.add_argument('--output',  default='.',
                        help='Output directory for saved files')
    parser.add_argument('--list',    action='store_true',
                        help='List saved hypothesis files')
    args = parser.parse_args()

    if args.list:
        files = sorted(f for f in os.listdir(args.output)
                       if f.startswith('hypothesis_') and
                       f.endswith('.json'))
        if files:
            print(f"Hypothesis files in {args.output}:")
            for f in files:
                try:
                    with open(os.path.join(args.output, f)) as fh:
                        h = json.load(fh)
                    print(f"  {f}: {h.get('name','?')} "
                          f"(lever {h.get('lever_pct','?')}%)")
                except Exception:
                    print(f"  {f}: (unreadable)")
        else:
            print(f"No hypothesis files in {args.output}")
        return

    if args.file and args.inject:
        with open(args.file) as f:
            hyp = json.load(f)
        ok, resp = send_to_aircraft(hyp, args.host)
        print(f"Inject {'OK' if ok else 'FAILED'}: {resp}")
        return

    if args.name:
        hyp = quick_mode(args.name, args.source or 'command line',
                         args.lever, args.params, args.notes)
    else:
        hyp = interactive_mode()

    if hyp is None:
        sys.exit(1)

    if args.inject:
        ok, resp = send_to_aircraft(hyp, args.host)
        print(f"Inject {'OK' if ok else 'FAILED'}: {resp}")
    else:
        save_to_file(hyp, args.output)


if __name__ == '__main__':
    main()


# ── Novel MCO-1D hypotheses (H-28 to H-32) ──────────────────────
# Added after 3-DOF per-wing sensing enabled these new experiments
MCO1D_NOVEL_HYPOTHESES = {
    'H-28': {
        'name': 'Port-stbd asymmetric cascade',
        'description': (
            'Two independent cascade phases: one for port wings (W1p-W4p), '
            'one for starboard (W1s-W4s). Enabled by 8 independent motors. '
            'Tests whether differential cascade timing creates yaw authority '
            'or differential wake capture efficiency.'),
        'variants': 6,
        'sensing_required': ['stroke'],
        'status': 'registered',
    },
    'H-29': {
        'name': 'Resonance-seeking per wing',
        'description': (
            'Each wing seeks its own mechanical resonant frequency. '
            'Manufacturing variation means each assembly resonates slightly '
            'differently. Near resonance: less motor energy for same amplitude.'),
        'variants': 4,
        'sensing_required': ['stroke', 'mechanical_power'],
        'status': 'registered',
    },
    'H-30': {
        'name': 'Power-smoothing cascade',
        'description': (
            'Distribute stroke reversals evenly across 8 wings (45° apart). '
            'Decelerating motors regenerate while accelerating motors consume. '
            'Minimises bus voltage ripple and battery current peaks.'),
        'variants': 3,
        'sensing_required': ['stroke', 'mechanical_power'],
        'status': 'registered',
    },
    'H-31': {
        'name': 'Arbitrary stroke path (Fourier series)',
        'description': (
            'Extension of H-16 sea turtle figure-eight. H-16 fixed the path '
            'as an ellipse. H-31 uses Fourier series to parameterise any 2D path '
            'in the stroke plane. Teardrop, figure-nine, D-shape, wide-bottom. '
            'Requires MCP3208 sweep feedback to verify path.'),
        'variants': 4,
        'sensing_required': ['sweep', 'stroke'],
        'status': 'registered',
    },
    'H-32': {
        'name': 'Multi-wake phase optimiser',
        'description': (
            'W3 and W4 receive wakes from multiple upstream pairs simultaneously. '
            'W3 sees W1+W2 wakes; W4 sees W1+W2+W3 wakes. '
            'Force waveform measurement detects wake arrivals as lift bumps. '
            'Optimises phase for simultaneous multi-wake capture. '
            'Beyond any published tandem-wing research.'),
        'variants': 2,
        'sensing_required': ['force_waveform', 'stroke'],
        'status': 'registered',
    },
}
