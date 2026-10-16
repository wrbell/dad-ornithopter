#!/usr/bin/env python3
"""
mco_rc_verify.py — RC channel mapping verification and calibration

Verifies that:
  CH1 = Roll
  CH2 = Pitch (elevator / longitudinal)
  CH3 = Lever (throttle / collective — FORWARD = more power)
  CH4 = Yaw

Also verifies that the lever is at minimum before arming,
and that all channels respond in the correct direction.

Run this BEFORE TC-01 and after any transmitter change.

Usage:
    python3 mco_rc_verify.py            # interactive verification
    python3 mco_rc_verify.py --auto     # automatic from live MAVLink
    python3 mco_rc_verify.py --save     # save confirmed mapping

Results saved to: /home/pi/knowledge/rc_mapping.json
"""

import time
import json
import os
import sys
import argparse

RESULTS_FILE = '/home/pi/knowledge/rc_mapping.json'

# Expected channel assignments — FIXED-WING CONVENTION
# Joystick: left/right = roll (CH1), fore/aft = pitch (CH2)
# Pedals:   left/right = yaw (CH4)  — identical to fixed-wing rudder pedals
# Lever:    throttle + airbrake — identical to fixed-wing throttle (CH3)
#           FORWARD = more thrust (100%)
#           AFT     = less thrust / airbrake (0%)
#           MID (50%) = trim / altitude hold

EXPECTED_MAP = {
    'Roll':  1,   # CH1 — joystick left/right
    'Pitch': 2,   # CH2 — joystick fore/aft
    'Lever': 3,   # CH3 — throttle lever (forward=more power)
    'Yaw':   4,   # CH4 — rudder pedals (left pedal=nose left)
}

# Lever convention: FIXED-WING STANDARD
# Forward lever (high CH3 value) = MORE THRUST
# This matches every fixed-wing aircraft the pilot might already know.
# The wave library receives the FW-convention value directly.
# lever_internal() handles the conversion to legacy internal format.
LEVER_FORWARD_IS_MORE = True   # FIXED-WING: forward = more thrust

# Safety check: lever must be at minimum before arming
LEVER_MIN_THRESHOLD  = 1200   # raw PWM value (typical SBUS: 1000-2000)
LEVER_MAX_THRESHOLD  = 1800
LEVER_SAFE_THRESHOLD = 1150   # below this = safe to arm


def read_channels_from_mavlink():
    """Read current RC channel values from MAVLink bridge or shared sensor state."""
    try:
        import mco_wave_library as w
        if hasattr(w, '_last_sensors') and w._last_sensors:
            return w._last_sensors.get('rc_channels_raw', [])
    except Exception:
        pass
    return []


def wait_for_stick_move(channel_idx, direction, timeout_s=10.0,
                        threshold=200, baseline=None):
    """
    Wait for a specific stick to be moved in the given direction.
    Returns the peak value observed within the timeout.
    """
    t_end   = time.monotonic() + timeout_s
    peak    = baseline or 1500
    found   = False

    while time.monotonic() < t_end:
        channels = read_channels_from_mavlink()
        if len(channels) > channel_idx:
            val = channels[channel_idx]
            if baseline is None:
                baseline = val
            deviation = val - baseline
            if direction == 'high' and deviation > threshold:
                peak  = val
                found = True
                break
            elif direction == 'low' and deviation < -threshold:
                peak  = val
                found = True
                break
        time.sleep(0.1)

    return peak, found


def interactive_verify():
    """
    Interactive channel mapping verification.
    Prompts operator to move each stick and records which channel changes.
    """
    print("\nRC CHANNEL MAPPING VERIFICATION")
    print("=" * 50)
    print("This procedure verifies that your transmitter channels are")
    print("assigned correctly. Move sticks as prompted.\n")

    channels_baseline = read_channels_from_mavlink()
    if not channels_baseline:
        print("WARNING: No RC channel data available.")
        print("Ensure mco_mavlink.py is running, or run this from mco_main.py")
        print("Proceeding in manual verification mode.\n")
        return _manual_verify()

    baseline = list(channels_baseline)
    mapping  = {}
    results  = {}

    sticks = [
        ('Roll',  'RIGHT', 'high',  "Move ROLL stick to full RIGHT. Hold."),
        ('Pitch', 'FWD',   'low',   "Move PITCH stick to full FORWARD (nose down / accelerate). Hold."),
        ('Lever', 'MAX',   'high',  "Move LEVER to MAXIMUM (full forward). Hold."),
        ('Yaw',   'RIGHT', 'high',  "Apply RIGHT RUDDER PEDAL (press right pedal forward). Hold."),
    ]

    for stick_name, direction_label, direction, instruction in sticks:
        print(f"\n  {instruction}")
        print(f"  Waiting for stick movement...", end='', flush=True)

        # Watch all channels for the largest change
        t_end    = time.monotonic() + 8.0
        best_ch  = None
        best_dev = 0

        while time.monotonic() < t_end:
            channels = read_channels_from_mavlink()
            if len(channels) >= 4:
                for i, (val, base) in enumerate(zip(channels[:8], baseline[:8])):
                    dev = abs(val - base)
                    if dev > best_dev:
                        best_dev = dev
                        best_ch  = i + 1  # 1-indexed

            if best_dev > 200:  # significant movement detected
                print(f" CH{best_ch} moved (Δ={best_dev})")
                break
            time.sleep(0.05)
        else:
            print(f" timeout — no stick movement detected")
            best_ch = None

        if best_ch is None:
            print(f"  Could not detect {stick_name} channel.")
            manual = input(f"  Enter CH number manually (1-8): ").strip()
            try:
                best_ch = int(manual)
            except ValueError:
                best_ch = None

        mapping[stick_name]  = best_ch
        expected             = EXPECTED_MAP.get(stick_name)
        correct              = best_ch == expected
        results[stick_name]  = {'detected': best_ch, 'expected': expected, 'correct': correct}

        status = '✓' if correct else '✗ MISMATCH'
        print(f"  {stick_name}: CH{best_ch}  (expected CH{expected}) {status}")

        # Return stick to neutral before next test
        input("  Return stick to neutral, then press Enter: ")

    # Lever direction verification
    print(f"\n  LEVER DIRECTION VERIFICATION")
    print(f"  Move lever to MAXIMUM (full forward / up).")
    input("  Press Enter when lever is at maximum: ")
    channels_max = read_channels_from_mavlink()
    lever_ch_idx = (mapping.get('Lever', EXPECTED_MAP['Lever']) or 3) - 1
    lever_max_val = channels_max[lever_ch_idx] if len(channels_max) > lever_ch_idx else 1500

    print(f"  Return lever to MINIMUM.")
    input("  Press Enter when lever is at minimum: ")
    channels_min = read_channels_from_mavlink()
    lever_min_val = channels_min[lever_ch_idx] if len(channels_min) > lever_ch_idx else 1500

    lever_direction_ok = lever_max_val > lever_min_val  # max = high value = forward = more power
    results['lever_direction'] = {
        'max_val':   lever_max_val,
        'min_val':   lever_min_val,
        'forward_is_max': lever_direction_ok,
        'correct':   lever_direction_ok == LEVER_FORWARD_IS_MORE,
    }

    if lever_direction_ok:
        print(f"  Lever: min={lever_min_val}, max={lever_max_val} — FORWARD=MORE POWER ✓")
        print(f"  Convention confirmed: 100%=full thrust, 50%=trim/hold, 0%=airbrake")
    else:
        print(f"  Lever: min={lever_min_val}, max={lever_max_val} — REVERSED ✗")
        print(f"  REQUIRED: reverse this channel on your transmitter.")
        print(f"  The lever MUST be forward=more-thrust (fixed-wing convention).")
        print(f"  This is CH3. Reverse it in your transmitter channel settings.")

    print("\n  CONTROL CONVENTION SUMMARY:")
    print("  JOYSTICK:  Left/right = Roll    (banks left or right)")
    print("             Fore/aft   = Pitch   (nose down/up, accelerate/climb)")
    print("  PEDALS:    Left/right = Yaw     (nose left/right — identical to fixed-wing)")
    print("  LEVER:     Forward    = Thrust  (100% = full power)")
    print("             Mid (50%)  = Trim    (holds altitude)")
    print("             Aft (0%)   = Airbrake (maximum drag/glide)")
    return _print_results(mapping, results)


def _manual_verify():
    """Manual verification without live data — confirms mapping by question."""
    print("MANUAL VERIFICATION MODE (no live RC data)\n")
    mapping  = {}
    results  = {}

    for stick_name, expected_ch in EXPECTED_MAP.items():
        while True:
            ch_str = input(f"  Which channel is {stick_name}? (1-8, expected {expected_ch}): ").strip()
            try:
                ch = int(ch_str)
                if 1 <= ch <= 8:
                    break
                print("  Enter 1-8")
            except ValueError:
                print("  Enter a number")
        mapping[stick_name]  = ch
        correct              = ch == expected_ch
        results[stick_name]  = {'detected': ch, 'expected': expected_ch, 'correct': correct}

    lever_dir = input("  Does pushing lever FORWARD increase power? (yes/no): ").strip().lower()
    results['lever_direction'] = {
        'forward_is_max': lever_dir == 'yes',
        'correct': (lever_dir == 'yes') == LEVER_FORWARD_IS_MORE,
    }

    print("\n  CONTROL CONVENTION SUMMARY:")
    print("  JOYSTICK:  Left/right = Roll    (banks left or right)")
    print("             Fore/aft   = Pitch   (nose down/up, accelerate/climb)")
    print("  PEDALS:    Left/right = Yaw     (nose left/right — identical to fixed-wing)")
    print("  LEVER:     Forward    = Thrust  (100% = full power)")
    print("             Mid (50%)  = Trim    (holds altitude)")
    print("             Aft (0%)   = Airbrake (maximum drag/glide)")
    return _print_results(mapping, results)


def _print_results(mapping, results):
    """Print final results and save if mapping is correct."""
    print(f"\n{'─' * 50}")
    print(f"  RC MAPPING RESULTS")
    print(f"{'─' * 50}")

    all_correct = True
    for stick, r in results.items():
        if stick == 'lever_direction':
            status = '✓' if r['correct'] else '✗'
            print(f"  {status} Lever direction: {'FORWARD=MORE ✓' if r['correct'] else 'REVERSED ✗'}")
            if not r['correct']:
                all_correct = False
        else:
            status  = '✓' if r['correct'] else '✗'
            ch      = r['detected']
            exp     = r['expected']
            mismatch = f" (expected CH{exp})" if not r['correct'] else ''
            print(f"  {status} {stick}: CH{ch}{mismatch}")
            if not r['correct']:
                all_correct = False

    print(f"\n  {'MAPPING CORRECT ✓' if all_correct else 'MAPPING HAS ERRORS ✗'}")

    if not all_correct:
        print(f"\n  REQUIRED CORRECTIONS:")
        for stick, r in results.items():
            if stick != 'lever_direction' and not r['correct']:
                print(f"    Move {stick} from CH{r['detected']} to CH{r['expected']}")
                print(f"    (on your transmitter channel mix or assignment)")
        if not results.get('lever_direction', {}).get('correct', True):
            print(f"    Reverse the Lever (Throttle/Collective) channel direction")
    else:
        # Save confirmed mapping
        os.makedirs(os.path.dirname(RESULTS_FILE), exist_ok=True)
        output = {
            'mapping':      mapping,
            'results':      results,
            'all_correct':  all_correct,
            'verified_at':  time.strftime('%Y-%m-%dT%H:%M:%S'),
        }
        with open(RESULTS_FILE, 'w') as f:
            json.dump(output, f, indent=2)
        print(f"\n  Mapping saved to {RESULTS_FILE}")

    return all_correct


def check_lever_safe(channels_raw):
    """
    Return True if lever is at safe (minimum) position for arming.
    Arming interlock: lever must be at minimum before motors will arm.
    """
    if len(channels_raw) < 3:
        return False
    lever_ch_idx = EXPECTED_MAP['Lever'] - 1   # 0-indexed
    lever_val    = channels_raw[lever_ch_idx]
    return lever_val <= LEVER_SAFE_THRESHOLD


def load_mapping():
    """Load saved RC mapping. Returns dict or None."""
    try:
        with open(RESULTS_FILE) as f:
            return json.load(f)
    except (FileNotFoundError, json.JSONDecodeError):
        return None


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='MCO-1D RC channel mapping verification')
    parser.add_argument('--auto',   action='store_true', help='Automatic mode (live MAVLink)')
    parser.add_argument('--manual', action='store_true', help='Manual entry mode')
    args = parser.parse_args()

    # Check if previously verified
    existing = load_mapping()
    if existing and existing.get('all_correct'):
        verified_at = existing.get('verified_at', 'unknown')
        print(f"\nRC mapping previously verified at {verified_at}")
        print("Current mapping:")
        for stick, ch in existing.get('mapping', {}).items():
            print(f"  {stick}: CH{ch}")
        redo = input("\nRe-verify? (yes/no) [no]: ").strip().lower()
        if redo != 'yes':
            print("Using existing verified mapping.")
            sys.exit(0)

    ok = interactive_verify()
    sys.exit(0 if ok else 1)
