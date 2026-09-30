#!/usr/bin/env python3
"""
export_test_card_telemetry.py — Automated Telemetry Exporter for MCO-1D Mark 1

Reads the aircraft's internal flight logs, ODrive telemetry, and test card results:
  - /home/pi/knowledge/tc_results.json
  - /home/pi/knowledge/build_measurements.json
  - /var/log/mco_flight.log

Formats and outputs a clean text/JSON summary matching the Test Card Worksheets
(WS-01, WS-02, WS-03, WS-04) ready to copy/paste into the chat or update simulation models.

Usage:
    python3 scripts/export_test_card_telemetry.py [--tc 1|2|3|4|all] [--json]
"""

import os
import sys
import json
import argparse
from datetime import datetime

RESULTS_FILE = '/home/pi/knowledge/tc_results.json'
BUILD_FILE   = '/home/pi/knowledge/build_measurements.json'
LOG_FILE     = '/var/log/mco_flight.log'

# Fallback local paths if running off-vehicle
LOCAL_RESULTS = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'knowledge', 'tc_results.json'))
LOCAL_BUILD   = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'knowledge', 'build_measurements.json'))

def load_data():
    res_path = RESULTS_FILE if os.path.exists(RESULTS_FILE) else LOCAL_RESULTS
    build_path = BUILD_FILE if os.path.exists(BUILD_FILE) else LOCAL_BUILD

    tc_data = {}
    build_data = {}

    if os.path.exists(res_path):
        try:
            with open(res_path, 'r') as f:
                tc_data = json.load(f)
        except Exception as e:
            print(f"[WARN] Error reading {res_path}: {e}")

    if os.path.exists(build_path):
        try:
            with open(build_path, 'r') as f:
                build_data = json.load(f)
        except Exception as e:
            print(f"[WARN] Error reading {build_path}: {e}")

    return tc_data, build_data

def format_tc01(tc_data):
    tc01 = tc_data.get('tc01', {})
    print("=" * 68)
    print("  MCO-1D TC-01 GROUND BENCH TEST — AUTOMATED TELEMETRY SUMMARY")
    print("=" * 68)
    if not tc01:
        print("  No TC-01 run data found in results. Execute: tc01_start")
        return

    print("  Point | Freq   | Amp    | Measured Thrust | Bus Current | Motor T | FET T")
    print("  ------+--------+--------+-----------------+-------------+---------+-------")
    steps = [
        ('P1', '1.0 Hz', '±60°', 'tc01_07', '8.0 N'),
        ('P2', '1.5 Hz', '±70°', 'tc01_08', '16.0 N'),
        ('P3', '2.0 Hz', '±80°', 'tc01_09', '24.0 N'),
        ('P4', '2.5 Hz', '±80°', 'tc01_10', '31.0 N'),
        ('P5', '3.0 Hz', '±80°', 'tc01_11', '37.0 N'),
        ('P6', '3.5 Hz', '±70°', 'tc01_12', '42.0 N'),
    ]
    for pt, freq, amp, step_key, target in steps:
        step_res = tc01.get(step_key, {})
        d = step_res.get('data', {})
        thrust = f"{d.get('thrust_n', 0.0):.1f} N" if 'thrust_n' in d else "[Pending]"
        current = f"{d.get('current_a', 0.0):.1f} A" if 'current_a' in d else "[Pending]"
        m_temp = f"{d.get('motor_temp_c', 0.0):.1f}°C" if 'motor_temp_c' in d else "[Pending]"
        f_temp = f"{d.get('fet_temp_c', 0.0):.1f}°C" if 'fet_temp_c' in d else "[Pending]"
        print(f"  {pt:5} | {freq:6} | {amp:6} | {thrust:15} | {current:11} | {m_temp:7} | {f_temp:7}")
    
    # 7.4V rail and vibration
    droop = tc01.get('tc01_13', {}).get('data', {}).get('servo_rail_v', 'N/A')
    vib   = tc01.get('tc01_14', {}).get('data', {}).get('vibration_rms_g', 'N/A')
    print("  ------+--------+--------+-----------------+-------------+---------+-------")
    print(f"  7.4V Servo Rail Voltage Under Load: {droop} V (Target: >= 7.10V)")
    print(f"  Airframe Accelerometer Vibration:   {vib} g RMS (Limit: <= 2.0g)")
    print("=" * 68)

def format_tc02(tc_data):
    tc02 = tc_data.get('tc02', {})
    print("=" * 68)
    print("  MCO-1D TC-02 TETHERED LIFT & HOVER — AUTOMATED TELEMETRY SUMMARY")
    print("=" * 68)
    if not tc02:
        print("  No TC-02 run data found in results. Execute: tc02_start")
        return
    
    h_freq = tc02.get('hover_freq_hz', tc02.get('tc02_03', {}).get('data', {}).get('hover_freq_hz', '[Pending]'))
    h_curr = tc02.get('tc02_03', {}).get('data', {}).get('current_a', '[Pending]')
    h_pwr  = tc02.get('tc02_03', {}).get('data', {}).get('power_watts', '[Pending]')
    p_rate = tc02.get('tc02_05', {}).get('data', {}).get('pitch_rate_dps', '[Pending]')
    r_rate = tc02.get('tc02_06', {}).get('data', {}).get('roll_rate_dps', '[Pending]')
    y_rate = tc02.get('tc02_07', {}).get('data', {}).get('yaw_rate_dps', '[Pending]')

    print(f"  Unstick Equilibrium Frequency (T/W = 1.00): {h_freq} Hz (Target: 1.85-2.10 Hz)")
    print(f"  Total Hover Current:                       {h_curr} A")
    print(f"  Total Hover Electrical Power:              {h_pwr} W (Target: 400-530 W)")
    print(f"  Pitch Control Response Rate:               {p_rate} °/s (Target: >= 15.0 °/s)")
    print(f"  Roll Control Response Rate:                {r_rate} °/s (Target: >= 20.0 °/s)")
    print(f"  Yaw Control Response Rate:                 {y_rate} °/s (Target: >= 12.0 °/s)")
    print("=" * 68)

def format_tc04(tc_data):
    tc04 = tc_data.get('tc04', {})
    print("=" * 68)
    print("  MCO-1D TC-04 FREE FLIGHT STOL — AUTOMATED TELEMETRY SUMMARY")
    print("=" * 68)
    if not tc04:
        print("  No TC-04 run data found in results. Execute: tc04_start")
        return
    
    to_dist = tc04.get('tc04_01', {}).get('data', {}).get('takeoff_roll_m', '[Pending]')
    v_rot   = tc04.get('tc04_01', {}).get('data', {}).get('v_rot_ms', '[Pending]')
    v_z     = tc04.get('tc04_02', {}).get('data', {}).get('climb_rate_ms', '[Pending]')
    v_cr    = tc04.get('tc04_03', {}).get('data', {}).get('cruise_speed_ms', '[Pending]')
    f_cr    = tc04.get('tc04_03', {}).get('data', {}).get('flapping_freq_hz', '[Pending]')
    p_cr    = tc04.get('tc04_03', {}).get('data', {}).get('cruise_power_w', '[Pending]')
    cot     = tc04.get('tc04_03', {}).get('data', {}).get('cost_of_transport_wh_km', '[Pending]')
    v_td    = tc04.get('tc04_04', {}).get('data', {}).get('touchdown_speed_ms', '[Pending]')
    l_dist  = tc04.get('tc04_04', {}).get('data', {}).get('rollout_m', '[Pending]')

    print(f"  Takeoff Roll Distance:       {to_dist} m (Limit: <= 12.0 m)")
    print(f"  Liftoff Indicated Airspeed:  {v_rot} m/s (Target: 6.20 m/s)")
    print(f"  Steady Rate of Climb (Vz):   {v_z} m/s (Target: 1.80-2.20 m/s)")
    print(f"  Level Cruise Airspeed:       {v_cr} m/s (Target: 9.00 m/s)")
    print(f"  Cruise Flapping Frequency:   {f_cr} Hz (Nominal: 2.20-2.50 Hz)")
    print(f"  Mean Cruise Power:           {p_cr} W (Target: 240-280 W)")
    print(f"  Cost of Transport:           {cot} Wh/km (Target: 7.4-8.6 Wh/km)")
    print(f"  Touchdown Speed:             {v_td} m/s (Target: 4.50-5.20 m/s)")
    print(f"  Landing Rollout Distance:    {l_dist} m (Limit: <= 8.5 m)")
    print("=" * 68)

def main():
    parser = argparse.ArgumentParser(description="Export MCO-1D Flight Test Telemetry")
    parser.add_argument('--tc', choices=['1', '2', '3', '4', 'all'], default='all', help='Test card to report')
    parser.add_argument('--json', action='store_true', help='Output raw JSON format')
    args = parser.parse_args()

    tc_data, build_data = load_data()

    if args.json:
        combined = {
            'timestamp': datetime.now().isoformat(),
            'build_measurements': build_data,
            'test_card_results': tc_data
        }
        print(json.dumps(combined, indent=2))
        return

    print("\n[MCO-1D MARK 1 FLIGHT TEST TELEMETRY EXPORTER]")
    print(f"Generated at: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")

    if args.tc in ('1', 'all'):
        format_tc01(tc_data)
        print()
    if args.tc in ('2', 'all'):
        format_tc02(tc_data)
        print()
    if args.tc in ('4', 'all'):
        format_tc04(tc_data)
        print()

    print("Tip: Run 'python3 scripts/export_test_card_telemetry.py --json' to get raw JSON for copy/pasting.\n")

if __name__ == '__main__':
    main()
