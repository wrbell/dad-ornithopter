#!/usr/bin/env python3
"""
mco_log_analysis.py — Flight log analysis and anomaly detection

Reads the 50 Hz CSV flight logs written by mco_logger.py and produces:
  - Summary statistics for each flight
  - Per-wing AoA and servo health report
  - Anomaly flags (stall events, rope slip, IMU spikes, tick overruns)
  - ASCII time-series plots of key parameters
  - TC-01/02/03 data extraction (thrust curve, hover freq, lift slope)

Usage:
    python3 mco_log_analysis.py                        # analyse latest log
    python3 mco_log_analysis.py --file LOGFILE.csv     # specific log
    python3 mco_log_analysis.py --all                  # analyse all logs
    python3 mco_log_analysis.py --tc01                 # extract TC-01 thrust data
    python3 mco_log_analysis.py --tc02                 # extract TC-02 hover data
    python3 mco_log_analysis.py --plot FIELD           # ASCII plot of any field
"""

import os
import sys
import csv
import math
import json
import time
import statistics
import argparse
import glob
from collections import defaultdict

LOG_DIR    = '/home/pi/logs'
OUTPUT_DIR = '/home/pi/knowledge'

# Fields expected in the CSV log
EXPECTED_FIELDS = [
    'timestamp_s', 'freq_hz', 'amp_deg', 'h_agl_m', 'V_ms',
    'roll_deg', 'pitch_deg', 'sink_rate_ms', 'Ps',
    'imu_rms_g', 'bus_voltage_v',
    # Per-wing AoA (8 wings)
    'aoa_w1p', 'aoa_w1s', 'aoa_w2p', 'aoa_w2s',
    'aoa_w3p', 'aoa_w3s', 'aoa_w4p', 'aoa_w4s',
    # Effective AoA
    'eff_aoa_w1p', 'eff_aoa_w1s', 'eff_aoa_w2p', 'eff_aoa_w2s',
    # Currents
    'i_w1p', 'i_w1s', 'i_w2p', 'i_w2s',
    'i_w3p', 'i_w3s', 'i_w4p', 'i_w4s',
    # Stroke positions
    'pos_w1p', 'pos_w1s', 'pos_w2p', 'pos_w2s',
    # Force and balance
    'F_vert_total', 'trim_balance',
    # Stall and health
    'stall_w1', 'stall_w2', 'stall_w3', 'stall_w4',
    'servo_health_rms_mean',
]

WING_NAMES = ['w1p', 'w1s', 'w2p', 'w2s', 'w3p', 'w3s', 'w4p', 'w4s']


def find_latest_log():
    logs = sorted(glob.glob(os.path.join(LOG_DIR, '*.csv')))
    return logs[-1] if logs else None


def load_log(filepath):
    """Load CSV log file. Returns list of dicts."""
    rows = []
    try:
        with open(filepath, newline='') as f:
            reader = csv.DictReader(f)
            for row in reader:
                # Convert numeric fields
                parsed = {}
                for k, v in row.items():
                    try:
                        parsed[k] = float(v)
                    except (ValueError, TypeError):
                        parsed[k] = v
                rows.append(parsed)
    except Exception as e:
        print(f"Error loading {filepath}: {e}")
    return rows


def field_values(rows, field):
    """Extract numeric values for a field, skipping None/NaN."""
    vals = []
    for r in rows:
        v = r.get(field)
        if v is not None:
            try:
                f = float(v)
                if not math.isnan(f) and not math.isinf(f):
                    vals.append(f)
            except (ValueError, TypeError):
                pass
    return vals


def summarise(rows, filepath):
    """Generate summary statistics for a log file."""
    if not rows:
        return {'error': 'empty log'}

    duration_s = (rows[-1].get('timestamp_s', 0) -
                  rows[0].get('timestamp_s', 0))
    n_rows     = len(rows)
    sample_rate = n_rows / max(duration_s, 1)

    summary = {
        'file':        os.path.basename(filepath),
        'duration_s':  round(duration_s, 1),
        'n_samples':   n_rows,
        'sample_rate': round(sample_rate, 1),
    }

    # Key flight parameters
    for field in ['freq_hz', 'amp_deg', 'h_agl_m', 'V_ms',
                  'roll_deg', 'pitch_deg', 'imu_rms_g', 'bus_voltage_v',
                  'trim_balance', 'servo_health_rms_mean']:
        vals = field_values(rows, field)
        if vals:
            summary[field] = {
                'mean': round(statistics.mean(vals), 3),
                'max':  round(max(vals), 3),
                'min':  round(min(vals), 3),
                'std':  round(statistics.stdev(vals) if len(vals) > 1 else 0, 3),
            }

    # Per-wing AoA analysis
    aoa_report = {}
    for wing in WING_NAMES:
        vals = field_values(rows, f'aoa_{wing}')
        if vals:
            aoa_report[wing] = {
                'mean': round(statistics.mean(vals), 2),
                'std':  round(statistics.stdev(vals) if len(vals)>1 else 0, 2),
                'max':  round(max(vals), 2),
            }
    summary['aoa_per_wing'] = aoa_report

    # Port vs stbd AoA symmetry
    port_wings  = ['w1p', 'w2p', 'w3p', 'w4p']
    stbd_wings  = ['w1s', 'w2s', 'w3s', 'w4s']
    port_aoa    = [aoa_report.get(w, {}).get('mean', 0) for w in port_wings if w in aoa_report]
    stbd_aoa    = [aoa_report.get(w, {}).get('mean', 0) for w in stbd_wings if w in aoa_report]
    if port_aoa and stbd_aoa:
        summary['aoa_symmetry'] = {
            'port_mean': round(statistics.mean(port_aoa), 2),
            'stbd_mean': round(statistics.mean(stbd_aoa), 2),
            'asymmetry_deg': round(abs(statistics.mean(port_aoa) -
                                       statistics.mean(stbd_aoa)), 2),
        }

    # Current analysis (rope health proxy)
    current_report = {}
    for wing in WING_NAMES:
        vals = field_values(rows, f'i_{wing}')
        if vals:
            current_report[wing] = {
                'mean': round(statistics.mean(vals), 3),
                'max':  round(max(vals), 3),
            }
    summary['current_per_wing'] = current_report

    # Anomaly detection
    anomalies = detect_anomalies(rows)
    summary['anomalies'] = anomalies

    return summary


def detect_anomalies(rows):
    """Detect anomalies in log data."""
    anomalies = []

    # IMU spikes
    imu_vals = field_values(rows, 'imu_rms_g')
    if imu_vals:
        imu_mean = statistics.mean(imu_vals)
        spikes = sum(1 for v in imu_vals if v > imu_mean * 3 and v > 0.3)
        if spikes > 5:
            anomalies.append({
                'type': 'IMU_SPIKES',
                'count': spikes,
                'message': f'{spikes} IMU spikes above {imu_mean*3:.3f}g — vibration or collision events'
            })

    # Stall events
    for pair in [1, 2, 3, 4]:
        stall_vals = field_values(rows, f'stall_w{pair}')
        if stall_vals:
            stall_count = sum(1 for v in stall_vals if v > 0.5)
            if stall_count > 10:
                anomalies.append({
                    'type': f'STALL_W{pair}',
                    'count': stall_count,
                    'message': f'W{pair} stall likelihood elevated for {stall_count} samples'
                })

    # Current asymmetry (rope slip indicator)
    for pair in [1, 2, 3, 4]:
        port_c = field_values(rows, f'i_w{pair}p')
        stbd_c = field_values(rows, f'i_w{pair}s')
        if port_c and stbd_c:
            port_mean = statistics.mean(port_c)
            stbd_mean = statistics.mean(stbd_c)
            if port_mean > 0.1:
                asym = abs(port_mean - stbd_mean) / port_mean
                if asym > 0.25:
                    anomalies.append({
                        'type': f'CURRENT_ASYMMETRY_W{pair}',
                        'port_A': round(port_mean, 3),
                        'stbd_A': round(stbd_mean, 3),
                        'asymmetry_pct': round(asym*100, 1),
                        'message': (f'W{pair} current asymmetry {asym*100:.0f}% '
                                    f'— check rope drive on heavier side')
                    })

    # Trim imbalance
    trim_vals = field_values(rows, 'trim_balance')
    if trim_vals:
        mean_trim = statistics.mean(trim_vals)
        if abs(mean_trim) > 0.08:
            anomalies.append({
                'type': 'TRIM_IMBALANCE',
                'mean_trim': round(mean_trim, 4),
                'message': (f'Port/stbd trim imbalance: {mean_trim:.3f} '
                            f'(>±0.08 is significant)')
            })

    # Battery voltage drop
    bus_vals = field_values(rows, 'bus_voltage_v')
    if len(bus_vals) > 100:
        early_v = statistics.mean(bus_vals[:50])
        late_v  = statistics.mean(bus_vals[-50:])
        drop    = early_v - late_v
        if drop > 1.5:
            anomalies.append({
                'type': 'BATTERY_DROP',
                'drop_V': round(drop, 2),
                'message': f'Battery dropped {drop:.1f}V during flight — check capacity'
            })

    return anomalies


def extract_tc01_thrust(rows):
    """Extract thrust curve data from a TC-01 static thrust log."""
    # Group rows by freq_hz to get mean force at each frequency step
    freq_groups = defaultdict(list)
    for r in rows:
        freq = r.get('freq_hz')
        force = r.get('F_vert_total')  # total vertical force from logger
        if freq and force:
            try:
                freq_groups[round(float(freq), 2)].append(float(force))
            except (ValueError, TypeError):
                pass

    thrust_curve = {}
    for freq, forces in sorted(freq_groups.items()):
        if len(forces) >= 10:  # need at least 10 samples at this frequency
            thrust_curve[freq] = {
                'mean_N': round(statistics.mean(forces), 2),
                'std_N':  round(statistics.stdev(forces) if len(forces)>1 else 0, 3),
                'n':      len(forces),
            }

    try:
        from mco_wave_library import MTOW_KG
        MTOW_N = MTOW_KG * 9.807
    except Exception:
        MTOW_N = 5.687 * 9.807
    max_thrust = max((v['mean_N'] for v in thrust_curve.values()), default=0)
    hover_freq = None
    for freq in sorted(thrust_curve.keys()):
        if thrust_curve[freq]['mean_N'] >= MTOW_N:
            hover_freq = freq
            break

    return {
        'thrust_curve': thrust_curve,
        'max_thrust_N': round(max_thrust, 2),
        'mtow_N':       round(MTOW_N, 2),
        'tc01_pass':    max_thrust >= MTOW_N,
        'hover_freq_estimate_hz': hover_freq,
    }


def extract_tc02_hover(rows):
    """Extract hover frequency from TC-02 captive hover log."""
    # Look for the period where h_agl is flat and freq_hz is constant
    # In TC-02, the load cell goes to zero at hover freq
    # The logger captures this as the moment F_vert_total crosses zero

    freq_at_zero_force = []
    for r in rows:
        f_vert = r.get('F_vert_total', 1.0)
        freq   = r.get('freq_hz', 0.0)
        if f_vert is not None and freq is not None:
            try:
                if abs(float(f_vert)) < 1.5 and float(freq) > 1.0:
                    freq_at_zero_force.append(float(freq))
            except (ValueError, TypeError):
                pass

    if freq_at_zero_force:
        hover_freq = statistics.median(freq_at_zero_force)
        return {
            'hover_freq_hz': round(hover_freq, 3),
            'n_hover_samples': len(freq_at_zero_force),
            'hover_freq_std': round(statistics.stdev(freq_at_zero_force)
                                     if len(freq_at_zero_force)>1 else 0, 4),
        }
    return {'hover_freq_hz': None, 'n_hover_samples': 0}


def ascii_plot(rows, field, width=60, height=10):
    """Print a simple ASCII time-series plot."""
    vals = field_values(rows, field)
    if not vals:
        print(f"  No data for field '{field}'")
        return

    timestamps = field_values(rows, 'timestamp_s')
    if timestamps and len(timestamps) == len(vals):
        t0 = timestamps[0]
        times = [t - t0 for t in timestamps]
    else:
        times = list(range(len(vals)))

    # Downsample to width points
    n = len(vals)
    step = max(1, n // width)
    sampled_v = vals[::step][:width]
    sampled_t = times[::step][:width]

    v_min = min(sampled_v)
    v_max = max(sampled_v)
    v_range = max(v_max - v_min, 0.001)

    # Build the chart
    chart = [[' '] * len(sampled_v) for _ in range(height)]
    for col, v in enumerate(sampled_v):
        row = int((v - v_min) / v_range * (height - 1))
        row = height - 1 - row  # invert (top = high)
        chart[row][col] = '█'

    print(f"\n  {field}  [{v_min:.3f} – {v_max:.3f}]")
    print(f"  {v_max:>8.3f} │")
    for i, row in enumerate(chart):
        label = ''
        if i == 0:
            label = f" {v_max:.3f}"
        elif i == height // 2:
            mid = (v_max + v_min) / 2
            label = f" {mid:.3f}"
        elif i == height - 1:
            label = f" {v_min:.3f}"
        print(f"           │{''.join(row)}{label}")
    print(f"  {v_min:>8.3f} │")
    t_end = sampled_t[-1] if sampled_t else 0
    print(f"           └{'─'*len(sampled_v)}")
    print(f"           0s{' '*(len(sampled_v)-8)}{t_end:.0f}s")


def print_summary(summary):
    """Print a formatted summary report."""
    print(f"\n{'═'*60}")
    print(f"  FLIGHT LOG ANALYSIS: {summary.get('file', '?')}")
    print(f"{'═'*60}")
    print(f"  Duration:    {summary.get('duration_s', 0):.1f} s")
    print(f"  Samples:     {summary.get('n_samples', 0):,} at {summary.get('sample_rate', 0):.0f} Hz")

    if 'freq_hz' in summary:
        f = summary['freq_hz']
        print(f"\n  FLAPPING: {f['mean']:.2f} Hz mean, "
              f"{f['min']:.2f}–{f['max']:.2f} Hz range")

    if 'imu_rms_g' in summary:
        i = summary['imu_rms_g']
        vib_flag = ' ⚠ HIGH VIBRATION' if i['max'] > 0.3 else ''
        print(f"  VIBRATION: {i['mean']:.4f}g mean, {i['max']:.4f}g peak{vib_flag}")

    if 'bus_voltage_v' in summary:
        b = summary['bus_voltage_v']
        print(f"  BATTERY:   {b['max']:.2f}V → {b['min']:.2f}V  (Δ{b['max']-b['min']:.2f}V)")

    if 'aoa_symmetry' in summary:
        sym = summary['aoa_symmetry']
        print(f"  AoA SYM:   Port {sym['port_mean']:.1f}°  Stbd {sym['stbd_mean']:.1f}°  "
              f"Δ{sym['asymmetry_deg']:.2f}°")

    if 'trim_balance' in summary:
        tb = summary['trim_balance']
        flag = ' ⚠' if abs(tb['mean']) > 0.08 else ''
        print(f"  TRIM BAL:  {tb['mean']:+.3f} (±{tb['std']:.3f}){flag}")

    anomalies = summary.get('anomalies', [])
    if anomalies:
        print(f"\n  ANOMALIES ({len(anomalies)}):")
        for a in anomalies:
            print(f"  ⚠ {a['type']}: {a['message']}")
    else:
        print(f"\n  ANOMALIES: none ✓")

    print(f"{'═'*60}\n")


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='MCO-1D flight log analysis')
    parser.add_argument('--file',  type=str,        help='Specific log file to analyse')
    parser.add_argument('--all',   action='store_true', help='Analyse all log files')
    parser.add_argument('--tc01',  action='store_true', help='Extract TC-01 thrust curve')
    parser.add_argument('--tc02',  action='store_true', help='Extract TC-02 hover frequency')
    parser.add_argument('--plot',  type=str,        help='ASCII plot of a specific field')
    args = parser.parse_args()

    if args.all:
        logs = sorted(glob.glob(os.path.join(LOG_DIR, '*.csv')))
        if not logs:
            print(f"No log files in {LOG_DIR}")
            sys.exit(1)
        for log_path in logs:
            rows = load_log(log_path)
            summary = summarise(rows, log_path)
            print_summary(summary)
    else:
        log_path = args.file or find_latest_log()
        if not log_path:
            print(f"No log files found in {LOG_DIR}")
            sys.exit(1)

        print(f"Loading: {log_path}")
        rows = load_log(log_path)
        if not rows:
            print("Empty or unreadable log file.")
            sys.exit(1)

        summary = summarise(rows, log_path)
        print_summary(summary)

        if args.tc01:
            thrust_data = extract_tc01_thrust(rows)
            print("\nTC-01 THRUST CURVE:")
            for freq, data in sorted(thrust_data['thrust_curve'].items()):
                flag = ' ← MTOW' if data['mean_N'] >= thrust_data['mtow_N'] else ''
                print(f"  {freq:.2f} Hz: {data['mean_N']:.1f} N{flag}")
            print(f"TC-01 pass: {thrust_data['tc01_pass']}")

        if args.tc02:
            hover_data = extract_tc02_hover(rows)
            print(f"\nTC-02 HOVER FREQUENCY: {hover_data['hover_freq_hz']} Hz "
                  f"({hover_data['n_hover_samples']} samples)")

        if args.plot:
            ascii_plot(rows, args.plot)

        # Save summary
        out_path = os.path.join(OUTPUT_DIR,
                    f"summary_{os.path.basename(log_path).replace('.csv', '.json')}")
        os.makedirs(OUTPUT_DIR, exist_ok=True)
        with open(out_path, 'w') as f:
            json.dump(summary, f, indent=2)
        print(f"Summary saved to {out_path}")

# ── Soft-stop log analysis ────────────────────────────────────────────────────
SOFT_STOP_ANALYSIS_FIELDS = ['soft_stop_factor', 'soft_stop_zone']


def analyse_soft_stop(rows):
    """
    Analyse soft-stop performance from flight log rows.
    Expects rows to contain 'soft_stop_factor_wN' fields (N=0-7).
    Returns per-wing taper statistics.

    Healthy operation: factor=1.0 for ~90% of ticks,
    dropping to ~0.35 briefly at each reversal (10% of ticks).
    min_factor should be in 0.30-0.40 range.

    If min_factor = 1.0 always: motor positions not flowing (CAN issue).
    If min_factor < 0.25: soft-stop zone may be too wide.
    """
    result = {}
    for wing_idx in range(8):
        col = f'soft_stop_factor_w{wing_idx}'
        factors = [r.get(col, 1.0) for r in rows if col in r]
        if not factors:
            result[f'w{wing_idx}'] = {'status': 'no_data'}
            continue
        in_zone   = sum(1 for f in factors if f < 0.99)
        min_f     = min(factors)
        mean_f    = sum(factors) / len(factors)
        pct_zone  = in_zone / len(factors) * 100
        healthy   = 0.28 <= min_f <= 0.42 and 5 < pct_zone < 20
        result[f'w{wing_idx}'] = {
            'zone_ticks':     in_zone,
            'total_ticks':    len(factors),
            'min_factor':     round(min_f, 3),
            'mean_factor':    round(mean_f, 3),
            'pct_in_zone':    round(pct_zone, 1),
            'healthy':        healthy,
            'status':         'OK' if healthy else
                              ('CAN_FAIL' if min_f >= 0.99 else 'TUNE_NEEDED'),
        }
    return result
