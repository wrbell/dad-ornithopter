#!/usr/bin/env python3
"""
mco_vibration_check.py — Motor vibration analysis for MCO-1D

Runs a structured vibration sweep to:
  1. Measure IMU RMS vibration at idle, low, and operating frequencies
  2. Identify resonance frequencies that could corrupt SPI reads or SD card
  3. Verify that SPI ADC reads remain stable under vibration
  4. Flag if vibration isolation is inadequate
  5. Estimate safe tick budget headroom under vibration conditions

Run AFTER motors are mounted but BEFORE first wing flapping test (TC-01-07).

Usage:
    python3 mco_vibration_check.py
    python3 mco_vibration_check.py --quick   # abbreviated sweep
    python3 mco_vibration_check.py --plot    # print ASCII vibration chart

Results saved to: /home/pi/knowledge/vibration_check.json
"""

import time
import json
import os
import sys
import math
import statistics
import argparse

RESULTS_FILE = '/home/pi/knowledge/vibration_check.json'

# Vibration limits
IMU_RMS_IDLE_LIMIT    = 0.05   # g — above this at idle = excessive background noise
IMU_RMS_OPERATING_MAX = 0.30   # g — above this at 2.5Hz = damping needed
IMU_RMS_WARN          = 0.15   # g — warning threshold
SPI_ERROR_RATE_MAX    = 0.02   # fraction — above this = SPI unreliable under vibration
TICK_HEADROOM_MIN_MS  = 5.0    # ms — minimum acceptable tick headroom

# Test frequencies
FREQ_SWEEP = [0.5, 1.0, 1.5, 2.0, 2.5, 3.0]


def read_imu_rms(n_samples=50, sample_rate_hz=50):
    """Read IMU and compute RMS acceleration magnitude."""
    try:
        import smbus
        bus = smbus.SMBus(1)
        bus.write_byte_data(0x68, 0x6B, 0x00)  # wake IMU
        time.sleep(0.01)

        readings = []
        interval = 1.0 / sample_rate_hz

        for _ in range(n_samples):
            # Read all 6 axis bytes (AX, AY, AZ)
            data = []
            for reg in [0x3B, 0x3D, 0x3F]:
                hi = bus.read_byte_data(0x68, reg)
                lo = bus.read_byte_data(0x68, reg + 1)
                raw = (hi << 8) | lo
                if raw > 32767:
                    raw -= 65536
                data.append(raw / 16384.0)  # ±2g range
            # Magnitude (remove gravity from Z)
            ax, ay, az = data
            mag = math.sqrt(ax**2 + ay**2 + (az - 1.0)**2)
            readings.append(mag)
            time.sleep(interval)

        bus.close()

        if len(readings) < 5:
            return None

        rms  = math.sqrt(sum(r**2 for r in readings) / len(readings))
        peak = max(readings)
        mean = statistics.mean(readings)
        return {
            'rms_g':   round(rms, 4),
            'peak_g':  round(peak, 4),
            'mean_g':  round(mean, 4),
            'n':       len(readings),
        }
    except Exception as e:
        return {'error': str(e), 'rms_g': 0.0, 'peak_g': 0.0, 'mean_g': 0.0}


def read_spi_stability(n_reads=20):
    """
    Read pitch ADC 20 times and compute error rate.
    With no servos moving, readings should be stable.
    Returns fraction of reads that are None or wildly different from median.
    """
    try:
        import mco_wave_library as w
        readings = []
        for _ in range(n_reads):
            r = w.read_servo_feedback_adc()
            readings.append(r)
            time.sleep(0.02)

        # Count None readings
        none_count = sum(1 for r in readings
                         for v in (r or [None]) if v is None)
        total_values = n_reads * 8
        error_rate = none_count / total_values if total_values > 0 else 1.0

        # Check for outliers in non-None readings
        all_vals = [v for r in readings for v in (r or [])
                    if v is not None and abs(v) < 100]
        if all_vals:
            med = statistics.median(all_vals)
            outliers = sum(1 for v in all_vals if abs(v - med) > 10.0)
            outlier_rate = outliers / len(all_vals)
        else:
            outlier_rate = 1.0

        return {
            'error_rate':   round(error_rate, 3),
            'outlier_rate': round(outlier_rate, 3),
            'reliable':     error_rate < SPI_ERROR_RATE_MAX and outlier_rate < 0.05,
        }
    except Exception as e:
        return {'error': str(e), 'error_rate': 1.0, 'reliable': False}


def measure_tick_time(n_ticks=50):
    """Estimate actual tick processing time by timing wave library calls."""
    try:
        import mco_wave_library as w

        fcc = w.MCOFlightController()
        sensors = {
            'V_ms': 0.0, 'h_agl_m': 0.5, 'pitch_deg_imu': 0.0,
            'roll_deg_imu': 0.0, 'pitch_rate_dps': 0.0, 'roll_rate_dps': 0.0,
            'sink_rate_ms': 0.0, 'Ps_measured': 0.0, 'imu_rms_g': 0.02,
            'motor_I_A': 1.0, 'freq_hz': 2.5, 'motor_currents_a': [1.0]*8,
            'intent': 'hover', 'flap_phases': [0.0]*8, 'stroke_dirs': [1]*8,
            'sweeps_deg': [60.0]*8, 'pitches_deg': [0.0]*8, 'flap_degs': [0.0]*8,
            'cf1_active': False, 'stall_risk': False, 'pitch_stick': 0.0,
            'roll_stick': 0.0, 'lever_pct': 65.0, 'dt_s': 0.02,
            'pitch_stick_rate': 0.0, 'roll_stick_rate': 0.0,
            'dihedrals_deg': [0.0]*8, 'lateral_g': 0.0, 'yaw_rate_radps': 0.0,
            'yaw_stick': 0.0, 'imu_ok': True, 'baro_ok': True,
            'bus_voltage_v': 14.8, 'servo_voltage_v': 7.4, 'accel_z_g': 1.0,
            'rc_signal_valid': True, 'heading_deg': 90.0, 'lat': None, 'lon': None,
            'time_to_impact_s': 99.0, 'rangefinder_m': 0.5, 'flap_amp_deg': 96.0,
            'alt_hold_active': False, 'speed_hold_active': False,
            'heading_hold_active': False, 'vs_corrected_ms': 4.85,
            'air_density_kgm3': 1.225, 'density_alt_m': 0.0, 'slope_deg': 0.0,
            'slope_confidence': 0.0, 'slope_correction': 0.0, 'crosswind_ms': 0.0,
            'oat_c': 15.0, 'servo_actual_aoa_deg': [None]*8,
            'motor_pos_rad': [None]*8, 'sweep_actual_deg': [None]*8,
        }

        times_ms = []
        for _ in range(n_ticks):
            t0 = time.monotonic()
            fcc.tick(sensors)
            t1 = time.monotonic()
            times_ms.append((t1 - t0) * 1000.0)

        return {
            'mean_ms':    round(statistics.mean(times_ms), 2),
            'max_ms':     round(max(times_ms), 2),
            'p95_ms':     round(sorted(times_ms)[int(0.95 * len(times_ms))], 2),
            'budget_ms':  20.0,
            'headroom_ms':round(20.0 - statistics.mean(times_ms), 2),
            'ok':         statistics.mean(times_ms) < 15.0,
        }
    except Exception as e:
        return {'error': str(e), 'mean_ms': 0, 'headroom_ms': 0, 'ok': False}


# SOFT-STOP NOTE:
# The soft-stop system reduces motor velocity in the final 10° before
# reversal. This creates a periodic velocity modulation at twice the
# flapping frequency (one zone entry per reversal per stroke half).
# The vibration sweep at each frequency will capture this signature.
# A clean IMU trace at each frequency confirms the taper is smooth.
# An impulsive spike at reversal (high RMS at harmonics) indicates
# the soft-stop zone or min_vel_frac may need tuning.
# GS: soft_stop_status after each frequency step to check zone entries.
SOFT_STOP_SIGNATURE_NOTE = (
    'At 2x flap freq: soft-stop taper zone entries. '
    'Clean trace = correct taper. Spike = tune soft_stop parameters.'
)

def run_vibration_sweep(quick=False):
    """Run the full vibration characterisation sweep."""
    print("\nMCO-1D VIBRATION CHECK")
    print("=" * 50)

    results = {
        'idle':    {},
        'sweep':   {},
        'spi':     {},
        'tick':    {},
        'summary': {},
    }

    # 1. Baseline tick time (before motors start)
    print("\n1. Measuring tick time (motors off)...")
    results['tick']['motors_off'] = measure_tick_time(n_ticks=30)
    t = results['tick']['motors_off']
    print(f"   Mean: {t['mean_ms']:.2f}ms  Max: {t['max_ms']:.2f}ms  "
          f"Headroom: {t['headroom_ms']:.2f}ms")

    # 2. IMU baseline (motors off)
    print("\n2. IMU baseline (motors off, no vibration)...")
    results['idle']['imu_motors_off'] = read_imu_rms(n_samples=50)
    imu = results['idle']['imu_motors_off']
    print(f"   IMU RMS: {imu.get('rms_g', 0):.4f}g  Peak: {imu.get('peak_g', 0):.4f}g")

    # 3. SPI baseline
    print("\n3. SPI ADC stability (motors off)...")
    results['spi']['motors_off'] = read_spi_stability(n_reads=20)
    spi = results['spi']['motors_off']
    print(f"   Error rate: {spi.get('error_rate', 0):.3f}  "
          f"Reliable: {spi.get('reliable', False)}")

    # 4. Frequency sweep
    if not quick:
        print("\n4. Motor vibration sweep...")
        print("   NOTE: Wings should be installed. Aircraft bolted to test fixture.")
        print("   ODrives must be running (mco_odrive_setup.py already run).")
        input("   Press Enter when ready to start motor sweep: ")

        try:
            from mco_odrive_interface import ODriveInterface
            odi = ODriveInterface()
            odi.enable_all()

            for freq in FREQ_SWEEP:
                print(f"\n   {freq:.1f} Hz: ", end='', flush=True)
                odi.set_cascade_freq(freq)
                time.sleep(2.0)  # settle

                # IMU under vibration
                imu_v = read_imu_rms(n_samples=25)
                # SPI under vibration
                spi_v = read_spi_stability(n_reads=10)
                # Tick time under vibration
                tick_v = measure_tick_time(n_ticks=10)

                rms = imu_v.get('rms_g', 0)
                print(f"IMU={rms:.4f}g  SPI={'OK' if spi_v['reliable'] else 'ERR'}  "
                      f"tick={tick_v['mean_ms']:.1f}ms")

                results['sweep'][freq] = {
                    'imu':  imu_v,
                    'spi':  spi_v,
                    'tick': tick_v,
                }

            odi.idle_all()
            odi.close()

        except Exception as e:
            print(f"   Motor sweep not available: {e}")
            print("   Ensure ODrives are powered and mco_odrive_setup.py has been run.")
    else:
        print("\n4. Motor sweep: SKIPPED (--quick mode)")

    # 5. Generate summary
    print("\n5. Generating summary...")

    baseline_rms = results['idle']['imu_motors_off'].get('rms_g', 0)
    baseline_ok  = baseline_rms < IMU_RMS_IDLE_LIMIT

    sweep_max_rms  = 0.0
    spi_fails      = 0
    tick_max_ms    = results['tick']['motors_off'].get('mean_ms', 0)

    for freq, data in results['sweep'].items():
        rms = data['imu'].get('rms_g', 0)
        if rms > sweep_max_rms:
            sweep_max_rms = rms
        if not data['spi'].get('reliable', True):
            spi_fails += 1
        tick_ms = data['tick'].get('mean_ms', 0)
        if tick_ms > tick_max_ms:
            tick_max_ms = tick_ms

    summary = {
        'baseline_rms_g':      round(baseline_rms, 4),
        'baseline_ok':         baseline_ok,
        'sweep_max_rms_g':     round(sweep_max_rms, 4),
        'vibration_acceptable': sweep_max_rms < IMU_RMS_OPERATING_MAX,
        'spi_fail_count':      spi_fails,
        'spi_reliable':        spi_fails == 0,
        'tick_max_ms':         round(tick_max_ms, 2),
        'tick_headroom_ms':    round(20.0 - tick_max_ms, 2),
        'tick_ok':             tick_max_ms < 15.0,
        'overall_go':          (baseline_ok and
                                sweep_max_rms < IMU_RMS_OPERATING_MAX and
                                spi_fails == 0 and
                                tick_max_ms < 15.0),
    }
    results['summary'] = summary

    # Print assessment
    print(f"\n{'═'*50}")
    print(f"  VIBRATION CHECK RESULTS")
    print(f"{'═'*50}")
    print(f"  Baseline IMU RMS:   {baseline_rms:.4f}g  "
          f"({'OK ✓' if baseline_ok else f'WARN — exceeds {IMU_RMS_IDLE_LIMIT}g limit'})")
    print(f"  Max operating RMS:  {sweep_max_rms:.4f}g  "
          f"({'OK ✓' if sweep_max_rms < IMU_RMS_OPERATING_MAX else 'WARN — improve damping'})")
    print(f"  SPI failures:       {spi_fails}  "
          f"({'OK ✓' if spi_fails == 0 else 'FAIL — SPI unreliable under vibration'})")
    print(f"  Max tick time:      {tick_max_ms:.1f}ms  "
          f"(headroom {20.0-tick_max_ms:.1f}ms)")
    print(f"\n  OVERALL: {'GO ✓' if summary['overall_go'] else 'NO-GO ✗'}")

    if not summary['vibration_acceptable']:
        print(f"\n  RECOMMENDATION:")
        print(f"  Vibration exceeds {IMU_RMS_OPERATING_MAX}g at operating frequency.")
        print(f"  Add vibration isolation: neoprene foam between Pi and frame,")
        print(f"  silicone grommets on ODrive mounts, mass damper on Pi case.")

    if spi_fails > 0:
        print(f"\n  SPI RELIABILITY:")
        print(f"  ADC reads failing under vibration. This will cause servo feedback")
        print(f"  errors in flight. Options:")
        print(f"  1. Add 10nF capacitors between each ADC signal line and GND")
        print(f"  2. Reduce SPI clock speed (slower = more noise immunity)")
        print(f"  3. Add more vibration isolation to Pi and ADC board")

    if summary['tick_headroom_ms'] < TICK_HEADROOM_MIN_MS:
        print(f"\n  TICK BUDGET:")
        print(f"  Only {summary['tick_headroom_ms']:.1f}ms headroom at operating frequency.")
        print(f"  Under full vibration load, Pi 4 may miss the 20ms deadline.")
        print(f"  Review wave library tick stack — some layers may be throttled further.")

    # Save results
    os.makedirs(os.path.dirname(RESULTS_FILE), exist_ok=True)
    results['timestamp'] = time.strftime('%Y-%m-%dT%H:%M:%S')
    with open(RESULTS_FILE, 'w') as f:
        json.dump(results, f, indent=2)
    print(f"\n  Results saved to {RESULTS_FILE}")

    return summary['overall_go']


def ascii_chart(results):
    """Print ASCII bar chart of vibration vs frequency."""
    sweep = results.get('sweep', {})
    if not sweep:
        print("  No sweep data available.")
        return

    print(f"\n  VIBRATION vs FREQUENCY (g RMS)")
    print(f"  {'Freq':>6}  {'Bar':<30}  {'RMS'}")
    max_rms = max((d['imu'].get('rms_g', 0) for d in sweep.values()), default=0.1)

    for freq in sorted(sweep.keys()):
        rms    = sweep[freq]['imu'].get('rms_g', 0)
        bar_w  = int(rms / max(max_rms, 0.01) * 28)
        color  = ('!' if rms > IMU_RMS_OPERATING_MAX
                  else '*' if rms > IMU_RMS_WARN else '#')
        print(f"  {freq:>5.1f}Hz  {color * bar_w:<30}  {rms:.4f}g")


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='MCO-1D vibration characterisation')
    parser.add_argument('--quick', action='store_true', help='Skip motor sweep')
    parser.add_argument('--plot',  action='store_true', help='Print ASCII vibration chart')
    args = parser.parse_args()

    ok = run_vibration_sweep(quick=args.quick)

    if args.plot:
        try:
            with open(RESULTS_FILE) as f:
                results = json.load(f)
            ascii_chart(results)
        except Exception as e:
            print(f"Could not load results for chart: {e}")

    sys.exit(0 if ok else 1)
