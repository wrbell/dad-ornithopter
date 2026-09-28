#!/usr/bin/env python3
"""
mco_loadcell.py — HX711 load cell reader for MCO-1D

Hardware:
    HX711 ADC connected to 50N (or larger) strain gauge load cell.
    HX711 DT  (data)  → GPIO 26
    HX711 SCK (clock) → GPIO  7
    Power: 3.3 V or 5 V from Pi (HX711 accepts both; 5V gives better noise margin)

Usage (standalone test):
    python3 mco_loadcell.py

Usage (from TC-01 / TC-02):
    import mco_loadcell as lc
    cell = lc.HX711Reader()
    cell.tare()
    newtons = cell.read_newtons(n_samples=20)

Calibration:
    1. Run: python3 mco_loadcell.py --calibrate
    2. Place a known mass (e.g. 1.000 kg = 9.807 N) on the load cell.
    3. Follow the prompts — the calibration factor is saved to
       /home/pi/knowledge/loadcell_cal.json and loaded automatically
       on every subsequent run.

Notes:
    The HX711 samples at 10 Hz (RATE pin low) or 80 Hz (RATE pin high).
    Default wiring leaves RATE pin floating → 10 Hz.
    read_newtons() with n_samples=20 therefore takes ~2 seconds.
    For TC-02 continuous monitoring, use read_newtons(n_samples=5) at 0.5 s intervals.
"""

import time
import os
import json
import sys
import math
import argparse
import statistics

try:
    import RPi.GPIO as GPIO
    _GPIO_AVAILABLE = True
except ImportError:
    _GPIO_AVAILABLE = False

# ── Pin assignments (match mco_wave_library.py and GPIO reference table) ────
DT_PIN   = 26    # HX711 DOUT
SCK_PIN  = 7     # HX711 PD_SCK
                 # NOTE: GPIO 7 is also SPI1 CE1 (pitch ADC chip select).
                 # HX711 and SPI1 CE1 share this pin.
                 # The HX711 is BENCH-ONLY — not installed in flight.
                 # SPI1 is only active when Python reads the ADC.
                 # HX711 SCK is only active when explicitly clocked.
                 # There is no conflict in practice: TC-01 and TC-02
                 # do not run the pitch ADC and the load cell simultaneously.
                 # If you need to run both, move HX711 SCK to a free GPIO.

# ── Calibration file ─────────────────────────────────────────────────────────
CAL_FILE = '/home/pi/knowledge/loadcell_cal.json'

# ── Default calibration (raw counts per newton) ──────────────────────────────
# This is a placeholder. Run --calibrate to get the real value.
# Typical HX711 with 50N load cell: ~20,000–80,000 counts/N depending on
# gain setting and load cell sensitivity.
DEFAULT_COUNTS_PER_NEWTON = 50000.0   # placeholder — replace via calibration
DEFAULT_GAIN               = 128       # HX711 channel A gain (64 or 128)


class HX711Reader:
    """
    HX711 24-bit ADC driver for load cell reading.

    The HX711 protocol is bit-banged on GPIO.
    After conversion is ready (DOUT goes LOW), 24 clock pulses read the data.
    Additional pulses after the 24th set the gain for the next conversion:
        25 pulses → Channel A, gain 128
        26 pulses → Channel B, gain 32
        27 pulses → Channel A, gain 64
    This driver uses 25 pulses (Channel A, gain 128) by default.
    """

    def __init__(self, dt_pin=DT_PIN, sck_pin=SCK_PIN, gain=DEFAULT_GAIN):
        self.dt_pin       = dt_pin
        self.sck_pin      = sck_pin
        self.gain         = gain
        self._tare_offset = 0
        self._counts_per_N = DEFAULT_COUNTS_PER_NEWTON
        self._cal_loaded   = False

        if gain == 128:
            self._pulses = 25
        elif gain == 64:
            self._pulses = 27
        elif gain == 32:
            self._pulses = 26
        else:
            raise ValueError(f"Invalid HX711 gain {gain}. Must be 64, 128, or 32.")

        if _GPIO_AVAILABLE:
            GPIO.setmode(GPIO.BCM)
            GPIO.setwarnings(False)
            GPIO.setup(self.sck_pin, GPIO.OUT)
            GPIO.setup(self.dt_pin,  GPIO.IN)
            GPIO.output(self.sck_pin, GPIO.LOW)
            # Power-on: cycle SCK to reset HX711
            self._reset()

        self._load_calibration()

    # ── GPIO low-level ────────────────────────────────────────────────────────

    def _reset(self):
        """Reset HX711 by holding SCK high for >60 µs."""
        if not _GPIO_AVAILABLE:
            return
        GPIO.output(self.sck_pin, GPIO.HIGH)
        time.sleep(0.0001)   # 100 µs
        GPIO.output(self.sck_pin, GPIO.LOW)
        time.sleep(0.4)      # Wait for first conversion after power-on

    def _is_ready(self):
        """Return True when HX711 has a conversion ready (DOUT low)."""
        if not _GPIO_AVAILABLE:
            return True
        return GPIO.input(self.dt_pin) == GPIO.LOW

    def _wait_ready(self, timeout_s=2.0):
        """Block until HX711 is ready or timeout."""
        t_end = time.monotonic() + timeout_s
        while not self._is_ready():
            if time.monotonic() > t_end:
                raise TimeoutError("HX711 not ready within timeout. "
                                   "Check wiring: DT to GPIO26, SCK to GPIO7, "
                                   "VCC to 3.3V or 5V, GND to GND.")
            time.sleep(0.001)

    def _read_raw(self):
        """
        Read one 24-bit raw value from HX711.
        Returns signed integer (two's complement).
        Raises TimeoutError if HX711 does not respond.
        """
        if not _GPIO_AVAILABLE:
            # Simulation mode: return a realistic noise value
            import random
            base = self._tare_offset + self._counts_per_N * 0.0
            return int(base + random.gauss(0, 100))

        self._wait_ready()

        raw = 0
        for _ in range(24):
            GPIO.output(self.sck_pin, GPIO.HIGH)
            time.sleep(0.000001)   # 1 µs
            raw = (raw << 1) | GPIO.input(self.dt_pin)
            GPIO.output(self.sck_pin, GPIO.LOW)
            time.sleep(0.000001)

        # Extra pulses to set gain for next conversion
        for _ in range(self._pulses - 24):
            GPIO.output(self.sck_pin, GPIO.HIGH)
            time.sleep(0.000001)
            GPIO.output(self.sck_pin, GPIO.LOW)
            time.sleep(0.000001)

        # Convert to signed 24-bit integer (two's complement)
        if raw & 0x800000:
            raw -= 0x1000000
        return raw

    # ── High-level interface ───────────────────────────────────────────────────

    def read_raw_average(self, n_samples=10):
        """
        Read n_samples raw counts and return the median.
        Median is more robust than mean against occasional outlier readings.
        At 10 Hz sample rate, n_samples=10 takes ~1 second.
        """
        readings = []
        errors   = 0
        while len(readings) < n_samples:
            try:
                readings.append(self._read_raw())
            except TimeoutError:
                errors += 1
                if errors > 5:
                    raise TimeoutError("HX711 repeatedly not responding. "
                                       "Check power and wiring.")
                time.sleep(0.1)
        return statistics.median(readings)

    def tare(self, n_samples=20):
        """
        Zero the load cell. Call with nothing on the cell (or with the
        aircraft weight if you want to measure ADDITIONAL force only).

        For TC-01/TC-02: tare with aircraft at rest on the post.
        Then increasing flapping frequency will show UPWARD force
        (load cell reading decreasing from tare).
        When load cell reads zero: aircraft is generating exactly
        its own weight in lift = hover point.
        """
        print(f"  Taring load cell ({n_samples} samples)...", end='', flush=True)
        self._tare_offset = self.read_raw_average(n_samples)
        print(f" done. Offset: {self._tare_offset:.0f} counts")
        return self._tare_offset

    def read_counts(self, n_samples=10):
        """Return tare-compensated raw count average."""
        return self.read_raw_average(n_samples) - self._tare_offset

    def read_newtons(self, n_samples=10):
        """
        Return force in Newtons.
        Positive = force pushing DOWN on load cell (weight / thrust downward).
        Negative = force pulling UP (aircraft generating lift > its own weight).

        After taring with aircraft at rest:
            read_newtons() = -(lift being generated)
        So hover point is when read_newtons() = 0.
        Full weight off the post: read_newtons() ≈ -MTOW_N

        For TC-02, monitor this value while stepping up frequency.
        Hover frequency = frequency at which read_newtons() first reaches 0.
        """
        counts = self.read_counts(n_samples)
        return counts / self._counts_per_N

    def read_kg(self, n_samples=10):
        """Convenience: return reading in kg."""
        return self.read_newtons(n_samples) / 9.807

    # ── Calibration ───────────────────────────────────────────────────────────

    def calibrate(self, known_mass_kg, n_samples=30):
        """
        Calibrate using a known mass placed on the load cell.

        Procedure:
          1. Call tare() with cell unloaded.
          2. Place known mass on cell.
          3. Call calibrate(known_mass_kg).
          4. Calibration is saved automatically.

        Args:
            known_mass_kg: The exact mass of the calibration weight in kg.
                           Example: 1.000 for a 1 kg weight.
        """
        known_N  = known_mass_kg * 9.807
        print(f"  Calibrating with {known_mass_kg:.3f} kg ({known_N:.3f} N)...")
        print(f"  Place calibration weight on load cell now.")
        input("  Press Enter when weight is stable on the cell: ")

        print(f"  Reading {n_samples} samples...", end='', flush=True)
        counts = self.read_counts(n_samples)
        print(f" done. Counts: {counts:.0f}")

        if abs(counts) < 100:
            print("  WARNING: Very few counts with calibration weight.")
            print("  Check that the weight is properly placed and the cell is deflecting.")
            print("  If using simulation mode, counts will be near zero.")

        self._counts_per_N = counts / known_N if known_N > 0 and abs(counts) > 100 else DEFAULT_COUNTS_PER_NEWTON
        print(f"  Calibration factor: {self._counts_per_N:.1f} counts/N")

        # Verify
        measured_N = self.read_newtons(n_samples=10)
        error_pct  = abs(measured_N - known_N) / known_N * 100
        print(f"  Verification: measured {measured_N:.3f} N, expected {known_N:.3f} N "
              f"(error: {error_pct:.1f}%)")

        self._save_calibration()
        return self._counts_per_N

    def _save_calibration(self):
        os.makedirs(os.path.dirname(CAL_FILE), exist_ok=True)
        cal = {
            'counts_per_newton': self._counts_per_N,
            'tare_offset':       self._tare_offset,
            'gain':              self.gain,
            'dt_pin':            self.dt_pin,
            'sck_pin':           self.sck_pin,
            'calibrated_at':     time.strftime('%Y-%m-%dT%H:%M:%S'),
        }
        with open(CAL_FILE, 'w') as f:
            json.dump(cal, f, indent=2)
        print(f"  Calibration saved to {CAL_FILE}")

    def _load_calibration(self):
        try:
            with open(CAL_FILE) as f:
                cal = json.load(f)
            self._counts_per_N = cal['counts_per_newton']
            self._tare_offset  = cal.get('tare_offset', 0)
            self._cal_loaded   = True
            print(f"  Load cell: calibration loaded "
                  f"({self._counts_per_N:.1f} counts/N, "
                  f"calibrated {cal.get('calibrated_at','?')})")
        except (FileNotFoundError, KeyError, json.JSONDecodeError):
            print(f"  Load cell: no calibration file found at {CAL_FILE}")
            print(f"  Using default {DEFAULT_COUNTS_PER_NEWTON:.0f} counts/N")
            print(f"  Run:  python3 mco_loadcell.py --calibrate")

    def close(self):
        if _GPIO_AVAILABLE:
            GPIO.cleanup([self.dt_pin, self.sck_pin])

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.close()


# ── Convenience functions (used by TC-01 and TC-02 directly) ──────────────

_default_reader = None

def _get_reader():
    global _default_reader
    if _default_reader is None:
        _default_reader = HX711Reader()
    return _default_reader

def tare():
    """Tare the default reader."""
    return _get_reader().tare()

def read_newtons(n_samples=10):
    """Read newtons from the default reader."""
    return _get_reader().read_newtons(n_samples)

def read_kg(n_samples=10):
    return _get_reader().read_kg(n_samples)


# ── TC-02 continuous monitoring ───────────────────────────────────────────────

def monitor_for_hover(freq_hz, duration_s=5.0, n_samples=5,
                      hover_threshold_N=1.0, report_fn=None):
    """
    Monitor load cell for hover_threshold_N seconds during TC-02.

    Called by TC02Runner._s02_lift_freq_curve() and _s03_find_hover_freq().

    Args:
        freq_hz:          Current wing frequency (for logging only)
        duration_s:       How long to monitor
        n_samples:        Samples per reading (5 = ~0.5s at 10Hz)
        hover_threshold_N: Force magnitude below this = "hovering"
        report_fn:        Optional callable(freq_hz, force_N, lift_N) for logging

    Returns:
        dict: {
            'mean_force_N':  mean load cell reading over duration,
            'mean_lift_N':   mean lift generated (tare - force),
            'min_force_N':   minimum force reading (peak lift moment),
            'hovering':      True if mean_force_N <= hover_threshold_N,
            'samples':       list of individual readings,
        }
    """
    reader   = _get_reader()
    readings = []
    t_end    = time.monotonic() + duration_s

    while time.monotonic() < t_end:
        try:
            N = reader.read_newtons(n_samples)
            readings.append(N)
            if report_fn:
                # Lift = negative of reading (tared against aircraft weight)
                # i.e. when aircraft generates lift, load cell reads less
                lift_N = -N
                report_fn(freq_hz, N, lift_N)
        except TimeoutError:
            pass
        time.sleep(0.1)

    if not readings:
        return {'mean_force_N': 0, 'mean_lift_N': 0,
                'min_force_N': 0, 'hovering': False, 'samples': []}

    mean_force = statistics.mean(readings)
    min_force  = min(readings)
    mean_lift  = -mean_force   # negative force = lift

    return {
        'mean_force_N': round(mean_force, 3),
        'mean_lift_N':  round(mean_lift,  3),
        'min_force_N':  round(min_force,  3),
        'hovering':     abs(mean_force) <= hover_threshold_N,
        'samples':      [round(r, 3) for r in readings],
    }


# ── CLI ───────────────────────────────────────────────────────────────────────

def _cli_test(reader):
    """Interactive test mode."""
    print("\nLOAD CELL TEST — press Ctrl+C to exit\n")
    reader.tare()
    print(f"{'Reading':>10}  {'Newtons':>10}  {'kg':>8}")
    print(f"{'─'*32}")
    try:
        while True:
            N   = reader.read_newtons(n_samples=5)
            kg  = N / 9.807
            print(f"  {reader.read_counts(n_samples=3):>10.0f}  {N:>10.3f} N  {kg:>8.4f} kg",
                  end='\r')
    except KeyboardInterrupt:
        print()

def _cli_calibrate(reader):
    """Interactive calibration mode."""
    print("\nLOAD CELL CALIBRATION\n")
    print("Step 1: Ensure load cell is UNLOADED.")
    input("Press Enter to tare: ")
    reader.tare(n_samples=30)

    print("\nStep 2: Place calibration weight on load cell.")
    mass_str = input("Enter calibration mass in kg (e.g. 1.000): ").strip()
    try:
        mass_kg = float(mass_str)
    except ValueError:
        print(f"Invalid mass '{mass_str}'")
        return

    reader.calibrate(mass_kg)
    print("\nCalibration complete. Testing...")

    print(f"\nReading with {mass_kg:.3f} kg still on cell:")
    for _ in range(5):
        N  = reader.read_newtons(n_samples=5)
        kg = N / 9.807
        print(f"  {N:.3f} N = {kg:.4f} kg  (expected {mass_kg:.4f} kg)")
        time.sleep(0.5)

def _cli_thrust_sweep(reader):
    """TC-01 thrust sweep — steps through frequencies and records thrust."""
    print("\nTHRUST SWEEP — TC-01 STEP 12\n")
    print("Aircraft must be secured to load cell fixture.")
    print("ODrives must be running (use mco_odrive_setup.py first).")
    input("Press Enter when ready: ")

    reader.tare(n_samples=30)

    try:
        from mco_odrive_interface import ODriveInterface
        odi = ODriveInterface()
        odi.enable_all()
    except Exception as e:
        print(f"ODrive not available: {e}")
        print("Running in measurement-only mode (no motor control).")
        odi = None

    freqs = [1.5, 1.75, 2.0, 2.25, 2.5, 2.75, 3.0]
    results = {}

    print(f"\n{'Freq Hz':>8}  {'Load cell N':>12}  {'Lift N':>10}  {'Status'}")
    print(f"{'─'*50}")

    for freq in freqs:
        if odi:
            odi.set_cascade_freq(freq)
            time.sleep(3.0)   # settle

        result = monitor_for_hover(freq, duration_s=5.0, n_samples=5)
        lift_N = result['mean_lift_N']
        force_N = result['mean_force_N']
        hovering = result['hovering']
        results[freq] = result

        status = 'HOVER' if hovering else ''
        print(f"  {freq:>6.2f} Hz  {force_N:>10.2f} N  {lift_N:>10.2f} N  {status}")

    if odi:
        odi.idle_all()
        odi.close()

    # Summary
    print(f"\nTHRUST SWEEP RESULTS:")
    best_lift = max(results.values(), key=lambda r: r['mean_lift_N'])
    best_freq = [f for f, r in results.items()
                 if r['mean_lift_N'] == best_lift['mean_lift_N']][0]
    print(f"  Max lift:        {best_lift['mean_lift_N']:.2f} N at {best_freq:.2f} Hz")
    try:
        from mco_wave_library import MTOW_KG
        MTOW_N = MTOW_KG * 9.807
    except Exception:
        MTOW_N = 5.687 * 9.807
    print(f"  MTOW:            {MTOW_N:.2f} N")
    print(f"  TC-01 PASS:      {'YES' if best_lift['mean_lift_N'] >= MTOW_N else 'NO'}")

    # Save
    import json
    out_path = '/home/pi/knowledge/tc01_thrust_sweep.json'
    os.makedirs(os.path.dirname(out_path), exist_ok=True)
    with open(out_path, 'w') as f:
        json.dump({'results': {str(k): v for k, v in results.items()},
                   'timestamp': time.strftime('%Y-%m-%dT%H:%M:%S')}, f, indent=2)
    print(f"  Saved to {out_path}")


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='MCO-1D HX711 load cell tool')
    parser.add_argument('--calibrate',    action='store_true',
                        help='Run calibration procedure')
    parser.add_argument('--thrust-sweep', action='store_true',
                        help='TC-01 thrust sweep (needs ODrives running)')
    parser.add_argument('--test',         action='store_true',
                        help='Continuous readout test')
    parser.add_argument('--dt',  type=int, default=DT_PIN,
                        help=f'HX711 DT GPIO pin (default {DT_PIN})')
    parser.add_argument('--sck', type=int, default=SCK_PIN,
                        help=f'HX711 SCK GPIO pin (default {SCK_PIN})')
    args = parser.parse_args()

    if not _GPIO_AVAILABLE:
        print("NOTE: RPi.GPIO not available — running in simulation mode")
        print("All readings will return near-zero values.")
        print()

    with HX711Reader(dt_pin=args.dt, sck_pin=args.sck) as reader:
        if args.calibrate:
            _cli_calibrate(reader)
        elif args.thrust_sweep:
            _cli_thrust_sweep(reader)
        else:
            _cli_test(reader)
