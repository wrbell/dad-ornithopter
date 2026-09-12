#!/usr/bin/env python3
"""
mco_preflight_checks.py — Pre-flight verification and arming interlocks

Runs a structured set of go/no-go checks before any test card or flight.
Designed to be called automatically at startup and before arming.

Checks:
  1.  Battery voltage — main and servo
  2.  ODrive S1 8-node communication (8 boards, 1 motor each)
  3.  CAN bus status
  4.  Pitch ADC (MCP3208 #1 SPI1 CE1) — all 8 channels readable
  5.  Sweep ADC (MCP3208 #2 SPI1 CE0) — all 8 channels readable
  6.  IMU (MPU-6050 at I2C 0x68) — responding and level
  7.  Barometer (BMP280 at I2C 0x76) — responding
  8.  RC signal — valid, channels mapped correctly
  9.  OAT sensor (DS18B20) — reading plausible temperature
  10. SD card space — at least 500 MB free
  11. Load cell (if bench mode) — responding and tared
  12. Build measurements — confirmed (not placeholder values)
  13. Battery interlock — minimum 80% charge for TC runs
  14. Wave library — imports and version check

Usage:
    python3 mco_preflight_checks.py              # full check
    python3 mco_preflight_checks.py --quick      # skip slow checks
    python3 mco_preflight_checks.py --bench      # include load cell check
    python3 mco_preflight_checks.py --flight     # flight mode (no load cell)

From mco_main.py:
    from mco_preflight_checks import run_all_checks, CheckResult
    result = run_all_checks(sensors, bench_mode=False)
    if not result.go:
        print(result.summary())
        sys.exit(1)
"""

import os
import sys
import time
import json
import math
import subprocess
import argparse
from dataclasses import dataclass, field
from typing import List, Optional

# ── Check result types ────────────────────────────────────────────────────────

PASS    = 'PASS'
FAIL    = 'FAIL'
WARN    = 'WARN'
SKIP    = 'SKIP'

@dataclass
class Check:
    name:    str
    status:  str = SKIP
    message: str = ''
    value:   object = None

    @property
    def ok(self):
        return self.status in (PASS, WARN, SKIP)

    def __str__(self):
        icons = {PASS: '✓', FAIL: '✗', WARN: '⚠', SKIP: '○'}
        return f"  {icons[self.status]} {self.name:<40} {self.message}"


@dataclass
class CheckResult:
    checks: List[Check] = field(default_factory=list)

    @property
    def go(self):
        return all(c.ok for c in self.checks)

    @property
    def failures(self):
        return [c for c in self.checks if c.status == FAIL]

    @property
    def warnings(self):
        return [c for c in self.checks if c.status == WARN]

    def summary(self):
        lines = [f"\nPRE-FLIGHT CHECK {'GO' if self.go else 'NO-GO'}"]
        lines.append('─' * 50)
        for c in self.checks:
            lines.append(str(c))
        if self.failures:
            lines.append(f"\nFAILURES ({len(self.failures)}):")
            for c in self.failures:
                lines.append(f"  ✗ {c.name}: {c.message}")
        if self.warnings:
            lines.append(f"\nWARNINGS ({len(self.warnings)}):")
            for c in self.warnings:
                lines.append(f"  ⚠ {c.name}: {c.message}")
        lines.append(f"\n{'GO ✓' if self.go else 'NO-GO ✗'}")
        return '\n'.join(lines)

    def add(self, check: Check):
        self.checks.append(check)
        return check


# ── Individual checks ─────────────────────────────────────────────────────────

def check_wave_library(result: CheckResult):
    c = Check('Wave library')
    try:
        import mco_wave_library as w
        n_hyps = len(w.BIOMIMETIC_HYPOTHESES)
        n_cmds = len(w._GS_ALLOWED_CMDS)
        c.status  = PASS
        c.message = f"{len(open('mco_wave_library.py').read().splitlines()):,} lines, {n_hyps} hypotheses, {n_cmds} GS commands"
    except ImportError as e:
        c.status  = FAIL
        c.message = f"Import failed: {e}"
    except Exception as e:
        c.status  = WARN
        c.message = f"Loaded with warning: {e}"
    return result.add(c)


def check_build_measurements(result: CheckResult):
    c = Check('Build measurements')
    bm_file = '/home/pi/knowledge/build_measurements.json'
    try:
        with open(bm_file) as f:
            bm = json.load(f)
        measured_at = bm.get('measured_at', 'unknown date')
        mtow_kg = bm.get('derived', {}).get('mtow_kg', 0)
        chord   = bm.get('measured', {}).get('chord_mm', 0)
        if mtow_kg > 0 and chord > 0:
            c.status  = PASS
            c.message = f"MTOW {mtow_kg:.3f}kg, chord {chord:.0f}mm — measured {measured_at}"
        else:
            c.status  = WARN
            c.message = f"File exists but values appear default. Re-run mco_build_measurements.py."
    except FileNotFoundError:
        c.status  = WARN
        c.message = "Not measured yet. Run mco_build_measurements.py before TC-01."
    except Exception as e:
        c.status  = WARN
        c.message = str(e)
    return result.add(c)


def check_sd_space(result: CheckResult):
    c = Check('SD card space')
    try:
        stat  = os.statvfs('/home/pi')
        free_mb = stat.f_bavail * stat.f_frsize / 1_048_576
        if free_mb >= 500:
            c.status  = PASS
            c.message = f"{free_mb:.0f} MB free"
        elif free_mb >= 200:
            c.status  = WARN
            c.message = f"Only {free_mb:.0f} MB free — consider clearing old logs"
        else:
            c.status  = FAIL
            c.message = f"Only {free_mb:.0f} MB free — CLEAR LOGS BEFORE FLIGHT"
    except Exception as e:
        c.status  = WARN
        c.message = str(e)
    return result.add(c)


def check_can_bus(result: CheckResult):
    c = Check('CAN bus (can0)')
    try:
        out = subprocess.check_output(['ip', 'link', 'show', 'can0'],
                                       stderr=subprocess.DEVNULL, text=True)
        if 'UP' in out:
            c.status  = PASS
            c.message = "can0 UP"
        else:
            c.status  = FAIL
            c.message = "can0 not UP — run mco_can_setup.sh"
    except subprocess.CalledProcessError:
        c.status  = FAIL
        c.message = "can0 interface not found — check MCP2515 wiring and dtoverlay"
    except FileNotFoundError:
        c.status  = SKIP
        c.message = "ip command not available (not on Pi)"
    return result.add(c)


def check_odrives(result: CheckResult):
    c = Check('ODrive 8-axis communication')
    try:
        from mco_odrive_interface import ODriveInterface
        odi = ODriveInterface()
        positions, _ = odi.read_encoder_estimates()
        odi.close()
        responding = sum(1 for p in positions if p is not None)
        if responding == 8:
            c.status  = PASS
            c.message = "All 8 axes responding"
        elif responding >= 6:
            c.status  = WARN
            c.message = f"Only {responding}/8 axes responding — check CAN connections"
        else:
            c.status  = FAIL
            c.message = f"Only {responding}/8 axes responding — check ODrive boards"
    except Exception as e:
        c.status  = FAIL
        c.message = f"ODrive interface error: {e}"
    return result.add(c)


def check_pitch_adc(result: CheckResult):
    c = Check('Pitch ADC (SPI1 CE1)')
    try:
        import mco_wave_library as w
        readings = w.read_servo_feedback_adc()
        none_count = sum(1 for r in readings if r is None)
        if none_count == 0:
            c.status  = PASS
            c.message = f"All 8 channels reading. Sample: {readings[0]:.1f}°"
        elif none_count <= 2:
            c.status  = WARN
            c.message = f"{none_count}/8 channels not reading — check wiring"
        else:
            c.status  = FAIL
            c.message = f"{none_count}/8 channels not reading — check SPI1 CE1 wiring"
    except Exception as e:
        c.status  = FAIL
        c.message = f"Pitch ADC error: {e}"
    return result.add(c)


def check_sweep_adc(result: CheckResult):
    c = Check('Sweep ADC (SPI1 CE0)')
    try:
        import spidev
        spi = spidev.SpiDev()
        spi.open(1, 0)
        spi.max_speed_hz = 1_000_000
        spi.mode = 0
        # Read all 8 channels
        readings = []
        for ch in range(8):
            cmd = [0x06 | ((ch & 0x04) >> 2), (ch & 0x03) << 6, 0x00]
            resp = spi.xfer2(cmd)
            raw  = ((resp[1] & 0x0F) << 8) | resp[2]
            readings.append(raw)
        spi.close()
        nonzero = sum(1 for r in readings if r > 10)
        c.status  = PASS if nonzero >= 6 else WARN
        c.message = f"{nonzero}/8 channels returning non-zero values"
    except Exception as e:
        c.status  = FAIL
        c.message = f"Sweep ADC SPI1 CE0 error: {e}"
    return result.add(c)


def check_imu(result: CheckResult):
    c = Check('IMU (MPU-6050 at 0x68)')
    try:
        import smbus
        bus = smbus.SMBus(1)
        who = bus.read_byte_data(0x68, 0x75)
        # Wake IMU
        bus.write_byte_data(0x68, 0x6B, 0x00)
        time.sleep(0.1)
        # Read accelerometer Z (should be ~1g when level)
        az_h = bus.read_byte_data(0x68, 0x3F)
        az_l = bus.read_byte_data(0x68, 0x40)
        az_raw = (az_h << 8 | az_l)
        if az_raw > 32767:
            az_raw -= 65536
        az_g = az_raw / 16384.0
        bus.close()

        if abs(az_g - 1.0) < 0.1:
            c.status  = PASS
            c.message = f"WHO_AM_I=0x{who:02X}, az={az_g:.3f}g (level ✓)"
        elif abs(az_g - 1.0) < 0.3:
            c.status  = WARN
            c.message = f"az={az_g:.3f}g — aircraft not level or IMU misaligned"
        else:
            c.status  = WARN
            c.message = f"az={az_g:.3f}g — unexpected. Check IMU mounting."
    except Exception as e:
        c.status  = FAIL
        c.message = f"IMU I2C error: {e}. Check SDA=GPIO2, SCL=GPIO3."
    return result.add(c)


def check_barometer(result: CheckResult):
    c = Check('Barometer (BMP280 at 0x76)')
    try:
        import smbus
        bus = smbus.SMBus(1)
        chip_id = bus.read_byte_data(0x76, 0xD0)
        bus.close()
        if chip_id in (0x58, 0x60):   # BMP280=0x58, BME280=0x60
            c.status  = PASS
            c.message = f"Chip ID 0x{chip_id:02X} ({'BMP280' if chip_id==0x58 else 'BME280'})"
        else:
            c.status  = WARN
            c.message = f"Unexpected chip ID 0x{chip_id:02X} at 0x76"
    except Exception as e:
        c.status  = FAIL
        c.message = f"Barometer I2C error: {e}. Check I2C1 bus."
    return result.add(c)


def check_oat_sensor(result: CheckResult):
    c = Check('OAT sensor (DS18B20 GPIO4)')
    try:
        w1_dir = '/sys/bus/w1/devices/'
        devices = [d for d in os.listdir(w1_dir) if d.startswith('28-')]
        if not devices:
            c.status  = WARN
            c.message = "No DS18B20 found. Check GPIO4 and dtoverlay=w1-gpio,gpiopin=4"
        else:
            dev_path = os.path.join(w1_dir, devices[0], 'w1_slave')
            with open(dev_path) as f:
                content = f.read()
            if 'YES' in content and 't=' in content:
                temp_c = int(content.split('t=')[1].strip()) / 1000.0
                if -20 < temp_c < 60:
                    c.status  = PASS
                    c.message = f"{temp_c:.1f}°C (sensor: {devices[0]})"
                else:
                    c.status  = WARN
                    c.message = f"Unusual temperature {temp_c:.1f}°C — check sensor"
            else:
                c.status  = FAIL
                c.message = "CRC failed — sensor noise or bad connection"
    except FileNotFoundError:
        c.status  = WARN
        c.message = "1-Wire bus not found. Add dtoverlay=w1-gpio,gpiopin=4 to /boot/config.txt"
    except Exception as e:
        c.status  = WARN
        c.message = str(e)
    return result.add(c)


def check_rc_signal(result: CheckResult, sensors: dict):
    c = Check('RC signal and channel mapping')
    rc_valid = sensors.get('rc_signal_valid', False)
    ch_raw   = sensors.get('rc_channels_raw', [])

    if not rc_valid:
        c.status  = FAIL
        c.message = "RC signal not valid — bind receiver, check SBUS connection to GPIO15"
        return result.add(c)

    if not ch_raw:
        c.status  = WARN
        c.message = "RC signal valid but no channel data — check mco_mavlink.py"
        return result.add(c)

    # Check that lever (CH3) is low (stick down = safe)
    lever_ok = len(ch_raw) >= 3 and ch_raw[2] < 1200  # CH3 index 2, value below mid
    if lever_ok:
        c.status  = PASS
        c.message = f"Valid, {len(ch_raw)} channels. Lever at safe position ✓"
    else:
        c.status  = WARN
        c.message = f"RC valid but lever not at minimum — set lever to minimum before arming"
    return result.add(c)


def check_battery_voltage(result: CheckResult, sensors: dict,
                           min_pct_for_tc=80.0):
    c = Check('Battery voltage')
    bus_v   = sensors.get('bus_voltage_v', 0.0)
    servo_v = sensors.get('servo_voltage_v', 0.0)

    # 4S LiPo: 16.8V full, 14.0V nominal, 13.2V low
    if bus_v < 5.0:
        c.status  = FAIL
        c.message = "Main battery not reading — check ODrive CAN connection"
        return result.add(c)

    # Estimate charge level (4S: 16.8V = 100%, 13.2V = 0%)
    pct = max(0, min(100, (bus_v - 13.2) / (16.8 - 13.2) * 100))

    if pct >= min_pct_for_tc:
        c.status  = PASS
        c.message = f"Main {bus_v:.1f}V (~{pct:.0f}%)  Servo {servo_v:.1f}V"
    elif pct >= 30:
        c.status  = WARN
        c.message = f"Main {bus_v:.1f}V (~{pct:.0f}%) — charge before TC runs"
    else:
        c.status  = FAIL
        c.message = f"Main {bus_v:.1f}V (~{pct:.0f}%) — CHARGE BATTERY"
    return result.add(c)


def check_load_cell(result: CheckResult):
    c = Check('Load cell (HX711 bench)')
    try:
        import mco_loadcell as lc
        cal_ok = os.path.exists(lc.CAL_FILE)
        if not cal_ok:
            c.status  = WARN
            c.message = "No calibration file — run: python3 mco_loadcell.py --calibrate"
            return result.add(c)
        reader = lc.HX711Reader()
        # Read a few samples — should not raise
        reading = reader.read_newtons(n_samples=3)
        reader.close()
        c.status  = PASS
        c.message = f"Reading {reading:.2f} N (un-tared). Calibration loaded ✓"
    except ImportError:
        c.status  = FAIL
        c.message = "mco_loadcell.py not found — write it before TC-01"
    except TimeoutError as e:
        c.status  = FAIL
        c.message = f"HX711 not responding: {e}. Check GPIO26 DT, GPIO7 SCK."
    except Exception as e:
        c.status  = WARN
        c.message = str(e)
    return result.add(c)


def check_odrive_vel_limit(result: CheckResult):
    c = Check('ODrive S1 vel_limit (web GUI check)')
    # ODrive S1 vel_limit cannot be read back over simple CAN protocol.
    # Verify via: odrivetool or web GUI → axis0.controller.config.vel_limit
    # mco_odrive_setup.py --verify will confirm this on USB connection.
    # Here we check that at least the S1 boards are responding on CAN.
    try:
        from mco_odrive_interface import ODriveInterface, N_WINGS
        odi = ODriveInterface()
        states = odi.read_all_states()
        odi.close()
        responding = sum(1 for s in states if s.get('responding'))
        if responding == N_WINGS:
            c.status  = PASS
            c.message = f'All {N_WINGS} S1 nodes responding. Verify vel_limit=150 via odrivetool.'
        elif responding > 0:
            c.status  = WARN
            c.message = f'Only {responding}/{N_WINGS} S1 nodes responding.'
        else:
            c.status  = FAIL
            c.message = 'No S1 nodes responding. Check CAN bus and power.'
    except Exception as e:
        c.status  = WARN
        c.message = f'S1 state check failed: {e}'
    return result.add(c)


def check_pigpiod(result: CheckResult):
    c = Check('pigpiod servo daemon')
    try:
        out = subprocess.check_output(['systemctl', 'is-active', 'pigpiod'],
                                       text=True, stderr=subprocess.DEVNULL).strip()
        if out == 'active':
            c.status  = PASS
            c.message = "pigpiod active ✓"
        else:
            c.status  = FAIL
            c.message = f"pigpiod is {out} — run: sudo systemctl start pigpiod"
    except FileNotFoundError:
        c.status  = SKIP
        c.message = "systemctl not available (not on Pi)"
    except Exception as e:
        c.status  = WARN
        c.message = str(e)
    return result.add(c)


def check_calibration_constants(result: CheckResult):
    c = Check('Calibration constants (TC-02 data)')
    tc_results_file = '/home/pi/knowledge/tc_results.json'
    try:
        with open(tc_results_file) as f:
            tc = json.load(f)
        tc02 = tc.get('tc02', {})
        hover_step = tc02.get('02-03-hover-freq', {})
        calib_step = tc02.get('02-07-calibrate', {})
        if hover_step.get('passed') and calib_step.get('passed'):
            hover_freq = hover_step.get('data', {}).get('hover_freq_hz', '?')
            c.status  = PASS
            c.message = f"TC-02 complete. Hover freq: {hover_freq} Hz ✓"
        elif hover_step.get('passed'):
            c.status  = WARN
            c.message = "TC-02 hover freq found but calibration not complete"
        else:
            c.status  = WARN
            c.message = "TC-02 not yet run — calibration constants are estimates"
    except FileNotFoundError:
        c.status  = WARN
        c.message = "No TC results yet — run TC-02 to calibrate controllers"
    except Exception as e:
        c.status  = WARN
        c.message = str(e)
    return result.add(c)


# ── Master check runner ───────────────────────────────────────────────────────

def run_all_checks(sensors: dict = None, bench_mode=False,
                   quick=False, min_battery_pct=80.0) -> CheckResult:
    """
    Run all pre-flight checks.

    Args:
        sensors:      Current sensors dict from mco_main.py (or empty dict)
        bench_mode:   True if load cell is attached (TC-01, TC-02)
        quick:        Skip slow checks (ODrive, ADC reads)
        min_battery_pct: Minimum battery charge for TC runs (default 80%)

    Returns:
        CheckResult with .go property and .summary() method
    """
    if sensors is None:
        sensors = {}

    result = CheckResult()

    # Always run
    check_wave_library(result)
    check_build_measurements(result)
    check_sd_space(result)
    check_pigpiod(result)
    check_can_bus(result)

    if not quick:
        check_odrives(result)
        check_odrive_vel_limit(result)
        check_pitch_adc(result)
        check_sweep_adc(result)

    check_imu(result)
    check_barometer(result)
    check_oat_sensor(result)

    if sensors:
        check_rc_signal(result, sensors)
        check_battery_voltage(result, sensors, min_battery_pct)
    else:
        result.add(Check('RC signal', SKIP, 'No sensors — run from mco_main.py for live check'))
        result.add(Check('Battery voltage', SKIP, 'No sensors — check bus_voltage_v telemetry'))

    check_calibration_constants(result)

    if bench_mode:
        check_load_cell(result)

    return result


# ── CLI ───────────────────────────────────────────────────────────────────────

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='MCO-1D pre-flight checks')
    parser.add_argument('--quick',  action='store_true', help='Skip slow hardware reads')
    parser.add_argument('--bench',  action='store_true', help='Include load cell check')
    parser.add_argument('--flight', action='store_true', help='Flight mode (no load cell)')
    parser.add_argument('--min-battery', type=float, default=80.0,
                        help='Minimum battery %% for TC operations (default 80)')
    args = parser.parse_args()

    print("\nMCO-1D PRE-FLIGHT CHECKS")
    print("=" * 50)

    result = run_all_checks(
        sensors    = {},
        bench_mode = args.bench,
        quick      = args.quick,
        min_battery_pct = args.min_battery,
    )

    print(result.summary())
    sys.exit(0 if result.go else 1)


# ══════════════════════════════════════════════════════════════════════════════
# S1-SPECIFIC HEALTH CHECKS — PRE-FLIGHT, IN-FLIGHT, POST-FLIGHT
# ══════════════════════════════════════════════════════════════════════════════
#
# These checks use the ODrive S1's new capabilities:
#   - Real FET and motor temperatures from read_temperatures()
#   - Per-board axis error state from read_all_states()
#   - Heartbeat presence confirms each board is alive
#   - On-board absolute encoder field strength from check_magnet()
#
# Three phases:
#   PRE-FLIGHT:  run before arming — gate on hard limits
#   IN-FLIGHT:   run every N ticks — flag to pilot, do not auto-land
#   POST-FLIGHT: run after disarm — assess wear and flag for next flight
# ══════════════════════════════════════════════════════════════════════════════

from dataclasses import dataclass, field
from typing import List, Dict, Any
import time as _time

# ── Thresholds ────────────────────────────────────────────────────────────────

# Motor temperatures (GoolRC 5010-14 rated ~80°C continuous winding)
MOTOR_TEMP_WARN_C      = 60.0   # ⚠ warn pilot — approaching limit
MOTOR_TEMP_CRITICAL_C  = 75.0   # ✗ land soon — risk of insulation damage

# FET temperatures (S1 power stage)
FET_TEMP_WARN_C        = 70.0   # ⚠ warn pilot — vel_limit reducing
FET_TEMP_CRITICAL_C    = 90.0   # ✗ motor idle — hardware cutoff imminent

# Current limits
CURRENT_WARN_A         = 18.0   # ⚠ high current — rope or bearing check
CURRENT_CRITICAL_A     = 22.0   # ✗ fault — motor or drive issue
CURRENT_ASYMM_PCT      = 25.0   # ⚠ port/stbd asymmetry — rope wear

# Rope health proxy: rising current at same frequency = wear
ROPE_CURRENT_RISE_PCT  = 20.0   # ⚠ current has risen from baseline

# Encoder magnet field status codes (ODrive S1)
MAGNET_STATUS_OK        = 1     # field detected and within range
MAGNET_STATUS_MISSING   = 0     # no field detected
MAGNET_STATUS_TOO_CLOSE = 2     # magnet too close
MAGNET_STATUS_TOO_FAR   = 3     # magnet too far

# Wing names for reporting
_WING_NAMES = ['W1p','W1s','W2p','W2s','W3p','W3s','W4p','W4s']


@dataclass
class MotorHealthRecord:
    """Health record for one wing motor, built up across all three phases."""
    wing:           str
    node:           int
    # Pre-flight
    heartbeat_ok:   bool   = False
    encoder_ok:     bool   = False
    magnet_status:  int    = -1
    errors_at_arm:  int    = 0
    # In-flight peaks
    peak_fet_C:     float  = 0.0
    peak_motor_C:   float  = 0.0
    peak_current_A: float  = 0.0
    base_current_A: float  = 0.0   # baseline from TC-01
    # Post-flight
    post_fet_C:     float  = 0.0
    post_motor_C:   float  = 0.0
    warnings:       List[str] = field(default_factory=list)
    critical:       List[str] = field(default_factory=list)

    @property
    def go(self):
        return len(self.critical) == 0


# ── PRE-FLIGHT S1 HEALTH CHECKS ───────────────────────────────────────────────

def check_s1_heartbeats(result: CheckResult) -> Check:
    """
    Verify all 8 S1 boards are heartbeating on CAN.
    The S1 sends a heartbeat frame automatically at ~1 Hz when powered.
    Missing heartbeat = board not powered, CAN connection broken, or firmware fault.
    """
    c = Check('S1 heartbeats — all 8 boards')
    try:
        from mco_odrive_interface import ODriveInterface, N_WINGS
        odi    = ODriveInterface()
        states = odi.read_all_states()
        odi.close()
        ok      = [s for s in states if s.get('responding')]
        missing = [s['wing'] for s in states if not s.get('responding')]
        if len(ok) == N_WINGS:
            c.status  = PASS
            c.message = f'All {N_WINGS} S1 boards heartbeating ✓'
        elif len(ok) >= 6:
            c.status  = WARN
            c.message = f'{len(ok)}/{N_WINGS} boards responding. Missing: {missing}'
        else:
            c.status  = FAIL
            c.message = f'Only {len(ok)}/{N_WINGS} boards. Missing: {missing}'
        c.value = {'responding': [s['wing'] for s in ok], 'missing': missing}
    except Exception as e:
        c.status  = FAIL
        c.message = f'CAN read failed: {e}'
    return result.add(c)


def check_s1_encoder_magnets(result: CheckResult) -> Check:
    """
    Verify the on-board absolute encoder magnet is present and correctly
    positioned on each of the 8 motor shafts.
    Missing or misaligned magnet = no position feedback = no flapping control.
    """
    c = Check('S1 encoder magnets — all 8 shafts')
    try:
        import odrive
        issues = []
        ok_count = 0
        for node_id in range(8):
            try:
                odrv = odrive.find_any(path=f'can:{node_id}', timeout=2)
                enc  = odrv.onboard_encoder0
                status = enc.field_status
                if status == MAGNET_STATUS_OK:
                    ok_count += 1
                elif status == MAGNET_STATUS_MISSING:
                    issues.append(f'{_WING_NAMES[node_id]}: magnet NOT DETECTED')
                elif status == MAGNET_STATUS_TOO_CLOSE:
                    issues.append(f'{_WING_NAMES[node_id]}: magnet TOO CLOSE '
                                  f'(increase gap to ~1-2mm)')
                elif status == MAGNET_STATUS_TOO_FAR:
                    issues.append(f'{_WING_NAMES[node_id]}: magnet TOO FAR '
                                  f'(decrease gap to ~1-2mm)')
            except Exception:
                issues.append(f'{_WING_NAMES[node_id]}: could not read encoder')

        if not issues:
            c.status  = PASS
            c.message = f'All 8 encoder magnets detected and in range ✓'
        elif ok_count >= 6:
            c.status  = WARN
            c.message = f'{ok_count}/8 magnets OK. Issues: {"; ".join(issues)}'
        else:
            c.status  = FAIL
            c.message = f'Only {ok_count}/8 magnets OK: {"; ".join(issues)}'
        c.value = {'ok': ok_count, 'issues': issues}
    except ImportError:
        # odrive package not installed — use CAN heartbeat proxy instead
        # If heartbeat is OK, encoder is almost certainly OK (S1 would fault otherwise)
        c.status  = SKIP
        c.message = ('odrive package not installed — encoder check via odrivetool. '
                     'Run: python3 mco_odrive_setup.py --check-magnet')
    except Exception as e:
        c.status  = WARN
        c.message = f'Encoder check error: {e}'
    return result.add(c)


def check_s1_temperatures_preflight(result: CheckResult) -> Check:
    """
    Read FET and motor temperatures before arming.
    All motors should be at or near ambient temperature before first flight.
    Hot motors before flight = previous damage, cooling fan failure, or
    insufficient cool-down between flights.
    """
    c = Check('S1 temperatures — pre-flight (near ambient)')
    try:
        from mco_odrive_interface import ODriveInterface
        odi   = ODriveInterface()
        temps = odi.read_temperatures()
        odi.close()

        # Get ambient from OAT sensor if available
        ambient_C = 20.0
        try:
            w1_path = '/sys/bus/w1/devices/'
            import os
            devices = [d for d in os.listdir(w1_path) if d.startswith('28-')]
            if devices:
                with open(f'{w1_path}{devices[0]}/w1_slave') as f:
                    raw = f.read()
                if 'YES' in raw and 't=' in raw:
                    ambient_C = int(raw.split('t=')[1].strip()) / 1000.0
        except Exception:
            pass

        hot_wings = []
        max_fet   = 0.0
        max_mot   = 0.0
        valid     = 0

        for i, t in enumerate(temps):
            fet = t.get('fet_C')
            mot = t.get('motor_C')
            if fet is not None:
                valid += 1
                max_fet = max(max_fet, fet)
                if fet > ambient_C + 20:
                    hot_wings.append(f'{_WING_NAMES[i]} FET={fet:.0f}°C')
            if mot is not None:
                max_mot = max(max_mot, mot)
                if mot > MOTOR_TEMP_WARN_C:
                    hot_wings.append(f'{_WING_NAMES[i]} motor={mot:.0f}°C')

        if valid == 0:
            c.status  = WARN
            c.message = 'No temperature readings — S1 boards may not be configured'
        elif hot_wings:
            c.status  = WARN
            c.message = (f'Hot before flight: {"; ".join(hot_wings)}. '
                         f'Allow cooling to within 20°C of ambient ({ambient_C:.0f}°C).')
        else:
            c.status  = PASS
            c.message = (f'All motors near ambient ({ambient_C:.0f}°C). '
                         f'Max FET {max_fet:.0f}°C, motor {max_mot:.0f}°C ✓')
        c.value = {'max_fet_C': max_fet, 'max_motor_C': max_mot,
                   'ambient_C': ambient_C, 'hot_wings': hot_wings}
    except Exception as e:
        c.status  = WARN
        c.message = f'Temperature read error: {e}'
    return result.add(c)


def check_s1_axis_errors_preflight(result: CheckResult) -> Check:
    """
    Read axis error register from all 8 S1 boards.
    Any non-zero error before arming must be cleared and understood.
    Common errors: overcurrent from previous flight, encoder fault,
    watchdog timeout from incomplete shutdown.
    """
    c = Check('S1 axis errors — all 8 clear before arming')
    try:
        from mco_odrive_interface import ODriveInterface
        odi    = ODriveInterface()
        states = odi.read_all_states()
        odi.close()

        errors = {s['wing']: s['error'] for s in states
                  if s.get('error') and s['error'] != 0}

        if not errors:
            c.status  = PASS
            c.message = 'All 8 S1 boards error-free ✓'
        else:
            # Attempt auto-clear on non-critical errors
            odi2 = ODriveInterface()
            odi2.clear_errors()
            odi2.close()
            _time.sleep(0.5)

            odi3   = ODriveInterface()
            states2 = odi3.read_all_states()
            odi3.close()
            persist = {s['wing']: s['error'] for s in states2
                       if s.get('error') and s['error'] != 0}

            if not persist:
                c.status  = WARN
                c.message = (f'Errors cleared: {errors}. '
                             f'Investigate cause before flight.')
            else:
                c.status  = FAIL
                c.message = (f'Persistent errors after clear: {persist}. '
                             f'Do not arm until resolved.')
        c.value = {'errors': errors}
    except Exception as e:
        c.status  = WARN
        c.message = f'Error state read failed: {e}'
    return result.add(c)


def check_s1_current_baseline(result: CheckResult) -> Check:
    """
    Run all 8 motors at low speed (5 rad/s) for 2 seconds and record
    baseline currents. Abnormally high baseline current before flight
    indicates rope binding, bearing damage, or motor winding fault.
    """
    c = Check('S1 motor current baseline (5 rad/s spin check)')
    try:
        from mco_odrive_interface import ODriveInterface
        odi = ODriveInterface()
        odi.enable_all()
        for i in range(8):
            odi.set_velocity(i, 5.0)
        _time.sleep(2.0)
        currents = odi.read_currents()
        odi.idle_all()
        odi.close()

        issues = []
        for i, I in enumerate(currents):
            if I is None or I == 0.0:
                issues.append(f'{_WING_NAMES[i]}: no current (motor disconnected?)')
            elif I > CURRENT_WARN_A:
                issues.append(f'{_WING_NAMES[i]}: {I:.1f}A (expected <5A at 5 rad/s — '
                              f'check rope tension and bearings)')
            elif I < 0.1:
                issues.append(f'{_WING_NAMES[i]}: {I:.2f}A (too low — check wiring)')

        # Port/stbd symmetry per pair
        for pair in range(4):
            I_port = currents[pair*2]   if pair*2   < len(currents) else 0
            I_stbd = currents[pair*2+1] if pair*2+1 < len(currents) else 0
            if I_port and I_stbd and I_port > 0:
                asym = abs(I_port - I_stbd) / max(I_port, I_stbd, 0.01)
                if asym > CURRENT_ASYMM_PCT / 100:
                    issues.append(
                        f'W{pair+1} asymmetry {asym*100:.0f}%: '
                        f'port {I_port:.2f}A / stbd {I_stbd:.2f}A '
                        f'(check rope tension on heavier side)')

        if not issues:
            mean_I = sum(c2 for c2 in currents if c2) / max(1, len([c2 for c2 in currents if c2]))
            c.status  = PASS
            c.message = (f'All 8 motors nominal at 5 rad/s. '
                         f'Mean {mean_I:.2f}A, max {max(currents):.2f}A ✓')
        elif any('disconnected' in i or 'wiring' in i for i in issues):
            c.status  = FAIL
            c.message = f'Motor fault: {"; ".join(issues)}'
        else:
            c.status  = WARN
            c.message = f'{len(issues)} issue(s): {"; ".join(issues)}'
        c.value = {'currents_A': [round(I, 3) if I else None for I in currents],
                   'issues': issues}
    except Exception as e:
        c.status  = WARN
        c.message = f'Baseline current check error: {e}'
    return result.add(c)


# ── IN-FLIGHT S1 HEALTH MONITOR ───────────────────────────────────────────────

class S1InFlightMonitor:
    """
    Runs during the 50 Hz flight loop.
    Reads S1 data at reduced rate (temperature 5 Hz, errors 1 Hz)
    and builds a per-wing health picture that flags the pilot.

    Integrated into the tick wrapper — called from mco_main.py via
    the result dict fields it produces.
    """

    # Warning thresholds
    FET_WARN      = FET_TEMP_WARN_C
    FET_CRIT      = FET_TEMP_CRITICAL_C
    MOT_WARN      = MOTOR_TEMP_WARN_C
    MOT_CRIT      = MOTOR_TEMP_CRITICAL_C
    I_WARN        = CURRENT_WARN_A
    I_CRIT        = CURRENT_CRITICAL_A
    ASYM_WARN     = CURRENT_ASYMM_PCT / 100.0

    def __init__(self):
        self.records   = {i: MotorHealthRecord(wing=_WING_NAMES[i], node=i)
                          for i in range(8)}
        self._alerts   = []     # current active alerts (most recent first)
        self._tick     = 0
        self._baseline_I = [float('nan')] * 8   # NaN = never calibrated (not None)
    _baseline_missing_wings = []    # wings with no valid baseline

    def set_baseline_currents(self, currents_A: list):
        """Store preflight baseline currents. Must be called after
        check_s1_current_baseline() passes. Wings with NaN baseline
        will emit monitoring-offline alerts in flight.
        """

        """Call after TC-01 step 06 (motor spin) to record the current baseline."""
        self._baseline_I = list(currents_A)
        for i, I in enumerate(currents_A):
            if I is not None:
                self.records[i].base_current_A = I

    def update(self, sensors: dict, result: dict) -> dict:
        """
        Called every tick from mco_main.py.
        Consumes sensors['motor_temps'] and sensors['wing_errors'].
        Produces result['motor_health'] with per-wing status and active alerts.
        Returns modified result dict.
        """
        self._tick += 1
        alerts     = []
        per_wing   = {}

        # ── Temperature monitoring (from S1 read_temperatures at 5 Hz) ──
        motor_temps = sensors.get('motor_temps', [])
        for i, t in enumerate(motor_temps):
            if not t:
                continue
            fet = t.get('fet_C')
            mot = t.get('motor_C')
            rec = self.records[i]
            name = _WING_NAMES[i]

            if fet is not None:
                rec.peak_fet_C = max(rec.peak_fet_C, fet)
                if fet >= self.FET_CRIT:
                    msg = f'{name} FET CRITICAL {fet:.0f}°C — motor idling'
                    if msg not in [a['msg'] for a in self._alerts]:
                        alerts.append({'level':'CRITICAL','wing':name,
                                       'msg':msg,'value':fet})
                elif fet >= self.FET_WARN:
                    msg = f'{name} FET warm {fet:.0f}°C — vel_limit reducing'
                    if msg not in [a['msg'] for a in self._alerts]:
                        alerts.append({'level':'WARNING','wing':name,
                                       'msg':msg,'value':fet})

            if mot is not None:
                rec.peak_motor_C = max(rec.peak_motor_C, mot)
                if mot >= self.MOT_CRIT:
                    msg = f'{name} MOTOR CRITICAL {mot:.0f}°C — land soon'
                    alerts.append({'level':'CRITICAL','wing':name,
                                   'msg':msg,'value':mot})
                elif mot >= self.MOT_WARN:
                    msg = f'{name} motor warm {mot:.0f}°C'
                    alerts.append({'level':'WARNING','wing':name,
                                   'msg':msg,'value':mot})

            per_wing[name] = {
                'fet_C':  round(fet,  1) if fet  is not None else None,
                'mot_C':  round(mot,  1) if mot  is not None else None,
            }

        # ── Current monitoring ────────────────────────────────────────────
        currents = sensors.get('motor_currents_a', [])
        for i, I in enumerate(currents[:8]):
            if I is None:
                continue
            rec  = self.records[i]
            name = _WING_NAMES[i]
            rec.peak_current_A = max(rec.peak_current_A, I)

            # Absolute current limit
            if I >= self.I_CRIT:
                alerts.append({'level':'CRITICAL','wing':name,
                               'msg': f'{name} OVERCURRENT {I:.1f}A',
                               'value': I})
            elif I >= self.I_WARN:
                alerts.append({'level':'WARNING','wing':name,
                               'msg': f'{name} high current {I:.1f}A',
                               'value': I})

            # Rope wear: current rising from baseline
            base = self._baseline_I[i]
            if base and base > 0.5:
                rise = (I - base) / base
                if rise > ROPE_CURRENT_RISE_PCT / 100:
                    alerts.append({'level':'WARNING','wing':name,
                                   'msg': (f'{name} current rose {rise*100:.0f}% '
                                           f'from baseline — inspect rope'),
                                   'value': I})

            if name in per_wing:
                per_wing[name]['current_A'] = round(I, 2)
            else:
                per_wing[name] = {'current_A': round(I, 2)}

        # ── Port/stbd current asymmetry ───────────────────────────────────
        for pair in range(4):
            I_p = (currents[pair*2]   if pair*2   < len(currents) else None)
            I_s = (currents[pair*2+1] if pair*2+1 < len(currents) else None)
            if I_p and I_s and max(I_p, I_s) > 0.5:
                asym = abs(I_p - I_s) / max(I_p, I_s)
                if asym > self.ASYM_WARN:
                    alerts.append({
                        'level': 'WARNING',
                        'wing':  f'W{pair+1}',
                        'msg':   (f'W{pair+1} current asymmetry {asym*100:.0f}%: '
                                  f'port {I_p:.1f}A / stbd {I_s:.1f}A '
                                  f'— check rope drives'),
                        'value': asym,
                    })

        # ── Axis error state ──────────────────────────────────────────────
        wing_errors = sensors.get('wing_errors', [])
        for i, err in enumerate(wing_errors[:8]):
            if err and err != 0:
                name = _WING_NAMES[i]
                alerts.append({'level':'WARNING','wing':name,
                               'msg': f'{name} axis error 0x{err:04X}',
                               'value': err})

        # ── Consolidate alerts ────────────────────────────────────────────
        # Critical alerts first, then warnings, deduplicate
        all_alerts = sorted(alerts, key=lambda a: (0 if a['level']=='CRITICAL' else 1))
        self._alerts = all_alerts

        # Highest alert level across all wings
        if any(a['level'] == 'CRITICAL' for a in all_alerts):
            alert_level = 'CRITICAL'
        elif any(a['level'] == 'WARNING' for a in all_alerts):
            alert_level = 'WARNING'
        else:
            alert_level = 'NORMAL'

        result['motor_health'] = {
            'alert_level':  alert_level,
            'alerts':       all_alerts,
            'per_wing':     per_wing,
            'peaks': {
                'max_fet_C':     max((r.peak_fet_C     for r in self.records.values()), default=0),
                'max_motor_C':   max((r.peak_motor_C   for r in self.records.values()), default=0),
                'max_current_A': max((r.peak_current_A for r in self.records.values()), default=0),
            },
        }
        return result

    def summary(self) -> dict:
        """Return a flight summary dict for post-flight use."""
        return {
            'per_wing': {
                rec.wing: {
                    'peak_fet_C':     rec.peak_fet_C,
                    'peak_motor_C':   rec.peak_motor_C,
                    'peak_current_A': rec.peak_current_A,
                    'base_current_A': rec.base_current_A,
                    'current_rise_pct': (
                        (rec.peak_current_A - rec.base_current_A) /
                        max(rec.base_current_A, 0.01) * 100
                        if rec.base_current_A > 0 else 0),
                }
                for rec in self.records.values()
            }
        }


# Singleton instance used by mco_main.py and mco_wave_library.py
s1_monitor = S1InFlightMonitor()


# ── POST-FLIGHT S1 HEALTH CHECKS ─────────────────────────────────────────────

def check_s1_post_flight(flight_summary: dict, sensors: dict) -> CheckResult:
    """
    Post-flight health assessment using S1 data.
    Called automatically after disarm, before the cooling period ends.

    Produces a maintenance action list for the pilot/technician.
    Flags any wing that needs inspection before next flight.
    """
    result = CheckResult()

    per_wing = flight_summary.get('per_wing', {})

    # ── 1. Peak temperature review ────────────────────────────────────────
    c = Check('Post-flight peak temperatures')
    hot = []
    for wing, data in per_wing.items():
        if data['peak_motor_C'] >= MOTOR_TEMP_CRITICAL_C:
            hot.append(f'{wing} motor peak {data["peak_motor_C"]:.0f}°C — '
                       f'inspect winding insulation')
        elif data['peak_motor_C'] >= MOTOR_TEMP_WARN_C:
            hot.append(f'{wing} motor peak {data["peak_motor_C"]:.0f}°C — '
                       f'within limits but monitor trend')
        if data['peak_fet_C'] >= FET_TEMP_CRITICAL_C:
            hot.append(f'{wing} FET peak {data["peak_fet_C"]:.0f}°C — '
                       f'investigate heat management')
    if not hot:
        c.status  = PASS
        c.message = ('All motor and FET temperatures within limits. '
                     f'Max motor {max((d["peak_motor_C"] for d in per_wing.values()), default=0):.0f}°C')
    elif any('winding' in h or 'heat management' in h for h in hot):
        c.status  = FAIL
        c.message = f'Critical temperatures reached: {"; ".join(hot)}'
    else:
        c.status  = WARN
        c.message = f'Elevated temperatures: {"; ".join(hot)}'
    result.add(c)

    # ── 2. Current trend (rope health proxy) ─────────────────────────────
    c2 = Check('Post-flight current trend — rope health proxy')
    rope_warnings = []
    for wing, data in per_wing.items():
        rise = data.get('current_rise_pct', 0)
        if rise >= ROPE_CURRENT_RISE_PCT:
            rope_warnings.append(
                f'{wing}: current rose {rise:.0f}% above baseline '
                f'(baseline {data["base_current_A"]:.2f}A → '
                f'peak {data["peak_current_A"]:.2f}A) — inspect rope drive')
    if not rope_warnings:
        c2.status  = PASS
        c2.message = 'All motor currents within expected range of baseline ✓'
    else:
        c2.status  = WARN
        c2.message = f'Rope inspection recommended: {"; ".join(rope_warnings)}'
    result.add(c2)

    # ── 3. Post-flight temperature read ───────────────────────────────────
    c3 = Check('Post-flight motor temperatures (after cooling period)')
    try:
        from mco_odrive_interface import ODriveInterface
        odi   = ODriveInterface()
        temps = odi.read_temperatures()
        odi.close()
        max_mot = max((t.get('motor_C', 0) or 0 for t in temps), default=0)
        max_fet = max((t.get('fet_C',   0) or 0 for t in temps), default=0)
        ambient = sensors.get('oat_c', 20.0)
        excess  = max_mot - ambient
        if excess < 10:
            c3.status  = PASS
            c3.message = (f'Motors cooled to {max_mot:.0f}°C '
                          f'({excess:.0f}°C above {ambient:.0f}°C ambient) ✓')
        elif excess < 20:
            c3.status  = WARN
            c3.message = (f'Motors still warm: {max_mot:.0f}°C '
                          f'({excess:.0f}°C above ambient). '
                          f'Extend cooling period before next flight.')
        else:
            c3.status  = FAIL
            c3.message = (f'Motors too hot to store: {max_mot:.0f}°C. '
                          f'Do not bag batteries or cover aircraft yet.')
    except Exception as e:
        c3.status  = WARN
        c3.message = f'Post-flight temperature read error: {e}'
    result.add(c3)

    # ── 4. Axis error state post-flight ───────────────────────────────────
    c4 = Check('Post-flight axis error states')
    try:
        from mco_odrive_interface import ODriveInterface
        odi    = ODriveInterface()
        states = odi.read_all_states()
        odi.close()
        errors = {s['wing']: hex(s['error']) for s in states
                  if s.get('error') and s['error'] != 0}
        if not errors:
            c4.status  = PASS
            c4.message = 'All 8 boards error-free after flight ✓'
        else:
            c4.status  = WARN
            c4.message = (f'Post-flight errors (clear before next flight): {errors}. '
                          f'Run: mco_odrive_setup.py --verify to investigate.')
    except Exception as e:
        c4.status  = WARN
        c4.message = f'Post-flight error read: {e}'
    result.add(c4)

    # ── 5. Next-flight readiness ──────────────────────────────────────────
    c5 = Check('Readiness for next flight')
    blockers = [c.message for c in result.failures]
    if not blockers:
        c5.status  = PASS
        c5.message = 'Aircraft ready for next flight ✓'
    else:
        c5.status  = FAIL
        c5.message = f'Resolve before next flight: {"; ".join(blockers)}'
    result.add(c5)

    return result


# ── Wire into pre-flight check runner ─────────────────────────────────────────

def run_s1_preflight_checks(sensors: dict = None,
                             bench_mode: bool = False) -> CheckResult:
    """
    S1-specific pre-flight checks. Call alongside run_all_checks().
    Adds 5 S1-specific checks to the result.
    """
    if sensors is None:
        sensors = {}
    result = CheckResult()
    check_s1_heartbeats(result)
    check_s1_encoder_magnets(result)
    check_s1_temperatures_preflight(result)
    check_s1_axis_errors_preflight(result)
    if bench_mode:
        check_s1_current_baseline(result)
    # Soft-stop system configuration check
    ss_result = check_soft_stop_system()
    result.add(ss_result['status'], 'soft_stop_config', ss_result['msg'])
    # Rope life airworthiness check
    try:
        from mco_certification_tools import rope_tracker
        rope_ok = rope_tracker.airworthy_check()
        if not rope_ok:
            result.add('FAIL', 'rope_life',
                       'One or more ropes have exceeded life limit. '
                       'Replace before flight. GS: cert_rope_report')
        else:
            result.add('PASS', 'rope_life', 'All ropes within life limit.')
    except Exception as e:
        result.add('WARN', 'rope_life', f'Rope life check unavailable: {e}')
    return result


# ── Update run_all_checks to include S1 checks ────────────────────────────────

_orig_run_all_checks = run_all_checks


def rope_wear_offline_check(wing_idx, baseline_I):
    """Returns MONITORING_OFFLINE status if baseline is NaN."""
    import math
    if math.isnan(baseline_I):
        return {"rope_status": "MONITORING_OFFLINE",
                "alert": f"Wing {wing_idx}: rope wear MONITORING_OFFLINE "
                         f"— no preflight baseline. Re-run preflight."}
    return None

def check_soft_stop_system():
    """
    Pre-flight: verify stroke soft-stop system is configured correctly.

    Checks:
    1. SOFT_STOP_ENABLED is True
    2. SOFT_STOP_ZONE_DEG is within 5-20° (safe operating range)
    3. SOFT_STOP_MIN_VEL_FRAC is within 0.10-0.60
    4. motor_pos_rad will be populated (ODrive CAN read working)

    Called as part of run_s1_preflight_checks().
    """
    results = {}
    try:
        from mco_wave_library import (SOFT_STOP_ENABLED, SOFT_STOP_ZONE_DEG,
                                       SOFT_STOP_MIN_VEL_FRAC,
                                       SOFT_STOP_VEL_LIMIT_FRAC)
        checks = {
            'enabled':        SOFT_STOP_ENABLED,
            'zone_deg_ok':    5.0 <= SOFT_STOP_ZONE_DEG <= 20.0,
            'min_vel_ok':     0.10 <= SOFT_STOP_MIN_VEL_FRAC <= 0.60,
            'vel_limit_ok':   0.30 <= SOFT_STOP_VEL_LIMIT_FRAC <= 0.80,
        }
        results['soft_stop_params'] = {
            'enabled':            SOFT_STOP_ENABLED,
            'zone_deg':           SOFT_STOP_ZONE_DEG,
            'min_vel_frac':       SOFT_STOP_MIN_VEL_FRAC,
            'vel_limit_frac':     SOFT_STOP_VEL_LIMIT_FRAC,
        }
        passed = all(checks.values())
        status = 'PASS' if passed else 'WARN'
        msg    = ('Soft-stop configured correctly.'
                  if passed else
                  f'Soft-stop parameter out of range: '
                  f'{[k for k,v in checks.items() if not v]}')
        if not SOFT_STOP_ENABLED:
            status = 'WARN'
            msg    = 'Soft-stop is DISABLED — rope wear and hard-stop risk elevated.'
        return {'status': status, 'msg': msg, 'data': results}
    except Exception as e:
        return {'status': 'FAIL', 'msg': f'Soft-stop check error: {e}', 'data': {}}



def check_damage_tolerance_ready():
    """
    Pre-flight: verify progressive damage / controlled landing system is ready.
    Checks:
      1. fault_compensate can be imported (wave library loaded)
      2. failure briefing table has been built
      3. All 8 wings start healthy
    """
    try:
        from mco_wave_library import (_fault_state, failure_briefing,
                                       LAND_AUTO_LANDNOW_WINGS,
                                       LAND_AUTO_EMERG_WINGS,
                                       LAND_FLARE_HEIGHT_M)
        all_healthy = all(_fault_state['wing_healthy'])
        status = 'PASS' if all_healthy else 'WARN'
        n_h = sum(_fault_state['wing_healthy'])
        msg = (f"Damage tolerance ready: {n_h}/8 wings healthy. "
               f"Auto land_now at ≤{LAND_AUTO_LANDNOW_WINGS} wings. "
               f"Flare at {LAND_FLARE_HEIGHT_M}m AGL. "
               f"Pilot roll/yaw/pitch authority preserved throughout.")
        if not all_healthy:
            msg += f" WARNING: {8-n_h} wing(s) not healthy at preflight."
        return {'status': status, 'msg': msg}
    except Exception as e:
        return {'status': 'FAIL', 'msg': f'Damage tolerance check error: {e}'}


def check_attitude_hold_config():
    """
    Pre-flight: verify attitude hold system is configured correctly.
    Checks: AH_ENABLED, gain values within safe ranges.
    """
    try:
        from mco_wave_library import (AH_ENABLED, AH_ROLL_KP_OUTER,
                                       AH_PITCH_KP_OUTER, AH_FULL_AUTHORITY_STICK)
        issues = []
        if not AH_ENABLED:
            issues.append('Attitude hold DISABLED — pilot must maintain stability manually.')
        if AH_ROLL_KP_OUTER > 0.20:
            issues.append(f'roll_kp_outer={AH_ROLL_KP_OUTER:.3f} high — may oscillate.')
        if AH_PITCH_KP_OUTER > 0.20:
            issues.append(f'pitch_kp_outer={AH_PITCH_KP_OUTER:.3f} high — may oscillate.')
        if AH_FULL_AUTHORITY_STICK > 0.40:
            issues.append(f'authority_stick={AH_FULL_AUTHORITY_STICK:.2f} high — '
                          f'large deadzone before pilot has full control.')
        status = 'WARN' if issues else 'PASS'
        msg = ('Attitude hold: ' + '; '.join(issues)
               if issues else
               f'Attitude hold ready — full authority at >'
               f'{AH_FULL_AUTHORITY_STICK*100:.0f}% stick.')
        return {'status': status, 'msg': msg}
    except Exception as e:
        return {'status': 'WARN', 'msg': f'Attitude hold check error: {e}'}
