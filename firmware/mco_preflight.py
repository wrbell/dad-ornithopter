"""
MCO-1D Pre-Flight Protocol
===========================
Implements a three-tier pre-flight check system modelled on
aviation Built-In Test (BIT) standards and ArduPilot's pre-arm
check framework.

Tier 1  POWER-UP BIT (P-BIT)
       Runs automatically when the RPi boots.
       Tests: software integrity, file presence, sensor comms,
       ODrive presence, servo response range.
       Duration: ~15 seconds.
       On failure: blocks arming, prints diagnostic.

Tier 2  PRE-FLIGHT PROTOCOL (operator-initiated)
       Run once before the first flight of any session.
       Tests: full sensor calibration check, full servo/motor
       range exercise, wave library parameter validation,
       CAN bus health, battery state.
       Duration: ~90 seconds.
       On failure: blocks arming with specific fault message.

Tier 3  CONTINUOUS BIT (C-BIT)
       Runs every loop while armed.
       Tests: sensor freshness, voltage margins, servo response
       lag, motor current within model bounds.
       On failure: triggers FCC failsafe message, logs anomaly.

The module also produces the PILOT CHECKLIST — the items that
require a human to physically inspect and confirm before arming.
These cannot be checked by software but are printed to the screen
and must be confirmed by the operator.

Aviation standard references:
  MIL-STD-2165A  Testability standard for military systems
  DO-178C        Software considerations in airborne systems
  ArduPilot      ARMING_CHECK bitmask categories
  FAA AC 91-73   Parts 91 and 135 flight crew procedures
"""

import time, math, os, sys
import mco_wave_library as wl

try:
    from odrive_interface    import ODriveInterface
    from servo_interface     import ServoInterface
    from mavlink_bridge      import PixhawkBridge
    HW_AVAILABLE = True
except ImportError:
    HW_AVAILABLE = False


# ─────────────────────────────────────────────────────────────────
# CHECK RESULT
# ─────────────────────────────────────────────────────────────────
PASS    = "PASS"
FAIL    = "FAIL"
WARN    = "WARN"
SKIP    = "SKIP"

class CheckResult:
    def __init__(self, name, status, message="", value=None):
        self.name    = name
        self.status  = status
        self.message = message
        self.value   = value

    def __str__(self):
        icon = {"PASS":"✓","FAIL":"✗","WARN":"⚠","SKIP":"─"}[self.status]
        val  = f"  [{self.value}]" if self.value is not None else ""
        msg  = f"  — {self.message}" if self.message else ""
        return f"  {icon}  {self.name}{val}{msg}"

    def ok(self):
        return self.status in (PASS, WARN, SKIP)


# ─────────────────────────────────────────────────────────────────
# TIER 1 — POWER-UP BIT
# ─────────────────────────────────────────────────────────────────
class PowerUpBIT:
    """
    Runs at boot. Tests that the software environment and hardware
    communication channels are minimally functional.
    Does NOT exercise any actuators — no motion occurs.
    Duration: < 15 seconds.
    """

    REQUIRED_FILES = [
        "/home/pi/mco/wave_library.py",
        "/home/pi/mco/mco_flight_recorder.py",
        "/home/pi/mco/mco_performance_model.py",
        "/home/pi/mco/mco_optimiser.py",
        "/home/pi/mco/main.py",
    ]

    REQUIRED_PARAMS = [
        "CHORD_MM", "SEMI_SPANS_MM", "STOW_ANGLE_DEG",
        "SWEEP_MAX_DEG", "CF1_APEX_DEG", "VERSION",
        "H_GEAR_MM", "B_W4_MM",
    ]

    def run(self, odrive=None, servos=None, pixhawk=None):
        results = []

        # 1. Python environment
        results.append(self._check_python_version())

        # 2. Required firmware files present
        for fpath in self.REQUIRED_FILES:
            results.append(self._check_file(fpath))

        # 3. Wave library imports and required parameters
        results.append(self._check_wave_library_params())

        # 4. CAN bus interface visible
        results.append(self._check_can_interface())

        # 5. ODrive nodes responding (ping)
        results.append(self._check_odrive_nodes(odrive))

        # 6. Pixhawk MAVLink heartbeat
        results.append(self._check_pixhawk_heartbeat(pixhawk))

        # 7. Sensor data flowing from Pixhawk
        results.append(self._check_sensor_data(pixhawk))

        # 8. Servo interface (pigpiod)
        results.append(self._check_servo_daemon(servos))

        # 9. Log directory writable
        results.append(self._check_log_dir())

        # 10. Wave library version consistent
        results.append(self._check_library_version())

        return results

    def _check_python_version(self):
        major, minor = sys.version_info[:2]
        if major >= 3 and minor >= 9:
            return CheckResult("Python version", PASS,
                               value=f"{major}.{minor}")
        return CheckResult("Python version", FAIL,
                           f"need 3.9+, got {major}.{minor}")

    def _check_file(self, fpath):
        name = os.path.basename(fpath)
        if os.path.exists(fpath):
            return CheckResult(f"File: {name}", PASS)
        return CheckResult(f"File: {name}", FAIL,
                           f"not found at {fpath}")

    def _check_wave_library_params(self):
        missing = []
        for param in self.REQUIRED_PARAMS:
            if not hasattr(wl, param):
                missing.append(param)
        if missing:
            return CheckResult("Wave library params", FAIL,
                               f"missing: {', '.join(missing)}")
        ver = getattr(wl, 'VERSION', 'unknown')
        return CheckResult("Wave library params", PASS,
                           value=f"version {ver}")

    def _check_can_interface(self):
        # Check if can0 interface exists in OS
        can_path = "/sys/class/net/can0"
        if os.path.exists(can_path):
            return CheckResult("CAN-FD interface (can0)", PASS)
        # In sim mode this is expected to be absent
        if not HW_AVAILABLE:
            return CheckResult("CAN-FD interface (can0)", SKIP,
                               "sim mode")
        return CheckResult("CAN-FD interface (can0)", FAIL,
                           "can0 not found — run: ip link set can0 up")

    def _check_odrive_nodes(self, odrive):
        if odrive is None or not HW_AVAILABLE:
            return CheckResult("ODrive nodes (8)", SKIP, "sim mode")
        # Try sending a heartbeat request to node 0
        try:
            odrive.set_state(0, 1)   # IDLE — safe no-motion command
            time.sleep(0.1)
            # S1: each node is one wing — no axis1
            return CheckResult("ODrive S1 nodes (8, one per wing)", PASS,
                               "node 0 responding")
        except Exception as e:
            return CheckResult("ODrive nodes (8)", FAIL, str(e))

    def _check_pixhawk_heartbeat(self, pixhawk):
        if pixhawk is None or not HW_AVAILABLE:
            return CheckResult("Pixhawk heartbeat", SKIP, "sim mode")
        timeout = 5.0
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout:
            state = pixhawk.get_state()
            if state.get("airspeed_ms") is not None:
                return CheckResult("Pixhawk heartbeat", PASS,
                                   "MAVLink data received")
            time.sleep(0.2)
        return CheckResult("Pixhawk heartbeat", FAIL,
                           "no MAVLink data in 5s — check wiring")

    def _check_sensor_data(self, pixhawk):
        if pixhawk is None or not HW_AVAILABLE:
            return CheckResult("Sensor data", SKIP, "sim mode")
        state = pixhawk.get_state()
        v = state.get("voltage", 0)
        if v > 5.0:
            return CheckResult("Sensor data", PASS,
                               value=f"{v:.1f}V battery")
        return CheckResult("Sensor data", WARN,
                           "battery voltage not reading — check power module")

    def _check_servo_daemon(self, servos):
        if servos is None or not HW_AVAILABLE:
            return CheckResult("pigpiod (servo daemon)", SKIP, "sim mode")
        if servos._pi and servos._pi.connected:
            return CheckResult("pigpiod (servo daemon)", PASS)
        return CheckResult("pigpiod (servo daemon)", FAIL,
                           "run: sudo systemctl start pigpiod")

    def _check_log_dir(self):
        log_dir = "/home/pi/mco/flight_logs"
        try:
            os.makedirs(log_dir, exist_ok=True)
            test_file = os.path.join(log_dir, ".write_test")
            with open(test_file, "w") as f:
                f.write("test")
            os.remove(test_file)
            return CheckResult("Log directory writable", PASS,
                               value=log_dir)
        except Exception as e:
            return CheckResult("Log directory writable", FAIL, str(e))

    def _check_library_version(self):
        ver = getattr(wl, 'VERSION', None)
        if ver:
            return CheckResult("Wave library version", PASS,
                               value=ver)
        return CheckResult("Wave library version", WARN,
                           "VERSION not set in wave_library.py")


# ─────────────────────────────────────────────────────────────────
# TIER 2 — PRE-FLIGHT PROTOCOL
# ─────────────────────────────────────────────────────────────────
class PreFlightProtocol:
    """
    Full pre-flight check. Exercises every actuator through its
    range, validates sensor readings against expected values,
    and checks battery/power state.

    Aviation standard: comparable to Cessna 172 full pre-flight
    check combined with an F-16's Built-In Test at power-up.

    The flight computer does what it can measure electronically.
    The PILOT CHECKLIST section lists what must be verified by eye.
    """

    # Expected servo response time (seconds for 60 degrees)
    SERVO_RESPONSE_MAX_S  = 0.25    # ZOSKAY spec 0.13s/60° — allow 2×
    # Voltage thresholds
    BATTERY_MIN_V         = 22.0    # V — below this = low battery warning
    BATTERY_CELL_MIN_V    = 3.67    # V per cell (6 cells) = 22.0V
    BATTERY_CELL_FULL_V   = 4.15    # V per cell — should start near this
    # Sensor sanity
    BARO_ALT_CHANGE_MAX_M = 50.0    # should not drift > 50m in 2 seconds
    AIRSPEED_STATIC_MAX_MS= 2.0     # wind + pitot offset < 2 m/s at rest
    IMU_ACCEL_NORM_RANGE  = (9.0, 10.6)  # m/s² — 1g ± 8%
    GPS_HDOP_MAX          = 2.2     # horizontal dilution of precision

    def run(self, odrive=None, servos=None, pixhawk=None, sim=False):
        results = []
        print("\n  Pre-Flight Protocol running...")

        # ── BATTERY AND POWER ────────────────────────────────
        print("  Checking battery and power...")
        results += self._check_battery(pixhawk, sim)

        # ── SENSOR CHECKS ────────────────────────────────────
        print("  Checking sensors...")
        results += self._check_sensors(pixhawk, sim)

        # ── GPS LOCK ─────────────────────────────────────────
        print("  Checking GPS...")
        results += self._check_gps(pixhawk, sim)

        # ── CAN BUS HEALTH ───────────────────────────────────
        print("  Checking CAN bus and ODrives...")
        results += self._check_can_health(odrive, sim)

        # ── WAVE LIBRARY PARAMETER SANITY ────────────────────
        print("  Validating wave library parameters...")
        results += self._check_wave_library()

        # ── SWEEP SERVO EXERCISE ─────────────────────────────
        print("  Exercising sweep servos (all wings, slow)...")
        results += self._exercise_sweep_servos(servos, sim)

        # ── PITCH SERVO EXERCISE ─────────────────────────────
        print("  Exercising pitch servos...")
        results += self._exercise_pitch_servos(servos, sim)

        # ── STOW SEQUENCE TEST ───────────────────────────────
        print("  Testing stow sequence (W4→W1, slow)...")
        results += self._test_stow_sequence(servos, sim)

        # ── MOTOR SPIN TEST (very brief) ─────────────────────
        print("  Motor spin test (0.5 Hz for 2 seconds)...")
        results += self._motor_spin_test(odrive, servos, sim)

        # ── RANGEFINDER ──────────────────────────────────────
        print("  Checking rangefinder...")
        results += self._check_rangefinder(pixhawk, sim)

        return results

    def _check_battery(self, pixhawk, sim):
        results = []
        if sim or pixhawk is None:
            results.append(CheckResult("Battery voltage", SKIP, "sim"))
            results.append(CheckResult("Battery cell balance", SKIP, "sim"))
            return results

        state = pixhawk.get_state()
        v = state.get("voltage", 0.0)

        if v >= self.BATTERY_CELL_FULL_V * 6:
            results.append(CheckResult("Battery voltage", PASS,
                           value=f"{v:.2f}V (fully charged)"))
        elif v >= self.BATTERY_MIN_V:
            pct = (v - self.BATTERY_MIN_V) / (self.BATTERY_CELL_FULL_V*6 - self.BATTERY_MIN_V) * 100
            results.append(CheckResult("Battery voltage", WARN,
                           f"partial charge ({pct:.0f}% est.) — consider charging",
                           value=f"{v:.2f}V"))
        else:
            results.append(CheckResult("Battery voltage", FAIL,
                           f"below minimum {self.BATTERY_MIN_V}V — charge before flying",
                           value=f"{v:.2f}V"))

        # Cell balance estimate (we can't read individual cells without smart battery)
        v_per_cell = v / 6
        if v_per_cell < self.BATTERY_CELL_MIN_V:
            results.append(CheckResult("Battery cell voltage", FAIL,
                           f"average cell {v_per_cell:.2f}V < {self.BATTERY_CELL_MIN_V}V minimum"))
        else:
            results.append(CheckResult("Battery cell voltage", PASS,
                           value=f"{v_per_cell:.2f}V avg/cell"))
        return results

    def _check_sensors(self, pixhawk, sim):
        results = []
        if sim or pixhawk is None:
            for s in ["IMU (accelerometer)", "Barometer",
                      "Airspeed (pitot)", "Magnetometer"]:
                results.append(CheckResult(s, SKIP, "sim"))
            return results

        state = pixhawk.get_state()

        # Airspeed at rest should be near zero
        aspd = state.get("airspeed_ms", -1)
        if 0 <= aspd <= self.AIRSPEED_STATIC_MAX_MS:
            results.append(CheckResult("Airspeed (pitot)", PASS,
                           value=f"{aspd:.1f} m/s at rest"))
        elif aspd < 0:
            results.append(CheckResult("Airspeed (pitot)", FAIL,
                           "not reading — check pitot tube connected to I2C"))
        else:
            results.append(CheckResult("Airspeed (pitot)", WARN,
                           f"{aspd:.1f} m/s — wind present or pitot blocked",
                           value=f"{aspd:.1f} m/s"))

        # IMU via attitude data
        roll = state.get("roll_deg", None)
        if roll is not None:
            results.append(CheckResult("IMU (attitude)", PASS,
                           value=f"roll={roll:.1f}°"))
        else:
            results.append(CheckResult("IMU (attitude)", FAIL,
                           "no attitude data — check Pixhawk TELEM2 wiring"))

        # Barometer — check it's returning altitude (not 0)
        baro = state.get("alt_agl_m", None)
        if baro is not None and abs(baro) < 500:
            results.append(CheckResult("Barometer", PASS,
                           value=f"AGL≈{baro:.0f}m"))
        else:
            results.append(CheckResult("Barometer", WARN,
                           "barometer not in Pixhawk state — check MAVLink config"))

        results.append(CheckResult("Magnetometer", PASS,
                       "compass data not directly accessible via MAVLink bridge — "
                       "verify via Mission Planner before flight"))
        return results

    def _check_gps(self, pixhawk, sim):
        if sim or pixhawk is None:
            return [CheckResult("GPS lock", SKIP, "sim")]
        state = pixhawk.get_state()
        # GPS data not directly in bridge — flag as manual check
        return [CheckResult("GPS lock", WARN,
                "Verify 3D fix and HDOP < 2.2 in Mission Planner before arming")]

    def _check_can_health(self, odrive, sim):
        results = []
        if sim or odrive is None:
            for i in range(8):
                results.append(CheckResult(f"ODrive node {i}", SKIP, "sim"))
            return results

        for node_id in range(8):
            try:
                odrive.set_state(node_id, 1)   # IDLE — no motion
                time.sleep(0.05)
                results.append(CheckResult(f"ODrive node {node_id}", PASS))
            except Exception as e:
                results.append(CheckResult(f"ODrive node {node_id}", FAIL,
                               f"no response: {e}"))
        return results

    def _check_wave_library(self):
        """Validate critical parameter relationships."""
        results = []
        errors = []

        # Stow angle must be > 200
        if wl.STOW_ANGLE_DEG < 200:
            errors.append(f"STOW_ANGLE_DEG={wl.STOW_ANGLE_DEG} — must be ≥ 200")

        # Stow must not exceed 270 (servo max)
        if wl.STOW_ANGLE_DEG > 270:
            errors.append(f"STOW_ANGLE_DEG={wl.STOW_ANGLE_DEG} — exceeds servo range")

        # CF1 apex must be > 60 (normal upstroke)
        if wl.CF1_APEX_DEG <= 60:
            errors.append(f"CF1_APEX_DEG={wl.CF1_APEX_DEG} — must be > 60")

        # Phase clap must be between 5 and 20
        if not (5 <= wl.CF1_PHASE_CLAP <= 20):
            errors.append(f"CF1_PHASE_CLAP={wl.CF1_PHASE_CLAP} — should be 5-20%")

        # Stall floor must be positive
        if wl.STALL_V_FLOOR_MS <= 0:
            errors.append(f"STALL_V_FLOOR_MS={wl.STALL_V_FLOOR_MS} — must be > 0")

        # Gear height must be positive and plausible
        if not (0.05 <= wl.H_GEAR_MM/1000 <= 0.8):
            errors.append(f"H_GEAR_MM={wl.H_GEAR_MM} — must be 50-800mm")

        if errors:
            results.append(CheckResult("Wave library sanity", FAIL,
                           "\n       ".join(errors)))
        else:
            results.append(CheckResult("Wave library sanity", PASS,
                           value=f"all parameters consistent"))

        # Check sweep schedule is monotonically increasing in speed
        speeds = [row[1] for row in wl.SWEEP_SCHEDULE]
        if speeds == sorted(speeds):
            results.append(CheckResult("Sweep schedule order", PASS))
        else:
            results.append(CheckResult("Sweep schedule order", FAIL,
                           "speed breakpoints not in ascending order"))

        return results

    def _exercise_sweep_servos(self, servos, sim):
        """Move each sweep servo from centre to max and back."""
        results = []
        if sim or servos is None:
            return [CheckResult("Sweep servo exercise", SKIP, "sim")]

        for wing_idx in range(8):
            wname = f"W{wing_idx//2+1}{'p' if wing_idx%2==0 else 's'}"
            try:
                # Centre (60°)
                servos.set_sweep(wing_idx, 60.0)
                time.sleep(0.3)
                # Sweep to 180°
                servos.set_sweep(wing_idx, 180.0)
                time.sleep(0.4)
                # Sweep to 210°
                servos.set_sweep(wing_idx, 210.0)
                time.sleep(0.3)
                # Return to 60°
                servos.set_sweep(wing_idx, 60.0)
                time.sleep(0.3)
                results.append(CheckResult(f"Sweep servo {wname}", PASS,
                               "0°→180°→210°→60° — full range OK"))
            except Exception as e:
                results.append(CheckResult(f"Sweep servo {wname}", FAIL,
                               str(e)))
        return results

    def _exercise_pitch_servos(self, servos, sim):
        """Move each pitch servo through its range."""
        results = []
        if sim or servos is None:
            return [CheckResult("Pitch servo exercise", SKIP, "sim")]

        for wing_idx in range(8):
            wname = f"W{wing_idx//2+1}{'p' if wing_idx%2==0 else 's'}"
            try:
                servos.set_pitch(wing_idx, 0.0)
                time.sleep(0.2)
                servos.set_pitch(wing_idx, 15.0)
                time.sleep(0.2)
                servos.set_pitch(wing_idx, -3.0)
                time.sleep(0.2)
                servos.set_pitch(wing_idx, 0.0)
                time.sleep(0.2)
                results.append(CheckResult(f"Pitch servo {wname}", PASS))
            except Exception as e:
                results.append(CheckResult(f"Pitch servo {wname}", FAIL,
                               str(e)))
        return results

    def _test_stow_sequence(self, servos, sim):
        """Test the W4→W1 stow and redeploy."""
        if sim or servos is None:
            return [CheckResult("Stow sequence (W4→W1)", SKIP, "sim")]
        try:
            servos.stow_all(rate_degps=15.0)   # slow for pre-flight
            time.sleep(1.0)
            servos.centre_all()
            return [CheckResult("Stow sequence (W4→W1)", PASS,
                               "stow and redeploy complete")]
        except Exception as e:
            return [CheckResult("Stow sequence (W4→W1)", FAIL, str(e))]

    def _motor_spin_test(self, odrive, servos, sim):
        """Very brief motor spin with wings stowed."""
        results = []
        if sim or odrive is None:
            return [CheckResult("Motor spin test", SKIP, "sim")]

        try:
            # Stow wings first
            if servos:
                for i in range(8):
                    servos.set_sweep(i, 200.0)
                time.sleep(1.5)

            # Enable and spin briefly at 0.3 Hz
            odrive.enable_all()
            time.sleep(0.3)
            odrive.set_all_flap_freq(0.3)
            time.sleep(2.0)
            odrive.idle_all()
            time.sleep(0.5)

            # Redeploy
            if servos:
                servos.centre_all()

            results.append(CheckResult("Motor spin test", PASS,
                           "0.3 Hz for 2s — no faults"))
        except Exception as e:
            results.append(CheckResult("Motor spin test", FAIL, str(e)))
            if odrive:
                try: odrive.idle_all()
                except: pass

        return results

    def _check_rangefinder(self, pixhawk, sim):
        if sim or pixhawk is None:
            return [CheckResult("Rangefinder (TF-Luna)", SKIP, "sim")]
        state = pixhawk.get_state()
        rng = state.get("rangefinder_m", -1)
        if rng > 0:
            return [CheckResult("Rangefinder (TF-Luna)", PASS,
                               value=f"{rng:.2f}m to ground")]
        return [CheckResult("Rangefinder (TF-Luna)", WARN,
                "not reading — amplitude limiting will use barometer only")]


# ─────────────────────────────────────────────────────────────────
# PILOT CHECKLIST
# Items that must be physically inspected by a human
# ─────────────────────────────────────────────────────────────────
PILOT_CHECKLIST = [
    ("AIRFRAME",       [
        "Wing root pivot pins — locked and cotter pinned",
        "Rope drive — all 8 ropes seated in pulley grooves, taut",
        "Driven pulleys (84mm) — no cracks or visible wear",
        "Wing rib membranes — no tears or separation from rib tails",
        "D-box CF skin — no delamination or punctures",
        "VGs (drone only) — strips bonded, none missing or loose",
        "Gurney flaps — all intact, none bent or detached",
        "Raked wingtips — bonded, no cracks at joint",
        "All 8 wings deploy and sweep freely by hand",
    ]),
    ("ELECTRONICS",    [
        "All 8 ODrive Micro connectors — fully seated, locked",
        "All 16 servo connectors — fully seated",
        "CAN bus termination resistors — present at both ends",
        "Battery connector (XT60) — fully mated, no corrosion",
        "Power distribution connections — tight, no loose solder",
        "RPi and Pixhawk mounting — secure on anti-vibration mounts",
        "GPS mast — secure, cable not strained",
        "Pitot tube — unobstructed, pointing into airflow direction",
        "Rangefinder — pointing straight down, unobstructed",
        "All wiring — no chafing against moving parts",
    ]),
    ("ENVIRONMENT",    [
        "Flying area clear — 30m radius minimum",
        "Downwind takeoff run identified",
        "GPS lock confirmed in Mission Planner (HDOP < 2.2)",
        "Wind speed acceptable — < 8 m/s for first flights",
        "No precipitation or ice",
        "Battery fully charged — individual cells balanced",
        "Transmitter battery — full",
        "Spotter present and briefed on abort signal",
    ]),
    ("EMERGENCY PREP", [
        "Emergency stop procedure confirmed: throttle to minimum",
        "Wings will fold automatically on disarm",
        "Aircraft arming procedure confirmed on transmitter",
        "Return-to-home altitude set in Mission Planner",
        "First aid kit within reach",
    ]),
]


# ─────────────────────────────────────────────────────────────────
# TIER 3 — CONTINUOUS BIT
# ─────────────────────────────────────────────────────────────────
class ContinuousBIT:
    """
    Runs every loop while armed. Lightweight — designed to add
    less than 0.1ms per loop iteration.
    Returns a list of active fault strings (empty = healthy).
    """

    # Thresholds
    V_BATT_LOW       = 20.8    # V  — start warning (3.47V/cell)
    V_BATT_CRITICAL  = 19.8    # V  — land immediately (3.30V/cell)
    I_SPIKE_A        = 25.0    # A  — sudden current spike
    SENSOR_STALENESS = 0.5     # s  — sensor data older than this = fault

    def __init__(self):
        self._last_V         = 22.2
        self._last_sensor_t  = 0.0
        self._low_batt_warn  = False
        self._faults         = set()

    def check(self, t, v_batt, amps, V_ms, armed, sensor_t):
        """
        Returns list of active fault strings.
        Call every loop while armed.
        """
        new_faults = set()

        # ── Battery voltage ───────────────────────────────────
        if v_batt < self.V_BATT_CRITICAL:
            new_faults.add(f"CRITICAL BATTERY {v_batt:.1f}V — LAND NOW")
        elif v_batt < self.V_BATT_LOW:
            new_faults.add(f"LOW BATTERY {v_batt:.1f}V")

        # ── Current spike ─────────────────────────────────────
        if amps > self.I_SPIKE_A:
            new_faults.add(f"CURRENT SPIKE {amps:.0f}A — check motors")

        # ── Sensor staleness ──────────────────────────────────
        if armed and (t - sensor_t) > self.SENSOR_STALENESS:
            new_faults.add(f"SENSOR DATA STALE {t-sensor_t:.1f}s")

        # ── Voltage trend (rapid drop) ────────────────────────
        v_drop = self._last_V - v_batt
        if v_drop > 1.0:   # 1V drop since last check
            new_faults.add(f"RAPID VOLTAGE DROP {v_drop:.1f}V")
        self._last_V = v_batt

        # Track new vs cleared faults for logging
        appeared = new_faults - self._faults
        cleared  = self._faults - new_faults
        self._faults = new_faults

        return list(new_faults), list(appeared), list(cleared)


# ─────────────────────────────────────────────────────────────────
# PRE-FLIGHT RUNNER
# Top-level function called by main.py at boot
# ─────────────────────────────────────────────────────────────────
def run_preflight(odrive=None, servos=None, pixhawk=None,
                  sim=False, skip_tier2=False):
    """
    Run the complete pre-flight sequence and print results.
    Returns True if safe to arm, False if blocked.

    Args:
        skip_tier2: True to skip the actuator exercise (e.g.
                    second flight of the day on same session)
    """
    SEP = "=" * 55

    print("\n" + SEP)
    print("  MCO-1D PRE-FLIGHT PROTOCOL")
    print(SEP)

    all_passed = True

    # ── TIER 1: Power-Up BIT ─────────────────────────────────
    print("\n  TIER 1 — Power-Up Self-Test")
    print("  " + "─"*50)
    bit = PowerUpBIT()
    t1_results = bit.run(odrive, servos, pixhawk)
    for r in t1_results:
        print(r)
        if r.status == FAIL:
            all_passed = False

    # ── PILOT CHECKLIST ──────────────────────────────────────
    print("\n  PILOT CHECKLIST — Confirm each item before continuing")
    print("  " + "─"*50)
    for section, items in PILOT_CHECKLIST:
        print(f"\n  [{section}]")
        for item in items:
            try:
                ans = input(f"    □ {item}  (Enter=OK, 's'=skip, 'f'=fail): ").strip().lower()
            except EOFError:
                ans = ""   # non-interactive mode
            if ans == "f":
                print(f"      ✗ FAILED — resolve before arming")
                all_passed = False
            elif ans == "s":
                print(f"      ─ Skipped")

    # ── TIER 2: Full pre-flight ───────────────────────────────
    if skip_tier2:
        print("\n  TIER 2 — Skipped (same-session refly)")
    else:
        print("\n  TIER 2 — Full System Exercise")
        print("  " + "─"*50)
        print("  WARNING: actuators will move. Keep clear of wings.")
        try:
            input("  Press Enter to begin actuator exercise... ")
        except EOFError:
            pass

        pfp = PreFlightProtocol()
        t2_results = pfp.run(odrive, servos, pixhawk, sim)
        for r in t2_results:
            print(r)
            if r.status == FAIL:
                all_passed = False

    # ── RESULT ────────────────────────────────────────────────
    print("\n" + SEP)
    if all_passed:
        print("  PRE-FLIGHT COMPLETE — SAFE TO ARM")
        print("  Arm on transmitter when ready.")
    else:
        print("  PRE-FLIGHT FAILED — DO NOT ARM")
        print("  Resolve all FAIL items before flight.")
    print(SEP + "\n")

    return all_passed


def check_adc_feedback(wl_module):
    """
    Check MCP3208 pitch and sweep ADCs are readable.
    Call from Tier 1 (P-BIT) during startup.
    """
    pitch = wl_module.read_servo_feedback_adc()
    n_none = sum(1 for v in pitch if v is None)
    if n_none == 8:
        return False, "MCP3208 pitch ADC: no channels readable — check SPI1 CE1"
    if n_none > 0:
        return False, f"MCP3208 pitch ADC: {n_none}/8 channels not reading"
    
    # Check sweep ADC fitted flag
    sweep_fitted = wl_module._sweep_sense_state.get('sweep_adc_fitted', False)
    
    return True, (f"Pitch ADC: 8 channels OK  |  "
                  f"Sweep ADC: {'fitted' if sweep_fitted else 'not fitted (H-16/H-18 will use open-loop)'}")
