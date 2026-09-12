#!/usr/bin/env python3
"""
MCO-1D Main Flight Loop
========================
The top-level entry point that runs on the Raspberry Pi.
Wires together all subsystems into a 50 Hz control loop.

Architecture:
    MCOMAVLink (background thread) → sensors dict (50 Hz)
         ↓
    MCOFlightController.tick() → result dict (50 Hz)
         ↓
    ODrive commands (CAN-FD) + Servo commands (GPIO/pigpio)
         ↓
    MCOLogger (background thread) ← result dict
    MCOGSServer (background thread) ← telemetry dict

Usage:
    python3 mco_main.py                     # normal flight
    python3 mco_main.py --sim               # simulation (no hardware)
    python3 mco_main.py --port /dev/ttyS0  # different MAVLink port

Dependencies (install on Pi):
    pip install pymavlink odrive
    pip install RPi.GPIO pigpio
    sudo apt install python3-pygame          # for panel
    sudo systemctl enable pigpiod && sudo systemctl start pigpiod
"""

import sys
import time
import gc
import math
import threading
import argparse
import signal

# ── Load wave library first (takes a moment — 45K lines) ─────────
print("Loading MCO-1D Wave Library...", end='', flush=True)
import mco_wave_library as w
from mco_preflight_checks import s1_monitor, check_s1_post_flight
from mco_preflight_checks import run_s1_preflight_checks
# Restore all cross-flight learning before first tick
_pl_result = w.persist_load_all(verbose=True)
print(f" OK ({len(open(__file__.replace('mco_main.py', 'mco_wave_library.py')).read().splitlines()):,} lines)")

# ── Optional hardware imports ─────────────────────────────────────
try:
    import pigpio
    HAS_PIGPIO = True
except ImportError:
    HAS_PIGPIO = False
    print("WARNING: pigpio not available — servos will not move")

try:
    import can
    HAS_CAN = True
except ImportError:
    HAS_CAN = False
    print("WARNING: python-can not available — ODrives will not respond")

from mco_mavlink   import MCOMAVLink
from mco_logger    import MCOLogger
from mco_gs_server import MCOGSServer


# ── Servo PWM constants ───────────────────────────────────────────
# Pin and motor constants — pulled from wave library to stay in sync
MOTOR_COUNT = w.MOTOR_COUNT       # 8 independent motors
REDUCER_RATIO = w.REDUCER_RATIO   # 7:1
PITCH_PINS  = list(w.PITCH_PINS)  # 8 pitch servo GPIO pins
SWEEP_PINS  = list(w.SWEEP_PINS)  # 8 sweep servo GPIO pins
MCP3208_SPI_BUS  = w.MCP3208_SPI_BUS
MCP3208_SPI_CE   = w.MCP3208_SPI_CE
MCP3208_SWEEP_CE = w.MCP3208_SWEEP_SPI_CE
SWEEP_PINS = [ 4, 17, 27, 22,  5,  6, 13, 19]

SERVO_MIN_US  = 900    # microseconds — full negative
SERVO_MID_US  = 1500   # microseconds — centre
SERVO_MAX_US  = 2100   # microseconds — full positive
SERVO_FREQ    = 50     # Hz for DS3225 sweep servos
PITCH_FREQ    = 333    # Hz for TORQ TCM-1001 pitch servos (7.4V capable)


def deg_to_servo_us(deg, deg_range=90.0):
    """Convert angle in degrees to servo pulse width in microseconds."""
    fraction = deg / deg_range    # -1.0 to +1.0
    fraction = max(-1.0, min(1.0, fraction))
    return int(SERVO_MID_US + fraction * (SERVO_MAX_US - SERVO_MID_US))


class MCOHardware:
    """Manages servo and ODrive output."""

    def __init__(self, sim_mode=False):
        self.sim_mode = sim_mode
        self.pi       = None
        self.can_bus  = None

        if not sim_mode:
            self._init_servos()
            self._init_can()

    def _init_servos(self):
        if not HAS_PIGPIO:
            return
        self.pi = pigpio.pi()
        if not self.pi.connected:
            print("ERROR: pigpio daemon not running. Run: sudo systemctl start pigpiod")
            self.pi = None
            return
        for pin in PITCH_PINS + SWEEP_PINS:
            self.pi.set_mode(pin, pigpio.OUTPUT)
            self.pi.set_servo_pulsewidth(pin, SERVO_MID_US)
        print(f"Servos initialised: {len(PITCH_PINS)} pitch, {len(SWEEP_PINS)} sweep")

    def _init_can(self):
        if not HAS_CAN:
            return
        try:
            self.can_bus = can.interface.Bus('can0', bustype='socketcan')
            print("CAN-FD bus: connected")
        except Exception as e:
            print(f"CAN-FD bus: failed ({e})")
            self.can_bus = None

    def _read_odrive_positions(self):
        """
        Read motor shaft position from all 8 ODrive axes.
        Returns list of 8 floats (radians) or [None]*8 in sim mode.
        Axis assignment: wing_idx = board_idx*2 + axis_idx
        Node 0=W1p, Node 1=W1s, Node 2=W2p, Node 3=W2s, Node 4=W3p, Node 5=W3s, Node 6=W4p, Node 7=W4s
        """
        if self.sim_mode or not self.can_bus:
            return [None] * 8
        positions = []
        for board in range(4):
            for axis in range(2):
                # ODrive GET_ENCODER_ESTIMATES (0x009) returns pos, vel
                arb_id = (wing_idx << 5) | 0x009  # S1: node_id = wing_idx
                try:
                    self.can_bus.send(can.Message(
                        arbitration_id=arb_id, data=b'', is_extended_id=False))
                    msg = self.can_bus.recv(timeout=0.002)
                    if msg and len(msg.data) >= 8:
                        import struct
                        pos_est, _ = struct.unpack('<ff', msg.data[:8])
                        positions.append(pos_est)
                    else:
                        positions.append(None)
                except Exception:
                    positions.append(None)
        return positions

    def _read_odrive_currents(self):
        """
        Read Iq_measured from all 8 ODrive axes.
        Returns list of 8 floats (amps) or [10.0]*8 in sim mode.
        """
        if self.sim_mode or not self.can_bus:
            return [10.0] * 8
        currents = []
        for board in range(4):
            for axis in range(2):
                arb_id = (wing_idx << 5) | 0x014  # S1 GET_IQ: node = wing_idx
                try:
                    self.can_bus.send(can.Message(
                        arbitration_id=arb_id, data=b'', is_extended_id=False))
                    msg = self.can_bus.recv(timeout=0.002)
                    if msg and len(msg.data) >= 8:
                        import struct
                        _, iq = struct.unpack('<ff', msg.data[:8])
                        currents.append(abs(iq))
                    else:
                        currents.append(10.0)
                except Exception:
                    currents.append(10.0)
        return currents

    def apply_commands(self, result: dict):
        """Apply wing commands to servos and ODrives."""
        if self.sim_mode:
            return

        wc      = result.get('wing_commands', [])
        freq_hz = result.get('freq_hz', 0.0)

        for i, cmd in enumerate(wc[:8]):
            if i < len(PITCH_PINS) and self.pi:
                pitch_deg = cmd.get('pitch_deg', 0.0)
                self.pi.set_servo_pulsewidth(
                    PITCH_PINS[i], deg_to_servo_us(pitch_deg))

            if i < len(SWEEP_PINS) and self.pi:
                sweep_deg = cmd.get('sweep_deg', 90.0) - 90.0  # centre at 90°
                self.pi.set_servo_pulsewidth(
                    SWEEP_PINS[i], deg_to_servo_us(sweep_deg))

        # ODrive velocity commands via CAN
        if self.can_bus and freq_hz > 0:
            self._send_odrive_velocity(
                    freq_hz, wc,
                    motor_positions_rad=sensors.get('motor_pos_rad'))

    def _send_odrive_velocity(self, freq_hz, wing_commands,
                               motor_positions_rad=None):
        """
        Send velocity setpoints to ODrives via CAN.

        Applies stroke soft-stop cosine taper in the final
        SOFT_STOP_ZONE_DEG before each stroke reversal.
        Per-wing vel_limit is reduced inside the zone to cap
        rope tension impulse at reversal, then restored immediately.

        motor_positions_rad: list[8] from sensors['motor_pos_rad'].
            If None (sim mode or CAN read failure), soft-stop is
            bypassed for this tick (safe — taper defaults to 1.0).
        """
        import struct
        from mco_wave_library import (soft_stop_apply_to_commands,
                                       REDUCER_RATIO, FLAP_AMP_DEG)

        reducer = REDUCER_RATIO   # 7.0 — verified at TC-01 step C1

        # ── Apply soft-stop taper to all 8 wings ─────────────────
        # Returns per-wing vel_limit overrides (None if no override)
        vel_limit_overrides = soft_stop_apply_to_commands(
            wing_commands,
            motor_positions_rad or [None] * 8,
            base_vel_limit_rps=150.0
        )

        # ── Send velocity + optional vel_limit per wing ───────────
        for i, cmd in enumerate(wing_commands[:8]):
            amp    = cmd.get('flap_amp_deg', FLAP_AMP_DEG)
            factor = cmd.get('soft_stop_factor', 1.0)

            # Full velocity = freq_hz × 2 × amp_rad × reducer
            # Tapered vel  = full_vel × factor
            vel = freq_hz * (2.0 * math.radians(amp)) * reducer * factor

            # S1: node_id == wing_idx (1:1 mapping)
            arb_id = (i << 5) | 0x0D   # SET_INPUT_VEL
            data   = struct.pack('<ff', vel, 0.0)
            try:
                msg = can.Message(arbitration_id=arb_id,
                                   data=data, is_extended_id=False)
                self.can_bus.send(msg)
            except Exception:
                pass

            # ── Per-wing vel_limit override inside soft-stop zone ─
            vl = vel_limit_overrides[i]
            if vl is not None and self.odi:
                try:
                    self.odi.set_velocity_limit(i, vl)
                except Exception:
                    pass

        # ── Restore vel_limits for wings that just left the zone ──
        for i, cmd in enumerate(wing_commands[:8]):
            if (not cmd.get('soft_stop_zone', False)
                    and vel_limit_overrides[i] is None):
                # Wing is outside zone — ensure limit is at base value.
                # Only send if we have a record this wing was recently
                # in-zone (avoid unnecessary CAN traffic).
                from mco_wave_library import _ss_state
                if _ss_state['in_zone'][i] is False and self.odi:
                    # Already out of zone, no restore needed this tick.
                    pass

    def shutdown(self):
        """Safe shutdown — centre all servos, stop ODrives."""
        if self.pi:
            for pin in PITCH_PINS + SWEEP_PINS:
                self.pi.set_servo_pulsewidth(pin, SERVO_MID_US)
            time.sleep(0.2)
            for pin in PITCH_PINS + SWEEP_PINS:
                self.pi.set_servo_pulsewidth(pin, 0)   # disable servos
            self.pi.stop()
        if self.can_bus:
            self.can_bus.shutdown()


class MCOMain:
    """Top-level flight loop."""

    TICK_RATE_HZ = 50
    TICK_S       = 1.0 / TICK_RATE_HZ

    def __init__(self, port='/dev/ttyAMA0', baud=921600, sim=False):
        self.sim_mode = sim
        self.alive    = True

        # Load knowledge file if present
        bc = w._BOOT_KNOWLEDGE_CHECK
        if bc.get('available'):
            print(f"Knowledge: {bc['message']}")

        # Subsystems
        self.mavlink  = MCOMAVLink(port, baud)
        self.logger   = MCOLogger()
        self.gs_server= MCOGSServer()
        self.hardware  = MCOHardware(sim_mode=sim)
        self.fcc       = w.MCOFlightController()

        # Initialise automated test card runners
        try:
            from mco_test_cards import init_test_runners, handle_tc_command
            self._handle_tc = handle_tc_command
            init_test_runners(
                self.fcc,
                lambda: self.mavlink.get_sensors(),
                self.hardware)
        except Exception as e:
            print(f'Test card runners not available: {e}')
            self._handle_tc = None

        # Graceful shutdown
        signal.signal(signal.SIGTERM, self._handle_signal)
        signal.signal(signal.SIGINT,  self._handle_signal)

    def _handle_signal(self, sig, frame):
        print(f"\nShutdown signal received ({sig})")
        self.alive = False

    def start(self):
        """Start all subsystems and enter flight loop."""
        self.mavlink.start()
        self.logger.start()
        self.gs_server.start_background()
        print(f"\nMCO-1D flight loop starting at {self.TICK_RATE_HZ} Hz")
        print(f"Sim mode: {self.sim_mode}")
        print("Ready. Pre-flight check via ground station or panel.\n")

        tick_count  = 0
        last_tick_t = time.monotonic()

        while self.alive:
            tick_start = time.monotonic()

            # ── Get sensor data ───────────────────────────────────
            sensors = self.mavlink.get_sensors()
            sensors['intent'] = 'cruise'   # default; override by GS cmd

            # ── Per-wing sensing (MCP3208 ADCs + ODrive pos) ─
            # Pitch servo feedback — MCP3208 #1 (SPI1 CE1)
            sensors['servo_actual_aoa_deg'] = w.read_servo_feedback_adc()
            # Sweep servo feedback — MCP3208 #2 (SPI1 CE0); None if not fitted
            sensors['sweep_actual_deg'] = (
                w._sweep_sense_state.get('actual_sweep_deg', [None]*8)
                if w._sweep_sense_state.get('sweep_adc_fitted') else [None]*8)
            # ODrive S1 stroke position — 8 values (1 per board, node == wing_idx)
            sensors['motor_pos_rad']    = self._read_odrive_positions()
            # ODrive S1 motor current — 8 values (1 per board)
            sensors['motor_currents_a'] = self._read_odrive_currents()
            # ODrive S1 FET and motor temperatures — NEW S1 capability
            # read_temperatures() returns list of 8 {'fet_C', 'motor_C'} dicts
            # Called at 5 Hz (every 10 ticks) to reduce CAN bus load
            if tick_count % 10 == 0:
                try:
                    sensors['motor_temps'] = self.hardware.odi.read_temperatures()
                except Exception:
                    sensors['motor_temps'] = []
            else:
                sensors['motor_temps'] = getattr(self, '_last_motor_temps', [])
            if sensors.get('motor_temps'):
                self._last_motor_temps = sensors['motor_temps']
            # ODrive S1 axis error states — for per-wing fault isolation
            # Called at 1 Hz (every 50 ticks) to reduce CAN bus load
            if tick_count % 50 == 0:
                try:
                    states = self.hardware.odi.read_all_states()
                    sensors['wing_errors'] = [s.get('error', 0) for s in states]
                except Exception:
                    sensors['wing_errors'] = []
            else:
                sensors['wing_errors'] = getattr(self, '_last_wing_errors', [])
            if sensors.get('wing_errors'):
                self._last_wing_errors = sensors['wing_errors']
            # Actual tick period for thermal calculations
            sensors['dt_s'] = max(0.001, time.monotonic() - tick_start)

            # ── Derived sensor fields ─────────────────────────────────
            # Air density (ISA approximation from OAT + baro altitude)
            oat_c = sensors.get('oat_c', 15.0)
            alt_m = sensors.get('baro_alt_m', 0.0)
            sensors['air_density_kgm3'] = (
                1.225 * ((288.15 - 0.0065 * alt_m) / 288.15) ** 4.256
                * (273.15 / (273.15 + oat_c)))
            # Roll/pitch rates already in sensors from mavlink IMU
            # Ensure lateral_g present (from IMU accel Y)
            if 'lateral_g' not in sensors:
                sensors['lateral_g'] = 0.0
            # Set baseline currents on first good reading
            if (tick_count == 5 and
                    sensors.get('motor_currents_a') and
                    s1_monitor._baseline_I == [None]*8):
                s1_monitor.set_baseline_currents(
                    sensors['motor_currents_a'])

            # ── Process ground station commands ───────────────────
            cmds = self.gs_server.get_pending_commands()
            for cmd_str in cmds:
                try:
                    cmd_dict = {'cmd': cmd_str}
                    # Try test card commands first
                    tc_ack = None
                    if self._handle_tc:
                        tc_ack = self._handle_tc(cmd_str)
                    ack = tc_ack if tc_ack is not None else w._gs_execute_cmd(cmd_dict)
                    self.gs_server.queue_ack(ack)
                except Exception as e:
                    self.gs_server.queue_ack(
                        {'type': 'error', 'msg': str(e)})

            # ── Flight control tick ───────────────────────────────
            try:
                result = self.fcc.tick(sensors)
            except Exception as e:
                print(f"Tick error: {e}")
                result = {'wing_commands': [{}]*8, 'freq_hz': 0.0}

            # ── Apply to hardware ─────────────────────────────────
            self.hardware.apply_commands(result)
            # Store intent for next tick's CG management
            self._last_intent = result.get('intent', 'cruise')

            # ── S1 dynamic velocity limit (thermal protection) ────
            # If the tick wrapper detected high FET temperature,
            # result['dyn_vel_limit_rad_s'] is set to a reduced limit.
            # Apply to all S1 boards over CAN.
            dyn_vlim = result.get('dyn_vel_limit_rad_s')
            if dyn_vlim is not None:
                try:
                    for wing_idx in range(8):
                        self.hardware.odi.set_velocity_limit(wing_idx, dyn_vlim)
                except Exception:
                    pass

            # ── S1 per-wing clear_errors (fault isolation) ────────
            wings_to_clear = result.get('wings_need_clear_errors', [])
            if wings_to_clear:
                try:
                    self.hardware.odi.clear_errors()
                except Exception:
                    pass

            # ── S1 in-flight health monitor ───────────────────────
            # Updates per-wing temperature, current, rope wear, and
            # axis error picture. Produces result['motor_health'].
            result = s1_monitor.update(sensors, result)

            # ── Log and telemetry ─────────────────────────────────
            self.logger.log(sensors, result)

            if tick_count % 5 == 0:   # 10 Hz telemetry
                telem = {
                    'V_ms':            sensors.get('V_ms', 0),
                    'h_agl_m':         sensors.get('h_agl_m', 0),
                    'pitch_deg':       sensors.get('pitch_deg_imu', 0),
                    'roll_deg':        sensors.get('roll_deg_imu', 0),
                    'bus_voltage_v':   sensors.get('bus_voltage_v', 0),
                    'freq_hz':         result.get('freq_hz', 0),
                    'ground_ops_mode': w._ground_ops_state.get('mode', ''),
                    'gnd_state':       result.get('ground_state', ''),
                    'l9_active':       w._L9_state.get('active', False),
                    'disc_active':     w._autonomous_disc_state.get('active', False),
                    'n_disc_promoted': w._autonomous_disc_state.get('n_promoted', 0),
                    'turb_intensity':  sensors.get('imu_rms_g', 0) / 0.1,
                    'preflight_status':w._preflight_state.get('status', 'idle'),
                    'cooldown_elapsed_s': w._ground_ops_state.get('cooldown_elapsed_s', 0),
                    # Per-wing sensing telemetry
                    'actual_aoa_deg':    result.get('actual_aoa_deg', []),
                    'effective_aoa_deg': result.get('effective_aoa_deg', []),
                    'trim_balance':      result.get('trim_balance', {}),
                    'force_waveform':    result.get('force_waveform', {}),
                    'mechanical_power':  result.get('mechanical_power', {}),
                    'strouhal_actual':   result.get('strouhal_actual', []),
                    'stall_detection':   result.get('stall_detection', {}),
                    'servo_health_rms':  result.get('servo_aoa_tracking_rms', []),
                    'trial_valid':       result.get('trial_three_axis_valid', True),
                    # S1 thermal telemetry (real FET/motor temps)
                    'fet_temp_max_C':    result.get('fet_temp_max_C'),
                    'motor_temp_max_C':  result.get('motor_temp_max_C'),
                    'dyn_vel_limit':     result.get('dyn_vel_limit_rad_s'),
                    'thermal_level':     result.get('thermal', {}).get('level', 'normal'),
                    # S1 in-flight health monitor output
                    'motor_health':      result.get('motor_health', {}),
                    'motor_alerts':      result.get('motor_health', {}).get('alerts', []),
                    'motor_alert_level': result.get('motor_health', {}).get('alert_level', 'NORMAL'),
                    # Pilot intent and mechanism weights
                    'roll_mechanisms':  result.get('roll_mechanisms', {}),
                    'pitch_mechanisms': result.get('pitch_mechanisms', {}),
                    'yaw_mechanisms':   result.get('yaw_mechanisms', {}),
                    'lever_fw_pct':     result.get('lever_fw_pct', 50),
                    'pilot_roll':       result.get('pilot_roll', 0),
                    'pilot_pitch':      result.get('pilot_pitch', 0),
                    'pilot_yaw':        result.get('pilot_yaw', 0),
                    'motor_peaks':       result.get('motor_health', {}).get('peaks', {}),
                    # Morphing hypothesis telemetry
                    'h33_active':        result.get('h33_active', False),
                    'h34_active':        result.get('h34_active', False),
                    'h35_active':        result.get('h35_active', False),
                    'h36_active':        result.get('h36_active', False),
                    'h33_variant':       result.get('h33_variant', ''),
                    'h35_lift_N':        result.get('h35_lift_N', 0),
                    'h36_power_W':       result.get('h36_power_W', 0),
                    'h36_endurance_min': result.get('h36_endurance_est_min', 0),
                }
                self.gs_server.set_telemetry(telem)
                self.mavlink.send_status(result)

            tick_count += 1

            # ── Maintain tick rate ────────────────────────────────
            elapsed = time.monotonic() - tick_start
            sleep_s = self.TICK_S - elapsed
            if sleep_s > 0:
                time.sleep(sleep_s)
            # Manual GC between ticks — bounded pause, predictable timing
            _gc_counter += 1
            if _gc_counter >= 100:
                gc.collect()
                _gc_counter = 0
            elif elapsed > self.TICK_S * 1.5:
                pass   # tick overrun — log but don't skip
            # ── Certification tools (structural loads + rope life) ──
            try:
                from mco_certification_tools import (
                    structural_recorder, rope_tracker)
                structural_recorder.record(sensors, result)
                rope_tracker.update(sensors, result)
            except Exception:
                pass

        self._shutdown()

    def _shutdown(self):
        """Clean shutdown sequence."""
        print("\nShutdown: running post-flight export...")
        try:
            from mco_certification_tools import structural_recorder
            structural_recorder.end_flight()
        except Exception:
            pass
        try:
            from mco_certification_tools import rope_tracker
            rope_tracker._save()
        except Exception:
            pass
        # Auto-run HQ analysis on the latest flight log
        try:
            from mco_certification_tools import hq_analyser
            import glob
            logs = sorted(glob.glob('/home/pi/logs/mco_flight_*.csv'))
            if logs:
                latest = logs[-1]
                for phase in ('hover', 'cruise'):
                    hq_analyser.analyse_log(latest, phase)
        except Exception:
            pass
        # Run Bayesian adaptive optimiser on post-flight data
        try:
            from mco_adaptive_optimiser import AdaptiveOptimiser
            _ao = AdaptiveOptimiser()
            _ao.post_flight_update()
        except Exception:
            pass
        # Save all cross-flight learning state
        try:
            w.persist_save_all(notes='end_of_flight')
        except Exception as e:
            print(f"  Persist failed: {e}")
        try:
            w.mco_knowledge_auto_export()
        except Exception as e:
            print(f"  Export failed: {e}")

        print("Shutdown: stopping hardware...")
        self.hardware.shutdown()
        self.logger.stop()
        self.gs_server.stop()
        print("MCO-1D shutdown complete.")


def main():
    # Disable automatic GC to prevent unbounded pause during 50 Hz loop.
    # GC is triggered manually between ticks in the control loop below.
    gc.disable()
    _gc_collect_interval = 100   # collect every 100 ticks (~2 s at 50 Hz)
    _gc_tick_counter     = [0]   # list so it can be mutated in nested scope
    parser = argparse.ArgumentParser(description='MCO-1D Flight Computer')
    parser.add_argument('--port', default='/dev/ttyAMA0',
                        help='MAVLink serial port')
    parser.add_argument('--baud', type=int, default=921600,
                        help='MAVLink baud rate')
    parser.add_argument('--sim',  action='store_true',
                        help='Simulation mode (no hardware output)')
    args = parser.parse_args()

    mco = MCOMain(port=args.port, baud=args.baud, sim=args.sim)
    mco.start()


if __name__ == '__main__':
    main()
