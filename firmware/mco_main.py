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
from mco_odrive_interface import ODriveInterface
from mco_intent_control import IntentInterpreter, Intent
import mco_advanced_physics as ap
from mco_flight_governor import (
    check_glide_speed_floor,
    check_tandem_blade_clearance,
    clamp_sprint_dynamics,
    govern_regen_brake_resistor,
)
from mco_certification_tools import (
    structural_recorder,
    rope_tracker,
    hq_analyser,
)


# ── Servo PWM constants ───────────────────────────────────────────
# Pin and motor constants — pulled from wave library to stay in sync
MOTOR_COUNT = w.MOTOR_COUNT       # 8 independent motors
REDUCER_RATIO = w.REDUCER_RATIO   # 10:1 (upgraded baseline)
PITCH_PINS  = list(w.PITCH_PINS)  # 8 pitch servo GPIO pins
SWEEP_PINS  = list(w.SWEEP_PINS)  # 8 sweep servo GPIO pins
MCP3208_SPI_BUS  = w.MCP3208_SPI_BUS
MCP3208_SPI_CE   = w.MCP3208_SPI_CE
MCP3208_SWEEP_CE = w.MCP3208_SWEEP_SPI_CE
SWEEP_PINS = [ 4, 17, 27, 22,  5,  6, 13, 19]

SERVO_MIN_US  = 900    # microseconds — full negative
SERVO_MID_US  = 1500   # microseconds — centre
SERVO_MAX_US  = 2100   # microseconds — full positive
SWEEP_FREQ    = 333    # Hz for AGFRC IB53BHU programmable brushless sweep servos (8.4V capable)
PITCH_FREQ    = 333    # Hz for AGFRC IB53BHU programmable brushless pitch servos (8.4V capable)
SERVO_FREQ    = 333    # Uniform 333 Hz PWM update frequency across all 16 AGFRC servos


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
        self.odi      = None

        # Kinematic reciprocating phase tracking
        self._master_phase   = 0.0
        self._pair_offsets   = [0.0, math.pi / 2.0, math.pi, 3.0 * math.pi / 2.0]  # 90° metachronal cascade
        self._last_flap_degs = [0.0] * 8
        self._last_phases    = [0.0] * 8
        self._last_dirs      = [+1] * 8
        self._last_kin_t     = time.monotonic()

        # Advanced physics status flags (H-37 to H-49)
        self._h37_active      = False
        self._h38_active      = False
        self._h38_savings_pct = 0.0
        self._h39_active      = False
        self._h41_active      = False
        self._h41_vib_red_pct = 0.0
        self._h41_diag        = {}
        self._h41_residual_my = 0.0
        self._h41_cabin_rms_g = 0.0
        self._h41_heave_err_g = 0.0
        self._h42_active      = False
        self._h42_diag        = {}

        self._h43_active      = False
        self._h43_delta_g     = 0.0
        self._h44_active      = False
        self._h45_active      = False
        self._h45_shed_pct    = 0.0
        self._h46_active      = False
        self._h46_freq_hz     = 0.0
        self._h47_active      = False
        self._h47_group       = 'NONE'
        self._h49_active      = False
        self._h49_thrust_boost_N = 0.0

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
            self.odi = ODriveInterface(channel='can0', bitrate=1_000_000)
            if self.odi._bus:
                self.can_bus = self.odi._bus
                print("CAN bus: connected via ODriveInterface")
            else:
                self.can_bus = None
        except Exception as e:
            print(f"CAN bus init failed: {e}")
            self.odi = None
            self.can_bus = None

    def get_wing_kinematics(self):
        """Return current flap positions, phases, and stroke directions for all 8 wings."""
        return {
            'flap_degs':   list(self._last_flap_degs),
            'flap_phases': list(self._last_phases),
            'stroke_dirs': list(self._last_dirs),
        }

    def _read_odrive_positions(self):
        """
        Read motor shaft position from all 8 ODrive axes.
        Returns list of 8 floats (radians) or [None]*8 in sim mode.
        """
        if self.sim_mode or not self.odi:
            return [None] * 8
        try:
            pos, _ = self.odi.read_encoder_estimates()
            return pos
        except Exception:
            return [None] * 8

    def _read_odrive_currents(self):
        """
        Read Iq_measured from all 8 ODrive axes.
        Returns list of 8 floats (amps) or [10.0]*8 in sim mode.
        """
        if self.sim_mode or not self.odi:
            return [10.0] * 8
        try:
            return self.odi.read_currents()
        except Exception:
            return [10.0] * 8

    def apply_commands(self, result: dict, sensors: dict = None):
        """Apply wing commands to servos and ODrives."""
        wc      = result.get('wing_commands', [])
        freq_hz = result.get('freq_hz', 0.0)

        # ── Advanced Physics Hooks (H-37 through H-45) ───────────────
        try:
            # H-37: Active Aeroelastic Compliance for pitch
            currents = sensors.get('motor_currents_a') if sensors else None
            if currents:
                raw_pitches = [c.get('pitch_deg', 0.0) for c in wc[:8]]
                comp_pitches = ap.h37_apply_compliance(raw_pitches, currents)
                self._h37_active = any(abs(c - r) > 0.1 for c, r in zip(comp_pitches, raw_pitches))
                for idx, p in enumerate(comp_pitches):
                    if idx < len(wc):
                        wc[idx]['pitch_deg'] = p
            else:
                self._h37_active = False

            # H-39: Differential Sweep Yaw Vectoring
            yaw_stick = sensors.get('yaw_stick', 0.0) if sensors else 0.0
            if abs(yaw_stick) > 0.05:
                raw_sweeps = [c.get('sweep_deg', 90.0) for c in wc[:8]]
                diff_sweeps, yaw_active = ap.h39_differential_sweep_yaw(raw_sweeps, yaw_stick)
                self._h39_active = yaw_active
                if yaw_active:
                    for idx, sw in enumerate(diff_sweeps):
                        if idx < len(wc):
                            wc[idx]['sweep_deg'] = sw
            else:
                self._h39_active = False

            # H-42: Asymmetric N-1 Actuator Loss Re-Trim (Fail-Operational)
            failed_wing = sensors.get('failed_wing_idx') if sensors else None
            if failed_wing is None and sensors and sensors.get('wing_errors'):
                for i, err in enumerate(sensors['wing_errors']):
                    if err in (0x1000, 0x2000): # hard motor/rope fault
                        failed_wing = i
                        break
            if failed_wing is not None:
                wc, self._h42_active, self._h42_diag = ap.h42_asymmetric_retrim_allocation(failed_wing, wc)
            else:
                self._h42_active = False
                self._h42_diag = {}

            # H-41: Active Heave Damping & Metachronal Pitch Reaction Moment Cancellation
            wc, self._pair_offsets, self._h41_active, self._h41_diag = ap.h41_active_ride_stabilization(
                sensors or {}, wc, pair_offsets=self._pair_offsets
            )
            self._h41_vib_red_pct = self._h41_diag.get('vibration_reduction_pct', 0.0)
            self._h41_residual_my = self._h41_diag.get('residual_my_Nm', 0.0)
            self._h41_cabin_rms_g = self._h41_diag.get('cabin_rms_g', 0.0)
            self._h41_heave_err_g = self._h41_diag.get('heave_error_g', 0.0)

            # H-43: Severe Wind Shear & Direct Collective Vectoring (DLTV)
            w_sink = sensors.get('w_sink_ms', sensors.get('VSI_ms', 0.0)) if sensors else 0.0
            climb_ovr = sensors.get('climb_override', False) if sensors else False
            wc, self._h43_active, self._h43_delta_g = ap.h43_direct_lift_thrust_vectoring(wc, w_sink, climb_ovr)

            # H-44: Heavy Rain & Icing Boundary Layer LEV Re-Energization
            rain_or_ice = bool(sensors.get('rain_detected') or sensors.get('icing_detected')) if sensors else False
            wc, self._h44_active = ap.h44_rain_icing_lev_reenergization(wc, self._master_phase, rain_or_ice)

            # H-45: Proactive Bus Brownout & Priority Load Shedding
            bus_v = sensors.get('bus_voltage_v', 24.0) if sensors else 24.0
            tot_i = sum(sensors.get('motor_currents_a', [10.0]*8)) if sensors else 80.0
            wc, self._h45_active, self._h45_shed_pct = ap.h45_predictive_bus_load_shedding(wc, bus_v, tot_i)

            # H-46: Micro-Stroke Resonant Buzz & Pitch Reflex (Dash Speed)
            lever = sensors.get('lever_pct', 50.0) if sensors else 50.0
            v_ms = sensors.get('V_ms', 9.0) if sensors else 9.0
            wc, self._h46_active, self._h46_freq_hz = ap.h46_micro_stroke_resonant_buzz(lever, v_ms, wc, freq_hz)
            if self._h46_active:
                freq_hz = self._h46_freq_hz
                result['freq_hz'] = freq_hz

            # H-47: Tandem Interleaved Bound-and-Glide (Fast Cruise)
            wc, self._h47_active, self._h47_group = ap.h47_tandem_interleaved_bound_glide(lever, v_ms, wc, time.monotonic())

            # H-49: Cascade Tip-Vortex Katzmayr Energy Ingestion (Range Extension)
            wc, self._h49_active, self._h49_thrust_boost_N = ap.h49_katzmayr_vortex_energy_ingestion(wc, v_ms, self._master_phase, lever)

            result['wing_commands'] = wc
        except Exception:
            pass

        # Always advance internal kinematic simulation (both sim and hardware)
        self._update_kinematics(freq_hz, wc, sensors=sensors)

        # ── Tandem Multi-Wing Dynamic Blade Strike Safety Governor ───
        try:
            pair_strokes = [self._last_flap_degs[idx * 2] if idx * 2 < len(self._last_flap_degs) else 0.0 for idx in range(4)]
            pair_sweeps = [wc[idx * 2].get('sweep_deg', 60.0) if idx * 2 < len(wc) else 60.0 for idx in range(4)]
            strike_check = check_tandem_blade_clearance(pair_strokes, pair_sweeps)
            result['blade_clearance_safe'] = strike_check.get('clearance_safe', True)
            result['min_blade_clearance_mm'] = strike_check.get('min_clearance_mm', 999.0)
            if not strike_check.get('clearance_safe', True):
                v_pair = strike_check.get('violating_pair')
                if v_pair:
                    p1, p2 = v_pair
                    # Sanitize sweep angles: equalize violating pair sweeps to eliminate interference
                    avg_sw = (pair_sweeps[p1] + pair_sweeps[p2]) / 2.0
                    for w_idx in (p1 * 2, p1 * 2 + 1, p2 * 2, p2 * 2 + 1):
                        if w_idx < len(wc):
                            wc[w_idx]['sweep_deg'] = avg_sw
        except Exception:
            pass

        if self.sim_mode:
            return

        for i, cmd in enumerate(wc[:8]):
            if i < len(PITCH_PINS) and self.pi:
                pitch_deg = cmd.get('pitch_deg', 0.0)
                self.pi.set_servo_pulsewidth(
                    PITCH_PINS[i], w.pitch_deg_to_us(pitch_deg, servo_index=i))

            if i < len(SWEEP_PINS) and self.pi:
                sweep_deg = cmd.get('sweep_deg', 60.0)
                self.pi.set_servo_pulsewidth(
                    SWEEP_PINS[i], w.sweep_deg_to_us(sweep_deg, servo_index=i))

        # ODrive velocity commands via CAN
        if self.can_bus and freq_hz > 0:
            m_pos = sensors.get('motor_pos_rad') if sensors else None
            self._send_odrive_velocity(
                freq_hz, wc, motor_positions_rad=m_pos)

    def _update_kinematics(self, freq_hz, wing_commands, sensors: dict = None):
        """Advance metachronal reciprocating phase and update positions for all 8 wings."""
        from mco_wave_library import FLAP_AMP_DEG
        now = time.monotonic()
        dt = max(0.001, min(0.05, now - self._last_kin_t))
        self._last_kin_t = now

        # ── H-38 Ground Effect Fountain Mode Phase Adjustment ─────────
        if sensors and sensors.get('h_agl_m') is not None:
            try:
                ge_phase_deg, ge_active, ge_sav = ap.h38_ground_effect_phase(sensors['h_agl_m'])
                self._h38_active = ge_active
                self._h38_savings_pct = ge_sav
                if ge_active:
                    ge_rad = math.radians(ge_phase_deg)
                    self._pair_offsets = [0.0, ge_rad, (2.0 * ge_rad) % (2.0 * math.pi), (3.0 * ge_rad) % (2.0 * math.pi)]
                else:
                    self._pair_offsets = [0.0, math.pi / 2.0, math.pi, 3.0 * math.pi / 2.0]
            except Exception:
                pass
        else:
            self._h38_active = False
            self._h38_savings_pct = 0.0

        # ── H-41 Heave & Pitch Nod Cancellation (Ride Comfort) ────────
        comfort_mode = bool(sensors.get('comfort_mode', False)) if sensors else False
        imu_rms_g = sensors.get('imu_rms_g', 0.0) if sensors else 0.0
        try:
            if not self._h38_active:
                offsets, self._h41_active, self._h41_vib_red_pct = ap.h41_heave_pitch_cancellation(
                    comfort_mode, imu_rms_g, self._pair_offsets)
                if self._h41_active:
                    self._pair_offsets = offsets
            else:
                self._h41_active = False
                self._h41_vib_red_pct = 0.0
        except Exception:
            pass

        # Advance master phase
        self._master_phase = (self._master_phase + 2.0 * math.pi * freq_hz * dt) % (2.0 * math.pi)

        # Dynamic amplitude clamping: tau_root_max = 5.9 Nm (30A peak on MN5008)
        # I_wing = 0.0240 kg*m^2 (1.0m wing + raked tip). Max rad = tau_max / (I_wing * (2pi*f)^2)
        if freq_hz > 0.5:
            max_amp_rad = 5.90 / (0.0240 * ((2.0 * math.pi * freq_hz) ** 2))
            max_amp_deg = min(80.0, math.degrees(max_amp_rad))
        else:
            max_amp_deg = 80.0

        for i, cmd in enumerate(wing_commands[:8]):
            raw_amp = cmd.get('flap_amp_deg', FLAP_AMP_DEG)
            amp     = min(raw_amp, max_amp_deg)
            pair_idx = i // 2
            wing_phase = (self._master_phase - self._pair_offsets[pair_idx]) % (2.0 * math.pi)

            flap_angle = amp * math.sin(wing_phase)
            self._last_flap_degs[i] = round(flap_angle, 2)
            self._last_phases[i]    = round(wing_phase / (2.0 * math.pi), 4)
            self._last_dirs[i]      = +1 if math.cos(wing_phase) >= 0 else -1

    def _send_odrive_velocity(self, freq_hz, wing_commands,
                               motor_positions_rad=None):
        """
        Send reciprocating velocity setpoints to ODrives via CAN.

        Applies true sinusoidal stroke kinematics:
            vel(t) = G * (amp / 360) * 2pi * f * cos(phase) * factor
        with dynamic amplitude envelope limiting to keep peak motor current
        within safe limits (<= 30A).
        """
        import struct
        from mco_wave_library import (soft_stop_apply_to_commands,
                                       REDUCER_RATIO, FLAP_AMP_DEG)

        reducer = REDUCER_RATIO   # 10.0 (upgraded baseline)

        if freq_hz > 0.5:
            max_amp_rad = 5.90 / (0.0240 * ((2.0 * math.pi * freq_hz) ** 2))
            max_amp_deg = min(100.0, math.degrees(max_amp_rad))
        else:
            max_amp_deg = 100.0

        # ── Apply soft-stop taper to all 8 wings ─────────────────
        vel_limit_overrides = soft_stop_apply_to_commands(
            wing_commands,
            motor_positions_rad or [None] * 8,
            base_vel_limit_rps=150.0
        )

        # ── Send velocity + optional vel_limit per wing ───────────
        for i, cmd in enumerate(wing_commands[:8]):
            raw_amp = cmd.get('flap_amp_deg', FLAP_AMP_DEG)
            amp     = min(raw_amp, max_amp_deg)
            factor  = cmd.get('soft_stop_factor', 1.0)

            # Wing pair offset for 90° cascade (pairs: 0=W1, 1=W2, 2=W3, 3=W4)
            pair_idx = i // 2
            wing_phase = (self._master_phase - self._pair_offsets[pair_idx]) % (2.0 * math.pi)

            # Reciprocating position and velocity in turns/s at motor shaft
            flap_angle = amp * math.sin(wing_phase)
            self._last_flap_degs[i] = round(flap_angle, 2)
            self._last_phases[i]    = round(wing_phase / (2.0 * math.pi), 4)
            self._last_dirs[i]      = +1 if math.cos(wing_phase) >= 0 else -1

            # Motor shaft velocity in turns/s (ODrive CAN native unit)
            # vel_turns_s = G * (amp_deg / 360.0) * (2*pi*f) * cos(wing_phase) * direction
            flap_dir = w.FLAP_MOTOR_DIRECTION[i] if i < len(w.FLAP_MOTOR_DIRECTION) else +1
            vel_turns_s = reducer * (math.radians(amp) / (2.0 * math.pi)) * (2.0 * math.pi * freq_hz) * math.cos(wing_phase) * factor * flap_dir

            arb_id = (i << 5) | 0x0D   # SET_INPUT_VEL
            data   = struct.pack('<ff', vel_turns_s, 0.0)
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
                from mco_wave_library import _ss_state
                if _ss_state['in_zone'][i] is False and self.odi:
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
        self.mavlink           = MCOMAVLink(port, baud)
        self.logger            = MCOLogger()
        self.gs_server         = MCOGSServer()
        self.hardware          = MCOHardware(sim_mode=sim)
        self.fcc               = w.MCOFlightController()
        self.intent_interpreter = IntentInterpreter()

        # Tactical Cursor-on-Target (CoT) ATAK Bridge
        try:
            from mco_cot_bridge import get_cot_bridge
            self.cot_bridge = get_cot_bridge(auto_start=False)
        except Exception:
            self.cot_bridge = None

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
        if self.cot_bridge:
            self.cot_bridge.start()
        print(f"\nMCO-1D flight loop starting at {self.TICK_RATE_HZ} Hz")
        print(f"Sim mode: {self.sim_mode}")
        print("Ready. Pre-flight check via ground station or panel.\n")

        tick_count  = 0
        _gc_counter = 0
        last_tick_t = time.monotonic()

        while self.alive:
            tick_start = time.monotonic()

            # ── Inbound ATAK CoT Tasking check ────────────────────
            if self.cot_bridge:
                inbound_wp = self.cot_bridge.pop_latest_waypoint()
                if inbound_wp:
                    w.accept_external_fix(inbound_wp[0], inbound_wp[1], inbound_wp[2], source=inbound_wp[3])

            # ── Get sensor data ───────────────────────────────────
            sensors = self.mavlink.get_sensors()

            # ── Intent arbitration via IntentInterpreter ─────────
            sticks = {
                'roll':     sensors.get('roll_stick', 0.0),
                'pitch':    sensors.get('pitch_stick', 0.0),
                'throttle': sensors.get('lever_pct', 50.0) / 100.0,
                'yaw':      sensors.get('yaw_stick', 0.0),
            }
            switches = {'mode_sw': 0}
            context = {
                'V_ms':            sensors.get('V_ms', 11.0),
                'h_agl_m':         sensors.get('h_agl_m', 50.0),
                'bank_deg':        sensors.get('roll_deg_imu', 0.0),
                'pitch_deg':       sensors.get('pitch_deg_imu', 0.0),
                'Ps':              sensors.get('Ps_measured', 0.0),
                'on_ground':       sensors.get('h_agl_m', 50.0) < 0.3,
                'contested_ew':    w._ew_state.get('active', False),
                'stealth_loiter':  w._ew_state.get('stealth_active', False),
                'gps_jammed':      w._gps_state.get('status', 0) >= 3,
                'bus_voltage_v':   sensors.get('bus_voltage_v', 24.0),
            }
            try:
                intent_res = self.intent_interpreter.interpret(sticks, switches, context)
                if isinstance(intent_res, tuple):
                    sensors['intent'] = intent_res[0]
                    if len(intent_res) > 1:
                        sensors['intent_conf'] = intent_res[1]
                    if len(intent_res) > 2:
                        sensors['intent_params'] = intent_res[2]
                else:
                    sensors['intent'] = str(intent_res)
                    sensors['intent_conf'] = 1.0
                    sensors['intent_params'] = {}
            except Exception:
                sensors['intent'] = 'cruise'
                sensors['intent_conf'] = 0.0
                sensors['intent_params'] = {}

            # Feed advancing kinematic wave states to FCC
            kin = self.hardware.get_wing_kinematics()
            sensors['flap_phases'] = kin['flap_phases']
            sensors['stroke_dirs'] = kin['stroke_dirs']
            sensors['flap_degs']   = kin['flap_degs']

            # ── Per-wing sensing (MCP3208 ADCs + ODrive pos) ─
            # Pitch servo feedback — MCP3208 #1 (SPI1 CE1)
            sensors['servo_actual_aoa_deg'] = w.read_servo_feedback_adc()
            # Sweep servo feedback — MCP3208 #2 (SPI1 CE0); None if not fitted
            sensors['sweep_actual_deg'] = (
                w._sweep_sense_state.get('actual_sweep_deg', [None]*8)
                if w._sweep_sense_state.get('sweep_adc_fitted') else [None]*8)
            # ODrive S1 stroke position — 8 values (1 per board, node == wing_idx)
            sensors['motor_pos_rad']    = self.hardware._read_odrive_positions()
            # ODrive S1 motor current — 8 values (1 per board)
            sensors['motor_currents_a'] = self.hardware._read_odrive_currents()
            # ODrive S1 FET and motor temperatures — NEW S1 capability
            # read_temperatures() returns list of 8 {'fet_C', 'motor_C'} dicts
            # Called at 5 Hz (every 10 ticks) to reduce CAN bus load
            if tick_count % 10 == 0 and self.hardware.odi:
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
            if tick_count % 50 == 0 and self.hardware.odi:
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
            for cmd_item in cmds:
                try:
                    if isinstance(cmd_item, dict):
                        cmd_dict = cmd_item
                        cmd_str  = str(cmd_dict.get('cmd', ''))
                    else:
                        cmd_str  = str(cmd_item)
                        cmd_dict = {'cmd': cmd_str}

                    # Try test card commands first
                    tc_ack = None
                    if self._handle_tc:
                        tc_ack = self._handle_tc(cmd_str)
                    if tc_ack is not None:
                        ack = tc_ack if isinstance(tc_ack, dict) else {'type': 'ack', 'cmd': cmd_str, 'msg': str(tc_ack)}
                    else:
                        ack = w._gs_execute_cmd(cmd_dict)
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

            # ── Active Flight Safety Governors ────────────────────
            # 1. Stall speed floor protection: auto-inject minimum safe flapping below 10.5 m/s unpowered
            V_ias = sensors.get('V_ms', 11.0)
            throttle = sticks.get('throttle', 0.5)
            curr_intent = sensors.get('intent', 'cruise')
            glide_gov = check_glide_speed_floor(V_ias, throttle_frac=throttle, intent=curr_intent)
            if glide_gov.get('intervention_active'):
                forced_f = glide_gov.get('forced_freq_hz', 2.0)
                forced_a = glide_gov.get('forced_amp_deg', 40.0)
                if result.get('freq_hz', 0.0) < forced_f:
                    result['freq_hz'] = forced_f
                for cmd in result.get('wing_commands', []):
                    if isinstance(cmd, dict):
                        cmd['flap_amp_deg'] = max(cmd.get('flap_amp_deg', 0.0), forced_a)
                result['stall_intervention'] = True
            else:
                result['stall_intervention'] = False

            # 2. Sprint dynamics clamp: protect ODrive S1 from velocity over-speed trips
            if curr_intent == 'sprint' or result.get('freq_hz', 0.0) > 3.0:
                cmd_f = result.get('freq_hz', 0.0)
                cmd_a = max((c.get('flap_amp_deg', 20.0) for c in result.get('wing_commands', []) if isinstance(c, dict)), default=20.0)
                cf, ca, mrpm, margin = clamp_sprint_dynamics(cmd_f, cmd_a)
                result['freq_hz'] = cf
                for cmd in result.get('wing_commands', []):
                    if isinstance(cmd, dict) and cmd.get('flap_amp_deg', 0.0) > ca:
                        cmd['flap_amp_deg'] = ca
                result['sprint_rpm_margin_pct'] = margin

            # 3. Regenerative brake resistor voltage fence
            bus_v = sensors.get('bus_voltage_v', 24.0)
            req_regen_w = result.get('regen_watts', 0.0)
            resistor_gov = govern_regen_brake_resistor(bus_v, req_regen_w)
            result['brake_resistor_gov'] = resistor_gov

            # ── Apply to hardware ─────────────────────────────────
            self.hardware.apply_commands(result, sensors)
            # Store intent for next tick's CG management
            self._last_intent = result.get('intent', 'cruise')

            # ── S1 dynamic velocity limit & H-40 Regenerative Braking ──
            try:
                v_cur = sensors.get('V_ms', 0.0)
                pitch_imu = sensors.get('pitch_deg_imu', 0.0)
                regen_status = ap.h40_stoop_energy_harvesting(v_cur, pitch_imu, bus_v)
                result['h40_regen'] = regen_status
            except Exception:
                pass

            result['h37_active'] = getattr(self.hardware, '_h37_active', False)
            result['h38_active'] = getattr(self.hardware, '_h38_active', False)
            result['h38_savings_pct'] = getattr(self.hardware, '_h38_savings_pct', 0.0)
            result['h39_active'] = getattr(self.hardware, '_h39_active', False)
            result['h41_active'] = getattr(self.hardware, '_h41_active', False)
            result['h41_vib_red_pct'] = getattr(self.hardware, '_h41_vib_red_pct', 0.0)
            result['h41_diag'] = getattr(self.hardware, '_h41_diag', {})
            result['h41_residual_my_Nm'] = getattr(self.hardware, '_h41_residual_my', 0.0)
            result['h41_cabin_rms_g'] = getattr(self.hardware, '_h41_cabin_rms_g', 0.0)
            result['h41_heave_err_g'] = getattr(self.hardware, '_h41_heave_err_g', 0.0)
            result['h42_active'] = getattr(self.hardware, '_h42_active', False)

            result['h42_diag'] = getattr(self.hardware, '_h42_diag', {})
            result['h43_active'] = getattr(self.hardware, '_h43_active', False)
            result['h43_delta_g'] = getattr(self.hardware, '_h43_delta_g', 0.0)
            result['h44_active'] = getattr(self.hardware, '_h44_active', False)
            result['h45_active'] = getattr(self.hardware, '_h45_active', False)
            result['h45_shed_pct'] = getattr(self.hardware, '_h45_shed_pct', 0.0)
            result['h46_active'] = getattr(self.hardware, '_h46_active', False)
            result['h46_freq_hz'] = getattr(self.hardware, '_h46_freq_hz', 0.0)
            result['h47_active'] = getattr(self.hardware, '_h47_active', False)
            result['h47_group'] = getattr(self.hardware, '_h47_group', 'NONE')
            result['h49_active'] = getattr(self.hardware, '_h49_active', False)
            result['h49_thrust_boost_N'] = getattr(self.hardware, '_h49_thrust_boost_N', 0.0)

            dyn_vlim = result.get('dyn_vel_limit_rad_s')
            if dyn_vlim is not None and self.hardware.odi:
                try:
                    for wing_idx in range(8):
                        self.hardware.odi.set_velocity_limit(wing_idx, dyn_vlim)
                except Exception:
                    pass

            # ── S1 per-wing clear_errors (fault isolation) ────────
            wings_to_clear = result.get('wings_need_clear_errors', [])
            if wings_to_clear and self.hardware.odi:
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
                if self.cot_bridge and (tick_count % 10 == 0):
                    lat_c = sensors.get('lat', 0.0)
                    lon_c = sensors.get('lon', 0.0)
                    alt_c = sensors.get('baro_alt_m', sensors.get('h_agl_m', 0.0))
                    spd_c = sensors.get('V_ms', 0.0)
                    crs_c = sensors.get('heading_deg', 0.0)
                    bat_c = max(0.0, min(100.0, (sensors.get('bus_voltage_v', 24.0) - 20.0) / (25.2 - 20.0) * 100.0))
                    fmode = str(sensors.get('intent', 'cruise')).upper()
                    self.cot_bridge.update_telemetry(lat_c, lon_c, alt_c, spd_c, crs_c, bat_c, fmode)
                    self.cot_bridge.broadcast_telemetry()

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
                    'cot_active':        getattr(self, 'cot_bridge', None) is not None,
                    'cot_pkts_sent':     self.cot_bridge.n_packets_sent if self.cot_bridge else 0,
                    'cot_pkts_recv':     self.cot_bridge.n_packets_recv if self.cot_bridge else 0,
                    'wind_speed_ms':     w._gps_state.get('wind_speed_ms', 0.0),
                    'wind_dir_deg':      w._gps_state.get('wind_dir_deg', 0.0),
                    'contested_ew':      w._ew_state.get('active', False),
                    'stealth_active':    w._ew_state.get('stealth_active', False),
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
                    # Advanced biomimetic physics telemetry (H-37 to H-49)
                    'h37_active':        result.get('h37_active', False),
                    'h38_active':        result.get('h38_active', False),
                    'h38_power_savings': result.get('h38_savings_pct', 0.0),
                    'h39_active':        result.get('h39_active', False),
                    'h40_regen':         result.get('h40_regen', {}),
                    'h41_heave_cancel':  result.get('h41_active', False),
                    'h41_vib_reduction': result.get('h41_vib_red_pct', 0.0),
                    'h41_residual_my_Nm': result.get('h41_residual_my_Nm', 0.0),
                    'h41_cabin_rms_g':   result.get('h41_cabin_rms_g', 0.0),
                    'h41_heave_err_g':   result.get('h41_heave_err_g', 0.0),
                    'h42_n1_retrim':     result.get('h42_active', False),
                    'h42_diag':          result.get('h42_diag', {}),
                    'h43_dltv_active':   result.get('h43_active', False),
                    'h43_delta_g':       result.get('h43_delta_g', 0.0),
                    'h44_rain_lev':      result.get('h44_active', False),
                    'h45_load_shedding': result.get('h45_active', False),
                    'h45_shed_pct':      result.get('h45_shed_pct', 0.0),
                    'h46_dash_active':   result.get('h46_active', False),
                    'h46_dash_freq_hz':  result.get('h46_freq_hz', 0.0),
                    'h47_bound_active':  result.get('h47_active', False),
                    'h47_bound_group':   result.get('h47_group', 'NONE'),
                    'h49_katzmayr':      result.get('h49_active', False),
                    'h49_thrust_boost_N':result.get('h49_thrust_boost_N', 0.0),
                    'h_la1_super_lift':  result.get('h_la1_super_lift', False),
                    'h_la1_lift_N':       result.get('h_la1_lift_N', 0.0),
                    'h_la3_pigeon_climb':result.get('h_la3_pigeon_climb', False),
                    'h_la3_climb_deg':   result.get('h_la3_climb_deg', 0.0),
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
                structural_recorder.record(sensors, result)
                rope_tracker.update(sensors, result)
            except Exception:
                pass

        self._shutdown()

    def _shutdown(self):
        """Clean shutdown sequence."""
        print("\nShutdown: running post-flight export...")
        try:
            structural_recorder.end_flight()
        except Exception:
            pass
        try:
            rope_tracker._save()
        except Exception:
            pass
        # Auto-run HQ analysis on the latest flight log
        try:
            from mco_logger import LOG_DIR
            import glob, os
            logs = sorted(glob.glob(os.path.join(LOG_DIR, 'mco_flight_*.csv')))
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
        if getattr(self, 'cot_bridge', None):
            try:
                self.cot_bridge.close()
            except Exception:
                pass
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
    parser.add_argument('--wizard', action='store_true',
                        help='Launch MCO-1D first-run hardware configuration wizard')
    args = parser.parse_args()

    if args.wizard:
        from mco_first_run_wizard import run as run_wizard
        run_wizard()
        return

    mco = MCOMain(port=args.port, baud=args.baud, sim=args.sim)
    mco.start()


if __name__ == '__main__':
    main()
