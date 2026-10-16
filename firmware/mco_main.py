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
    check_pitot_saturation,
    clamp_sprint_dynamics,
    clamp_universal_kinematics,
    govern_regen_brake_resistor,
    get_sprint_thermal_status,
    govern_dive_recovery,
    get_dive_recovery_status,
    govern_ground_wind_safety,
    govern_servo_bus,
    govern_weight_on_wheels,
    govern_servo_rates,
    govern_turn_coordination,
    set_turn_coordination,
    is_turn_coordination_enabled,
    govern_strouhal_cruise,
    govern_cascade_slipstream,
    check_pitot_blockage_and_synthetic_airspeed,
    govern_desert_thermal_interleaving,
    govern_preflight_snow_shake,
    govern_zero_friction_ice_steering,
    estimate_cold_lipo_ocv,
    govern_extreme_gust_squeeze,
    govern_pilot_alpine_takeoff_and_rollout,
    govern_unsteady_lev_rain,
    govern_orographic_wave_loiter,
    govern_wrist_gear_damage,
    govern_symmetrical_rth,
    reset_symmetrical_rth_governor,
    govern_hard_wig_floor,
)
from mco_config import (
    ADC_PITCH_CE,
    ADC_SPI_BUS,
    ADC_SWEEP_CE,
    GEAR_RATIO,
    MAVLINK_BAUD,
    MAVLINK_PORT,
    MOTOR_COUNT as CONFIG_MOTOR_COUNT,
    PITCH_GPIO_PINS,
    SERVO_PWM_RATE_HZ,
    SWEEP_GPIO_PINS,
)
from mco_kinematics import MetachronalSynthesizer


# ── Servo PWM constants ───────────────────────────────────────────
# Pin and motor constants — pulled from mco_config.py single source of truth
MOTOR_COUNT = CONFIG_MOTOR_COUNT
REDUCER_RATIO = int(GEAR_RATIO)
PITCH_PINS  = list(PITCH_GPIO_PINS)   # 8 pitch servo GPIO pins: [4, 5, 6, 12, 13, 16, 17, 18]
SWEEP_PINS  = list(SWEEP_GPIO_PINS)   # 8 sweep servo GPIO pins: [19, 20, 21, 22, 23, 24, 25, 26]
MCP3208_SPI_BUS  = ADC_SPI_BUS
MCP3208_SPI_CE   = ADC_PITCH_CE
MCP3208_SWEEP_CE = ADC_SWEEP_CE

SERVO_MIN_US  = 900    # microseconds — full negative
SERVO_MID_US  = 1500   # microseconds — centre
SERVO_MAX_US  = 2100   # microseconds — full positive
SWEEP_FREQ    = SERVO_PWM_RATE_HZ
PITCH_FREQ    = SERVO_PWM_RATE_HZ
SERVO_FREQ    = SERVO_PWM_RATE_HZ


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

        # Deterministic 50 Hz metachronal kinematics synthesizer
        self.kinematics      = MetachronalSynthesizer()
        self._master_phase   = 0.0
        self._pair_offsets   = [0.0, math.pi / 2.0, math.pi, 3.0 * math.pi / 2.0]  # 90° metachronal cascade
        self._last_flap_degs = [0.0] * 8
        self._last_phases    = [0.0] * 8
        self._last_dirs      = [+1] * 8
        self._last_pitch_servo_degs = [0.0] * 8
        self._last_sweep_servo_degs = [60.0] * 8
        self._last_motor_pos_turns  = [0.0] * 8
        self._last_motor_vel_rps    = [0.0] * 8
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
        self._h41_stall_margin_deg = 13.0
        self._h41_servo_derate     = 1.0
        self._h42_active      = False
        self._h42_diag        = {}
        self._symmetrical_rth_active = False
        self._symmetrical_rth_telem  = {}

        self._h43_active      = False
        self._h43_delta_g     = 0.0
        self._h43_freq_boost  = 0.0
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
        """Return current flap positions, phases, stroke directions, and servo/motor targets for all 8 wings."""
        kin = self.kinematics.get_wing_kinematics()
        kin['flap_degs']   = list(self._last_flap_degs)
        kin['flap_phases'] = list(self._last_phases)
        kin['stroke_dirs'] = list(self._last_dirs)
        return kin

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

            # H-39: Differential Sweep Yaw Vectoring & Crosswind Sideslip Decoupling
            yaw_stick = sensors.get('yaw_stick', 0.0) if sensors else 0.0
            sideslip_beta = sensors.get('sideslip_beta_deg', sensors.get('beta_deg', 0.0)) if sensors else 0.0
            if abs(yaw_stick) > 0.05 or abs(sideslip_beta) > 0.5:
                raw_sweeps = [c.get('sweep_deg', 90.0) for c in wc[:8]]
                max_amp = max((c.get('flap_amp_deg', 80.0) for c in wc[:8]), default=80.0)
                diff_sweeps, yaw_active = ap.h39_differential_sweep_yaw(
                    raw_sweeps, yaw_stick, sideslip_beta_deg=sideslip_beta, flap_amp_deg=max_amp
                )
                self._h39_active = yaw_active
                if yaw_active:
                    for idx, sw in enumerate(diff_sweeps):
                        if idx < len(wc):
                            wc[idx]['sweep_deg'] = sw
            else:
                self._h39_active = False

            # Progressive Wrist Gear Damage Monitor & Mitigation (H-GEAR-COPE-1)
            actual_pitches = sensors.get('servo_actual_aoa_deg') if sensors else None
            actual_sweeps = sensors.get('sweep_actual_deg') if sensors else None
            wrist_gear_telem = govern_wrist_gear_damage(
                wing_commands=wc,
                actual_pitch_degs=actual_pitches,
                actual_sweep_degs=actual_sweeps,
                dt=0.02,
                result=result
            )
            self._wrist_gear_telem = wrist_gear_telem
            result['wrist_gear_health'] = wrist_gear_telem

            # H-42: Asymmetric N-1 Actuator Loss Re-Trim (Fail-Operational)
            failed_wing = sensors.get('failed_wing_idx') if sensors else None
            if failed_wing is None and wrist_gear_telem.get('primary_failed_wing') is not None:
                failed_wing = wrist_gear_telem['primary_failed_wing']
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

            # H-RTH-6WING / H-42-RTH: Symmetrical Pair Quarantine & 6-Wing Sustained RTH
            failed_wings = []
            if failed_wing is not None:
                failed_wings.append(failed_wing)
            if sensors and sensors.get('failed_wings'):
                fw = sensors['failed_wings']
                if isinstance(fw, (list, tuple, set)):
                    failed_wings.extend(list(fw))
                elif isinstance(fw, int):
                    failed_wings.append(fw)
            if wrist_gear_telem.get('stripped_wings'):
                failed_wings.extend(wrist_gear_telem['stripped_wings'])
            failed_wings = sorted(list(set(w for w in failed_wings if 0 <= w < 8)))

            curr_intent = (sensors.get('intent') or result.get('intent') or 'cruise') if (sensors or result) else 'cruise'
            curr_intent = str(curr_intent).lower()

            h_agl = sensors.get('h_agl_m', 50.0) if sensors else 50.0
            frozen_angles = sensors.get('frozen_angles', {}) if sensors else {}
            rope_lost = bool(sensors.get('rope_integrity_lost', False)) if sensors else False

            rth_telem = govern_symmetrical_rth(
                wing_commands=wc,
                failed_wings=failed_wings if failed_wings else None,
                intent=curr_intent,
                dt=0.02,
                base_freq_hz=freq_hz,
                result=result,
                h_agl_m=h_agl,
                frozen_angles=frozen_angles,
                rope_integrity_lost=rope_lost
            )
            self._symmetrical_rth_telem = rth_telem
            self._symmetrical_rth_active = rth_telem.get('rth_engaged', False)
            if self._symmetrical_rth_active:
                freq_hz = rth_telem.get('recommended_freq_hz', freq_hz)
                result['freq_hz'] = freq_hz
                self._h42_active = False  # Symmetrical quarantine cleanly supersedes asymmetric N-1 trim
                if rth_telem.get('recommended_pair_offsets_rad'):
                    self._pair_offsets = rth_telem['recommended_pair_offsets_rad']
                    result['pair_phase_offsets'] = self._pair_offsets

            is_braking = bool(result.get('rollout_suction_braking_active'))
            if not is_braking:
                # H-41: Active Heave Damping & Metachronal Pitch Reaction Moment Cancellation
                wc, self._pair_offsets, self._h41_active, self._h41_diag = ap.h41_active_ride_stabilization(
                    sensors or {}, wc, pair_offsets=self._pair_offsets
                )
                self._h41_vib_red_pct = self._h41_diag.get('vibration_reduction_pct', 0.0)
                self._h41_residual_my = self._h41_diag.get('residual_my_Nm', 0.0)
                self._h41_cabin_rms_g = self._h41_diag.get('cabin_rms_g', 0.0)
                self._h41_heave_err_g = self._h41_diag.get('heave_error_g', 0.0)
                self._h41_stall_margin_deg = self._h41_diag.get('stall_margin_deg', 13.0)
                self._h41_servo_derate = self._h41_diag.get('servo_bus_derate', 1.0)

                # H-43: Severe Wind Shear & Direct Collective Vectoring (DLTV)
                w_sink = sensors.get('w_sink_ms', sensors.get('VSI_ms', 0.0)) if sensors else 0.0
                climb_ovr = sensors.get('climb_override', False) if sensors else False
                v_ias = sensors.get('V_ms', sensors.get('airspeed_ms', 10.0)) if sensors else 10.0
                wc, self._h43_active, self._h43_delta_g = ap.h43_direct_lift_thrust_vectoring(
                    wc, w_sink, climb_ovr, V_ias_ms=v_ias
                )
                if self._h43_active:
                    freq_boost = getattr(ap.h43_direct_lift_thrust_vectoring, 'last_freq_boost_hz', 0.0)
                    self._h43_freq_boost = freq_boost
                    if freq_boost > 0.0:
                        freq_hz = min(2.6, freq_hz + freq_boost)
                        result['freq_hz'] = freq_hz
                else:
                    self._h43_freq_boost = 0.0
            else:
                self._h41_active = False
                self._h43_active = False
                self._h43_freq_boost = 0.0

            # H-44: Heavy Rain & Icing Boundary Layer LEV Re-Energization
            rain_or_ice = bool(sensors.get('rain_detected') or sensors.get('icing_detected')) if sensors else False
            wc, self._h44_active = ap.h44_rain_icing_lev_reenergization(wc, self._master_phase, rain_or_ice)

            # H-45: Proactive Bus Brownout & Priority Load Shedding
            bus_v = sensors.get('bus_voltage_v', 24.0) if sensors else 24.0
            tot_i = sum(sensors.get('motor_currents_a', [10.0]*8)) if sensors else 80.0
            wc, self._h45_active, self._h45_shed_pct = ap.h45_predictive_bus_load_shedding(wc, bus_v, tot_i)

            if not is_braking and not self._symmetrical_rth_active and curr_intent == 'cruise':
                # H-46: Micro-Stroke Resonant Buzz & Pitch Reflex (Dash Speed)
                lever = sensors.get('lever_pct', 50.0) if sensors else 50.0
                v_ms = sensors.get('V_ms', 9.0) if sensors else 9.0
                airfoil = sensors.get('airfoil_type', 'S1223') if sensors else 'S1223'
                wc, self._h46_active, self._h46_freq_hz = ap.h46_micro_stroke_resonant_buzz(lever, v_ms, wc, freq_hz, airfoil_type=airfoil)
                if self._h46_active:
                    freq_hz = self._h46_freq_hz
                    result['freq_hz'] = freq_hz

                # H-47: Tandem Interleaved Bound-and-Glide (Fast Cruise)
                wc, self._h47_active, self._h47_group = ap.h47_tandem_interleaved_bound_glide(lever, v_ms, wc, time.monotonic())

                # H-49: Cascade Tip-Vortex Katzmayr Energy Ingestion (Range Extension)
                wc, self._h49_active, self._h49_thrust_boost_N = ap.h49_katzmayr_vortex_energy_ingestion(wc, v_ms, self._master_phase, lever, freq_hz=freq_hz)
            else:
                self._h46_active = False
                self._h47_active = False
                self._h47_group = 'NONE'
                self._h49_active = False

            # Pelican Deep-Stroke Mode (H-DEEP-STROKE): High-thrust free-flight acceleration (1.9 Hz, ±30.0°, aft splay fan)
            if curr_intent == 'pelican_deep_stroke':
                freq_hz = 1.90
                result['freq_hz'] = 1.90
                splay_sweeps = [56.0, 56.0, 60.0, 60.0, 68.0, 68.0, 76.0, 76.0]
                splay_pitches = [12.0, 12.0, 10.0, 10.0, 10.0, 10.0, 8.0, 8.0]
                for i in range(min(8, len(wc))):
                    wc[i]['flap_amp_deg'] = 30.0
                    wc[i]['sweep_deg'] = splay_sweeps[i]
                    wc[i]['pitch_deg'] = splay_pitches[i]

            # Hard Wing-In-Ground (WIG) Floor Governor with Asymmetric Dihedral Bias (H-WIG-BIAS)
            try:
                p_stk = sensors.get('pitch_stick', 0.0) if sensors else 0.0
                thr = sensors.get('throttle', 0.5) if sensors else 0.5
                on_gnd = bool(sensors.get('on_ground', False)) if sensors else False
                wig_telem = govern_hard_wig_floor(
                    h_agl_m=h_agl, V_ms=v_ms, vz_ms=w_sink,
                    pitch_stick=p_stk, throttle=thr,
                    on_ground=on_gnd, intent=curr_intent,
                    dt_s=0.02, wing_commands=wc
                )
                result['wig_floor_telem'] = wig_telem
            except Exception:
                pass

            result['wing_commands'] = wc
        except Exception:
            pass

        # Always advance internal kinematic synthesizer (both sim and hardware)
        self._update_kinematics(freq_hz, wc, sensors=sensors, result=result)

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
                            wc[w_idx]['sweep_servo_deg'] = avg_sw
        except Exception:
            pass

        if self.sim_mode:
            return

        for i, cmd in enumerate(wc[:8]):
            if i < len(PITCH_PINS) and self.pi:
                pitch_deg = cmd.get('pitch_servo_deg', cmd.get('pitch_deg', 0.0))
                self.pi.set_servo_pulsewidth(
                    PITCH_PINS[i], w.pitch_deg_to_us(pitch_deg, servo_index=i))

            if i < len(SWEEP_PINS) and self.pi:
                sweep_deg = cmd.get('sweep_servo_deg', cmd.get('sweep_deg', 60.0))
                self.pi.set_servo_pulsewidth(
                    SWEEP_PINS[i], w.sweep_deg_to_us(sweep_deg, servo_index=i))

        # ODrive velocity commands via CAN
        if self.can_bus and freq_hz > 0:
            m_pos = sensors.get('motor_pos_rad') if sensors else None
            self._send_odrive_velocity(
                freq_hz, wc, motor_positions_rad=m_pos)

    def _update_kinematics(self, freq_hz, wing_commands, sensors: dict = None, result: dict = None):
        """Advance metachronal reciprocating phase and update 8-wing setpoints via MetachronalSynthesizer."""
        now = time.monotonic()
        if sensors and sensors.get('dt_s') is not None:
            dt = max(0.001, min(0.05, float(sensors['dt_s'])))
        else:
            dt = max(0.001, min(0.05, now - self._last_kin_t))
        self._last_kin_t = now

        # ── H-38 Ground Effect Fountain Mode Phase Adjustment ─────────
        if not self._symmetrical_rth_active and sensors and sensors.get('h_agl_m') is not None:
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
        elif not self._symmetrical_rth_active:
            self._h38_active = False
            self._h38_savings_pct = 0.0

        # ── H-41 Heave & Pitch Nod Cancellation (Ride Comfort) ────────
        comfort_mode = bool(sensors.get('comfort_mode', False)) if sensors else False
        imu_rms_g = sensors.get('imu_rms_g', 0.0) if sensors else 0.0
        try:
            if not self._h38_active and not self._symmetrical_rth_active:
                offsets, h41_phase_active, h41_vib = ap.h41_heave_pitch_cancellation(
                    comfort_mode, imu_rms_g, self._pair_offsets)
                if h41_phase_active:
                    self._pair_offsets = offsets
                    self._h41_active = True
                    self._h41_vib_red_pct = h41_vib
        except Exception:
            pass

        # Step the deterministic 50 Hz MetachronalSynthesizer
        kin_frame = self.kinematics.step(
            dt_s=dt,
            commands={
                'freq_hz': freq_hz,
                'wing_commands': wing_commands,
            },
            pair_offsets_rad=self._pair_offsets,
        )

        self._master_phase = kin_frame['master_phase_rad']
        self._last_flap_degs = [round(x, 2) for x in kin_frame['flap_angle_deg']]
        self._last_phases = list(kin_frame['flap_phases_norm'])
        self._last_dirs = list(kin_frame['stroke_dir'])
        self._last_pitch_servo_degs = [round(x, 2) for x in kin_frame['pitch_servo_deg']]
        self._last_sweep_servo_degs = [round(x, 2) for x in kin_frame['sweep_servo_deg']]
        self._last_motor_pos_turns = [round(x, 4) for x in kin_frame['motor_pos_turns']]
        self._last_motor_vel_rps = [round(x, 4) for x in kin_frame['motor_vel_rps']]

        for i, cmd in enumerate(wing_commands[:8]):
            if isinstance(cmd, dict):
                cmd['flap_angle_deg'] = self._last_flap_degs[i]
                cmd['stroke_dir'] = self._last_dirs[i]
                cmd['pitch_servo_deg'] = self._last_pitch_servo_degs[i]
                cmd['sweep_servo_deg'] = self._last_sweep_servo_degs[i]
                cmd['motor_pos_turns'] = self._last_motor_pos_turns[i]
                cmd['motor_vel_rps'] = self._last_motor_vel_rps[i]

        if isinstance(result, dict):
            result['kinematics_step'] = kin_frame

    def _send_odrive_velocity(self, freq_hz, wing_commands,
                               motor_positions_rad=None):
        """
        Send reciprocating velocity setpoints to ODrives via CAN.

        Uses the 50 Hz `MetachronalSynthesizer` velocity and position targets
        combined with soft-stop taper protection inside the capstan travel limits.
        """
        import struct
        from mco_wave_library import soft_stop_apply_to_commands

        vel_limit_overrides = soft_stop_apply_to_commands(
            wing_commands,
            motor_positions_rad or [None] * 8,
            base_vel_limit_rps=40.0
        )

        for i, cmd in enumerate(wing_commands[:8]):
            factor   = cmd.get('soft_stop_factor', 1.0)
            flap_dir = w.FLAP_MOTOR_DIRECTION[i] if i < len(w.FLAP_MOTOR_DIRECTION) else +1
            base_vel_rps = cmd.get(
                'motor_vel_rps',
                self._last_motor_vel_rps[i] if i < len(self._last_motor_vel_rps) else 0.0,
            )
            vel_turns_s = float(base_vel_rps) * float(factor) * float(flap_dir)

            arb_id = (i << 5) | 0x0D   # SET_INPUT_VEL
            data   = struct.pack('<ff', vel_turns_s, 0.0)
            try:
                msg = can.Message(arbitration_id=arb_id,
                                   data=data, is_extended_id=False)
                self.can_bus.send(msg)
            except Exception:
                pass

            vl = vel_limit_overrides[i]
            if vl is not None and self.odi:
                try:
                    self.odi.set_velocity_limit(i, vl)
                except Exception:
                    pass

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

    def __init__(self, port=MAVLINK_PORT, baud=MAVLINK_BAUD, sim=False):
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
        self.fc                = self
        self.intent_interpreter = IntentInterpreter()

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

        # Real-time 50 Hz control loop deadline monitor & watchdog state
        self._tick_latencies = []
        self._deadline_miss_count = 0
        self._watchdog_trip_count = 0
        self.cot_bridge = None

        # Graceful shutdown
        signal.signal(signal.SIGTERM, self._handle_signal)
        signal.signal(signal.SIGINT,  self._handle_signal)

    def _handle_signal(self, sig, frame):
        print(f"\nShutdown signal received ({sig})")
        self.alive = False

    def tick(self, sensors=None, tick_count=0, tick_start=None):
        """Execute a single 50 Hz control frame (flight dynamics + safety governors)."""
        if tick_start is None:
            tick_start = time.monotonic()
        if sensors is None:
            sensors = self.mavlink.get_sensors()
        else:
            sensors = dict(sensors)

        # ── Intent arbitration via IntentInterpreter ─────────
        sticks = {
            'roll':     sensors.get('roll', sensors.get('roll_stick', 0.0)),
            'pitch':    sensors.get('pitch', sensors.get('pitch_stick', 0.0)),
            'throttle': sensors.get('throttle', (sensors.get('lever_pct', 50.0) / 100.0) if 'lever_pct' in sensors else 0.5),
            'yaw':      sensors.get('yaw', sensors.get('yaw_stick', 0.0)),
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

        # ── Flight control tick ───────────────────────────────
        try:
            result = self.fcc.tick(sensors)
        except Exception as e:
            print(f"Tick error: {e}")
            result = {'wing_commands': [{}]*8, 'freq_hz': 0.0}

        # ── Active Flight Safety Governors ────────────────────
        # 1. Stall speed floor protection & pitot saturation/blockage safeguard
        raw_v = sensors.get('V_ms', 11.0)
        pitot_guard = check_pitot_saturation(
            V_pitot_ms=raw_v,
            q_dyn_pa=sensors.get('pitot_pressure_pa', None),
            ekf_v_ms=sensors.get('v_ekf_ms', None),
            gps_v_ms=sensors.get('gps_speed_ms', None)
        )
        throttle = sticks.get('throttle', 0.5)
        curr_intent = sensors.get('intent', 'cruise')
        alt_m = sensors.get('altitude_m', sensors.get('baro_alt_m', sensors.get('h_agl_m', 100.0)))
        pitot_block = check_pitot_blockage_and_synthetic_airspeed(
            V_pitot_ms=pitot_guard['effective_v_ms'],
            gps_v_ms=sensors.get('gps_speed_ms', sensors.get('v_ekf_ms')),
            throttle_frac=throttle,
            h_agl_m=sensors.get('h_agl_m', alt_m),
            on_ground=sensors.get('on_ground', False),
            result=result
        )
        V_ias = pitot_block['effective_v_ms']
        result['pitot_saturated'] = pitot_guard['is_saturated']
        result['pitot_status'] = pitot_guard['status']

        # Density calculation (ISA barometric + ambient temperature model)
        p_baro = sensors.get('baro_press_pa', sensors.get('pitot_static_pa', 101325.0))
        t_ambient_c = sensors.get('ambient_temp_c', sensors.get('imu_temp_c', 20.0))
        t_kelvin = max(220.0, min(335.0, t_ambient_c + 273.15))
        rho_air = p_baro / (287.05 * t_kelvin)
        sensors['air_density_kg_m3'] = rho_air
        result['air_density_kg_m3'] = rho_air

        glide_gov = check_glide_speed_floor(V_ias, throttle_frac=throttle, intent=curr_intent, rho_kg_m3=rho_air)
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

        # 2. Sprint dynamics clamp: protect ODrive S1 from velocity over-speed trips & thermal duty fence (Fix 4)
        is_sprint_active = (curr_intent == 'sprint' or result.get('freq_hz', 0.0) > 3.0)
        servo_curr = sum(sensors.get('servo_currents_a', [0.0]*16)) if sensors else 0.0
        cmd_f = result.get('freq_hz', 0.0)
        cmd_a = max((c.get('flap_amp_deg', 20.0) for c in result.get('wing_commands', []) if isinstance(c, dict)), default=20.0)
        m_temps = sensors.get('motor_temps', []) if sensors else []
        cf, ca, mrpm, margin = clamp_sprint_dynamics(
            cmd_f, cmd_a, dt_s=0.02, is_sprint=is_sprint_active, servo_current_a=servo_curr,
            motor_temps=m_temps, ambient_temp_c=t_ambient_c
        )
        # Apply universal kinematic hardware fence across ALL modes (max 1,800 RPM / f*amp <= 91.7 deg*Hz)
        cf, ca = clamp_universal_kinematics(cf, ca)
        result['freq_hz'] = cf
        for cmd in result.get('wing_commands', []):
            if isinstance(cmd, dict) and cmd.get('flap_amp_deg', 0.0) > ca:
                cmd['flap_amp_deg'] = ca
        result['sprint_rpm_margin_pct'] = margin
        result['sprint_thermal_status'] = get_sprint_thermal_status()

        # 3. Continuous Strouhal Matching in Cruise (St ~ 0.28) & Cascade Slipstream Phase Lock
        curr_amp = max((c.get('flap_amp_deg', 20.0) for c in result.get('wing_commands', []) if isinstance(c, dict)), default=20.0)
        vsi = sensors.get('vsi_ms', sensors.get('climb_rate_ms', 0.0))
        strouhal_gov = govern_strouhal_cruise(
            V_ias_ms=V_ias,
            flap_amp_deg=curr_amp,
            current_freq_hz=result.get('freq_hz', 2.0),
            intent=curr_intent,
            dt=0.02,
            climb_rate_ms=vsi,
            throttle_frac=throttle,
            rho_kg_m3=rho_air,
            result=result
        )
        if strouhal_gov.get('active'):
            result['freq_hz'] = strouhal_gov['optimal_freq_hz']

        slipstream_gov = govern_cascade_slipstream(
            V_ias_ms=V_ias,
            freq_hz=result.get('freq_hz', 2.0),
            result=result
        )
        if slipstream_gov.get('active'):
            # Dynamically set inter-tier phase offsets across 4 stations (Pairs 1 to 4)
            tier_phase = slipstream_gov['intertier_phase_deg']
            for idx, cmd in enumerate(result.get('wing_commands', [])):
                if isinstance(cmd, dict):
                    station = idx // 2
                    cmd['phase_offset_deg'] = (station * tier_phase) % 360.0

        # 4. Regenerative brake resistor voltage fence
        bus_v = sensors.get('bus_voltage_v', 24.0)
        req_regen_w = result.get('regen_watts', 0.0)
        resistor_gov = govern_regen_brake_resistor(bus_v, req_regen_w)
        result['brake_resistor_gov'] = resistor_gov

        # 4b. Cold LiPo OCV & Transient Sag Discrimination (H-COLD-LIPO-1)
        tot_bus_current_a = sum(sensors.get('motor_currents_a', [10.0]*8)) if sensors else 80.0
        batt_temp = sensors.get('batt_temp_c', sensors.get('ambient_temp_c', 20.0))
        lipo_ocv_gov = estimate_cold_lipo_ocv(
            bus_voltage_v=bus_v,
            bus_current_a=tot_bus_current_a,
            batt_temp_c=batt_temp,
            result=result
        )

        # 5. 2S 7.4V Servo Bus Brownout Governor
        servo_bus_v = sensors.get('servo_bus_voltage_v', sensors.get('bus_voltage_7v4', 7.4))
        servo_bus_gov = govern_servo_bus(
            bus_voltage_v=servo_bus_v,
            flap_amp_deg=curr_amp,
            result=result
        )

        # 6. Dive speed closure & recovery governor (H-PULL-2 / H-DIVE-1 / H-BOYD-1 / H-SWEEP-VNE-1 / H-SWEEP-PULL-1)
        current_sweep = 60.0
        wing_cmds = result.get('wing_commands', [])
        if wing_cmds and isinstance(wing_cmds, list) and isinstance(wing_cmds[0], dict) and 'sweep_deg' in wing_cmds[0]:
            current_sweep = sum(c.get('sweep_deg', 60.0) for c in wing_cmds[:8]) / min(8, len(wing_cmds))
        elif 'sweep_deg' in sensors:
            current_sweep = float(sensors['sweep_deg'])

        dive_gov = govern_dive_recovery(
            V_ms=V_ias,
            pitch_cmd_deg=sticks.get('pitch', 0.0),
            bank_deg=sensors.get('roll_deg_imu', sticks.get('roll', 0.0) * 30.0),
            altitude_m=alt_m,
            dt=0.02,
            intent=curr_intent,
            sweep_deg=current_sweep,
            result=result
        )
        result['dive_recovery_gov'] = dive_gov
        if dive_gov.get('inhibit_flag'):
            # Clamp / freeze commanded pitch stick to prevent servo stall tuck
            result['pitch_cmd_deg'] = dive_gov.get('effective_pitch_cmd_deg', 0.0)
            for cmd in result.get('wing_commands', []):
                if isinstance(cmd, dict) and cmd.get('pitch_deg', 0.0) > 0.0:
                    cmd['pitch_deg'] = 0.0

        # 7. Unified Multi-Sensor Weight-on-Wheels (WoW) Fusion
        wow_gov = govern_weight_on_wheels(
            h_agl_m=sensors.get('h_agl_m', alt_m),
            az_g=sensors.get('accel_z_g', 1.0),
            gear_contact=sensors.get('gear_contact', None),
            ground_speed_ms=sensors.get('gps_speed_ms', 0.0),
            dt=0.02,
            result=result
        )
        if 'on_ground' in sensors:
            on_ground_flag = bool(sensors['on_ground'])
        else:
            on_ground_flag = wow_gov['on_ground'] or (curr_intent in ('taxi_fwd', 'taxi_rev', 'park', 'takeoff_roll', 'preflight', 'snow_shake'))

        # 8. Ground Wind & Crosswind Safety Governor (H-TAXI-WIND-1 / H-TAXI-REEF-1 / H-TAXI-YAW-1 / H-TAXI-TAIL-1)
        if on_ground_flag:
            w_spd = sensors.get('V_wind_ms', sensors.get('wind_speed_ms', 0.0))
            w_brg = sensors.get('wind_bearing_deg', sensors.get('wind_dir_deg', 0.0))
            craft_hdg = sensors.get('heading_deg', sensors.get('yaw_deg', 0.0))
            rwy_hdg = sensors.get('runway_heading_deg', craft_hdg)
            ground_wind_gov = govern_ground_wind_safety(
                on_ground=True,
                V_wind_ms=w_spd,
                wind_bearing_deg=w_brg,
                heading_deg=craft_hdg,
                runway_heading_deg=rwy_hdg,
                pilot_throttle=throttle,
                pilot_yaw=sticks.get('yaw', 0.0),
                pilot_pitch=sticks.get('pitch', 0.0),
                intent=curr_intent,
                mass_kg=sensors.get('gross_weight_kg', 5.367),
                servo_currents_a=sensors.get('servo_currents_a'),
                pitot_v_ms=V_ias,
                result=result
            )
            result['ground_wind_gov'] = ground_wind_gov

            # Pre-Flight Snow / Frost Shake Governor (H-COLD-SHAKE-1)
            snow_req = (curr_intent == 'snow_shake') or sensors.get('snow_shake_requested', False)
            snow_gov = govern_preflight_snow_shake(
                dt_s=0.02,
                on_ground=True,
                throttle_frac=throttle,
                requested=snow_req,
                result=result
            )

            # Zero-Friction Ice & Snow Differential Sweep Steering (H-COLD-ICE-STEER-1)
            ice_surf = bool(sensors.get('ice_surface') or sensors.get('ice_mode') or (curr_intent == 'ice_taxi'))
            if ice_surf:
                govern_zero_friction_ice_steering(
                    wing_commands=result.get('wing_commands', []),
                    pilot_yaw_stick=sticks.get('yaw', 0.0),
                    on_ground=True,
                    ice_surface=True,
                    result=result
                )

        # 8b. Pilot-Centered Alpine Takeoff Assist & Unified Rollout/Abort Braking (H-ALPINE-TO-1)
        vz_val = sensors.get('vz_ms', sensors.get('Vz_ms', -sensors.get('sink_rate_ms', 0.0)))
        alpine_gov = govern_pilot_alpine_takeoff_and_rollout(
            dt_s=0.02,
            V_ias_ms=V_ias,
            p_baro_pa=p_baro,
            t_ambient_c=t_ambient_c,
            throttle_frac=throttle,
            pilot_pitch_stick=sticks.get('pitch', 0.0),
            on_ground=on_ground_flag,
            intent=curr_intent,
            wing_commands=result.get('wing_commands', []),
            result=result,
            vz_ms=vz_val
        )

        # 9. Fly-By-Wire Turn Coordination (Pilot toggleable; bypassed in flat turn mode)
        if 'turn_coordination' in sticks:
            set_turn_coordination(bool(sticks['turn_coordination']))
        bank_val = sensors.get('roll_deg_imu', sticks.get('roll', 0.0) * 30.0)
        pitch_val = sensors.get('pitch_deg_imu', sticks.get('pitch', 0.0) * 20.0)
        turn_coord_gov = govern_turn_coordination(
            bank_deg=bank_val,
            pitch_deg=pitch_val,
            V_ms=V_ias,
            on_ground=on_ground_flag,
            intent=curr_intent,
            pilot_yaw_stick=sticks.get('yaw', 0.0),
            coord_mode=result.get('coord_mode', 'auto'),
            result=result
        )
        if turn_coord_gov.get('coordination_active') and curr_intent != 'flat_turn':
            coord_yaw = turn_coord_gov['delta_yaw_cmd']
            result['pilot_yaw'] = result.get('pilot_yaw', sticks.get('yaw', 0.0)) + coord_yaw
            # Apply proverse yaw trim directly to wing commands via differential sweep
            delta_sweep_deg = coord_yaw * 6.0   # Up to +/- 6 deg differential sweep
            delta_pitch_turn = turn_coord_gov.get('delta_pitch_cmd', 0.0)
            pitch_taper = turn_coord_gov.get('pitch_taper_deg', [delta_pitch_turn]*4)
            turn_freq_mult = turn_coord_gov.get('turn_freq_mult', 1.0)
            if turn_freq_mult > 1.0 and not on_ground_flag:
                # Energy coordination: counteract n^2 induced drag surge in banked turns
                curr_f = result.get('freq_hz', 2.0)
                result['freq_hz'] = min(3.2, round(curr_f * turn_freq_mult, 2))

            for idx, cmd in enumerate(result.get('wing_commands', [])):
                if isinstance(cmd, dict):
                    is_port = (idx % 2 == 0)
                    cmd['sweep_deg'] = cmd.get('sweep_deg', 60.0) + (delta_sweep_deg if is_port else -delta_sweep_deg)
                    stn = idx // 2
                    d_p = pitch_taper[stn] if stn < len(pitch_taper) else delta_pitch_turn
                    if d_p > 0.0:
                        cmd['pitch_deg'] = min(12.5, cmd.get('pitch_deg', 6.0) + d_p)

        # 9b. Desert Thermal Interleaver (H-DESERT-COOL-1: 30s tier duty swap across pairs 1&3 vs 2&4)
        if not on_ground_flag and V_ias >= 10.0:
            motor_temps = sensors.get('motor_temps_c', [])
            fet_temps = sensors.get('fet_temps_c', [])
            desert_force = bool(sensors.get('desert_thermal_mode') or (curr_intent == 'desert_cruise'))
            govern_desert_thermal_interleaving(
                dt_s=0.02,
                motor_temps_c=motor_temps,
                fet_temps_c=fet_temps,
                wing_commands=result.get('wing_commands', []),
                V_ias_ms=V_ias,
                on_ground=False,
                forced=desert_force,
                result=result
            )

        # 9c. Extreme Gust & Convective Shear Decambering (H-GUST-SQUEEZE-1)
        az_val = sensors.get('accel_z_g', sensors.get('az_g', 1.0))
        govern_extreme_gust_squeeze(
            az_g=az_val,
            dt_s=0.02,
            wing_commands=result.get('wing_commands', []),
            on_ground=on_ground_flag,
            result=result
        )

        # 9d. Unsteady LEV Rain Penetration & Boundary Layer Boost (H-RAIN-LEV-1)
        rain_flag = bool(sensors.get('rain_detected', False) or sensors.get('rain_rate_mm_h', 0.0) >= 1.5)
        govern_unsteady_lev_rain(
            dt_s=0.02,
            rain_detected=rain_flag,
            rain_rate_mm_h=sensors.get('rain_rate_mm_h', 0.0),
            base_freq_hz=result.get('freq_hz', 2.2),
            wing_commands=result.get('wing_commands', []),
            result=result
        )

        # 9e. Energy-Throttled Orographic Mountain Wave Loiter (H-WAVE-LOITER-1)
        govern_orographic_wave_loiter(
            dt_s=0.02,
            V_ias_ms=V_ias,
            vz_ms=vz_val,
            pitch_deg=pitch_val,
            throttle_frac=throttle,
            pilot_pitch_stick=sticks.get('pitch', 0.0),
            on_ground=on_ground_flag,
            base_freq_hz=result.get('freq_hz', 2.2),
            wing_commands=result.get('wing_commands', []),
            result=result,
            w_atm_override=sensors.get('w_atm_ms')
        )

        # 10. Servo Dynamic Slew Rate & Acceleration Clamp (16 AGFRC IB53BHU servos)
        q_dyn = sensors.get('pitot_pressure_pa', 0.5 * 1.225 * (V_ias ** 2))
        wing_cmds = result.get('wing_commands', [])
        result['wing_commands'] = govern_servo_rates(
            wing_commands=wing_cmds,
            dt=0.02,
            q_dyn_pa=q_dyn,
            result=result
        )

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
        result['h41_stall_margin_deg'] = getattr(self.hardware, '_h41_stall_margin_deg', 13.0)
        result['h41_servo_derate'] = getattr(self.hardware, '_h41_servo_derate', 1.0)
        result['h42_active'] = getattr(self.hardware, '_h42_active', False)
        result['h42_diag'] = getattr(self.hardware, '_h42_diag', {})
        result['symmetrical_rth_active'] = getattr(self.hardware, '_symmetrical_rth_active', False)
        result['symmetrical_rth_telem'] = getattr(self.hardware, '_symmetrical_rth_telem', {})
        result['h43_active'] = getattr(self.hardware, '_h43_active', False)
        result['h43_delta_g'] = getattr(self.hardware, '_h43_delta_g', 0.0)
        result['h43_freq_boost_hz'] = getattr(self.hardware, '_h43_freq_boost', 0.0)
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

        return result

    def start(self, duration=None):
        """Start all subsystems and enter flight loop."""
        self.mavlink.start()
        self.logger.start()
        self.gs_server.start_background()
        if self.cot_bridge:
            self.cot_bridge.start()
        print(f"\nMCO-1D flight loop starting at {self.TICK_RATE_HZ} Hz")
        print(f"Sim mode: {self.sim_mode}")
        if duration is not None:
            print(f"Running for {duration:.1f}s...")
        print("Ready. Pre-flight check via ground station or panel.\n")

        tick_count  = 0
        _gc_counter = 0
        last_tick_t = time.monotonic()
        loop_start_t = time.monotonic()

        while self.alive:
            if duration is not None and (time.monotonic() - loop_start_t) >= duration:
                break
            tick_start = time.monotonic()

            # ── Inbound ATAK CoT Tasking check ────────────────────
            if self.cot_bridge:
                inbound_wp = self.cot_bridge.pop_latest_waypoint()
                if inbound_wp:
                    w.accept_external_fix(inbound_wp[0], inbound_wp[1], inbound_wp[2], source=inbound_wp[3])

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

                    # Handle pilot / GS turn coordination toggle
                    if cmd_str in ('coord_off', 'turn_coord_off') or cmd_dict.get('action') == 'disable_turn_coordination':
                        set_turn_coordination(False)
                        ack = {'type': 'ack', 'cmd': cmd_str, 'msg': 'Turn coordination disabled'}
                    elif cmd_str in ('coord_on', 'turn_coord_on') or cmd_dict.get('action') == 'enable_turn_coordination':
                        set_turn_coordination(True)
                        ack = {'type': 'ack', 'cmd': cmd_str, 'msg': 'Turn coordination enabled'}
                    else:
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

            # ── Get sensor data & tick flight control ─────────────
            sensors = self.mavlink.get_sensors()
            result = self.tick(sensors, tick_count=tick_count, tick_start=tick_start)


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
                    'dive_inhibit_flag': result.get('dive_inhibit_flag', False),
                    'dive_brake_active': result.get('dive_brake_active', False),
                    'dive_brake_fraction': result.get('dive_brake_fraction', 0.0),
                    'dive_recovery_status': result.get('dive_recovery_status', ''),
                    'dive_v_target_ms': result.get('dive_v_target_ms', 16.3),
                    'boyd_energy_saved_j': result.get('boyd_energy_saved_j', 0.0),
                    'dive_vne_ms':       result.get('dive_vne_ms', 28.0),
                    'dive_sweep_deg':    result.get('dive_sweep_deg', 60.0),
                    'commanded_sweep_deg': result.get('commanded_sweep_deg', 60.0),
                    'dive_unfold_status': result.get('dive_unfold_status', ''),
                    'pitot_saturated': result.get('pitot_saturated', False),
                    'pitot_status': result.get('pitot_status', 'PITOT_NOMINAL_UNSATURATED'),
                    'pitot_blocked': result.get('pitot_blocked', False),
                    'synthetic_airspeed_active': result.get('synthetic_airspeed_active', False),
                    'servo_bus_status': result.get('servo_bus_status', 'NOMINAL'),
                    'servo_bus_derate': result.get('servo_bus_derate_factor', 1.0),
                    'servo_bus_brownout_lock': result.get('servo_bus_brownout_lock', False),
                    'wow_on_ground': result.get('wow_on_ground', False),
                    'wow_state': result.get('wow_state', 'ON_GROUND'),
                    'wow_confidence': result.get('wow_confidence', 1.0),
                    'turn_coord_active': result.get('turn_coordination_active', False),
                    'turn_coord_enabled': is_turn_coordination_enabled(),
                    'turn_coord_yaw_rate': result.get('turn_coord_yaw_rate_deg_s', 0.0),
                    'strouhal_savings_pct': result.get('strouhal_savings_pct', 0.0),
                    'strouhal_opt_freq_hz': result.get('strouhal_opt_freq_hz', 0.0),
                    'cascade_slipstream_phase': result.get('cascade_slipstream_phase_deg', 90.0),
                    'ground_wind_state': result.get('ground_wind_state', 'STANDBY'),
                    'ground_suction_pitch_deg': result.get('ground_suction_pitch_deg', 0.0),
                    'ground_gsf_compensated': result.get('ground_gsf_compensated', 99.0),
                    'ground_rollover_mitigated': result.get('ground_rollover_mitigated', True),
                    'ground_takeoff_mode': result.get('ground_takeoff_mode', 'CONV-STOL-1'),
                    'deadline_misses': self._deadline_miss_count,
                    'watchdog_trips': self._watchdog_trip_count,
                }
                self.gs_server.set_telemetry(telem)
                self.mavlink.send_status(result)

            tick_count += 1

            # ── 50 Hz Control Loop Deadline Monitor & Watchdog ────
            elapsed = time.monotonic() - tick_start
            self._tick_latencies.append(elapsed)
            if len(self._tick_latencies) > 50:
                self._tick_latencies.pop(0)

            if elapsed > 0.018:   # 18 ms deadline warning threshold (>90% frame budget)
                self._deadline_miss_count += 1
                if self._deadline_miss_count % 10 == 1:
                    print(f"WARN: 50 Hz loop deadline overrun: {elapsed*1000.0:.2f} ms (>18 ms, total misses: {self._deadline_miss_count})")

            if elapsed > 0.050:   # 50 ms watchdog threshold (>2.5 frames missed)
                self._watchdog_trip_count += 1
                result['watchdog_safe_hold'] = True
                print(f"CRITICAL: 50 Hz loop watchdog tripped! Tick latency {elapsed*1000.0:.2f} ms > 50 ms. Engaging safe-hold.")

            # ── Maintain tick rate ────────────────────────────────
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

        self._shutdown()

    def _shutdown(self):
        """Clean shutdown sequence."""
        print("\nShutdown: running post-flight export...")
        # Save cross-flight state if supported
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
    parser = argparse.ArgumentParser(description='MCO-1D Flight Computer')
    parser.add_argument('--port', default=MAVLINK_PORT,
                        help='MAVLink serial port')
    parser.add_argument('--baud', type=int, default=MAVLINK_BAUD,
                        help='MAVLink baud rate')
    parser.add_argument('--sim',  action='store_true',
                        help='Simulation mode (no hardware output)')
    parser.add_argument('--wizard', action='store_true',
                        help='Launch MCO-1D first-run hardware configuration wizard')
    parser.add_argument('--duration', type=float, default=None,
                        help='Run flight loop for specified duration (seconds) then shutdown cleanly')
    args = parser.parse_args()

    if args.wizard:
        from mco_first_run_wizard import run as run_wizard
        run_wizard()
        return

    mco = MCOMain(port=args.port, baud=args.baud, sim=args.sim)
    mco.start(duration=args.duration)


if __name__ == '__main__':
    main()
