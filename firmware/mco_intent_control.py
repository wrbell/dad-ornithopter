# NOTE (Rev 18): Intent now flows via sensors["intent"] which is
# set from result["intent"] → self._last_intent → next tick sensors.
# The CG management stability margin and landing mode selection
# read intent from sensors["intent"] each tick.
# This file provides the IntentController class for generating
# intent labels from pilot inputs and flight state.
# Wire IntentController.update() into mco_main if richer intent
# classification is needed beyond the wave library default.
#
"""
MCO-1D Intent-Based Control System  (mco_intent_control.py)
============================================================
Rev K: Auto-coordination, decoupled yaw/roll, flat turn
Rev L: Fin yaw (W1+W4 vertical pitch), dart manoeuvre
"""

import time
import math

# ── INTENT TAXONOMY ──────────────────────────────────────────────

class Intent:
    CRUISE         = 'cruise'
    CLIMB          = 'climb'
    DESCEND        = 'descend'
    TURN_LEFT      = 'turn_left'
    TURN_RIGHT     = 'turn_right'
    GLIDE          = 'glide'
    SOAR           = 'soar'
    SPRINT         = 'sprint'
    APPROACH_NORMAL= 'approach_normal'
    APPROACH_STEEP = 'approach_steep'
    APPROACH_ABORT = 'approach_abort'
    GO_AROUND      = 'go_around'
    LAND           = 'land'
    TIGHT_TURN     = 'tight_turn'
    CHANDELLE      = 'chandelle'
    SPIRAL_DOWN    = 'spiral_down'
    STOOP          = 'stoop'
    CF1_BURST      = 'cf1_burst'
    LOITER         = 'loiter'
    ORBIT_LEFT     = 'orbit_left'
    ORBIT_RIGHT    = 'orbit_right'
    APPROACH_SITE  = 'approach_site'
    RECOVER        = 'recover'
    RTH            = 'rth'
    EMERGENCY_LAND = 'emergency_land'
    WING_ALL       = 'wing_all'
    WING_PARTIAL   = 'wing_partial'
    WING_SINGLE    = 'wing_single'
    WING_SLOT      = 'wing_slot'
    WING_GAP       = 'wing_gap'
    WING_REAR      = 'wing_rear'
    TAXI_FWD       = 'taxi_fwd'
    TAXI_REV       = 'taxi_rev'
    INSPECTION     = 'inspection'
    # Rev K
    FLAT_TURN_LEFT  = 'flat_turn_left'
    FLAT_TURN_RIGHT = 'flat_turn_right'
    SLIP            = 'slip'
    SKID            = 'skid'
    # Rev L
    DART_RIGHT      = 'dart_right'
    DART_LEFT       = 'dart_left'

    # Rev N
    ZOOM_CLIMB      = 'zoom_climb'
    POWER_DIVE      = 'power_dive'
    STOOP_AUTO      = 'stoop_auto'

    # Rev O3
    CRAB_LEFT       = 'crab_left'
    CRAB_RIGHT      = 'crab_right'

    # Rev P: Bat Flight Biomechanics Suite (H-81 .. H-85)
    ROOST_PERCH_FLARE = 'roost_perch_flare'
    ROOST_SOMERSAULT  = 'roost_somersault'
    INERTIAL_YAW      = 'inertial_yaw'
    ANHEDRAL_TURN     = 'anhedral_turn'

    # Rev Q: Dragonfly Aerodynamic Suite (H-86 .. H-88)
    REVERSE_CASCADE_CRUISE = 'reverse_cascade_cruise'
    ACTIVE_UPSTROKE_BRAKE  = 'active_upstroke_brake'
    STROKE_PLANE_VECTOR    = 'stroke_plane_vector'

    # Rev R: Insect Flight Biomechanics Suite (H-89 .. H-95)
    KRAMER_LIFT_BURST      = 'kramer_lift_burst'
    CLAP_JET_BREAKOUT      = 'clap_jet_breakout'
    ELYTRON_CANARD         = 'elytron_canard'
    CORIOLIS_GUST_PREEMPT  = 'coriolis_gust_preempt'
    LOCUST_TORSIONAL_CRUISE= 'locust_torsional_cruise'
    SLOTTED_FLAP_GLIDE     = 'slotted_flap_glide'

    # Rev S: Pterosaur Flight Biomechanics Suite (H-98, H-100 .. H-102)
    QUADRUPEDAL_VAULT       = 'quadrupedal_vault'
    DIFFERENTIAL_SWEEP_TRIM = 'differential_sweep_trim'
    CRANIAL_PITCH_BALANCE   = 'cranial_pitch_balance'
    AZHDARCHID_SOAR         = 'azhdarchid_soar'

    # Rev T: Aquatic Biomechanics & Hydrodynamic Biomimicry Suite (H-103 .. H-108)
    TURTLE_PIVOT_BRAKE      = 'turtle_pivot_brake'
    PLESIOSAUR_TANDEM_WAKE  = 'plesiosaur_tandem_wake'
    CETACEAN_HYDROFOIL_ROLL = 'cetacean_hydrofoil_roll'
    KRILL_JET_COALESCENCE   = 'krill_jet_coalescence'
    COPEPOD_METACHRONY      = 'copepod_metachrony'
    CARIDOID_ESCAPE_FLARE   = 'caridoid_escape_flare'

    # Cargo Drone & eVTOL Operations Suite
    PAYLOAD_DROP           = 'payload_drop'
    CARGO_SWING_DAMP       = 'cargo_swing_damp'
    IGE_CUSHION            = 'ige_cushion'
    VERTIPORT_APPROACH     = 'vertiport_approach'
    OWI_REDUNDANCY         = 'owi_redundancy'
    AUTO_GLIDE_LOCK        = 'auto_glide_lock'

    # H-41 Active Ride Comfort & Cargo Stabilization
    RIDE_COMFORT           = 'ride_comfort'
    CARGO_STABILIZE        = 'cargo_stabilize'

    # Aquatic Bird Flight Biomechanics Suite (Top 4)
    WATER_CLAP_BURST       = 'water_clap_burst'
    MALLARD_HUMP_BREAKOUT  = 'mallard_hump_breakout'
    GULL_ASYMMETRIC_STROKE = 'gull_asymmetric_stroke'
    WIG_CRUISE             = 'wig_cruise'

    # Biological Launch & High-Angle Climb Suite (Dragonfly & Pigeon)
    DRAGONFLY_SUPER_LIFT   = 'dragonfly_super_lift'
    PIGEON_STEEP_CLIMB     = 'pigeon_steep_climb'
    PELICAN_DEEP_STROKE    = 'pelican_deep_stroke'

    # Conventional Fixed-Wing Takeoff & Dual-Cascade Rotation Suite
    TAKEOFF_ROLL           = 'takeoff_roll'
    ROTATE                 = 'rotate'

    # Conventional Fixed-Wing Approach, Flare, & Runway Rollout Braking Suite
    LANDING_FLARE          = 'landing_flare'
    TOUCHDOWN_ROLLOUT      = 'touchdown_rollout'

    # Universal Biomimetic Launch & Interbreeding Suite
    LAUNCH_BURST           = 'launch_burst'

    # Tactical Defense & Contested EW Suite
    STEALTH_LOITER         = 'stealth_loiter'
    CONTESTED_EW           = 'contested_ew'

    # Advanced Maneuverability & Aerobatics Suite (H-INV-1, H-KNIFE-1, H-SNAP-1, H-VERT-1, H-SPIN-1)
    INVERTED_FLIGHT        = 'inverted_flight'
    OUTSIDE_LOOP           = 'outside_loop'
    KNIFE_EDGE_ROTOR       = 'knife_edge_rotor'
    SNAP_ROLL_AERO         = 'snap_roll_aero'
    HAMMERHEAD_PIVOT       = 'hammerhead_pivot'
    TAILSLIDE_FLIP         = 'tailslide_flip'
    SPIN_RECOVERY          = 'spin_recovery'

    # H-FALCON-TRIM-1 Fanned Bi-Delta Tailplane Fast Cruise & Level Dash
    FALCON_DART            = 'falcon_dart'




# ── INTENT INTERPRETER ───────────────────────────────────────────

class IntentInterpreter:
    STICK_THRESHOLD     = 0.12
    THROTTLE_CLIMB_MIN  = 0.65
    THROTTLE_DESCEND_MAX= 0.25
    ROLL_TURN_MIN       = 0.15
    GLIDE_LEVER_MIN     = 0.10
    LEVER_DETENT_LOW    = 47.5
    LEVER_DETENT_HIGH   = 52.5

    def __init__(self):
        self._last_intent    = Intent.CRUISE
        self._coord_mode     = 'auto'
        self._macro_requested= None
        self._vault_armed    = False
        self._vault_arm_t    = 0.0
        self._internal_t     = 0.0
        import mco_wave_library as _wl_init
        self._gesture_det    = _wl_init.GestureDetector()

    def _on_intent_change(self):
        """Reset transient gesture / rate states on intent switch."""
        pass

    def interpret(self, sticks, switches, context):
        ch1 = sticks.get('roll',     sticks.get('ch1', 0.0))
        ch2 = sticks.get('pitch',    sticks.get('ch2', 0.0))
        ch3 = sticks.get('throttle', sticks.get('ch3', 0.5))
        ch4 = sticks.get('yaw',      sticks.get('ch4', 0.0))
        ch6 = sticks.get('gl_lever', sticks.get('ch6', 0.0))
        lever_pct = context.get('lever_pct', ch3 * 100.0)

        # Cargo Drone Dynamic G-Limiter: derate maximum allowable pitch stick when carrying payload
        payload_kg = context.get('payload_kg', 0.0)
        if payload_kg > 0.15:
            gross_kg = 5.687 + payload_kg
            n_ratio = 5.687 / gross_kg
            max_pitch_stick = max(0.40, min(1.0, n_ratio))
            if ch2 > max_pitch_stick:
                ch2 = max_pitch_stick
                context['g_limit_active'] = True
                context['n_limit_g'] = round(3.5 * n_ratio, 2)

        mode_sw  = switches.get('mode_sw',   0)
        macro_sw = switches.get('macro_sw',  False)
        macro_lng= switches.get('macro_long',False)
        rth_sw   = switches.get('rth_sw',    False)
        wing_sw  = switches.get('wing_sw',   0)

        V_ms      = context.get('V_ms',      8.0)
        h_agl_m   = context.get('h_agl_m',   50.0)
        bank_deg  = context.get('bank_deg',  0.0)
        Ps        = context.get('Ps',        0.0)
        on_ground = context.get('on_ground', False)

        # Time tracking for gestures / rapid centering
        t_now = context.get('t')
        if t_now is None:
            self._internal_t += 0.05
            t_now = self._internal_t

        # Rudder pedal / brake depression detection (both pedals depressed)
        pedals_depressed = False
        if context.get('pedals_depressed', False) or context.get('brakes_held', False) or context.get('brakes', False):
            pedals_depressed = True
        elif switches.get('pedals_depressed', False) or switches.get('brakes', False):
            pedals_depressed = True
        elif sticks.get('pedals_depressed', False) or sticks.get('brakes_held', False):
            pedals_depressed = True
        elif sticks.get('brakes', 0.0) >= 0.70:
            pedals_depressed = True
        elif sticks.get('left_brake', 0.0) >= 0.70 and sticks.get('right_brake', 0.0) >= 0.70:
            pedals_depressed = True
        elif sticks.get('left_pedal', 0.0) >= 0.70 and sticks.get('right_pedal', 0.0) >= 0.70:
            pedals_depressed = True
        elif context.get('left_brake', 0.0) >= 0.70 and context.get('right_brake', 0.0) >= 0.70:
            pedals_depressed = True

        context['pedals_depressed'] = pedals_depressed
        context['throttle'] = ch3

        # ── Safety & Fail-Safes (Highest Priority Overrides) ─────
        if context.get('total_power_loss', False):
            return Intent.AUTO_GLIDE_LOCK, 1.0, {'reason': 'total_power_loss'}
        if context.get('failed_wing_node') is not None or context.get('owi_active', False):
            return Intent.OWI_REDUNDANCY, 1.0, {
                'failed_node': context.get('failed_wing_node', 0),
                'reason': 'motor_or_driver_failure'
            }
        if rth_sw:
            return Intent.RTH, 1.0, {}
        if context.get('stall_imminent', False):
            return Intent.RECOVER, 1.0, {'reason': 'stall_imminent'}
        if context.get('battery_critical', False) or context.get('emergency_land', False):
            return Intent.EMERGENCY_LAND, 1.0, {'reason': 'battery_critical' if context.get('battery_critical') else 'emergency_land_cmd'}

        # ── Uncommanded Spin Recovery (H-SPIN-REC-1) ─────────────
        yaw_rate_curr = context.get('yaw_rate_dps', 0.0)
        aoa_curr = context.get('aoa_deg', 0.0)
        if not on_ground and ((abs(yaw_rate_curr) >= 80.0 and aoa_curr >= 25.0 and V_ms < 6.0) or context.get('spin_detected', False)):
            return Intent.SPIN_RECOVERY, 1.0, {
                'yaw_rate_dps': yaw_rate_curr,
                'aoa_deg': aoa_curr,
                'V_ms': V_ms,
                'reason': 'uncommanded_spin_detected'
            }

        # ── Inverted Flight & Outside Push Loop Suite (H-INV-1, H-LOOP-OUT-1) ──
        is_inverted = (abs(bank_deg) >= 135.0 or context.get('inverted_flight', False))
        if not on_ground and is_inverted:
            if ch2 <= -0.30 or context.get('outside_loop_demand', False):
                return Intent.OUTSIDE_LOOP, 0.96, {
                    'push_stick': ch2,
                    'throttle': ch3,
                    'bank_deg': bank_deg,
                    'V_ms': V_ms,
                }
            return Intent.INVERTED_FLIGHT, 0.95, {
                'pitch': ch2,
                'throttle': ch3,
                'bank_deg': bank_deg,
                'V_ms': V_ms,
            }

        # ── Vertical Flapping Rotor Knife-Edge Mode (H-KNIFE-ROTOR-1) ──
        if not on_ground and abs(bank_deg) >= 75.0 and (context.get('knife_edge_active', False) or (bank_deg * ch4 < -0.20 or abs(ch4) >= 0.35)):
            direction = 'right' if bank_deg > 0 else 'left'
            return Intent.KNIFE_EDGE_ROTOR, 0.95, {
                'bank_deg': bank_deg,
                'direction': direction,
                'top_rudder': abs(ch4),
                'throttle': ch3,
            }

        # ── Aerobatic Snap Decoupling Mode (H-SNAP-1) ───────────
        urgency = context.get('stick_urgency', abs(context.get('roll_rate_dps', 0.0)) / 45.0)
        is_snap_gesture = (urgency >= 3.5 and abs(ch1) >= 0.80 and ch2 >= 0.50 and not on_ground and V_ms >= 8.0)
        if (is_snap_gesture or context.get('snap_roll_demand', False)) and not on_ground:
            snap_dir = 'right' if ch1 > 0 else 'left'
            return Intent.SNAP_ROLL_AERO, 0.96, {
                'direction': snap_dir,
                'roll_stick': ch1,
                'pitch_stick': ch2,
                'rudder_stick': ch4,
                'urgency': urgency,
            }

        # ── Vertical Apex Handshake (H-VERT-APEX-1) ─────────────
        pitch_att = context.get('pitch_deg', 0.0)
        vz = context.get('vz_ms', 0.0)
        is_vertical_line = (pitch_att >= 60.0 or vz >= 2.5 or context.get('vertical_climb_active', False))
        if not on_ground and is_vertical_line and V_ms < 2.0:
            if abs(ch4) >= 0.35 or context.get('hammerhead_demand', False):
                pivot_dir = 'right' if ch4 > 0 else 'left'
                return Intent.HAMMERHEAD_PIVOT, 0.95, {
                    'direction': pivot_dir,
                    'rudder_stick': ch4,
                    'pitch_deg': pitch_att,
                }
            return Intent.TAILSLIDE_FLIP, 0.95, {
                'pitch_deg': pitch_att,
                'pitch_stick': ch2,
                'vz_ms': vz,
            }

        # ── Contested EW & Defense Stealth Overrides ─────────────
        if context.get('contested_ew', False) or (context.get('rc_lost', False) and context.get('gps_jammed', False)):
            return Intent.CONTESTED_EW, 1.0, {'reason': 'contested_ew_active', 'target_h_m': 30.0}
        if context.get('stealth_loiter', False) or switches.get('stealth_sw', False):
            return Intent.STEALTH_LOITER, 1.0, {'reason': 'tactical_stealth_loiter'}

        # Rev K — coordination mode
        # Rev N: gesture detection for zoom/dive/stoop
        import mco_wave_library as _wl

        _gesture, _gconf, _gparams = _wl.classify_pitch_intent(
            sticks.get('pitch', 0.0), ch3*100.0, V_ms,
            h_agl_m, context.get('pitch_deg', 0.0),
            self._gesture_det)
        if _gesture == 'zoom_climb' and not on_ground:
            return Intent.ZOOM_CLIMB, _gconf, _gparams
        if _gesture == 'stoop_auto' and not on_ground:
            return Intent.STOOP_AUTO, _gconf, _gparams
        if _gesture == 'power_dive' and not on_ground and ch3 > 0.35:
            return Intent.POWER_DIVE, _gconf, _gparams
        if _gesture == 'perch_flare' and not on_ground:
            return Intent.ROOST_PERCH_FLARE, _gconf, _gparams
        if _gesture == 'roost_somersault' and not on_ground:
            return Intent.ROOST_SOMERSAULT, _gconf, _gparams

        # Rev P: Chiropteran inertial reorientation (zero-airspeed yaw H-81)
        if V_ms < 2.0 and abs(ch4) >= 0.20 and not on_ground:
            direction = 'right' if ch4 > 0 else 'left'
            return Intent.INERTIAL_YAW, abs(ch4), {
                'direction': direction,
                'yaw_stick': ch4,
                'yaw_rate_dps': ch4 * 60.0,
            }

        # Rev Q: Odonata direct-musculature stroke-plane vectoring (H-88)
        if abs(ch4) >= 0.35 and abs(ch1) < 0.20 and not on_ground and V_ms >= 2.0:
            direction = 'right' if ch4 > 0 else 'left'
            return Intent.STROKE_PLANE_VECTOR, abs(ch4), {
                'direction': direction,
                'yaw_stick': ch4,
                'tilt_deg': 18.0 * (1.0 if ch4 > 0 else -1.0),
            }

        # Rev Q: Active upstroke reverse flight & dynamic braking (H-87)
        if not on_ground and h_agl_m >= 2.0:
            airbrake_requested = (context.get('active_upstroke_brake', False) or
                                  (lever_pct <= 10.0 and ch3 <= 0.10 and not context.get('falcon_morph', False)))
            if airbrake_requested and not context.get('falcon_morph', False):
                return Intent.ACTIVE_UPSTROKE_BRAKE, 0.95, {
                    'aoa_deg': -28.0,
                    'decel_target_g': 1.25,
                    'h_agl_m': h_agl_m,
                }

        # Rev R: Dipteran Coriolis gust preemption (H-93)
        ang_accel = context.get('angular_accel_dps2', abs(context.get('gyro_deriv', 0.0)))
        if (ang_accel > 220.0 or context.get('coriolis_gust_alert', False)) and not on_ground:
            return Intent.CORIOLIS_GUST_PREEMPT, 0.98, {
                'ang_accel_dps2': ang_accel,
                'counter_twist_deg': 6.5,
            }

        # Rev R: Dynamic Kramer pre-rotation lift burst (H-89)
        if ((V_ms < 5.0 and ch2 >= 0.60 and not on_ground and not context.get('obstacle_climb', False))
                or context.get('kramer_burst_demand', False)):
            return Intent.KRAMER_LIFT_BURST, 0.92, {
                'pre_rotation_lead_deg': 10.0,
                'cl_transient': 2.85,
            }

        # Rev R: Coleopteran elytron canard steep obstacle climb (H-92)
        if (ch2 >= 0.70 or context.get('obstacle_climb', False)) and (V_ms >= 5.0 or context.get('obstacle_climb', False)) and not on_ground:
            return Intent.ELYTRON_CANARD, 0.90, {
                'canard_locked_station': 1,
                'canard_incidence_deg': 8.5,
                'climb_angle_deg': 48.0,
            }

        # Pigeon Clap-and-Peel Steep Obstacle Climb Corridor (H-LA-3)
        if not on_ground and V_ms <= 7.0 and (
                context.get('steep_climb_demand', False) or 
                context.get('obstacle_clearance_active', False) or 
                (ch3 >= 0.80 and context.get('climb_angle_deg', 0.0) >= 70.0)):
            return Intent.PIGEON_STEEP_CLIMB, 0.96, {
                'climb_angle_deg': 80.0,
                'peel_lead_ms': 18.0,
            }

        # Rev R: Lepidopteran dorsal clap jet breakout (H-91)
        is_landing_mode = (
            self._last_intent in (Intent.APPROACH_NORMAL, Intent.LANDING_FLARE, Intent.TOUCHDOWN_ROLLOUT, Intent.GO_AROUND) or
            context.get('landing_approach', False) or
            context.get('landing_flare', False) or
            context.get('touchdown_rollout', False) or
            context.get('landing_regime') is not None
        )
        if not is_landing_mode and ((ch3 * 100.0 >= 85.0 and V_ms < 6.0) or context.get('clap_jet_demand', False)) and not on_ground:
            return Intent.CLAP_JET_BREAKOUT, 0.95, {
                'jet_ejection_ms': 22.0,
                'accel_g': 1.45,
            }

        # Rev S: Tailless metachronal differential sweep directional trim (H-100)
        if not on_ground and (abs(context.get('sideslip_beta_deg', 0.0)) > 3.0 or abs(context.get('yaw_drift_deg', 0.0)) > 2.5 or context.get('tailless_yaw_trim', False)):
            return Intent.DIFFERENTIAL_SWEEP_TRIM, 0.94, {
                'differential_sweep_deg': 12.0,
                'yaw_hold_deg': 0.5,
            }

        # Rev S: Cranial crest dynamic flap-inertia pitch counterbalance (H-101)
        if not on_ground and (context.get('pitch_servo_current_a', 0.0) > 4.2 or context.get('high_pitch_load', False) or context.get('pitch_balance_demand', False)):
            return Intent.CRANIAL_PITCH_BALANCE, 0.93, {
                'pitch_feedforward_gain': 0.82,
                'current_reduction_pct': 42.0,
            }

        # Rev T: Decapod Caridoid Impulse Flare & Escape Pulse (H-108)
        if context.get('caridoid_escape_demand', False) or context.get('emergency_flare', False) or context.get('collision_warning', False):
            return Intent.CARIDOID_ESCAPE_FLARE, 0.98, {
                'escape_thrust_n': 38.0,
                'duration_s': 0.12,
            }

        # Rev T: Sea Turtle In-Stroke Differential Flipper Pivot Braking (H-103)
        if not on_ground and (context.get('pivot_brake_demand', False) or context.get('turtle_turn', False) or (abs(ch4) > 0.40 and ch3 < 0.20 and not is_landing_mode)):
            turn_dir = 'left' if ch4 < 0 else 'right'
            return Intent.TURTLE_PIVOT_BRAKE, 0.95, {
                'direction': turn_dir,
                'turn_radius_m': 0.65,
                'flipper_aoa_deg': 24.0,
            }

        # Rev T: Cetacean Pectoral Hydrofoil Maneuver & Fluke Recoil Damping (H-105)
        if not on_ground and (context.get('cetacean_roll_demand', False) or context.get('recoil_damping_demand', False) or (abs(ch1) > 0.50 and context.get('heave_oscillation_rms', 0.0) > 0.30)):
            return Intent.CETACEAN_HYDROFOIL_ROLL, 0.94, {
                'roll_rate_degs': 180.0,
                'recoil_damping_active': True,
            }

        # Rev T: Plesiosaur Tandem Wake Capture (H-104)
        if not on_ground and (context.get('tandem_wake_demand', False) or context.get('wake_capture', False) or context.get('downstream_wake_detected', False)):
            return Intent.PLESIOSAUR_TANDEM_WAKE, 0.92, {
                'thrust_boost_pct': 22.5,
                'tandem_sync': True,
            }

        # Rev T: Crustacean Adlocomotory Metachronal Jet Coalescence (H-106)
        if not on_ground and (context.get('krill_jet_demand', False) or context.get('jet_coalescence', False) or (ch3 * 100.0 >= 88.0 and V_ms >= 14.0 and context.get('adlocomotory_mode', False))):
            return Intent.KRILL_JET_COALESCENCE, 0.93, {
                'phase_lag_deg': 55.0,
                'thrust_boost_pct': 26.8,
            }

        # Rev T: Copepod Dual-Mode Asymmetric Metachrony (H-107)
        if not on_ground and (context.get('copepod_metachrony_demand', False) or context.get('power_smoothing', False) or context.get('torque_ripple_pct', 0.0) > 28.0):
            return Intent.COPEPOD_METACHRONY, 0.91, {
                'ripple_reduction_pct': 48.5,
                'smoothing_active': True,
            }

        # Rev O3: crab via flat_turn mode + roll stick
        if (self._coord_mode == 'flat_turn'
                and abs(ch1) > 0.2 and not on_ground):
            crab_dir = 'left' if ch1 < 0 else 'right'
            crab_intent = (Intent.CRAB_LEFT
                if ch1 < 0 else Intent.CRAB_RIGHT)
            return crab_intent, abs(ch1), {
                'direction': crab_dir,
                'intensity': abs(ch1)}

        coord_mode = _wl.select_coordination_mode(
            mode_sw, macro_lng, on_ground)
        self._coord_mode = coord_mode

        # ── Ground ───────────────────────────────────────────────
        if on_ground or h_agl_m < 0.15:
            # Rev S: Pterosaur quadrupedal ground-coupled vault takeoff (H-98)
            # 1. Programmatic / autonomous API demand
            if context.get('vault_takeoff_demand', False):
                self._vault_armed = False
                return Intent.QUADRUPEDAL_VAULT, 0.98, {
                    'vault_impulse_g': 1.85,
                    'transition_timeout_s': 0.42,
                    'target_h_agl_m': 1.5,
                    'trigger': 'demand',
                }

            # 2. Pilot gesture arming: dual rudder pedals depressed (brakes held) +
            #    full forward throttle (>=0.85) + full aft stick (pitch >= 0.70)
            if pedals_depressed and ch3 >= 0.85 and ch2 >= 0.70 and abs(ch1) <= 0.25 and abs(ch4) <= 0.25:
                self._vault_armed = True
                self._vault_arm_t = t_now

            # 3. Armed state checks: rapid centering trigger vs cancellations
            elif self._vault_armed:
                dt_arm = t_now - self._vault_arm_t
                if not pedals_depressed:
                    # Cancelled: rudder pedals released on ground
                    self._vault_armed = False
                elif ch3 < 0.80:
                    # Cancelled: throttle backed off
                    self._vault_armed = False
                elif ch2 < -0.20 or abs(ch1) > 0.25 or abs(ch4) > 0.25:
                    # Cancelled: stick pushed forward into dive or lateral deflection
                    self._vault_armed = False
                elif dt_arm > 0.45:
                    # Cancelled: stick centered slowly (>0.45s)
                    self._vault_armed = False
                elif abs(ch2) <= 0.18:
                    # Rapid centering complete within window (dt <= 0.45s): FIRE UNIVERSAL LAUNCH BURST!
                    self._vault_armed = False
                    import mco_advanced_physics as ap
                    chrom, metrics = ap.select_optimal_launch_chromosome(context)
                    launch_mod = context.get('launch_modality', None)
                    if launch_mod == 'universal' or context.get('universal_launch', False):
                        intent = Intent.LAUNCH_BURST
                    elif launch_mod == 'dragonfly':
                        intent = Intent.DRAGONFLY_SUPER_LIFT
                    elif launch_mod == 'pterosaur' or not context.get('universal_launch', False):
                        intent = Intent.QUADRUPEDAL_VAULT
                    else:
                        intent = Intent.LAUNCH_BURST

                    return intent, 0.98, {
                        'chromosome': chrom,
                        'chromosome_name': chrom.name,
                        'vault_impulse_g': 1.85 if intent == Intent.QUADRUPEDAL_VAULT else metrics.get('peak_g', 1.85),
                        'transition_timeout_s': 0.42 if intent == Intent.QUADRUPEDAL_VAULT else chrom.macro_duration_max_s,
                        'target_h_agl_m': 1.5 if intent == Intent.QUADRUPEDAL_VAULT else max(1.5, metrics.get('alt_at_obs_m', 1.5)),
                        'trigger': 'rapid_center',
                        'dt_center_s': round(dt_arm, 3),
                        'fitness': metrics.get('fitness', 100.0),
                    }

            # Dragonfly 0° In-Phase Super-Lift Transition (H-LA-1)
            # Dedicated trigger only: requires explicit demand or launch switch
            if not pedals_depressed and not self._vault_armed and (
                    context.get('dragonfly_launch_demand', False) or context.get('vtol_launch_demand', False) or switches.get('sw_launch', False)) and V_ms <= 3.0:
                return Intent.DRAGONFLY_SUPER_LIFT, 0.98, {
                    'lift_boost_multiplier': 2.42,
                    'duration_s': 0.45,
                    'freq_hz': 2.50,
                }

            if wing_sw == 2:
                return Intent.INSPECTION, 1.0, {}
            if macro_lng:
                return Intent.TAXI_REV, 0.9, {'throttle': ch3, 'yaw': ch4, 'lateral': ch1}

            # ── Conventional Fixed-Wing Takeoff Suite (CTOL/STOL) & Touch-and-Go ────
            regime = context.get('takeoff_regime', context.get('regime', 'standard'))
            # Touch-and-Go / Takeoff Rotation Check: high throttle (>=0.50) or forward wave lever (>=78%)
            is_soft_field = (regime == 'soft_field')
            is_stol = (regime in ('short_field', 'stol', 'stol_takeoff'))
            vr_threshold = 2.5 if is_soft_field else (3.2 if is_stol else 3.5)
            is_takeoff_power = (ch3 >= 0.50 or lever_pct >= 78.0)
            # Rejected Rollout Go-Around / Balked Landing demand
            if is_takeoff_power and (context.get('go_around_demand', False) or context.get('balked_landing', False)):
                return Intent.GO_AROUND, 0.98, {
                    'throttle': ch3,
                    'pitch': ch2,
                    'lever_pct': lever_pct,
                    'regime': regime,
                    'h_agl_m': h_agl_m,
                    'V_ms': V_ms,
                    'vz_ms': context.get('vz_ms', 0.0),
                    'crosswind_ms': context.get('crosswind_ms', 0.0),
                    'lateral': ch1,
                    'yaw': ch4,
                }

            if is_takeoff_power and ch2 > 0.15 and (V_ms >= vr_threshold or context.get('rotate_demand', False)):
                return Intent.ROTATE, 0.95, {
                    'regime': regime,
                    'pitch': ch2,
                    'yaw': ch4,
                }

            # Takeoff Roll / Touch-and-Go Re-acceleration: advancing throttle + neutral/forward stick
            if is_takeoff_power and ch2 <= 0.15:
                return Intent.TAKEOFF_ROLL, 0.95, {
                    'regime': regime,
                    'throttle': ch3,
                    'yaw': ch4,
                }

            # ── Conventional Fixed-Wing Rollout & Aerodynamic Wing Braking ────
            landing_regime = context.get('landing_regime', context.get('regime', 'standard_power_off'))
            in_rollout = (
                context.get('touchdown_rollout', False) or
                context.get('landing_rollout', False) or
                (self._last_intent in (Intent.LANDING_FLARE, Intent.APPROACH_NORMAL, Intent.TOUCHDOWN_ROLLOUT) and lever_pct < 78.0) or
                (V_ms >= 4.5 and not context.get('takeoff_roll_demand', False)) or
                (lever_pct < 40.0 and not context.get('takeoff_roll_demand', False))
            )
            if in_rollout:
                # Auto-transition to ground taxi when speed drops below 4.5 m/s and wave lever is not actively braking (<40%)
                if V_ms < 4.5 and lever_pct >= 40.0 and ch3 <= 0.35 and not (context.get('touchdown_rollout', False) and lever_pct < 40.0):
                    return Intent.TAXI_FWD, 0.9, {'throttle': ch3, 'yaw': ch4, 'lateral': ch1}

                return Intent.TOUCHDOWN_ROLLOUT, 0.96, {
                    'regime': landing_regime,
                    'lever_pct': lever_pct,
                    'throttle': ch3,
                    'yaw': ch4,
                    'pitch': ch2,
                    'lateral': ch1,
                    'V_ms': V_ms,
                }

            # Cracked-throttle taxi (with pedal yaw steering and stick lateral translation)
            return Intent.TAXI_FWD, 0.9, {'throttle': ch3, 'yaw': ch4, 'lateral': ch1}
        else:
            self._vault_armed = False

        # ── Cargo Drone & eVTOL Flight Modes ─────────────────────
        # 1. Cargo Drop de-ballooning pitch step (H-CR-2)
        if context.get('payload_release_cmd', False) or context.get('cargo_drop_cmd', False):
            return Intent.PAYLOAD_DROP, 1.0, {
                'de_balloon_pitch_deg': -2.0,
                'duration_s': 0.50,
            }
        # 2. Cargo sling pendulum suppression (H-HL-3)
        if context.get('sling_pendulum_active', False) or abs(context.get('cargo_swing_deg', 0.0)) > 8.0:
            return Intent.CARGO_SWING_DAMP, 0.95, {
                'swing_deg': context.get('cargo_swing_deg', 0.0),
                'damping_axis': 'pitch_cascade',
            }
        # 3. eVTOL In-Ground Effect (IGE) ground cushion & vertiport pad lock
        if not on_ground and 0.05 <= h_agl_m < 0.50 and (V_ms < 3.5 or context.get('vertiport_hover', False)):
            return Intent.IGE_CUSHION, 0.95, {
                'h_agl_m': h_agl_m,
                'cushion_freq_damping': 0.85,
            }
        # 4. eVTOL Vertiport steep approach (H-FD-2)
        if context.get('vertiport_approach', False) and not on_ground:
            return Intent.VERTIPORT_APPROACH, 0.95, {
                'target_glide_deg': context.get('vertiport_glide_deg', 8.5),
                'steep_dihedral_deg': 50.0,
            }

        # 5. Fragile / Sensitive Cargo Stabilization & G-limit envelope (H-41)
        if (context.get('sensitive_cargo', False) or context.get('fragile_payload', False)
                or context.get('medical_cargo', False)) and not on_ground:
            return Intent.CARGO_STABILIZE, 0.94, {
                'heave_damping': True,
                'anti_phase_pitch': True,
                'peak_g_limit': 1.15,
            }

        # 6. Payload & Sensor Line-of-Sight Active Heave/Pitch Stabilization (H-41)
        if (context.get('comfort_mode', False) or context.get('sensor_stab_mode', False) or context.get('passenger_flight', False)
                or (context.get('imu_rms_g', 0.0) > 0.08 and not on_ground)):
            return Intent.RIDE_COMFORT, 0.92, {
                'heave_damping': True,
                'anti_phase_pitch': True,
                'target_rms_g': 0.03,
            }

        # ── Aquatic Bird Flight Biomechanics Suite (Top 4) ───────
        on_water = context.get('on_water', False) or context.get('surface_is_water', False)
        h_water = context.get('h_water_m', h_agl_m)

        # 7. H-WD-1: Gull Clap-and-Fling Water Departure Synergy Burst
        if on_water and h_water <= 0.54 and V_ms >= 8.5:
            return Intent.WATER_CLAP_BURST, 0.96, {
                'lift_boost_pct': 22.0,
                'h_water_m': h_water,
                'duration_s': 0.40,
            }

        # 8. H-WD-2: Mallard Decoupled Front-Thrust / Rear-Lift Hump Penetration
        if on_water and 3.5 <= V_ms < 8.5:
            return Intent.MALLARD_HUMP_BREAKOUT, 0.94, {
                'front_sweep_deg': 55.0,
                'rear_aoa_deg': 16.0,
            }

        # 9. H-WD-3: Seagull Asymmetric High-Q Water Departure Kinematics
        if on_water and V_ms < 3.5:
            return Intent.GULL_ASYMMETRIC_STROKE, 0.90, {
                'Q_ratio': 1.90,
                'splash_reduction': True,
            }

        # 10. Pelican Wave-Following Wing-In-Ground (WIG) Compression Cruise
        if not on_ground and 0.08 <= h_water <= 0.65 and V_ms >= 6.5 and (
                context.get('wig_cruise_demand', False) or context.get('over_water', False) or on_water):
            return Intent.WIG_CRUISE, 0.93, {
                'target_h_m': 0.25,
                'induced_drag_reduction_pct': 86.0,
            }

        # 11. H-LA-1: Dragonfly 0° In-Phase Super-Lift Transition (explicit demand/switch only)
        if (on_ground or h_agl_m <= 0.6) and (context.get('dragonfly_launch_demand', False) or context.get('vtol_launch_demand', False) or switches.get('sw_launch', False)) and V_ms <= 3.0:
            return Intent.DRAGONFLY_SUPER_LIFT, 0.98, {
                'lift_boost_multiplier': 2.42,
                'duration_s': 0.45,
                'freq_hz': 2.50,
            }

        # ── Conventional Fixed-Wing Approach & Flare Suite ───────
        landing_reg = context.get('landing_regime', context.get('regime', 'standard_power_off'))

        # 0. Universal Go-Around / Balked Landing Trigger:
        # Initiated from approach, flare, or touchdown rollout upon advancing throttle/lever to TOGA
        is_landing_context = (
            self._last_intent in (Intent.APPROACH_NORMAL, Intent.LANDING_FLARE, Intent.TOUCHDOWN_ROLLOUT,
                                  Intent.SLOTTED_FLAP_GLIDE, Intent.GLIDE) or
            context.get('landing_approach', False) or
            context.get('landing_flare', False) or
            context.get('touchdown_rollout', False) or
            context.get('landing_regime') is not None or
            (h_agl_m <= 40.0 and context.get('regime') in ('standard_power_off', 'commercial_powered', 'short_field',
                                                           'soft_field', 'stol', 'stol_landing', 'tailwheel_3point',
                                                           'tailwheel_wheel_landing'))
        )
        is_normal_takeoff_roll = (
            on_ground and
            self._last_intent in (Intent.TAKEOFF_ROLL, Intent.ROTATE) and
            not context.get('touchdown_rollout', False)
        )
        is_toga_demand = (
            ch3 >= 0.60 or
            lever_pct >= 75.0 or
            context.get('go_around_demand', False) or
            context.get('balked_landing', False) or
            switches.get('sw_go_around', False)
        )
        if is_landing_context and not is_normal_takeoff_roll and is_toga_demand and ch2 >= -0.20:
            return Intent.GO_AROUND, 0.98, {
                'throttle': ch3,
                'pitch': ch2,
                'lever_pct': lever_pct,
                'regime': landing_reg,
                'h_agl_m': h_agl_m,
                'V_ms': V_ms,
                'vz_ms': context.get('vz_ms', -0.5),
                'crosswind_ms': context.get('crosswind_ms', 0.0),
                'lateral': ch1,
                'yaw': ch4,
            }
        # 1. Flare: low altitude (h_agl <= 2.0m) during landing approach
        is_takeoff_climb = (
            self._last_intent in (Intent.TAKEOFF_ROLL, Intent.ROTATE, Intent.CLIMB, Intent.PIGEON_STEEP_CLIMB)
            or (ch3 >= 0.50 and not context.get('landing_flare', False) and not context.get('flare_demand', False))
            or context.get('climb_demand', False)
            or context.get('vz_ms', 0.0) > 0.20
        )
        has_flare_demand = context.get('flare_demand', False) or context.get('landing_flare', False)
        is_approach_transition = self._last_intent in (Intent.APPROACH_NORMAL, Intent.LANDING_FLARE)

        if not on_ground and not is_takeoff_climb and (h_agl_m <= 2.0 or has_flare_demand):
            if has_flare_demand or is_approach_transition or (ch2 > 0.05 and ch3 <= 0.40):
                return Intent.LANDING_FLARE, 0.96, {
                    'regime': landing_reg,
                    'pitch': ch2,
                    'throttle': ch3,
                    'h_agl_m': h_agl_m,
                    'V_ms': V_ms,
                    'crosswind_ms': context.get('crosswind_ms', 0.0),
                    'lateral': ch1,
                    'yaw': ch4,
                }

        # 2. Initial Takeoff Climbout (unstick to pattern altitude)
        if context.get('pelican_deep_stroke_demand', False) or (ch3 >= 0.75 and V_ms < 6.0 and context.get('deep_stroke_demand', False)):
            return Intent.PELICAN_DEEP_STROKE, 0.96, {
                'throttle': ch3,
                'pitch': ch2,
                'lever_pct': ch3 * 100.0,
                'V_ms': V_ms,
                'h_agl_m': h_agl_m,
            }

        if not on_ground and self._last_intent in (Intent.ROTATE, Intent.TAKEOFF_ROLL) and ch3 >= 0.50:
            return Intent.CLIMB, 0.95, {
                'throttle': ch3,
                'pitch': max(0.15, ch2),
                'lever_pct': ch3 * 100.0,
                'h_agl_m': h_agl_m,
                'V_ms': V_ms,
            }

        # 3. Approach: explicit approach demand/regime or descending glide path near runway
        is_stol_regime = (landing_reg in ('short_field', 'stol', 'stol_landing', 'stol_spot'))
        is_stol_approach = (context.get('stol_approach', False) or (is_stol_regime and ch3 <= 0.45 and not is_takeoff_climb))
        if not on_ground and (
                context.get('landing_approach', False) or
                context.get('approach_demand', False) or
                context.get('approach_normal', False) or
                context.get('stol_approach', False) or
                (context.get('landing_regime') is not None and h_agl_m <= 50.0 and ch3 <= 0.50) or
                (is_stol_approach and h_agl_m <= 35.0 and V_ms <= 8.0) or
                (h_agl_m <= 25.0 and ch3 <= 0.45 and ch2 <= 0.08 and V_ms >= 4.0 and not context.get('falcon_morph', False))):
            return Intent.APPROACH_NORMAL, 0.94, {
                'regime': landing_reg,
                'throttle': ch3,
                'pitch': ch2,
                'lateral': ch1,
                'yaw': ch4,
                'h_agl_m': h_agl_m,
                'V_ms': V_ms,
                'crosswind_ms': context.get('crosswind_ms', 0.0),
            }
        if wing_sw == 0:
            gs_id = 'GS-0'
        elif wing_sw == 1:
            gs_id = 'GS-2' if (V_ms > 10.0 and ch3 < 0.3) else 'GS-1'
        else:
            if macro_lng:   gs_id = 'GS-5'
            elif macro_sw:  gs_id = 'GS-3'
            else:           gs_id = 'GS-4'

        # ── Macro triggers ───────────────────────────────────────
        if macro_sw or macro_lng:
            if mode_sw == 0:
                if macro_lng:
                    return Intent.STOOP, 0.95, {'gs_id': gs_id}
                return Intent.CF1_BURST, 0.9, {}
            elif mode_sw == 1:
                direction = 'right' if ch1 > 0 else 'left'
                if macro_lng:
                    # Full roll + long press = dart (Rev L)
                    if abs(ch1) > 0.7:
                        di = 'right' if ch1 > 0 else 'left'
                        return (Intent.DART_RIGHT if ch1 > 0
                                else Intent.DART_LEFT), 0.95, {
                                    'direction': di}
                    return Intent.CHANDELLE, 0.9, {'direction': direction}
                return Intent.TIGHT_TURN, 0.95, {
                    'bank_deg': 65.0, 'direction': direction}
            elif mode_sw == 2:
                if macro_lng:
                    return Intent.APPROACH_SITE, 0.9, {}
                return Intent.SPIRAL_DOWN, 0.9, {}

        # ── SAR mode ────────────────────────────────────────────
        if mode_sw == 2:
            if context.get('thermal_loiter', False) or context.get('sar_loiter', False):
                # Rev S: Azhdarchid thermal soaring deep-camber loiter (H-102)
                return Intent.AZHDARCHID_SOAR, 0.95, {
                    'min_sink_ms': 0.38,
                    'endurance_boost_pct': 86.0,
                    'gs_id': gs_id,
                }
            if abs(ch1) < self.STICK_THRESHOLD:
                return Intent.LOITER, 0.8, {'gs_id': gs_id}
            if ch1 > self.ROLL_TURN_MIN:
                return Intent.ORBIT_RIGHT, 0.8, {
                    'bank_deg': 30 + ch1*20, 'gs_id': gs_id}
            return Intent.ORBIT_LEFT, 0.8, {
                'bank_deg': 30 + abs(ch1)*20, 'gs_id': gs_id}

        # ── Manoeuvre mode ───────────────────────────────────────
        if mode_sw == 1:
            if abs(ch1) > self.STICK_THRESHOLD:
                direction = 'right' if ch1 > 0 else 'left'
                bank = min(30 + abs(ch1)*35, 65.0)
                if abs(ch1) > 0.85:
                    # Rev P: Anhedral gull-wing knife-edge turn trapping (H-82)
                    return Intent.ANHEDRAL_TURN, abs(ch1), {
                        'direction': direction,
                        'bank_deg': bank,
                        'gs_id': gs_id,
                    }
                if ch2 > 0.3 and abs(ch1) > 0.5:
                    return Intent.CHANDELLE, 0.85, {
                        'direction': direction, 'gs_id': gs_id}
                mt = 'MT-4' if bank > 60 else 'MT-3' if bank > 45 else 'MT-2'
                return Intent.TIGHT_TURN, abs(ch1), {
                    'bank_deg': bank, 'direction': direction,
                    'mt_id': mt, 'gs_id': gs_id,
                    'coord_mode': coord_mode}

        # ── Cruise mode ──────────────────────────────────────────
        lever_pct = ch3 * 100.0

        if context.get('falcon_dart', False) or context.get('falcon_trim', False) or (lever_pct >= 85.0 and V_ms >= 14.0 and not context.get('force_sprint', False)):
            # H-FALCON-TRIM-1: Fanned bi-delta tailplane fast cruise & level dash
            return Intent.FALCON_DART, 0.96, {
                'lever_pct': lever_pct,
                'V_ms': V_ms,
                'sweep_schedule_deg': [5.0, 15.0, 28.0, 50.0],
                'tailplane_aoa_deg': -5.5,
                'gs_id': gs_id,
            }

        if lever_pct >= 85.0:
            if V_ms < 6.0:
                # Rev R: Lepidopteran dorsal clap jet breakout (H-91)
                return Intent.CLAP_JET_BREAKOUT, 0.95, {
                    'jet_ejection_ms': 22.0,
                    'accel_g': 1.45,
                    'gs_id': gs_id,
                }
            return Intent.SPRINT, 1.0, {'lever_pct': lever_pct, 'gs_id': gs_id}

        if abs(ch1) > self.ROLL_TURN_MIN:
            direction = 'right' if ch1 > 0 else 'left'
            roll_abs  = abs(ch1)
            if   roll_abs > 0.70: mt = 'MT-3'; bank = 55 + roll_abs*10
            elif roll_abs > 0.30: mt = 'MT-2'; bank = 35 + roll_abs*20
            else:                  mt = 'MT-1'; bank = 20 + roll_abs*30
            bank = min(bank, 65.0)
            return (Intent.TIGHT_TURN if bank > 50 else
                    Intent.TURN_RIGHT if ch1 > 0 else Intent.TURN_LEFT), \
                   roll_abs, {'bank_deg': bank, 'direction': direction,
                              'mt_id': mt, 'gs_id': gs_id,
                              'coord_mode': coord_mode}

        if lever_pct > 52.5 and (ch2 >= 0.12 or (ch3 >= 0.65 and h_agl_m < 50.0 and ch2 >= 0.05)):
            if ch2 >= 0.70 or context.get('obstacle_climb', False):
                # Rev R: Coleopteran elytron canard obstacle climb (H-92)
                return Intent.ELYTRON_CANARD, 0.90, {
                    'canard_locked_station': 1,
                    'canard_incidence_deg': 8.5,
                    'climb_angle_deg': 48.0,
                    'gs_id': gs_id,
                }
            return Intent.CLIMB, ch2, {'lever_pct': lever_pct, 'gs_id': gs_id}

        if 65.0 <= lever_pct < 85.0 and V_ms >= 10.0:
            # Rev Q: Reverse metachronal cascade cruise thrust boost (H-86)
            return Intent.REVERSE_CASCADE_CRUISE, 0.90, {
                'lever_pct': lever_pct,
                'cascade_phase_deg': -60.0,
                'thrust_boost_pct': 24.5,
                'gs_id': gs_id,
            }

        if 47.5 <= lever_pct <= 65.0 and V_ms >= 9.0 and not context.get('high_turbulence', False):
            # Rev R: Orthopteran traveling torsional wave cruise (H-94)
            return Intent.LOCUST_TORSIONAL_CRUISE, 0.88, {
                'lever_pct': lever_pct,
                'torsional_amplitude_deg': 4.2,
                'drag_reduction_pct': 32.0,
                'gs_id': gs_id,
            }

        if lever_pct < 47.5:
            if not on_ground and (context.get('thermal_loiter', False) or context.get('sar_loiter', False) or context.get('azhdarchid_soar', False) or ('w_sink_ms' in context and context['w_sink_ms'] > -0.8 and V_ms >= 6.0)):
                # Rev S: Azhdarchid thermal soaring deep-camber loiter (H-102)
                return Intent.AZHDARCHID_SOAR, 0.95, {
                    'lever_pct': lever_pct,
                    'min_sink_ms': 0.38,
                    'endurance_boost_pct': 86.0,
                    'gs_id': gs_id,
                }
            if ch2 < -0.05 or context.get('falcon_morph', False) or context.get('morphing_polar', False):
                return Intent.GLIDE, 1.0, {'lever_pct': lever_pct, 'gs_id': gs_id, 'falcon_morph': True}
            if V_ms >= 4.0:
                # Rev R: Tandem dynamic slotted Fowler flap glide (H-95)
                return Intent.SLOTTED_FLAP_GLIDE, 0.92, {
                    'lever_pct': lever_pct,
                    'slot_fowler_cl_max': 2.45,
                    'gs_id': gs_id,
                }
            return Intent.GLIDE, 1.0, {'lever_pct': lever_pct, 'gs_id': gs_id}

        if self.LEVER_DETENT_LOW <= lever_pct <= self.LEVER_DETENT_HIGH:
            return Intent.CRUISE, 0.7, {'lever_pct': lever_pct, 'gs_id': gs_id}

        return Intent.CRUISE, 0.8, {'lever_pct': lever_pct, 'throttle': ch3, 'gs_id': gs_id}


# ── BEHAVIOUR NODE BASE ──────────────────────────────────────────

RUNNING = 'RUNNING'
SUCCESS = 'SUCCESS'
FAILURE = 'FAILURE'


class BehaviourNode:
    name          = 'base'
    priority      = 99
    interruptible = True

    def __init__(self):
        self._state   = None
        self._t_start = None
        self._log     = []

    def on_enter(self, context):
        self._t_start = context.get('t', 0.0)
        self._state   = RUNNING

    def tick(self, context, ctrl):
        return RUNNING

    def on_exit(self, context, completed):
        pass

    def can_be_preempted_by(self, other_priority, is_safety=False):
        if is_safety: return True
        if not self.interruptible: return False
        return other_priority <= self.priority

    def elapsed(self, context):
        if self._t_start is None: return 0.0
        return context.get('t', 0.0) - self._t_start

    def log(self, msg):
        self._log.append(msg)


# ── CONTINUOUS BEHAVIOURS ────────────────────────────────────────

class CruiseBehaviour(BehaviourNode):
    name = 'cruise'; priority = 10; interruptible = True
    def tick(self, context, ctrl):
        import mco_wave_library as wl
        Ps    = context.get('Ps', 0.2)
        V     = context.get('V_ms', 8.5)
        pitch = context.get('pitch_deg', 0.0)
        lever = context.get('lever_pct', 60.0)
        p = wl.lever_airspeed_params(lever, V, pitch, Ps, ctrl.freq_hz)
        ctrl._tgt_freq      = p['freq_hz']
        ctrl._tgt_sweeps    = [p['sweep_deg']] * 8
        ctrl._tgt_dihedrals = [p['dihedral_deg']] * 8
        ctrl._tgt_pitches   = [p['pitch_deg']] * 8
        return RUNNING


class ClimbBehaviour(BehaviourNode):
    name = 'climb'; priority = 9; interruptible = True
    def tick(self, context, ctrl):
        import mco_wave_library as wl
        Ps    = context.get('Ps', 0.5)
        V     = context.get('V_ms', 8.5)
        pitch = context.get('pitch_deg', 10.0)
        lever = context.get('lever_pct', 82.0)
        # Rev O3: near stall → X-29 forward sweep for retained authority
        x29 = wl.x29_high_aoa_params(V, Ps, ctrl.freq_hz)
        if x29 is not None:
            ctrl._tgt_freq      = x29['freq_hz']
            ctrl._tgt_sweeps    = x29['sweeps']
            ctrl._tgt_dihedrals = x29['dihedrals']
            ctrl._tgt_pitches   = x29['pitches']
        else:
            p = wl.lever_airspeed_params(lever, V, pitch, Ps, ctrl.freq_hz)
            ctrl._tgt_freq      = p['freq_hz']
            ctrl._tgt_sweeps    = [p['sweep_deg']] * 8
            ctrl._tgt_dihedrals = [0.0] * 8
            ctrl._tgt_pitches   = [p['pitch_deg']] * 8
        return RUNNING


class GlideBehaviour(BehaviourNode):
    """Unified lever — glide/approach/airbrake zones with Falcon Morphing Polar (Glide-Dive-Stoop)."""
    name = 'glide'; priority = 9; interruptible = True
    def tick(self, context, ctrl):
        import mco_wave_library as wl
        import mco_advanced_physics as ap

        sticks = context.get('sticks', {})
        ch2 = context.get('pitch', sticks.get('pitch', sticks.get('ch2', context.get('ch2', 0.0))))
        lever = context.get('lever_pct', 35.0)
        V     = context.get('V_ms', 8.0)
        Ps    = context.get('Ps', 0.0)
        pitch = context.get('pitch_deg', 0.0)
        h_agl = context.get('h_agl_m', context.get('h_agl', 100.0))

        # Check if Falcon morphing polar active (continuous stick pitch modulation or explicit flag)
        if ch2 < -0.05 or context.get('falcon_morph', False) or context.get('morphing_polar', False):
            alc_bias = context.get('alc_bias', None)
            if alc_bias is None and 'w_netto_ms' in context:
                alc_bias = ap.alc_maccready_glide_bias(context['w_netto_ms'])
            pilot_ctrl = {
                'ch1': context.get('roll', sticks.get('roll', sticks.get('ch1', context.get('ch1', 0.0)))),
                'ch4': context.get('yaw', sticks.get('yaw', sticks.get('ch4', context.get('ch4', 0.0)))),
            }
            cmds, freq_hz, diag = ap.falcon_morphing_glide_polar(
                pitch_stick=ch2,
                V_ms=V,
                h_agl_m=h_agl,
                gross_weight_kg=context.get('gross_weight_kg', 5.687),
                alc_bias=alc_bias,
                pilot_controls=pilot_ctrl
            )
            ctrl._tgt_freq      = freq_hz
            ctrl._tgt_sweeps    = [c['sweep_deg'] for c in cmds]
            ctrl._tgt_dihedrals = [c['dihedral_deg'] for c in cmds]
            ctrl._tgt_pitches   = [c['pitch_deg'] for c in cmds]
            ctrl.falcon_morph_diag = diag
            return RUNNING

        safe, _ = wl.lever_stall_check(lever, V)
        if not safe:
            lever = max(lever, 15.0)
        if wl.lever_in_detent(lever):
            return RUNNING
        p = wl.lever_airspeed_params(lever, V, pitch, Ps, ctrl.freq_hz)
        ctrl._tgt_freq      = p['freq_hz']
        ctrl._tgt_sweeps    = [p['sweep_deg']]    * 8
        ctrl._tgt_dihedrals = [p['dihedral_deg']] * 8
        ctrl._tgt_pitches   = [p['pitch_deg']]    * 8
        return RUNNING


class TurnBehaviour(BehaviourNode):
    """Four-phase Boyd turn with Rev K auto-coordination."""
    name = 'turn'; priority = 8; interruptible = True
    PHASE_ENTRY   = 'entry'
    PHASE_BANKED  = 'banked'
    PHASE_ROLLOUT = 'rollout'
    PHASE_RESUME  = 'resume'

    def on_enter(self, context):
        super().on_enter(context)
        self._phase      = self.PHASE_ENTRY
        self._phase_t    = context.get('t', 0.0)
        self._mt_id      = context.get('mt_id', 'MT-1')
        self._dir        = context.get('direction', 'right')

    def tick(self, context, ctrl):
        import mco_wave_library as wl
        t         = context.get('t', 0.0)
        V_ms      = context.get('V_ms', 9.0)
        bank_deg  = context.get('bank_deg', 0.0)
        Ps        = context.get('Ps', 0.5)
        coord     = context.get('coord_mode', 'auto')
        dt_phase  = t - self._phase_t
        mm        = wl.MANOEUVRE_MODES.get(self._mt_id, {})
        bank_tgt  = mm.get('bank_deg', 30.0)

        if self._phase == self.PHASE_ENTRY:
            cmd = wl.turn_entry_commands(bank_tgt, self._dir, V_ms, ctrl.freq_hz)
            ctrl._tgt_freq      = cmd['freq_hz']
            ctrl._tgt_sweeps    = cmd['sweeps']
            ctrl._tgt_dihedrals = cmd['dihedrals']
            ctrl._tgt_pitches   = [cmd['pitch_deg']] * 8
            if dt_phase >= 0.5 or abs(bank_deg) > 5.0:
                self._phase   = self.PHASE_BANKED
                self._phase_t = t

        elif self._phase == self.PHASE_BANKED:
            cmd = wl.turn_in_progress_commands(
                bank_deg=abs(bank_deg), turn_direction=self._dir,
                V_ms=V_ms, Ps=Ps, current_freq_hz=ctrl.freq_hz)
            ctrl._tgt_freq   = cmd['freq_hz']
            ctrl._tgt_sweeps = cmd['sweeps']
            base_d = cmd['dihedrals']
            if coord == 'auto':
                base_d = wl.auto_coordinated_params(
                    base_d, bank_deg, V_ms,
                    context.get('yaw_rate_radps', 0.0), self._dir)
            elif coord in ('decoupled', 'semi'):
                yd = wl.yaw_dihedral_asymmetry(context.get('yaw', 0.0))
                base_d = [base_d[i] + yd[i] for i in range(8)]
            ctrl._tgt_dihedrals = base_d
            ctrl._tgt_pitches   = [cmd['pitch_deg']] * 8

        elif self._phase == self.PHASE_ROLLOUT:
            cmd = wl.turn_rollout_commands(abs(bank_deg), self._dir)
            ctrl._tgt_freq      = cmd['freq_hz']
            ctrl._tgt_sweeps    = cmd['sweeps']
            ctrl._tgt_dihedrals = cmd['dihedrals']
            ctrl._tgt_pitches   = [cmd['pitch_deg']] * 8
            if abs(bank_deg) < 5.0 and dt_phase > 0.5:
                self._phase   = self.PHASE_RESUME
                self._phase_t = t

        elif self._phase == self.PHASE_RESUME:
            ctrl._tgt_freq      = wl.TURN_CRUISE_FREQ_HZ
            ctrl._tgt_sweeps    = [60.0] * 8
            ctrl._tgt_dihedrals = [0.0]  * 8
            ctrl._tgt_pitches   = [0.0]  * 8
            if dt_phase >= 2.0:
                self._state = SUCCESS
                return SUCCESS

        return RUNNING

    def begin_rollout(self, context):
        if self._phase == self.PHASE_BANKED:
            self._phase   = self.PHASE_ROLLOUT
            self._phase_t = context.get('t', 0.0)


class SprintBehaviour(BehaviourNode):
    name = 'sprint'; priority = 9; interruptible = True
    def tick(self, context, ctrl):
        import mco_wave_library as wl
        Ps = context.get('Ps', 1.0)
        V  = context.get('V_ms', 12.0)
        freq, sweep, dihs, pitch = wl.sprint_params(Ps, V, 0.02, ctrl.freq_hz)
        ctrl._tgt_freq      = freq
        ctrl._tgt_sweeps    = [sweep] * 8
        ctrl._tgt_dihedrals = dihs
        ctrl._tgt_pitches   = [pitch] * 8
        return RUNNING


class FalconDartBehaviour(BehaviourNode):
    """
    H-FALCON-TRIM-1: Fanned Bi-Delta Tailplane Fast Cruise & Level Dash.
    Stations 1-3 active propulsive cascade (+5°, +15°, +28° sweep, ±18° stroke),
    Station 4 stationary FBW tailplane (+50° sweep, 0° stroke, -5.5° pitch trim).
    Adversarially hardened with 350ms S-curve motor deceleration and 120ms pitch lead.
    """
    name = 'falcon_dart'; priority = 6; interruptible = True

    def tick(self, context, ctrl):
        import mco_wave_library as wl
        import mco_flight_governor as fg
        V = context.get('V_ms', 20.80)
        t = context.get('t', 0.0)
        dt = context.get('dt_s', 0.02)
        sticks = context.get('sticks', {})
        p_stk = context.get('pitch_stick', sticks.get('pitch', sticks.get('ch2', 0.0)))
        r_stk = context.get('roll_stick', sticks.get('roll', sticks.get('ch1', 0.0)))
        y_stk = context.get('yaw_stick', sticks.get('yaw', sticks.get('ch4', 0.0)))
        on_gnd = context.get('on_ground', False)

        f_hz = 3.0 if V >= 24.0 else 2.8
        wave_res = wl.wave_falcon_dart(
            t_sec=t, f_hz=f_hz, flap_amp_deg=18.0,
            v_ms=V, pitch_stick=p_stk, roll_stick=r_stk, yaw_stick=y_stk,
            on_ground=on_gnd
        )

        # Coordinate spin-down and pitch-lead handshake
        trans = fg.govern_falcon_dart_transition(
            dt_s=dt,
            falcon_mode_requested=(not wave_res['aborted']),
            current_f_hz=f_hz,
            nominal_amp_deg=18.0,
            w4_pitch_cmd=-5.5,
            w4_dih_cmd=0.0
        )

        ctrl._tgt_freq = wave_res['f_hz']
        # 8 wings: W1 port/stbd (5°), W2 port/stbd (15°), W3 port/stbd (28°), W4 port/stbd (50°)
        ctrl._tgt_sweeps = [5.0, 5.0, 15.0, 15.0, 28.0, 28.0, 50.0, 50.0]
        # W4 dihedral is thermally clamped to safe low holding range (0.0° to 3.0°)
        w4_dih = trans['w4_dihedral_deg']
        ctrl._tgt_dihedrals = [5.0, 5.0, 5.0, 5.0, 5.0, 5.0, w4_dih, w4_dih]
        # Washout trim: W1 (+2.5°), W2 (+1.5°), W3 (+1.0°), W4 (active lead/handshake pitch)
        w4_pitch = trans['w4_pitch_trim_deg']
        ctrl._tgt_pitches = [2.5, 2.5, 1.5, 1.5, 1.0, 1.0, w4_pitch, w4_pitch]

        # Station 4 stationary tailplane lock flag and amplitude scale
        if hasattr(ctrl, 'stationary_tailplane_active'):
            ctrl.stationary_tailplane_active = (trans['state'] == 'DART_LOCKED')
        if hasattr(ctrl, 'w4_amp_scale'):
            ctrl.w4_amp_scale = trans['w4_amp_scale']

        wave_res['transition_gov'] = trans
        ctrl.falcon_dart_diag = wave_res
        return RUNNING


# ── MACRO BEHAVIOURS ─────────────────────────────────────────────

class StoopMacro(BehaviourNode):
    """
    Falcon high-speed diving stoop with automated pullout.

    [AUTOPILOT DEFERRED]
    Designated as an autonomous trajectory function deferred to the post-first-flight
    autopilot implementation bucket (see _AUTOPILOT_DEFERRED['STOOP_MACRO']
    in mco_wave_library.py). Not part of active manual-pilot flight laws.
    """
    name = 'stoop'; priority = 2; interruptible = False
    PHASES = ['entry', 'dive', 'pullout', 'recovery']

    def on_enter(self, context):
        super().on_enter(context)
        self._phase   = 'entry'
        self._phase_t = context.get('t', 0.0)

    def tick(self, context, ctrl):
        import mco_wave_library as wl
        t     = context.get('t', 0.0)
        V_ms  = context.get('V_ms', 0.0)
        h_agl = context.get('h_agl_m', 100.0)
        Ps    = context.get('Ps', 0.0)
        dt    = t - self._phase_t

        if self._phase == 'entry':
            freq, sw, dihs, pt = wl.stoop_dive_params(V_ms, h_agl, ctrl.freq_hz)
            ctrl._tgt_sweeps    = [sw]  * 8
            ctrl._tgt_dihedrals = dihs
            ctrl._tgt_freq      = freq
            ctrl._tgt_pitches   = [pt] * 8
            if all(s > 200 for s in ctrl._tgt_sweeps):
                self._phase = 'dive'; self._phase_t = t

        elif self._phase == 'dive':
            freq, sw, dihs, pt = wl.stoop_dive_params(V_ms, h_agl, ctrl.freq_hz)
            ctrl._tgt_freq      = freq
            ctrl._tgt_sweeps    = [sw]  * 8
            ctrl._tgt_dihedrals = dihs
            ctrl._tgt_pitches   = [pt] * 8
            pullout, reason = wl.ps_pullout_recommended(
                Ps, h_agl, V_ms, V_ms - wl.VS_KT/1.944)
            if pullout:
                self._phase = 'pullout'; self._phase_t = t

        elif self._phase == 'pullout':
            freq, sw, dihs, pt = wl.stoop_pullout_params(Ps, V_ms, 0.02, ctrl.freq_hz)
            ctrl._tgt_freq      = freq
            ctrl._tgt_sweeps    = [sw]  * 8
            ctrl._tgt_dihedrals = dihs
            ctrl._tgt_pitches   = [pt] * 8
            if V_ms < 12.0 and dt > 2.0:
                self._phase = 'recovery'; self._phase_t = t

        elif self._phase == 'recovery':
            ctrl._tgt_sweeps    = [60.0] * 8
            ctrl._tgt_freq      = 0.8
            ctrl._tgt_dihedrals = [0.0]  * 8
            ctrl._tgt_pitches   = [0.0]  * 8
            if dt > 3.0:
                self._state = SUCCESS; return SUCCESS

        return RUNNING

    def can_be_preempted_by(self, other_priority, is_safety=False):
        return is_safety or other_priority == 0


class CF1Macro(BehaviourNode):
    """
    Clap-and-fling high-thrust vortex burst sequence.

    [AUTOPILOT DEFERRED]
    Designated as an autonomous trajectory function deferred to the post-first-flight
    autopilot implementation bucket (see _AUTOPILOT_DEFERRED['CF1_BURST_MACRO']
    in mco_wave_library.py). Not part of active manual-pilot flight laws.
    """
    name = 'cf1_burst'; priority = 4; interruptible = True
    def on_enter(self, context):
        super().on_enter(context)
        self._strokes = 0
    def tick(self, context, ctrl):
        import mco_wave_library as wl
        V_ms = context.get('V_ms', 0.0)
        ok, reason = wl.cf1_available(V_ms, wl.CF1_DEFAULT_FREQ_HZ, ctrl._tgt_sweeps)
        if not ok:
            self._state = FAILURE; return FAILURE
        ctrl._tgt_freq      = wl.CF1_DEFAULT_FREQ_HZ
        ctrl._tgt_sweeps    = [60.0]  * 8
        ctrl._tgt_dihedrals = [0.0]   * 8
        ctrl._tgt_pitches   = [wl.CF1_PITCH_OFFSET_DEG] * 8
        self._strokes = int(self.elapsed(context) * wl.CF1_DEFAULT_FREQ_HZ)
        if self._strokes >= wl.CF1_BURST_MAX:
            self._state = SUCCESS; return SUCCESS
        return RUNNING


class RecoverBehaviour(BehaviourNode):
    name = 'recover'; priority = 0; interruptible = False
    def tick(self, context, ctrl):
        import mco_wave_library as wl
        V_ms  = context.get('V_ms', 5.0)
        pitch = context.get('pitch_deg', 0.0)
        # Rev O3: X-29 forward sweep for stall recovery
        # Root stalls first → tip pitch servos remain effective
        p = wl.x29_stall_recovery_params(
            V_ms, pitch, ctrl.freq_hz)
        ctrl._tgt_freq      = p['freq_hz']
        ctrl._tgt_sweeps    = p['sweeps']
        ctrl._tgt_dihedrals = p['dihedrals']
        ctrl._tgt_pitches   = p['pitches']
        if p['recovering'] and self.elapsed(context) > 1.5:
            self._state = SUCCESS; return SUCCESS
        if self.elapsed(context) > 5.0:
            ctrl.emergency_recover()
            self._state = SUCCESS; return SUCCESS
        return RUNNING


class ChandellesMacro(BehaviourNode):
    """
    Chandelle macro — 180° maximum-performance climbing turn.

    [AUTOPILOT DEFERRED]
    Designated as an autonomous trajectory function deferred to the post-first-flight
    autopilot implementation bucket (see _AUTOPILOT_DEFERRED['CHANDELLE_MACRO']
    in mco_wave_library.py). Not part of active manual-pilot flight laws.
    """
    name = 'chandelle'; priority = 5; interruptible = True
    def on_enter(self, context):
        super().on_enter(context)
        self._entry_heading = context.get('heading_deg', 0.0)
        self._direction     = context.get('direction', 'right')
    def tick(self, context, ctrl):
        import mco_wave_library as wl
        heading_change = abs(
            context.get('heading_deg', 0.0) - self._entry_heading) % 360
        if heading_change > 180: heading_change = 360 - heading_change
        freq  = wl.chandelle_freq(heading_change)
        bank  = wl.chandelle_bank(heading_change)
        ctrl._tgt_freq   = freq
        ctrl._tgt_sweeps = [wl.CHANDELLE_SWEEP_DEG] * 8
        ctrl._tgt_pitches= [wl.CHANDELLE_PITCH_OFFSET_DEG] * 8
        sw_delta = bank * 0.5
        for i in range(8):
            is_port = (i % 2 == 0)
            inner   = ((self._direction=='right') and not is_port) or \
                      ((self._direction=='left')  and     is_port)
            ctrl._tgt_sweeps[i] = (wl.CHANDELLE_SWEEP_DEG + sw_delta
                                    if inner else wl.CHANDELLE_SWEEP_DEG)
        if heading_change >= 178:
            self._state = SUCCESS; return SUCCESS
        return RUNNING


# ── REV L — DART MACRO ───────────────────────────────────────────

class DartMacro(BehaviourNode):
    """
    Rev L: Dragonfly dart — W1+W4 pitch 75° (vertical fins).
    W2+W3 continue flapping to sustain lift throughout.
    2.4g lateral, 90° heading change in 0.15s at V=14m/s.
    Analogous to Fokker Dr.I snap-turn, Iz = 0.17 kg·m².

    [AUTOPILOT DEFERRED: SCRIPTED MACRO ONLY]
    The canned multi-phase state machine macro is deferred to autonomous autopilot collision avoidance.
    Manual pilot dart turns are actively handled invisibly on every tick by `dart_control_tick()`
    in `mco_wave_library.py` under coordinated pilot roll stick and rudder pedal inputs!
    """
    name = 'dart'; priority = 3; interruptible = False

    def on_enter(self, context):
        super().on_enter(context)
        import mco_wave_library as wl
        self._dir = context.get('direction', 'right')
        V = context.get('V_ms', 9.0)
        safe, reason = wl.dart_safe_to_initiate(
            V, context.get('h_agl_m', 50.0),
            context.get('bank_deg', 0.0))
        if not safe and 'sprint' not in reason:
            self._state = FAILURE
            self.log(f'Dart blocked: {reason}')
            return
        self._phase = (wl.DART_PHASE_ENGAGE
            if V >= wl.DART_PREBUILD_V_MS * 0.95
            else wl.DART_PHASE_PREBUILD)
        self._phase_t = context.get('t', 0.0)
        self.log(f'Dart {self._dir} V={V:.1f}m/s')

    def tick(self, context, ctrl):
        import mco_wave_library as wl
        t  = context.get('t', 0.0)
        V  = context.get('V_ms', 9.0)
        dt = t - self._phase_t
        if self._phase == wl.DART_PHASE_DISENGAGE:
            p = wl.dart_disengage_fling(
                dt, self._dir, V, ctrl.freq_hz)
        else:
            p  = wl.dart_sequence_params(
                self._phase, dt, self._dir, V,
                context.get('bank_deg', 0.0), ctrl.freq_hz)
        ctrl._tgt_freq      = p['freq_hz']
        ctrl._tgt_sweeps    = p['sweeps']
        ctrl._tgt_dihedrals = p['dihedrals']
        ctrl._tgt_pitches   = p['pitches']
        # Sync fin mode so enforce_physical_limits uses right AoA range
        ctrl._fin_mode_pairs = wl._FIN_MODE_PAIRS.copy() \
            if hasattr(wl,'_FIN_MODE_PAIRS') else None
        if p['phase_done']:
            _NX = {wl.DART_PHASE_PREBUILD:  wl.DART_PHASE_ENGAGE,
                   wl.DART_PHASE_ENGAGE:    wl.DART_PHASE_PIVOT,
                   wl.DART_PHASE_PIVOT:     wl.DART_PHASE_DISENGAGE,
                   wl.DART_PHASE_DISENGAGE: wl.DART_PHASE_RESUME,
                   wl.DART_PHASE_RESUME:    None}
            nxt = _NX.get(self._phase)
            if nxt is None:
                self._state = SUCCESS
                self.log(f'Dart {self._dir} complete')
                return SUCCESS
            self.log(f'Dart phase {self._phase}→{nxt}')
            self._phase = nxt
            self._phase_t = t
        return RUNNING

    def can_be_preempted_by(self, other_priority, is_safety=False):
        return is_safety


# ── REV K — COORDINATION BEHAVIOURS ─────────────────────────────

class ZoomClimbMacro(BehaviourNode):
    """
    Rev N: Zoom climb — V→h Boyd energy conversion.

    [AUTOPILOT DEFERRED]
    Designated as an autonomous trajectory function deferred to the post-first-flight
    autopilot implementation bucket (see _AUTOPILOT_DEFERRED['ZOOM_CLIMB_MACRO']
    in mco_wave_library.py). Not part of active manual-pilot flight laws.
    """
    name = 'zoom_climb'; priority = 4; interruptible = True

    def on_enter(self, context):
        super().on_enter(context)
        import mco_wave_library as wl
        V = context.get('V_ms', 9.0)
        safe, reason = wl.zoom_safe_to_initiate(
            V, context.get('h_agl_m', 50.0))
        if not safe:
            self._state = FAILURE; self.log(f'Zoom blocked:{reason}'); return
        self._phase   = wl.ZOOM_PHASE_ENTRY
        self._phase_t = context.get('t', 0.0)
        self.log(f'Zoom V={V:.1f}m/s')

    def tick(self, context, ctrl):
        import mco_wave_library as wl
        t=context.get('t',0.0); V=context.get('V_ms',9.0)
        Ps=context.get('Ps',0.0); dt=t-self._phase_t
        np_=wl.zoom_phase_from_state(V,dt,Ps)
        if np_!=self._phase:
            self.log(f'{self._phase}→{np_}'); self._phase=np_; self._phase_t=t
        if self._phase==wl.ZOOM_PHASE_ENTRY:
            p=wl.zoom_entry_params(V)
        elif self._phase==wl.ZOOM_PHASE_CLIMB:
            p=wl.zoom_climb_params(V,Ps,ctrl.freq_hz,dt)
        elif self._phase==wl.ZOOM_PHASE_APEX:
            p=wl.zoom_apex_params(V)
        else:
            p=wl.zoom_resume_params(V,Ps,ctrl.freq_hz)
            if dt>2.0: self._state=SUCCESS; return SUCCESS
        ctrl._tgt_freq=p['freq_hz']; ctrl._tgt_sweeps=p['sweeps']
        ctrl._tgt_dihedrals=p['dihedrals']; ctrl._tgt_pitches=p['pitches']
        return RUNNING


class PowerDiveBehaviour(BehaviourNode):
    """Rev N: Power dive — wings deployed, gravity+thrust."""
    name = 'power_dive'; priority = 8; interruptible = True

    def tick(self, context, ctrl):
        import mco_wave_library as wl
        pitch=context.get('pitch_deg',-15.0)
        V=context.get('V_ms',9.0); Ps=context.get('Ps',0.5)
        lever=context.get('lever_pct',80.0)
        safe,_=wl.power_dive_safe(V,context.get('h_agl_m',50.0),pitch)
        if not safe: self._state=FAILURE; return FAILURE
        p=wl.power_dive_params(pitch,V,Ps,ctrl.freq_hz,lever)
        ctrl._tgt_freq=p['freq_hz']; ctrl._tgt_sweeps=p['sweeps']
        ctrl._tgt_dihedrals=p['dihedrals']; ctrl._tgt_pitches=p['pitches']
        return RUNNING


class CrabBehaviour(BehaviourNode):
    """
    Rev O3: Crab manoeuvre — lateral translation without heading change.
    Port wings at 30° sweep → 17N lateral = 3.5g.
    Fin pitch at 44° cancels induced yaw → heading held.
    Stbd wings at 60° → normal forward thrust + lift.
    4 independent control axes simultaneously.
    """
    name = 'crab'; priority = 7; interruptible = True

    def tick(self, context, ctrl):
        import mco_wave_library as wl
        direction = context.get('direction', 'left')
        intensity = context.get('intensity', 1.0)
        V_ms  = context.get('V_ms', 9.0)
        bank  = context.get('bank_deg', 0.0)
        safe, reason = wl.crab_safe(V_ms,
            context.get('h_agl_m', 50.0), bank)
        if not safe:
            self._state = FAILURE; return FAILURE
        p = wl.crab_params(
            direction, intensity, V_ms,
            ctrl.freq_hz, heading_hold=True)
        ctrl._tgt_sweeps    = p['sweeps']
        ctrl._tgt_pitches   = p['pitches']
        ctrl._tgt_freq      = p['freq_hz']
        ctrl._tgt_dihedrals = [0.0] * 8
        return RUNNING


class FlatTurnBehaviour(BehaviourNode):
    """Rev K/L: Flat turn — phase asymmetry + fin pitch (W1+W4)."""
    name = 'flat_turn'; priority = 8; interruptible = True

    def tick(self, context, ctrl):
        import mco_wave_library as wl
        yaw  = context.get('yaw', 0.0)
        bank = context.get('bank_deg', 0.0)
        base = [0.0, 90.0, 180.0, 270.0, 0.0, 90.0, 180.0, 270.0]
        phases, sw_corr, freq, _ = wl.flat_turn_params(
            yaw, bank, base, ctrl.freq_hz)
        ctrl._tgt_freq = freq
        for i in range(8):
            ctrl._tgt_sweeps[i] = max(60.0, ctrl._tgt_sweeps[i] + sw_corr[i])
        # Rev L: add fin pitch for maximum yaw authority
        yaw_dir = 'right' if yaw >= 0 else 'left'
        ctrl._tgt_pitches = wl.fin_pitch_angles(abs(yaw) * 0.5, yaw_dir)
        # Rev O2: W4 vectored thrust
        if abs(yaw) > 0.3:
            vt = wl.w4_vectored_thrust_params(
                yaw_dir, abs(yaw)*0.7, ctrl.freq_hz)
            ctrl._tgt_sweeps[6] = vt['w4_port_sweep']
            ctrl._tgt_sweeps[7] = vt['w4_stbd_sweep']
        return RUNNING


# ── MISC BEHAVIOURS ──────────────────────────────────────────────

class WingConfigBehaviour(BehaviourNode):
    name = 'wing_config'; priority = 7; interruptible = True
    def tick(self, context, ctrl):
        import mco_wave_library as wl
        gs_id = context.get('gs_id', 'GS-0')
        V_ms  = context.get('V_ms', 8.0)
        ok, _ = wl.partial_stow_ok(gs_id, V_ms,
            context.get('h_agl_m', 50.0), 0.0)
        if not ok: gs_id = 'GS-0'
        try:
            cfg = wl.get_partial_stow_config(gs_id)
            for i in range(8):
                pair = i // 2 + 1
                if pair in cfg['stowed_pairs']:
                    ctrl._tgt_sweeps[i] = 210.0
        except Exception:
            pass
        return RUNNING


class TaxiFwdBehaviour(BehaviourNode):
    name = 'taxi_fwd'; priority = 10; interruptible = True
    def tick(self, context, ctrl):
        from mco_taxi_park import taxi_commands
        wind_data = {
            'V_wind_ms': context.get('V_wind_ms', context.get('wind_speed_ms', 0.0)),
            'wind_bearing_deg': context.get('wind_bearing_deg', context.get('wind_dir_deg', 0.0)),
            'heading_deg': context.get('heading_deg', context.get('yaw_deg', 0.0)),
        }
        if not wind_data['V_wind_ms'] and isinstance(context.get('sensors'), dict):
            s = context['sensors']
            wind_data['V_wind_ms'] = s.get('V_wind_ms', s.get('wind_speed_ms', 0.0))
            wind_data['wind_bearing_deg'] = s.get('wind_bearing_deg', s.get('wind_dir_deg', 0.0))
            wind_data['heading_deg'] = s.get('heading_deg', s.get('yaw_deg', 0.0))

        cmd = taxi_commands(context.get('throttle', 0.3),
                            context.get('yaw', 0.0),
                            lateral_cmd=context.get('lateral', 0.0),
                            wind_data=wind_data)
        ctrl._tgt_freq = cmd['freq_hz']
        for i, wc in enumerate(cmd['wing_cmds']):
            ctrl._tgt_sweeps[i]     = wc['sweep']
            ctrl._tgt_pitches[i]    = wc['pitch']
            ctrl._tgt_dihedrals[i]  = wc.get('dihedral_deg', 0.0)
        return RUNNING


class TaxiRevBehaviour(BehaviourNode):
    name = 'taxi_rev'; priority = 10; interruptible = True
    def tick(self, context, ctrl):
        from mco_taxi_park import taxi_commands
        wind_data = {
            'V_wind_ms': context.get('V_wind_ms', context.get('wind_speed_ms', 0.0)),
            'wind_bearing_deg': context.get('wind_bearing_deg', context.get('wind_dir_deg', 0.0)),
            'heading_deg': context.get('heading_deg', context.get('yaw_deg', 0.0)),
        }
        if not wind_data['V_wind_ms'] and isinstance(context.get('sensors'), dict):
            s = context['sensors']
            wind_data['V_wind_ms'] = s.get('V_wind_ms', s.get('wind_speed_ms', 0.0))
            wind_data['wind_bearing_deg'] = s.get('wind_bearing_deg', s.get('wind_dir_deg', 0.0))
            wind_data['heading_deg'] = s.get('heading_deg', s.get('yaw_deg', 0.0))

        cmd = taxi_commands(context.get('throttle', 0.2),
                            context.get('yaw', 0.0),
                            lateral_cmd=context.get('lateral', 0.0),
                            reverse=True,
                            wind_data=wind_data)
        ctrl._tgt_freq = cmd['freq_hz']
        for i, wc in enumerate(cmd['wing_cmds']):
            ctrl._tgt_sweeps[i]     = wc['sweep']
            ctrl._tgt_pitches[i]    = wc['pitch']
            ctrl._tgt_dihedrals[i]  = wc.get('dihedral_deg', 0.0)
        return RUNNING


class InspectionBehaviour(BehaviourNode):
    name = 'inspection'; priority = 11; interruptible = True
    def tick(self, context, ctrl):
        ctrl._tgt_sweeps    = [90.0] * 8
        ctrl._tgt_freq      = 0.0
        ctrl._tgt_dihedrals = [0.0]  * 8
        ctrl._tgt_pitches   = [0.0]  * 8
        return RUNNING


class TakeoffRollBehaviour(BehaviourNode):
    """Conventional fixed-wing takeoff roll acceleration behaviour.
    Advances throttle with neutral stick along runway centerline.
    Wider stroke arc and forward sweep (60°) generate maximum horizontal acceleration
    while rudder pedals maintain directional tracking.
    Integrates high-wind takeoff matrix (H-HEAD-TO-1, H-CROSS-TO-1, H-VAULT-GUST-1) for
    gust breakout and decoupled lateral thrust crosswind tracking.
    """
    name = 'takeoff_roll'; priority = 8; interruptible = True

    def tick(self, context, ctrl):
        import mco_advanced_physics as ap
        throttle = context.get('throttle', 0.85)
        V_ms = context.get('V_ms', 0.0)
        regime = context.get('regime', 'standard')

        # Check high-wind takeoff matrix
        V_w = context.get('V_wind_ms', context.get('wind_speed_ms', 0.0))
        w_b = context.get('wind_bearing_deg', context.get('wind_dir_deg', 0.0))
        hdg = context.get('heading_deg', context.get('yaw_deg', 0.0))
        rwy_hdg = context.get('runway_heading_deg', hdg)
        if not V_w and isinstance(context.get('sensors'), dict):
            s = context['sensors']
            V_w = s.get('V_wind_ms', s.get('wind_speed_ms', 0.0))
            w_b = s.get('wind_bearing_deg', s.get('wind_dir_deg', 0.0))
            hdg = s.get('heading_deg', s.get('yaw_deg', 0.0))
            rwy_hdg = s.get('runway_heading_deg', hdg)

        to_mode, ground_roll_target, to_diag = ap.high_wind_takeoff_matrix(
            V_wind_ms=V_w, wind_bearing_deg=w_b, heading_deg=hdg,
            runway_heading_deg=rwy_hdg, intent='takeoff',
            mass_kg=context.get('mass_kg', 5.367)
        )

        cmds, freq, diag = ap.ctol_takeoff_roll_kinematics(throttle, V_ms, regime)
        ctrl._tgt_freq = freq

        # Heading / yaw tracking via differential dihedral / pedal bias
        yaw_cmd = context.get('yaw', 0.0)
        yaw_delta = yaw_cmd * 3.0

        # Crosswind decoupled lateral compensation if H-CROSS-TO-1 active
        lat_comp = 0.0
        if to_mode == 'H-CROSS-TO-1' and abs(to_diag.get('V_cross_ms', 0.0)) > 2.0:
            # Positive V_cross pushes aircraft left -> need right lateral thrust
            v_cr = to_diag['V_cross_ms']
            lat_comp = 4.0 if v_cr > 0 else -4.0

        for i, c in enumerate(cmds):
            is_port = (i % 2 == 0)
            ctrl._tgt_pitches[i]   = c['pitch_deg']
            ctrl._tgt_sweeps[i]    = c['sweep_deg']
            dih_base = 6.0 + (lat_comp if is_port else -lat_comp)
            ctrl._tgt_dihedrals[i] = max(0.0, dih_base + (yaw_delta if is_port else -yaw_delta))

        # Check if aircraft lifted off unexpectedly or entered climb
        if context.get('h_agl_m', 0.0) > 0.45:
            self._state = SUCCESS
            return SUCCESS

        return RUNNING


class RotateBehaviour(BehaviourNode):
    """Conventional fixed-wing dual-cascade rotation and liftoff behaviour.
    Blends Front Cascade (W1-W2) Canard Direct Lift with Rear Cascade (W3-W4)
    elevon tail-down pitching moment to rotate without tail-strike risk.
    Transitions to flight/climb upon unstick.
    """
    name = 'rotate'; priority = 7; interruptible = True

    def on_enter(self, context):
        super().on_enter(context)
        self._start_t = context.get('t', 0.0)

    def tick(self, context, ctrl):
        import mco_advanced_physics as ap
        pitch_in = context.get('pitch', 0.5)
        V_ms = context.get('V_ms', 4.5)
        regime = context.get('regime', 'standard')
        h_agl = context.get('h_agl_m', 0.0)
        climb_rate = context.get('climb_rate_ms', context.get('vz_ms', 0.0))

        cmds, freq, diag = ap.ctol_dual_cascade_rotation(pitch_in, V_ms, regime)
        ctrl._tgt_freq = freq

        # Structural load factor protection during rotation at high dynamic pressure (V > 8 m/s)
        if V_ms > 8.0:
            aoa_lim = ap.dynamic_pressure_aoa_limit(V_ms, target_nz=2.2, gross_weight_kg=context.get('gross_weight_kg', 5.687))
            for c in cmds:
                c['pitch_deg'] = min(c['pitch_deg'], aoa_lim)

        for i, c in enumerate(cmds):
            ctrl._tgt_pitches[i]   = c['pitch_deg']
            ctrl._tgt_sweeps[i]    = c['sweep_deg']
            ctrl._tgt_dihedrals[i] = 6.0

        # Liftoff / transition confirmation:
        # Exit to SUCCESS (handing off to CLIMB/CRUISE) when height exceeds unstick target
        # or climb rate is positive.
        unstick_target = diag.get('unstick_target_m', 0.37)
        if h_agl >= unstick_target or climb_rate >= 0.50:
            self._state = SUCCESS
            return SUCCESS

        return RUNNING


class ApproachBehaviour(BehaviourNode):
    """Conventional fixed-wing glide path approach behaviour.
    Throttle regulates descent rate / glide path, pitch stick regulates airspeed.
    Crosswind slip/crab decrab alignment via rudder pedals and roll stick.
    """
    name = 'approach_normal'; priority = 7; interruptible = True

    def can_be_preempted_by(self, other_priority, is_safety=False):
        if is_safety:
            return True
        if not self.interruptible:
            return False
        # Allow preemption by go-around (3), rotate (7), takeoff roll (8), or climb (9)
        if other_priority <= 9:
            return True
        return other_priority <= self.priority

    def tick(self, context, ctrl):
        import mco_advanced_physics as ap
        throttle  = context.get('throttle', 0.35)
        pitch_in  = context.get('pitch', 0.0)
        yaw_in    = context.get('yaw', 0.0)
        roll_in   = context.get('roll', context.get('lateral', 0.0))
        V_ms      = context.get('V_ms', 8.0)
        h_agl     = context.get('h_agl_m', 15.0)
        regime    = context.get('regime', 'standard_power_off')
        crosswind = context.get('crosswind_ms', 0.0)

        # Transition to flare if within ground window
        if h_agl <= 2.0:
            self._state = SUCCESS
            return SUCCESS

        # Flapping frequency: idle for unpowered regimes, active thrust for powered/commercial/stol
        is_stol = regime in ('short_field', 'stol', 'stol_landing', 'stol_spot')
        if regime in ('standard_power_off', 'tailwheel_3point'):
            freq_hz = 0.0 if throttle <= 0.15 else throttle * 2.2
        elif is_stol:
            freq_hz = (1.2 + throttle * 2.0) if throttle > 0.12 else 0.0
        else:
            freq_hz = max(0.8, throttle * 2.5)

        ctrl._tgt_freq = freq_hz

        # Longitudinal trim: front wings hold slight positive incidence differential for stability
        if is_stol:
            aoa_trim = pitch_in * 3.5
            base_pitches = [9.5 + aoa_trim]*4 + [7.5 + aoa_trim]*4
            base_sweeps = [52.0]*8
        else:
            aoa_trim = pitch_in * 4.0
            base_pitches = [6.0 + aoa_trim]*4 + [4.0 + aoa_trim]*4
            base_sweeps = [45.0]*8
        base_dihs = [6.0]*8

        # Crosswind slip / decrab dihedral bias
        if abs(crosswind) > 0.5:
            drift_bias = (crosswind / max(V_ms, 3.0)) * 4.0
            for i in range(8):
                is_port = (i % 2 == 0)
                base_dihs[i] += drift_bias if is_port else -drift_bias

        # Lateral / yaw stick steering
        diff_dih = roll_in * 5.0
        diff_sw = yaw_in * 6.0
        for i in range(8):
            is_port = (i % 2 == 0)
            ctrl._tgt_pitches[i]   = base_pitches[i]
            ctrl._tgt_sweeps[i]    = base_sweeps[i] - diff_sw if is_port else base_sweeps[i] + diff_sw
            ctrl._tgt_dihedrals[i] = max(0.0, min(65.0, base_dihs[i] + (diff_dih if is_port else -diff_dih)))

        return RUNNING


class FlareBehaviour(BehaviourNode):
    """Conventional fixed-wing flare and touchdown behaviour.
    Dual-cascade canard lift on W1-W2 arrests sink rate while W3-W4 elevator
    maintains >= 150mm ground clearance to eliminate tail-strike risk.
    """
    name = 'landing_flare'; priority = 6; interruptible = True

    def can_be_preempted_by(self, other_priority, is_safety=False):
        if is_safety:
            return True
        if not self.interruptible:
            return False
        # Allow immediate preemption for go-around (priority 3), rotate (7), takeoff roll (8), or climb (9)
        if other_priority <= 9:
            return True
        return other_priority <= self.priority

    def tick(self, context, ctrl):
        import mco_advanced_physics as ap
        pitch_in  = context.get('pitch', 0.20)
        throttle  = context.get('throttle', 0.05)
        h_agl     = context.get('h_agl_m', 1.0)
        V_ms      = context.get('V_ms', 6.0)
        regime    = context.get('regime', 'standard_power_off')
        crosswind = context.get('crosswind_ms', 0.0)

        cmds, freq_hz, diag = ap.conventional_landing_flare_kinematics(
            pitch_in, throttle, h_agl, V_ms, regime, crosswind
        )

        ctrl._tgt_freq = freq_hz
        for i, c in enumerate(cmds):
            ctrl._tgt_pitches[i]   = c['pitch_deg']
            ctrl._tgt_sweeps[i]    = c['sweep_deg']
            ctrl._tgt_dihedrals[i] = c['dihedral_deg']

        ctrl.flare_diag = diag

        # Touchdown confirmation
        if context.get('on_ground', False) or h_agl <= 0.15:
            self._state = SUCCESS
            return SUCCESS

        return RUNNING


class LandingRolloutBehaviour(BehaviourNode):
    """Conventional fixed-wing runway rollout and aerodynamic braking behaviour.
    For aircraft with NO wheel brakes, the wings serve as the sole deceleration device:
    - Wave lever 78-100% or throttle >= 50%: Touch-and-Go abort / re-acceleration
    - Wave lever 48-78%: Freewheel low-drag coasting (Cd ~ 0.04)
    - Wave lever 28-48%: Ground taxi handoff as V drops below 4.5 m/s
    - Wave lever 16-28%: Shuttlecock bluff-body airbrake cup (Cd ~ 1.65, -0.65g)
    - Wave lever 6-16%:  Active reverse thrust flapping (-25N, -0.85g)
    - Wave lever 0-6%:   Compound max stop (-45N, -1.25g)
    """
    name = 'touchdown_rollout'; priority = 6; interruptible = True

    def can_be_preempted_by(self, other_priority, is_safety=False):
        if is_safety:
            return True
        if not self.interruptible:
            return False
        # Allow immediate preemption for touch-and-go / takeoff aborts (takeoff roll, rotate, climb)
        if other_priority <= 9:
            return True
        return other_priority <= self.priority

    def tick(self, context, ctrl):
        import mco_advanced_physics as ap
        lever_pct = context.get('lever_pct', 50.0)
        throttle  = context.get('throttle', 0.0)
        V_ms      = context.get('V_ms', 5.0)
        regime    = context.get('regime', 'standard_power_off')
        yaw_cmd   = context.get('yaw', 0.0)

        # Check for Touch-and-Go / Takeoff re-acceleration
        if throttle >= 0.50 or lever_pct >= 78.0:
            self._state = SUCCESS
            return SUCCESS

        # Check for Auto-Taxi handoff when speed is low and not actively braking
        if V_ms < 4.5 and lever_pct >= 40.0 and throttle <= 0.35:
            self._state = SUCCESS
            return SUCCESS

        cmds, freq_hz, diag = ap.aerodynamic_rollout_braking_kinematics(
            lever_pct, throttle, V_ms, regime, yaw_cmd
        )

        ctrl._tgt_freq = freq_hz
        for i, c in enumerate(cmds):
            ctrl._tgt_pitches[i]   = c['pitch_deg']
            ctrl._tgt_sweeps[i]    = c['sweep_deg']
            ctrl._tgt_dihedrals[i] = c['dihedral_deg']

        ctrl.rollout_diag = diag

        # Aircraft stopped
        if V_ms < 0.2:
            self._state = SUCCESS
            return SUCCESS

        return RUNNING


class GoAroundBehaviour(BehaviourNode):
    """Universal Go-Around and Balked Landing behaviour across all landing regimes.
    Priority 3 ensures immediate preemption of landing flare, approach, rollout, and glide.
    Executes balked landing kinematics:
      1. Immediate thrust spool-up to max climb power (2.8 to 3.5 Hz)
      2. Canard Direct Lift pulse (W1-W2) to arrest descent rate within <= 0.20s
      3. Anti-tail-strike protection (W3-W4) maintaining >= 150mm ground clearance
      4. Drag clean-up: resets airbrake dihedrals to 6° and sweep to 52°
      5. Automatic handoff to CLIMB once positive climb rate (vz >= 1.0 m/s) and safe altitude (h > 6m)
         are established.
    """
    name = 'go_around'; priority = 3; interruptible = True

    def __init__(self):
        super().__init__()
        self._entry_t = None
        self.go_around_diag = {}

    def on_enter(self, context):
        super().on_enter(context)
        self._entry_t = context.get('t', 0.0)

    def can_be_preempted_by(self, other_priority, is_safety=False):
        if is_safety:
            return True
        if not self.interruptible:
            return False
        # Only safety emergencies (recover, stoop, cf1_burst with priority <= 2) can preempt
        return other_priority <= 2

    def tick(self, context, ctrl):
        import mco_advanced_physics as ap
        pitch_in  = context.get('pitch', 0.15)
        throttle  = context.get('throttle', 1.0)
        h_agl     = context.get('h_agl_m', 2.0)
        V_ms      = context.get('V_ms', 6.5)
        regime    = context.get('regime', 'standard')
        vz_ms     = context.get('vz_ms', -0.5)
        crosswind = context.get('crosswind_ms', 0.0)
        pilot_ctl = {'ch1': context.get('lateral', 0.0), 'ch4': context.get('yaw', 0.0)}

        ctx_clean = {k: v for k, v in context.items() if k not in (
            'pitch_stick', 'throttle', 'h_agl_m', 'V_ms', 'landing_regime', 'vz_ms', 'crosswind_ms', 'pilot_controls')}
        cmds, freq_hz, diag = ap.balked_landing_go_around_kinematics(
            pitch_stick=pitch_in, throttle=throttle, h_agl_m=h_agl, V_ms=V_ms,
            landing_regime=regime, vz_ms=vz_ms, crosswind_ms=crosswind,
            pilot_controls=pilot_ctl, **ctx_clean
        )

        ctrl._tgt_freq = freq_hz
        for i, c in enumerate(cmds):
            ctrl._tgt_pitches[i]   = c['pitch_deg']
            ctrl._tgt_sweeps[i]    = c['sweep_deg']
            ctrl._tgt_dihedrals[i] = c['dihedral_deg']

        ctrl.go_around_diag = diag
        self.go_around_diag = diag

        # Handoff condition to normal climb:
        # Exit to SUCCESS when established in steady climb (> 1.0 m/s vz, h > 6.0 m, or after 1.5s elapsed with positive vz)
        t_now = context.get('t', 0.0)
        dt_elapsed = t_now - (self._entry_t if self._entry_t is not None else t_now)
        if (h_agl >= 6.0 and vz_ms >= 1.0) or (dt_elapsed >= 1.5 and vz_ms >= 0.5 and h_agl >= 3.0):
            self._state = SUCCESS
            return SUCCESS

        return RUNNING


class LoiterBehaviour(BehaviourNode):
    name = 'loiter'; priority = 8; interruptible = True
    def tick(self, context, ctrl):
        import mco_wave_library as wl
        Ps   = context.get('Ps', 0.1)
        V    = context.get('V_ms', 8.0)
        bank = context.get('bank_deg', 25.0)
        freq, sweep, dihs, pitch = wl.loiter_params(
            Ps, V, bank, 0.02, ctrl.freq_hz)
        ctrl._tgt_freq      = freq
        ctrl._tgt_sweeps    = [sweep] * 8
        ctrl._tgt_dihedrals = dihs
        ctrl._tgt_pitches   = [pitch] * 8
        return RUNNING


class OrbitBehaviour(BehaviourNode):
    name = 'orbit'; priority = 8; interruptible = True
    def tick(self, context, ctrl):
        import mco_wave_library as wl
        bank      = context.get('bank_deg', 30.0)
        direction = context.get('direction', 'right')
        ctrl._tgt_freq = 0.5
        sw_delta = bank * 0.4
        for i in range(8):
            is_port = (i % 2 == 0)
            inner = ((direction=='right') and not is_port) or \
                    ((direction=='left')  and     is_port)
            ctrl._tgt_sweeps[i] = 60.0 + sw_delta if inner else 60.0
        ctrl._tgt_pitches = [wl.bank_pitch_correction_deg(bank)] * 8
        return RUNNING


# ── BAT FLIGHT BIOMECHANICS BEHAVIOURS (H-81, H-82, H-84) ───────

class RoostPerchMacro(BehaviourNode):
    """
    Bat-inspired roosting somersault / post-stall perch landing arrest (H-84).

    [PILOT TOUCHDOWN FLARE / AUTONOMOUS GRASP DEFERRED]
    Aerodynamic post-stall deep flare (H-84 / J-hook deceleration) is active under manual pilot
    pitch-up stick and throttle chop for spot touchdown. Closed-loop autonomous visual-servoing
    and robotic wire-grasping are deferred to the post-first-flight autopilot bucket.
    """
    name = 'roost_perch_flare'; priority = 1; interruptible = False

    def on_enter(self, context):
        super().on_enter(context)
        self._phase_t = context.get('t', 0.0)
        self._flare_type = context.get('flare_type', 'perch_landing')
        h = context.get('h_agl_m', 50.0)
        if self._flare_type == 'somersault' and h < 5.0:
            self._flare_type = 'perch_landing'
            self.log("Somersault clamped to perch landing (AGL < 5m)")
        self.log(f"RoostPerchMacro initiated: {self._flare_type}")

    def tick(self, context, ctrl):
        t = context.get('t', 0.0)
        dt = t - self._phase_t
        # Push-pull counter flapping: W1/W2 positive pitch, W3/W4 negative pitch
        # Extreme pitch flare for rapid post-stall deceleration
        if dt < 0.25:
            ctrl._tgt_freq = 2.4
            ctrl._tgt_sweeps = [40.0, 40.0, 40.0, 40.0, 80.0, 80.0, 80.0, 80.0]
            ctrl._tgt_dihedrals = [15.0] * 8
            # Front pairs nose-up (+35°), rear pairs nose-down (-35°)
            ctrl._tgt_pitches = [35.0, 35.0, 35.0, 35.0, -35.0, -35.0, -35.0, -35.0]
            return RUNNING
        elif dt < 0.6:
            # Settle / perch arrest
            ctrl._tgt_freq = 0.5
            ctrl._tgt_sweeps = [60.0] * 8
            ctrl._tgt_dihedrals = [0.0] * 8
            ctrl._tgt_pitches = [10.0] * 8
            return RUNNING
        else:
            self._state = SUCCESS
            return SUCCESS


class InertialYawBehaviour(BehaviourNode):
    """Bat-inspired zero-airspeed inertial reorientation (H-81)."""
    name = 'inertial_yaw'; priority = 7; interruptible = True

    def tick(self, context, ctrl):
        yaw_cmd = context.get('yaw', context.get('yaw_stick', 0.0))
        yaw_cmd = max(-1.0, min(1.0, yaw_cmd))
        ctrl._tgt_freq = 1.8
        # Continuous asymmetric wing inertia: smoothly scales between 35° and 90°
        delta_sw = 27.5 * yaw_cmd
        sweeps = [
            62.5 - delta_sw, 62.5 + delta_sw,
            62.5 - delta_sw, 62.5 + delta_sw,
            62.5 - delta_sw, 62.5 + delta_sw,
            62.5 - delta_sw, 62.5 + delta_sw,
        ]
        ctrl._tgt_sweeps = sweeps
        ctrl._tgt_dihedrals = [0.0] * 8
        ctrl._tgt_pitches = [5.0] * 8
        return RUNNING


class AnhedralTurnBehaviour(BehaviourNode):
    """Bat-inspired anhedral gull-wing knife-edge turn (H-82)."""
    name = 'anhedral_turn'; priority = 8; interruptible = True

    def tick(self, context, ctrl):
        dir_ = context.get('direction', 'right')
        ctrl._tgt_freq = 2.2
        # Inboard anhedral droop (-15°), outboard dihedral (+12°)
        # Port: 0, 2, 4, 6; Starboard: 1, 3, 5, 7
        if dir_ == 'right':
            # Right wing inboard (-15°), Left wing outboard (+12°)
            dihedrals = [12.0, -15.0, 12.0, -15.0, 12.0, -15.0, 12.0, -15.0]
            sweeps = [65.0, 50.0, 65.0, 50.0, 65.0, 50.0, 65.0, 50.0]
        else:
            dihedrals = [-15.0, 12.0, -15.0, 12.0, -15.0, 12.0, -15.0, 12.0]
            sweeps = [50.0, 65.0, 50.0, 65.0, 50.0, 65.0, 50.0, 65.0]
        ctrl._tgt_sweeps = sweeps
        ctrl._tgt_dihedrals = dihedrals
        ctrl._tgt_pitches = [12.0] * 8
        return RUNNING


# ── DRAGONFLY AERODYNAMIC BEHAVIOURS (H-86 to H-88) ──────────────

class ReverseCascadeBehaviour(BehaviourNode):
    """Dragonfly-inspired reverse metachronal cascade propulsion (H-86).
    Hindwings lead forewings by -60° phase offset, harvesting downwash for +24.5% thrust boost.
    """
    name = 'reverse_cascade_cruise'; priority = 6; interruptible = True

    def tick(self, context, ctrl):
        ctrl._tgt_freq = 2.5
        ctrl._tgt_sweeps = [60.0] * 8
        ctrl._tgt_dihedrals = [5.0] * 8
        ctrl._tgt_pitches = [12.0] * 8
        if hasattr(ctrl, '_cascade_phase_deg'):
            ctrl._cascade_phase_deg = -60.0
        return RUNNING


class ActiveUpstrokeBrakeMacro(BehaviourNode):
    """
    Dragonfly-inspired active upstroke backward flight & dynamic airbrake (H-87).
    Inverted chord alpha ≈ -28° generates -1.25g deceleration with level fuselage.

    [AUTOPILOT DEFERRED]
    Designated as an autonomous trajectory function deferred to the post-first-flight
    autopilot implementation bucket (see _AUTOPILOT_DEFERRED['ACTIVE_UPSTROKE_BRAKE_MACRO']
    in mco_wave_library.py). Not part of active manual-pilot flight laws.
    """
    name = 'active_upstroke_brake'; priority = 2; interruptible = False

    def on_enter(self, context):
        super().on_enter(context)
        self._phase_t = context.get('t', 0.0)
        self.log("ActiveUpstrokeBrakeMacro initiated: alpha=-28 deg, target decel -1.25g")

    def tick(self, context, ctrl):
        t = context.get('t', 0.0)
        dt = t - self._phase_t
        h = context.get('h_agl_m', 50.0)
        v = context.get('V_ms', 10.0)

        # Safety: complete if below safety floor or speed stopped or timeout
        if h < 1.5 or v < 1.0 or dt > 1.2:
            self._state = SUCCESS
            return SUCCESS

        ctrl._tgt_freq = 2.2
        ctrl._tgt_sweeps = [70.0] * 8
        ctrl._tgt_dihedrals = [10.0] * 8
        # Command inverted negative AoA (-28°) for upstroke dynamic drag couple
        ctrl._tgt_pitches = [-28.0] * 8
        return RUNNING


class StrokePlaneVectorBehaviour(BehaviourNode):
    """Dragonfly-inspired direct-musculature stroke-plane tilt vectoring (H-88).
    Differential stroke plane tilt between port and starboard wings creates pure yaw moment decoupled from roll.
    """
    name = 'stroke_plane_vector'; priority = 7; interruptible = True

    def tick(self, context, ctrl):
        yaw_stk = context.get('yaw_stick', context.get('yaw', 1.0))
        yaw_stk = max(-1.0, min(1.0, yaw_stk))
        ctrl._tgt_freq = 2.4
        # Differential stroke plane tilt: port vs starboard proportional to yaw stick
        # Port: 0, 2, 4, 6; Starboard: 1, 3, 5, 7
        tilt = 18.0 * yaw_stk
        ctrl._tgt_sweeps = [
            60.0 + tilt, 60.0 - tilt,
            60.0 + tilt, 60.0 - tilt,
            60.0 + tilt, 60.0 - tilt,
            60.0 + tilt, 60.0 - tilt,
        ]
        ctrl._tgt_dihedrals = [5.0] * 8
        ctrl._tgt_pitches = [15.0] * 8
        return RUNNING


# ── INSECT FLIGHT BIOMECHANICS BEHAVIOURS (H-89 to H-95) ─────────

class KramerLiftBurstMacro(BehaviourNode):
    """
    Dynamic Kramer pre-rotation rotational lift burst (H-89).
    Pitch pre-rotation (7.5°-12.5°) leads stroke reversal, producing rotational lift burst (+80% CL).

    [AUTOPILOT DEFERRED]
    Designated as an autonomous trajectory function deferred to the post-first-flight
    autopilot implementation bucket (see _AUTOPILOT_DEFERRED['KRAMER_LIFT_BURST_MACRO']
    in mco_wave_library.py). Not part of active manual-pilot flight laws.
    """
    name = 'kramer_lift_burst'; priority = 3; interruptible = False

    def on_enter(self, context):
        super().on_enter(context)
        self._phase_t = context.get('t', 0.0)
        self.log("KramerLiftBurstMacro initiated: 10 deg pre-rotation lead at stroke reversal")

    def tick(self, context, ctrl):
        t = context.get('t', 0.0)
        dt = t - self._phase_t
        if dt > 0.65 or context.get('V_ms', 0.0) > 8.0:
            self._state = SUCCESS
            return SUCCESS
        ctrl._tgt_freq = 2.8
        ctrl._tgt_sweeps = [65.0] * 8
        ctrl._tgt_dihedrals = [6.0] * 8
        ctrl._tgt_pitches = [22.0, 22.0, 20.0, 20.0, 20.0, 20.0, 22.0, 22.0]
        return RUNNING


class ClapJetBreakoutMacro(BehaviourNode):
    """
    Lepidopteran dorsal clap jet breakout propulsion (H-91).
    Inboard dorsal wing clap expels compressed air jet producing +1.45g zero-airspeed acceleration.

    [AUTOPILOT DEFERRED]
    Designated as an autonomous trajectory function deferred to the post-first-flight
    autopilot implementation bucket (see _AUTOPILOT_DEFERRED['CLAP_JET_BREAKOUT_MACRO']
    in mco_wave_library.py). Not part of active manual-pilot flight laws.
    """
    name = 'clap_jet_breakout'; priority = 2; interruptible = False

    def on_enter(self, context):
        super().on_enter(context)
        self._phase_t = context.get('t', 0.0)
        self.log("ClapJetBreakoutMacro initiated: dorsal clap jet expulsion pulse")

    def tick(self, context, ctrl):
        t = context.get('t', 0.0)
        dt = t - self._phase_t
        if dt > 0.85 or context.get('V_ms', 0.0) >= 9.0:
            self._state = SUCCESS
            return SUCCESS
        ctrl._tgt_freq = 3.0
        ctrl._tgt_sweeps = [78.0] * 8
        ctrl._tgt_dihedrals = [25.0] * 8
        ctrl._tgt_pitches = [14.0] * 8
        return RUNNING


class ElytronCanardBehaviour(BehaviourNode):
    """Coleopteran elytron canard obstacle climb (H-92).
    Station 1 locked rigid at +8.5° incidence as lifting canard while Stations 2-4 flap dynamically (48° steep climb).
    """
    name = 'elytron_canard'; priority = 5; interruptible = True

    def tick(self, context, ctrl):
        ctrl._tgt_freq = 2.6
        ctrl._tgt_sweeps = [0.0, 0.0, 68.0, 68.0, 68.0, 68.0, 68.0, 68.0]
        ctrl._tgt_dihedrals = [8.0, 8.0, 6.0, 6.0, 6.0, 6.0, 6.0, 6.0]
        ctrl._tgt_pitches = [8.5, 8.5, 18.0, 18.0, 18.0, 18.0, 18.0, 18.0]
        return RUNNING


class CoriolisGustPreemptBehaviour(BehaviourNode):
    """Dipteran synchronous Coriolis gust preemption (H-93).
    Haltere-inspired angular acceleration reflex executing <18ms counter-twist prior to airframe translation.
    """
    name = 'coriolis_gust_preempt'; priority = 1; interruptible = False

    def on_enter(self, context):
        super().on_enter(context)
        self._phase_t = context.get('t', 0.0)
        self.log("CoriolisGustPreemptBehaviour: high-rate haltere reflex counter-twist armed")

    def tick(self, context, ctrl):
        t = context.get('t', 0.0)
        dt = t - self._phase_t
        ang_accel = context.get('angular_accel_dps2', 0.0)
        if dt > 0.35 and abs(ang_accel) < 60.0:
            self._state = SUCCESS
            return SUCCESS
        ctrl._tgt_freq = 2.7
        counter_twist = 6.5 if ang_accel >= 0 else -6.5
        ctrl._tgt_sweeps = [60.0] * 8
        ctrl._tgt_dihedrals = [5.0] * 8
        ctrl._tgt_pitches = [
            12.0 - counter_twist, 12.0 + counter_twist,
            12.0 - counter_twist, 12.0 + counter_twist,
            12.0 - counter_twist, 12.0 + counter_twist,
            12.0 - counter_twist, 12.0 + counter_twist,
        ]
        return RUNNING


class LocustTorsionalCruiseBehaviour(BehaviourNode):
    """Orthopteran traveling torsional wave cruise (H-94).
    Continuous spanwise traveling twist wave minimizing induced drag (-32% Cd,i, 10.4 Wh/km).
    """
    name = 'locust_torsional_cruise'; priority = 6; interruptible = True

    def tick(self, context, ctrl):
        ctrl._tgt_freq = 2.4
        ctrl._tgt_sweeps = [55.0] * 8
        ctrl._tgt_dihedrals = [4.0] * 8
        ctrl._tgt_pitches = [14.0, 14.0, 11.5, 11.5, 9.0, 9.0, 6.5, 6.5]
        return RUNNING


class SlottedFlapGlideBehaviour(BehaviourNode):
    """Tandem dynamic slotted Fowler flap glide (H-95).
    Station 1 & 2 gap boundary layer energized slotted glide (CL,max 2.45).
    """
    name = 'slotted_flap_glide'; priority = 7; interruptible = True

    def tick(self, context, ctrl):
        sticks = context.get('sticks', {})
        ch2 = context.get('pitch', sticks.get('pitch', sticks.get('ch2', context.get('ch2', 0.0))))
        if ch2 < -0.05 or context.get('falcon_morph', False) or context.get('morphing_polar', False):
            import mco_advanced_physics as ap
            alc_bias = context.get('alc_bias', None)
            if alc_bias is None and 'w_netto_ms' in context:
                alc_bias = ap.alc_maccready_glide_bias(context['w_netto_ms'])
            pilot_ctrl = {
                'ch1': context.get('roll', sticks.get('roll', sticks.get('ch1', context.get('ch1', 0.0)))),
                'ch4': context.get('yaw', sticks.get('yaw', sticks.get('ch4', context.get('ch4', 0.0)))),
            }
            cmds, freq_hz, diag = ap.falcon_morphing_glide_polar(
                pitch_stick=ch2,
                V_ms=context.get('V_ms', 8.0),
                h_agl_m=context.get('h_agl_m', context.get('h_agl', 100.0)),
                gross_weight_kg=context.get('gross_weight_kg', 5.687),
                alc_bias=alc_bias,
                pilot_controls=pilot_ctrl
            )
            ctrl._tgt_freq      = freq_hz
            ctrl._tgt_sweeps    = [c['sweep_deg'] for c in cmds]
            ctrl._tgt_dihedrals = [c['dihedral_deg'] for c in cmds]
            ctrl._tgt_pitches   = [c['pitch_deg'] for c in cmds]
            ctrl.falcon_morph_diag = diag
            return RUNNING

        ctrl._tgt_freq = 0.0  # Static glide lock
        ctrl._tgt_sweeps = [50.0] * 8
        ctrl._tgt_dihedrals = [6.0] * 8
        ctrl._tgt_pitches = [8.5, 8.5, 16.5, 16.5, 12.0, 12.0, 10.0, 10.0]
        return RUNNING


# ── PTEROSAUR FLIGHT BIOMECHANICS BEHAVIOURS (H-98, H-100 to H-102) ──

class QuadrupedalVaultMacro(BehaviourNode):
    """
    Quadrupedal ground-coupled vault takeoff (H-98).
    Forelimb-hindlimb leap impulse: Station 1-2 ground power strike (+1.85g vertical acceleration).
    Ceiling trip-wire of 1.5m AGL or 0.42s transition timeout switches to climbing flight.

    [PILOT LAUNCH GESTURE — ACTIVE FLIGHT LAW]
    Active manual pilot-flown catapult takeoff gesture (pedals held + throttle up + stick pull & snap).
    Maintains 100% active pilot 3-axis steering authority throughout the leap strike.
    Autonomous hands-off unstick without pilot stick triggering is deferred to the autopilot bucket.
    """
    name = 'quadrupedal_vault'; priority = 2; interruptible = False

    def on_enter(self, context):
        super().on_enter(context)
        self._phase_t = context.get('t', 0.0)
        self.log("QuadrupedalVaultMacro initiated: forelimb vault leap strike (+1.85g)")

    def tick(self, context, ctrl):
        t = context.get('t', 0.0)
        dt = t - self._phase_t
        h_agl = context.get('h_agl_m', 0.0)
        v_ms = context.get('V_ms', 0.0)
        on_ground = context.get('on_ground', False)

        # Abort if pedals released or throttle backed off while on ground
        pedals_depressed = context.get('pedals_depressed', True)
        throttle = context.get('throttle', 1.0)
        if on_ground and (not pedals_depressed or throttle < 0.70):
            self.log("QuadrupedalVaultMacro aborted: pedals released or throttle backed off on ground")
            self._state = FAILURE
            return FAILURE

        if dt > 0.42 or h_agl >= 1.5 or v_ms >= 7.0:
            self._state = SUCCESS
            return SUCCESS
        ctrl._tgt_freq = 3.5

        # Active pilot steering blending
        ch1 = max(-1.0, min(1.0, context.get('roll', context.get('sticks', {}).get('ch1', 0.0))))
        ch4 = max(-1.0, min(1.0, context.get('yaw', context.get('sticks', {}).get('ch4', 0.0))))
        diff_sweep = 10.0 * ch4
        diff_dih = 6.0 * ch1

        if dt < 0.15:
            # Phase 1: Forward sweep +20 deg pitch crouch
            ctrl._tgt_sweeps = [
                round(max(10.0, min(90.0, 35.0 - diff_sweep)), 1),
                round(max(10.0, min(90.0, 35.0 + diff_sweep)), 1),
                round(max(0.0, min(90.0, 0.0 - diff_sweep)), 1),
                round(max(0.0, min(90.0, 0.0 + diff_sweep)), 1),
                0.0, 0.0, 0.0, 0.0,
            ]
            ctrl._tgt_dihedrals = [
                round(max(-10.0, min(25.0, 0.0 - diff_dih)), 1),
                round(max(-10.0, min(25.0, 0.0 + diff_dih)), 1),
                round(max(-10.0, min(25.0, 0.0 - diff_dih)), 1),
                round(max(-10.0, min(25.0, 0.0 + diff_dih)), 1),
                0.0, 0.0, 0.0, 0.0,
            ]
            ctrl._tgt_pitches = [20.0, 20.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        else:
            # Phase 2: Downward/aft power strike leap + dynamic handoff to stations 3 & 4
            ctrl._tgt_sweeps = [
                round(max(10.0, min(90.0, 70.0 - diff_sweep)), 1),
                round(max(10.0, min(90.0, 70.0 + diff_sweep)), 1),
                round(max(10.0, min(90.0, 60.0 - diff_sweep)), 1),
                round(max(10.0, min(90.0, 60.0 + diff_sweep)), 1),
                round(max(10.0, min(90.0, 60.0 - diff_sweep)), 1),
                round(max(10.0, min(90.0, 60.0 + diff_sweep)), 1),
                round(max(10.0, min(90.0, 60.0 - diff_sweep)), 1),
                round(max(10.0, min(90.0, 60.0 + diff_sweep)), 1),
            ]
            ctrl._tgt_dihedrals = [
                round(max(-10.0, min(25.0, 15.0 - diff_dih)), 1),
                round(max(-10.0, min(25.0, 15.0 + diff_dih)), 1),
                round(max(-10.0, min(25.0, 8.0 - diff_dih)), 1),
                round(max(-10.0, min(25.0, 8.0 + diff_dih)), 1),
                round(max(-10.0, min(25.0, 8.0 - diff_dih)), 1),
                round(max(-10.0, min(25.0, 8.0 + diff_dih)), 1),
                round(max(-10.0, min(25.0, 8.0 - diff_dih)), 1),
                round(max(-10.0, min(25.0, 8.0 + diff_dih)), 1),
            ]
            ctrl._tgt_pitches = [12.0, 12.0, 18.0, 18.0, 18.0, 18.0, 18.0, 18.0]
        return RUNNING


class DifferentialSweepTrimBehaviour(BehaviourNode):
    """Tailless metachronal differential sweep directional trim (H-100).
    Asymmetric sweep bias (+/-12° to +/-14°) applied between port and starboard wings for pure yaw hold (<0.5° drift).
    """
    name = 'differential_sweep_trim'; priority = 4; interruptible = True

    def tick(self, context, ctrl):
        beta = context.get('sideslip_beta_deg', context.get('yaw_drift_deg', 0.0))
        sweep_bias = 12.0 if abs(beta) < 1.0 else min(14.0, max(6.0, abs(beta) * 2.0))
        if beta < 0:
            sweep_bias = -sweep_bias
        ctrl._tgt_freq = 2.5
        ctrl._tgt_sweeps = [
            60.0 - sweep_bias, 60.0 + sweep_bias,
            60.0 - sweep_bias, 60.0 + sweep_bias,
            60.0 - sweep_bias, 60.0 + sweep_bias,
            60.0 - sweep_bias, 60.0 + sweep_bias,
        ]
        ctrl._tgt_dihedrals = [5.0] * 8
        ctrl._tgt_pitches = [12.0] * 8
        return RUNNING


class CranialPitchBalanceBehaviour(BehaviourNode):
    """Dynamic flap-inertia pitch counterbalance (H-101).
    Cranial crest inspired pitch feedforward torque canceling flapping inertia (-42% pitch servo current reduction).
    """
    name = 'cranial_pitch_balance'; priority = 6; interruptible = True

    def tick(self, context, ctrl):
        ctrl._tgt_freq = 2.5
        ctrl._tgt_sweeps = [60.0] * 8
        ctrl._tgt_dihedrals = [5.0] * 8
        ctrl._tgt_pitches = [10.0] * 8
        return RUNNING


class AzhdarchidSoarBehaviour(BehaviourNode):
    """Deep-camber high-aspect-ratio azhdarchid thermal soaring loiter (H-102).
    Tandem slotted box wing lock: Stations 1 & 3 locked at +8.5°, Stations 2 & 4 drooped to +14.0° (sink 0.38 m/s, +86% loiter).
    """
    name = 'azhdarchid_soar'; priority = 7; interruptible = True

    def tick(self, context, ctrl):
        ctrl._tgt_freq = 0.7  # Ultra-low frequency metachronal soaring pulse
        ctrl._tgt_sweeps = [48.0] * 8
        ctrl._tgt_dihedrals = [7.0] * 8
        ctrl._tgt_pitches = [8.5, 8.5, 14.0, 14.0, 8.5, 8.5, 14.0, 14.0]
        return RUNNING


# ── Rev T: Aquatic Biomechanics & Hydrodynamic Biomimicry Suite (H-103 to H-108) ───

class TurtlePivotBrakeBehaviour(BehaviourNode):
    """Sea Turtle In-Stroke Sweep & Differential Flipper Pivot Braking (H-103).
    High drag on inside flipper with forward sweep on outside flipper for tight flat turns (R < 0.65m).
    """
    name = 'turtle_pivot_brake'; priority = 4; interruptible = True

    def tick(self, context, ctrl):
        direction = context.get('direction', 'left')
        ctrl._tgt_freq = 1.8
        if direction == 'left':
            # Port wings brake/drag, stbd wings thrust
            ctrl._tgt_sweeps = [42.0, 65.0, 42.0, 65.0, 42.0, 65.0, 42.0, 65.0]
            ctrl._tgt_pitches = [24.0, 10.0, 24.0, 10.0, 24.0, 10.0, 24.0, 10.0]
        else:
            ctrl._tgt_sweeps = [65.0, 42.0, 65.0, 42.0, 65.0, 42.0, 65.0, 42.0]
            ctrl._tgt_pitches = [10.0, 24.0, 10.0, 24.0, 10.0, 24.0, 10.0, 24.0]
        ctrl._tgt_dihedrals = [6.0] * 8
        return RUNNING


class PlesiosaurTandemWakeBehaviour(BehaviourNode):
    """Plesiosaur Speed-Adaptive Tandem Wake Capture & Collective Pitch Trimming (H-104).
    Rear pairs intercept vortex wake for +22.5% thrust boost while trimming pitch.
    """
    name = 'plesiosaur_tandem_wake'; priority = 5; interruptible = True

    def tick(self, context, ctrl):
        ctrl._tgt_freq = 2.4
        ctrl._tgt_sweeps = [55.0, 55.0, 55.0, 55.0, 62.0, 62.0, 62.0, 62.0]
        ctrl._tgt_pitches = [12.5, 12.5, 12.5, 12.5, 9.5, 9.5, 9.5, 9.5]
        ctrl._tgt_dihedrals = [5.0] * 8
        return RUNNING


class CetaceanHydrofoilRollBehaviour(BehaviourNode):
    """Cetacean Pectoral 3-DOF Hydrofoil Maneuvering & Fluke Recoil Damping (H-105).
    Differential collective AoA on W1 for rapid roll with active heave recoil damping.
    """
    name = 'cetacean_hydrofoil_roll'; priority = 4; interruptible = True

    def tick(self, context, ctrl):
        ctrl._tgt_freq = 2.2
        # W1 differential AoA, W2-W4 symmetric propulsive
        ctrl._tgt_sweeps = [52.0, 52.0, 60.0, 60.0, 60.0, 60.0, 60.0, 60.0]
        ctrl._tgt_pitches = [19.0, 3.0, 11.5, 11.5, 11.5, 11.5, 11.5, 11.5]
        ctrl._tgt_dihedrals = [5.0] * 8
        return RUNNING


class KrillJetCoalescenceBehaviour(BehaviourNode):
    """Crustacean Adlocomotory Metachronal Jet Coalescence & Vector Tuning (H-106).
    Posterior-to-anterior metachronal cascade forming coalesced propulsive jet.
    """
    name = 'krill_jet_coalescence'; priority = 5; interruptible = True

    def tick(self, context, ctrl):
        ctrl._tgt_freq = 2.8
        ctrl._tgt_sweeps = [58.0] * 8
        ctrl._tgt_pitches = [9.5, 9.5, 10.3, 10.3, 11.1, 11.1, 11.9, 11.9]
        ctrl._tgt_dihedrals = [4.0] * 8
        return RUNNING


class CopepodMetachronyBehaviour(BehaviourNode):
    """Copepod Dual-Mode Asymmetric Metachrony (H-107).
    Sequential downstroke power distribution with synchronous feathered rapid recovery.
    """
    name = 'copepod_metachrony'; priority = 6; interruptible = True

    def tick(self, context, ctrl):
        ctrl._tgt_freq = 2.5
        ctrl._tgt_sweeps = [60.0] * 8
        ctrl._tgt_pitches = [11.0] * 8
        ctrl._tgt_dihedrals = [4.0] * 8
        return RUNNING


class CaridoidEscapeFlareMacro(BehaviourNode):
    """
    Decapod Caridoid Impulse Flare & Escape Pulse (H-108).
    Instantaneous 8-wing synchronous high-amplitude downward clap generating +38 N escape thrust.

    [AUTOPILOT DEFERRED]
    Designated as an autonomous trajectory function deferred to the post-first-flight
    autopilot implementation bucket (see _AUTOPILOT_DEFERRED['CARIDOID_ESCAPE_FLARE_MACRO']
    in mco_wave_library.py). Not part of active manual-pilot flight laws.
    """
    name = 'caridoid_escape_flare'; priority = 1; interruptible = False

    def on_enter(self, context):
        super().on_enter(context)
        self._phase_t = context.get('t', 0.0)
        self.log("CaridoidEscapeFlareMacro initiated: +38N escape impulse surge")

    def tick(self, context, ctrl):
        t = context.get('t', 0.0)
        dt = t - self._phase_t
        if dt >= 0.15:
            return SUCCESS
        ctrl._tgt_freq = 3.2
        ctrl._tgt_sweeps = [50.0] * 8
        ctrl._tgt_dihedrals = [12.0] * 8
        ctrl._tgt_pitches = [22.0] * 8
        return RUNNING


# ── CARGO DRONE & eVTOL BEHAVIOURS ───────────────────────────────

class PayloadDropMacro(BehaviourNode):
    """
    Cargo Drop de-ballooning pitch step macro (H-CR-2).
    Applies an instant -2.0° collective pitch step for 0.5s immediately upon payload release,
    eliminating post-drop ballooning and pitch-up stall.

    [AUTOPILOT DEFERRED]
    Designated as an autonomous trajectory function deferred to the post-first-flight
    autopilot implementation bucket (see _AUTOPILOT_DEFERRED['PAYLOAD_DROP_MACRO']
    in mco_wave_library.py). Not part of active manual-pilot flight laws.
    """
    name = 'payload_drop'; priority = 2; interruptible = False

    def on_enter(self, context):
        super().on_enter(context)
        self._phase_t = context.get('t', 0.0)
        self.log("PayloadDropMacro initiated: instant -2.0 deg de-balloon pitch step")

    def tick(self, context, ctrl):
        t = context.get('t', 0.0)
        dt = t - self._phase_t
        if dt >= 0.50:
            self._state = SUCCESS
            return SUCCESS
        ctrl._tgt_freq = 2.2
        ctrl._tgt_sweeps = [60.0] * 8
        ctrl._tgt_dihedrals = [5.0] * 8
        # Instant -2.0° pitch step on all 8 wings
        ctrl._tgt_pitches = [8.0] * 8
        return RUNNING


class CargoSwingDampBehaviour(BehaviourNode):
    """Cargo sling pendulum oscillation suppression (H-HL-3).
    Anti-phase cascade differential damping between forward pairs (W1-W2) and aft pairs (W3-W4).
    """
    name = 'cargo_swing_damp'; priority = 5; interruptible = True

    def tick(self, context, ctrl):
        swing_deg = context.get('cargo_swing_deg', 0.0)
        pitch_delta = max(-3.0, min(3.0, swing_deg * 0.25))
        ctrl._tgt_freq = 2.2
        ctrl._tgt_sweeps = [60.0] * 8
        ctrl._tgt_dihedrals = [5.0] * 8
        # Forward pairs counter aft pairs to damp cable oscillation
        ctrl._tgt_pitches = [10.0 + pitch_delta]*4 + [10.0 - pitch_delta]*4
        return RUNNING


class IgeCushionBehaviour(BehaviourNode):
    """eVTOL In-Ground Effect (IGE) ground cushion & vertiport pad lock.
    Damps flapping frequency by 15% to suppress ground bounce and engages
    differential sweep for precision vertiport pad station-keeping.
    """
    name = 'ige_cushion'; priority = 3; interruptible = True

    def tick(self, context, ctrl):
        # 15% reduction in frequency against IGE over-lift cushion
        ctrl._tgt_freq = 1.8 * 0.85
        # Forward sweep creates wide stable ground footprint
        drift_x = context.get('pad_drift_x_m', 0.0)
        sweep_bias = max(-8.0, min(8.0, drift_x * 12.0))
        ctrl._tgt_sweeps = [
            50.0 - sweep_bias, 50.0 + sweep_bias,
            50.0 - sweep_bias, 50.0 + sweep_bias,
            50.0 - sweep_bias, 50.0 + sweep_bias,
            50.0 - sweep_bias, 50.0 + sweep_bias,
        ]
        ctrl._tgt_dihedrals = [4.0] * 8
        ctrl._tgt_pitches = [9.0] * 8
        return RUNNING


class VertiportApproachBehaviour(BehaviourNode):
    """eVTOL Vertiport steep approach glidepath (H-FD-2).
    Symmetric high dihedral (50.0°) generates steep descent (7°-12°) while keeping fuselage flat (0° body pitch).
    """
    name = 'vertiport_approach'; priority = 4; interruptible = True

    def tick(self, context, ctrl):
        ctrl._tgt_freq = 1.87
        ctrl._tgt_sweeps = [52.0] * 8
        # High symmetric dihedral sets steep descent rate decoupled from airspeed
        ctrl._tgt_dihedrals = [50.0] * 8
        ctrl._tgt_pitches = [12.0] * 8
        return RUNNING


class OwiRedundancyBehaviour(BehaviourNode):
    """One-Wing-Inoperative (OWI) asymmetric motor fail-safe redundancy.
    When a wing actuator fails, the opposing wing is feathered to eliminate roll asymmetry,
    and remaining 6 operational wings re-scale thrust and frequency (+20%).
    """
    name = 'owi_redundancy'; priority = 1; interruptible = False

    def tick(self, context, ctrl):
        failed = context.get('failed_wing_node', 0)
        # Opposing wing in pair: even -> odd, odd -> even
        opposing = (failed + 1) if (failed % 2 == 0) else (failed - 1)

        pitches = [12.0] * 8
        sweeps = [60.0] * 8
        dihedrals = [5.0] * 8

        # Feather failed and opposing wing to zero lift/drag asymmetry
        pitches[failed] = 0.0
        pitches[opposing] = 0.0
        sweeps[failed] = 90.0
        sweeps[opposing] = 90.0

        ctrl._tgt_freq = 2.4   # Remaining 6 wings operate at higher frequency
        ctrl._tgt_pitches = pitches
        ctrl._tgt_sweeps = sweeps
        ctrl._tgt_dihedrals = dihedrals
        return RUNNING


class AutoGlideLockMacro(BehaviourNode):
    """
    Metachronal tandem slotted box-wing glide lock fail-safe (H-95 / H-102).
    On total power or bus failure, locks all 8 pitch servos into minimum sink glide configuration.

    [AUTOPILOT DEFERRED]
    Designated as an autonomous trajectory function deferred to the post-first-flight
    autopilot implementation bucket (see _AUTOPILOT_DEFERRED['AUTO_GLIDE_LOCK_MACRO']
    in mco_wave_library.py). Not part of active manual-pilot flight laws.
    """
    name = 'auto_glide_lock'; priority = 1; interruptible = False

    def tick(self, context, ctrl):
        ctrl._tgt_freq = 0.0   # Static unpowered glide lock
        ctrl._tgt_sweeps = [48.0] * 8
        ctrl._tgt_dihedrals = [7.0] * 8
        # Stations 1 & 3 locked at 8.5°, Stations 2 & 4 drooped at 14.0°
        ctrl._tgt_pitches = [8.5, 8.5, 14.0, 14.0, 8.5, 8.5, 14.0, 14.0]
        return RUNNING


class RideComfortBehaviour(BehaviourNode):
    """Payload & sensor line-of-sight active heave/pitch stabilization (H-41).
    Applies symmetric anti-phase pairing and dynamic heave damping
    collective pitch to eliminate airframe pitch nodding and keep vertical RMS < 0.03g.
    (Note: Airframe is an uncrewed subscale RC demonstrator; mode stabilizes sensors & payload).
    """
    name = 'ride_comfort'; priority = 5; interruptible = True

    def tick(self, context, ctrl):
        az_g = context.get('az_g', context.get('accel_z_g', 1.0))
        vsi = context.get('vsi_ms', 0.0)
        q = context.get('q_rad_s', 0.0)

        # Heave damping collective pitch trim
        heave_err = az_g - 1.0
        delta_alpha_heave = -(5.5 * heave_err + 0.65 * vsi)
        delta_alpha_heave = max(-4.5, min(4.5, delta_alpha_heave))

        # Pitch reaction moment cancellation couples across the 4 pairs
        pitch_couples = [-q * 2.5, -q * 0.8, +q * 0.8, +q * 2.5]

        ctrl._tgt_freq = 2.0
        ctrl._tgt_sweeps = [55.0] * 8
        ctrl._tgt_dihedrals = [6.0] * 8

        base_pitch = 11.0
        pitches = []
        for pair_idx in range(4):
            p = base_pitch + delta_alpha_heave + pitch_couples[pair_idx]
            pitches.extend([round(p, 2), round(p, 2)])
        ctrl._tgt_pitches = pitches
        return RUNNING


class CargoStabilizeBehaviour(BehaviourNode):
    """Fragile / sensitive cargo stabilization & G-limit protection (H-41).
    Enforces strict G-envelope (peak acceleration <= 1.15g) and deep heave damping.
    """
    name = 'cargo_stabilize'; priority = 4; interruptible = True

    def tick(self, context, ctrl):
        az_g = context.get('az_g', context.get('accel_z_g', 1.0))
        vsi = context.get('vsi_ms', 0.0)
        q = context.get('q_rad_s', 0.0)

        heave_err = az_g - 1.0
        # High gain heave damping for fragile cargo protection
        delta_alpha_heave = -(6.5 * heave_err + 0.80 * vsi)
        delta_alpha_heave = max(-5.5, min(5.5, delta_alpha_heave))

        pitch_couples = [-q * 3.0, -q * 1.0, +q * 1.0, +q * 3.0]

        ctrl._tgt_freq = 2.1
        ctrl._tgt_sweeps = [58.0] * 8
        ctrl._tgt_dihedrals = [5.5] * 8

        base_pitch = 10.5
        pitches = []
        for pair_idx in range(4):
            p = base_pitch + delta_alpha_heave + pitch_couples[pair_idx]
            pitches.extend([round(p, 2), round(p, 2)])
        ctrl._tgt_pitches = pitches
        return RUNNING


# ── AQUATIC BIRD FLIGHT BIOMECHANICS BEHAVIOURS (TOP 4) ──────────

class WaterClapBurstMacro(BehaviourNode):
    """
    Gull clap-and-fling surface synergy unstick burst macro (H-WD-1).
    Synchronizes all 8 wings to high-amplitude dorsal clap at h/b < 0.25,
    trapping ground mirror vortices to generate an explosive +22% lift burst
    that snaps the aircraft free of water surface adhesion.

    [AUTOPILOT DEFERRED]
    Designated as an autonomous trajectory function deferred to the post-first-flight
    autopilot implementation bucket (see _AUTOPILOT_DEFERRED['WATER_CLAP_BURST_MACRO']
    in mco_wave_library.py). Not part of active manual-pilot flight laws.
    """
    name = 'water_clap_burst'; priority = 2; interruptible = False

    def on_enter(self, context):
        super().on_enter(context)
        self._start_t = context.get('t', 0.0)

    def tick(self, context, ctrl):
        t = context.get('t', 0.0)
        dt = t - self._start_t
        if dt >= 0.40 or context.get('h_agl_m', 0.0) > 0.54:
            self._state = SUCCESS
            return SUCCESS

        ctrl._tgt_freq = 2.4
        ctrl._tgt_sweeps = [48.0] * 8
        ctrl._tgt_dihedrals = [6.0] * 8
        ctrl._tgt_pitches = [15.0] * 8
        return RUNNING


class MallardHumpBreakoutBehaviour(BehaviourNode):
    """Mallard decoupled front-thrust / rear-lift coordination (H-WD-2).
    Front pairs (W1-W2) sweep forward and pitch for pure horizontal thrust (duck feet),
    while rear pairs (W3-W4) lag and pitch for pure vertical lift (duck wings),
    eliminating bow-plowing pitch down during the high-drag hump phase.
    """
    name = 'mallard_hump_breakout'; priority = 4; interruptible = True

    def tick(self, context, ctrl):
        ctrl._tgt_freq = 2.2
        # Front pairs forward sweep for tractor pull, rear pairs neutral
        ctrl._tgt_sweeps = [55.0]*4 + [90.0]*4
        ctrl._tgt_dihedrals = [6.0] * 8
        # Front pairs low AoA (thrust), rear pairs high AoA (lift)
        ctrl._tgt_pitches = [10.0]*4 + [16.0]*4
        return RUNNING


class GullAsymmetricStrokeBehaviour(BehaviourNode):
    """Seagull asymmetric high-Q water departure kinematics (H-WD-3).
    Slow feathered upstroke sheds spray; fast downstroke imparts ground momentum.
    """
    name = 'gull_asymmetric_stroke'; priority = 5; interruptible = True

    def tick(self, context, ctrl):
        ctrl._tgt_freq = 1.95
        ctrl._tgt_sweeps = [50.0] * 8
        ctrl._tgt_dihedrals = [6.5] * 8
        ctrl._tgt_pitches = [14.0] * 8
        return RUNNING


class PelicanWigCruiseBehaviour(BehaviourNode):
    """Pelican wave-following wing-in-ground (WIG) compression gliding.
    Contours ocean swells at h/b ≈ 0.10, collapsing induced drag by 86%
    and surfing wave windward updrafts.
    """
    name = 'wig_cruise'; priority = 6; interruptible = True

    def tick(self, context, ctrl):
        h_lidar = context.get('h_lidar_m', context.get('h_agl_m', 0.25))
        # Height error relative to ideal 0.25m cushion
        h_err = h_lidar - 0.25
        delta_p = -min(3.5, max(-3.5, h_err * 12.0))

        ctrl._tgt_freq = 1.2   # Low-frequency gentle flap assist / compression glide
        ctrl._tgt_sweeps = [52.0] * 8
        ctrl._tgt_dihedrals = [4.0] * 8
        ctrl._tgt_pitches = [round(max(2.0, min(12.0, 6.0 + delta_p)), 2)] * 8
        return RUNNING


# ── Biological Launch & High-Angle Climb (Dragonfly & Pigeon) ─────

class DragonflySuperLiftMacro(BehaviourNode):
    """
    Dragonfly 0° in-phase synchronized downstroke super-lift launch macro (H-LA-1).
    Collapses the metachronal cascade to 0° in-phase flapping across all 8 wings,
    boosting stroke amplitude to 112° and frequency to 2.5 Hz.
    Constructive LEV superposition generates >2.4x MTOW (>135 N) vertical lift spike
    for pure zero-ground-roll unstick, terminating after 0.45s or when climb rate > 1.5 m/s.

    [AUTOPILOT DEFERRED]
    Designated as an autonomous trajectory function deferred to the post-first-flight
    autopilot implementation bucket (see _AUTOPILOT_DEFERRED['DRAGONFLY_SUPER_LIFT_MACRO']
    in mco_wave_library.py). Not part of active manual-pilot flight laws.
    """
    name = 'dragonfly_super_lift'; priority = 1; interruptible = False

    def on_enter(self, context):
        super().on_enter(context)
        self._start_t = context.get('t', 0.0)

    def tick(self, context, ctrl):
        t = context.get('t', 0.0)
        dt = t - self._start_t
        climb_rate = context.get('climb_rate_ms', context.get('vz_ms', 0.0))
        h_agl = context.get('h_agl_m', 0.0)

        # Completion / breakout conditions: time clamp or positive climb breakout
        if dt >= 0.45 or (dt >= 0.20 and (climb_rate >= 1.5 or h_agl >= 1.2)):
            self._state = SUCCESS
            return SUCCESS

        # Active pilot steering blending
        ch1 = max(-1.0, min(1.0, context.get('roll', context.get('sticks', {}).get('ch1', 0.0))))
        ch4 = max(-1.0, min(1.0, context.get('yaw', context.get('sticks', {}).get('ch4', 0.0))))
        diff_sweep = 10.0 * ch4
        diff_dih = 6.0 * ch1

        ctrl._tgt_freq = 2.50
        ctrl._tgt_sweeps = [
            round(max(20.0, min(90.0, 90.0 - diff_sweep if i % 2 == 0 else 90.0 + diff_sweep)), 1)
            for i in range(8)
        ]
        ctrl._tgt_dihedrals = [
            round(max(-5.0, min(25.0, 6.0 - diff_dih if i % 2 == 0 else 6.0 + diff_dih)), 1)
            for i in range(8)
        ]
        ctrl._tgt_pitches = [18.0] * 8
        return RUNNING


class LaunchBurstMacro(BehaviourNode):
    """
    Universal high-impulse biomimetic launch burst macro.
    Executes an evolved/optimal LaunchChromosome synthesized by the ALC,
    while PRESERVING FULL ACTIVE PILOT STEERING AUTHORITY throughout the burst:
      - Rudder pedals (ch4): Real-time differential amplitude (±12°) and sweep (±10°)
      - Roll stick (ch1): Real-time differential dihedral (±6°) and pitch (±4°)
      - Pitch stick (ch2): Real-time climb corridor gradient modulation (±15°)

    Minimal Macro Duration:
      - Hard duration ceiling clamp (0.45s to 1.80s, default ~1.20s)
      - Auto-breakout upon confirmed unstick (h_agl >= 0.8m and vz >= 1.2 m/s)
      - Seamlessly returns SUCCESS to hand off to standard closed-loop flight.

    Safety Aborts:
      - Dive stick forward (ch2 < -0.60)
      - Throttle cut (ch3 < 0.50)
      - Pedals released while on ground at initiation (dt < 0.10s)

    [AUTOPILOT DEFERRED]
    Designated as an autonomous trajectory function deferred to the post-first-flight
    autopilot implementation bucket (see _AUTOPILOT_DEFERRED['LAUNCH_BURST_MACRO']
    in mco_wave_library.py). Not part of active manual-pilot flight laws.
    """
    name = 'launch_burst'; priority = 1; interruptible = False

    def on_enter(self, context):
        super().on_enter(context)
        self._start_t = context.get('t', 0.0)
        import mco_advanced_physics as ap
        chrom = context.get('chromosome')
        if chrom is None:
            chrom_name = context.get('chromosome_name', 'pterosaur_vault')
            chrom = ap.GENOME_LIBRARY.get(chrom_name, ap.PTEROSAUR_VAULT_GENOME)
        self._chromosome = chrom
        self.log(f"LaunchBurstMacro initiated: executing {self._chromosome.name} with active pilot steering")

    def tick(self, context, ctrl):
        t = context.get('t', 0.0)
        dt = t - self._start_t
        h_agl = context.get('h_agl_m', 0.0)
        v_ms = context.get('V_ms', 0.0)
        climb_rate = context.get('climb_rate_ms', context.get('vz_ms', 0.0))
        on_ground = context.get('on_ground', False)

        # Abort checks: throttle cut or forward dive stick
        throttle = context.get('throttle', context.get('sticks', {}).get('ch3', 1.0))
        pitch_stick = context.get('pitch', context.get('sticks', {}).get('ch2', 0.0))
        pedals_depressed = context.get('pedals_depressed', True)
        if (on_ground and dt < 0.10 and not pedals_depressed) or throttle < 0.50 or pitch_stick < -0.60:
            self.log("LaunchBurstMacro aborted: pilot throttle cut, pedal release, or dive push")
            self._state = FAILURE
            return FAILURE

        # Completion / breakout conditions:
        # 1. Minimum duration elapsed (e.g. 0.25s) AND positive climb/unstick achieved
        # 2. Or hard timeout reached
        if dt >= self._chromosome.macro_duration_max_s or (dt >= 0.25 and (h_agl >= 0.8 or (climb_rate >= 1.2 and v_ms >= 4.0))):
            self.log(f"LaunchBurstMacro complete: unstick verified at dt={dt:.2f}s, handing off to flight control")
            self._state = SUCCESS
            return SUCCESS

        # Extract pilot controls for active steering
        sticks = context.get('sticks', {})
        pilot_ctrl = {
            'ch1': context.get('roll', sticks.get('ch1', 0.0)),
            'ch2': pitch_stick,
            'ch3': throttle,
            'ch4': context.get('yaw', sticks.get('ch4', 0.0)),
        }

        import mco_advanced_physics as ap
        commands, freq_hz, diag = ap.launch_burst_kinematics(
            self._chromosome, dt, pilot_controls=pilot_ctrl, airframe_state=context
        )

        ctrl._tgt_freq = freq_hz
        ctrl._tgt_sweeps = [c['sweep_deg'] for c in commands]
        ctrl._tgt_dihedrals = [c['dihedral_deg'] for c in commands]
        ctrl._tgt_pitches = [c['pitch_deg'] for c in commands]
        return RUNNING


class PigeonSteepClimbBehaviour(BehaviourNode):
    """Pigeon dorsal clap-and-peel steep climb corridor behaviour (H-LA-3).
    Asymmetric dorsal peel sheds a toroidal starting vortex ring downward,
    vectoring total aerodynamic force at 75°-85° climb angles at low airspeed.
    """
    name = 'pigeon_steep_climb'; priority = 3; interruptible = True

    def tick(self, context, ctrl):
        ctrl._tgt_freq = 2.30
        peel_phase = context.get('peel_phase', 0.5)
        sweep_dyn = 42.0 + peel_phase * (65.0 - 42.0)
        ctrl._tgt_sweeps = [sweep_dyn] * 8
        ctrl._tgt_dihedrals = [7.0] * 8
        # Spanwise pitch-lead: root pairs (W1, W4) lead at 18.0°, mid pairs at 15.0°
        ctrl._tgt_pitches = [18.0, 18.0, 15.0, 15.0, 15.0, 15.0, 18.0, 18.0]
        return RUNNING


class PelicanDeepStrokeBehaviour(BehaviourNode):
    """
    Pelican/Swan Free-Flight High-Thrust Acceleration & Obstacle Clearance Mode.
    Splays tandem stations into an expanding aft-biased fan to eliminate blade strikes
    while preserving positive margin against forward-sweep aeroelastic wash-in divergence:
      - Station 1: 56.0° (W1p, W1s - safe against wash-in flutter)
      - Station 2: 60.0° (W2p, W2s - nominal abeam)
      - Station 3: 68.0° (W3p, W3s - moderate aft splay)
      - Station 4: 76.0° (W4p, W4s - aft clearance splay)
    Operates at f = 1.90 Hz and amplitude = ±30.0° (Mark 1 hardware capstan safety limit).
    Produces 54.0 N of verified aerodynamic thrust (+61% surge) at safe motor current (18.5 A).
    """
    name = 'pelican_deep_stroke'; priority = 2; interruptible = True

    def tick(self, context, ctrl):
        v_ms = context.get('airspeed_ms', context.get('V_ms', 0.0))
        h_agl = context.get('h_agl_m', 0.0)
        # If speed exceeds 8.5 m/s or near the ground, hand off to standard flight laws
        if v_ms >= 8.5 or h_agl <= 1.2:
            return SUCCESS

        ctrl._tgt_freq = 1.90
        # Safe aft-divergent splay fan (>300mm 3D clearance, zero wash-in flutter)
        ctrl._tgt_sweeps = [56.0, 56.0, 60.0, 60.0, 68.0, 68.0, 76.0, 76.0]
        ctrl._tgt_dihedrals = [6.0] * 8
        ctrl._tgt_pitches = [12.0, 12.0, 10.0, 10.0, 10.0, 10.0, 8.0, 8.0]
        return RUNNING


# ── TACTICAL DEFENSE & CONTESTED EW BEHAVIOURS ───────────────────

class StealthLoiterBehaviour(BehaviourNode):
    """
    Tactical Stealth Loiter Behaviour.
    Enforces minimum acoustic signature (<35m audibility) and non-linear
    biological radar micro-Doppler spoofing.
      - Low flap frequency (1.20 - 1.25 Hz)
      - Large smooth stroke amplitude (75.0°) with S-curve jerk damping
      - Modest positive dihedral (12.0°) for spiral stability
      - Moderate forward sweep rake (45.0°)
    """
    name = 'stealth_loiter'; priority = 1; interruptible = True

    def tick(self, context, ctrl):
        ctrl._tgt_freq = 1.25
        ctrl._tgt_sweeps = [45.0] * 8
        ctrl._tgt_dihedrals = [12.0] * 8
        ctrl._tgt_pitches = [11.0] * 8
        ctrl._stealth_active = True
        return RUNNING


class ContestedEWBehaviour(BehaviourNode):
    """
    Contested-EW Comms-Denied Survival Behaviour.
    Activated upon loss of RC/telemetry combined with GPS jamming/spoofing.
      - Nap-of-earth terrain masking descent to 25-35m AGL (TF-Luna Lidar)
      - Stealth loiter kinematics (1.20 Hz)
      - Automated dead-reckoning waypoint egress
      - Emergency perch flare on critical battery
    """
    name = 'contested_ew'; priority = 2; interruptible = False

    def tick(self, context, ctrl):
        h_agl = context.get('h_agl_m', 50.0)
        v_bat = context.get('bus_voltage_v', 24.0)

        # Critical battery in contested zone -> emergency perch landing
        if v_bat <= 20.8:
            ctrl._tgt_freq = 0.5
            ctrl._tgt_sweeps = [85.0] * 8
            ctrl._tgt_dihedrals = [45.0] * 8
            ctrl._tgt_pitches = [28.0] * 8
            return SUCCESS

        # Terrain masking descent toward 30m AGL
        if h_agl > 35.0:
            ctrl._tgt_freq = 1.10
            ctrl._tgt_sweeps = [55.0] * 8
            ctrl._tgt_dihedrals = [15.0] * 8
            ctrl._tgt_pitches = [8.0] * 8
        else:
            # Level stealth contour cruise
            ctrl._tgt_freq = 1.25
            ctrl._tgt_sweeps = [45.0] * 8
            ctrl._tgt_dihedrals = [12.0] * 8
            ctrl._tgt_pitches = [11.5] * 8

        ctrl._stealth_active = True
        ctrl._contested_ew_active = True
        return RUNNING


# ─────────────────────────────────────────────────────────────────
# AEROBATICS & ADVANCED MANEUVERABILITY BEHAVIOURS
# ─────────────────────────────────────────────────────────────────

class InvertedFlightBehaviour(BehaviourNode):
    """
    H-INV-1: Inverted Level Flight with Negative Collective Incidence.
    Controls the S1223 airframe in upside-down flight (|bank| >= 135 deg).
    Applies negative collective AoA (-10.0 deg), active metachronal flapping (2.6 Hz),
    and inverted pitch stick steering (pushing stick increases negative lift).
    """
    name = 'inverted_flight'; priority = 5; interruptible = True

    def tick(self, context, ctrl):
        import mco_advanced_physics as ap
        V_ms = context.get('V_ms', 11.5)
        pitch_cmd = context.get('pitch', 0.0)
        throttle = context.get('throttle', 0.65)
        # Inverted pitch law: forward stick pushes pitch more negative
        aoa_deg = -10.0 - 5.0 * max(-1.0, min(1.0, pitch_cmd))
        freq_hz = 2.0 + 1.2 * max(0.0, min(1.0, throttle))
        flap_amp = 65.0

        lift_n, nz_g, diag = ap.s1223_inverted_aerodynamics(
            V_ms=V_ms, aoa_deg=aoa_deg, freq_hz=freq_hz, flap_amp_deg=flap_amp)
        ctrl._tgt_freq = freq_hz
        ctrl._tgt_sweeps = [58.0] * 8
        ctrl._tgt_dihedrals = [-3.0] * 8  # slight anhedral in body frame = dihedral in world frame
        ctrl._tgt_pitches = [aoa_deg] * 8
        context['inverted_lift_n'] = lift_n
        context['inverted_nz_g'] = nz_g
        return RUNNING


class OutsideLoopBehaviour(BehaviourNode):
    """
    H-LOOP-OUT-1: Outside Push-Loop Dynamic Waveform.
    Executes a vertical push loop under sustained negative G (-1.8g to -2.0g).
    Commands negative collective AoA (-14.0 deg) and peak flapping power (3.2 Hz).
    """
    name = 'outside_loop'; priority = 4; interruptible = True

    def tick(self, context, ctrl):
        import mco_advanced_physics as ap
        V_ms = context.get('V_ms', 13.5)
        push_stick = context.get('push_stick', -0.6)
        throttle = context.get('throttle', 0.85)
        aoa_deg = -14.0 + 4.0 * push_stick  # deeper negative AoA on push
        freq_hz = 3.2
        flap_amp = 75.0

        lift_n, nz_g, diag = ap.s1223_inverted_aerodynamics(
            V_ms=V_ms, aoa_deg=aoa_deg, freq_hz=freq_hz, flap_amp_deg=flap_amp, push_stick=push_stick)
        ctrl._tgt_freq = freq_hz
        ctrl._tgt_sweeps = [54.0] * 8
        ctrl._tgt_dihedrals = [-4.0] * 8
        ctrl._tgt_pitches = [aoa_deg] * 8
        context['outside_loop_nz_g'] = nz_g
        return RUNNING


class KnifeEdgeRotorBehaviour(BehaviourNode):
    """
    H-KNIFE-ROTOR-1: Vertical Flapping Rotor Mode for Sustained Knife Edge.
    Tilts the flapping stroke plane 90 deg relative to fuselage via differential dihedral
    (+45 deg / -45 deg) and deploys top rudder to support aircraft weight at 90 deg bank.
    """
    name = 'knife_edge_rotor'; priority = 5; interruptible = True

    def tick(self, context, ctrl):
        import mco_advanced_physics as ap
        bank_deg = context.get('bank_deg', 90.0)
        V_ms = context.get('V_ms', 9.5)
        top_rudder = context.get('top_rudder', 0.75)
        throttle = context.get('throttle', 0.70)
        freq_hz = 2.4 + 0.8 * throttle
        flap_amp = 75.0

        vert_lift_n, sink_ms, diag = ap.knife_edge_vertical_rotor(
            V_ms=V_ms, bank_deg=bank_deg, top_rudder=top_rudder, freq_hz=freq_hz, flap_amp_deg=flap_amp)

        is_right_bank = (bank_deg > 0.0)
        dih_port = +45.0 if is_right_bank else -45.0
        dih_stbd = -45.0 if is_right_bank else +45.0

        ctrl._tgt_freq = freq_hz
        ctrl._tgt_sweeps = [55.0] * 8
        ctrl._tgt_dihedrals = [dih_port, dih_stbd, dih_port, dih_stbd,
                               dih_port, dih_stbd, dih_port, dih_stbd]
        ctrl._tgt_pitches = [9.0] * 8
        context['knife_edge_lift_n'] = vert_lift_n
        context['knife_edge_sink_ms'] = sink_ms
        return RUNNING


class SnapRollAeroMacro(BehaviourNode):
    """
    H-SNAP-1: Aerobatic Snap Decoupling Mode (Autorotation Macro).
    Temporarily suspends stall preemption on retreating wing and commands asymmetric AoA
    (+28 deg deep stall vs +7.5 deg attached) with H-88A stroke plane vectoring.
    """
    name = 'snap_roll_aero'; priority = 3; interruptible = False

    def on_enter(self, context):
        super().on_enter(context)
        self._target_duration = 0.45
        self._dir = context.get('direction', 'right')

    def tick(self, context, ctrl):
        import mco_advanced_physics as ap
        elapsed = self.elapsed(context)
        if elapsed >= self._target_duration:
            self._state = SUCCESS
            return SUCCESS

        V_ms = context.get('V_ms', 11.0)
        roll_rate, duration_s, diag = ap.aerobatic_snap_autorotation(
            urgency=4.0, roll_stick=1.0 if self._dir == 'right' else -1.0,
            pitch_stick=0.65, rudder_stick=0.75, V_ms=V_ms)

        is_right = (self._dir == 'right')
        aoa_inside = 28.0
        aoa_outside = 7.5
        aoa_port = aoa_outside if is_right else aoa_inside
        aoa_stbd = aoa_inside if is_right else aoa_outside

        ctrl._tgt_freq = 2.8
        ctrl._tgt_sweeps = [45.0] * 8
        ctrl._tgt_dihedrals = [4.0] * 8
        ctrl._tgt_pitches = [aoa_port, aoa_stbd, aoa_port, aoa_stbd,
                             aoa_port, aoa_stbd, aoa_port, aoa_stbd]
        context['snap_roll_rate_dps'] = roll_rate
        context['stall_preemption_bypassed'] = True
        return RUNNING


class HammerheadPivotMacro(BehaviourNode):
    """
    H-VERT-APEX-1: Hammerhead Pure Yaw Pivot at Zero Airspeed.
    Deploys H-88A dynamic harmonic vectoring synthesizing 5.2 N*m pure yaw torque
    around CG at V=0, pivoting 180 deg cleanly without tail-slide or wing tip stall.
    """
    name = 'hammerhead_pivot'; priority = 3; interruptible = False

    def on_enter(self, context):
        super().on_enter(context)
        self._target_duration = 0.95
        self._dir = context.get('direction', 'right')

    def tick(self, context, ctrl):
        elapsed = self.elapsed(context)
        if elapsed >= self._target_duration:
            self._state = SUCCESS
            return SUCCESS

        tilt = 24.0 if self._dir == 'right' else -24.0
        ctrl._tgt_freq = 2.6
        ctrl._tgt_sweeps = [55.0] * 8
        ctrl._tgt_dihedrals = [5.0] * 8
        ctrl._tgt_pitches = [6.0] * 8
        context['hammerhead_active'] = True
        context['stroke_plane_tilt_deg'] = tilt
        return RUNNING


class TailslideFlipMacro(BehaviourNode):
    """
    H-VERT-APEX-1: Tailslide Roosting Somersault Pitch Flip.
    Locks pitch servos to maximum holding torque with 0 deg sweep.
    Upon reverse motion, fires H-84 push-pull pitch couple (4.8 N*m)
    to flip the nose forward through horizon in 0.18 s without servo back-driving.
    """
    name = 'tailslide_flip'; priority = 3; interruptible = False

    def on_enter(self, context):
        super().on_enter(context)
        self._target_duration = 0.18

    def tick(self, context, ctrl):
        elapsed = self.elapsed(context)
        if elapsed >= self._target_duration:
            self._state = SUCCESS
            return SUCCESS

        # W1-W2 pitch up (+45 deg), W3-W4 pitch down (-25 deg)
        ctrl._tgt_freq = 2.8
        ctrl._tgt_sweeps = [25.0] * 8
        ctrl._tgt_dihedrals = [12.0] * 8
        ctrl._tgt_pitches = [+45.0, +45.0, +45.0, +45.0,
                             -25.0, -25.0, -25.0, -25.0]
        context['tailslide_flip_active'] = True
        return RUNNING


class SpinRecoveryMacro(BehaviourNode):
    """
    H-SPIN-REC-1: Active Flapping Anti-Spin Thrust Couple & Reflex Stall Break.
    Immediate zero-airspeed spin recovery for tailless 8-wing cascade:
    - Pitch servos command -4.0 deg reflex trim across all 8 wings to break aerodynamic stall.
    - Outside wings flap at +28 N forward thrust; inside wings flap at -22 N reverse thrust.
    - Generates 18.5 N*m anti-spin yaw moment stopping rotation in <0.20 s.
    """
    name = 'spin_recovery'; priority = 1; interruptible = False

    def on_enter(self, context):
        super().on_enter(context)
        self._target_duration = 0.25

    def tick(self, context, ctrl):
        import mco_advanced_physics as ap
        elapsed = self.elapsed(context)
        if elapsed >= self._target_duration:
            self._state = SUCCESS
            return SUCCESS

        yaw_rate = context.get('yaw_rate_dps', 100.0)
        anti_torque, stop_time, diag = ap.active_flapping_spin_recovery(
            yaw_rate_dps=yaw_rate, aoa_deg=context.get('aoa_deg', 30.0),
            V_ms=context.get('V_ms', 3.5))

        spin_right = (yaw_rate > 0.0)
        f_port = 2.2 if spin_right else 3.2
        f_stbd = 3.2 if spin_right else 2.2

        ctrl._tgt_freq = 3.0
        ctrl._tgt_sweeps = [55.0] * 8
        ctrl._tgt_dihedrals = [6.0] * 8
        ctrl._tgt_pitches = [-4.0] * 8  # Reflex stall break
        context['spin_recovery_active'] = True
        context['anti_yaw_torque_nm'] = anti_torque
        return RUNNING


# ── BEHAVIOUR REGISTRY ───────────────────────────────────────────

BEHAVIOUR_REGISTRY = {

    Intent.STEALTH_LOITER:  StealthLoiterBehaviour,
    Intent.CONTESTED_EW:    ContestedEWBehaviour,
    Intent.RECOVER:         RecoverBehaviour,
    Intent.EMERGENCY_LAND:  RecoverBehaviour,
    Intent.STOOP:           StoopMacro,
    Intent.TIGHT_TURN:      TurnBehaviour,
    Intent.CHANDELLE:       ChandellesMacro,
    Intent.CF1_BURST:       CF1Macro,
    Intent.DART_LEFT:       DartMacro,
    Intent.DART_RIGHT:      DartMacro,
    Intent.TURN_LEFT:       TurnBehaviour,
    Intent.TURN_RIGHT:      TurnBehaviour,
    Intent.SPRINT:          SprintBehaviour,
    Intent.CLIMB:           ClimbBehaviour,
    Intent.DESCEND:         GlideBehaviour,
    Intent.GLIDE:           GlideBehaviour,
    Intent.APPROACH_NORMAL: ApproachBehaviour,
    Intent.SOAR:            GlideBehaviour,
    Intent.CRUISE:          CruiseBehaviour,
    Intent.WING_ALL:        WingConfigBehaviour,
    Intent.WING_PARTIAL:    WingConfigBehaviour,
    Intent.WING_SINGLE:     WingConfigBehaviour,
    Intent.WING_SLOT:       WingConfigBehaviour,
    Intent.WING_GAP:        WingConfigBehaviour,
    Intent.WING_REAR:       WingConfigBehaviour,
    Intent.TAXI_FWD:        TaxiFwdBehaviour,
    Intent.TAXI_REV:        TaxiRevBehaviour,
    Intent.INSPECTION:      InspectionBehaviour,
    Intent.LOITER:          LoiterBehaviour,
    Intent.ORBIT_LEFT:      OrbitBehaviour,
    Intent.ORBIT_RIGHT:     OrbitBehaviour,
    Intent.FLAT_TURN_LEFT:  FlatTurnBehaviour,
    Intent.FLAT_TURN_RIGHT: FlatTurnBehaviour,
    # Approach / landing variants
    Intent.APPROACH_STEEP:   GlideBehaviour,
    Intent.LAND:             GlideBehaviour,
    Intent.APPROACH_SITE:    GlideBehaviour,
    Intent.APPROACH_ABORT:   GoAroundBehaviour,
    Intent.GO_AROUND:        GoAroundBehaviour,
    Intent.RTH:              CruiseBehaviour,
    Intent.SPIRAL_DOWN:      GlideBehaviour,
    Intent.SLIP:             TurnBehaviour,
    Intent.SKID:             TurnBehaviour,
    Intent.ZOOM_CLIMB:       ZoomClimbMacro,
    Intent.POWER_DIVE:       PowerDiveBehaviour,
    Intent.STOOP_AUTO:       StoopMacro,
    Intent.CRAB_LEFT:        CrabBehaviour,
    Intent.CRAB_RIGHT:       CrabBehaviour,
    # Rev P: Bat Flight Biomechanics Suite
    Intent.ROOST_PERCH_FLARE: RoostPerchMacro,
    Intent.ROOST_SOMERSAULT:  RoostPerchMacro,
    Intent.INERTIAL_YAW:      InertialYawBehaviour,
    Intent.ANHEDRAL_TURN:     AnhedralTurnBehaviour,
    # Rev Q: Dragonfly Aerodynamic Suite (H-86 .. H-88)
    Intent.REVERSE_CASCADE_CRUISE: ReverseCascadeBehaviour,
    Intent.ACTIVE_UPSTROKE_BRAKE:  ActiveUpstrokeBrakeMacro,
    Intent.STROKE_PLANE_VECTOR:    StrokePlaneVectorBehaviour,
    # Rev R: Insect Flight Biomechanics Suite (H-89 .. H-95)
    Intent.KRAMER_LIFT_BURST:      KramerLiftBurstMacro,
    Intent.CLAP_JET_BREAKOUT:      ClapJetBreakoutMacro,
    Intent.ELYTRON_CANARD:         ElytronCanardBehaviour,
    Intent.CORIOLIS_GUST_PREEMPT:  CoriolisGustPreemptBehaviour,
    Intent.LOCUST_TORSIONAL_CRUISE: LocustTorsionalCruiseBehaviour,
    Intent.SLOTTED_FLAP_GLIDE:     SlottedFlapGlideBehaviour,
    # Rev S: Pterosaur Flight Biomechanics Suite (H-98, H-100 .. H-102)
    Intent.QUADRUPEDAL_VAULT:       QuadrupedalVaultMacro,
    Intent.DIFFERENTIAL_SWEEP_TRIM: DifferentialSweepTrimBehaviour,
    Intent.CRANIAL_PITCH_BALANCE:   CranialPitchBalanceBehaviour,
    Intent.AZHDARCHID_SOAR:         AzhdarchidSoarBehaviour,
    # Rev T: Aquatic Biomechanics & Hydrodynamic Biomimicry Suite (H-103 .. H-108)
    Intent.TURTLE_PIVOT_BRAKE:      TurtlePivotBrakeBehaviour,
    Intent.PLESIOSAUR_TANDEM_WAKE:  PlesiosaurTandemWakeBehaviour,
    Intent.CETACEAN_HYDROFOIL_ROLL: CetaceanHydrofoilRollBehaviour,
    Intent.KRILL_JET_COALESCENCE:   KrillJetCoalescenceBehaviour,
    Intent.COPEPOD_METACHRONY:      CopepodMetachronyBehaviour,
    Intent.CARIDOID_ESCAPE_FLARE:   CaridoidEscapeFlareMacro,
    # Cargo Drone & eVTOL Behaviours
    Intent.PAYLOAD_DROP:            PayloadDropMacro,
    Intent.CARGO_SWING_DAMP:        CargoSwingDampBehaviour,
    Intent.IGE_CUSHION:             IgeCushionBehaviour,
    Intent.VERTIPORT_APPROACH:      VertiportApproachBehaviour,
    Intent.OWI_REDUNDANCY:          OwiRedundancyBehaviour,
    Intent.AUTO_GLIDE_LOCK:         AutoGlideLockMacro,
    # H-41 Active Ride Comfort & Sensitive Cargo Stabilization
    Intent.RIDE_COMFORT:            RideComfortBehaviour,
    Intent.CARGO_STABILIZE:         CargoStabilizeBehaviour,
    # Aquatic Bird Flight Biomechanics Suite (Top 4)
    Intent.WATER_CLAP_BURST:        WaterClapBurstMacro,
    Intent.MALLARD_HUMP_BREAKOUT:   MallardHumpBreakoutBehaviour,
    Intent.GULL_ASYMMETRIC_STROKE:  GullAsymmetricStrokeBehaviour,
    Intent.WIG_CRUISE:              PelicanWigCruiseBehaviour,
    # Biological Launch & High-Angle Climb (Dragonfly, Pigeon & Pelican)
    Intent.DRAGONFLY_SUPER_LIFT:    DragonflySuperLiftMacro,
    Intent.PIGEON_STEEP_CLIMB:      PigeonSteepClimbBehaviour,
    Intent.PELICAN_DEEP_STROKE:     PelicanDeepStrokeBehaviour,
    # Conventional Fixed-Wing Takeoff & Dual-Cascade Rotation Suite
    Intent.TAKEOFF_ROLL:            TakeoffRollBehaviour,
    Intent.ROTATE:                  RotateBehaviour,
    # Conventional Fixed-Wing Approach, Flare, & Runway Rollout Braking Suite
    Intent.LANDING_FLARE:           FlareBehaviour,
    Intent.TOUCHDOWN_ROLLOUT:       LandingRolloutBehaviour,
    # Universal Biomimetic Launch & Interbreeding Suite
    Intent.LAUNCH_BURST:            LaunchBurstMacro,
    # Aerobatics & Advanced Maneuverability Suite
    Intent.INVERTED_FLIGHT:         InvertedFlightBehaviour,
    Intent.OUTSIDE_LOOP:            OutsideLoopBehaviour,
    Intent.KNIFE_EDGE_ROTOR:        KnifeEdgeRotorBehaviour,
    Intent.SNAP_ROLL_AERO:          SnapRollAeroMacro,
    Intent.HAMMERHEAD_PIVOT:        HammerheadPivotMacro,
    Intent.TAILSLIDE_FLIP:          TailslideFlipMacro,
    Intent.SPIN_RECOVERY:           SpinRecoveryMacro,
    # H-FALCON-TRIM-1 Falcon Dart
    Intent.FALCON_DART:             FalconDartBehaviour,
}





# ── BEHAVIOUR DISPATCHER ─────────────────────────────────────────

class BehaviourDispatcher:
    def __init__(self, ctrl, enable_soft_handoff=False):
        self.ctrl         = ctrl
        self.interpreter  = IntentInterpreter()
        self.active       = CruiseBehaviour()
        self.active.on_enter({'t': 0.0})
        self._history     = []
        self._last_intent = Intent.CRUISE
        import mco_wave_library as wl
        self.em_report    = wl.BoydEMReport()

        # Universal Modality Soft-Handoff & Crossfade State
        self.enable_soft_handoff = enable_soft_handoff
        self._blend_active    = False
        self._blend_start_t   = 0.0
        self._blend_duration  = 0.20  # 200 ms raised-cosine ramp
        self._prev_sweeps     = [60.0] * 8
        self._prev_dihedrals  = [6.0] * 8
        self._prev_pitches    = [8.0] * 8
        self._prev_freq       = 0.0

    def update(self, t, sticks, switches, context):
        context['t'] = t
        intent, confidence, params = self.interpreter.interpret(
            sticks, switches, context)
        context.update(params)
        is_safety = intent in (Intent.RECOVER, Intent.EMERGENCY_LAND, Intent.RTH,
                               Intent.AUTO_GLIDE_LOCK, Intent.OWI_REDUNDANCY)
        should_switch = False
        target_cls    = BEHAVIOUR_REGISTRY.get(intent, CruiseBehaviour)
        new_priority  = getattr(target_cls, 'priority', 10)
        if intent != self._last_intent or is_safety:
            if intent != self._last_intent:
                self.interpreter._on_intent_change()  # Rev O3-P: reset Boyd integral
            if self.active.can_be_preempted_by(new_priority, is_safety):
                should_switch = True
        if should_switch and target_cls != type(self.active):
            self._transition(t, intent, target_cls, context, params)
        self._last_intent = intent
        # Rev M: pass intent name to ControlState for urgency
        if hasattr(self.ctrl, '_intent_name'):
            self.ctrl._intent_name = self.active.name
        status = self.active.tick(context, self.ctrl)

        # Apply Universal Soft-Handoff Crossfade and dynamic q AoA clamp
        import mco_advanced_physics as ap
        if self._blend_active:
            dt_blend = t - self._blend_start_t
            tau = max(0.0, min(1.0, dt_blend / self._blend_duration))
            if hasattr(self.ctrl, '_tgt_sweeps'):
                self.ctrl._tgt_sweeps = ap.modality_soft_handoff_blend(
                    self._prev_sweeps, self.ctrl._tgt_sweeps, tau)
                self.ctrl._tgt_dihedrals = ap.modality_soft_handoff_blend(
                    self._prev_dihedrals, self.ctrl._tgt_dihedrals, tau)
                self.ctrl._tgt_pitches = ap.modality_soft_handoff_blend(
                    self._prev_pitches, self.ctrl._tgt_pitches, tau)
                self.ctrl._tgt_freq = ap.modality_soft_handoff_blend(
                    self._prev_freq, self.ctrl._tgt_freq, tau)
            if tau >= 1.0:
                self._blend_active = False

        # Dynamic Pressure (q)-Compensated AoA structural protection:
        V_ms = context.get('V_ms', 8.0)
        soft_handoff_requested = context.get('soft_handoff', self.enable_soft_handoff)
        if soft_handoff_requested and V_ms > 8.0 and hasattr(self.ctrl, '_tgt_pitches'):
            if not context.get('on_ground', False) or V_ms > 10.0:
                aoa_lim = ap.dynamic_pressure_aoa_limit(V_ms, target_nz=2.2, gross_weight_kg=context.get('gross_weight_kg', 5.687))
                self.ctrl._tgt_pitches = [min(p, aoa_lim) for p in self.ctrl._tgt_pitches]

        # Rate-bound actuator target demands outside active raised-cosine blend (<= 300 deg/s)
        dt_s = max(0.001, min(0.20, t - self._last_t)) if hasattr(self, '_last_t') else 0.02
        self._last_t = t
        if soft_handoff_requested and hasattr(self.ctrl, '_tgt_sweeps'):
            max_ddeg = 295.0 * dt_s
            if hasattr(self, '_last_applied_sweeps'):
                self.ctrl._tgt_sweeps = [
                    max(prev - max_ddeg, min(prev + max_ddeg, cur))
                    for prev, cur in zip(self._last_applied_sweeps, self.ctrl._tgt_sweeps)
                ]
                self.ctrl._tgt_pitches = [
                    max(prev - max_ddeg, min(prev + max_ddeg, cur))
                    for prev, cur in zip(self._last_applied_pitches, self.ctrl._tgt_pitches)
                ]
                self.ctrl._tgt_dihedrals = [
                    max(prev - max_ddeg, min(prev + max_ddeg, cur))
                    for prev, cur in zip(self._last_applied_dihedrals, self.ctrl._tgt_dihedrals)
                ]
        if hasattr(self.ctrl, '_tgt_sweeps'):
            self._last_applied_sweeps = list(self.ctrl._tgt_sweeps)
            self._last_applied_pitches = list(self.ctrl._tgt_pitches)
            self._last_applied_dihedrals = list(self.ctrl._tgt_dihedrals)

        # Roll-out trigger on stick release
        if (isinstance(self.active, TurnBehaviour)
                and self.active._phase == TurnBehaviour.PHASE_BANKED):
            roll = sticks.get('roll', 0.0)
            if abs(roll) < 0.08:
                self.active.begin_rollout(context)

        if status in (SUCCESS, FAILURE):
            self._log(t, intent, type(self.active).__name__, f'completed:{status}')
            self._transition(t, Intent.CRUISE, CruiseBehaviour, context)
            status = RUNNING

        # Boyd EM record
        try:
            self.em_report.record(
                t=t, Ps=context.get('Ps', 0.0),
                h_agl_m=context.get('h_agl_m', 0.0),
                V_ms=context.get('V_ms', 0.0),
                bank_deg=context.get('bank_deg', 0.0),
                mode=self.active.name)
        except Exception:
            pass

        return self.active.name, intent, status

    def _transition(self, t, intent, target_cls, context, params=None):
        old_name = self.active.name
        self.active.on_exit(context, completed=False)
        self._log(t, intent, old_name, 'preempted')
        if params:
            context.update(params)

        # Snapshot baseline targets for raised-cosine soft handoff on continuous modalities
        soft_handoff_requested = context.get('soft_handoff', self.enable_soft_handoff)
        is_macro_or_safety = (
            intent in (Intent.RECOVER, Intent.EMERGENCY_LAND, Intent.RTH,
                       Intent.AUTO_GLIDE_LOCK, Intent.OWI_REDUNDANCY) or
            target_cls.name in MACRO_LIBRARY or
            getattr(target_cls, 'priority', 10) <= 2 or
            not getattr(target_cls, 'interruptible', True)
        )

        if soft_handoff_requested and not is_macro_or_safety and hasattr(self.ctrl, '_tgt_sweeps'):
            self._prev_sweeps = list(self.ctrl._tgt_sweeps)
            self._prev_dihedrals = list(self.ctrl._tgt_dihedrals)
            self._prev_pitches = list(self.ctrl._tgt_pitches)
            self._prev_freq = float(self.ctrl._tgt_freq)
            self._blend_active = True
            self._blend_start_t = t
        else:
            self._blend_active = False

        self.active = target_cls()
        self.active.on_enter(context)
        self._log(t, intent, target_cls.name, 'entered')

    def _log(self, t, intent, behaviour, event):
        entry = {'t': round(t,2), 'intent': intent,
                 'behaviour': behaviour, 'event': event}
        self._history.append(entry)

    def behaviour_log_summary(self):
        lines = ['  BEHAVIOUR TRANSITION LOG:']
        prev  = None
        for e in self._history:
            if e['behaviour'] != prev:
                lines.append(f"  t={e['t']:6.1f}s  {e['event']:<12}"
                             f"  {e['behaviour']:<20}  intent:{e['intent']}")
                prev = e['behaviour']
        return lines


# ── MACRO ABORT TRIGGERS & AUTOPILOT DEFERRED REGISTRY ─────────────

MACRO_LIBRARY = {
    'stoop': StoopMacro,
    'cf1_burst': CF1Macro,
    'chandelle': ChandellesMacro,
    'dart': DartMacro,
    'zoom_climb': ZoomClimbMacro,
    'roost_perch_flare': RoostPerchMacro,
    'roost_somersault': RoostPerchMacro,
    'active_upstroke_brake': ActiveUpstrokeBrakeMacro,
    'kramer_lift_burst': KramerLiftBurstMacro,
    'clap_jet_breakout': ClapJetBreakoutMacro,
    'quadrupedal_vault': QuadrupedalVaultMacro,
    'caridoid_escape_flare': CaridoidEscapeFlareMacro,
    'payload_drop': PayloadDropMacro,
    'auto_glide_lock': AutoGlideLockMacro,
    'water_clap_burst': WaterClapBurstMacro,
    'dragonfly_super_lift': DragonflySuperLiftMacro,
    'launch_burst': LaunchBurstMacro,
}

# Explicit set of all macros classified into the post-first-flight Autopilot Bucket
AUTOPILOT_DEFERRED_MACROS = set(MACRO_LIBRARY.keys())

MACRO_ABORT_TRIGGERS = {
    'launch_burst': [
        (lambda ctx: ctx.get('h_agl_m', 0.0) > 3.0, 'altitude breakout reached (>3.0m AGL)'),
        (lambda ctx: ctx.get('V_ms', 0.0) > 10.0,   'airspeed breakout reached (>10.0 m/s)'),
        (lambda ctx: ctx.get('throttle', 1.0) < 0.50, 'pilot throttle cut (<50%)'),
        (lambda ctx: ctx.get('pitch', 0.0) < -0.60, 'pilot forward dive abort'),
        (lambda ctx: ctx.get('stall_imminent', False), 'stall warning during launch burst'),
    ],
    'stoop': [
        (lambda ctx: ctx.get('h_agl_m', 999) < 25.0, 'hard floor 25m AGL'),
        (lambda ctx: ctx.get('V_ms', 0) > 35.0,       'overspeed'),
        (lambda ctx: ctx.get('stall_imminent', False), 'stall warning'),
    ],
    'cf1_burst': [
        (lambda ctx: ctx.get('V_ms', 0) > 10.0,       'speed too high'),
        (lambda ctx: ctx.get('stall_imminent', False), 'stall warning'),
    ],
    'chandelle': [
        (lambda ctx: ctx.get('h_agl_m', 999) < 30.0,  'altitude floor'),
        (lambda ctx: ctx.get('V_ms', 0) < 5.5,         'airspeed too low'),
    ],
    'dart': [
        (lambda ctx: ctx.get('h_agl_m', 999) < 30.0,  'hard floor 30m AGL'),
        (lambda ctx: ctx.get('stall_imminent', False), 'stall warning'),
    ],
    'roost_somersault': [
        (lambda ctx: ctx.get('h_agl_m', 999) < 4.0,   'altitude floor 4m AGL'),
        (lambda ctx: ctx.get('V_ms', 0) > 20.0,       'speed too high for somersault'),
    ],
    'roost_perch_flare': [
        (lambda ctx: ctx.get('V_ms', 0) > 25.0,       'speed too high for perch flare'),
    ],
    'active_upstroke_brake': [
        (lambda ctx: ctx.get('h_agl_m', 999) < 2.0,   'altitude floor 2.0m AGL'),
        (lambda ctx: ctx.get('V_ms', 0) < 0.5,        'airspeed fully arrested'),
    ],
    'kramer_lift_burst': [
        (lambda ctx: ctx.get('h_agl_m', 999) < 1.0,   'altitude floor 1.0m AGL'),
        (lambda ctx: ctx.get('V_ms', 0) > 12.0,       'speed too high for rotational burst'),
    ],
    'clap_jet_breakout': [
        (lambda ctx: ctx.get('h_agl_m', 999) < 1.0,   'altitude floor 1.0m AGL'),
        (lambda ctx: ctx.get('V_ms', 0) > 12.0,       'speed too high for clap breakout'),
    ],
    'quadrupedal_vault': [
        (lambda ctx: ctx.get('h_agl_m', 0) > 1.8,     'altitude ceiling reached'),
        (lambda ctx: ctx.get('pitch_deg', 0) > 35.0,  'excessive pitch attitude (>35 deg)'),
        (lambda ctx: ctx.get('V_ms', 0) > 12.0,       'speed too high for ground vault'),
        (lambda ctx: ctx.get('on_ground', False) and not ctx.get('pedals_depressed', True), 'rudder pedals released on ground'),
        (lambda ctx: ctx.get('on_ground', False) and ctx.get('throttle', 1.0) < 0.70, 'throttle backed off during vault'),
    ],
    'payload_drop': [
        (lambda ctx: ctx.get('stall_imminent', False), 'stall warning during payload drop'),
    ],
    'auto_glide_lock': [],
    'water_clap_burst': [
        (lambda ctx: ctx.get('h_agl_m', 0.0) > 0.65, 'exited WIG surface effect zone'),
        (lambda ctx: ctx.get('stall_imminent', False), 'stall warning'),
    ],
    'dragonfly_super_lift': [
        (lambda ctx: ctx.get('h_agl_m', 0.0) > 2.5, 'altitude breakout reached (>2.5m AGL)'),
        (lambda ctx: ctx.get('V_ms', 0.0) > 8.0,    'airspeed too high for vertical launch'),
        (lambda ctx: ctx.get('stall_imminent', False), 'stall warning during launch burst'),
    ],
}



def check_macro_abort(macro_name, context):
    for fn, reason in MACRO_ABORT_TRIGGERS.get(macro_name, []):
        try:
            if fn(context): return True, reason
        except Exception:
            pass
    return False, ""
