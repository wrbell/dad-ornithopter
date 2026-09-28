#!/usr/bin/env python3
"""
tests/test_bush_plane_stol_suite.py

Comprehensive validation test suite for STOL (Short Takeoff and Landing)
Bush Plane handling qualities, control laws, and physical kinematics.

Validates:
  1. STOL Takeoff Roll: Maximum thrust generation, forward rake sweep (55°),
     rapid amplitude buildup, and ground roll acceleration.
  2. STOL Unstick / Flap-Pop Rotation: Prompt rotation at Vr (3.2 - 3.8 m/s),
     Canard Direct Lift (W1-W2 AoA +15°, amplitude +18%), tail clearance >= 150mm.
  3. Ground-Effect Skim & Obstacle Clearance Climb: Hand-off to climb/acceleration
     once unstick target height is reached.
  4. STOL Approach Behind the Power Curve: Region of reversed command where
     pitch stick controls AoA/airspeed (AoA ~ 9.5° - 13°), and throttle controls
     sink rate / vertical thrust (frequency 1.8 - 2.2 Hz) while hanging on the wash.
  5. STOL Spot Landing & "Chop-and-Drop": At roundout / spot threshold, cutting
     throttle dumps flapping frequency to 0.0 Hz, extinguishing induced lift
     to plant wheels firmly without floating.
  6. Aerodynamic Rollout Braking (No Wheel Brakes): Outboard Shuttlecock bluff cup,
     active reverse thrust flapping, and compound emergency stop with anti-nose-over canard.
  7. Baulked Landing / Touch-and-Go Preemption: Advancing throttle or lever aborts
     rollout braking immediately and commands unstick thrust.
  8. IntentDispatcher End-to-End Pipeline: Verifies full pilot control flow.
"""

import os
import sys
import math
import unittest

# Ensure firmware directory is in path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'firmware'))

import mco_advanced_physics as ap
import mco_intent_control as ic


class MockFlightController:
    """Mock flight controller tracking output states."""
    def __init__(self):
        self.freq_hz = 0.0
        self._tgt_freq = 0.0
        self._tgt_pitches = [0.0] * 8
        self._tgt_sweeps = [0.0] * 8
        self._tgt_dihedrals = [0.0] * 8
        self._intent_name = 'idle'
        self.flare_diag = {}
        self.rollout_diag = {}


class TestBushPlaneSTOLSuite(unittest.TestCase):
    """Validation test suite for STOL Bush Plane flight dynamics."""

    def test_01_stol_regimes_and_aliases_exist(self):
        """Verify STOL regimes and aliases exist in both takeoff and landing dictionaries."""
        # Takeoff regimes
        self.assertIn('short_field', ap.TAKEOFF_REGIMES)
        self.assertIn('stol', ap.TAKEOFF_REGIMES)
        self.assertIn('stol_takeoff', ap.TAKEOFF_REGIMES)
        self.assertEqual(ap.TAKEOFF_REGIMES['stol']['Vr_ms'], 3.8)
        self.assertEqual(ap.TAKEOFF_REGIMES['stol']['sweep_deg'], 55.0)

        # Landing regimes
        self.assertIn('short_field', ap.LANDING_REGIMES)
        self.assertIn('stol', ap.LANDING_REGIMES)
        self.assertIn('stol_landing', ap.LANDING_REGIMES)
        self.assertIn('stol_spot', ap.LANDING_REGIMES)
        self.assertTrue(ap.LANDING_REGIMES['stol']['chop_and_drop'])
        self.assertGreater(ap.LANDING_REGIMES['stol']['approach_freq_hz'], 1.5)

    def test_02_stol_takeoff_roll_kinematics(self):
        """Verify STOL takeoff roll commands forward rake sweep and high frequency."""
        throttle = 1.0  # Full power
        V_ms = 2.0      # Early roll
        cmds, freq_hz, diag = ap.ctol_takeoff_roll_kinematics(
            throttle_frac=throttle,
            V_ms=V_ms,
            regime='stol'
        )
        self.assertEqual(len(cmds), 8)
        self.assertEqual(freq_hz, 2.5)  # Max flapping frequency
        # Sweep should be 55° (forward rake for high static thrust)
        for c in cmds:
            self.assertEqual(c['sweep_deg'], 55.0)
        self.assertEqual(diag['status'], 'TAKEOFF_ROLL_ACCELERATING')
        self.assertFalse(diag['ready_to_rotate'])

    def test_03_stol_unstick_rotation_and_tail_clearance(self):
        """Verify STOL rotation delivers Canard Direct Lift and protects tail clearance."""
        pitch_stick = 1.0  # Full aft stick to unstick
        V_ms = 3.8         # At Vr
        cmds, freq_hz, diag = ap.ctol_dual_cascade_rotation(
            pitch_stick=pitch_stick,
            V_ms=V_ms,
            regime='stol'
        )
        self.assertEqual(freq_hz, 2.5)
        # Front cascade (W1-W2): Canard Direct Lift
        front_pitches = [cmds[i]['pitch_deg'] for i in range(4)]
        rear_pitches = [cmds[i]['pitch_deg'] for i in range(4, 8)]
        front_mean = sum(front_pitches) / len(front_pitches)
        rear_mean = sum(rear_pitches) / len(rear_pitches)

        self.assertAlmostEqual(front_mean, 15.0, places=1)
        self.assertAlmostEqual(rear_mean, 8.0, places=1)
        self.assertTrue(diag['canard_lift_priority'])
        self.assertTrue(diag['tail_strike_prevented'])
        self.assertEqual(diag['unstick_target_m'], 0.30)

    def test_04_stol_approach_behind_power_curve_kinematics(self):
        """Verify STOL approach maintains high AoA trim and active carried power."""
        pitch_stick = 0.20  # Fine-tune airspeed with stick
        throttle = 0.40     # Carrying power behind the power curve
        h_agl = 8.0         # On approach
        V_ms = 5.8          # Slow flight (behind power curve)

        cmds, freq_hz, diag = ap.conventional_landing_flare_kinematics(
            pitch_in=pitch_stick,
            throttle=throttle,
            h_agl_m=h_agl,
            V_ms=V_ms,
            regime='stol'
        )
        self.assertEqual(diag['phase'], 'APPROACH')
        # Active carried power: frequency > 1.5 Hz
        self.assertGreater(freq_hz, 1.5)
        # Higher base AoA for slow flight (> 9.0°)
        self.assertGreaterEqual(diag['front_aoa_deg'], 9.5)
        # Forward rake sweep (52°)
        for c in cmds:
            self.assertEqual(c['sweep_deg'], 52.0)

    def test_05_stol_chop_and_drop_spot_landing(self):
        """Verify cutting throttle at flare dumps flapping frequency to 0 Hz for zero float."""
        # 1. Carrying power on short final
        cmds_pow, freq_pow, diag_pow = ap.conventional_landing_flare_kinematics(
            pitch_in=0.20,
            throttle=0.35,
            h_agl_m=0.80,
            V_ms=5.5,
            regime='stol'
        )
        self.assertEqual(diag_pow['phase'], 'FLARE')
        self.assertGreater(freq_pow, 1.5)  # Flapping active while carrying power

        # 2. Pilot chops throttle to idle right over the spot (throttle = 0.05)
        cmds_chop, freq_chop, diag_chop = ap.conventional_landing_flare_kinematics(
            pitch_in=0.20,
            throttle=0.05,
            h_agl_m=0.80,
            V_ms=5.5,
            regime='stol'
        )
        self.assertEqual(diag_chop['phase'], 'FLARE')
        # Frequency cuts to 0.0 Hz immediately: induced lift is dumped!
        self.assertEqual(freq_chop, 0.0)
        # Tail clearance is preserved
        self.assertGreaterEqual(diag_chop['tail_clearance_w4_m'], 0.15)
        self.assertTrue(diag_chop['tail_strike_safe'])

    def test_06_stol_aerodynamic_rollout_braking(self):
        """Verify aerodynamic braking stops the aircraft without wheel brakes."""
        # Shuttlecock airbrake (16-28% lever)
        cmds_shuttle, freq_shuttle, diag_shuttle = ap.aerodynamic_rollout_braking_kinematics(
            lever_pct=22.0,
            throttle=0.0,
            V_ms=6.0,
            regime='stol'
        )
        self.assertEqual(diag_shuttle['modality'], 'SHUTTLECOCK_AIRBRAKE')
        self.assertEqual(freq_shuttle, 0.0)
        self.assertGreaterEqual(diag_shuttle['retarding_force_n'], 20.0)
        self.assertGreaterEqual(diag_shuttle['decel_g'], 0.30)
        # Anti-nose-over canard incidence (front pitches elevated above 0)
        self.assertGreater(cmds_shuttle[0]['pitch_deg'], 0.0)

        # Compound max stop (< 6% lever)
        cmds_stop, freq_stop, diag_stop = ap.aerodynamic_rollout_braking_kinematics(
            lever_pct=3.0,
            throttle=0.0,
            V_ms=6.0,
            regime='stol'
        )
        self.assertEqual(diag_stop['modality'], 'COMPOUND_MAX_STOP')
        self.assertGreater(freq_stop, 2.0)
        self.assertGreaterEqual(diag_stop['decel_g'], 1.0)
        self.assertGreaterEqual(diag_stop['retarding_force_n'], 60.0)

    def test_07_stol_touch_and_go_preemption(self):
        """Verify advancing throttle or lever immediately cancels rollout braking."""
        # Advancing throttle to 80% on the runway
        cmds_tg, freq_tg, diag_tg = ap.aerodynamic_rollout_braking_kinematics(
            lever_pct=20.0,  # Lever was in airbrake
            throttle=0.80,   # Pilot pushes throttle to firewall
            V_ms=4.0,
            regime='stol'
        )
        self.assertEqual(diag_tg['modality'], 'TOUCH_AND_GO')
        self.assertTrue(diag_tg['touch_and_go_ready'])
        self.assertGreater(freq_tg, 2.0)

    def test_08_intent_interpreter_stol_takeoff(self):
        """Verify IntentInterpreter routes STOL ground inputs correctly."""
        interp = ic.IntentInterpreter()

        # 1. Takeoff roll with throttle 90%, neutral stick
        pilot_channels_roll = {'ch1': 0.0, 'ch2': 0.0, 'ch3': 0.90, 'ch4': 0.0}
        switches = {}
        context_roll = {
            'on_ground': True,
            'h_agl_m': 0.0,
            'V_ms': 2.0,
            'takeoff_regime': 'stol',
        }
        intent, conf, params = interp.interpret(pilot_channels_roll, switches, context_roll)
        self.assertEqual(intent, ic.Intent.TAKEOFF_ROLL)
        self.assertEqual(params['regime'], 'stol')

        # 2. Pilot pulls stick (ch2 = 0.5) at Vr = 3.3 m/s -> ROTATE
        pilot_channels_rot = {'ch1': 0.0, 'ch2': 0.50, 'ch3': 0.90, 'ch4': 0.0}
        context_rot = {
            'on_ground': True,
            'h_agl_m': 0.0,
            'V_ms': 3.3,  # Above STOL vr_threshold of 3.2
            'takeoff_regime': 'stol',
        }
        intent_rot, conf_rot, params_rot = interp.interpret(pilot_channels_rot, switches, context_rot)
        self.assertEqual(intent_rot, ic.Intent.ROTATE)
        self.assertEqual(params_rot['regime'], 'stol')

    def test_09_intent_interpreter_stol_approach_and_chop(self):
        """Verify IntentInterpreter detects STOL approach and low-altitude chop."""
        interp = ic.IntentInterpreter()

        # 1. STOL approach: 20m AGL, slow flight V=6.0 m/s, carrying 50% power
        pilot_channels_app = {'ch1': 0.0, 'ch2': 0.0, 'ch3': 0.50, 'ch4': 0.0}
        switches = {}
        context_app = {
            'on_ground': False,
            'h_agl_m': 20.0,
            'V_ms': 6.0,
            'landing_regime': 'stol',
        }
        intent_app, conf_app, params_app = interp.interpret(pilot_channels_app, switches, context_app)
        self.assertEqual(intent_app, ic.Intent.APPROACH_NORMAL)
        self.assertEqual(params_app['regime'], 'stol')

        # 2. Low altitude (h=1.0m), pilot initiates flare / chop
        pilot_channels_flare = {'ch1': 0.0, 'ch2': 0.25, 'ch3': 0.05, 'ch4': 0.0}
        context_flare = {
            'on_ground': False,
            'h_agl_m': 1.0,
            'V_ms': 5.2,
            'landing_regime': 'stol',
        }
        intent_flare, conf_flare, params_flare = interp.interpret(pilot_channels_flare, switches, context_flare)
        self.assertEqual(intent_flare, ic.Intent.LANDING_FLARE)
        self.assertEqual(params_flare['regime'], 'stol')

    def test_10_dispatcher_end_to_end_stol_pipeline(self):
        """Verify BehaviourDispatcher executes Approach -> Flare -> Rollout for STOL."""
        ctrl = MockFlightController()
        disp = ic.BehaviourDispatcher(ctrl)

        # 1. STOL Approach (active flapping with carried throttle)
        ctx_app = {
            'throttle': 0.40,
            'pitch': 0.10,
            'V_ms': 5.8,
            'h_agl_m': 15.0,
            'regime': 'stol',
            'crosswind_ms': 0.0,
            'on_ground': False,
        }
        disp.update(t=1.0, sticks={'pitch': 0.10, 'throttle': 0.40}, switches={}, context=ctx_app)
        self.assertIsInstance(disp.active, ic.ApproachBehaviour)
        self.assertGreater(ctrl._tgt_freq, 1.5)
        # Front wing pitch trim should reflect slow-flight backside trim (> 9.0°)
        self.assertGreaterEqual(ctrl._tgt_pitches[0], 9.5)

        # 2. STOL Flare with chop (throttle = 0.0)
        ctx_flare = {
            'throttle': 0.0,
            'pitch': 0.35,
            'h_agl_m': 0.8,
            'V_ms': 5.2,
            'regime': 'stol',
            'crosswind_ms': 0.0,
            'on_ground': False,
        }
        disp.update(t=2.0, sticks={'pitch': 0.35, 'throttle': 0.0}, switches={}, context=ctx_flare)
        self.assertIsInstance(disp.active, ic.FlareBehaviour)
        # Chop dumps frequency immediately
        self.assertEqual(ctrl._tgt_freq, 0.0)
        self.assertTrue(ctrl.flare_diag['tail_strike_safe'])

        # 3. Touchdown & Aerodynamic Rollout Stop (lever = 5%, compound max stop)
        ctx_rollout = {
            'lever_pct': 5.0,
            'throttle': 0.0,
            'V_ms': 4.5,
            'regime': 'stol',
            'on_ground': True,
            'h_agl_m': 0.05,
        }
        disp.update(t=3.0, sticks={'pitch': 0.0, 'throttle': 0.0}, switches={}, context=ctx_rollout)
        self.assertIsInstance(disp.active, ic.LandingRolloutBehaviour)
        self.assertEqual(ctrl.rollout_diag['modality'], 'COMPOUND_MAX_STOP')
        self.assertGreaterEqual(ctrl.rollout_diag['decel_g'], 1.0)


if __name__ == '__main__':
    unittest.main()
