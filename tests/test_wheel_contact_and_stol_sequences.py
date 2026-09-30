#!/usr/bin/env python3
"""
tests/test_wheel_contact_and_stol_sequences.py
==============================================
Verification and Red Team Adversarial Test Suite for:
  1. 4-Way Gated Voting Touchdown & Wheel Contact Detector (WheelContactDetector)
  2. Anti-Bounce Touchdown Latch & Pilot Abort Preemption
  3. Ground-Roll Slipstream Ingestion (Item 3, Takeoff Thrust Multiplier)
  4. Trailing-Edge Differential Reflex Decambering on Touchdown (Item 4, Downforce & Anti-Nose-Over)
"""

import math
import os
import sys
import unittest

# Ensure firmware is in sys.path
REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
FIRMWARE_DIR = os.path.join(REPO_ROOT, 'firmware')
if FIRMWARE_DIR not in sys.path:
    sys.path.insert(0, FIRMWARE_DIR)

import mco_advanced_physics as ap


class TestWheelContactDetector(unittest.TestCase):
    """Verifies the 4-way gated voting touchdown and wheel contact detector."""

    def setUp(self):
        self.detector = ap.WheelContactDetector()

    def test_01_clean_grease_landing_detection(self):
        """A soft, grease landing with zero G-spike reliably latches touchdown."""
        # 1. Approach descent (1.0m AGL, descending at -0.5 m/s)
        res1 = self.detector.update(
            h_agl_m=1.0,
            vz_ms=-0.5,
            gyro_rates_rads=(0.05, 0.05, 0.0),
            pitch_deg=5.0,
            throttle=0.10,
            intent='landing_approach',
        )
        self.assertFalse(res1['contact_confirmed'])
        self.assertFalse(res1['latched'])
        self.assertEqual(res1['status'], 'AIRBORNE')

        # 2. Wheel contact conditions established:
        # h = 0.115m (on wheels), Vz arrested to 0.02 m/s, angular rates constrained, throttle idle
        for cycle in range(1, 3):
            res_p = self.detector.update(
                h_agl_m=0.115,
                vz_ms=0.02,
                gyro_rates_rads=(0.04, 0.03, 0.01),
                pitch_deg=4.0,
                throttle=0.05,
                intent='flare',
            )
            self.assertTrue(all(res_p['gates_passed'].values()), f"Failed on cycle {cycle}")
            if cycle < 3:
                self.assertFalse(res_p['latched'])
                self.assertEqual(res_p['status'], 'PERSISTING')

        # 3. Third consecutive cycle (60ms persistence) latches confirmed ground contact
        res_latched = self.detector.update(
            h_agl_m=0.115,
            vz_ms=0.01,
            gyro_rates_rads=(0.03, 0.02, 0.01),
            pitch_deg=4.0,
            throttle=0.05,
            intent='touchdown',
        )
        self.assertTrue(res_latched['contact_confirmed'])
        self.assertTrue(res_latched['latched'])
        self.assertEqual(res_latched['status'], 'LATCHED_ON_GROUND')

    def test_02_turbulence_mid_air_spike_rejection(self):
        """In-flight turbulence or stroke reversal at high AGL cannot trigger touchdown."""
        # High AGL (1.5m), zero sink rate momentarily (in thermal/flare peak)
        res = self.detector.update(
            h_agl_m=1.5,
            vz_ms=0.01,
            gyro_rates_rads=(0.05, 0.05, 0.0),
            pitch_deg=10.0,
            throttle=0.15,
            intent='flare',
        )
        self.assertFalse(res['gates_passed']['altitude'])
        self.assertFalse(res['contact_confirmed'])
        self.assertFalse(res['latched'])
        self.assertEqual(res['status'], 'AIRBORNE')

    def test_03_high_sink_rate_rejection(self):
        """Low altitude with un-arrested descent (passing through 0.11m) does not trigger."""
        # Fast descent (-0.8 m/s) passing near ground (not settled)
        res = self.detector.update(
            h_agl_m=0.11,
            vz_ms=-0.80,
            gyro_rates_rads=(0.05, 0.05, 0.0),
            pitch_deg=5.0,
            throttle=0.05,
            intent='flare',
        )
        self.assertTrue(res['gates_passed']['altitude'])
        self.assertFalse(res['gates_passed']['sink_arrest'])
        self.assertFalse(res['contact_confirmed'])

    def test_04_high_speed_low_pass_lockout(self):
        """High-speed low pass with open throttle does not trigger touchdown."""
        res = self.detector.update(
            h_agl_m=0.10,
            vz_ms=0.01,
            gyro_rates_rads=(0.02, 0.02, 0.0),
            pitch_deg=0.0,
            throttle=0.75,  # High power
            intent='cruise',
        )
        self.assertFalse(res['gates_passed']['intent_throttle'])
        self.assertFalse(res['contact_confirmed'])

    def test_05_angular_rate_dither_rejection(self):
        """Excessive unconstrained rolling/pitching in flight delays or blocks contact latch."""
        res = self.detector.update(
            h_agl_m=0.11,
            vz_ms=0.02,
            gyro_rates_rads=(0.65, 0.40, 0.10),  # High angular rate
            pitch_deg=5.0,
            throttle=0.05,
            intent='flare',
        )
        self.assertFalse(res['gates_passed']['angular_damping'])
        self.assertFalse(res['contact_confirmed'])

    def test_06_anti_bounce_latch_hysteresis(self):
        """After touchdown is latched, small elastic rebounds do not toggle the state."""
        # 1. Latch touchdown over 3 cycles
        for _ in range(3):
            self.detector.update(
                h_agl_m=0.115,
                vz_ms=0.01,
                gyro_rates_rads=(0.02, 0.02, 0.0),
                pitch_deg=4.0,
                throttle=0.05,
                intent='touchdown',
                dt=0.02,
            )

        # 2. Tire rebounds 5cm into the air (h = 0.165m, Vz = +0.15 m/s) for 0.5s
        res_rebound = self.detector.update(
            h_agl_m=0.165,
            vz_ms=0.15,
            gyro_rates_rads=(0.10, 0.05, 0.0),
            pitch_deg=6.0,
            throttle=0.0,
            intent='rollout',
            dt=0.02,
        )
        self.assertTrue(res_rebound['latched'])
        self.assertTrue(res_rebound['contact_confirmed'])
        self.assertEqual(res_rebound['status'], 'LATCHED_ON_GROUND')

    def test_07_pilot_abort_and_go_around_preemption(self):
        """Pilot pulling aft stick (>0.25) or firewalling throttle immediately unlatches."""
        # 1. Latch touchdown
        for _ in range(3):
            self.detector.update(
                h_agl_m=0.115,
                vz_ms=0.01,
                gyro_rates_rads=(0.02, 0.02, 0.0),
                pitch_deg=4.0,
                throttle=0.05,
                intent='touchdown',
            )
        self.assertTrue(self.detector._latched)

        # 2. Pilot firewalls throttle to 0.85 and pulls aft stick 0.35 (Balked Landing / Go-Around)
        res_abort = self.detector.update(
            h_agl_m=0.12,
            vz_ms=0.0,
            gyro_rates_rads=(0.02, 0.02, 0.0),
            pitch_deg=4.0,
            throttle=0.85,
            pitch_stick=0.35,
            intent='go_around',
        )
        self.assertFalse(res_abort['latched'])
        self.assertFalse(res_abort['contact_confirmed'])
        self.assertTrue(res_abort['abort_requested'])
        self.assertEqual(res_abort['status'], 'ABORTED')

    def test_08_convenience_function_wrapper(self):
        """detect_wheel_contact convenience function operates cleanly."""
        res = ap.detect_wheel_contact(
            h_agl_m=0.115,
            vz_ms=0.01,
            gyro_rates_rads=(0.02, 0.02, 0.0),
            pitch_deg=3.0,
            throttle=0.05,
            intent='landing',
        )
        self.assertIn('contact_confirmed', res)
        self.assertIn('gates_passed', res)


class TestGroundRollSlipstreamIngestion(unittest.TestCase):
    """Verifies Item 3: Ground-Roll Slipstream Ingestion (Takeoff Thrust Multiplier)."""

    def test_short_field_regime_parameters(self):
        """STOL short-field takeoff regime defines 20° cascade phase and 1.25x thrust gain."""
        cfg = ap.TAKEOFF_REGIMES['short_field']
        self.assertEqual(cfg['cascade_phase_deg'], 20.0)
        self.assertTrue(cfg['slipstream_ingestion'])
        self.assertEqual(cfg['thrust_gain_factor'], 1.25)
        self.assertEqual(cfg['sweep_deg'], 55.0)

    def test_takeoff_roll_slipstream_kinematics(self):
        """ctol_takeoff_roll_kinematics sets 20° cascade phase and reports +25% thrust gain."""
        cmds, freq_hz, diag = ap.ctol_takeoff_roll_kinematics(
            throttle_frac=1.0,
            V_ms=1.5,
            regime='short_field',
        )
        self.assertEqual(len(cmds), 8)
        self.assertEqual(freq_hz, 2.5)
        self.assertEqual(diag['cascade_phase_deg'], 20.0)
        self.assertTrue(diag['slipstream_ingestion_active'])
        self.assertEqual(diag['thrust_gain_factor'], 1.25)
        self.assertEqual(diag['static_thrust_boost_pct'], 25.0)

        for c in cmds:
            self.assertEqual(c['cascade_phase_deg'], 20.0)
            self.assertEqual(c['sweep_deg'], 55.0)

    def test_standard_takeoff_maintains_baseline_phase(self):
        """Standard CTOL takeoff maintains 90° cascade phase and 1.0x baseline thrust."""
        cmds, freq_hz, diag = ap.ctol_takeoff_roll_kinematics(
            throttle_frac=1.0,
            V_ms=1.5,
            regime='standard',
        )
        self.assertEqual(diag['cascade_phase_deg'], 90.0)
        self.assertFalse(diag['slipstream_ingestion_active'])
        self.assertEqual(diag['thrust_gain_factor'], 1.0)
        self.assertEqual(diag['static_thrust_boost_pct'], 0.0)


class TestTouchdownReflexDecambering(unittest.TestCase):
    """Verifies Item 4: Trailing-Edge Differential Reflex Decambering on Touchdown."""

    def test_short_field_landing_regime_reflex_config(self):
        """Short-field landing regime defines differential reflex decambering angles."""
        cfg = ap.LANDING_REGIMES['short_field']
        self.assertTrue(cfg.get('reflex_decambering'))
        self.assertEqual(cfg.get('front_reflex_aoa_deg'), -3.5)
        self.assertEqual(cfg.get('rear_reflex_aoa_deg'), -8.0)

    def test_flare_kinematics_rollout_reflex(self):
        """In rollout/touchdown, short_field applies differential reflex decambering."""
        cmds, freq_hz, diag = ap.conventional_landing_flare_kinematics(
            pitch_stick=0.0,
            throttle=0.0,
            h_agl_m=0.08,  # Rollout phase
            V_ms=5.0,
            landing_regime='short_field',
        )
        self.assertEqual(diag['phase'], 'ROLLOUT')
        self.assertTrue(diag['differential_reflex_active'])
        self.assertTrue(diag['nose_over_protected'])
        self.assertTrue(diag['aerodynamic_downforce_active'])

        # Front stations (0..3) at -3.5°, Rear stations (4..7) at -8.0°
        front_pitches = [cmds[i]['pitch_deg'] for i in range(4)]
        rear_pitches = [cmds[i]['pitch_deg'] for i in range(4, 8)]

        for p in front_pitches:
            self.assertAlmostEqual(p, -3.5, places=1)
        for p in rear_pitches:
            self.assertAlmostEqual(p, -8.0, places=1)

    def test_soft_field_preserves_nose_high_hold_off(self):
        """Soft-field landing preserves nosewheel hold-off (+14.5° canard pitch) without reflex."""
        cmds, freq_hz, diag = ap.conventional_landing_flare_kinematics(
            pitch_stick=0.0,
            throttle=0.0,
            h_agl_m=0.08,
            V_ms=6.0,
            landing_regime='soft_field',
        )
        self.assertEqual(diag['phase'], 'ROLLOUT')
        self.assertFalse(diag['differential_reflex_active'])
        front_pitches = [cmds[i]['pitch_deg'] for i in range(4)]
        for p in front_pitches:
            self.assertGreaterEqual(p, 14.0)

    def test_rollout_braking_freewheel_reflex_downforce(self):
        """Freewheel coasting in short_field commands differential reflex and downforce."""
        cmds, freq_hz, diag = ap.aerodynamic_rollout_braking_kinematics(
            lever_pct=55.0,
            throttle=0.0,
            V_ms=7.0,
            regime='short_field',
        )
        self.assertEqual(diag['modality'], 'FREEWHEEL_COAST')
        self.assertTrue(diag['differential_reflex_active'])
        self.assertEqual(diag['front_reflex_aoa_deg'], -3.5)
        self.assertEqual(diag['rear_reflex_aoa_deg'], -8.0)
        self.assertTrue(diag['nose_over_protected'])
        self.assertGreater(diag['aerodynamic_downforce_n'], 9.0)

        # Confirm front wings at -3.5° and rear wings at -8.0°
        front_pitches = [cmds[i]['pitch_deg'] for i in range(4)]
        rear_pitches = [cmds[i]['pitch_deg'] for i in range(4, 8)]
        for p in front_pitches:
            self.assertAlmostEqual(p, -3.5, places=1)
        for p in rear_pitches:
            self.assertAlmostEqual(p, -8.0, places=1)


if __name__ == '__main__':
    unittest.main()
