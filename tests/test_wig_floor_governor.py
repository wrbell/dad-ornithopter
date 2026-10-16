#!/usr/bin/env python3
"""
tests/test_wig_floor_governor.py
================================
Unit test suite for HardWIGFloorGovernor in mco_flight_governor.py.
Verifies:
  1. Ground Idle: No intervention while on ground or below 0.12m.
  2. Free Flight: Nominal transparent control above WIG ceiling (0.85m).
  3. WIG Buffer Protection (0.35m - 0.65m):
       - Progressive damping of forward stick as altitude approaches 0.35m.
       - Proportional restorative pitch bias & canard lift pulses when descending.
  4. Hard Floor Enforcement (<= 0.35m):
       - Absolute forward stick lockout (stick clamped to >= 0.0).
       - Maximum restorative impulse (+12° pitch, +35N canard lift).
  5. Flare Dissolution (Landability Anticipation):
       - Dissolves floor from 0.35m to 0.0m upon flare detection.
       - Forward stick clamping completely disengaged during flare to enable touchdown.
  6. Balked Landing / Go-Around Reset:
       - Resets floor immediately if flare is aborted and aircraft climbs.
"""

import unittest
import sys
import os

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
FIRMWARE_DIR = os.path.join(REPO_ROOT, 'firmware')
if FIRMWARE_DIR not in sys.path:
    sys.path.insert(0, FIRMWARE_DIR)

from mco_flight_governor import HardWIGFloorGovernor


class TestHardWIGFloorGovernor(unittest.TestCase):

    def setUp(self):
        self.gov = HardWIGFloorGovernor(
            hard_floor_m=0.35,
            buffer_zone_m=0.65,
            wig_ceiling_m=0.85
        )

    def test_ground_idle_state(self):
        """Governor is inactive while on wheels on ground."""
        res = self.gov.update(h_agl_m=0.05, V_ms=5.0, pitch_stick=-0.80, on_ground=True)
        self.assertEqual(res['state'], 'GROUND_IDLE')
        self.assertEqual(res['clamped_pitch_stick'], -0.80)
        self.assertFalse(res['forward_stick_clamped'])
        self.assertFalse(res['pushback_active'])
        self.assertEqual(res['floor_effective_m'], 0.0)

    def test_free_flight_above_wig_ceiling(self):
        """Above 0.85m, pilot has transparent uninhibited control."""
        res = self.gov.update(h_agl_m=1.20, V_ms=10.0, pitch_stick=-0.75, on_ground=False)
        self.assertEqual(res['state'], 'FREE_FLIGHT')
        self.assertEqual(res['clamped_pitch_stick'], -0.75)
        self.assertFalse(res['forward_stick_clamped'])
        self.assertFalse(res['pushback_active'])

    def test_wig_skimming_nominal(self):
        """In ground effect (0.75m), neutral or positive stick is unaffected."""
        res = self.gov.update(h_agl_m=0.75, V_ms=9.0, pitch_stick=0.20, vz_ms=0.0, on_ground=False)
        self.assertEqual(res['state'], 'WIG_ACTIVE')
        self.assertEqual(res['clamped_pitch_stick'], 0.20)
        self.assertFalse(res['forward_stick_clamped'])
        self.assertFalse(res['pushback_active'])

    def test_buffer_zone_progressive_forward_stick_clamping(self):
        """At 0.50m (midway between 0.35m and 0.65m), max forward stick is clamped to -0.50."""
        # Halfway point: ratio = (0.50 - 0.35) / (0.65 - 0.35) = 0.50
        res = self.gov.update(h_agl_m=0.50, V_ms=9.0, pitch_stick=-0.90, vz_ms=0.0, on_ground=False)
        self.assertEqual(res['state'], 'WIG_ACTIVE')
        self.assertTrue(res['forward_stick_clamped'])
        self.assertAlmostEqual(res['clamped_pitch_stick'], -0.50, places=2)

    def test_buffer_zone_sink_rate_pushback(self):
        """Descending at -0.8 m/s in buffer zone triggers restorative pitch and canard lift."""
        res = self.gov.update(h_agl_m=0.45, V_ms=9.0, pitch_stick=0.0, vz_ms=-0.8, on_ground=False)
        self.assertEqual(res['state'], 'WIG_ACTIVE')
        self.assertTrue(res['pushback_active'])
        self.assertGreater(res['restorative_pitch_deg'], 3.0)
        self.assertGreater(res['canard_lift_boost_n'], 5.0)

    def test_hard_floor_lockout(self):
        """At or below 0.35m, forward stick is 100% locked out (clamped to 0.0)."""
        res = self.gov.update(h_agl_m=0.34, V_ms=9.0, pitch_stick=-0.60, vz_ms=-0.5, on_ground=False)
        self.assertEqual(res['state'], 'RESTRICTED_PUSHBACK')
        self.assertTrue(res['forward_stick_clamped'])
        self.assertEqual(res['clamped_pitch_stick'], 0.0)
        self.assertTrue(res['pushback_active'])
        self.assertEqual(res['canard_lift_boost_n'], 35.0)
        self.assertGreaterEqual(res['restorative_pitch_deg'], 8.0)

    def test_flare_dissolution_allows_touchdown(self):
        """Initiating a landing flare dissolves the floor smoothly to 0.0m."""
        # Step 1: Detect flare at 0.50m
        res1 = self.gov.update(h_agl_m=0.50, V_ms=7.0, pitch_stick=0.50, throttle=0.20,
                              on_ground=False, is_flare_active=True, dt_s=0.05)
        self.assertEqual(res1['state'], 'FLARE_DISSOLUTION')
        self.assertFalse(res1['forward_stick_clamped'])
        self.assertLess(res1['floor_effective_m'], 0.35)

        # Step 2: Progress through 0.25s dissolution time
        for _ in range(5):
            res_decay = self.gov.update(h_agl_m=0.30, V_ms=5.0, pitch_stick=0.60, throttle=0.10,
                                        on_ground=False, is_flare_active=True, dt_s=0.05)
        self.assertEqual(res_decay['floor_effective_m'], 0.0)
        self.assertTrue(res_decay['flare_dissolved'])

        # Step 3: Aircraft descends below old hard floor (0.20m) without any pushback or lockout
        res_td = self.gov.update(h_agl_m=0.18, V_ms=3.0, pitch_stick=0.10, throttle=0.05,
                                 on_ground=False, is_flare_active=True, dt_s=0.02)
        self.assertFalse(res_td['forward_stick_clamped'])
        self.assertFalse(res_td['pushback_active'])
        self.assertEqual(res_td['floor_effective_m'], 0.0)

    def test_balked_landing_go_around_re_arms_floor(self):
        """If flare is aborted and aircraft climbs, floor resets to 0.35m."""
        # Start in flare
        self.gov.update(h_agl_m=0.40, V_ms=6.0, on_ground=False, is_flare_active=True, dt_s=0.10)
        self.assertEqual(self.gov.state, 'FLARE_DISSOLUTION')

        # Go-around initiated: full throttle, climb, flare False
        res_ga = self.gov.update(h_agl_m=0.50, V_ms=8.0, pitch_stick=0.30, throttle=1.0,
                                 on_ground=False, is_flare_active=False, intent='climb_out', dt_s=0.02)
        self.assertEqual(res_ga['floor_effective_m'], 0.35)
        self.assertFalse(res_ga['flare_dissolved'])


if __name__ == '__main__':
    unittest.main()
