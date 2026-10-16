#!/usr/bin/env python3
"""
tests/test_sweep_governor_hypotheses.py
======================================
Comprehensive verification suite for:
  1. H-SWEEP-VNE-1: Aeroelastic Sweep-Adaptive VNE Schedule
  2. H-SWEEP-PULL-1: Boyd Energy-Maneuverability Dynamic Wing Unfolding Sequencer
  3. Integrated DiveRecoveryGovernor sweep adaptation and envelope protection
  4. Integration with wave library hypotheses and ALSS Curriculum C
"""

import math
import os
import sys
import unittest

# Ensure firmware path is accessible
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'firmware')))

import mco_flight_governor as fg
import mco_wave_library as wl


class TestSweepAdaptiveVne(unittest.TestCase):
    """Verifies H-SWEEP-VNE-1: Aeroelastic wash-out relief dynamic VNE schedule."""

    def test_baseline_unswept_vne(self):
        """At baseline sweep (60° or 0° relative), VNE is pegged at baseline 28.0 m/s."""
        self.assertEqual(fg.compute_sweep_vne(60.0), 28.0)
        self.assertEqual(fg.compute_sweep_vne(55.0), 28.0)
        self.assertEqual(fg.compute_sweep_vne(0.0), 28.0)

    def test_moderate_sweep_vne(self):
        """Moderate sweep (74° to 90°) scales VNE smoothly above 28.0 m/s."""
        vne_74 = fg.compute_sweep_vne(74.0)
        vne_85 = fg.compute_sweep_vne(85.0)
        vne_90 = fg.compute_sweep_vne(90.0)

        self.assertGreater(vne_74, 28.0)
        self.assertGreater(vne_85, vne_74)
        self.assertGreater(vne_90, vne_85)
        self.assertAlmostEqual(vne_90, 31.0, delta=0.5)

    def test_high_sweep_stoop_vne(self):
        """At high sweep (110° to 135°), VNE expands up to 38-45 m/s."""
        vne_110 = fg.compute_sweep_vne(110.0)
        vne_135 = fg.compute_sweep_vne(135.0)
        vne_180 = fg.compute_sweep_vne(180.0)

        self.assertGreaterEqual(vne_110, 37.0)
        self.assertEqual(vne_135, 45.0)
        self.assertEqual(vne_180, 45.0)

    def test_relative_sweep_input(self):
        """Accepts relative sweep angle directly (0° to 80°)."""
        self.assertEqual(fg.compute_sweep_vne(0.0, is_relative=True), 28.0)
        self.assertAlmostEqual(fg.compute_sweep_vne(30.0, is_relative=True), 31.0, delta=0.5)
        self.assertEqual(fg.compute_sweep_vne(75.0, is_relative=True), 45.0)


class TestBoydUnfoldingSequencer(unittest.TestCase):
    """Verifies H-SWEEP-PULL-1: Boyd Dynamic Wing Unfolding Sequencer."""

    def test_safe_speed_unfolding_permitted(self):
        """When airspeed is at or below safe entry speed, full unfolding is permitted."""
        sched = fg.compute_sweep_unfolding_schedule(
            V_ms=14.0, n_cmd=2.0, bank_deg=0.0, current_sweep_deg=85.0
        )
        self.assertTrue(sched['unfold_permitted'])
        self.assertEqual(sched['target_sweep_deg'], 60.0)
        self.assertEqual(sched['status'], 'PULLOUT_UNFOLD_PERMITTED_SAFE_SPEED')

    def test_high_speed_unfolding_constrained(self):
        """At high airspeed (40 m/s), wings are constrained to remain swept to protect servos."""
        sched = fg.compute_sweep_unfolding_schedule(
            V_ms=40.0, n_cmd=3.0, bank_deg=0.0, current_sweep_deg=110.0
        )
        self.assertFalse(sched['unfold_permitted'])
        self.assertTrue(sched['unfold_constrained'])
        self.assertGreater(sched['target_sweep_deg'], 100.0)
        self.assertEqual(sched['status'], 'PULLOUT_SWEEP_HOLD_HIGH_SPEED')

        # Normal component of airspeed must not exceed safe speed
        v_safe = sched['v_safe_ms']
        lambda_eff_deg = sched['min_rel_sweep_deg'] * 0.85
        v_normal = 40.0 * math.cos(math.radians(lambda_eff_deg))
        self.assertLessEqual(v_normal, v_safe + 0.1)

    def test_slew_rate_limiting(self):
        """Wrist servo unfolding respects max slew rate limit (120 deg/s)."""
        dt = 0.02
        sched = fg.compute_sweep_unfolding_schedule(
            V_ms=14.0, n_cmd=2.0, current_sweep_deg=100.0,
            max_slew_dps=120.0, dt=dt
        )
        # In 0.02s, max change is 120 * 0.02 = 2.4 deg
        expected_slew = 100.0 - 2.4
        self.assertAlmostEqual(sched['slew_limited_sweep_deg'], expected_slew, places=1)


class TestDiveRecoveryGovernorSweepIntegration(unittest.TestCase):
    """Verifies integrated DiveRecoveryGovernor with sweep-adaptive features."""

    def setUp(self):
        self.gov = fg.DiveRecoveryGovernor()

    def test_high_speed_swept_dash_does_not_trip_baseline_vne(self):
        """At 35 m/s with 110° sweep, VNE is expanded (38 m/s), avoiding false overspeed trips."""
        telem = self.gov.tick(
            V_ms=35.0, pitch_cmd_deg=0.0, bank_deg=0.0,
            intent='stoop', sweep_deg=110.0
        )
        self.assertFalse(telem['vne_exceeded'])
        self.assertEqual(telem['vne_ms'], fg.compute_sweep_vne(110.0))
        self.assertEqual(telem['brake_fraction'], 0.0)
        self.assertEqual(telem['status'], 'STOOP_HIGH_SPEED_DASH_ARMED')

    def test_overspeed_beyond_swept_vne_trips_speed_brake(self):
        """If airspeed exceeds the sweep-adapted VNE, emergency speed braking engages."""
        # At 110° sweep, VNE is ~38 m/s. At 40 m/s, it should trip.
        telem = self.gov.tick(
            V_ms=40.0, pitch_cmd_deg=0.0, bank_deg=0.0,
            intent='stoop', sweep_deg=110.0
        )
        self.assertTrue(telem['vne_exceeded'])
        self.assertTrue(telem['inhibit_flag'])
        self.assertEqual(telem['brake_fraction'], 1.0)
        self.assertEqual(telem['status'], 'VNE_EXCEEDED_FULL_SPEED_BRAKE')

    def test_pullout_from_stoop_coordinates_unfolding(self):
        """Ordering a pullout at 35 m/s with 110° sweep applies Boyd braking and holds sweep."""
        telem = self.gov.tick(
            V_ms=35.0, pitch_cmd_deg=10.0, bank_deg=0.0,
            intent='stoop', sweep_deg=110.0
        )
        # Pullout requested at 35 m/s: must brake and constrain unfolding
        self.assertTrue(telem['inhibit_flag'])
        self.assertGreater(telem['brake_fraction'], 0.0)
        self.assertTrue(telem['unfold_schedule']['unfold_constrained'])
        self.assertGreater(telem['commanded_sweep_deg'], 90.0)

    def test_govern_dive_recovery_hook_clamps_wing_commands(self):
        """govern_dive_recovery automatically prevents wing commands from unfolding prematurely."""
        result = {
            'sweep_deg': 110.0,
            'wing_commands': [{'sweep_deg': 60.0} for _ in range(8)]
        }
        telem = fg.govern_dive_recovery(
            V_ms=35.0, pitch_cmd_deg=10.0, bank_deg=0.0,
            intent='stoop', result=result
        )
        self.assertTrue(result['dive_inhibit_flag'])
        self.assertGreater(result['dive_vne_ms'], 28.0)
        self.assertEqual(result['dive_sweep_deg'], 110.0)
        # Wing commands must be clamped to commanded_sweep_deg
        for cmd in result['wing_commands']:
            self.assertGreaterEqual(cmd['sweep_deg'], telem['commanded_sweep_deg'] - 0.1)


class TestWaveLibrarySweepRegistration(unittest.TestCase):
    """Verifies hypothesis registration in the wave library and curriculum."""

    def test_hypotheses_in_aero_and_queue(self):
        """H-SWEEP-VNE-1 and H-SWEEP-PULL-1 must be registered in AERO_HYPOTHESES."""
        ids = [h['id'] for h in wl.AERO_HYPOTHESES]
        self.assertIn('H-SWEEP-VNE-1', ids)
        self.assertIn('H-SWEEP-PULL-1', ids)

        q_ids = [h['id'] for h in wl._hypothesis_queue]
        self.assertIn('H-SWEEP-VNE-1', q_ids)
        self.assertIn('H-SWEEP-PULL-1', q_ids)

    def test_curriculum_segments(self):
        """ALSS segments must be registered in _ALSS_SEGMENTS and Curriculum C."""
        self.assertIn('sweep_vne1_sched_c', wl._ALSS_SEGMENTS)
        self.assertIn('sweep_pull1_unfold_c', wl._ALSS_SEGMENTS)
        self.assertIn('sweep_vne1_sched_c', wl._ALSS_CURRICULUM['C'])
        self.assertIn('sweep_pull1_unfold_c', wl._ALSS_CURRICULUM['C'])

    def test_dive_status_output(self):
        """dive_status() must execute cleanly and display sweep-adaptive metrics."""
        report = wl.dive_status()
        self.assertIn("Never-Exceed Speed (VNE):", report)
        self.assertIn("Wrist Unfolding Status:", report)
        self.assertIn("Peregrine Stoop (135°)", report)


if __name__ == '__main__':
    unittest.main()
