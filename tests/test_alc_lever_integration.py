"""
Unit & Integration Tests: ALC Hypothesis Generation, Refinement, and Wave Lever Integration
===========================================================================================
Verifies:
  1. Dynamic hypothesis generation via alc_generate_hypothesis_variant with physical bounds
     and parameter integrity.
  2. In-flight continuous hypothesis refinement via alc_refine_active_hypotheses.
  3. Closed-loop throttle lever hypothesis shuffling via _update_genome_from_trial,
     lever_mode_promote, and lever_mode_demote.
  4. Dynamic mode selection in unified_lever_to_params via _lv_select.
  5. Transparent, monotonic pilot throttle feel across 0% to 100% lever positions.
  6. Ground station reporting in lever_map_status.
"""

import sys
import os
import unittest
import math

# Ensure firmware directory is in python path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'firmware'))

import mco_wave_library as wl


class TestALCHypothesisGeneration(unittest.TestCase):
    """Verifies autonomous generation of bounded hypothesis variants."""

    def test_generate_efficiency_variant(self):
        parent_id = 'H-GLIDER-1'
        var = wl.alc_generate_hypothesis_variant(parent_id, mutation_axis='efficiency', sensors={'V_ms': 10.0})
        self.assertIsNotNone(var)
        self.assertTrue(var['id'].startswith('H-GLIDER-1-VAR'))
        self.assertEqual(var['parent_id'], parent_id)
        self.assertEqual(var['mutation_axis'], 'efficiency')

        # Check config limits
        cfg = var['config']
        self.assertGreaterEqual(cfg['freq_hz'], 0.1)
        self.assertLessEqual(cfg['freq_hz'], 4.2)
        self.assertGreaterEqual(cfg['aoa_deg'], -5.0)
        self.assertLessEqual(cfg['aoa_deg'], 35.0)
        self.assertGreaterEqual(cfg['sweep_deg'], 15.0)
        self.assertLessEqual(cfg['sweep_deg'], 90.0)
        self.assertGreaterEqual(cfg['amp_port_deg'], 20.0)
        self.assertLessEqual(cfg['amp_port_deg'], 110.0)

        # Check registration
        aero_ids = {h['id'] for h in wl.AERO_HYPOTHESES}
        self.assertIn(var['id'], aero_ids)
        self.assertIn(var['id'], wl._HYP_TO_LEVER_ZONE)
        zone = wl._HYP_TO_LEVER_ZONE[var['id']]
        self.assertIn(var['id'], wl._lever_mode_weights[zone])

    def test_generate_climb_variant(self):
        parent_id = 'H-65'
        var = wl.alc_generate_hypothesis_variant(parent_id, mutation_axis='climb', target_regime='POWERED_CLIMB')
        self.assertIsNotNone(var)
        self.assertTrue(var['id'].startswith('H-65-VAR'))
        self.assertEqual(wl._HYP_TO_LEVER_ZONE[var['id']], 'POWERED_CLIMB')

        cfg = var['config']
        self.assertGreaterEqual(cfg['cascade_phase_deg'], 15.0)
        self.assertLessEqual(cfg['cascade_phase_deg'], 30.0)

    def test_generate_sprint_variant(self):
        parent_id = 'H-MICRO-1'
        var = wl.alc_generate_hypothesis_variant(parent_id, mutation_axis='sprint', target_regime='MAX_THRUST')
        self.assertIsNotNone(var)
        self.assertEqual(wl._HYP_TO_LEVER_ZONE[var['id']], 'MAX_THRUST')

        cfg = var['config']
        self.assertEqual(cfg['cascade_phase_deg'], 0.0)
        self.assertLessEqual(cfg['sweep_deg'], 50.0)


class TestALCThrottleLeverShuffling(unittest.TestCase):
    """Verifies that closed-loop scoring dynamically shuffles hypotheses on the throttle lever."""

    def setUp(self):
        # Save snapshot of weights
        self._orig_weights = {z: dict(w) for z, w in wl._lever_mode_weights.items()}

    def tearDown(self):
        # Restore weights
        for z, w in self._orig_weights.items():
            wl._lever_mode_weights[z] = dict(w)

    def test_promote_variant_takes_over_zone(self):
        # Generate a candidate variant in CRUISE
        var = wl.alc_generate_hypothesis_variant('H-GLIDER-1', mutation_axis='efficiency', target_regime='CRUISE')
        var_id = var['id']

        # Repeatedly reward this variant for stellar efficiency
        for _ in range(5):
            wl._update_genome_from_trial(var_id, 0.40, {'wh_per_km': 7.2, 'V_ms': 10.0})

        # Check that its weight in CRUISE has grown
        w_cruise = wl._lever_mode_weights['CRUISE']
        self.assertIn(var_id, w_cruise)

        # Check dynamic lever selection at cruise throttle (60%)
        params = wl.unified_lever_to_params(60.0, V_ms=10.0, h_agl_m=100.0)
        self.assertEqual(params['zone'], 'CRUISE')
        self.assertEqual(params['mode_id'], var_id)

    def test_demote_underperforming_hypothesis(self):
        # Demote H-GLIDER-1 significantly in CRUISE
        for _ in range(8):
            wl._update_genome_from_trial('H-GLIDER-1', -0.50, {'wh_per_km': 15.0})

        w_cruise = wl._lever_mode_weights['CRUISE']
        self.assertLess(w_cruise.get('H-GLIDER-1', 1.0), 1.0)

    def test_continuous_throttle_monotonicity(self):
        # Verify that even with promoted hypotheses, throttle parameters remain smooth and safe
        V_ms = 10.0
        h_agl = 100.0
        last_freq = 0.0

        # Sweep throttle from detent (50%) to max thrust (100%)
        for lev in range(50, 101, 5):
            p = wl.unified_lever_to_params(float(lev), V_ms=V_ms, h_agl_m=h_agl)
            # Throttle lever progression should have non-decreasing frequency demand
            self.assertGreaterEqual(p['freq_hz'], last_freq - 0.05, f"Discontinuity at {lev}% lever")
            last_freq = p['freq_hz']
            self.assertTrue(p['efficiency_governed'])


class TestALCActiveRefinement(unittest.TestCase):
    """Verifies telemetry-driven closed-loop refinement via alc_refine_active_hypotheses."""

    def test_refine_in_cruise(self):
        sensors = {
            'lever_pct': 60.0,
            'V_ms': 10.0,
            'wh_per_km': 8.0,
            'bus_voltage_ripple_v': 0.3,
            'imu_rms_g': 0.015,
        }
        report = wl.alc_refine_active_hypotheses(sensors=sensors)
        self.assertIsNotNone(report)
        self.assertEqual(report['zone'], 'CRUISE')
        self.assertGreater(report['score_delta'], 0.0)

    def test_refine_in_climb(self):
        sensors = {
            'lever_pct': 75.0,
            'V_ms': 8.5,
            'VSI_ms': 2.5,
            'bus_voltage_ripple_v': 0.5,
        }
        report = wl.alc_refine_active_hypotheses(sensors=sensors)
        self.assertIsNotNone(report)
        self.assertEqual(report['zone'], 'POWERED_CLIMB')
        self.assertGreater(report['score_delta'], 0.0)


class TestLeverMapStatus(unittest.TestCase):
    """Verifies GS command lever_map_status returns comprehensive rankings."""

    def test_lever_map_status_report(self):
        rep = wl.lever_map_status()
        self.assertIsInstance(rep, dict)
        self.assertIn('zones', rep)
        self.assertIn('MAX_THRUST', rep['zones'])
        self.assertIn('CRUISE', rep['zones'])
        self.assertIn('POWERED_CLIMB', rep['zones'])
        cruise_rep = rep['zones']['CRUISE']
        self.assertIn('best', cruise_rep)
        self.assertIn('rankings', cruise_rep)


if __name__ == '__main__':
    unittest.main()
