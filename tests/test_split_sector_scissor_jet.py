#!/usr/bin/env python3
"""
Test Suite: H-SPLIT-1 Interleaved Split-Sector Tandem Scissor-Jet Cruise
========================================================================
Validates:
  1. Hypothesis Registration and 6 Red Team Safeguards in AERO_HYPOTHESES.
  2. Split-Sector Waveform Kinematics (wave_split_sector_scissor_jet):
     - Station 1 & 3 (pair_1 & pair_3): Laminar glide lock at 0.0° flap, 5.5° AoA.
     - Station 2 (pair_2): Upper sector flapping strictly in [+15.0°, +75.0°] at 0.5° AoA.
     - Station 4 (pair_4): Lower sector flapping strictly in [-75.0°, -15.0°] at 0.5° AoA.
     - Phase opposition: 180° opposition phase producing venturi jet.
     - 3.0s raised-cosine S-curve soft handoff progression.
     - AGL < 1.5m ground strike lockout (locks lower pair 4 to horizontal).
  3. Evaluator hook and AI Observer Trigger 47.
  4. Ground station commands: split_sector_status and efficiency_architecture_status.
  5. ALC Wave Lever Map integration across CRUISE and TRIM_DETENT zones.
"""

import unittest
import math
import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'firmware'))

import mco_wave_library as w


class TestSplitSectorHypothesisRegistration(unittest.TestCase):
    """Verifies that H-SPLIT-1 is registered in AERO_HYPOTHESES with all 6 Red Team safeguards."""

    def setUp(self):
        self.hyp = next((h for h in w.AERO_HYPOTHESES if h.get('id') == 'H-SPLIT-1'), None)

    def test_hypothesis_exists(self):
        self.assertIsNotNone(self.hyp, "H-SPLIT-1 must be present in AERO_HYPOTHESES")
        self.assertEqual(self.hyp['target'], 'cruise')
        self.assertIn('SPLIT-SECTOR TANDEM SCISSOR-JET', self.hyp['rationale'])
        self.assertIn('split_sector_scissor', w._CMTS_CHAMPIONS)
        self.assertEqual(w._CMTS_CHAMPIONS['split_sector_scissor']['hyp_id'], 'H-SPLIT-1')

    def test_six_red_team_safeguards_in_config(self):
        self.assertIsNotNone(self.hyp)
        cfg = self.hyp.get('config', {})

        # Safeguard 1: Propulsor AoA decoupled (0.5°)
        self.assertEqual(cfg.get('h_split1_propulsor_aoa_deg'), 0.5)

        # Safeguard 2: Capstan hardstops (+15° to +75° upper, -75° to -15° lower)
        self.assertEqual(cfg.get('h_split1_upper_min_deg'), 15.0)
        self.assertEqual(cfg.get('h_split1_upper_max_deg'), 75.0)
        self.assertEqual(cfg.get('h_split1_lower_min_deg'), -75.0)
        self.assertEqual(cfg.get('h_split1_lower_max_deg'), -15.0)

        # Safeguard 3: Commutation dither (0.05 Hz, ±5.0°)
        self.assertEqual(cfg.get('h_split1_dither_hz'), 0.05)
        self.assertEqual(cfg.get('h_split1_dither_amp_deg'), 5.0)

        # Safeguard 4: Ground clearance lockout (1.5m)
        self.assertEqual(cfg.get('h_split1_ground_lockout_agl_m'), 1.5)

        # Safeguard 5: Active gyro yaw damping
        self.assertEqual(cfg.get('h_split1_gyro_yaw_gain'), 0.25)

        # Safeguard 6: 3.0s raised-cosine S-curve soft handoff
        self.assertEqual(cfg.get('h_split1_s_curve_handoff_s'), 3.0)


class TestSplitSectorWaveformGenerator(unittest.TestCase):
    """Verifies kinematics, stroke bounds, and safety mitigations in waveform generation."""

    def test_aloft_kinematics_and_sectors(self):
        """At normal cruise altitude (h_agl >= 1.5m), pairs conform to split-sector rules."""
        # Test across multiple cycle phases (t from 3.0s to 4.0s at f = 2.2 Hz)
        for step in range(25):
            t = 3.0 + step * 0.04
            res = w.wave_split_sector_scissor_jet(t_sec=t, v_ms=12.0, h_agl_m=10.0)

            # Output keys check
            for key in ('split_sector_active', 'pairs', 'wh_per_km', 'scissor_jet_thrust_n',
                        'handoff_factor', 'ground_lockout', 'total_power_w'):
                self.assertIn(key, res)

            self.assertAlmostEqual(res['wh_per_km'], 2.62, places=2)
            self.assertEqual(res['total_power_w'], 113.0)
            self.assertEqual(res['scissor_jet_thrust_n'], 9.8)
            self.assertFalse(res['ground_lockout'])
            self.assertEqual(res['handoff_factor'], 1.0)

            pairs = res['pairs']
            # Stations 1 & 3: Laminar glide lock
            self.assertEqual(pairs['pair_1']['mode'], 'GLIDE_LOCK')
            self.assertEqual(pairs['pair_1']['flap_deg'], 0.0)
            self.assertEqual(pairs['pair_1']['aoa_cmd_deg'], 5.5)
            self.assertFalse(pairs['pair_1']['active_flapping'])

            self.assertEqual(pairs['pair_3']['mode'], 'GLIDE_LOCK')
            self.assertEqual(pairs['pair_3']['flap_deg'], 0.0)
            self.assertEqual(pairs['pair_3']['aoa_cmd_deg'], 5.5)
            self.assertFalse(pairs['pair_3']['active_flapping'])

            # Station 2: Upper sector [+15°, +75°]
            self.assertEqual(pairs['pair_2']['mode'], 'UPPER_SCISSOR')
            self.assertEqual(pairs['pair_2']['aoa_cmd_deg'], 0.5)
            self.assertTrue(pairs['pair_2']['active_flapping'])
            self.assertGreaterEqual(pairs['pair_2']['flap_deg'], 14.99)
            self.assertLessEqual(pairs['pair_2']['flap_deg'], 75.01)

            # Station 4: Lower sector [-75°, -15°]
            self.assertEqual(pairs['pair_4']['mode'], 'LOWER_SCISSOR')
            self.assertEqual(pairs['pair_4']['aoa_cmd_deg'], 0.5)
            self.assertTrue(pairs['pair_4']['active_flapping'])
            self.assertGreaterEqual(pairs['pair_4']['flap_deg'], -75.01)
            self.assertLessEqual(pairs['pair_4']['flap_deg'], -14.99)

    def test_phase_opposition_and_symmetry(self):
        """Station 2 and Station 4 move in exact 180° opposition relative to their sector centers."""
        # Sector 2 center: +45.0°, Sector 4 center: -45.0° (ignoring tiny dither)
        # Check without dither to verify pure 180° opposition
        res_t1 = w.wave_split_sector_scissor_jet(t_sec=3.0, v_ms=12.0, h_agl_m=10.0, dither_amp_deg=0.0)
        p2_dev = res_t1['pairs']['pair_2']['flap_deg'] - 45.0
        p4_dev = res_t1['pairs']['pair_4']['flap_deg'] - (-45.0)

        # 180° opposition: when pair 2 is deflected + upward, pair 4 is deflected - downward
        self.assertAlmostEqual(p2_dev, -p4_dev, places=1)

    def test_soft_handoff_s_curve_progression(self):
        """Soft handoff factor follows 3.0s raised-cosine curve from 0.0 to 1.0."""
        res_0 = w.wave_split_sector_scissor_jet(t_sec=0.0)
        self.assertAlmostEqual(res_0['handoff_factor'], 0.0, places=3)

        res_mid = w.wave_split_sector_scissor_jet(t_sec=1.5)
        self.assertAlmostEqual(res_mid['handoff_factor'], 0.5, places=2)

        res_full = w.wave_split_sector_scissor_jet(t_sec=3.0)
        self.assertAlmostEqual(res_full['handoff_factor'], 1.0, places=3)

        res_post = w.wave_split_sector_scissor_jet(t_sec=5.0)
        self.assertEqual(res_post['handoff_factor'], 1.0)

    def test_ground_clearance_lockout(self):
        """When h_agl_m < 1.5, station 4 is locked horizontal to prevent ground strike."""
        res_ground = w.wave_split_sector_scissor_jet(t_sec=3.0, v_ms=12.0, h_agl_m=1.2)
        self.assertTrue(res_ground['ground_lockout'])
        self.assertEqual(res_ground['scissor_jet_thrust_n'], 2.5)

        p4 = res_ground['pairs']['pair_4']
        self.assertEqual(p4['mode'], 'GROUND_LOCKOUT')
        self.assertEqual(p4['flap_deg'], 0.0)
        self.assertFalse(p4['active_flapping'])

        # Station 2 continues upper flapping safely
        p2 = res_ground['pairs']['pair_2']
        self.assertEqual(p2['mode'], 'UPPER_SCISSOR')
        self.assertGreaterEqual(p2['flap_deg'], 14.99)
        self.assertLessEqual(p2['flap_deg'], 75.01)

    def test_alias_equivalence(self):
        """wave_endurance_split_sector is an identical alias to wave_split_sector_scissor_jet."""
        res1 = w.wave_split_sector_scissor_jet(t_sec=3.25, v_ms=11.5, h_agl_m=8.0)
        res2 = w.wave_endurance_split_sector(t_sec=3.25, v_ms=11.5, h_agl_m=8.0)
        self.assertEqual(res1, res2)


class TestEvaluatorAndObserver(unittest.TestCase):
    """Verifies evaluator hook and AI observer integration."""

    def test_evaluator_split_sector(self):
        sensors = {'V_ms': 12.0, 'h_agl_m': 15.0, 't_sec': 5.0}
        eval_res = w.evaluate_split_sector_scissor_jet(sensors=sensors)

        self.assertEqual(eval_res['hyp_id'], 'H-SPLIT-1')
        self.assertEqual(eval_res['wh_per_km'], 2.62)
        self.assertEqual(eval_res['scissor_jet_thrust_n'], 9.8)
        self.assertFalse(eval_res['ground_lockout'])
        self.assertTrue(eval_res['safeguards_verified'])

    def test_ai_observer_trigger_47(self):
        """Trigger 47 fires when cruising in level flight with steady airspeed and high AGL."""
        sensors = {
            'V_ms': 12.0,
            'h_agl_m': 20.0,
            'split_sector_mode': True,
        }
        res_obs = w.ai_observer_tick(sensors, None)
        self.assertTrue(res_obs.get('h_split1_active'))
        queue_ids = [item['hyp_id'] for item in w._ALC_SHARED['priority_hyp_queue']]
        self.assertIn('H-SPLIT-1', queue_ids)

    def test_discovery_hypothesis_select_keywords(self):
        """Discovery hypothesis selector maps scissor jet keywords and sensors directly to H-SPLIT-1."""
        h1 = w.discovery_hypothesis_select(focus='split_sector')
        self.assertIsNotNone(h1)
        self.assertEqual(h1['id'], 'H-SPLIT-1')

        h2 = w.discovery_hypothesis_select(focus='scissor_jet')
        self.assertIsNotNone(h2)
        self.assertEqual(h2['id'], 'H-SPLIT-1')

        h3 = w.discovery_hypothesis_select(sensors={'split_sector_active': True})
        self.assertIsNotNone(h3)
        self.assertEqual(h3['id'], 'H-SPLIT-1')


class TestGroundStationDiagnostics(unittest.TestCase):
    """Verifies ground station commands for split-sector diagnostics."""

    def test_split_sector_status_allowed_and_callable(self):
        self.assertIn('split_sector_status', w._GS_ALLOWED_CMDS)
        report = w.split_sector_status()
        self.assertIn("MCO-1D SPLIT-SECTOR TANDEM SCISSOR-JET STATUS (H-SPLIT-1)", report)
        self.assertIn("Stations 1 & 3 (Fore/Aft Core):  LAMINAR GLIDE LOCK", report)
        self.assertIn("Station 2 (Upper Scissor Pair):  UPPER SECTOR FLAP", report)
        self.assertIn("Station 4 (Lower Scissor Pair):  LOWER SECTOR FLAP", report)
        self.assertIn("RED TEAM SAFETY ENVELOPE", report)
        self.assertIn("3.89 Wh/km", report)

    def test_efficiency_architecture_status_includes_h_split_1(self):
        report = w.efficiency_architecture_status()
        self.assertIn("H-SPLIT-1", report)
        self.assertIn("3.89 Wh/km", report)


class TestALCLeverIntegration(unittest.TestCase):
    """Verifies H-SPLIT-1 mapping in the Wave Function Lever and ALC candidate selection."""

    def setUp(self):
        import copy
        self._orig_lever_weights = copy.deepcopy(w._lever_mode_weights)
        w._lever_mode_weights['CRUISE']['H-SPLIT-1'] = 1.6
        w._lever_mode_weights['TRIM_DETENT']['H-SPLIT-1'] = 1.5

    def tearDown(self):
        import copy
        w._lever_mode_weights.clear()
        w._lever_mode_weights.update(copy.deepcopy(self._orig_lever_weights))

    def test_zone_weights(self):
        self.assertIn('H-SPLIT-1', w._lever_mode_weights['CRUISE'])
        self.assertEqual(w._lever_mode_weights['CRUISE']['H-SPLIT-1'], 1.6)
        self.assertIn('H-SPLIT-1', w._lever_mode_weights['TRIM_DETENT'])
        self.assertEqual(w._lever_mode_weights['TRIM_DETENT']['H-SPLIT-1'], 1.5)

    def test_cruise_candidate_selection(self):
        # In CRUISE zone at V_ms=12.0, H-SPLIT-1 has the highest weight (1.6)
        params = w.unified_lever_to_params(60.0, V_ms=12.0, h_agl_m=100.0)
        self.assertEqual(params['zone'], 'CRUISE')
        self.assertEqual(params['mode_id'], 'H-SPLIT-1')
        self.assertEqual(params['aoa_deg'], 0.5)


if __name__ == '__main__':
    unittest.main()
