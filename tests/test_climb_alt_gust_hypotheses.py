"""
Unit & Integration Tests: Consolidated Aerodynamic & High-DA Core Modes (Option A)
=============================================================================
Verifies that:
  1. The 4 redundant duplicate hypotheses (H-VX-CLIMB-1, H-VY-CLIMB-1,
     H-HIGH-ALT-1, H-GUST-REFLEX-1) are completely purged from all catalogs.
  2. The grounded physical innovations have been consolidated into core modes:
     - CLB2_ANGLE (HIGH_THRUST): differential pitch schedule (front 16.0°, rear 11.0°,
       cascade phase 20.0°).
     - CLB1_RATE (POWERED_CLIMB): regenerative bus shuttling & metachronal climb.
     - H-DA-5 (CRUISE / TRIM_DETENT): station-wise twist distribution (inboard AoA 4.8°,
       outboard reflex -2.5°).
  3. Closed-loop ALC lever mode weighting and unified_lever_to_params parameter
     synthesis reflect the consolidated physics.
  4. Genome update routing in _update_genome_from_trial promotes the core modes.
"""

import sys
import os
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'firmware'))

import mco_wave_library as wl


class TestDuplicateHypothesesPurged(unittest.TestCase):
    """Ensures redundant duplicate hypotheses are absent across all structures."""

    PURGED_HIDS = ('H-VX-CLIMB-1', 'H-VY-CLIMB-1', 'H-HIGH-ALT-1', 'H-GUST-REFLEX-1')

    def test_not_in_adv_new_hyps(self):
        adv_ids = {h['id'] for h in wl._ADV_NEW_HYPS}
        for hid in self.PURGED_HIDS:
            self.assertNotIn(hid, adv_ids, f"{hid} still in _ADV_NEW_HYPS")

    def test_not_in_aero_hypotheses(self):
        aero_ids = {h['id'] for h in wl.AERO_HYPOTHESES}
        for hid in self.PURGED_HIDS:
            self.assertNotIn(hid, aero_ids, f"{hid} still in AERO_HYPOTHESES")

    def test_not_in_lever_mode_weights(self):
        for zone, w_dict in wl._lever_mode_weights.items():
            for hid in self.PURGED_HIDS:
                self.assertNotIn(hid, w_dict, f"{hid} still in _lever_mode_weights[{zone}]")

    def test_not_in_hyp_to_lever_zone(self):
        for hid in self.PURGED_HIDS:
            self.assertNotIn(hid, wl._HYP_TO_LEVER_ZONE, f"{hid} still in _HYP_TO_LEVER_ZONE")

    def test_not_in_cmts_champions(self):
        for champ_name, champ_cfg in wl._CMTS_CHAMPIONS.items():
            for hid in self.PURGED_HIDS:
                self.assertNotEqual(champ_cfg.get('hyp_id'), hid,
                                    f"{hid} still champion for {champ_name}")


class TestConsolidatedCoreModes(unittest.TestCase):
    """Validates the enhancement of core modes with verified physics."""

    def test_clb2_angle_weights_and_zone(self):
        ht = wl._lever_mode_weights.get('HIGH_THRUST', {})
        self.assertIn('CLB2_ANGLE', ht)
        self.assertGreaterEqual(ht['CLB2_ANGLE'], 1.4)
        self.assertEqual(wl._HYP_TO_LEVER_ZONE.get('CLB2_ANGLE'), 'HIGH_THRUST')

    def test_clb1_rate_weights_and_zone(self):
        pc = wl._lever_mode_weights.get('POWERED_CLIMB', {})
        self.assertIn('CLB1_RATE', pc)
        self.assertGreaterEqual(pc['CLB1_RATE'], 1.4)
        self.assertEqual(wl._HYP_TO_LEVER_ZONE.get('CLB1_RATE'), 'POWERED_CLIMB')

    def test_h_da_5_weights_and_zone(self):
        cr = wl._lever_mode_weights.get('CRUISE', {})
        td = wl._lever_mode_weights.get('TRIM_DETENT', {})
        self.assertIn('H-DA-5', cr)
        self.assertIn('H-DA-5', td)
        self.assertGreaterEqual(cr['H-DA-5'], 1.4)
        self.assertGreaterEqual(td['H-DA-5'], 1.4)
        self.assertEqual(wl._HYP_TO_LEVER_ZONE.get('H-DA-5'), 'CRUISE')

    def test_h_da_5_station_twist_distribution(self):
        h = next((h for h in wl.AERO_HYPOTHESES if h['id'] == 'H-DA-5'), None)
        self.assertIsNotNone(h, "H-DA-5 not found in AERO_HYPOTHESES")
        cfg = h['config']
        self.assertTrue(cfg.get('station_twist_distribution'), "station_twist_distribution flag missing")
        self.assertEqual(cfg.get('inboard_aoa_deg'), 4.8)
        self.assertEqual(cfg.get('outboard_reflex_deg'), -2.5)

    def test_high_thrust_differential_pitch_synthesis(self):
        """Verifies unified_lever_to_params synthesizes differential pitch schedule in HIGH_THRUST."""
        params = wl.unified_lever_to_params(
            lev_pct=85.0,
            V_ms=6.0,
            h_agl_m=100.0,
        )
        self.assertEqual(params.get('mode_id'), 'CLB2_ANGLE')
        self.assertIn('front_aoa_deg', params)
        self.assertIn('rear_aoa_deg', params)
        self.assertEqual(params['front_aoa_deg'], 16.0)
        self.assertEqual(params['rear_aoa_deg'], 11.0)
        self.assertEqual(params['cascade_phase_deg'], 20.0)

    def test_genome_update_routing_for_core_modes(self):
        """Verifies ALC weight adaptations for CLB2_ANGLE and H-DA-5."""
        w_before_clb = wl._lever_mode_weights['HIGH_THRUST']['CLB2_ANGLE']
        total_before_clb = sum(wl._lever_mode_weights['HIGH_THRUST'].values())
        share_before_clb = w_before_clb / total_before_clb

        wl._update_genome_from_trial('CLB2_ANGLE', +0.20, {'climb_gradient_deg': 14.5})

        w_after_clb = wl._lever_mode_weights['HIGH_THRUST']['CLB2_ANGLE']
        total_after_clb = sum(wl._lever_mode_weights['HIGH_THRUST'].values())
        share_after_clb = w_after_clb / total_after_clb
        self.assertGreater(share_after_clb, share_before_clb)

        w_before_da = wl._lever_mode_weights['CRUISE']['H-DA-5']
        total_before_da = sum(wl._lever_mode_weights['CRUISE'].values())
        share_before_da = w_before_da / total_before_da

        wl._update_genome_from_trial('H-DA-5', +0.20, {'wh_per_km': 13.5})

        w_after_da = wl._lever_mode_weights['CRUISE']['H-DA-5']
        total_after_da = sum(wl._lever_mode_weights['CRUISE'].values())
        share_after_da = w_after_da / total_after_da
        self.assertGreater(share_after_da, share_before_da)


if __name__ == '__main__':
    unittest.main()
