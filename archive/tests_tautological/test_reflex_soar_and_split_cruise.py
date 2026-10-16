#!/usr/bin/env python3
"""
Unit & Integration Tests: Hypotheses H-REFLEX-SOAR-1 & H-SPLIT-CRUISE-1
================================================================================
Comprehensive verification suite covering:
  1. Hypothesis Registration & Metadata in AERO_HYPOTHESES, _ADV_NEW_HYPS,
     _hypothesis_queue, and BIOMIMETIC_HYPOTHESES
  2. Configuration validation, aerodynamic bounds, and Red Team safeguards
  3. Scoring Tasks in _SG_TASKS, Curriculum in _ALSS_SEGMENTS / _ALSS_CURRICULUM,
     and CMTS Context Champions in _CMTS_CHAMPIONS
  4. Expanded Discovery Bounds in _EXPANDED_DISCOVERY_BOUNDS
  5. Wave Function Lever & Throttle Mapping in _HYP_TO_LEVER_ZONE and _lever_mode_weights
  6. Genome Update Routing in _update_genome_from_trial
"""

import sys
import os
import unittest

# Ensure firmware directory is in python path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'firmware'))

import mco_wave_library as wl


class TestReflexSoarAndSplitCruiseRegistration(unittest.TestCase):
    """Verifies H-REFLEX-SOAR-1 and H-SPLIT-CRUISE-1 registration across all catalogs."""

    TARGET_HIDS = ('H-REFLEX-SOAR-1', 'H-SPLIT-CRUISE-1')

    def test_registered_in_adv_new_hyps(self):
        adv_ids = {h['id'] for h in wl._ADV_NEW_HYPS}
        for hid in self.TARGET_HIDS:
            self.assertIn(hid, adv_ids, f"{hid} missing from _ADV_NEW_HYPS")

    def test_registered_in_aero_hypotheses(self):
        aero_ids = {h['id'] for h in wl.AERO_HYPOTHESES}
        for hid in self.TARGET_HIDS:
            self.assertIn(hid, aero_ids, f"{hid} missing from AERO_HYPOTHESES")

        h_rs = next(h for h in wl.AERO_HYPOTHESES if h['id'] == 'H-REFLEX-SOAR-1')
        self.assertEqual(h_rs['target'], 'endurance')
        self.assertIn('cruise', h_rs['eval_phases'])
        self.assertIn('endurance', h_rs['eval_phases'])
        self.assertEqual(h_rs['priority'], 1)
        self.assertEqual(h_rs['v_min'], 7.5)
        self.assertEqual(h_rs['v_max'], 16.0)

        h_sc = next(h for h in wl.AERO_HYPOTHESES if h['id'] == 'H-SPLIT-CRUISE-1')
        self.assertEqual(h_sc['target'], 'cruise')
        self.assertIn('cruise', h_sc['eval_phases'])
        self.assertIn('endurance', h_sc['eval_phases'])
        self.assertIn('sprint', h_sc['eval_phases'])
        self.assertEqual(h_sc['priority'], 1)
        self.assertEqual(h_sc['v_min'], 9.5)
        self.assertEqual(h_sc['v_max'], 20.0)

    def test_registered_in_hypothesis_queue(self):
        q_ids = {h['id'] for h in wl._hypothesis_queue}
        for hid in self.TARGET_HIDS:
            self.assertIn(hid, q_ids, f"{hid} missing from _hypothesis_queue")

    def test_registered_in_biomimetic_catalog(self):
        for hid in self.TARGET_HIDS:
            self.assertIn(hid, wl.BIOMIMETIC_HYPOTHESES, f"{hid} missing from BIOMIMETIC_HYPOTHESES")


class TestReflexSoarAndSplitCruiseConfigIntegrity(unittest.TestCase):
    """Validates configuration parameters and boundaries for both hypotheses."""

    def test_h_reflex_soar_1_config(self):
        h = next(h for h in wl.AERO_HYPOTHESES if h['id'] == 'H-REFLEX-SOAR-1')
        cfg = h['config']
        self.assertTrue(cfg.get('_h_reflex_soar_1'))
        self.assertEqual(cfg.get('h_rs1_active_pair'), 4)
        self.assertEqual(cfg.get('h_rs1_glide_pairs'), [1, 2, 3])
        self.assertEqual(cfg.get('h_rs1_glide_aoa_deg'), 6.0)
        self.assertEqual(cfg.get('h_rs1_reflex_trim_deg'), 1.5)
        self.assertEqual(cfg.get('h_rs1_min_amp_deg'), 22.0)
        self.assertEqual(cfg.get('h_rs1_max_amp_deg'), 32.0)
        self.assertEqual(cfg.get('h_rs1_holding_current_a'), 4.0)
        self.assertEqual(cfg.get('h_rs1_thermal_rotation_s'), 30.0)
        self.assertEqual(cfg.get('h_rs1_v_min_abort_ms'), 7.5)
        self.assertEqual(cfg.get('h_rs1_pilot_stick_abort'), 0.30)
        self.assertEqual(cfg.get('_scorer'), 'reflex_soar_efficiency_wh_km')

    def test_h_split_cruise_1_config(self):
        h = next(h for h in wl.AERO_HYPOTHESES if h['id'] == 'H-SPLIT-CRUISE-1')
        cfg = h['config']
        self.assertTrue(cfg.get('_h_split_cruise_1'))
        self.assertEqual(cfg.get('h_sc1_upper_pair'), 2)
        self.assertEqual(cfg.get('h_sc1_lower_pair'), 4)
        self.assertEqual(cfg.get('h_sc1_glide_pairs'), [1, 3])
        self.assertEqual(cfg.get('h_sc1_upper_min_deg'), 20.0)
        self.assertEqual(cfg.get('h_sc1_upper_max_deg'), 48.0)
        self.assertEqual(cfg.get('h_sc1_lower_min_deg'), -48.0)
        self.assertEqual(cfg.get('h_sc1_lower_max_deg'), -20.0)
        self.assertEqual(cfg.get('h_sc1_glide_reflex_deg'), -2.0)
        self.assertEqual(cfg.get('h_sc1_glide_aoa_deg'), 4.5)
        self.assertEqual(cfg.get('h_sc1_dither_hz'), 0.05)
        self.assertEqual(cfg.get('h_sc1_dither_amp_deg'), 3.0)
        self.assertEqual(cfg.get('h_sc1_ground_lockout_agl_m'), 1.5)
        self.assertEqual(cfg.get('h_sc1_v_min_abort_ms'), 9.5)
        self.assertEqual(cfg.get('h_sc1_holding_current_a'), 5.0)
        self.assertEqual(cfg.get('h_sc1_pilot_stick_abort'), 0.35)
        self.assertEqual(cfg.get('_scorer'), 'split_cruise_efficiency_wh_km')


class TestReflexSoarAndSplitCruiseScoringCurriculumAndCMTS(unittest.TestCase):
    """Verifies scoring tasks, ALSS segments, curriculum, and CMTS context champions."""

    def test_sg_task_metrics(self):
        self.assertIn('reflex_soar_efficiency_wh_km', wl._SG_TASKS)
        t_rs = wl._SG_TASKS['reflex_soar_efficiency_wh_km']
        self.assertEqual(t_rs['direction'], -1)
        self.assertEqual(t_rs['axis'], 'endurance')
        self.assertEqual(t_rs['sensor'], 'wh_per_km')

        self.assertIn('split_cruise_efficiency_wh_km', wl._SG_TASKS)
        t_sc = wl._SG_TASKS['split_cruise_efficiency_wh_km']
        self.assertEqual(t_sc['direction'], -1)
        self.assertEqual(t_sc['axis'], 'endurance')
        self.assertEqual(t_sc['sensor'], 'wh_per_km')

    def test_alss_curriculum_segments(self):
        self.assertIn('rs1_reflex_soar_b', wl._ALSS_SEGMENTS)
        seg_rs = wl._ALSS_SEGMENTS['rs1_reflex_soar_b']
        self.assertEqual(seg_rs[0], 'cruise')
        self.assertEqual(seg_rs[1], 'H-REFLEX-SOAR-1')
        self.assertEqual(seg_rs[2], 20.0)
        self.assertEqual(seg_rs[3], 120.0)
        self.assertEqual(seg_rs[4], 9.0)
        self.assertIn('rs1_reflex_soar_b', wl._ALSS_CURRICULUM.get('B', []))

        self.assertIn('sc1_split_cruise_b', wl._ALSS_SEGMENTS)
        seg_sc = wl._ALSS_SEGMENTS['sc1_split_cruise_b']
        self.assertEqual(seg_sc[0], 'cruise')
        self.assertEqual(seg_sc[1], 'H-SPLIT-CRUISE-1')
        self.assertEqual(seg_sc[2], 20.0)
        self.assertEqual(seg_sc[3], 120.0)
        self.assertEqual(seg_sc[4], 12.0)
        self.assertIn('sc1_split_cruise_b', wl._ALSS_CURRICULUM.get('B', []))

    def test_phase_to_sg_task_mapping(self):
        self.assertIn('reflex_soar', wl._PHASE_TO_SG_TASK)
        self.assertIn('reflex_soar_efficiency_wh_km', wl._PHASE_TO_SG_TASK['reflex_soar'])
        self.assertIn('motorglider_efficiency_wh_km', wl._PHASE_TO_SG_TASK['reflex_soar'])
        self.assertIn('strouhal_optimal', wl._PHASE_TO_SG_TASK['reflex_soar'])

        self.assertIn('split_cruise', wl._PHASE_TO_SG_TASK)
        self.assertIn('split_cruise_efficiency_wh_km', wl._PHASE_TO_SG_TASK['split_cruise'])
        self.assertIn('scissor_jet_thrust_n', wl._PHASE_TO_SG_TASK['split_cruise'])
        self.assertIn('parasite_drag_reduction_pct', wl._PHASE_TO_SG_TASK['split_cruise'])

        self.assertIn('reflex_soar_efficiency_wh_km', wl._PHASE_TO_SG_TASK['endurance_suite'])
        self.assertIn('split_cruise_efficiency_wh_km', wl._PHASE_TO_SG_TASK['endurance_suite'])

    def test_cmts_champions(self):
        self.assertIn('reflex_soar', wl._CMTS_CHAMPIONS)
        champ_rs = wl._CMTS_CHAMPIONS['reflex_soar']
        self.assertEqual(champ_rs['hyp_id'], 'H-REFLEX-SOAR-1')
        self.assertAlmostEqual(champ_rs['wh_per_km'], 2.72, places=2)

        self.assertIn('split_cruise', wl._CMTS_CHAMPIONS)
        champ_sc = wl._CMTS_CHAMPIONS['split_cruise']
        self.assertEqual(champ_sc['hyp_id'], 'H-SPLIT-CRUISE-1')
        self.assertAlmostEqual(champ_sc['wh_per_km'], 2.36, places=2)


class TestReflexSoarAndSplitCruiseDiscoveryBounds(unittest.TestCase):
    """Verifies expanded discovery parameter bounds."""

    def test_reflex_soar_bounds(self):
        expected_bounds = [
            'h_rs1_flap_amp_deg',
            'h_rs1_glide_aoa_deg',
            'h_rs1_reflex_trim_deg',
            'h_rs1_freq_hz',
            'h_rs1_v_min_abort_ms',
            'h_rs1_holding_current_a',
        ]
        for param in expected_bounds:
            self.assertIn(param, wl._EXPANDED_DISCOVERY_BOUNDS, f"{param} missing from bounds")
            b_min, b_max = wl._EXPANDED_DISCOVERY_BOUNDS[param]
            self.assertLess(b_min, b_max)

    def test_split_cruise_bounds(self):
        expected_bounds = [
            'h_sc1_upper_min_deg',
            'h_sc1_upper_max_deg',
            'h_sc1_lower_min_deg',
            'h_sc1_lower_max_deg',
            'h_sc1_glide_reflex_deg',
            'h_sc1_glide_aoa_deg',
            'h_sc1_freq_hz',
            'h_sc1_holding_current_a',
        ]
        for param in expected_bounds:
            self.assertIn(param, wl._EXPANDED_DISCOVERY_BOUNDS, f"{param} missing from bounds")
            b_min, b_max = wl._EXPANDED_DISCOVERY_BOUNDS[param]
            self.assertLess(b_min, b_max)


class TestReflexSoarAndSplitCruiseWaveLeverIntegration(unittest.TestCase):
    """Verifies hypothesis mapping in Wave Function Lever and throttle zones."""

    def test_zone_mapping(self):
        self.assertIn('H-REFLEX-SOAR-1', wl._HYP_TO_LEVER_ZONE)
        self.assertEqual(wl._HYP_TO_LEVER_ZONE['H-REFLEX-SOAR-1'], 'GLIDE_SOAR')

        self.assertIn('H-SPLIT-CRUISE-1', wl._HYP_TO_LEVER_ZONE)
        self.assertEqual(wl._HYP_TO_LEVER_ZONE['H-SPLIT-CRUISE-1'], 'CRUISE')

    def test_zone_weights(self):
        self.assertIn('GLIDE_SOAR', wl._lever_mode_weights)
        self.assertIn('H-REFLEX-SOAR-1', wl._lever_mode_weights['GLIDE_SOAR'])
        self.assertGreater(wl._lever_mode_weights['GLIDE_SOAR']['H-REFLEX-SOAR-1'], 1.2)

        self.assertIn('CRUISE', wl._lever_mode_weights)
        self.assertIn('H-SPLIT-CRUISE-1', wl._lever_mode_weights['CRUISE'])
        self.assertGreater(wl._lever_mode_weights['CRUISE']['H-SPLIT-CRUISE-1'], 1.2)

        self.assertIn('TRIM_DETENT', wl._lever_mode_weights)
        self.assertIn('H-REFLEX-SOAR-1', wl._lever_mode_weights['TRIM_DETENT'])
        self.assertIn('H-SPLIT-CRUISE-1', wl._lever_mode_weights['TRIM_DETENT'])


class TestReflexSoarAndSplitCruiseGenomeRouting(unittest.TestCase):
    """Verifies trial update routing in _update_genome_from_trial."""

    def setUp(self):
        self._orig_cruise_weights = dict(wl._lever_mode_weights.get('CRUISE', {}))
        self._orig_soar_weights = dict(wl._lever_mode_weights.get('GLIDE_SOAR', {}))
        self._orig_endurance = dict(wl._MW_ENDURANCE)
        self._orig_perf = dict(wl._MW_PERFORMANCE)
        self._orig_glide = dict(wl._MW_GLIDE)
        self._orig_soar = dict(wl._MW_SOARING)
        self._orig_pitch = dict(wl._MW_PITCH)

        # Reset axes to equal weights so the test does not hit 0.60 saturation clamp
        for ax in (wl._MW_ENDURANCE, wl._MW_PERFORMANCE, wl._MW_GLIDE, wl._MW_SOARING, wl._MW_PITCH):
            eq = 1.0 / len(ax)
            for k in ax: ax[k] = eq

    def tearDown(self):
        wl._lever_mode_weights['CRUISE'] = self._orig_cruise_weights
        wl._lever_mode_weights['GLIDE_SOAR'] = self._orig_soar_weights
        wl._MW_ENDURANCE.update(self._orig_endurance)
        wl._MW_PERFORMANCE.update(self._orig_perf)
        wl._MW_GLIDE.update(self._orig_glide)
        wl._MW_SOARING.update(self._orig_soar)
        wl._MW_PITCH.update(self._orig_pitch)

    def test_h_reflex_soar_1_genome_update(self):
        sensors = {'V_ms': 9.0, 'wh_per_km': 2.72}
        init_endurance = wl._MW_ENDURANCE.get('sustained_wh_per_km', 0.0)
        init_glide = wl._MW_GLIDE.get('min_sink_aoa', 0.0)
        init_soar = wl._MW_SOARING.get('glide_aoa', 0.0)
        init_eff = wl._MW_PERFORMANCE.get('cruise_efficiency', 0.0)

        wl._update_genome_from_trial('H-REFLEX-SOAR-1', 0.20, sensors)

        self.assertGreater(wl._MW_ENDURANCE['sustained_wh_per_km'], init_endurance)
        self.assertGreater(wl._MW_GLIDE['min_sink_aoa'], init_glide)
        self.assertGreater(wl._MW_SOARING['glide_aoa'], init_soar)
        self.assertGreater(wl._MW_PERFORMANCE['cruise_efficiency'], init_eff)

    def test_h_split_cruise_1_genome_update(self):
        sensors = {'V_ms': 12.0, 'wh_per_km': 2.36}
        init_endurance = wl._MW_ENDURANCE.get('sustained_wh_per_km', 0.0)
        init_eff = wl._MW_PERFORMANCE.get('cruise_efficiency', 0.0)
        init_top_speed = wl._MW_PERFORMANCE.get('top_speed', 0.0)
        init_pitch = wl._MW_PITCH.get('pitch_aoa', 0.0)

        wl._update_genome_from_trial('H-SPLIT-CRUISE-1', 0.20, sensors)

        self.assertGreater(wl._MW_ENDURANCE['sustained_wh_per_km'], init_endurance)
        self.assertGreater(wl._MW_PERFORMANCE['cruise_efficiency'], init_eff)
        self.assertGreater(wl._MW_PERFORMANCE['top_speed'], init_top_speed)
        self.assertGreater(wl._MW_PITCH['pitch_aoa'], init_pitch)


if __name__ == '__main__':
    unittest.main()
