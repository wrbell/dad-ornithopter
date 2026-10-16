#!/usr/bin/env python3
"""
Unit & Integration Tests: Hypotheses H-BAND-1 & H-ZONE-1
Stroke Band Allocation & Lever Zone Adaptive Regime Partitioning
================================================================================
Comprehensive verification suite covering all 5 ALC modalities:
  1. Hypothesis Registration & Metadata in AERO_HYPOTHESES, _ADV_NEW_HYPS,
     _hypothesis_queue, and BIOMIMETIC_HYPOTHESES
  2. Configuration validation, aerodynamic bounds, and Red Team physical safeguards
     (H-BAND-1: 3-band geometric stroke decomposition 80_40_80, 2+4+2 allocation
      resolving flap angle as dihedral to eliminate lateral force cancellation;
      H-ZONE-1: Lever zone adaptive regime partitioning, cruise halves 4+4 vs
      hover horizon 2+4+2 with raised-cosine S-curve smoothing)
  3. Scoring Tasks in _SG_TASKS, ALSS Segments and Curricula in _ALSS_SEGMENTS &
     _ALSS_CURRICULUM, and CMTS Context Champions in _CMTS_CHAMPIONS
  4. Expanded Discovery Bounds in _EXPANDED_DISCOVERY_BOUNDS
  5. Wave Function Lever & Throttle Mapping in _HYP_TO_LEVER_ZONE and _lever_mode_weights
  6. Genome Update Routing in _update_genome_from_trial
  7. AI Observer Triggers 49 & 50 in ai_observer_tick
  8. Discovery Selection by focus keywords and sensor triggers
  9. Kinematic Wave Generators wave_stroke_band_allocation, wave_regime_partitioning,
     and Evaluator hooks evaluate_band_allocation, evaluate_regime_partitioning
  10. Diagnostics and status reporting in band_status and advanced_physics_status
"""

import sys
import os
import unittest
import math

# Ensure firmware directory is in python path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'firmware'))

import mco_wave_library as wl


class TestBandZoneRegistration(unittest.TestCase):
    """Verifies H-BAND-1 and H-ZONE-1 registration across all catalogs and tables."""

    TARGET_HIDS = ('H-BAND-1', 'H-ZONE-1')

    def test_registered_in_adv_new_hyps(self):
        adv_ids = {h['id'] for h in wl._ADV_NEW_HYPS}
        for hid in self.TARGET_HIDS:
            self.assertIn(hid, adv_ids, f"{hid} missing from _ADV_NEW_HYPS")

    def test_registered_in_aero_hypotheses(self):
        aero_ids = {h['id'] for h in wl.AERO_HYPOTHESES}
        for hid in self.TARGET_HIDS:
            self.assertIn(hid, aero_ids, f"{hid} missing from AERO_HYPOTHESES")

        h_band = next(h for h in wl.AERO_HYPOTHESES if h['id'] == 'H-BAND-1')
        self.assertEqual(h_band['target'], 'trim')
        self.assertIn('hover', h_band['eval_phases'])
        self.assertIn('loiter', h_band['eval_phases'])
        self.assertIn('cruise', h_band['eval_phases'])
        self.assertIn('endurance', h_band['eval_phases'])
        self.assertEqual(h_band['priority'], 1)
        self.assertEqual(h_band['v_min'], 0.0)
        self.assertEqual(h_band['v_max'], 16.0)

        h_zone = next(h for h in wl.AERO_HYPOTHESES if h['id'] == 'H-ZONE-1')
        self.assertEqual(h_zone['target'], 'cruise')
        self.assertIn('hover', h_zone['eval_phases'])
        self.assertIn('loiter', h_zone['eval_phases'])
        self.assertIn('cruise', h_zone['eval_phases'])
        self.assertIn('endurance', h_zone['eval_phases'])
        self.assertEqual(h_zone['priority'], 1)
        self.assertEqual(h_zone['v_min'], 0.0)
        self.assertEqual(h_zone['v_max'], 20.0)

    def test_registered_in_hypothesis_queue(self):
        q_ids = {h['id'] for h in wl._hypothesis_queue}
        for hid in self.TARGET_HIDS:
            self.assertIn(hid, q_ids, f"{hid} missing from _hypothesis_queue")

    def test_registered_in_biomimetic_catalog(self):
        for hid in self.TARGET_HIDS:
            self.assertIn(hid, wl.BIOMIMETIC_HYPOTHESES, f"{hid} missing from BIOMIMETIC_HYPOTHESES")


class TestBandZoneConfigIntegrity(unittest.TestCase):
    """Validates configuration parameters and boundaries for both hypotheses."""

    def test_h_band_1_config(self):
        h = next(h for h in wl.AERO_HYPOTHESES if h['id'] == 'H-BAND-1')
        cfg = h['config']
        self.assertTrue(cfg.get('_h_band_1'))
        self.assertEqual(cfg.get('preset'), '80_40_80')
        self.assertEqual(cfg.get('allocation'), (2, 4, 2))
        self.assertEqual(cfg.get('h_band1_preset'), '80_40_80')
        self.assertEqual(cfg.get('h_band1_horizon_wings'), 4)
        self.assertEqual(cfg.get('h_band1_outer_wings'), 2)
        self.assertEqual(cfg.get('h_band1_v_ref_ms'), 9.0)
        self.assertEqual(cfg.get('h_band1_freq_hz'), 1.87)
        self.assertEqual(cfg.get('_scorer'), 'mean_lift_efficiency')

    def test_h_zone_1_config(self):
        h = next(h for h in wl.AERO_HYPOTHESES if h['id'] == 'H-ZONE-1')
        cfg = h['config']
        self.assertTrue(cfg.get('_h_zone_1'))
        self.assertEqual(cfg.get('cruise_preset'), 'halves')
        self.assertEqual(cfg.get('cruise_allocation'), (4, 4))
        self.assertEqual(cfg.get('hover_preset'), '80_40_80')
        self.assertEqual(cfg.get('hover_allocation'), (2, 4, 2))
        self.assertEqual(cfg.get('transition_s'), 2.0)
        self.assertEqual(cfg.get('v_switch_ms'), 6.0)
        self.assertEqual(cfg.get('h_zone1_transition_dur_s'), 2.0)
        self.assertEqual(cfg.get('h_zone1_v_switch_ms'), 6.0)
        self.assertEqual(cfg.get('_scorer'), 'band_transition_force_step_n')


class TestBandZoneScoringCurriculumAndCMTS(unittest.TestCase):
    """Verifies scoring tasks, ALSS segments, curriculum, and CMTS context champions."""

    def test_sg_task_metrics(self):
        expected_tasks = [
            ('mean_lift_efficiency', +1, 'endurance', 'mean_lift_efficiency'),
            ('lift_per_watt', +1, 'endurance', 'lift_per_watt'),
            ('band_transition_force_step_n', -1, 'stability', 'band_transition_force_step_n'),
            ('lateral_force_waste_frac', -1, 'endurance', 'lateral_force_waste_frac'),
        ]
        for task_name, direction, axis, sensor in expected_tasks:
            self.assertIn(task_name, wl._SG_TASKS, f"{task_name} missing from _SG_TASKS")
            task = wl._SG_TASKS[task_name]
            self.assertEqual(task['direction'], direction)
            self.assertEqual(task['axis'], axis)
            self.assertEqual(task['sensor'], sensor)

    def test_alss_curriculum_segments(self):
        self.assertIn('band1_allocation_b', wl._ALSS_SEGMENTS)
        seg_band = wl._ALSS_SEGMENTS['band1_allocation_b']
        self.assertEqual(seg_band[0], 'cruise')
        self.assertEqual(seg_band[1], 'H-BAND-1')
        self.assertIn('band1_allocation_b', wl._ALSS_CURRICULUM.get('B', []))

        self.assertIn('zone1_partitioning_b', wl._ALSS_SEGMENTS)
        seg_zone = wl._ALSS_SEGMENTS['zone1_partitioning_b']
        self.assertEqual(seg_zone[0], 'cruise')
        self.assertEqual(seg_zone[1], 'H-ZONE-1')
        self.assertIn('zone1_partitioning_b', wl._ALSS_CURRICULUM.get('B', []))

    def test_phase_to_sg_task_mapping(self):
        self.assertIn('stroke_band_allocation', wl._PHASE_TO_SG_TASK)
        tasks_band = wl._PHASE_TO_SG_TASK['stroke_band_allocation']
        self.assertIn('mean_lift_efficiency', tasks_band)
        self.assertIn('lift_per_watt', tasks_band)
        self.assertIn('lateral_force_waste_frac', tasks_band)

        self.assertIn('regime_partitioning', wl._PHASE_TO_SG_TASK)
        tasks_zone = wl._PHASE_TO_SG_TASK['regime_partitioning']
        self.assertIn('band_transition_force_step_n', tasks_zone)
        self.assertIn('mean_lift_efficiency', tasks_zone)

        tasks_endurance = wl._PHASE_TO_SG_TASK.get('endurance_suite', [])
        self.assertIn('mean_lift_efficiency', tasks_endurance)
        self.assertIn('lift_per_watt', tasks_endurance)
        self.assertIn('band_transition_force_step_n', tasks_endurance)
        self.assertIn('lateral_force_waste_frac', tasks_endurance)

    def test_cmts_champions(self):
        self.assertIn('stroke_band_allocation', wl._CMTS_CHAMPIONS)
        champ_b = wl._CMTS_CHAMPIONS['stroke_band_allocation']
        self.assertEqual(champ_b['hyp_id'], 'H-BAND-1')
        self.assertAlmostEqual(champ_b['mean_lift_efficiency'], 0.750, places=3)

        self.assertIn('regime_partitioning', wl._CMTS_CHAMPIONS)
        champ_z = wl._CMTS_CHAMPIONS['regime_partitioning']
        self.assertEqual(champ_z['hyp_id'], 'H-ZONE-1')
        self.assertEqual(champ_z['band_transition_force_step_n'], 0.0)


class TestBandZoneDiscoveryBounds(unittest.TestCase):
    """Verifies expanded discovery bounds."""

    def test_discovery_bounds_existence_and_ranges(self):
        expected_bounds = [
            'h_band1_horizon_wings',
            'h_band1_outer_wings',
            'h_band1_v_ref_ms',
            'h_band1_freq_hz',
            'h_zone1_transition_dur_s',
            'h_zone1_v_switch_ms',
        ]
        for param in expected_bounds:
            self.assertIn(param, wl._EXPANDED_DISCOVERY_BOUNDS, f"{param} missing from _EXPANDED_DISCOVERY_BOUNDS")
            low, high = wl._EXPANDED_DISCOVERY_BOUNDS[param]
            self.assertLess(low, high)


class TestBandZoneWaveLeverIntegration(unittest.TestCase):
    """Verifies lever zone assignment and weight weighting."""

    def test_lever_zone_mappings(self):
        self.assertIn('H-BAND-1', wl._HYP_TO_LEVER_ZONE)
        self.assertEqual(wl._HYP_TO_LEVER_ZONE['H-BAND-1'], 'TRIM_DETENT')

        self.assertIn('H-ZONE-1', wl._HYP_TO_LEVER_ZONE)
        self.assertEqual(wl._HYP_TO_LEVER_ZONE['H-ZONE-1'], 'CRUISE')

    def test_lever_weights(self):
        self.assertIn('TRIM_DETENT', wl._lever_mode_weights)
        self.assertIn('H-BAND-1', wl._lever_mode_weights['TRIM_DETENT'])
        self.assertGreater(wl._lever_mode_weights['TRIM_DETENT']['H-BAND-1'], 0.8)

        self.assertIn('CRUISE', wl._lever_mode_weights)
        self.assertIn('H-ZONE-1', wl._lever_mode_weights['CRUISE'])
        self.assertGreater(wl._lever_mode_weights['CRUISE']['H-ZONE-1'], 0.8)


class TestBandZoneGenomeRouting(unittest.TestCase):
    """Verifies trial update routing in _update_genome_from_trial."""

    def setUp(self):
        self._orig_endurance = dict(wl._MW_ENDURANCE)
        self._orig_perf = dict(wl._MW_PERFORMANCE)
        self._orig_stability = dict(wl._MW_STABILITY)
        self._orig_transition = dict(wl._MW_TRANSITION)

        for ax in (wl._MW_ENDURANCE, wl._MW_PERFORMANCE, wl._MW_STABILITY, wl._MW_TRANSITION):
            eq = 1.0 / len(ax)
            for k in ax: ax[k] = eq

    def tearDown(self):
        wl._MW_ENDURANCE.update(self._orig_endurance)
        wl._MW_PERFORMANCE.update(self._orig_perf)
        wl._MW_STABILITY.update(self._orig_stability)
        wl._MW_TRANSITION.update(self._orig_transition)

    def test_h_band_1_genome_update(self):
        sensors = {'V_ms': 9.0}
        init_endurance = wl._MW_ENDURANCE.get('sustained_wh_per_km', 0.0)
        init_eff = wl._MW_PERFORMANCE.get('cruise_efficiency', 0.0)
        init_dih = wl._MW_STABILITY.get('stroke_dihedral', 0.0)

        wl._update_genome_from_trial('H-BAND-1', 0.20, sensors)

        self.assertGreater(wl._MW_ENDURANCE['sustained_wh_per_km'], init_endurance)
        self.assertGreater(wl._MW_PERFORMANCE['cruise_efficiency'], init_eff)
        self.assertGreater(wl._MW_STABILITY['stroke_dihedral'], init_dih)

    def test_h_zone_1_genome_update(self):
        sensors = {'V_ms': 8.0}
        init_endurance = wl._MW_ENDURANCE.get('sustained_wh_per_km', 0.0)
        init_eff = wl._MW_PERFORMANCE.get('cruise_efficiency', 0.0)
        init_trans = wl._MW_TRANSITION.get('blend_smoothness', 0.0)

        wl._update_genome_from_trial('H-ZONE-1', 0.20, sensors)

        self.assertGreater(wl._MW_ENDURANCE['sustained_wh_per_km'], init_endurance)
        self.assertGreater(wl._MW_PERFORMANCE['cruise_efficiency'], init_eff)
        self.assertGreater(wl._MW_TRANSITION['blend_smoothness'], init_trans)


class TestBandZoneAIObserverTriggers(unittest.TestCase):
    """Verifies AI Observer triggers 49 & 50 for H-BAND-1 and H-ZONE-1."""

    def setUp(self):
        wl._ALC_SHARED['priority_hyp_queue'].clear()

    def test_h_band_1_observer_trigger(self):
        sensors = {'V_ms': 5.0, 'hover_lift_demand': True}
        obs = wl.ai_observer_tick(sensors, {})
        self.assertTrue(obs.get('h_band1_active'))
        queue_hids = [item['hyp_id'] for item in wl._ALC_SHARED['priority_hyp_queue']]
        self.assertIn('H-BAND-1', queue_hids)

    def test_h_zone_1_observer_trigger(self):
        sensors = {'V_ms': 7.0, 'lever_pct': 40.0, 'adaptive_regime_transition': True}
        obs = wl.ai_observer_tick(sensors, {})
        self.assertTrue(obs.get('h_zone1_active'))
        queue_hids = [item['hyp_id'] for item in wl._ALC_SHARED['priority_hyp_queue']]
        self.assertIn('H-ZONE-1', queue_hids)


class TestBandZoneDiscoverySelector(unittest.TestCase):
    """Verifies discovery selection by focus keywords and sensor triggers."""

    def test_discovery_by_focus_keywords(self):
        band_keywords = ['h_band_1', 'band_allocation', 'stroke_band', 'stroke_band_allocation', 'dihedral_stroke']
        for kw in band_keywords:
            h = wl.discovery_hypothesis_select(focus=kw)
            self.assertIsNotNone(h, f"Failed for focus={kw}")
            self.assertEqual(h['id'], 'H-BAND-1')

        zone_keywords = ['h_zone_1', 'regime_partitioning', 'lever_zone_partitioning', 'zone_partitioning', 'adaptive_regime']
        for kw in zone_keywords:
            h = wl.discovery_hypothesis_select(focus=kw)
            self.assertIsNotNone(h, f"Failed for focus={kw}")
            self.assertEqual(h['id'], 'H-ZONE-1')

    def test_discovery_by_sensor_triggers(self):
        cases = [
            ({'h_band_1_active': True}, 'H-BAND-1'),
            ({'stroke_band_allocation_active': True}, 'H-BAND-1'),
            ({'band_allocation_active': True}, 'H-BAND-1'),
            ({'h_zone_1_active': True}, 'H-ZONE-1'),
            ({'regime_partitioning_active': True}, 'H-ZONE-1'),
            ({'adaptive_regime_active': True}, 'H-ZONE-1'),
        ]
        for s_dict, expected_hid in cases:
            h = wl.discovery_hypothesis_select(sensors=s_dict)
            self.assertIsNotNone(h, f"Failed for sensors={s_dict}")
            self.assertEqual(h['id'], expected_hid)


class TestBandZoneKinematicGeneratorsAndEvaluators(unittest.TestCase):
    """Verifies waveform generators, analytic calculations, and evaluators."""

    def test_band_mean_lift_efficiency_analytics(self):
        # Baseline halves: cos(50 deg) ~ 0.64278
        eff_halves = wl.band_mean_lift_efficiency('halves', (4, 4))
        self.assertAlmostEqual(eff_halves, 0.643, places=3)

        # 80_40_80 with (2, 4, 2): 4 wings at cos(0)=1.0, 4 wings at cos(60)=0.5 -> mean = (4*1.0 + 4*0.5)/8 = 0.750
        eff_3band_242 = wl.band_mean_lift_efficiency('80_40_80', (2, 4, 2))
        self.assertAlmostEqual(eff_3band_242, 0.750, places=3)

        # 80_40_80 with (1, 6, 1): 6 wings at cos(0)=1.0, 2 wings at cos(60)=0.5 -> mean = (6*1.0 + 2*0.5)/8 = 0.875
        eff_3band_161 = wl.band_mean_lift_efficiency('80_40_80', (1, 6, 1))
        self.assertAlmostEqual(eff_3band_161, 0.875, places=3)

    def test_wave_stroke_band_allocation_generator(self):
        res = wl.wave_stroke_band_allocation(t_sec=0.25, f_hz=1.87, preset='80_40_80', allocation=(2, 4, 2))
        self.assertEqual(res['hyp_id'], 'H-BAND-1')
        self.assertAlmostEqual(res['mean_lift_efficiency'], 0.750, places=3)
        self.assertAlmostEqual(res['lateral_force_waste_frac'], 0.250, places=3)
        self.assertGreater(res['lift_per_watt'], 0.10)
        self.assertEqual(len(res['bands']), 3)

        # Band 1: Upper (+60 deg centre)
        self.assertAlmostEqual(res['bands'][0]['centre_deg'], 60.0)
        # Band 2: Horizon (0 deg centre)
        self.assertAlmostEqual(res['bands'][1]['centre_deg'], 0.0)
        # Band 3: Lower (-60 deg centre)
        self.assertAlmostEqual(res['bands'][2]['centre_deg'], -60.0)

    def test_evaluate_band_allocation(self):
        ev = wl.evaluate_band_allocation({'preset': '80_40_80', 'allocation': (2, 4, 2)})
        self.assertEqual(ev['hyp_id'], 'H-BAND-1')
        self.assertAlmostEqual(ev['mean_lift_efficiency'], 0.750, places=3)
        self.assertTrue(ev['safeguards_verified'])

    def test_wave_regime_partitioning_and_s_curve(self):
        # Reset band state
        wl._band_state['active_preset'] = 'halves'
        wl._band_state['active_allocation'] = (4, 4)
        wl._band_state['transition_in_progress'] = False

        # Cruise regime (high speed)
        res_cruise = wl.wave_regime_partitioning(t_sec=0.0, v_ms=12.0, lever_pct=60.0)
        self.assertEqual(res_cruise['active_regime'], 'CRUISE')
        self.assertEqual(res_cruise['active_preset'], 'halves')

        # Request transition to hover (low speed)
        res_hover = wl.wave_regime_partitioning(t_sec=1.0, v_ms=4.0, lever_pct=30.0, transition_dur_s=2.0)
        self.assertEqual(res_hover['active_regime'], 'HOVER')
        self.assertTrue(res_hover['transition_in_progress'])
        self.assertGreater(res_hover['band_transition_force_step_n'], 0.0)

        # Midway through transition (t = 2.0s -> elapsed = 1.0s of 2.0s = 50%)
        res_mid = wl.wave_regime_partitioning(t_sec=2.0, v_ms=4.0, lever_pct=30.0, transition_dur_s=2.0)
        self.assertTrue(res_mid['transition_in_progress'])
        # Raised-cosine blend at 50% progress should be halfway between 0.643 and 0.750 (~0.696)
        self.assertGreater(res_mid['mean_lift_efficiency'], 0.643)
        self.assertLess(res_mid['mean_lift_efficiency'], 0.750)

        # After transition completes (t = 3.5s -> elapsed = 2.5s >= 2.0s)
        res_done = wl.wave_regime_partitioning(t_sec=3.5, v_ms=4.0, lever_pct=30.0, transition_dur_s=2.0)
        self.assertFalse(res_done['transition_in_progress'])
        self.assertEqual(res_done['active_preset'], '80_40_80')
        self.assertAlmostEqual(res_done['mean_lift_efficiency'], 0.750, places=3)

    def test_evaluate_regime_partitioning(self):
        ev = wl.evaluate_regime_partitioning({'V_ms': 10.0, 'lever_pct': 60.0})
        self.assertEqual(ev['hyp_id'], 'H-ZONE-1')
        self.assertEqual(ev['active_regime'], 'CRUISE')
        self.assertTrue(ev['safeguards_verified'])


class TestBandStatusTelemetry(unittest.TestCase):
    """Verifies that band_status command executes and contains key diagnostics."""

    def test_band_status_report(self):
        rep = wl.band_status()
        self.assertIn('H-BAND-1', rep)
        self.assertIn('H-ZONE-1', rep)
        self.assertIn('STROKE BAND ALLOCATION STATUS', rep)
        self.assertIn('ANALYTIC RANKING OF CANDIDATE ALLOCATIONS', rep)
        self.assertIn('MODEL CAVEAT, STATED PLAINLY', rep)


if __name__ == '__main__':
    unittest.main()
