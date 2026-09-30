"""
Unit & Integration Tests: Hypothesis H-GLIDER-1
Tandem Metachronal Motorglider Sustainer Loiter
================================================================================
Comprehensive verification suite covering:
  1. Hypothesis Registration & Metadata in AERO_HYPOTHESES, _ADV_NEW_HYPS,
     _hypothesis_queue, and BIOMIMETIC_HYPOTHESES
  2. Configuration validation and parameter integrity (4 pairs, 3 glide @ 6°,
     1 sustainer, holding current limit 7.0A, thermal rotation 30.0s)
  3. Scoring Tasks in _SG_TASKS and Curriculum Segments in _ALSS_SEGMENTS / _ALSS_CURRICULUM
  4. CMTS Context Champions in _CMTS_CHAMPIONS ('motorglider')
  5. Discovery Bounds in _DISCOVERY_BOUNDS
  6. Genome Update Routing in _update_genome_from_trial
  7. AI Observer Trigger 46 in ai_observer_tick
  8. Discovery Selection by focus keywords and sensor triggers
  9. Wave Kinematic Generator wave_endurance_motorglider_loiter
  10. Diagnostics and status reporting in advanced_physics_status and endurance_status
"""

import sys
import os
import unittest
import math

# Ensure firmware directory is in python path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'firmware'))

import mco_wave_library as wl


class TestHGlider1Registration(unittest.TestCase):
    """Verifies H-GLIDER-1 registration across all catalogs and tables."""

    TARGET_HID = 'H-GLIDER-1'

    def test_registered_in_adv_new_hyps(self):
        adv_ids = {h['id'] for h in wl._ADV_NEW_HYPS}
        self.assertIn(self.TARGET_HID, adv_ids, f"{self.TARGET_HID} missing from _ADV_NEW_HYPS")

    def test_registered_in_aero_hypotheses(self):
        aero_ids = {h['id'] for h in wl.AERO_HYPOTHESES}
        self.assertIn(self.TARGET_HID, aero_ids, f"{self.TARGET_HID} missing from AERO_HYPOTHESES")
        h = next(h for h in wl.AERO_HYPOTHESES if h['id'] == self.TARGET_HID)
        self.assertEqual(h['target'], 'endurance')
        self.assertIn('cruise', h['eval_phases'])
        self.assertIn('endurance', h['eval_phases'])
        self.assertEqual(h['priority'], 1)
        self.assertEqual(h['v_min'], 9.0)
        self.assertEqual(h['v_max'], 22.0)

    def test_registered_in_hypothesis_queue(self):
        q_ids = {h['id'] for h in wl._hypothesis_queue}
        self.assertIn(self.TARGET_HID, q_ids, f"{self.TARGET_HID} missing from _hypothesis_queue")

    def test_registered_in_biomimetic_catalog(self):
        self.assertIn(self.TARGET_HID, wl.BIOMIMETIC_HYPOTHESES, f"{self.TARGET_HID} missing from BIOMIMETIC_HYPOTHESES")


class TestHGlider1ConfigIntegrity(unittest.TestCase):
    """Validates configuration parameters and boundaries for H-GLIDER-1."""

    def test_h_glider_1_config(self):
        h = next(h for h in wl.AERO_HYPOTHESES if h['id'] == 'H-GLIDER-1')
        cfg = h['config']
        self.assertTrue(cfg.get('_h_glider_1'))
        self.assertEqual(cfg.get('h_glider1_active_pair'), 4)
        self.assertEqual(cfg.get('h_glider1_glide_pairs_count'), 3)
        self.assertEqual(cfg.get('h_glider1_glide_dihedral_deg'), 6.0)
        self.assertEqual(cfg.get('h_glider1_holding_current_limit_a'), 7.0)
        self.assertEqual(cfg.get('h_glider1_thermal_rotation_s'), 30.0)
        self.assertEqual(cfg.get('_scorer'), 'motorglider_efficiency_wh_km')
        self.assertEqual(cfg.get('freq_hz'), 2.2)
        self.assertEqual(cfg.get('amp_port_deg'), 85.0)
        self.assertEqual(cfg.get('amp_stbd_deg'), 85.0)


class TestHGlider1ScoringCurriculumAndCMTS(unittest.TestCase):
    """Verifies scoring tasks, ALSS segments, curriculum, and CMTS context champions."""

    def test_sg_tasks_metrics(self):
        expected_tasks = [
            ('motorglider_efficiency_wh_km', -1),
            ('sustainer_thrust_fraction', +1),
        ]
        for task_name, direction in expected_tasks:
            self.assertIn(task_name, wl._SG_TASKS, f"{task_name} missing from _SG_TASKS")
            self.assertEqual(wl._SG_TASKS[task_name]['direction'], direction)

    def test_alss_curriculum_segments(self):
        self.assertIn('glider1_sustainer_b', wl._ALSS_SEGMENTS)
        seg = wl._ALSS_SEGMENTS['glider1_sustainer_b']
        self.assertEqual(seg[0], 'cruise')
        self.assertEqual(seg[1], 'H-GLIDER-1')
        self.assertEqual(seg[2], 25.0)  # min duration
        self.assertEqual(seg[3], 180.0) # max duration
        self.assertEqual(seg[4], 10.5)  # target airspeed V_ms
        self.assertIn('glider1_sustainer_b', wl._ALSS_CURRICULUM.get('B', []))

    def test_cmts_champions(self):
        self.assertIn('motorglider', wl._CMTS_CHAMPIONS)
        champ = wl._CMTS_CHAMPIONS['motorglider']
        self.assertEqual(champ['hyp_id'], 'H-GLIDER-1')
        self.assertLessEqual(champ['wh_per_km'], 8.5)
        self.assertIn('config', champ)


class TestHGlider1DiscoveryBoundsAndGenome(unittest.TestCase):
    """Verifies discovery bounds and genome trial update routing."""

    def setUp(self):
        import copy
        self._orig_endurance = dict(wl._MW_ENDURANCE)
        self._orig_glide = dict(wl._MW_GLIDE)
        self._orig_perf = dict(wl._MW_PERFORMANCE)
        self._orig_lever_weights = copy.deepcopy(wl._lever_mode_weights)

    def tearDown(self):
        import copy
        wl._MW_ENDURANCE.clear()
        wl._MW_ENDURANCE.update(self._orig_endurance)
        wl._MW_GLIDE.clear()
        wl._MW_GLIDE.update(self._orig_glide)
        wl._MW_PERFORMANCE.clear()
        wl._MW_PERFORMANCE.update(self._orig_perf)
        wl._lever_mode_weights.clear()
        wl._lever_mode_weights.update(copy.deepcopy(self._orig_lever_weights))

    def test_discovery_bounds(self):
        expected_bounds = [
            'h_glider1_active_pair',
            'h_glider1_glide_pairs_count',
            'h_glider1_glide_dihedral_deg',
            'h_glider1_holding_current_limit_a',
            'h_glider1_thermal_rotation_s',
        ]
        for param in expected_bounds:
            self.assertIn(param, wl._EXPANDED_DISCOVERY_BOUNDS, f"{param} missing from _EXPANDED_DISCOVERY_BOUNDS")
            b_min, b_max = wl._EXPANDED_DISCOVERY_BOUNDS[param]
            self.assertLess(b_min, b_max)

    def test_genome_update_routing(self):
        sensors = {'V_ms': 10.5, 'wh_per_km': 7.8, 'sink_rate_ms': 0.45}
        initial_endurance = wl._MW_ENDURANCE.get('sustained_wh_per_km', 0.0)
        initial_glide = wl._MW_GLIDE.get('min_sink_aoa', 0.0)
        initial_perf = wl._MW_PERFORMANCE.get('cruise_efficiency', 0.0)

        wl._update_genome_from_trial('H-GLIDER-1', 0.25, sensors)

        self.assertGreater(wl._MW_ENDURANCE['sustained_wh_per_km'], initial_endurance)
        self.assertGreater(wl._MW_GLIDE['min_sink_aoa'], initial_glide)
        self.assertGreater(wl._MW_PERFORMANCE['cruise_efficiency'], initial_perf)


class TestHGlider1AIObserverAndSelection(unittest.TestCase):
    """Verifies AI Observer Trigger 46 and Discovery Selection."""

    def test_ai_observer_trigger_46(self):
        wl._ALC_SHARED['priority_hyp_queue'].clear()
        sensors = {
            'V_ms': 11.0,
            'lever_pct': 50.0,
            'motorglider_mode': True,
        }
        obs = wl.ai_observer_tick(sensors, {})
        self.assertTrue(obs.get('h_glider1_active'))
        queue_hids = [item['hyp_id'] for item in wl._ALC_SHARED['priority_hyp_queue']]
        self.assertIn('H-GLIDER-1', queue_hids)

    def test_discovery_select_by_focus_keywords(self):
        focus_keywords = ['h_glider_1', 'motorglider', 'sustainer_loiter', 'thermal_rotation']
        for kw in focus_keywords:
            h = wl.discovery_hypothesis_select(focus=kw)
            self.assertIsNotNone(h, f"Focus '{kw}' returned None")
            self.assertEqual(h['id'], 'H-GLIDER-1', f"Focus '{kw}' did not return H-GLIDER-1")

    def test_discovery_select_by_sensor_triggers(self):
        sensor_cases = [
            {'h_glider_1_active': True},
            {'motorglider_active': True},
            {'sustainer_loiter_active': True},
        ]
        for s in sensor_cases:
            h = wl.discovery_hypothesis_select(sensors=s)
            self.assertIsNotNone(h, f"Sensors '{s}' returned None")
            self.assertEqual(h['id'], 'H-GLIDER-1', f"Sensors '{s}' did not return H-GLIDER-1")

    def test_discovery_select_endurance_suite(self):
        h = wl.discovery_hypothesis_select(focus='endurance_suite')
        self.assertIsNotNone(h)
        self.assertIn(h['id'], ('H-SERVO-1', 'H-DIST-1', 'H-SWEEP-1', 'H-DUTY-1', 'H-MICRO-1', 'H-PULSE-1', 'H-SHUTTLE-1', 'H-GLIDER-1'))
        # Verify motorglider tasks in endurance_suite
        tasks = wl._PHASE_TO_SG_TASK.get('endurance_suite', [])
        self.assertIn('motorglider_efficiency_wh_km', tasks)
        self.assertIn('sustainer_thrust_fraction', tasks)


class TestHGlider1WaveGenerator(unittest.TestCase):
    """Verifies kinematic waveform generation for motorglider sustainer loiter."""

    def test_wave_endurance_motorglider_loiter_structure(self):
        res = wl.wave_endurance_motorglider_loiter(
            t_sec=5.0,
            f_hz=2.2,
            flap_amp_deg=85.0,
            active_pair=4,
            glide_dihedral_deg=6.0,
            thermal_rotation_s=30.0,
            holding_current_limit_a=7.0,
            v_ms=10.5,
            auto_rotate=False
        )
        self.assertTrue(res['motorglider_active'])
        self.assertEqual(res['active_pair'], 4)
        self.assertEqual(res['glide_pairs_count'], 3)
        self.assertEqual(res['active_pairs_count'], 1)
        self.assertEqual(res['holding_current_limit_a'], 7.0)

        # Pair 4 must be SUSTAINER
        pair4 = res['pairs']['pair_4']
        self.assertEqual(pair4['mode'], 'SUSTAINER')
        self.assertTrue(pair4['active'])
        self.assertEqual(pair4['dihedral_cmd_deg'], 0.0)
        self.assertEqual(pair4['current_limit_a'], 35.0)

        # Pairs 1-3 must be GLIDE_LOCK
        for p in (1, 2, 3):
            pair_p = res['pairs'][f'pair_{p}']
            self.assertEqual(pair_p['mode'], 'GLIDE_LOCK')
            self.assertFalse(pair_p['active'])
            self.assertEqual(pair_p['flap_deg'], 0.0)
            self.assertEqual(pair_p['dihedral_cmd_deg'], 6.0)
            self.assertEqual(pair_p['current_limit_a'], 7.0)

        # Power & energy efficiency assertions
        self.assertEqual(res['total_power_w'], 125.0)
        self.assertLessEqual(res['wh_per_km'], 8.5)
        self.assertGreater(res['power_reduction_pct'], 55.0)

    def test_wave_endurance_motorglider_thermal_rotation(self):
        # With auto_rotate=True and period=30s:
        # t in [0, 30)   -> Pair 1
        # t in [30, 60)  -> Pair 2
        # t in [60, 90)  -> Pair 3
        # t in [90, 120) -> Pair 4
        res_0 = wl.wave_endurance_motorglider_loiter(t_sec=5.0, thermal_rotation_s=30.0, auto_rotate=True)
        self.assertEqual(res_0['active_pair'], 1)
        self.assertEqual(res_0['pairs']['pair_1']['mode'], 'SUSTAINER')
        self.assertEqual(res_0['pairs']['pair_2']['mode'], 'GLIDE_LOCK')

        res_35 = wl.wave_endurance_motorglider_loiter(t_sec=35.0, thermal_rotation_s=30.0, auto_rotate=True)
        self.assertEqual(res_35['active_pair'], 2)
        self.assertEqual(res_35['pairs']['pair_2']['mode'], 'SUSTAINER')
        self.assertEqual(res_35['pairs']['pair_1']['mode'], 'GLIDE_LOCK')

        res_65 = wl.wave_endurance_motorglider_loiter(t_sec=65.0, thermal_rotation_s=30.0, auto_rotate=True)
        self.assertEqual(res_65['active_pair'], 3)
        self.assertEqual(res_65['pairs']['pair_3']['mode'], 'SUSTAINER')

        res_95 = wl.wave_endurance_motorglider_loiter(t_sec=95.0, thermal_rotation_s=30.0, auto_rotate=True)
        self.assertEqual(res_95['active_pair'], 4)
        self.assertEqual(res_95['pairs']['pair_4']['mode'], 'SUSTAINER')


class TestHGlider1DiagnosticsAndStatus(unittest.TestCase):
    """Verifies ground station diagnostics output for H-GLIDER-1."""

    def test_advanced_physics_status_contains_h_glider_1(self):
        report = wl.advanced_physics_status()
        self.assertIn('H-GLIDER-1', report)
        self.assertIn('Tandem Metachronal Motorglider', report)

    def test_endurance_status_contains_h_glider_1(self):
        report = wl.endurance_status()
        self.assertIn('H-GLIDER-1', report)
        self.assertIn('Motorglider Sustainer', report)
        self.assertIn('Motorglider (10.5 m/s)', report)


if __name__ == '__main__':
    unittest.main()
