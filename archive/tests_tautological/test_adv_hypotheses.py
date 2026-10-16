"""
Unit & Integration Tests: Advanced High-Endurance Hypotheses Suite
(H-MICRO-1, H-PULSE-1, H-SHUTTLE-1)
==================================================================
Comprehensive verification suite covering:
  1. Hypothesis Registration & Metadata in AERO_HYPOTHESES, _ADV_NEW_HYPS, _hypothesis_queue, BIOMIMETIC_HYPOTHESES
  2. Configuration validation and parameter integrity
  3. Scoring Tasks in _SG_TASKS and Curriculum Segments in _ALSS_SEGMENTS / _ALSS_CURRICULUM
  4. CMTS Context Champions in _CMTS_CHAMPIONS (micro_stroke, pulse_bounding, bus_shuttle)
  5. Discovery Bounds in _DISCOVERY_BOUNDS
  6. Genome Update Routing in _update_genome_from_trial
  7. AI Observer Triggers 43-45 in ai_observer_tick
  8. Discovery Selection by focus keywords and sensor triggers
  9. Wave Kinematic Generators in mco_wave_library
  10. Diagnostics and status reporting in advanced_physics_status and endurance_status
"""

import sys
import os
import unittest
import math

# Ensure firmware directory is in python path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'firmware'))

import mco_wave_library as wl


class TestAdvancedHypothesesRegistration(unittest.TestCase):
    """Verifies H-MICRO-1, H-PULSE-1, H-SHUTTLE-1 registration across all catalogs."""

    TARGET_HIDS = ('H-MICRO-1', 'H-PULSE-1', 'H-SHUTTLE-1')

    def test_registered_in_adv_new_hyps(self):
        adv_ids = {h['id'] for h in wl._ADV_NEW_HYPS}
        for hid in self.TARGET_HIDS:
            self.assertIn(hid, adv_ids, f"{hid} missing from _ADV_NEW_HYPS")

    def test_registered_in_aero_hypotheses(self):
        aero_ids = {h['id'] for h in wl.AERO_HYPOTHESES}
        for hid in self.TARGET_HIDS:
            self.assertIn(hid, aero_ids, f"{hid} missing from AERO_HYPOTHESES")
            h = next(h for h in wl.AERO_HYPOTHESES if h['id'] == hid)
            self.assertEqual(h['target'], 'endurance')
            self.assertIn('cruise', h['eval_phases'])

    def test_registered_in_hypothesis_queue(self):
        q_ids = {h['id'] for h in wl._hypothesis_queue}
        for hid in self.TARGET_HIDS:
            self.assertIn(hid, q_ids, f"{hid} missing from _hypothesis_queue")

    def test_registered_in_biomimetic_catalog(self):
        for hid in self.TARGET_HIDS:
            self.assertIn(hid, wl.BIOMIMETIC_HYPOTHESES, f"{hid} missing from BIOMIMETIC_HYPOTHESES")


class TestAdvancedHypothesesConfigIntegrity(unittest.TestCase):
    """Validates configuration parameters and boundaries for the 3 hypotheses."""

    def test_h_micro_1_config(self):
        h = next(h for h in wl.AERO_HYPOTHESES if h['id'] == 'H-MICRO-1')
        cfg = h['config']
        self.assertTrue(cfg.get('_h_micro_1'))
        self.assertEqual(cfg.get('h_micro1_min_amp_deg'), 20.0)
        self.assertEqual(cfg.get('h_micro1_base_amp_deg'), 96.0)
        self.assertEqual(cfg.get('h_micro1_v_ref_ms'), 9.5)
        self.assertEqual(cfg.get('_scorer'), 'micro_stroke_efficiency_wh_km')
        self.assertGreaterEqual(h['v_min'], 9.5)

    def test_h_pulse_1_config(self):
        h = next(h for h in wl.AERO_HYPOTHESES if h['id'] == 'H-PULSE-1')
        cfg = h['config']
        self.assertTrue(cfg.get('_h_pulse_1'))
        self.assertEqual(cfg.get('h_pulse1_cycle_period_s'), 9.0)
        self.assertAlmostEqual(cfg.get('h_pulse1_flap_duty_frac'), 0.333, places=3)
        self.assertEqual(cfg.get('h_pulse1_glide_dihedral_deg'), 6.0)
        self.assertEqual(cfg.get('_scorer'), 'bounding_duty_ratio')

    def test_h_shuttle_1_config(self):
        h = next(h for h in wl.AERO_HYPOTHESES if h['id'] == 'H-SHUTTLE-1')
        cfg = h['config']
        self.assertTrue(cfg.get('_h_shuttle_1'))
        self.assertEqual(cfg.get('h_shuttle1_phase_stagger_deg'), 90.0)
        self.assertEqual(cfg.get('h_shuttle1_v_fence_min'), 21.8)
        self.assertEqual(cfg.get('h_shuttle1_v_fence_max'), 25.2)
        self.assertEqual(cfg.get('_scorer'), 'bus_shuttle_exchange_fraction')


class TestAdvancedScoringCurriculumAndCMTS(unittest.TestCase):
    """Verifies scoring tasks, ALSS segments, curriculum, and CMTS context champions."""

    def test_sg_tasks_metrics(self):
        expected_tasks = [
            ('micro_stroke_efficiency_wh_km', -1),
            ('bounding_duty_ratio', +1),
            ('bus_shuttle_exchange_fraction', +1),
        ]
        for task_name, direction in expected_tasks:
            self.assertIn(task_name, wl._SG_TASKS, f"{task_name} missing from _SG_TASKS")
            self.assertEqual(wl._SG_TASKS[task_name]['direction'], direction)

    def test_phase_to_sg_task_mapping(self):
        tasks = wl._PHASE_TO_SG_TASK.get('endurance_suite', [])
        self.assertIn('micro_stroke_efficiency_wh_km', tasks)
        self.assertIn('bounding_duty_ratio', tasks)
        self.assertIn('bus_shuttle_exchange_fraction', tasks)

    def test_alss_curriculum_segments(self):
        expected_segs = {
            'micro1_scaling_b': 'H-MICRO-1',
            'pulse1_bounding_b': 'H-PULSE-1',
            'shuttle1_exchange_b': 'H-SHUTTLE-1',
        }
        for seg_name, target_hid in expected_segs.items():
            self.assertIn(seg_name, wl._ALSS_SEGMENTS, f"{seg_name} missing from _ALSS_SEGMENTS")
            self.assertEqual(wl._ALSS_SEGMENTS[seg_name][1], target_hid)
            self.assertIn(seg_name, wl._ALSS_CURRICULUM.get('B', []), f"{seg_name} not in curriculum B")

    def test_cmts_champions(self):
        expected_champs = {
            'micro_stroke': 'H-MICRO-1',
            'pulse_bounding': 'H-PULSE-1',
            'bus_shuttle': 'H-SHUTTLE-1',
        }
        for ctx, hid in expected_champs.items():
            self.assertIn(ctx, wl._CMTS_CHAMPIONS, f"{ctx} missing from _CMTS_CHAMPIONS")
            self.assertEqual(wl._CMTS_CHAMPIONS[ctx]['hyp_id'], hid)

    def test_discovery_bounds(self):
        expected_bounds = [
            'h_micro1_min_amp_deg',
            'h_micro1_base_amp_deg',
            'h_micro1_v_ref_ms',
            'h_pulse1_cycle_period_s',
            'h_pulse1_flap_duty_frac',
            'h_pulse1_glide_dihedral_deg',
            'h_shuttle1_phase_stagger_deg',
            'h_shuttle1_v_fence_min',
            'h_shuttle1_v_fence_max',
        ]
        for param in expected_bounds:
            self.assertIn(param, wl._EXPANDED_DISCOVERY_BOUNDS, f"{param} missing from _EXPANDED_DISCOVERY_BOUNDS")
            low, high = wl._EXPANDED_DISCOVERY_BOUNDS[param]
            self.assertLess(low, high)


class TestAdvancedDiscoveryAndRouting(unittest.TestCase):
    """Verifies keyword focus routing, sensor triggers, and genome evolution routing."""

    def setUp(self):
        import copy
        self._orig_endurance = dict(wl._MW_ENDURANCE)
        self._orig_glide = dict(wl._MW_GLIDE)
        self._orig_resilience = dict(wl._MW_RESILIENCE)
        self._orig_perf = dict(wl._MW_PERFORMANCE)
        self._orig_lever_weights = copy.deepcopy(wl._lever_mode_weights)

    def tearDown(self):
        import copy
        wl._MW_ENDURANCE.clear()
        wl._MW_ENDURANCE.update(self._orig_endurance)
        wl._MW_GLIDE.clear()
        wl._MW_GLIDE.update(self._orig_glide)
        wl._MW_RESILIENCE.clear()
        wl._MW_RESILIENCE.update(self._orig_resilience)
        wl._MW_PERFORMANCE.clear()
        wl._MW_PERFORMANCE.update(self._orig_perf)
        wl._lever_mode_weights.clear()
        wl._lever_mode_weights.update(copy.deepcopy(self._orig_lever_weights))

    def test_discovery_by_focus_keywords(self):
        test_cases = [
            ('h_micro_1', 'H-MICRO-1'),
            ('micro_stroke', 'H-MICRO-1'),
            ('airspeed_scaling', 'H-MICRO-1'),
            ('amplitude_derate', 'H-MICRO-1'),
            ('h_pulse_1', 'H-PULSE-1'),
            ('pulse_bounding', 'H-PULSE-1'),
            ('bird_bounding', 'H-PULSE-1'),
            ('pulse_flapping', 'H-PULSE-1'),
            ('intermittent_glide', 'H-PULSE-1'),
            ('h_shuttle_1', 'H-SHUTTLE-1'),
            ('bus_shuttle', 'H-SHUTTLE-1'),
            ('energy_shuttling', 'H-SHUTTLE-1'),
            ('rail_exchange', 'H-SHUTTLE-1'),
        ]
        for kw, expected_hid in test_cases:
            h = wl.discovery_hypothesis_select(focus=kw)
            self.assertIsNotNone(h, f"Failed to select hypothesis for focus='{kw}'")
            self.assertEqual(h['id'], expected_hid)

    def test_discovery_by_sensor_triggers(self):
        cases = [
            ({'h_micro_1_active': True}, 'H-MICRO-1'),
            ({'micro_stroke_active': True}, 'H-MICRO-1'),
            ({'airspeed_derate_active': True, 'V_ms': 10.5}, 'H-MICRO-1'),
            ({'h_pulse_1_active': True}, 'H-PULSE-1'),
            ({'bounding_glide_active': True}, 'H-PULSE-1'),
            ({'pulse_flapping_active': True}, 'H-PULSE-1'),
            ({'pulse_bounding_active': True}, 'H-PULSE-1'),
            ({'h_shuttle_1_active': True}, 'H-SHUTTLE-1'),
            ({'bus_shuttle_active': True}, 'H-SHUTTLE-1'),
            ({'energy_shuttling_active': True}, 'H-SHUTTLE-1'),
        ]
        for s_dict, expected_hid in cases:
            h = wl.discovery_hypothesis_select(sensors=s_dict)
            self.assertIsNotNone(h, f"Failed to select hypothesis for sensors={s_dict}")
            self.assertEqual(h['id'], expected_hid)

    def test_genome_routing(self):
        sensors = {'V_ms': 12.0, 'h_agl_m': 30.0}

        # H-MICRO-1 -> ENDURANCE (sustained_wh_per_km), PERFORMANCE (top_speed, efficiency)
        orig_wh = wl._MW_ENDURANCE.get('sustained_wh_per_km', 0.0)
        wl._update_genome_from_trial('H-MICRO-1', 0.20, sensors)
        self.assertGreater(wl._MW_ENDURANCE.get('sustained_wh_per_km', 0.0), orig_wh)

        # H-PULSE-1 -> ENDURANCE (sustained_wh_per_km, thermal_budget), GLIDE (min_sink_aoa)
        orig_glide = wl._MW_GLIDE.get('min_sink_aoa', 0.0)
        wl._update_genome_from_trial('H-PULSE-1', 0.20, sensors)
        self.assertGreater(wl._MW_GLIDE.get('min_sink_aoa', 0.0), orig_glide)

        # H-SHUTTLE-1 -> ENDURANCE (sustained_wh_per_km, current_variance), RESILIENCE (single_pair_loss)
        orig_resilience = wl._MW_RESILIENCE.get('single_pair_loss', 0.0)
        wl._update_genome_from_trial('H-SHUTTLE-1', 0.20, sensors)
        self.assertGreater(wl._MW_RESILIENCE.get('single_pair_loss', 0.0), orig_resilience)


class TestAIObserverTriggers(unittest.TestCase):
    """Verifies AI Observer triggers 43-45 for H-MICRO-1, H-PULSE-1, H-SHUTTLE-1."""

    def setUp(self):
        wl._ALC_SHARED['priority_hyp_queue'].clear()

    def test_ai_observer_h_micro_1_trigger(self):
        sensors = {'V_ms': 11.0, 'lever_pct': 60.0}
        obs = wl.ai_observer_tick(sensors, {})
        self.assertTrue(obs.get('h_micro1_active'))
        queue_hids = [item['hyp_id'] for item in wl._ALC_SHARED['priority_hyp_queue']]
        self.assertIn('H-MICRO-1', queue_hids)

    def test_ai_observer_h_pulse_1_trigger(self):
        sensors = {'V_ms': 12.0, 'lever_pct': 55.0, 'motor_temps': [{'motor_C': 55.0}]}
        obs = wl.ai_observer_tick(sensors, {})
        self.assertTrue(obs.get('h_pulse1_active'))
        queue_hids = [item['hyp_id'] for item in wl._ALC_SHARED['priority_hyp_queue']]
        self.assertIn('H-PULSE-1', queue_hids)

    def test_ai_observer_h_shuttle_1_trigger(self):
        sensors = {'bus_v': 24.0, 'bus_voltage_ripple_v': 1.4, 'lever_pct': 50.0}
        obs = wl.ai_observer_tick(sensors, {})
        self.assertTrue(obs.get('h_shuttle1_active'))
        queue_hids = [item['hyp_id'] for item in wl._ALC_SHARED['priority_hyp_queue']]
        self.assertIn('H-SHUTTLE-1', queue_hids)


class TestWaveKinematicGenerators(unittest.TestCase):
    """Verifies kinematic generators for micro stroke, pulse bounding, and bus shuttle."""

    def test_wave_endurance_micro_stroke(self):
        # Above reference speed (14.5 m/s vs 9.5 m/s): derating should be active
        res_fast = wl.wave_endurance_micro_stroke(t_sec=0.1, v_ms=14.5, f_hz=2.0)
        self.assertTrue(res_fast['derating_active'])
        self.assertLess(res_fast['amp_cmd_deg'], 96.0)
        self.assertGreaterEqual(res_fast['amp_cmd_deg'], 24.0)
        self.assertGreater(res_fast['inertial_reduction_pct'], 0.0)

        # Below reference speed (8.0 m/s < 9.5 m/s): stall floor snapback to full amplitude
        res_slow = wl.wave_endurance_micro_stroke(t_sec=0.1, v_ms=8.0, f_hz=2.0)
        self.assertFalse(res_slow['derating_active'])
        self.assertEqual(res_slow['amp_cmd_deg'], 96.0)
        self.assertEqual(res_slow['inertial_reduction_pct'], 0.0)

    def test_wave_endurance_pulse_bounding(self):
        # Flapping phase (t=1.0s, duty=0.333 of 9.0s -> flapping for first 3.0s)
        res_flap = wl.wave_endurance_pulse_bounding(t_sec=1.0, f_hz=2.0, cycle_period_s=9.0, flap_duty_frac=0.333)
        self.assertTrue(res_flap['is_flapping'])
        self.assertEqual(res_flap['mode'], 'FLAPPING')
        self.assertEqual(res_flap['dihedral_cmd_deg'], 0.0)

        # Glide lock phase (t=5.0s, in the 3.0s - 9.0s window)
        res_glide = wl.wave_endurance_pulse_bounding(t_sec=5.0, f_hz=2.0, cycle_period_s=9.0, flap_duty_frac=0.333)
        self.assertFalse(res_glide['is_flapping'])
        self.assertEqual(res_glide['mode'], 'GLIDE_LOCK')
        self.assertEqual(res_glide['flap_deg'], 0.0)
        self.assertEqual(res_glide['dihedral_cmd_deg'], 6.0)
        self.assertLess(res_glide['estimated_avg_power_w'], 150.0)

    def test_wave_endurance_bus_shuttle(self):
        # 90 deg stagger with nominal bus 24.0V
        res = wl.wave_endurance_bus_shuttle(t_sec=0.125, f_hz=2.0, phase_stagger_deg=90.0, bus_v=24.0)
        self.assertIn('flap_lead_deg', res)
        self.assertIn('flap_lag_deg', res)
        self.assertTrue(res['bus_safe'])
        self.assertGreater(res['ripple_reduction_pct'], 50.0)


class TestDiagnosticsAndStatus(unittest.TestCase):
    """Verifies that ground station and physics status include the new hypotheses."""

    def test_advanced_physics_status_contains_hypotheses(self):
        report = wl.advanced_physics_status()
        self.assertIn('H-MICRO-1', report)
        self.assertIn('H-PULSE-1', report)
        self.assertIn('H-SHUTTLE-1', report)

    def test_endurance_status_contains_hypotheses(self):
        report = wl.endurance_status()
        self.assertIn('H-MICRO-1', report)
        self.assertIn('H-PULSE-1', report)
        self.assertIn('H-SHUTTLE-1', report)


if __name__ == '__main__':
    unittest.main()
