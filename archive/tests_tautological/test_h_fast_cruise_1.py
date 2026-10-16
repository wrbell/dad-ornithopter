#!/usr/bin/env python3
"""
Unit & Integration Tests: Hypothesis H-FAST-CRUISE-1
Station-Wise Feathered 4-Wing Fast Cruise & Reflex Decambering
================================================================================
Comprehensive verification suite covering all 5 ALC modalities:
  1. Hypothesis Registration & Metadata in AERO_HYPOTHESES, _ADV_NEW_HYPS,
     _hypothesis_queue, and BIOMIMETIC_HYPOTHESES
  2. Configuration validation, Red Team physics safeguards & parameter integrity
     (Stations 1 & 4 feathered horizontal at 0° AoA / -4° reflex pitch,
      Stations 2 & 3 active propulsive micro-stroke core St=0.25,
      holding current limit <= 8.0A, 80ms stick/airspeed/ground abort)
  3. Scoring Tasks in _SG_TASKS, Curriculum in _ALSS_SEGMENTS / _ALSS_CURRICULUM,
     and CMTS Context Champions in _CMTS_CHAMPIONS ('feathered_fast_cruise')
  4. Expanded Discovery Bounds in _EXPANDED_DISCOVERY_BOUNDS
  5. Genome Update Routing in _update_genome_from_trial
  6. Wave Function Lever & Throttle Mapping in _HYP_TO_LEVER_ZONE and _lever_mode_weights
  7. AI Observer Trigger 48 in ai_observer_tick with Red Team abort fences
  8. Discovery Selection by focus keywords and sensor triggers
  9. Kinematic Wave Generator wave_fast_cruise_feathered and evaluator hook
  10. Diagnostics and status reporting in fast_cruise_status,
      advanced_physics_status, and efficiency_architecture_status
"""

import sys
import os
import unittest
import math

# Ensure firmware directory is in python path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'firmware'))

import mco_wave_library as wl


class TestHFastCruise1Registration(unittest.TestCase):
    """Verifies H-FAST-CRUISE-1 registration across all catalogs and tables."""

    TARGET_HID = 'H-FAST-CRUISE-1'

    def test_registered_in_adv_new_hyps(self):
        adv_ids = {h['id'] for h in wl._ADV_NEW_HYPS}
        self.assertIn(self.TARGET_HID, adv_ids, f"{self.TARGET_HID} missing from _ADV_NEW_HYPS")

    def test_registered_in_aero_hypotheses(self):
        aero_ids = {h['id'] for h in wl.AERO_HYPOTHESES}
        self.assertIn(self.TARGET_HID, aero_ids, f"{self.TARGET_HID} missing from AERO_HYPOTHESES")
        h = next(h for h in wl.AERO_HYPOTHESES if h['id'] == self.TARGET_HID)
        self.assertEqual(h['target'], 'sprint')
        self.assertIn('cruise', h['eval_phases'])
        self.assertIn('sprint', h['eval_phases'])
        self.assertIn('endurance', h['eval_phases'])
        self.assertEqual(h['priority'], 1)
        self.assertEqual(h['v_min'], 13.5)
        self.assertEqual(h['v_max'], 26.0)

    def test_registered_in_hypothesis_queue(self):
        q_ids = {h['id'] for h in wl._hypothesis_queue}
        self.assertIn(self.TARGET_HID, q_ids, f"{self.TARGET_HID} missing from _hypothesis_queue")

    def test_registered_in_biomimetic_catalog(self):
        self.assertIn(self.TARGET_HID, wl.BIOMIMETIC_HYPOTHESES, f"{self.TARGET_HID} missing from BIOMIMETIC_HYPOTHESES")


class TestHFastCruise1ConfigIntegrity(unittest.TestCase):
    """Validates configuration parameters and boundaries for H-FAST-CRUISE-1."""

    def setUp(self):
        self.h = next(h for h in wl.AERO_HYPOTHESES if h['id'] == 'H-FAST-CRUISE-1')
        self.cfg = self.h['config']

    def test_h_fast_cruise_1_config_flag_and_pairs(self):
        self.assertTrue(self.cfg.get('_h_fast_cruise_1'))
        self.assertEqual(self.cfg.get('h_fc1_parked_pairs'), [1, 4])
        self.assertEqual(self.cfg.get('h_fc1_active_pairs'), [2, 3])

    def test_aerodynamic_and_kinematic_bounds(self):
        self.assertEqual(self.cfg.get('h_fc1_reflex_pitch_deg'), -4.0)
        self.assertEqual(self.cfg.get('h_fc1_min_amp_deg'), 25.0)
        self.assertEqual(self.cfg.get('h_fc1_max_amp_deg'), 32.0)
        self.assertEqual(self.cfg.get('h_fc1_target_strouhal'), 0.25)
        self.assertEqual(self.cfg.get('freq_hz'), 2.1)
        self.assertEqual(self.cfg.get('aoa_deg'), 0.0)

    def test_abort_and_safety_parameters(self):
        self.assertEqual(self.cfg.get('h_fc1_stall_abort_ms'), 12.0)
        self.assertEqual(self.cfg.get('h_fc1_pilot_stick_abort'), 0.35)
        self.assertEqual(self.cfg.get('_scorer'), 'feathered_fast_cruise_wh_km')


class TestHFastCruise1ScoringCurriculumAndCMTS(unittest.TestCase):
    """Verifies scoring tasks, ALSS segments, curriculum, and CMTS context champions."""

    def test_sg_task_metric(self):
        self.assertIn('feathered_fast_cruise_wh_km', wl._SG_TASKS)
        task = wl._SG_TASKS['feathered_fast_cruise_wh_km']
        self.assertEqual(task['direction'], -1)
        self.assertEqual(task['axis'], 'endurance')
        self.assertEqual(task['sensor'], 'wh_per_km')

    def test_alss_curriculum_segment(self):
        self.assertIn('fc1_feathered_cruise_b', wl._ALSS_SEGMENTS)
        seg = wl._ALSS_SEGMENTS['fc1_feathered_cruise_b']
        self.assertEqual(seg[0], 'cruise')
        self.assertEqual(seg[1], 'H-FAST-CRUISE-1')
        self.assertEqual(seg[2], 20.0)  # min duration
        self.assertEqual(seg[3], 120.0) # max duration
        self.assertEqual(seg[4], 16.0)  # target airspeed V_ms
        self.assertIn('fc1_feathered_cruise_b', wl._ALSS_CURRICULUM.get('B', []))

    def test_phase_to_sg_task_mapping(self):
        self.assertIn('feathered_fast_cruise', wl._PHASE_TO_SG_TASK)
        tasks = wl._PHASE_TO_SG_TASK['feathered_fast_cruise']
        self.assertIn('feathered_fast_cruise_wh_km', tasks)
        self.assertIn('dash_speed_ms', tasks)
        self.assertIn('strouhal_optimal', tasks)
        self.assertIn('parasite_drag_reduction_pct', tasks)

    def test_cmts_champions(self):
        self.assertIn('feathered_fast_cruise', wl._CMTS_CHAMPIONS)
        champ = wl._CMTS_CHAMPIONS['feathered_fast_cruise']
        self.assertEqual(champ['hyp_id'], 'H-FAST-CRUISE-1')
        self.assertAlmostEqual(champ['wh_per_km'], 2.15, places=2)
        self.assertIn('config', champ)


class TestHFastCruise1DiscoveryBoundsAndGenome(unittest.TestCase):
    """Verifies discovery bounds and genome trial update routing."""

    def setUp(self):
        import copy
        self._orig_endurance = dict(wl._MW_ENDURANCE)
        self._orig_perf = dict(wl._MW_PERFORMANCE)
        self._orig_pitch = dict(wl._MW_PITCH)
        self._orig_lever_weights = copy.deepcopy(wl._lever_mode_weights)

    def tearDown(self):
        import copy
        wl._MW_ENDURANCE.clear()
        wl._MW_ENDURANCE.update(self._orig_endurance)
        wl._MW_PERFORMANCE.clear()
        wl._MW_PERFORMANCE.update(self._orig_perf)
        wl._MW_PITCH.clear()
        wl._MW_PITCH.update(self._orig_pitch)
        wl._lever_mode_weights.clear()
        wl._lever_mode_weights.update(copy.deepcopy(self._orig_lever_weights))

    def test_discovery_bounds(self):
        expected_bounds = [
            'h_fc1_flap_amp_deg',
            'h_fc1_reflex_pitch_deg',
            'h_fc1_freq_hz',
            'h_fc1_v_min_abort_ms',
            'h_fc1_holding_current_a',
        ]
        for param in expected_bounds:
            self.assertIn(param, wl._EXPANDED_DISCOVERY_BOUNDS, f"{param} missing from _EXPANDED_DISCOVERY_BOUNDS")
            b_min, b_max = wl._EXPANDED_DISCOVERY_BOUNDS[param]
            self.assertLess(b_min, b_max)

    def test_genome_update_routing(self):
        sensors = {'V_ms': 16.0, 'wh_per_km': 1.89, 'strouhal': 0.25}
        initial_endurance = wl._MW_ENDURANCE.get('sustained_wh_per_km', 0.0)
        initial_top_speed = wl._MW_PERFORMANCE.get('top_speed', 0.0)
        initial_eff = wl._MW_PERFORMANCE.get('cruise_efficiency', 0.0)
        initial_pitch = wl._MW_PITCH.get('pitch_aoa', 0.0)

        wl._update_genome_from_trial('H-FAST-CRUISE-1', 0.25, sensors)

        self.assertGreater(wl._MW_ENDURANCE['sustained_wh_per_km'], initial_endurance)
        self.assertGreater(wl._MW_PERFORMANCE['top_speed'], initial_top_speed)
        self.assertGreater(wl._MW_PERFORMANCE['cruise_efficiency'], initial_eff)
        self.assertGreater(wl._MW_PITCH['pitch_aoa'], initial_pitch)


class TestHFastCruise1WaveLeverIntegration(unittest.TestCase):
    """Verifies H-FAST-CRUISE-1 mapping in Wave Function Lever and throttle zones."""

    def test_zone_mapping(self):
        self.assertIn('H-FAST-CRUISE-1', wl._HYP_TO_LEVER_ZONE)
        self.assertEqual(wl._HYP_TO_LEVER_ZONE['H-FAST-CRUISE-1'], 'MAX_THRUST')

    def test_zone_weight_champion(self):
        self.assertIn('MAX_THRUST', wl._lever_mode_weights)
        weights = wl._lever_mode_weights['MAX_THRUST']
        self.assertIn('H-FAST-CRUISE-1', weights)
        self.assertGreaterEqual(weights['H-FAST-CRUISE-1'], 1.5)


class TestHFastCruise1AIObserverAndSelection(unittest.TestCase):
    """Verifies AI Observer Trigger 48 and Discovery Selection."""

    def setUp(self):
        wl._ALC_SHARED['priority_hyp_queue'].clear()

    def test_ai_observer_trigger_48_active_cruise(self):
        """Trigger 48 fires when airspeed >= 14 m/s and lever >= 70% in steady flight."""
        sensors = {
            'V_ms': 15.5,
            'lever_pct': 75.0,
            'pitch_stick': 0.05,
            'roll_stick': 0.05,
            'yaw_stick': 0.02,
            'on_ground': False,
        }
        obs = wl.ai_observer_tick(sensors, {})
        self.assertTrue(obs.get('h_fast_cruise1_active'))
        queue_hids = [item['hyp_id'] for item in wl._ALC_SHARED['priority_hyp_queue']]
        self.assertIn('H-FAST-CRUISE-1', queue_hids)

    def test_ai_observer_trigger_48_fast_cruise_flag(self):
        """Trigger 48 fires when fast_cruise_mode flag is set."""
        sensors = {
            'V_ms': 14.5,
            'lever_pct': 50.0,
            'fast_cruise_mode': True,
            'on_ground': False,
        }
        obs = wl.ai_observer_tick(sensors, {})
        self.assertTrue(obs.get('h_fast_cruise1_active'))

    def test_ai_observer_trigger_48_abort_on_pitch_stick(self):
        """Trigger 48 aborts when pitch stick > 0.35."""
        sensors = {
            'V_ms': 16.0,
            'lever_pct': 80.0,
            'pitch_stick': 0.40,
            'on_ground': False,
        }
        obs = wl.ai_observer_tick(sensors, {})
        self.assertFalse(obs.get('h_fast_cruise1_active', False))
        queue_hids = [item['hyp_id'] for item in wl._ALC_SHARED['priority_hyp_queue']]
        self.assertNotIn('H-FAST-CRUISE-1', queue_hids)

    def test_ai_observer_trigger_48_abort_on_roll_stick(self):
        """Trigger 48 aborts when roll stick > 0.35."""
        sensors = {
            'V_ms': 16.0,
            'lever_pct': 80.0,
            'roll_stick': -0.38,
            'on_ground': False,
        }
        obs = wl.ai_observer_tick(sensors, {})
        self.assertFalse(obs.get('h_fast_cruise1_active', False))

    def test_ai_observer_trigger_48_abort_on_yaw_stick(self):
        """Trigger 48 aborts when yaw stick > 0.15."""
        sensors = {
            'V_ms': 16.0,
            'lever_pct': 80.0,
            'yaw_stick': 0.20,
            'on_ground': False,
        }
        obs = wl.ai_observer_tick(sensors, {})
        self.assertFalse(obs.get('h_fast_cruise1_active', False))

    def test_ai_observer_trigger_48_abort_on_low_speed(self):
        """Trigger 48 aborts when airspeed drops below 12.0 m/s."""
        sensors = {
            'V_ms': 11.5,
            'lever_pct': 80.0,
            'fast_cruise_mode': True,
            'on_ground': False,
        }
        obs = wl.ai_observer_tick(sensors, {})
        self.assertFalse(obs.get('h_fast_cruise1_active', False))

    def test_ai_observer_trigger_48_abort_on_ground(self):
        """Trigger 48 aborts immediately upon ground contact."""
        sensors = {
            'V_ms': 16.0,
            'lever_pct': 80.0,
            'on_ground': True,
        }
        obs = wl.ai_observer_tick(sensors, {})
        self.assertFalse(obs.get('h_fast_cruise1_active', False))

    def test_discovery_select_by_focus_keywords(self):
        focus_keywords = ['h_fast_cruise_1', 'feathered_fast_cruise', 'reflex_decambering', 'feathered_cruise', 'station_wise_feathered']
        for kw in focus_keywords:
            h = wl.discovery_hypothesis_select(focus=kw)
            self.assertIsNotNone(h, f"Focus '{kw}' returned None")
            self.assertEqual(h['id'], 'H-FAST-CRUISE-1', f"Focus '{kw}' did not return H-FAST-CRUISE-1")

    def test_discovery_select_by_sensor_triggers(self):
        sensor_cases = [
            {'h_fast_cruise1_active': True},
            {'h_fast_cruise_1_active': True},
            {'feathered_fast_cruise_active': True},
        ]
        for s in sensor_cases:
            h = wl.discovery_hypothesis_select(sensors=s)
            self.assertIsNotNone(h, f"Sensors '{s}' returned None")
            self.assertEqual(h['id'], 'H-FAST-CRUISE-1', f"Sensors '{s}' did not return H-FAST-CRUISE-1")


class TestHFastCruise1WaveGenerator(unittest.TestCase):
    """Verifies kinematic waveform generation for feathered fast cruise."""

    def test_wave_fast_cruise_feathered_structure(self):
        res = wl.wave_fast_cruise_feathered(
            t_sec=2.5,
            f_hz=2.1,
            flap_amp_deg=28.0,
            reflex_pitch_deg=-4.0,
            holding_current_a=8.0,
            v_ms=16.0
        )
        self.assertTrue(res['fast_cruise_active'])
        self.assertFalse(res['aborted'])
        self.assertEqual(res['reflex_pitch_deg'], -4.0)
        self.assertEqual(res['holding_current_limit_a'], 8.0)
        self.assertEqual(res['v_ms'], 16.0)
        self.assertEqual(res['total_power_w'], 109.0)
        self.assertAlmostEqual(res['wh_per_km'], 1.89, places=2)

        # Strouhal number check in propulsive window [0.20, 0.30]
        self.assertGreaterEqual(res['strouhal'], 0.20)
        self.assertLessEqual(res['strouhal'], 0.30)

        pairs = res['pairs']
        # Stations 1 & 4 must be FEATHERED_REFLEX
        for p in (1, 4):
            pair_p = pairs[f'pair_{p}']
            self.assertEqual(pair_p['mode'], 'FEATHERED_REFLEX')
            self.assertFalse(pair_p['active_flapping'])
            self.assertEqual(pair_p['flap_deg'], 0.0)
            self.assertEqual(pair_p['aoa_cmd_deg'], 0.0)
            self.assertEqual(pair_p['reflex_pitch_deg'], -4.0)
            self.assertEqual(pair_p['current_limit_a'], 8.0)

        # Stations 2 & 3 must be PROPULSIVE_CORE
        for p in (2, 3):
            pair_p = pairs[f'pair_{p}']
            self.assertEqual(pair_p['mode'], 'PROPULSIVE_CORE')
            self.assertTrue(pair_p['active_flapping'])
            self.assertEqual(pair_p['aoa_cmd_deg'], 0.0)
            self.assertEqual(pair_p['reflex_pitch_deg'], 0.0)
            self.assertEqual(pair_p['current_limit_a'], 35.0)

    def test_wave_generator_abort_cases(self):
        # Abort on pitch stick
        res_pitch = wl.wave_fast_cruise_feathered(t_sec=1.0, pitch_stick=0.40)
        self.assertFalse(res_pitch['fast_cruise_active'])
        self.assertTrue(res_pitch['aborted'])
        self.assertEqual(res_pitch['pairs']['pair_2']['mode'], 'ABORT_STANDBY')
        self.assertEqual(res_pitch['pairs']['pair_2']['flap_deg'], 0.0)

        # Abort on roll stick
        res_roll = wl.wave_fast_cruise_feathered(t_sec=1.0, roll_stick=-0.45)
        self.assertTrue(res_roll['aborted'])

        # Abort on yaw stick
        res_yaw = wl.wave_fast_cruise_feathered(t_sec=1.0, yaw_stick=0.20)
        self.assertTrue(res_yaw['aborted'])

        # Abort on low airspeed
        res_speed = wl.wave_fast_cruise_feathered(t_sec=1.0, v_ms=11.0)
        self.assertTrue(res_speed['aborted'])

        # Abort on ground contact
        res_ground = wl.wave_fast_cruise_feathered(t_sec=1.0, on_ground=True)
        self.assertTrue(res_ground['aborted'])

    def test_wave_generator_alias_equivalence(self):
        res1 = wl.wave_fast_cruise_feathered(t_sec=3.5, v_ms=15.0)
        res2 = wl.wave_station_wise_feathered_fast_cruise(t_sec=3.5, v_ms=15.0)
        self.assertEqual(res1, res2)

    def test_evaluator_hook(self):
        sensors = {'V_ms': 16.0, 't_sec': 5.0}
        eval_res = wl.evaluate_fast_cruise_feathered(sensors=sensors)
        self.assertEqual(eval_res['hyp_id'], 'H-FAST-CRUISE-1')
        self.assertAlmostEqual(eval_res['wh_per_km'], 1.89, places=2)
        self.assertFalse(eval_res['aborted'])
        self.assertTrue(eval_res['safeguards_verified'])


class TestHFastCruise1DiagnosticsAndStatus(unittest.TestCase):
    """Verifies ground station diagnostics output for H-FAST-CRUISE-1."""

    def test_fast_cruise_status_allowed_and_callable(self):
        self.assertIn('fast_cruise_status', wl._GS_ALLOWED_CMDS)
        report = wl.fast_cruise_status()
        self.assertIn('H-FAST-CRUISE-1', report)
        self.assertIn('STATION-WISE FEATHERED FAST CRUISE', report)
        self.assertIn('Stations 1 & 4 (Outboard Boundary): FEATHERED STREAMLINE', report)
        self.assertIn('Stations 2 & 3 (Inboard Core):     PROPULSIVE CORE FLAP', report)
        self.assertIn('-4.0°', report)
        self.assertIn('2.15 Wh/km', report)

    def test_advanced_physics_status_contains_h_fast_cruise_1(self):
        report = wl.advanced_physics_status()
        self.assertIn('H-FAST-CRUISE-1', report)
        self.assertIn('Feathered 4-Wing Fast Cruise', report)

    def test_efficiency_architecture_status_contains_h_fast_cruise_1(self):
        report = wl.efficiency_architecture_status()
        self.assertIn('H-FAST-CRUISE-1', report)
        self.assertIn('Station-Wise Feathered 4-Wing Fast Cruise', report)


if __name__ == '__main__':
    unittest.main()
