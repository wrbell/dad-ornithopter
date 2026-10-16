"""
Unit Tests: Bat Flight Biomechanics Suite (H-81 to H-85)
=========================================================
Tests coverage for:
  - H-81: Chiropteran Inertial Reorientation
  - H-82: Anhedral Gull-Wing Turn Trapping
  - H-83: Dynamic Inflow-Aligned LEV Trapping
  - H-84: Metachronal Cascade Roosting Somersault
  - H-85: Synthetic Somatosensory Stall Preemption
Tests verify:
  1. Hypothesis definitions and metadata in AERO_HYPOTHESES and _ADV_NEW_HYPS
  2. Scoring metrics in _SG_TASKS
  3. ALSS segments and curriculum enrolment in Curricula A & B
  4. Expanded discovery bounds in _EXPANDED_DISCOVERY_BOUNDS
  5. Context champions in _CMTS_CHAMPIONS
  6. AI observer trigger conditions in ai_observer_tick
  7. Gesture detection in GestureDetector
  8. Intent interpretation and behaviour dispatching in mco_intent_control
"""

import sys
import os
import unittest

# Ensure firmware directory is in python path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'firmware'))

import mco_wave_library as wl
import mco_intent_control as ic


class TestBatFlightSuiteRegistration(unittest.TestCase):
    """Test hypothesis registration and metadata for H-81 through H-85."""

    def test_hypotheses_registered_in_adv_new_hyps(self):
        adv_ids = {h['id'] for h in wl._ADV_NEW_HYPS}
        for hid in ('H-81', 'H-82', 'H-83', 'H-84', 'H-85'):
            self.assertIn(hid, adv_ids, f"{hid} missing from _ADV_NEW_HYPS")

    def test_hypotheses_registered_in_aero_hypotheses(self):
        aero_ids = {h['id'] for h in wl.AERO_HYPOTHESES}
        for hid in ('H-81', 'H-82', 'H-83', 'H-84', 'H-85'):
            self.assertIn(hid, aero_ids, f"{hid} missing from AERO_HYPOTHESES")

    def test_sg_tasks_metrics(self):
        expected_metrics = [
            'zero_speed_inertial_yaw_dps',
            'anhedral_turn_radius_m',
            'lev_lift_boost_ratio',
            'perch_flare_stop_time_s',
            'incipient_stall_recovery_ms',
        ]
        for metric in expected_metrics:
            self.assertIn(metric, wl._SG_TASKS, f"{metric} missing from _SG_TASKS")
            task = wl._SG_TASKS[metric]
            self.assertIn('axis', task)
            self.assertIn('sensor', task)

    def test_alss_curriculum_enrolment(self):
        self.assertIn('h84_roost_somersault_a', wl._ALSS_CURRICULUM['A'],
                      "h84_roost_somersault_a should be in Curriculum A (arrest/safety)")
        for seg in ('h81_inertial_yaw_b', 'h82_anhedral_turn_b', 'h83_lev_trapping_b', 'h85_stall_preempt_b'):
            self.assertIn(seg, wl._ALSS_CURRICULUM['B'],
                          f"{seg} should be in Curriculum B (agility/performance)")

    def test_discovery_bounds(self):
        expected_param_keys = [
            'h81_sweep_delta_deg', 'h81_zero_v_yaw_rate_dps',
            'h82_inboard_anhedral_deg', 'h82_outboard_dihedral_deg', 'h82_turn_radius_m',
            'h83_cyclic_pitch_bias_deg', 'h83_cl_max_boost',
            'h84_pitch_couple_nm', 'h84_flare_stop_time_s',
            'h85_jitter_threshold_a', 'h85_reflex_pitch_deg',
        ]
        for key in expected_param_keys:
            self.assertIn(key, wl._EXPANDED_DISCOVERY_BOUNDS,
                          f"{key} missing from _EXPANDED_DISCOVERY_BOUNDS")
            lo, hi = wl._EXPANDED_DISCOVERY_BOUNDS[key]
            self.assertLess(lo, hi)

    def test_cmts_champions(self):
        expected_champions = {
            'chiropteran_inertial_yaw': 'H-81',
            'anhedral_knife_edge_turn': 'H-82',
            'inflow_aligned_lev_trapping': 'H-83',
            'roosting_perch_somersault': 'H-84',
            'somatosensory_stall_preemption': 'H-85',
        }
        for champ_name, expected_hid in expected_champions.items():
            self.assertIn(champ_name, wl._CMTS_CHAMPIONS, f"{champ_name} missing from _CMTS_CHAMPIONS")
            self.assertEqual(wl._CMTS_CHAMPIONS[champ_name].get('hyp_id'), expected_hid)


class TestBatFlightObserverTriggers(unittest.TestCase):
    """Test AI observer triggers for queuing H-81 through H-85."""

    def setUp(self):
        wl._ALC_SHARED['priority_hyp_queue'].clear()
        wl._ALC_SHARED['alert_log'].clear()

    def test_observer_trigger_h81_inertial_yaw(self):
        sensors = {'V_ms': 1.5, 'yaw_stick': 0.65}
        obs = wl.ai_observer_tick(sensors, {})
        self.assertTrue(obs.get('h81_inertial_yaw_active'))
        queued_ids = [q['hyp_id'] for q in wl._ALC_SHARED['priority_hyp_queue']]
        self.assertIn('H-81', queued_ids)

    def test_observer_trigger_h82_anhedral_turn(self):
        sensors = {'V_ms': 8.0, 'roll_stick': 0.75}
        obs = wl.ai_observer_tick(sensors, {})
        self.assertTrue(obs.get('h82_anhedral_turn_active'))
        queued_ids = [q['hyp_id'] for q in wl._ALC_SHARED['priority_hyp_queue']]
        self.assertIn('H-82', queued_ids)

    def test_observer_trigger_h83_lev_trapping(self):
        sensors = {'V_ms': 8.0, 'pitch_stick': 0.70}
        obs = wl.ai_observer_tick(sensors, {})
        self.assertTrue(obs.get('h83_lev_trapping_active'))

    def test_observer_trigger_h84_roost_somersault(self):
        sensors = {'lever_pct': 10.0, 'pitch_stick': 0.85}
        obs = wl.ai_observer_tick(sensors, {})
        self.assertTrue(obs.get('h84_roost_perch_flare'))
        queued_ids = [q['hyp_id'] for q in wl._ALC_SHARED['priority_hyp_queue']]
        self.assertIn('H-84', queued_ids)

    def test_observer_trigger_h85_somatosensory_stall(self):
        sensors = {'motor_current_jitter_a': 1.6}
        obs = wl.ai_observer_tick(sensors, {})
        self.assertTrue(obs.get('h85_tactile_stall_preempt'))
        alert_texts = [a['text'] for a in wl._ALC_SHARED['alert_log']]
        self.assertTrue(any('H-85' in t for t in alert_texts))


class TestGestureAndIntentControl(unittest.TestCase):
    """Test gesture detection, intent taxonomy, and behaviour dispatching."""

    def setUp(self):
        self.interp = ic.IntentInterpreter()

    def test_inertial_yaw_intent(self):
        intent, conf, params = self.interp.interpret(
            {'yaw': 0.5, 'roll': 0.0, 'pitch': 0.0, 'throttle': 0.5},
            {},
            {'V_ms': 0.5, 'h_agl_m': 10.0, 'on_ground': False}
        )
        self.assertEqual(intent, ic.Intent.INERTIAL_YAW)
        self.assertEqual(params.get('direction'), 'right')
        self.assertAlmostEqual(params.get('yaw_rate_dps'), 30.0)

    def test_anhedral_turn_intent_in_manoeuvre_mode(self):
        intent, conf, params = self.interp.interpret(
            {'yaw': 0.0, 'roll': 0.9, 'pitch': 0.0, 'throttle': 0.5},
            {'mode_sw': 1},
            {'V_ms': 8.0, 'h_agl_m': 20.0, 'on_ground': False}
        )
        self.assertEqual(intent, ic.Intent.ANHEDRAL_TURN)
        self.assertEqual(params.get('direction'), 'right')

    def test_perch_flare_gesture(self):
        for _ in range(25):
            self.interp._gesture_det.update(0.0, 85.0, 6.0)
        intent, conf, params = self.interp.interpret(
            {'pitch': 0.95, 'throttle': 0.85},
            {},
            {'V_ms': 6.0, 'h_agl_m': 15.0, 'on_ground': False}
        )
        self.assertEqual(intent, ic.Intent.ROOST_PERCH_FLARE)
        self.assertGreaterEqual(conf, 0.8)

    def test_roost_somersault_altitude_clamping(self):
        for _ in range(25):
            self.interp._gesture_det.update(0.0, 50.0, 6.0)
        gesture, conf, params = wl.classify_pitch_intent(
            0.95, 50.0, 6.0, 3.0, 0.0, self.interp._gesture_det
        )
        self.assertEqual(gesture, 'perch_flare')
        self.assertEqual(params.get('flare_type'), 'envelope_clamped_perch')

    def test_behaviour_dispatcher_roost_perch(self):
        class MockCtrl:
            freq_hz = 1.0
            _tgt_freq = 1.0
            _tgt_sweeps = [60.0] * 8
            _tgt_dihedrals = [0.0] * 8
            _tgt_pitches = [0.0] * 8

        ctrl = MockCtrl()
        disp = ic.BehaviourDispatcher(ctrl)
        for _ in range(25):
            disp.interpreter._gesture_det.update(0.0, 85.0, 6.0)

        name, intent, status = disp.update(
            0.1,
            {'pitch': 0.95, 'throttle': 0.85},
            {},
            {'V_ms': 6.0, 'h_agl_m': 15.0, 'on_ground': False}
        )
        self.assertEqual(name, 'roost_perch_flare')
        self.assertEqual(intent, ic.Intent.ROOST_PERCH_FLARE)
        self.assertEqual(status, ic.RUNNING)
        self.assertEqual(ctrl._tgt_pitches[:4], [35.0, 35.0, 35.0, 35.0])
        self.assertEqual(ctrl._tgt_pitches[4:], [-35.0, -35.0, -35.0, -35.0])


if __name__ == '__main__':
    unittest.main()
