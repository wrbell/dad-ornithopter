"""
Unit & Integration Tests: Endurance Hypotheses Suite (H-SERVO-1, H-DIST-1, H-SWEEP-1, H-DUTY-1)
================================================================================================
Comprehensive verification suite covering:
  1. Hypothesis Registration & Metadata in AERO_HYPOTHESES, _ADV_NEW_HYPS, and _hypothesis_queue
  2. Scoring Tasks in _SG_TASKS and Curriculum in _ALSS_SEGMENTS / _ALSS_CURRICULUM
  3. CMTS Context Champions in _CMTS_CHAMPIONS (servo_power, load_profile, sweep_inertia, duty_asymmetry)
  4. Genome Update Routing in _update_genome_from_trial
  5. Discovery Selection by focus keywords and sensor triggers
  6. AI Observer Triggers 39-42 in ai_observer_tick
  7. Wave Kinematic Generators in mco_wave_library
  8. Pilot Safety Clamps & Governors in mco_flight_governor:
     - ServoPowerTracker 50 Hz rolling telemetry and software fallback
     - compute_h_dist_pitch_compensation aft incidence counter-pitch
     - compute_h_duty_asymmetric_profile 52%-57% clamp and >2.5 Hz derating
     - compute_h_sweep_upstroke_trajectory <= 12.0° hardware clamp
  9. Performance Model System Power Accounting and evaluate_endurance_suite flight time extension
  10. Ground Station endurance_status command output
"""

import sys
import os
import unittest
import math

# Ensure firmware directory is in python path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'firmware'))

import mco_wave_library as wl
from mco_flight_governor import (
    ServoPowerTracker,
    _servo_tick,
    compute_h_dist_pitch_compensation,
    compute_h_duty_asymmetric_profile,
    compute_h_sweep_upstroke_trajectory,
)
from mco_performance_model import PerformanceModel


class TestEnduranceHypothesisRegistration(unittest.TestCase):
    """Verifies H-SERVO-1, H-DIST-1, H-SWEEP-1, H-DUTY-1 registration and catalog metadata."""

    def test_registered_in_adv_new_hyps(self):
        adv_ids = {h['id'] for h in wl._ADV_NEW_HYPS}
        for hid in ('H-SERVO-1', 'H-DIST-1', 'H-SWEEP-1', 'H-DUTY-1'):
            self.assertIn(hid, adv_ids, f"{hid} missing from _ADV_NEW_HYPS")

    def test_registered_in_aero_hypotheses(self):
        aero_ids = {h['id'] for h in wl.AERO_HYPOTHESES}
        for hid in ('H-SERVO-1', 'H-DIST-1', 'H-SWEEP-1', 'H-DUTY-1'):
            self.assertIn(hid, aero_ids, f"{hid} missing from AERO_HYPOTHESES")
            h = next(h for h in wl.AERO_HYPOTHESES if h['id'] == hid)
            self.assertEqual(h['target'], 'endurance')
            self.assertIn('cruise', h['eval_phases'])

    def test_registered_in_hypothesis_queue(self):
        q_ids = {h['id'] for h in wl._hypothesis_queue}
        for hid in ('H-SERVO-1', 'H-DIST-1', 'H-SWEEP-1', 'H-DUTY-1'):
            self.assertIn(hid, q_ids, f"{hid} missing from _hypothesis_queue")

    def test_registered_in_biomimetic_catalog(self):
        for hid in ('H-SERVO-1', 'H-DIST-1', 'H-SWEEP-1', 'H-DUTY-1'):
            self.assertIn(hid, wl.BIOMIMETIC_HYPOTHESES, f"{hid} missing from BIOMIMETIC_HYPOTHESES")


class TestEnduranceScoringCurriculumAndCMTS(unittest.TestCase):
    """Verifies SG scoring tasks, ALSS segments, curriculum, and CMTS context champions."""

    def test_sg_tasks_metrics(self):
        expected_tasks = [
            ('servo_rail_power_w', -1),
            ('aoa_tracking_error_deg', -1),
            ('passive_twist_deg', +1),
            ('total_bus_w_at_thrust', -1),
            ('upstroke_inertia_ratio', -1),
            ('downstroke_fraction', +1),
        ]
        for task_name, direction in expected_tasks:
            self.assertIn(task_name, wl._SG_TASKS, f"{task_name} missing from _SG_TASKS")
            self.assertEqual(wl._SG_TASKS[task_name]['direction'], direction)

    def test_alss_curriculum_segments(self):
        expected_segs = {
            'servo1_power_b': 'H-SERVO-1',
            'dist1_loading_b': 'H-DIST-1',
            'sweep1_upstroke_b': 'H-SWEEP-1',
            'duty1_asymmetry_b': 'H-DUTY-1',
        }
        for seg_name, target_hid in expected_segs.items():
            self.assertIn(seg_name, wl._ALSS_SEGMENTS, f"{seg_name} missing from _ALSS_SEGMENTS")
            self.assertEqual(wl._ALSS_SEGMENTS[seg_name][1], target_hid)
            self.assertIn(seg_name, wl._ALSS_CURRICULUM.get('B', []), f"{seg_name} not in curriculum B")

    def test_cmts_champions(self):
        expected_champs = {
            'servo_power': 'H-SERVO-1',
            'load_profile': 'H-DIST-1',
            'sweep_inertia': 'H-SWEEP-1',
            'duty_asymmetry': 'H-DUTY-1',
        }
        for ctx, hid in expected_champs.items():
            self.assertIn(ctx, wl._CMTS_CHAMPIONS, f"{ctx} missing from _CMTS_CHAMPIONS")
            self.assertEqual(wl._CMTS_CHAMPIONS[ctx]['hyp_id'], hid)


class TestEnduranceDiscoveryAndRouting(unittest.TestCase):
    """Verifies keyword focus routing, sensor triggers, and genome evolution routing."""

    def setUp(self):
        import copy
        self._orig_endurance = dict(wl._MW_ENDURANCE)
        self._orig_pitch = dict(wl._MW_PITCH)
        self._orig_perf = dict(wl._MW_PERFORMANCE)
        self._orig_stability = dict(wl._MW_STABILITY)
        self._orig_lever_weights = copy.deepcopy(wl._lever_mode_weights)

    def tearDown(self):
        import copy
        wl._MW_ENDURANCE.clear()
        wl._MW_ENDURANCE.update(self._orig_endurance)
        wl._MW_PITCH.clear()
        wl._MW_PITCH.update(self._orig_pitch)
        wl._MW_PERFORMANCE.clear()
        wl._MW_PERFORMANCE.update(self._orig_perf)
        wl._MW_STABILITY.clear()
        wl._MW_STABILITY.update(self._orig_stability)
        wl._lever_mode_weights.clear()
        wl._lever_mode_weights.update(copy.deepcopy(self._orig_lever_weights))

    def test_discovery_by_focus_keywords(self):
        test_cases = [
            ('h_servo_1', 'H-SERVO-1'),
            ('servo_power', 'H-SERVO-1'),
            ('passive_twist', 'H-SERVO-1'),
            ('h_dist_1', 'H-DIST-1'),
            ('streamwise_loading', 'H-DIST-1'),
            ('pitch_feedforward', 'H-DIST-1'),
            ('h_sweep_1', 'H-SWEEP-1'),
            ('upstroke_sweep', 'H-SWEEP-1'),
            ('sweep_clamp', 'H-SWEEP-1'),
            ('h_duty_1', 'H-DUTY-1'),
            ('asymmetric_duty', 'H-DUTY-1'),
            ('downstroke_ratio', 'H-DUTY-1'),
            ('endurance_suite', 'H-SERVO-1'),
        ]
        for kw, expected_hid in test_cases:
            h = wl.discovery_hypothesis_select(focus=kw)
            self.assertIsNotNone(h, f"Failed to select hypothesis for focus='{kw}'")
            self.assertEqual(h['id'], expected_hid)

    def test_discovery_by_sensor_triggers(self):
        cases = [
            ({'h_servo_1_active': True}, 'H-SERVO-1'),
            ({'servo_rail_power_w': 65.0}, 'H-SERVO-1'),
            ({'h_dist_1_active': True}, 'H-DIST-1'),
            ({'pitch_trim_error_deg': 3.0}, 'H-DIST-1'),
            ({'h_sweep_1_active': True}, 'H-SWEEP-1'),
            ({'sweep_retraction_active': True}, 'H-SWEEP-1'),
            ({'h_duty_1_active': True}, 'H-DUTY-1'),
            ({'flapping_duty_asymmetric': True}, 'H-DUTY-1'),
        ]
        for s_dict, expected_hid in cases:
            h = wl.discovery_hypothesis_select(sensors=s_dict)
            self.assertIsNotNone(h, f"Failed to select hypothesis for sensors={s_dict}")
            self.assertEqual(h['id'], expected_hid)

    def test_genome_routing(self):
        sensors = {'V_ms': 9.0, 'h_agl_m': 30.0}

        # H-SERVO-1 -> ENDURANCE (sustained_wh_per_km), PERFORMANCE (efficiency)
        orig_wh = wl._MW_ENDURANCE.get('sustained_wh_per_km', 0.0)
        wl._update_genome_from_trial('H-SERVO-1', 0.20, sensors)
        self.assertGreater(wl._MW_ENDURANCE.get('sustained_wh_per_km', 0.0), orig_wh)

        # H-DIST-1 -> PITCH (pitch_aoa), STABILITY (proverse_yaw)
        orig_pitch = wl._MW_PITCH.get('pitch_aoa', 0.0)
        wl._update_genome_from_trial('H-DIST-1', 0.20, sensors)
        self.assertGreater(wl._MW_PITCH.get('pitch_aoa', 0.0), orig_pitch)

        # H-DUTY-1 -> ENDURANCE (thermal_budget)
        orig_thermal = wl._MW_ENDURANCE.get('thermal_budget', 0.0)
        wl._update_genome_from_trial('H-DUTY-1', 0.20, sensors)
        self.assertGreater(wl._MW_ENDURANCE.get('thermal_budget', 0.0), orig_thermal)


class TestAIObserverTriggers(unittest.TestCase):
    """Verifies AI Observer triggers 39-42 for the endurance suite."""

    def setUp(self):
        wl._ALC_SHARED['priority_hyp_queue'].clear()

    def test_ai_observer_h_servo_1_trigger(self):
        sensors = {'servo_rail_power_w': 55.0, 'V_ms': 9.0, 'lever_pct': 60.0}
        obs = wl.ai_observer_tick(sensors, {})
        self.assertTrue(obs.get('h_servo1_active'))
        queue_hids = [item['hyp_id'] for item in wl._ALC_SHARED['priority_hyp_queue']]
        self.assertIn('H-SERVO-1', queue_hids)

    def test_ai_observer_h_dist_1_trigger(self):
        sensors = {'pitch_trim_error_deg': 2.8, 'V_ms': 8.0, 'lever_pct': 60.0}
        obs = wl.ai_observer_tick(sensors, {})
        self.assertTrue(obs.get('h_dist1_active'))
        queue_hids = [item['hyp_id'] for item in wl._ALC_SHARED['priority_hyp_queue']]
        self.assertIn('H-DIST-1', queue_hids)

    def test_ai_observer_h_sweep_1_trigger(self):
        sensors = {'sweep_retraction_active': True, 'V_ms': 9.0, 'lever_pct': 60.0}
        obs = wl.ai_observer_tick(sensors, {})
        self.assertTrue(obs.get('h_sweep1_active'))
        queue_hids = [item['hyp_id'] for item in wl._ALC_SHARED['priority_hyp_queue']]
        self.assertIn('H-SWEEP-1', queue_hids)

    def test_ai_observer_h_duty_1_trigger(self):
        sensors = {'motor_currents_a': [28.0, 15.0, 14.0], 'V_ms': 5.0, 'lever_pct': 55.0}
        obs = wl.ai_observer_tick(sensors, {})
        self.assertTrue(obs.get('h_duty1_active'))
        queue_hids = [item['hyp_id'] for item in wl._ALC_SHARED['priority_hyp_queue']]
        self.assertIn('H-DUTY-1', queue_hids)


class TestWaveKinematicGenerators(unittest.TestCase):
    """Verifies wave generators in mco_wave_library."""

    def test_wave_endurance_passive_twist(self):
        res = wl.wave_endurance_passive_twist(t_sec=0.125, f_hz=2.0, cmd_aoa_amp=14.0, aero_twist_deg=4.0)
        self.assertIn('aoa_cmd_deg', res)
        self.assertIn('passive_twist_deg', res)
        self.assertIn('power_savings_w', res)
        self.assertGreater(res['power_savings_w'], 0.0)

    def test_wave_endurance_streamwise_distribution(self):
        res = wl.wave_endurance_streamwise_distribution(t_sec=0.1, f_hz=2.0, base_amp_deg=96.0, amp_spread_deg=8.0)
        self.assertEqual(res['amp_w1_deg'], 100.0)
        self.assertEqual(res['amp_w3_deg'], 92.0)
        self.assertTrue(res['pitch_moment_balanced'])

    def test_wave_endurance_upstroke_sweep_clamp(self):
        # Even with 35.0 deg input target, must clamp to <= 12.0 deg
        res = wl.wave_endurance_upstroke_sweep(t_sec=0.35, f_hz=2.0, sweep_amp_deg=35.0)
        self.assertLessEqual(res['sweep_deg'], 12.0)
        self.assertTrue(res['sweep_clamp_enforced'])

    def test_wave_endurance_asymmetric_duty_clamp(self):
        # Even with 70% input downstroke, must clamp to <= 57.0%
        res = wl.wave_endurance_asymmetric_duty(t_sec=0.2, f_hz=2.0, downstroke_pct=70.0)
        self.assertLessEqual(res['duty_downstroke_pct'], 57.0)
        self.assertGreaterEqual(res['duty_downstroke_pct'], 52.0)
        self.assertTrue(res['current_headroom_ok'])


class TestFlightGovernorEnduranceGovernors(unittest.TestCase):
    """Verifies flight governor safety clamps, servo power tracking, and compensators."""

    def test_servo_power_tracker_hardware_measurement(self):
        tracker = ServoPowerTracker(window_sec=1.0, sample_rate_hz=50.0)
        # Feed 10 samples of 7.4V, 2.7A (~20W)
        for i in range(10):
            res = tracker.tick(v_servo_v=7.4, i_servo_a=2.7, t_s=i * 0.02, is_hardware_measured=True)
        self.assertAlmostEqual(res['mean_power_5s_w'], 19.98, places=1)
        self.assertGreater(res['power_savings_w'], 30.0)  # 55W - 20W = 35W savings
        self.assertEqual(res['status'], 'NOMINAL')

    def test_servo_power_tracker_hardware_offline_fallback(self):
        tracker = ServoPowerTracker(window_sec=1.0, sample_rate_hz=50.0)
        # Hardware offline: uses software model based on passive twist (4 deg)
        res = tracker.tick(t_s=0.02, is_hardware_measured=False, passive_twist_deg=4.0)
        self.assertFalse(res['hardware_online'])
        self.assertGreater(res['current_power_w'], 15.0)
        self.assertLess(res['current_power_w'], 40.0)

    def test_servo_tick_hook(self):
        result_dict = {}
        st = _servo_tick({'v_servo_v': 7.4, 'i_servo_a': 3.0, 'servo_ina226_valid': True}, result_dict)
        self.assertIn('servo_rail_power_w', result_dict)
        self.assertIn('servo_power_savings_w', result_dict)

    def test_h_dist_pitch_compensation(self):
        # 15% front bias generates +4.56 Nm pitch-up moment, requiring ~2.5° aft down-trim
        res = compute_h_dist_pitch_compensation(front_bias=1.15, base_aoa_deg=14.0, nominal_moment_nm=4.56)
        self.assertEqual(res['pitch_trim_feedforward_deg'], 2.5)
        self.assertEqual(res['aft_aoa_compensated_deg'], 11.5)
        self.assertTrue(res['longitudinal_balanced'])

    def test_h_duty_asymmetric_profile_clamp_and_derating(self):
        # Normal frequency 2.0 Hz: duty cycle clamped to 57% max
        res_normal = compute_h_duty_asymmetric_profile(t_sec=0.1, f_hz=2.0, downstroke_pct=65.0)
        self.assertEqual(res_normal['duty_downstroke_pct'], 57.0)
        self.assertTrue(res_normal['current_limit_35a_safe'])
        self.assertFalse(res_normal['frequency_derating_applied'])

        # High frequency 3.0 Hz: frequency derating applied towards symmetric
        res_high_f = compute_h_duty_asymmetric_profile(t_sec=0.1, f_hz=3.0, downstroke_pct=56.0)
        self.assertTrue(res_high_f['frequency_derating_applied'])
        self.assertLess(res_high_f['duty_downstroke_pct'], 56.0)
        self.assertTrue(res_high_f['current_limit_35a_safe'])

    def test_h_sweep_upstroke_trajectory_clamp(self):
        # Test target of 35° derated strictly to <= 12°
        res = compute_h_sweep_upstroke_trajectory(t_sec=0.35, f_hz=2.0, flap_amp_deg=90.0, sweep_target_deg=35.0)
        self.assertLessEqual(res['sweep_deg'], 12.0)
        self.assertEqual(res['clamped_sweep_deg'], 12.0)
        self.assertTrue(res['clamp_enforced'])


class TestPerformanceModelEnduranceAccounting(unittest.TestCase):
    """Verifies system power accounting and evaluate_endurance_suite extension."""

    def setUp(self):
        self.pm = PerformanceModel()

    def test_calculate_includes_servo_and_avionics_power(self):
        res = self.pm.calculate(freq_hz=2.0, V_ms=9.0, sweep_deg_mean=60.0,
                                amplitude_deg=96.0, throttle_frac=0.85,
                                p_servo_w=20.0, p_avionics_w=18.0)
        self.assertIn('p_motor_w', res)
        self.assertIn('p_servo_w', res)
        self.assertIn('p_avionics_w', res)
        self.assertIn('p_total_w', res)
        self.assertEqual(res['p_servo_w'], 20.0)
        self.assertEqual(res['p_avionics_w'], 18.0)
        self.assertAlmostEqual(res['p_total_w'], res['p_motor_w'] + 20.0 + 18.0, places=1)
        self.assertIn('endurance_total_min', res)
        self.assertGreater(res['endurance_total_min'], 4.0)

    def test_evaluate_endurance_suite_gains(self):
        eval_res = self.pm.evaluate_endurance_suite()
        self.assertIn('baseline', eval_res)
        self.assertIn('endurance_suite', eval_res)
        self.assertIn('gains', eval_res)

        # Net flight time gain in cruise must be >= 1.5 minutes (>15%)
        self.assertGreater(eval_res['gains']['cruise_net_gain_min'], 1.5)
        self.assertGreater(eval_res['gains']['cruise_gain_pct'], 15.0)

        # Servo power savings must be 35.0W
        self.assertEqual(eval_res['energy_breakdown_w']['servo_savings_w'], 35.0)


class TestGroundStationDiagnostics(unittest.TestCase):
    """Verifies ground station commands for endurance status."""

    def test_endurance_status_output(self):
        report = wl.endurance_status()
        self.assertIn('H-SERVO-1', report)
        self.assertIn('H-DIST-1', report)
        self.assertIn('H-SWEEP-1', report)
        self.assertIn('H-DUTY-1', report)
        self.assertIn('POWER CONSUMPTION BREAKDOWN', report)

    def test_advanced_physics_status_contains_endurance_suite(self):
        report = wl.advanced_physics_status()
        self.assertIn('ENDURANCE HYPOTHESIS SUITE', report)
        self.assertIn('H-SERVO-1', report)
        self.assertIn('H-DIST-1', report)


if __name__ == '__main__':
    unittest.main()
