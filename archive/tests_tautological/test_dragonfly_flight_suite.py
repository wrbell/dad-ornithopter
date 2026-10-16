"""
Unit Tests: Dragonfly Aerodynamic Suite (H-86 to H-88)
======================================================
Tests coverage for:
  - H-86: Reverse Metachronal Cascade Hindwing-Leading Thrust Boost
  - H-87: Active Upstroke Dynamic Airbrake & Reverse Flight
  - H-88: Odonata Direct-Musculature Stroke-Plane Tilt Vectoring

Tests verify:
  1. Hypothesis definitions and metadata in AERO_HYPOTHESES and _ADV_NEW_HYPS
  2. Scoring metrics in _SG_TASKS
  3. ALSS segments and curriculum enrolment in Curricula A & B and _PHASE_TO_SG_TASK
  4. Expanded discovery bounds in _EXPANDED_DISCOVERY_BOUNDS
  5. Context champions in _CMTS_CHAMPIONS
  6. AI observer trigger conditions in ai_observer_tick
  7. Keyword and sensor routing in discovery_hypothesis_select
  8. Genome evolution weight vector routing in _update_genome_from_trial
  9. Intent interpretation, behaviour execution, and macro aborts in mco_intent_control
  10. Ground station diagnostic command advanced_physics_status
"""

import sys
import os
import unittest

# Ensure firmware directory is in python path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'firmware'))

import mco_wave_library as wl
import mco_intent_control as ic


class TestDragonflySuiteRegistration(unittest.TestCase):
    """Test hypothesis registration and metadata for H-86 through H-88, H-88A, H-88B."""

    def test_hypotheses_registered_in_adv_new_hyps(self):
        adv_ids = {h['id'] for h in wl._ADV_NEW_HYPS}
        for hid in ('H-86', 'H-87', 'H-88', 'H-88A', 'H-88B'):
            self.assertIn(hid, adv_ids, f"{hid} missing from _ADV_NEW_HYPS")

    def test_hypotheses_registered_in_aero_hypotheses(self):
        aero_ids = {h['id'] for h in wl.AERO_HYPOTHESES}
        for hid in ('H-86', 'H-87', 'H-88', 'H-88A', 'H-88B'):
            self.assertIn(hid, aero_ids, f"{hid} missing from AERO_HYPOTHESES")

    def test_sg_tasks_metrics(self):
        expected_metrics = [
            'reverse_cascade_thrust_gain_pct',
            'active_upstroke_decel_g',
            'stroke_plane_yaw_moment_nm',
            'sweep_bias_yaw_moment_nm',
        ]
        for metric in expected_metrics:
            self.assertIn(metric, wl._SG_TASKS, f"{metric} missing from _SG_TASKS")
            task = wl._SG_TASKS[metric]
            self.assertIn('axis', task)
            self.assertIn('sensor', task)
            self.assertIn('direction', task)

    def test_alss_curriculum_enrolment(self):
        self.assertIn('h87_active_upstroke_a', wl._ALSS_CURRICULUM['A'],
                      "h87_active_upstroke_a should be in Curriculum A (arrest/safety)")
        self.assertIn('h86_reverse_cascade_b', wl._ALSS_CURRICULUM['B'],
                      "h86_reverse_cascade_b should be in Curriculum B (cruise performance)")
        self.assertIn('h88_stroke_plane_vector_b', wl._ALSS_CURRICULUM['B'],
                      "h88_stroke_plane_vector_b should be in Curriculum B (agility/vectoring)")
        self.assertIn('h88a_dynamic_vector_b', wl._ALSS_CURRICULUM['B'],
                      "h88a_dynamic_vector_b should be in Curriculum B (harmonic vectoring)")
        self.assertIn('h88b_sweep_bias_b', wl._ALSS_CURRICULUM['B'],
                      "h88b_sweep_bias_b should be in Curriculum B (quasi-steady sweep bias)")

    def test_phase_to_sg_task_mapping(self):
        self.assertIn('dragonfly_aerodynamics', wl._PHASE_TO_SG_TASK)
        tasks = wl._PHASE_TO_SG_TASK['dragonfly_aerodynamics']
        self.assertIn('reverse_cascade_thrust_gain_pct', tasks)
        self.assertIn('active_upstroke_decel_g', tasks)
        self.assertIn('stroke_plane_yaw_moment_nm', tasks)
        self.assertIn('sweep_bias_yaw_moment_nm', tasks)

    def test_discovery_bounds(self):
        expected_param_keys = [
            'h86_reverse_phase_deg', 'h86_thrust_boost_pct',
            'h87_upstroke_aoa_deg', 'h87_decel_g',
            'h88_stroke_plane_tilt_deg', 'h88_yaw_moment_nm',
            'h88a_stroke_plane_tilt_deg', 'h88a_sweep_harmonic_amp_deg', 'h88a_yaw_moment_nm',
            'h88b_sweep_bias_deg', 'h88b_yaw_moment_nm',
        ]
        for key in expected_param_keys:
            self.assertIn(key, wl._EXPANDED_DISCOVERY_BOUNDS,
                          f"{key} missing from _EXPANDED_DISCOVERY_BOUNDS")
            lo, hi = wl._EXPANDED_DISCOVERY_BOUNDS[key]
            self.assertLess(lo, hi)

    def test_cmts_champions(self):
        expected_champions = {
            'reverse_metachronal_thrust': 'H-86',
            'active_upstroke_reverse_brake': 'H-87',
            'odonata_stroke_plane_vectoring': 'H-88',
            'odonata_dynamic_vectoring': 'H-88A',
            'odonata_sweep_bias_trim': 'H-88B',
        }
        for champ_name, expected_hid in expected_champions.items():
            self.assertIn(champ_name, wl._CMTS_CHAMPIONS, f"{champ_name} missing from _CMTS_CHAMPIONS")
            self.assertEqual(wl._CMTS_CHAMPIONS[champ_name].get('hyp_id'), expected_hid)


class TestDragonflyObserverTriggers(unittest.TestCase):
    """Test AI observer triggers for queuing H-86 through H-88."""

    def setUp(self):
        wl._ALC_SHARED['priority_hyp_queue'].clear()
        wl._ALC_SHARED['alert_log'].clear()

    def test_observer_trigger_h86_reverse_cascade(self):
        sensors = {'lever_pct': 72.0, 'V_ms': 12.0}
        obs = wl.ai_observer_tick(sensors, {})
        self.assertTrue(obs.get('h86_reverse_cascade_active'))
        queued_ids = [q['hyp_id'] for q in wl._ALC_SHARED['priority_hyp_queue']]
        self.assertIn('H-86', queued_ids)

    def test_observer_trigger_h87_active_upstroke_brake(self):
        sensors = {'lever_pct': 5.0, 'pitch_stick': -0.50}
        obs = wl.ai_observer_tick(sensors, {})
        self.assertTrue(obs.get('h87_active_upstroke_brake'))
        queued_ids = [q['hyp_id'] for q in wl._ALC_SHARED['priority_hyp_queue']]
        self.assertIn('H-87', queued_ids)

    def test_observer_trigger_h88_stroke_plane_vector(self):
        sensors = {'yaw_stick': 0.55, 'roll_stick': 0.05}
        obs = wl.ai_observer_tick(sensors, {})
        self.assertTrue(obs.get('h88_stroke_plane_vector_active'))
        queued_ids = [q['hyp_id'] for q in wl._ALC_SHARED['priority_hyp_queue']]
        self.assertIn('H-88', queued_ids)

    def test_observer_trigger_h88a_dynamic_vector(self):
        sensors = {'dynamic_vectoring': True}
        obs = wl.ai_observer_tick(sensors, {})
        self.assertTrue(obs.get('h88a_dynamic_vector_active'))
        queued_ids = [q['hyp_id'] for q in wl._ALC_SHARED['priority_hyp_queue']]
        self.assertIn('H-88A', queued_ids)

    def test_observer_trigger_h88b_sweep_bias(self):
        sensors = {'sweep_bias_trim': True}
        obs = wl.ai_observer_tick(sensors, {})
        self.assertTrue(obs.get('h88b_sweep_bias_active'))
        queued_ids = [q['hyp_id'] for q in wl._ALC_SHARED['priority_hyp_queue']]
        self.assertIn('H-88B', queued_ids)


class TestDragonflyDiscoverySelector(unittest.TestCase):
    """Test discovery hypothesis selector keyword and sensor routing."""

    def test_focus_keyword_dragonfly(self):
        h = wl.discovery_hypothesis_select(focus='dragonfly')
        self.assertIsNotNone(h)
        self.assertIn(h['id'], ('H-86', 'H-87', 'H-88'))

    def test_focus_keyword_reverse_metachronal(self):
        h = wl.discovery_hypothesis_select(focus='reverse_metachronal')
        self.assertIsNotNone(h)
        self.assertEqual(h['id'], 'H-86')

    def test_focus_keyword_active_upstroke(self):
        h = wl.discovery_hypothesis_select(focus='active_upstroke')
        self.assertIsNotNone(h)
        self.assertEqual(h['id'], 'H-87')

    def test_focus_keyword_direct_musculature(self):
        h = wl.discovery_hypothesis_select(focus='direct_musculature')
        self.assertIsNotNone(h)
        self.assertEqual(h['id'], 'H-88')

    def test_focus_keyword_dynamic_vectoring(self):
        h = wl.discovery_hypothesis_select(focus='dynamic_vectoring')
        self.assertIsNotNone(h)
        self.assertEqual(h['id'], 'H-88A')

    def test_focus_keyword_harmonic_stroke_plane(self):
        h = wl.discovery_hypothesis_select(focus='harmonic_stroke_plane')
        self.assertIsNotNone(h)
        self.assertEqual(h['id'], 'H-88A')

    def test_focus_keyword_sweep_bias(self):
        h = wl.discovery_hypothesis_select(focus='sweep_bias')
        self.assertIsNotNone(h)
        self.assertEqual(h['id'], 'H-88B')

    def test_focus_keyword_quasi_steady_sweep(self):
        h = wl.discovery_hypothesis_select(focus='quasi_steady_sweep')
        self.assertIsNotNone(h)
        self.assertEqual(h['id'], 'H-88B')

    def test_sensor_routing_decoupled_yaw(self):
        sensors = {'yaw_stick': 0.45, 'roll_stick': 0.05, 'V_ms': 8.0, 'lever_pct': 50.0}
        h = wl.discovery_hypothesis_select(sensors=sensors)
        self.assertIsNotNone(h)
        self.assertEqual(h['id'], 'H-88')

    def test_sensor_routing_active_braking(self):
        sensors = {'lever_pct': 10.0, 'V_ms': 10.0, 'pitch_stick': 0.0, 'roll_stick': 0.0, 'yaw_stick': 0.0}
        h = wl.discovery_hypothesis_select(sensors=sensors)
        self.assertIsNotNone(h)
        self.assertEqual(h['id'], 'H-87')

    def test_sensor_routing_fast_cruise_thrust(self):
        sensors = {'lever_pct': 72.0, 'V_ms': 12.5, 'pitch_stick': 0.0, 'roll_stick': 0.0, 'yaw_stick': 0.0}
        h = wl.discovery_hypothesis_select(sensors=sensors)
        self.assertIsNotNone(h)
        self.assertEqual(h['id'], 'H-86')


class TestDragonflyGenomeEvolution(unittest.TestCase):
    """Test genome routing for H-86, H-87, H-88, H-88A, and H-88B."""

    def test_genome_routing_execution(self):
        try:
            wl._update_genome_from_trial('H-86', 1.5, {'V_ms': 12.0})
            wl._update_genome_from_trial('H-87', 0.8, {'V_ms': 8.0})
            wl._update_genome_from_trial('H-88', 1.2, {'V_ms': 7.0})
            wl._update_genome_from_trial('H-88A', 1.4, {'V_ms': 8.0})
            wl._update_genome_from_trial('H-88B', 1.1, {'V_ms': 9.0})
        except Exception as e:
            self.fail(f"_update_genome_from_trial raised exception for dragonfly hypotheses: {e}")


class TestDragonflyIntentAndBehaviour(unittest.TestCase):
    """Test intent classification, behaviour dispatching, and safety aborts."""

    def setUp(self):
        self.interp = ic.IntentInterpreter()

    def test_stroke_plane_vector_intent(self):
        intent, conf, params = self.interp.interpret(
            {'yaw': 0.5, 'roll': 0.0, 'pitch': 0.0, 'throttle': 0.5},
            {},
            {'V_ms': 6.0, 'h_agl_m': 20.0, 'on_ground': False}
        )
        self.assertEqual(intent, ic.Intent.STROKE_PLANE_VECTOR)
        self.assertEqual(params.get('direction'), 'right')
        self.assertAlmostEqual(params.get('tilt_deg'), 18.0)

    def test_active_upstroke_brake_intent(self):
        intent, conf, params = self.interp.interpret(
            {'yaw': 0.0, 'roll': 0.0, 'pitch': -0.25, 'throttle': 0.10},
            {},
            {'V_ms': 8.0, 'h_agl_m': 15.0, 'on_ground': False}
        )
        self.assertEqual(intent, ic.Intent.ACTIVE_UPSTROKE_BRAKE)
        self.assertAlmostEqual(params.get('aoa_deg'), -28.0)
        self.assertAlmostEqual(params.get('decel_target_g'), 1.25)

    def test_reverse_cascade_cruise_intent(self):
        intent, conf, params = self.interp.interpret(
            {'yaw': 0.0, 'roll': 0.0, 'pitch': 0.0, 'throttle': 0.75},
            {},
            {'V_ms': 12.0, 'h_agl_m': 30.0, 'on_ground': False}
        )
        self.assertEqual(intent, ic.Intent.REVERSE_CASCADE_CRUISE)
        self.assertAlmostEqual(params.get('cascade_phase_deg'), -60.0)
        self.assertAlmostEqual(params.get('thrust_boost_pct'), 24.5)

    def test_behaviour_dispatcher_active_upstroke_brake(self):
        class MockCtrl:
            freq_hz = 1.0
            _tgt_freq = 1.0
            _tgt_sweeps = [60.0] * 8
            _tgt_dihedrals = [0.0] * 8
            _tgt_pitches = [0.0] * 8

        ctrl = MockCtrl()
        disp = ic.BehaviourDispatcher(ctrl)

        name, intent, status = disp.update(
            0.1,
            {'pitch': -0.25, 'throttle': 0.10, 'roll': 0.0, 'yaw': 0.0},
            {},
            {'V_ms': 8.0, 'h_agl_m': 15.0, 'on_ground': False}
        )
        self.assertEqual(name, 'active_upstroke_brake')
        self.assertEqual(intent, ic.Intent.ACTIVE_UPSTROKE_BRAKE)
        self.assertEqual(status, ic.RUNNING)
        self.assertEqual(ctrl._tgt_pitches, [-28.0] * 8)
        self.assertEqual(ctrl._tgt_sweeps, [70.0] * 8)

    def test_active_upstroke_brake_macro_abort(self):
        aborted, reason = ic.check_macro_abort('active_upstroke_brake', {'h_agl_m': 1.2, 'V_ms': 5.0})
        self.assertTrue(aborted)
        self.assertIn('altitude floor', reason)

        aborted_speed, reason_speed = ic.check_macro_abort('active_upstroke_brake', {'h_agl_m': 10.0, 'V_ms': 0.2})
        self.assertTrue(aborted_speed)
        self.assertIn('airspeed', reason_speed)


class TestDragonflyWaveGenerators(unittest.TestCase):
    """Test kinematic wave generators for H-88A and H-88B."""

    def test_wave_odonata_dynamic_vectoring(self):
        res = wl.wave_odonata_dynamic_vectoring(0.1, f_hz=2.6, flap_amp_deg=24.0,
                                                stroke_plane_tilt_deg=18.0, yaw_demand=1.0)
        self.assertIn('flap_port_deg', res)
        self.assertIn('sweep_port_deg', res)
        self.assertIn('eff_tilt_deg', res)
        self.assertEqual(res['eff_tilt_deg'], 18.0)
        self.assertTrue(res['decoupled_yaw_active'])
        self.assertAlmostEqual(res['sweep_port_deg'], -res['sweep_stbd_deg'])

    def test_wave_odonata_sweep_bias_trim(self):
        res = wl.wave_odonata_sweep_bias_trim(yaw_demand=0.5, bias_max_deg=12.0)
        self.assertEqual(res['sweep_bias_port_deg'], 6.0)
        self.assertEqual(res['sweep_bias_stbd_deg'], -6.0)
        self.assertEqual(res['differential_sweep_deg'], 12.0)
        self.assertTrue(res['is_trim_active'])
        self.assertEqual(res['power_penalty_w'], 0.0)


class TestGroundStationDiagnostics(unittest.TestCase):
    """Verify advanced_physics_status diagnostic output execution."""

    def test_advanced_physics_status_runs(self):
        import io
        from contextlib import redirect_stdout

        f = io.StringIO()
        with redirect_stdout(f):
            wl.advanced_physics_status()
        out = f.getvalue()
        self.assertIn("DRAGONFLY AERODYNAMIC SUITE (H-86 to H-88)", out)
        self.assertIn("H-86 Reverse Metachronal Cascade", out)
        self.assertIn("H-87 Active Upstroke Dynamic Brake", out)
        self.assertIn("H-88 Odonata Stroke-Plane Vectoring", out)
        self.assertIn("H-88A Odonata Dynamic Vectoring", out)
        self.assertIn("H-88B Odonata Sweep Bias Trim", out)


if __name__ == '__main__':
    unittest.main()
