"""
Unit Tests: Insect Flight Biomechanics Suite (H-89 to H-95)
===========================================================
Tests coverage for:
  - H-89: Dynamic Kramer Pre-Rotation Lift Burst (Diptera/Hymenoptera)
  - H-90: Trapped-Vortex Cavity Boundary Layer Energization (Odonata/Meganeuropsis)
  - H-91: Lepidopteran Dorsal Clap Jet Propulsion (Papilionoidea)
  - H-92: Coleopteran Elytron Canard Obstacle Climb (Coleoptera)
  - H-93: Dipteran Synchronous Coriolis Gust Preemption (Diptera Halteres)
  - H-94: Orthopteran Traveling Torsional Wave Cruise (Orthoptera Locusts)
  - H-95: Tandem Dynamic Slotted Fowler Flap Glide (Hymenoptera Hamuli)

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


class TestInsectSuiteRegistration(unittest.TestCase):
    """Test hypothesis registration and metadata for H-89 through H-95."""

    def test_hypotheses_registered_in_adv_new_hyps(self):
        adv_ids = {h['id'] for h in wl._ADV_NEW_HYPS}
        for hid in ('H-89', 'H-90', 'H-91', 'H-92', 'H-93', 'H-94', 'H-95'):
            self.assertIn(hid, adv_ids, f"{hid} missing from _ADV_NEW_HYPS")

    def test_hypotheses_registered_in_aero_hypotheses(self):
        aero_ids = {h['id'] for h in wl.AERO_HYPOTHESES}
        for hid in ('H-89', 'H-90', 'H-91', 'H-92', 'H-93', 'H-94', 'H-95'):
            self.assertIn(hid, aero_ids, f"{hid} missing from AERO_HYPOTHESES")

    def test_sg_tasks_metrics(self):
        expected_metrics = [
            'kramer_circulation_lift_boost_pct',
            'trapped_vortex_stall_alpha_deg',
            'clap_jet_impulse_n',
            'canard_climb_gradient_deg',
            'coriolis_demod_rms_g',
            'torsional_wave_wh_per_km',
            'slotted_flap_cl_max',
        ]
        for metric in expected_metrics:
            self.assertIn(metric, wl._SG_TASKS, f"{metric} missing from _SG_TASKS")
            task = wl._SG_TASKS[metric]
            self.assertIn('axis', task)
            self.assertIn('sensor', task)
            self.assertIn('direction', task)

    def test_alss_curriculum_enrolment(self):
        self.assertIn('h91_clap_jet_a', wl._ALSS_CURRICULUM['A'],
                      "h91_clap_jet_a should be in Curriculum A (burst acceleration)")
        self.assertIn('h93_coriolis_preempt_a', wl._ALSS_CURRICULUM['A'],
                      "h93_coriolis_preempt_a should be in Curriculum A (gust rejection)")
        self.assertIn('h89_kramer_burst_b', wl._ALSS_CURRICULUM['B'],
                      "h89_kramer_burst_b should be in Curriculum B")
        self.assertIn('h90_trapped_vortex_b', wl._ALSS_CURRICULUM['B'],
                      "h90_trapped_vortex_b should be in Curriculum B")
        self.assertIn('h92_elytron_canard_b', wl._ALSS_CURRICULUM['B'],
                      "h92_elytron_canard_b should be in Curriculum B (obstacle clearance)")
        self.assertIn('h94_torsional_wave_b', wl._ALSS_CURRICULUM['B'],
                      "h94_torsional_wave_b should be in Curriculum B")
        self.assertIn('h95_slotted_flap_b', wl._ALSS_CURRICULUM['B'],
                      "h95_slotted_flap_b should be in Curriculum B")

    def test_phase_to_sg_task_mapping(self):
        self.assertIn('insect_biomechanics', wl._PHASE_TO_SG_TASK)
        tasks = wl._PHASE_TO_SG_TASK['insect_biomechanics']
        self.assertIn('kramer_circulation_lift_boost_pct', tasks)
        self.assertIn('trapped_vortex_stall_alpha_deg', tasks)
        self.assertIn('clap_jet_impulse_n', tasks)
        self.assertIn('canard_climb_gradient_deg', tasks)
        self.assertIn('coriolis_demod_rms_g', tasks)
        self.assertIn('torsional_wave_wh_per_km', tasks)
        self.assertIn('slotted_flap_cl_max', tasks)

    def test_discovery_bounds(self):
        expected_param_keys = [
            'h89_pitch_lead_ms', 'h89_lift_boost_pct',
            'h90_cavity_depth_mm', 'h90_stall_alpha_deg',
            'h91_clap_dihedral_deg', 'h91_jet_impulse_n',
            'h92_canard_dihedral_deg', 'h92_climb_gradient_deg',
            'h93_jitter_rms_g', 'h93_filter_latency_ms',
            'h94_twist_amplitude_deg', 'h94_wh_per_km',
            'h95_slot_phase_lead_deg', 'h95_cl_max',
        ]
        for key in expected_param_keys:
            self.assertIn(key, wl._EXPANDED_DISCOVERY_BOUNDS,
                          f"{key} missing from _EXPANDED_DISCOVERY_BOUNDS")
            lo, hi = wl._EXPANDED_DISCOVERY_BOUNDS[key]
            self.assertLess(lo, hi)

    def test_cmts_champions(self):
        expected_champions = {
            'kramer_lift_burst': 'H-89',
            'trapped_vortex_cavity': 'H-90',
            'lepidopteran_clap_jet': 'H-91',
            'coleopteran_elytron_canard': 'H-92',
            'dipteran_coriolis_gust': 'H-93',
            'orthopteran_torsional_wave': 'H-94',
            'hymenopteran_slotted_flap': 'H-95',
        }
        for champ_name, expected_hid in expected_champions.items():
            self.assertIn(champ_name, wl._CMTS_CHAMPIONS, f"{champ_name} missing from _CMTS_CHAMPIONS")
            self.assertEqual(wl._CMTS_CHAMPIONS[champ_name].get('hyp_id'), expected_hid)


class TestInsectObserverTriggers(unittest.TestCase):
    """Test AI observer triggers for queuing H-89 through H-95."""

    def setUp(self):
        wl._ALC_SHARED['priority_hyp_queue'].clear()
        wl._ALC_SHARED['alert_log'].clear()

    def test_observer_trigger_h89_kramer_burst(self):
        sensors = {'V_ms': 4.2, 'pitch_stick': 0.65}
        obs = wl.ai_observer_tick(sensors, {})
        self.assertTrue(obs.get('h89_kramer_burst_active'))
        queued_ids = [q['hyp_id'] for q in wl._ALC_SHARED['priority_hyp_queue']]
        self.assertIn('H-89', queued_ids)

    def test_observer_trigger_h90_trapped_vortex(self):
        sensors = {'lever_pct': 35.0, 'V_ms': 8.0}
        obs = wl.ai_observer_tick(sensors, {})
        self.assertTrue(obs.get('h90_trapped_vortex_active'))
        queued_ids = [q['hyp_id'] for q in wl._ALC_SHARED['priority_hyp_queue']]
        self.assertIn('H-90', queued_ids)

    def test_observer_trigger_h91_clap_jet(self):
        sensors = {'lever_pct': 92.0, 'V_ms': 4.5}
        obs = wl.ai_observer_tick(sensors, {})
        self.assertTrue(obs.get('h91_clap_jet_active'))
        queued_ids = [q['hyp_id'] for q in wl._ALC_SHARED['priority_hyp_queue']]
        self.assertIn('H-91', queued_ids)

    def test_observer_trigger_h92_elytron_canard(self):
        sensors = {'pitch_stick': 0.75, 'V_ms': 6.5}
        obs = wl.ai_observer_tick(sensors, {})
        self.assertTrue(obs.get('h92_elytron_canard_active'))
        queued_ids = [q['hyp_id'] for q in wl._ALC_SHARED['priority_hyp_queue']]
        self.assertIn('H-92', queued_ids)

    def test_observer_trigger_h93_coriolis_preempt(self):
        sensors = {'angular_accel_dps2': 285.0}
        obs = wl.ai_observer_tick(sensors, {})
        self.assertTrue(obs.get('h93_coriolis_gust_active'))
        queued_ids = [q['hyp_id'] for q in wl._ALC_SHARED['priority_hyp_queue']]
        self.assertIn('H-93', queued_ids)

    def test_observer_trigger_h94_torsional_wave(self):
        sensors = {'lever_pct': 58.0, 'V_ms': 10.5, 'high_turbulence': False}
        obs = wl.ai_observer_tick(sensors, {})
        self.assertTrue(obs.get('h94_torsional_wave_active'))
        queued_ids = [q['hyp_id'] for q in wl._ALC_SHARED['priority_hyp_queue']]
        self.assertIn('H-94', queued_ids)

    def test_observer_trigger_h95_slotted_flap(self):
        sensors = {'lever_pct': 22.0, 'V_ms': 6.0}
        obs = wl.ai_observer_tick(sensors, {})
        self.assertTrue(obs.get('h95_slotted_flap_active'))
        queued_ids = [q['hyp_id'] for q in wl._ALC_SHARED['priority_hyp_queue']]
        self.assertIn('H-95', queued_ids)


class TestInsectDiscoverySelector(unittest.TestCase):
    """Test discovery hypothesis selector keyword and sensor routing."""

    def test_focus_keyword_insect_suite(self):
        h = wl.discovery_hypothesis_select(focus='insect_suite')
        self.assertIsNotNone(h)
        self.assertIn(h['id'], ('H-89', 'H-90', 'H-91', 'H-92', 'H-93', 'H-94', 'H-95'))

    def test_focus_keyword_kramer(self):
        h = wl.discovery_hypothesis_select(focus='kramer')
        self.assertIsNotNone(h)
        self.assertEqual(h['id'], 'H-89')

    def test_focus_keyword_trapped_vortex(self):
        h = wl.discovery_hypothesis_select(focus='trapped_vortex')
        self.assertIsNotNone(h)
        self.assertEqual(h['id'], 'H-90')

    def test_focus_keyword_clap_jet(self):
        h = wl.discovery_hypothesis_select(focus='clap_jet')
        self.assertIsNotNone(h)
        self.assertEqual(h['id'], 'H-91')

    def test_focus_keyword_elytron(self):
        h = wl.discovery_hypothesis_select(focus='elytron')
        self.assertIsNotNone(h)
        self.assertEqual(h['id'], 'H-92')

    def test_focus_keyword_coriolis(self):
        h = wl.discovery_hypothesis_select(focus='coriolis')
        self.assertIsNotNone(h)
        self.assertEqual(h['id'], 'H-93')

    def test_focus_keyword_locust(self):
        h = wl.discovery_hypothesis_select(focus='locust')
        self.assertIsNotNone(h)
        self.assertEqual(h['id'], 'H-94')

    def test_focus_keyword_slotted_glide(self):
        h = wl.discovery_hypothesis_select(focus='slotted_glide')
        self.assertIsNotNone(h)
        self.assertEqual(h['id'], 'H-95')

    def test_sensor_routing_coriolis_gust(self):
        sensors = {'angular_accel_dps2': 260.0, 'V_ms': 9.0}
        h = wl.discovery_hypothesis_select(sensors=sensors)
        self.assertIsNotNone(h)
        self.assertEqual(h['id'], 'H-93')

    def test_sensor_routing_elytron_canard_climb(self):
        sensors = {'pitch_stick': 0.75, 'V_ms': 7.0, 'lever_pct': 80.0}
        h = wl.discovery_hypothesis_select(sensors=sensors)
        self.assertIsNotNone(h)
        self.assertEqual(h['id'], 'H-92')

    def test_sensor_routing_clap_jet_breakout(self):
        sensors = {'lever_pct': 92.0, 'V_ms': 4.0, 'pitch_stick': 0.0, 'roll_stick': 0.0, 'yaw_stick': 0.0}
        h = wl.discovery_hypothesis_select(sensors=sensors)
        self.assertIsNotNone(h)
        self.assertEqual(h['id'], 'H-91')

    def test_sensor_routing_kramer_burst(self):
        sensors = {'pitch_stick': 0.65, 'V_ms': 4.0, 'lever_pct': 50.0}
        h = wl.discovery_hypothesis_select(sensors=sensors)
        self.assertIsNotNone(h)
        self.assertEqual(h['id'], 'H-89')


class TestInsectGenomeEvolution(unittest.TestCase):
    """Test genome routing for H-89 through H-95."""

    def test_genome_routing_execution(self):
        try:
            for hid in ('H-89', 'H-90', 'H-91', 'H-92', 'H-93', 'H-94', 'H-95'):
                wl._update_genome_from_trial(hid, 1.2, {'V_ms': 8.0})
        except Exception as e:
            self.fail(f"_update_genome_from_trial raised exception for insect hypotheses: {e}")


class TestInsectIntentAndBehaviour(unittest.TestCase):
    """Test intent classification, behaviour dispatching, and macro aborts for insect suite."""

    def setUp(self):
        self.interp = ic.IntentInterpreter()

    def test_kramer_lift_burst_intent(self):
        intent, conf, params = self.interp.interpret(
            {'yaw': 0.0, 'roll': 0.0, 'pitch': 0.65, 'throttle': 0.5},
            {},
            {'V_ms': 3.8, 'h_agl_m': 10.0, 'on_ground': False}
        )
        self.assertEqual(intent, ic.Intent.KRAMER_LIFT_BURST)
        self.assertAlmostEqual(params.get('pre_rotation_lead_deg'), 10.0)
        self.assertAlmostEqual(params.get('cl_transient'), 2.85)

    def test_clap_jet_breakout_intent(self):
        intent, conf, params = self.interp.interpret(
            {'yaw': 0.0, 'roll': 0.0, 'pitch': 0.0, 'throttle': 0.90},
            {},
            {'V_ms': 4.2, 'h_agl_m': 15.0, 'on_ground': False}
        )
        self.assertEqual(intent, ic.Intent.CLAP_JET_BREAKOUT)
        self.assertAlmostEqual(params.get('accel_g'), 1.45)

    def test_elytron_canard_climb_intent(self):
        intent, conf, params = self.interp.interpret(
            {'yaw': 0.0, 'roll': 0.0, 'pitch': 0.75, 'throttle': 0.70},
            {},
            {'V_ms': 7.5, 'h_agl_m': 25.0, 'on_ground': False}
        )
        self.assertEqual(intent, ic.Intent.ELYTRON_CANARD)
        self.assertEqual(params.get('canard_locked_station'), 1)
        self.assertAlmostEqual(params.get('canard_incidence_deg'), 8.5)

    def test_coriolis_gust_preempt_intent(self):
        intent, conf, params = self.interp.interpret(
            {'yaw': 0.0, 'roll': 0.0, 'pitch': 0.0, 'throttle': 0.50},
            {},
            {'V_ms': 10.0, 'h_agl_m': 30.0, 'on_ground': False, 'angular_accel_dps2': 250.0}
        )
        self.assertEqual(intent, ic.Intent.CORIOLIS_GUST_PREEMPT)
        self.assertAlmostEqual(params.get('counter_twist_deg'), 6.5)

    def test_locust_torsional_cruise_intent(self):
        intent, conf, params = self.interp.interpret(
            {'yaw': 0.0, 'roll': 0.0, 'pitch': 0.0, 'throttle': 0.55},
            {},
            {'V_ms': 11.0, 'h_agl_m': 40.0, 'on_ground': False}
        )
        self.assertEqual(intent, ic.Intent.LOCUST_TORSIONAL_CRUISE)
        self.assertAlmostEqual(params.get('drag_reduction_pct'), 32.0)

    def test_slotted_flap_glide_intent(self):
        intent, conf, params = self.interp.interpret(
            {'yaw': 0.0, 'roll': 0.0, 'pitch': 0.0, 'throttle': 0.35},
            {},
            {'V_ms': 8.0, 'h_agl_m': 40.0, 'on_ground': False}
        )
        self.assertEqual(intent, ic.Intent.SLOTTED_FLAP_GLIDE)
        self.assertAlmostEqual(params.get('slot_fowler_cl_max'), 2.45)

    def test_behaviour_dispatcher_elytron_canard(self):
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
            {'pitch': 0.75, 'throttle': 0.70, 'roll': 0.0, 'yaw': 0.0},
            {},
            {'V_ms': 7.5, 'h_agl_m': 25.0, 'on_ground': False}
        )
        self.assertEqual(name, 'elytron_canard')
        self.assertEqual(intent, ic.Intent.ELYTRON_CANARD)
        self.assertEqual(status, ic.RUNNING)
        # Verify station 1 locked canard (sweeps=0, pitch=8.5) and stations 2-4 flapping (sweeps=68, pitch=18)
        self.assertEqual(ctrl._tgt_sweeps[:2], [0.0, 0.0])
        self.assertEqual(ctrl._tgt_pitches[:2], [8.5, 8.5])
        self.assertEqual(ctrl._tgt_sweeps[2:], [68.0] * 6)
        self.assertEqual(ctrl._tgt_pitches[2:], [18.0] * 6)

    def test_kramer_and_clap_jet_macro_aborts(self):
        aborted_kramer, reason_k = ic.check_macro_abort('kramer_lift_burst', {'h_agl_m': 0.8, 'V_ms': 4.0})
        self.assertTrue(aborted_kramer)
        self.assertIn('altitude floor', reason_k)

        aborted_clap, reason_c = ic.check_macro_abort('clap_jet_breakout', {'h_agl_m': 15.0, 'V_ms': 14.0})
        self.assertTrue(aborted_clap)
        self.assertIn('speed too high', reason_c)


class TestInsectGroundStationDiagnostics(unittest.TestCase):
    """Verify advanced_physics_status diagnostic output execution with insect suite."""

    def test_advanced_physics_status_runs_with_insects(self):
        import io
        from contextlib import redirect_stdout

        f = io.StringIO()
        with redirect_stdout(f):
            wl.advanced_physics_status()
        out = f.getvalue()
        self.assertIn("INSECT FLIGHT BIOMECHANICS SUITE (H-89 to H-95)", out)
        self.assertIn("H-89 Dynamic Kramer Lift Burst", out)
        self.assertIn("H-90 Trapped-Vortex Cavity Control", out)
        self.assertIn("H-91 Lepidopteran Clap Jet", out)
        self.assertIn("H-92 Coleopteran Elytron Canard", out)
        self.assertIn("H-93 Dipteran Coriolis Preemption", out)
        self.assertIn("H-94 Orthopteran Torsional Wave", out)
        self.assertIn("H-95 Tandem Slotted Fowler Flap", out)


if __name__ == '__main__':
    unittest.main()
