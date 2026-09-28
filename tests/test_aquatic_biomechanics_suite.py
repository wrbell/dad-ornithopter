"""
Unit Tests: Aquatic Biomechanics & Hydrodynamic Biomimicry Suite (H-103 to H-108)
================================================================================
Tests complete coverage for:
  - H-103: Sea Turtle In-Stroke Sweep & Differential Flipper Pivot Braking (Caretta caretta / Chelonia mydas)
  - H-104: Plesiosaur Speed-Adaptive Tandem Wake Capture & Collective Pitch Trimming (Sauropterygia)
  - H-105: Cetacean Pectoral 3-DOF Hydrofoil Maneuvering & Fluke Recoil Damping (Megaptera novaeangliae / Tursiops truncatus)
  - H-106: Crustacean Adlocomotory Metachronal Jet Coalescence & Vector Tuning (Euphausia superba)
  - H-107: Copepod Dual-Mode Asymmetric Metachrony (Sequential Power Downstroke x Synchronous Recovery Upstroke) (Calanus finmarchicus)
  - H-108: Decapod Caridoid Impulse Flare & Escape Pulse (Homarus americanus / Procambarus clarkii)

Tests verify:
  1. Hypothesis definitions and metadata in AERO_HYPOTHESES and _ADV_NEW_HYPS
  2. Scoring metrics in _SG_TASKS
  3. ALSS segments and curriculum enrolment in Curricula A & B and _PHASE_TO_SG_TASK
  4. Expanded discovery bounds in _EXPANDED_DISCOVERY_BOUNDS
  5. Context champions in _CMTS_CHAMPIONS
  6. AI observer trigger conditions in ai_observer_tick (triggers 28-33)
  7. Keyword and sensor routing in discovery_hypothesis_select
  8. Hydrodynamic simulation models in mco_advanced_physics
  9. Intent interpretation and behaviour execution in mco_intent_control
  10. Genome evolution weight vector routing in _update_genome_from_trial
  11. Ground station diagnostic command advanced_physics_status
"""

import sys
import os
import io
from contextlib import redirect_stdout
import unittest

# Ensure firmware directory is in python path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'firmware'))

import mco_wave_library as wl
import mco_advanced_physics as ap
import mco_intent_control as ic


class TestAquaticSuiteRegistration(unittest.TestCase):
    """Test hypothesis registration and metadata for H-103 through H-108."""

    def test_hypotheses_registered_in_adv_new_hyps(self):
        adv_ids = {h['id'] for h in wl._ADV_NEW_HYPS}
        for hid in ('H-103', 'H-104', 'H-105', 'H-106', 'H-107', 'H-108'):
            self.assertIn(hid, adv_ids, f"{hid} missing from _ADV_NEW_HYPS")

    def test_hypotheses_registered_in_aero_hypotheses(self):
        aero_ids = {h['id'] for h in wl.AERO_HYPOTHESES}
        for hid in ('H-103', 'H-104', 'H-105', 'H-106', 'H-107', 'H-108'):
            self.assertIn(hid, aero_ids, f"{hid} missing from AERO_HYPOTHESES")

    def test_sg_tasks_metrics(self):
        expected_metrics = [
            'turtle_pivot_turn_radius_m',
            'plesiosaur_tandem_thrust_boost_pct',
            'cetacean_roll_rate_degs',
            'krill_jet_coalescence_thrust_boost_pct',
            'copepod_power_smoothing_ripple_pct',
            'caridoid_escape_impulse_n',
        ]
        for metric in expected_metrics:
            self.assertIn(metric, wl._SG_TASKS, f"{metric} missing from _SG_TASKS")
            task = wl._SG_TASKS[metric]
            self.assertIn('axis', task)
            self.assertIn('sensor', task)
            self.assertIn('direction', task)

    def test_alss_curriculum_enrolment(self):
        # Curriculum A: burst, maneuver, safety
        self.assertIn('h103_turtle_pivot_a', wl._ALSS_CURRICULUM['A'],
                      "h103_turtle_pivot_a should be in Curriculum A (maneuver/pivot brake)")
        self.assertIn('h107_copepod_metachrony_a', wl._ALSS_CURRICULUM['A'],
                      "h107_copepod_metachrony_a should be in Curriculum A (power smoothing)")
        self.assertIn('h108_caridoid_flare_a', wl._ALSS_CURRICULUM['A'],
                      "h108_caridoid_flare_a should be in Curriculum A (emergency escape/flare)")

        # Curriculum B: cruise, efficiency, trim
        self.assertIn('h104_plesiosaur_wake_b', wl._ALSS_CURRICULUM['B'],
                      "h104_plesiosaur_wake_b should be in Curriculum B (wake capture/cruise)")
        self.assertIn('h105_cetacean_roll_b', wl._ALSS_CURRICULUM['B'],
                      "h105_cetacean_roll_b should be in Curriculum B (hydrofoil roll/agility)")
        self.assertIn('h106_krill_jet_b', wl._ALSS_CURRICULUM['B'],
                      "h106_krill_jet_b should be in Curriculum B (coalesced jet propulsion)")

    def test_phase_to_sg_task_mapping(self):
        self.assertIn('aquatic_biomechanics', wl._PHASE_TO_SG_TASK)
        tasks = wl._PHASE_TO_SG_TASK['aquatic_biomechanics']
        self.assertIn('turtle_pivot_turn_radius_m', tasks)
        self.assertIn('plesiosaur_tandem_thrust_boost_pct', tasks)
        self.assertIn('cetacean_roll_rate_degs', tasks)
        self.assertIn('krill_jet_coalescence_thrust_boost_pct', tasks)
        self.assertIn('copepod_power_smoothing_ripple_pct', tasks)
        self.assertIn('caridoid_escape_impulse_n', tasks)

        # Alias verification
        self.assertIn('aquatic_software_suite', wl._PHASE_TO_SG_TASK)
        tasks_alias = wl._PHASE_TO_SG_TASK['aquatic_software_suite']
        self.assertEqual(tasks, tasks_alias)

    def test_discovery_bounds(self):
        expected_param_keys = [
            'h103_pivot_sweep_deg', 'h103_pivot_turn_radius_m',
            'h104_phase_advance_deg', 'h104_thrust_boost_pct',
            'h105_diff_aoa_deg', 'h105_roll_rate_degs',
            'h106_phase_lag_deg', 'h106_thrust_boost_pct',
            'h107_ripple_reduction_pct',
            'h108_impulse_n',
        ]
        for key in expected_param_keys:
            self.assertIn(key, wl._EXPANDED_DISCOVERY_BOUNDS,
                          f"{key} missing from _EXPANDED_DISCOVERY_BOUNDS")
            lo, hi = wl._EXPANDED_DISCOVERY_BOUNDS[key]
            self.assertLess(lo, hi, f"Bound min ({lo}) should be less than max ({hi}) for {key}")

    def test_cmts_champions(self):
        expected_champions = {
            'turtle_pivot_brake': 'H-103',
            'plesiosaur_wake_capture': 'H-104',
            'cetacean_roll_recoil': 'H-105',
            'krill_jet_coalescence': 'H-106',
            'copepod_asymmetric_metachrony': 'H-107',
            'caridoid_escape_flare': 'H-108',
        }
        for champ_name, expected_hid in expected_champions.items():
            self.assertIn(champ_name, wl._CMTS_CHAMPIONS, f"{champ_name} missing from _CMTS_CHAMPIONS")
            self.assertEqual(wl._CMTS_CHAMPIONS[champ_name].get('hyp_id'), expected_hid)


class TestAquaticObserverTriggers(unittest.TestCase):
    """Test AI observer triggers 28-33 for H-103 through H-108."""

    def setUp(self):
        wl._ALC_SHARED['priority_hyp_queue'].clear()
        wl._ALC_SHARED['alert_log'].clear()

    def test_observer_trigger_h103_turtle_pivot(self):
        sensors = {'V_ms': 4.0, 'yaw_stick': 0.50}
        obs = wl.ai_observer_tick(sensors, {})
        self.assertTrue(obs.get('h103_turtle_pivot_active'))
        queued_ids = [q['hyp_id'] for q in wl._ALC_SHARED['priority_hyp_queue']]
        self.assertIn('H-103', queued_ids)

    def test_observer_trigger_h104_plesiosaur_wake(self):
        sensors = {'plesiosaur_wake': True}
        obs = wl.ai_observer_tick(sensors, {})
        self.assertTrue(obs.get('h104_plesiosaur_wake_active'))
        queued_ids = [q['hyp_id'] for q in wl._ALC_SHARED['priority_hyp_queue']]
        self.assertIn('H-104', queued_ids)

    def test_observer_trigger_h105_cetacean_roll(self):
        sensors = {'cetacean_roll': True}
        obs = wl.ai_observer_tick(sensors, {})
        self.assertTrue(obs.get('h105_cetacean_roll_recoil_active'))
        queued_ids = [q['hyp_id'] for q in wl._ALC_SHARED['priority_hyp_queue']]
        self.assertIn('H-105', queued_ids)

    def test_observer_trigger_h106_krill_jet(self):
        sensors = {'krill_jet': True}
        obs = wl.ai_observer_tick(sensors, {})
        self.assertTrue(obs.get('h106_krill_jet_active'))
        queued_ids = [q['hyp_id'] for q in wl._ALC_SHARED['priority_hyp_queue']]
        self.assertIn('H-106', queued_ids)

    def test_observer_trigger_h107_copepod_metachrony(self):
        sensors = {'bus_ripple_high': True}
        obs = wl.ai_observer_tick(sensors, {})
        self.assertTrue(obs.get('h107_copepod_metachrony_active'))
        queued_ids = [q['hyp_id'] for q in wl._ALC_SHARED['priority_hyp_queue']]
        self.assertIn('H-107', queued_ids)

    def test_observer_trigger_h108_caridoid_flare(self):
        sensors = {'emergency_flare': True}
        obs = wl.ai_observer_tick(sensors, {})
        self.assertTrue(obs.get('h108_caridoid_flare_active'))
        queued_ids = [q['hyp_id'] for q in wl._ALC_SHARED['priority_hyp_queue']]
        self.assertIn('H-108', queued_ids)


class TestAquaticDiscoveryHypothesisSelect(unittest.TestCase):
    """Test focus keyword and sensor routing in discovery_hypothesis_select."""

    def test_focus_keyword_aquatic(self):
        h = wl.discovery_hypothesis_select(focus='aquatic')
        self.assertIsNotNone(h)
        self.assertEqual(h['id'], 'H-103')

    def test_focus_keyword_turtle(self):
        h = wl.discovery_hypothesis_select(focus='turtle')
        self.assertIsNotNone(h)
        self.assertEqual(h['id'], 'H-103')

    def test_focus_keyword_plesiosaur(self):
        h = wl.discovery_hypothesis_select(focus='plesiosaur')
        self.assertIsNotNone(h)
        self.assertEqual(h['id'], 'H-104')

    def test_focus_keyword_cetacean(self):
        h = wl.discovery_hypothesis_select(focus='cetacean')
        self.assertIsNotNone(h)
        self.assertEqual(h['id'], 'H-105')

    def test_focus_keyword_krill(self):
        h = wl.discovery_hypothesis_select(focus='krill')
        self.assertIsNotNone(h)
        self.assertEqual(h['id'], 'H-106')

    def test_focus_keyword_copepod(self):
        h = wl.discovery_hypothesis_select(focus='copepod')
        self.assertIsNotNone(h)
        self.assertEqual(h['id'], 'H-107')

    def test_focus_keyword_caridoid(self):
        h = wl.discovery_hypothesis_select(focus='caridoid')
        self.assertIsNotNone(h)
        self.assertEqual(h['id'], 'H-108')

    def test_sensor_routing_caridoid_escape_flare(self):
        sensors = {'emergency_flare': True, 'V_ms': 8.0, 'lever_pct': 50.0}
        h = wl.discovery_hypothesis_select(sensors=sensors)
        self.assertIsNotNone(h)
        self.assertEqual(h['id'], 'H-108')

    def test_sensor_routing_turtle_pivot_brake(self):
        sensors = {'pivot_brake': True, 'V_ms': 6.0, 'lever_pct': 40.0}
        h = wl.discovery_hypothesis_select(sensors=sensors)
        self.assertIsNotNone(h)
        self.assertEqual(h['id'], 'H-103')

    def test_sensor_routing_cetacean_roll(self):
        sensors = {'heave_oscillation_rms': 0.45, 'V_ms': 9.0, 'lever_pct': 50.0}
        h = wl.discovery_hypothesis_select(sensors=sensors)
        self.assertIsNotNone(h)
        self.assertEqual(h['id'], 'H-105')

    def test_sensor_routing_plesiosaur_wake(self):
        sensors = {'wake_capture': True, 'V_ms': 9.0, 'lever_pct': 50.0}
        h = wl.discovery_hypothesis_select(sensors=sensors)
        self.assertIsNotNone(h)
        self.assertEqual(h['id'], 'H-104')

    def test_sensor_routing_krill_jet(self):
        sensors = {'jet_coalescence': True, 'V_ms': 12.0, 'lever_pct': 70.0}
        h = wl.discovery_hypothesis_select(sensors=sensors)
        self.assertIsNotNone(h)
        self.assertEqual(h['id'], 'H-106')

    def test_sensor_routing_copepod_power_smoothing(self):
        sensors = {'torque_ripple_pct': 35.0, 'V_ms': 8.0, 'lever_pct': 50.0}
        h = wl.discovery_hypothesis_select(sensors=sensors)
        self.assertIsNotNone(h)
        self.assertEqual(h['id'], 'H-107')


class TestAquaticAdvancedPhysicsModels(unittest.TestCase):
    """Test hydrodynamic simulation functions in mco_advanced_physics."""

    def test_h103_sea_turtle_physics(self):
        cmds, yaw_rate, turn_r, diag = ap.h103_sea_turtle_pivot_brake(V_ms=6.0, yaw_rate_req_degs=45.0)
        self.assertEqual(len(cmds), 8)
        self.assertGreater(yaw_rate, 20.0)
        self.assertEqual(diag['hypothesis'], 'H-103')

        # Also verify low speed tight radius capability
        cmds_lo, yaw_rate_lo, turn_r_lo, diag_lo = ap.h103_sea_turtle_pivot_brake(V_ms=2.0, yaw_rate_req_degs=90.0)
        self.assertGreater(yaw_rate_lo, 10.0)
        self.assertLess(turn_r_lo, 10.0)

    def test_h104_plesiosaur_physics(self):
        cmds, thrust_boost, pitch_trim, diag = ap.h104_plesiosaur_tandem_wake_capture(V_ms=9.0, freq_hz=2.6)
        self.assertEqual(len(cmds), 8)
        self.assertGreater(thrust_boost, 20.0)
        self.assertEqual(diag['hypothesis'], 'H-104')

    def test_h105_cetacean_physics(self):
        cmds, roll_rate, jitter_atten, diag = ap.h105_cetacean_roll_and_recoil_damping(
            roll_rate_cmd_degs=180.0, V_ms=9.0, heave_accel_ms2=0.5
        )
        self.assertEqual(len(cmds), 8)
        self.assertGreater(roll_rate, 100.0)
        self.assertGreater(jitter_atten, 40.0)
        self.assertEqual(diag['hypothesis'], 'H-105')

    def test_h106_krill_physics(self):
        cmds, jet_tilt, thrust_boost, diag = ap.h106_krill_metachronal_jet_coalescence(V_ms=8.5, freq_hz=2.8)
        self.assertEqual(len(cmds), 8)
        self.assertGreater(thrust_boost, 15.0)
        self.assertEqual(diag['hypothesis'], 'H-106')

    def test_h107_copepod_physics(self):
        cmds, ripple_red, smoothed, diag = ap.h107_copepod_asymmetric_metachrony(V_ms=9.0, freq_hz=3.0)
        self.assertEqual(len(cmds), 8)
        self.assertGreater(ripple_red, 40.0)
        self.assertTrue(smoothed)
        self.assertEqual(diag['hypothesis'], 'H-107')

    def test_h108_caridoid_physics(self):
        cmds, impulse_n, sink_arrested, diag = ap.h108_caridoid_escape_flare(armed=True, V_ms=8.0, vz_ms=-1.5)
        self.assertEqual(len(cmds), 8)
        self.assertGreater(impulse_n, 35.0)
        self.assertTrue(sink_arrested)
        self.assertEqual(diag['hypothesis'], 'H-108')


class TestAquaticIntentControlAndBehaviours(unittest.TestCase):
    """Test intent classification, behaviour execution, and registry in mco_intent_control."""

    def setUp(self):
        self.interp = ic.IntentInterpreter()
        class MockCtrl:
            freq_hz = 1.0
            _tgt_freq = 1.0
            _tgt_sweeps = [60.0] * 8
            _tgt_dihedrals = [0.0] * 8
            _tgt_pitches = [0.0] * 8
        self.ctrl = MockCtrl()

    def test_intent_caridoid_escape_flare(self):
        intent, conf, params = self.interp.interpret(
            {'yaw': 0.0, 'roll': 0.0, 'pitch': 0.0, 'throttle': 0.5},
            {},
            {'emergency_flare': True, 'collision_warning': True}
        )
        self.assertEqual(intent, ic.Intent.CARIDOID_ESCAPE_FLARE)
        self.assertAlmostEqual(params.get('escape_thrust_n'), 38.0)

    def test_intent_turtle_pivot_brake(self):
        intent, conf, params = self.interp.interpret(
            {'yaw': 0.0, 'roll': 0.0, 'pitch': 0.0, 'throttle': 0.5},
            {},
            {'pivot_brake_demand': True, 'on_ground': False}
        )
        self.assertEqual(intent, ic.Intent.TURTLE_PIVOT_BRAKE)
        self.assertAlmostEqual(params.get('turn_radius_m'), 0.65)

    def test_intent_cetacean_hydrofoil_roll(self):
        intent, conf, params = self.interp.interpret(
            {'yaw': 0.0, 'roll': 0.0, 'pitch': 0.0, 'throttle': 0.5},
            {},
            {'cetacean_roll_demand': True, 'on_ground': False}
        )
        self.assertEqual(intent, ic.Intent.CETACEAN_HYDROFOIL_ROLL)
        self.assertTrue(params.get('recoil_damping_active'))

    def test_intent_plesiosaur_tandem_wake(self):
        intent, conf, params = self.interp.interpret(
            {'yaw': 0.0, 'roll': 0.0, 'pitch': 0.0, 'throttle': 0.5},
            {},
            {'tandem_wake_demand': True, 'on_ground': False}
        )
        self.assertEqual(intent, ic.Intent.PLESIOSAUR_TANDEM_WAKE)
        self.assertAlmostEqual(params.get('thrust_boost_pct'), 22.5)

    def test_intent_krill_jet_coalescence(self):
        intent, conf, params = self.interp.interpret(
            {'yaw': 0.0, 'roll': 0.0, 'pitch': 0.0, 'throttle': 0.5},
            {},
            {'krill_jet_demand': True, 'on_ground': False}
        )
        self.assertEqual(intent, ic.Intent.KRILL_JET_COALESCENCE)
        self.assertAlmostEqual(params.get('phase_lag_deg'), 55.0)

    def test_intent_copepod_metachrony(self):
        intent, conf, params = self.interp.interpret(
            {'yaw': 0.0, 'roll': 0.0, 'pitch': 0.0, 'throttle': 0.5},
            {},
            {'copepod_metachrony_demand': True, 'on_ground': False}
        )
        self.assertEqual(intent, ic.Intent.COPEPOD_METACHRONY)
        self.assertAlmostEqual(params.get('ripple_reduction_pct'), 48.5)

    def test_behaviour_registry_mappings(self):
        expected_registry = {
            ic.Intent.TURTLE_PIVOT_BRAKE:      ic.TurtlePivotBrakeBehaviour,
            ic.Intent.PLESIOSAUR_TANDEM_WAKE:  ic.PlesiosaurTandemWakeBehaviour,
            ic.Intent.CETACEAN_HYDROFOIL_ROLL: ic.CetaceanHydrofoilRollBehaviour,
            ic.Intent.KRILL_JET_COALESCENCE:   ic.KrillJetCoalescenceBehaviour,
            ic.Intent.COPEPOD_METACHRONY:      ic.CopepodMetachronyBehaviour,
            ic.Intent.CARIDOID_ESCAPE_FLARE:   ic.CaridoidEscapeFlareMacro,
        }
        for intent_enum, expected_cls in expected_registry.items():
            self.assertIn(intent_enum, ic.BEHAVIOUR_REGISTRY)
            self.assertEqual(ic.BEHAVIOUR_REGISTRY[intent_enum], expected_cls)

    def test_behaviour_execution_tick(self):
        for intent_enum, b_cls in [
            (ic.Intent.TURTLE_PIVOT_BRAKE, ic.TurtlePivotBrakeBehaviour),
            (ic.Intent.PLESIOSAUR_TANDEM_WAKE, ic.PlesiosaurTandemWakeBehaviour),
            (ic.Intent.CETACEAN_HYDROFOIL_ROLL, ic.CetaceanHydrofoilRollBehaviour),
            (ic.Intent.KRILL_JET_COALESCENCE, ic.KrillJetCoalescenceBehaviour),
            (ic.Intent.COPEPOD_METACHRONY, ic.CopepodMetachronyBehaviour),
            (ic.Intent.CARIDOID_ESCAPE_FLARE, ic.CaridoidEscapeFlareMacro),
        ]:
            node = b_cls()
            node.on_enter({'t': 10.0})
            status = node.tick({'t': 10.05, 'direction': 'left'}, self.ctrl)
            self.assertIn(status, (ic.RUNNING, ic.SUCCESS), f"{b_cls.__name__} returned invalid status")


class TestAquaticGenomeEvolution(unittest.TestCase):
    """Test genome routing for H-103 through H-108."""

    def test_genome_routing_execution(self):
        try:
            for hid in ('H-103', 'H-104', 'H-105', 'H-106', 'H-107', 'H-108'):
                wl._update_genome_from_trial(hid, 1.25, {'V_ms': 8.0})
        except Exception as e:
            self.fail(f"_update_genome_from_trial raised exception for aquatic hypotheses: {e}")


class TestAquaticGroundStationDiagnostic(unittest.TestCase):
    """Test ground station diagnostic command advanced_physics_status."""

    def test_advanced_physics_status_runs_without_error(self):
        buf = io.StringIO()
        with redirect_stdout(buf):
            wl.advanced_physics_status()
        out = buf.getvalue()
        self.assertIn("AQUATIC BIOMECHANICS & HYDRODYNAMIC BIOMIMICRY SUITE (H-103 to H-108)", out)
        self.assertIn("H-103", out)
        self.assertIn("H-104", out)
        self.assertIn("H-105", out)
        self.assertIn("H-106", out)
        self.assertIn("H-107", out)
        self.assertIn("H-108", out)


if __name__ == '__main__':
    unittest.main()
