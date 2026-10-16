"""
Unit Tests: Pterosaur Flight Biomechanics Suite (H-98, H-100 to H-102)
======================================================================
Tests coverage for:
  - H-98:  Quadrupedal Ground-Coupled Vault Takeoff (Pterosauria / Habib 2008)
  - H-100: Tailless Metachronal Differential Sweep Directional Trim (Pterodactyloidea)
  - H-101: Dynamic Flap-Inertia Pitch Counterbalance (Pteranodontia / Tapejaridae)
  - H-102: Deep-Camber High-Aspect-Ratio Azhdarchid Thermal Soaring Loiter (Azhdarchidae)

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
import io
from contextlib import redirect_stdout
import unittest

# Ensure firmware directory is in python path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'firmware'))

import mco_wave_library as wl
import mco_intent_control as ic


class TestPterosaurSuiteRegistration(unittest.TestCase):
    """Test hypothesis registration and metadata for H-98, H-100, H-101, H-102."""

    def test_hypotheses_registered_in_adv_new_hyps(self):
        adv_ids = {h['id'] for h in wl._ADV_NEW_HYPS}
        for hid in ('H-98', 'H-100', 'H-101', 'H-102'):
            self.assertIn(hid, adv_ids, f"{hid} missing from _ADV_NEW_HYPS")

    def test_hypotheses_registered_in_aero_hypotheses(self):
        aero_ids = {h['id'] for h in wl.AERO_HYPOTHESES}
        for hid in ('H-98', 'H-100', 'H-101', 'H-102'):
            self.assertIn(hid, aero_ids, f"{hid} missing from AERO_HYPOTHESES")

    def test_sg_tasks_metrics(self):
        expected_metrics = [
            'vault_takeoff_vertical_accel_g',
            'differential_sweep_yaw_drift_deg',
            'pitch_servo_current_reduction_pct',
            'azhdarchid_soar_sink_rate_ms',
        ]
        for metric in expected_metrics:
            self.assertIn(metric, wl._SG_TASKS, f"{metric} missing from _SG_TASKS")
            task = wl._SG_TASKS[metric]
            self.assertIn('axis', task)
            self.assertIn('sensor', task)
            self.assertIn('direction', task)

    def test_alss_curriculum_enrolment(self):
        self.assertIn('h98_vault_takeoff_a', wl._ALSS_CURRICULUM['A'],
                      "h98_vault_takeoff_a should be in Curriculum A (launch/takeoff)")
        self.assertIn('h100_differential_sweep_b', wl._ALSS_CURRICULUM['B'],
                      "h100_differential_sweep_b should be in Curriculum B (handling/trim)")
        self.assertIn('h101_cranial_pitch_b', wl._ALSS_CURRICULUM['B'],
                      "h101_cranial_pitch_b should be in Curriculum B (resilience/pitch)")
        self.assertIn('h102_azhdarchid_soar_b', wl._ALSS_CURRICULUM['B'],
                      "h102_azhdarchid_soar_b should be in Curriculum B (endurance/soar)")

    def test_phase_to_sg_task_mapping(self):
        self.assertIn('pterosaur_biomechanics', wl._PHASE_TO_SG_TASK)
        tasks = wl._PHASE_TO_SG_TASK['pterosaur_biomechanics']
        self.assertIn('vault_takeoff_vertical_accel_g', tasks)
        self.assertIn('differential_sweep_yaw_drift_deg', tasks)
        self.assertIn('pitch_servo_current_reduction_pct', tasks)
        self.assertIn('azhdarchid_soar_sink_rate_ms', tasks)

        # Also check alias
        self.assertIn('pterosaur_software_suite', wl._PHASE_TO_SG_TASK)
        tasks_alias = wl._PHASE_TO_SG_TASK['pterosaur_software_suite']
        self.assertEqual(tasks, tasks_alias)

    def test_discovery_bounds(self):
        expected_param_keys = [
            'h98_vault_accel_g', 'h98_vault_duration_s',
            'h100_max_sweep_bias_deg', 'h100_yaw_hold_deg',
            'h101_current_reduction_pct',
            'h102_min_sink_ms', 'h102_endurance_boost_pct',
        ]
        for key in expected_param_keys:
            self.assertIn(key, wl._EXPANDED_DISCOVERY_BOUNDS,
                          f"{key} missing from _EXPANDED_DISCOVERY_BOUNDS")
            lo, hi = wl._EXPANDED_DISCOVERY_BOUNDS[key]
            self.assertLess(lo, hi, f"Bound min ({lo}) should be less than max ({hi}) for {key}")

    def test_cmts_champions(self):
        expected_champions = {
            'quadrupedal_vault_takeoff': 'H-98',
            'pterodactyloid_differential_sweep': 'H-100',
            'cranial_crest_pitch_balance': 'H-101',
            'azhdarchid_thermal_loiter': 'H-102',
        }
        for champ_name, expected_hid in expected_champions.items():
            self.assertIn(champ_name, wl._CMTS_CHAMPIONS, f"{champ_name} missing from _CMTS_CHAMPIONS")
            self.assertEqual(wl._CMTS_CHAMPIONS[champ_name].get('hyp_id'), expected_hid)


class TestPterosaurObserverTriggers(unittest.TestCase):
    """Test AI observer triggers for queuing H-98, H-100, H-101, H-102."""

    def setUp(self):
        wl._ALC_SHARED['priority_hyp_queue'].clear()
        wl._ALC_SHARED['alert_log'].clear()

    def test_observer_trigger_h98_vault(self):
        sensors = {'on_ground': True, 'lever_pct': 75.0, 'V_ms': 0.0}
        obs = wl.ai_observer_tick(sensors, {})
        self.assertTrue(obs.get('h98_vault_takeoff_active'))
        queued_ids = [q['hyp_id'] for q in wl._ALC_SHARED['priority_hyp_queue']]
        self.assertIn('H-98', queued_ids)

    def test_observer_trigger_h100_differential_sweep(self):
        sensors = {'V_ms': 7.5, 'sideslip_beta_deg': 3.8}
        obs = wl.ai_observer_tick(sensors, {})
        self.assertTrue(obs.get('h100_differential_sweep_active'))
        queued_ids = [q['hyp_id'] for q in wl._ALC_SHARED['priority_hyp_queue']]
        self.assertIn('H-100', queued_ids)

    def test_observer_trigger_h101_pitch_counterbalance(self):
        sensors = {'pitch_servo_current_a': 4.6}
        obs = wl.ai_observer_tick(sensors, {})
        self.assertTrue(obs.get('h101_pitch_counterbalance_active'))
        queued_ids = [q['hyp_id'] for q in wl._ALC_SHARED['priority_hyp_queue']]
        self.assertIn('H-101', queued_ids)

    def test_observer_trigger_h102_azhdarchid_soar(self):
        sensors = {'lever_pct': 32.0, 'V_ms': 8.0, 'thermal_loiter': True}
        obs = wl.ai_observer_tick(sensors, {})
        self.assertTrue(obs.get('h102_azhdarchid_soar_active'))
        queued_ids = [q['hyp_id'] for q in wl._ALC_SHARED['priority_hyp_queue']]
        self.assertIn('H-102', queued_ids)


class TestPterosaurDiscoveryHypothesisSelect(unittest.TestCase):
    """Test focus keyword and sensor routing in discovery_hypothesis_select."""

    def test_focus_keyword_pterosaur(self):
        h = wl.discovery_hypothesis_select(focus='pterosaur')
        self.assertIsNotNone(h)
        self.assertEqual(h['id'], 'H-98')

    def test_focus_keyword_quadrupedal_vault(self):
        h = wl.discovery_hypothesis_select(focus='quadrupedal_vault')
        self.assertIsNotNone(h)
        self.assertEqual(h['id'], 'H-98')

    def test_focus_keyword_differential_sweep(self):
        h = wl.discovery_hypothesis_select(focus='differential_sweep')
        self.assertIsNotNone(h)
        self.assertEqual(h['id'], 'H-100')

    def test_focus_keyword_tailless_trim(self):
        h = wl.discovery_hypothesis_select(focus='tailless_trim')
        self.assertIsNotNone(h)
        self.assertEqual(h['id'], 'H-100')

    def test_focus_keyword_cranial_crest(self):
        h = wl.discovery_hypothesis_select(focus='cranial_crest')
        self.assertIsNotNone(h)
        self.assertEqual(h['id'], 'H-101')

    def test_focus_keyword_pitch_counterbalance(self):
        h = wl.discovery_hypothesis_select(focus='pitch_counterbalance')
        self.assertIsNotNone(h)
        self.assertEqual(h['id'], 'H-101')

    def test_focus_keyword_azhdarchid(self):
        h = wl.discovery_hypothesis_select(focus='azhdarchid')
        self.assertIsNotNone(h)
        self.assertEqual(h['id'], 'H-102')

    def test_focus_keyword_thermal_loiter(self):
        h = wl.discovery_hypothesis_select(focus='thermal_loiter')
        self.assertIsNotNone(h)
        self.assertEqual(h['id'], 'H-102')

    def test_sensor_routing_vault_takeoff(self):
        sensors = {'on_ground': True, 'lever_pct': 70.0, 'V_ms': 0.0}
        h = wl.discovery_hypothesis_select(sensors=sensors)
        self.assertIsNotNone(h)
        self.assertEqual(h['id'], 'H-98')

    def test_sensor_routing_differential_sweep(self):
        sensors = {'sideslip_beta_deg': 4.0, 'V_ms': 8.0, 'lever_pct': 50.0}
        h = wl.discovery_hypothesis_select(sensors=sensors)
        self.assertIsNotNone(h)
        self.assertEqual(h['id'], 'H-100')

    def test_sensor_routing_pitch_counterbalance(self):
        sensors = {'pitch_servo_current_a': 4.8, 'V_ms': 8.0, 'lever_pct': 50.0}
        h = wl.discovery_hypothesis_select(sensors=sensors)
        self.assertIsNotNone(h)
        self.assertEqual(h['id'], 'H-101')

    def test_sensor_routing_azhdarchid_thermal_soar(self):
        sensors = {'lever_pct': 32.0, 'V_ms': 8.0, 'thermal_loiter': True}
        h = wl.discovery_hypothesis_select(sensors=sensors)
        self.assertIsNotNone(h)
        self.assertEqual(h['id'], 'H-102')


class TestPterosaurGenomeEvolution(unittest.TestCase):
    """Test genome routing for H-98, H-100, H-101, H-102."""

    def test_genome_routing_execution(self):
        try:
            for hid in ('H-98', 'H-100', 'H-101', 'H-102'):
                wl._update_genome_from_trial(hid, 1.25, {'V_ms': 8.0})
        except Exception as e:
            self.fail(f"_update_genome_from_trial raised exception for pterosaur hypotheses: {e}")


class TestPterosaurIntentAndBehaviour(unittest.TestCase):
    """Test intent classification, behaviour execution, and macro aborts for pterosaur suite."""

    def setUp(self):
        self.interp = ic.IntentInterpreter()

    def test_quadrupedal_vault_intent_on_ground_demand(self):
        intent, conf, params = self.interp.interpret(
            {'yaw': 0.0, 'roll': 0.0, 'pitch': 0.0, 'throttle': 0.75},
            {},
            {'V_ms': 0.0, 'h_agl_m': 0.0, 'on_ground': True, 'vault_takeoff_demand': True}
        )
        self.assertEqual(intent, ic.Intent.QUADRUPEDAL_VAULT)
        self.assertAlmostEqual(params.get('vault_impulse_g'), 1.85)
        self.assertAlmostEqual(params.get('transition_timeout_s'), 0.42)

    def test_quadrupedal_vault_pilot_trigger_gesture(self):
        """Test full aft stick + full throttle + dual pedals depressed arms crouch,
        and rapid stick centering fires the quadrupedal vault strike.
        """
        interp = ic.IntentInterpreter()
        # Step 1: Pilot on ground holds dual pedals (brakes), full throttle (0.90), full aft stick (0.80)
        intent1, conf1, params1 = interp.interpret(
            {'yaw': 0.0, 'roll': 0.0, 'pitch': 0.80, 'throttle': 0.90, 'pedals_depressed': True},
            {},
            {'t': 0.0, 'V_ms': 0.0, 'h_agl_m': 0.0, 'on_ground': True}
        )
        self.assertEqual(intent1, ic.Intent.TAXI_FWD)
        self.assertTrue(interp._vault_armed, "Vault sequence should be armed")

        # Step 2: Pilot rapidly centers stick (0.02) at t = 0.20s (dt = 0.20s <= 0.45s)
        intent2, conf2, params2 = interp.interpret(
            {'yaw': 0.0, 'roll': 0.0, 'pitch': 0.02, 'throttle': 0.90, 'pedals_depressed': True},
            {},
            {'t': 0.20, 'V_ms': 0.0, 'h_agl_m': 0.0, 'on_ground': True}
        )
        self.assertEqual(intent2, ic.Intent.QUADRUPEDAL_VAULT)
        self.assertEqual(params2.get('trigger'), 'rapid_center')
        self.assertAlmostEqual(params2.get('vault_impulse_g'), 1.85)
        self.assertFalse(interp._vault_armed, "Armed flag should be reset upon trigger")

    def test_quadrupedal_vault_cancellation_throttle_backoff(self):
        """Test macro arming is cancelled if pilot backs off throttle before centering."""
        interp = ic.IntentInterpreter()
        # Arm
        interp.interpret(
            {'pitch': 0.80, 'throttle': 0.90, 'pedals_depressed': True},
            {},
            {'t': 0.0, 'on_ground': True, 'h_agl_m': 0.0}
        )
        self.assertTrue(interp._vault_armed)

        # Back off throttle (<0.80)
        interp.interpret(
            {'pitch': 0.80, 'throttle': 0.65, 'pedals_depressed': True},
            {},
            {'t': 0.10, 'on_ground': True, 'h_agl_m': 0.0}
        )
        self.assertFalse(interp._vault_armed, "Should disarm on throttle backoff")

        # Rapid center after disarm -> should NOT fire
        intent, _, _ = interp.interpret(
            {'pitch': 0.0, 'throttle': 0.65, 'pedals_depressed': True},
            {},
            {'t': 0.20, 'on_ground': True, 'h_agl_m': 0.0}
        )
        self.assertNotEqual(intent, ic.Intent.QUADRUPEDAL_VAULT)
        self.assertIn(intent, (ic.Intent.TAXI_FWD, ic.Intent.TAKEOFF_ROLL))

    def test_quadrupedal_vault_cancellation_pedal_release(self):
        """Test macro arming is cancelled if pilot releases rudder pedals."""
        interp = ic.IntentInterpreter()
        # Arm
        interp.interpret(
            {'pitch': 0.80, 'throttle': 0.90, 'pedals_depressed': True},
            {},
            {'t': 0.0, 'on_ground': True, 'h_agl_m': 0.0}
        )
        self.assertTrue(interp._vault_armed)

        # Release pedals
        interp.interpret(
            {'pitch': 0.80, 'throttle': 0.90, 'pedals_depressed': False},
            {},
            {'t': 0.10, 'on_ground': True, 'h_agl_m': 0.0}
        )
        self.assertFalse(interp._vault_armed, "Should disarm when pedals released")

        # Center stick -> should NOT fire
        intent, _, _ = interp.interpret(
            {'pitch': 0.0, 'throttle': 0.90, 'pedals_depressed': False},
            {},
            {'t': 0.20, 'on_ground': True, 'h_agl_m': 0.0}
        )
        self.assertNotEqual(intent, ic.Intent.QUADRUPEDAL_VAULT)
        self.assertIn(intent, (ic.Intent.TAXI_FWD, ic.Intent.TAKEOFF_ROLL))

    def test_quadrupedal_vault_cancellation_slow_centering(self):
        """Test macro arming is cancelled if stick is centered slowly (>0.45s)."""
        interp = ic.IntentInterpreter()
        # Arm
        interp.interpret(
            {'pitch': 0.80, 'throttle': 0.90, 'pedals_depressed': True},
            {},
            {'t': 0.0, 'on_ground': True, 'h_agl_m': 0.0}
        )
        self.assertTrue(interp._vault_armed)

        # Centering slowly at t = 0.50s (dt = 0.50s > 0.45s timeout)
        intent, _, _ = interp.interpret(
            {'pitch': 0.0, 'throttle': 0.90, 'pedals_depressed': True},
            {},
            {'t': 0.50, 'on_ground': True, 'h_agl_m': 0.0}
        )
        self.assertNotEqual(intent, ic.Intent.QUADRUPEDAL_VAULT)
        self.assertIn(intent, (ic.Intent.TAXI_FWD, ic.Intent.TAKEOFF_ROLL))
        self.assertFalse(interp._vault_armed)

    def test_quadrupedal_vault_cancellation_forward_or_lateral_stick(self):
        """Test forward stick dive push or lateral roll/yaw cancels arming."""
        # Forward dive stick
        interp1 = ic.IntentInterpreter()
        interp1.interpret(
            {'pitch': 0.80, 'throttle': 0.90, 'pedals_depressed': True},
            {},
            {'t': 0.0, 'on_ground': True, 'h_agl_m': 0.0}
        )
        self.assertTrue(interp1._vault_armed)
        interp1.interpret(
            {'pitch': -0.30, 'throttle': 0.90, 'pedals_depressed': True},
            {},
            {'t': 0.10, 'on_ground': True, 'h_agl_m': 0.0}
        )
        self.assertFalse(interp1._vault_armed, "Should disarm on forward dive stick push")

        # Lateral roll
        interp2 = ic.IntentInterpreter()
        interp2.interpret(
            {'pitch': 0.80, 'throttle': 0.90, 'pedals_depressed': True},
            {},
            {'t': 0.0, 'on_ground': True, 'h_agl_m': 0.0}
        )
        self.assertTrue(interp2._vault_armed)
        interp2.interpret(
            {'pitch': 0.40, 'roll': 0.35, 'throttle': 0.90, 'pedals_depressed': True},
            {},
            {'t': 0.10, 'on_ground': True, 'h_agl_m': 0.0}
        )
        self.assertFalse(interp2._vault_armed, "Should disarm on lateral roll deflection")

    def test_differential_sweep_trim_intent(self):
        intent, conf, params = self.interp.interpret(
            {'yaw': 0.0, 'roll': 0.0, 'pitch': 0.0, 'throttle': 0.50},
            {},
            {'V_ms': 8.0, 'h_agl_m': 30.0, 'on_ground': False, 'sideslip_beta_deg': 3.5}
        )
        self.assertEqual(intent, ic.Intent.DIFFERENTIAL_SWEEP_TRIM)
        self.assertAlmostEqual(params.get('differential_sweep_deg'), 12.0)
        self.assertAlmostEqual(params.get('yaw_hold_deg'), 0.5)

    def test_cranial_pitch_balance_intent(self):
        intent, conf, params = self.interp.interpret(
            {'yaw': 0.0, 'roll': 0.0, 'pitch': 0.0, 'throttle': 0.50},
            {},
            {'V_ms': 8.0, 'h_agl_m': 30.0, 'on_ground': False, 'pitch_servo_current_a': 4.6}
        )
        self.assertEqual(intent, ic.Intent.CRANIAL_PITCH_BALANCE)
        self.assertAlmostEqual(params.get('current_reduction_pct'), 42.0)

    def test_azhdarchid_soar_intent(self):
        intent, conf, params = self.interp.interpret(
            {'yaw': 0.0, 'roll': 0.0, 'pitch': 0.0, 'throttle': 0.35},
            {},
            {'V_ms': 8.0, 'h_agl_m': 40.0, 'on_ground': False, 'thermal_loiter': True}
        )
        self.assertEqual(intent, ic.Intent.AZHDARCHID_SOAR)
        self.assertAlmostEqual(params.get('min_sink_ms'), 0.38)
        self.assertAlmostEqual(params.get('endurance_boost_pct'), 86.0)

    def test_behaviour_dispatcher_quadrupedal_vault(self):
        class MockCtrl:
            freq_hz = 1.0
            _tgt_freq = 1.0
            _tgt_sweeps = [60.0] * 8
            _tgt_dihedrals = [0.0] * 8
            _tgt_pitches = [0.0] * 8

        ctrl = MockCtrl()
        disp = ic.BehaviourDispatcher(ctrl)

        # Arm via aft stick + full throttle + pedals
        disp.update(
            0.0,
            {'pitch': 0.80, 'throttle': 0.90, 'roll': 0.0, 'yaw': 0.0, 'pedals_depressed': True},
            {},
            {'V_ms': 0.0, 'h_agl_m': 0.0, 'on_ground': True}
        )

        # Rapid centering at t = 0.05 triggers vault Phase 1 crouch
        name, intent, status = disp.update(
            0.05,
            {'pitch': 0.0, 'throttle': 0.90, 'roll': 0.0, 'yaw': 0.0, 'pedals_depressed': True},
            {},
            {'V_ms': 0.0, 'h_agl_m': 0.0, 'on_ground': True}
        )
        self.assertEqual(name, 'quadrupedal_vault')
        self.assertEqual(intent, ic.Intent.QUADRUPEDAL_VAULT)
        self.assertEqual(status, ic.RUNNING)
        # Phase 1: Forward sweep +20 deg pitch crouch
        self.assertEqual(ctrl._tgt_sweeps[:2], [35.0, 35.0])
        self.assertEqual(ctrl._tgt_pitches[:2], [20.0, 20.0])

        # Step into phase 2 (dt > 0.15s)
        name2, intent2, status2 = disp.update(
            0.20,
            {'pitch': 0.0, 'throttle': 0.90, 'roll': 0.0, 'yaw': 0.0, 'pedals_depressed': True},
            {},
            {'V_ms': 2.0, 'h_agl_m': 0.3, 'on_ground': False}
        )
        self.assertEqual(name2, 'quadrupedal_vault')
        self.assertEqual(ctrl._tgt_sweeps[:2], [70.0, 70.0])
        self.assertEqual(ctrl._tgt_pitches[:2], [12.0, 12.0])

    def test_behaviour_dispatcher_differential_sweep(self):
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
            {'pitch': 0.0, 'throttle': 0.50, 'roll': 0.0, 'yaw': 0.0},
            {},
            {'V_ms': 8.0, 'h_agl_m': 30.0, 'on_ground': False, 'sideslip_beta_deg': 4.0}
        )
        self.assertEqual(name, 'differential_sweep_trim')
        self.assertEqual(intent, ic.Intent.DIFFERENTIAL_SWEEP_TRIM)
        self.assertEqual(status, ic.RUNNING)
        # Port (even) vs Stbd (odd) asymmetric sweep
        self.assertNotEqual(ctrl._tgt_sweeps[0], ctrl._tgt_sweeps[1])

    def test_behaviour_dispatcher_azhdarchid_soar(self):
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
            {'pitch': 0.0, 'throttle': 0.35, 'roll': 0.0, 'yaw': 0.0},
            {},
            {'V_ms': 8.0, 'h_agl_m': 30.0, 'on_ground': False, 'thermal_loiter': True}
        )
        self.assertEqual(name, 'azhdarchid_soar')
        self.assertEqual(intent, ic.Intent.AZHDARCHID_SOAR)
        self.assertEqual(status, ic.RUNNING)
        # Slotted box wing lock: Stations 1 & 3 locked at 8.5°, Stations 2 & 4 drooped at 14.0°
        self.assertEqual(ctrl._tgt_pitches, [8.5, 8.5, 14.0, 14.0, 8.5, 8.5, 14.0, 14.0])
        self.assertAlmostEqual(ctrl._tgt_freq, 0.7)

    def test_quadrupedal_vault_macro_aborts(self):
        aborted_ceil, reason_c = ic.check_macro_abort('quadrupedal_vault', {'h_agl_m': 2.0, 'pitch_deg': 5.0})
        self.assertTrue(aborted_ceil)
        self.assertIn('altitude ceiling', reason_c)

        aborted_pitch, reason_p = ic.check_macro_abort('quadrupedal_vault', {'h_agl_m': 0.5, 'pitch_deg': 38.0})
        self.assertTrue(aborted_pitch)
        self.assertIn('excessive pitch attitude', reason_p)

        aborted_spd, reason_s = ic.check_macro_abort('quadrupedal_vault', {'h_agl_m': 0.5, 'V_ms': 14.0})
        self.assertTrue(aborted_spd)
        self.assertIn('speed too high', reason_s)

        # Abort if rudder pedals released on ground
        aborted_pedal, reason_pedal = ic.check_macro_abort('quadrupedal_vault', {'on_ground': True, 'pedals_depressed': False})
        self.assertTrue(aborted_pedal)
        self.assertIn('rudder pedals released', reason_pedal)

        # Abort if throttle backed off on ground
        aborted_thr, reason_thr = ic.check_macro_abort('quadrupedal_vault', {'on_ground': True, 'throttle': 0.50})
        self.assertTrue(aborted_thr)
        self.assertIn('throttle backed off', reason_thr)

        # Airborne: pedal release or throttle adjust should not abort vault
        aborted_air, _ = ic.check_macro_abort('quadrupedal_vault', {'on_ground': False, 'pedals_depressed': False, 'throttle': 0.50, 'h_agl_m': 0.8, 'pitch_deg': 15.0, 'V_ms': 4.0})
        self.assertFalse(aborted_air)


class TestPterosaurGroundStationDiagnostics(unittest.TestCase):
    """Verify advanced_physics_status diagnostic output execution with pterosaur suite."""

    def test_advanced_physics_status_runs_with_pterosaurs(self):
        f = io.StringIO()
        with redirect_stdout(f):
            wl.advanced_physics_status()
        out = f.getvalue()
        self.assertIn("PTEROSAUR FLIGHT BIOMECHANICS SUITE (H-98, H-100 to H-102)", out)
        self.assertIn("H-98 Quadrupedal Vault Takeoff", out)
        self.assertIn("H-100 Tailless Differential Sweep", out)
        self.assertIn("H-101 Flap-Inertia Counterbalance", out)
        self.assertIn("H-102 Azhdarchid Thermal Soaring", out)


if __name__ == '__main__':
    unittest.main()
