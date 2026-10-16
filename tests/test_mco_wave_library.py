"""
Unit & Integration Tests: MCO Wave Library & Adversarial Flight Software Subsystems
===================================================================================
Comprehensive verification suite covering:
  1. Phase-Aware Hypothesis Filtering & Mission Seeding (seed_hypotheses_for_mission)
  2. Hypothesis Propose & Deduplication Safeguards (hypothesis_propose_next, _config_distance)
  3. Expanded Intent Vocabulary & Aerodynamic AoA Mapping (PITCH_AOA_BY_INTENT)
  4. Active Flight Safety Governors Integration (stall floor, tandem strike, sprint clamp, regen fence)
  5. Module Import Resilience & Hostile Exit Prevention (mco_panel, mco_api_proxy)
  6. Airframe Physical Constants Synchronization (mco_flight_test_protocol)
"""

import os
import sys
import unittest
import math

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'firmware'))

import mco_wave_library as wl
from mco_flight_governor import (
    check_glide_speed_floor,
    check_tandem_blade_clearance,
    clamp_sprint_dynamics,
    govern_regen_brake_resistor,
    evaluate_proprioceptive_cascade_clearance,
    apply_proprioceptive_deconfliction,
)


class TestPhaseAwareHypothesisSeeding(unittest.TestCase):
    """Verifies phase-aware mission hypothesis filtering and prioritization."""

    @classmethod
    def setUpClass(cls):
        cls._orig_queue = list(wl._hypothesis_queue)

    @classmethod
    def tearDownClass(cls):
        wl._hypothesis_queue[:] = cls._orig_queue

    def test_seed_cruise_phase(self):
        """Verify cruise phase seeding only yields hypotheses applicable to cruise."""
        n = wl.seed_hypotheses_for_mission(flight_phase='cruise', V_ms=9.0)
        self.assertGreater(n, 0)
        for h in wl._hypothesis_queue:
            eval_phases = h.get('eval_phases', [])
            self.assertIn('cruise', eval_phases, f"Hypothesis {h['id']} missing 'cruise' in eval_phases: {eval_phases}")

    def test_seed_hover_phase(self):
        """Verify hover phase seeding only yields hypotheses applicable to hover."""
        n = wl.seed_hypotheses_for_mission(flight_phase='hover', V_ms=0.0)
        self.assertGreater(n, 0)
        for h in wl._hypothesis_queue:
            eval_phases = h.get('eval_phases', [])
            self.assertIn('hover', eval_phases, f"Hypothesis {h['id']} missing 'hover' in eval_phases: {eval_phases}")

    def test_seed_takeoff_climb_phase(self):
        """Verify takeoff/climb phase seeding."""
        n_takeoff = wl.seed_hypotheses_for_mission(flight_phase='takeoff', V_ms=5.0)
        self.assertGreater(n_takeoff, 0)
        for h in wl._hypothesis_queue:
            self.assertIn('takeoff', h.get('eval_phases', []))

    def test_seed_unknown_phase_fallback(self):
        """Verify unknown or unmapped phase safely falls back to all airspeed-applicable hypotheses."""
        n_fallback = wl.seed_hypotheses_for_mission(flight_phase='suborbital_boost', V_ms=9.0)
        self.assertGreater(n_fallback, 0)
        # Should contain general cruise hypotheses as safe fallback
        self.assertGreaterEqual(n_fallback, 5)

    def test_seed_hypotheses_airspeed_gating(self):
        """Verify airspeed filtering limits queued hypotheses to valid flight envelope."""
        wl.seed_hypotheses_for_mission(flight_phase=None, V_ms=1.0)
        low_speed_ids = [h['id'] for h in wl._hypothesis_queue]

        wl.seed_hypotheses_for_mission(flight_phase=None, V_ms=25.0)
        high_speed_ids = [h['id'] for h in wl._hypothesis_queue]

        # Low-speed and high-speed queues must differ based on aerodynamic regime
        self.assertNotEqual(low_speed_ids, high_speed_ids)

    def test_seed_priority_ordering(self):
        """Verify higher priority hypotheses (priority 1) appear before priority 2 or 3."""
        wl.seed_hypotheses_for_mission(flight_phase='cruise', V_ms=9.0)
        priorities = [h.get('priority', 2) for h in wl._hypothesis_queue]
        # Should be monotonically non-decreasing
        for i in range(len(priorities) - 1):
            self.assertLessEqual(priorities[i], priorities[i + 1])


class TestHypothesisDeduplicationAndConvergence(unittest.TestCase):
    """Verifies parameter distance checking and duplicate rejection safeguards."""

    @classmethod
    def setUpClass(cls):
        cls._orig_queue = list(wl._hypothesis_queue)
        cls._orig_eval_history = list(wl._hypothesis_eval_history)
        cls._orig_results = dict(wl._hypothesis_results)

    @classmethod
    def tearDownClass(cls):
        wl._hypothesis_queue[:] = cls._orig_queue
        wl._hypothesis_eval_history[:] = cls._orig_eval_history
        wl._hypothesis_results.clear()
        wl._hypothesis_results.update(cls._orig_results)

    def setUp(self):
        # Reset queue and history before each test
        wl._hypothesis_queue.clear()
        wl._hypothesis_eval_history.clear()
        wl._hypothesis_results.clear()

    def test_config_distance_metric(self):
        """Verify normalized Euclidean distance across wave configuration parameters."""
        cfg1 = {'freq_hz': 2.0, 'sweep_deg': 60.0, 'aoa_deg': 20.0, 'amp_port_deg': 96.0, 'cascade_phase_deg': 90.0}
        cfg2 = dict(cfg1)
        self.assertAlmostEqual(wl._config_distance(cfg1, cfg2), 0.0, places=5)

        # Perturbed config
        cfg3 = dict(cfg1)
        cfg3['freq_hz'] = 2.8
        dist = wl._config_distance(cfg1, cfg3)
        self.assertGreater(dist, 0.05)

    def test_deduplication_against_current_operating_config(self):
        """Verify near-identical configs are flagged and bypassed when alternatives exist."""
        # Seed 2 configs: one identical to current, one different
        wl._hypothesis_queue = [
            {'id': 'H-TEST-NEAR', 'config': {'freq_hz': 2.0, 'sweep_deg': 60.0, 'aoa_deg': 20.0, 'amp_port_deg': 96.0, 'cascade_phase_deg': 90.0}},
            {'id': 'H-TEST-DIFF', 'config': {'freq_hz': 1.2, 'sweep_deg': 90.0, 'aoa_deg': 12.0, 'amp_port_deg': 80.0, 'cascade_phase_deg': 45.0}},
        ]
        curr_op = {'freq_hz': 2.0, 'sweep_deg': 60.0, 'aoa_deg': 20.0, 'amp_port_deg': 96.0, 'cascade_phase_deg': 90.0}

        # First propose should skip H-TEST-NEAR and return H-TEST-DIFF
        proposed = wl.hypothesis_propose_next(current_config=curr_op, min_distance=0.04)
        self.assertIsNotNone(proposed)
        self.assertEqual(proposed['id'], 'H-TEST-DIFF')

    def test_deduplication_against_eval_history(self):
        """Verify candidate nearly identical to a previously tested hypothesis is bypassed."""
        # Record a past test result
        wl.hypothesis_record('H-PAST', Ps=0.25, V_ms=9.0, promoted=False,
                             config={'freq_hz': 2.1, 'sweep_deg': 60.0, 'aoa_deg': 20.0, 'amp_port_deg': 96.0, 'cascade_phase_deg': 90.0})

        wl._hypothesis_queue = [
            {'id': 'H-CANDIDATE-CLONE', 'config': {'freq_hz': 2.1, 'sweep_deg': 60.0, 'aoa_deg': 20.0, 'amp_port_deg': 96.0, 'cascade_phase_deg': 90.0}},
            {'id': 'H-CANDIDATE-NOVEL', 'config': {'freq_hz': 2.8, 'sweep_deg': 75.0, 'aoa_deg': 25.0, 'amp_port_deg': 88.0, 'cascade_phase_deg': 60.0}},
        ]

        proposed = wl.hypothesis_propose_next(min_distance=0.04)
        self.assertIsNotNone(proposed)
        self.assertEqual(proposed['id'], 'H-CANDIDATE-NOVEL')


class TestIntentVocabularyAndAoAMapping(unittest.TestCase):
    """Verifies that all intent taxonomies map to valid aerodynamic AoA schedules."""

    def test_complete_intent_vocabulary_coverage(self):
        """Ensure all primary flight intents are registered in PITCH_AOA_BY_INTENT."""
        expected_intents = [
            'slow', 'climb', 'cruise', 'sprint', 'glide', 'soar', 'deadstick',
            'manoeuvre', 'gust', 'approach_normal', 'landing_flare', 'touchdown_rollout',
            'takeoff_roll', 'rotate', 'go_around', 'zoom_climb', 'pigeon_steep_climb',
            'active_upstroke_brake', 'turtle_pivot_brake', 'roost_perch_flare',
            'stoop_auto', 'power_dive', 'wig_cruise', 'anhedral_turn',
            'dart_left', 'dart_right', 'taxi_fwd', 'taxi_rev',
            'kramer_lift_burst', 'coriolis_gust_preempt'
        ]
        for intent in expected_intents:
            self.assertIn(intent, wl.PITCH_AOA_BY_INTENT, f"Intent '{intent}' missing from PITCH_AOA_BY_INTENT")
            entry = wl.PITCH_AOA_BY_INTENT[intent]
            self.assertIn('up', entry)
            self.assertIn('down', entry)
            self.assertIn('profile', entry)
            self.assertIsInstance(entry['up'], (int, float))
            self.assertIsInstance(entry['down'], (int, float))
            self.assertIn(entry['profile'], ('static', 'trapezoid', 'advanced'))

    def test_pitch_stroke_command_with_expanded_intents(self):
        """Ensure pitch_stroke_command executes correctly for newly mapped intents."""
        for intent in ['landing_flare', 'approach_normal', 'touchdown_rollout', 'active_upstroke_brake']:
            cmd_up = wl.pitch_stroke_command(0.5, stroke_direction=+1, intent=intent)
            cmd_down = wl.pitch_stroke_command(0.5, stroke_direction=-1, intent=intent)
            self.assertIsInstance(cmd_up, float)
            self.assertIsInstance(cmd_down, float)
            self.assertGreater(cmd_up, -45.0)
            self.assertLess(cmd_up, 45.0)


class TestSafetyGovernors(unittest.TestCase):
    """Verifies flight safety governor calculations."""

    def test_glide_speed_floor_intervention(self):
        """Speed below 10.5 m/s unpowered must trigger active intervention."""
        res = check_glide_speed_floor(V_ias_ms=9.0, throttle_frac=0.0, intent='glide')
        self.assertTrue(res['intervention_active'])
        self.assertGreater(res['forced_freq_hz'], 0.0)

        # Safe speed above 11.0 m/s
        res_safe = check_glide_speed_floor(V_ias_ms=12.0, throttle_frac=0.0, intent='glide')
        self.assertFalse(res_safe['intervention_active'])

    def test_tandem_blade_clearance(self):
        """Opposing crossing with > 15 deg sweep differential triggers collision hazard."""
        pair_strokes = [+35.0, -35.0, 0.0, 0.0]
        pair_sweeps = [60.0, 85.0, 60.0, 60.0]  # 25 deg differential
        strike = check_tandem_blade_clearance(pair_strokes, pair_sweeps)
        self.assertFalse(strike['clearance_safe'])
        self.assertEqual(strike['violating_pair'], (0, 1))

    def test_sprint_dynamics_clamp(self):
        """Sprint flapping clamped to <= 3.2 Hz and <= 24 deg amplitude."""
        f, amp, rpm, margin = clamp_sprint_dynamics(freq_hz=4.5, amp_deg=35.0)
        self.assertLessEqual(f, 3.2)
        self.assertLessEqual(amp, 24.0)
        self.assertGreater(margin, 20.0)

    def test_regen_brake_resistor_governor(self):
        """Voltage >= 25.2V shunts 100% power to brake resistor."""
        gov = govern_regen_brake_resistor(bus_voltage_v=25.5, requested_regen_watts=40.0)
        self.assertTrue(gov['regen_allowed'])
        self.assertEqual(gov['battery_charge_w'], 0.0)
        self.assertEqual(gov['shunt_to_resistor_w'], 40.0)

    def test_proprioceptive_metachronal_clearance(self):
        """Metachronal 90 deg phase offset preserves clearance at ultra-compact s = 135mm."""
        # 4 stations with 90 deg metachronal phase lag at theta = 70 deg peak stroke
        stations = [
            {'stroke_deg': 0.0, 'sweep_deg': 90.0, 'pitch_deg': 5.0},     # St 1 mid-stroke
            {'stroke_deg': 70.0, 'sweep_deg': 90.0, 'pitch_deg': 0.0},    # St 2 peak dorsal
            {'stroke_deg': 0.0, 'sweep_deg': 90.0, 'pitch_deg': -5.0},    # St 3 mid-stroke
            {'stroke_deg': -70.0, 'sweep_deg': 90.0, 'pitch_deg': 0.0},   # St 4 peak ventral
        ]
        res = evaluate_proprioceptive_cascade_clearance(stations, stagger_mm=135.0, chord_mm=165.0)
        self.assertTrue(res['clearance_safe'])
        self.assertGreaterEqual(res['min_distance_mm'], 35.0)
        self.assertFalse(res['intervention_required'])

    def test_proprioceptive_active_deconfliction(self):
        """Active deconfliction fans sweep angles when in-phase collision is imminent."""
        # Simultaneous horizontal crossing (in-phase clash risk)
        clash_stations = [
            {'stroke_deg': 0.0, 'sweep_deg': 90.0, 'pitch_deg': 10.0},
            {'stroke_deg': 0.0, 'sweep_deg': 90.0, 'pitch_deg': 10.0},
            {'stroke_deg': 0.0, 'sweep_deg': 90.0, 'pitch_deg': 10.0},
            {'stroke_deg': 0.0, 'sweep_deg': 90.0, 'pitch_deg': 10.0},
        ]
        res = evaluate_proprioceptive_cascade_clearance(clash_stations, stagger_mm=135.0, chord_mm=165.0, vert_drop_mm=0.0)
        self.assertFalse(res['clearance_safe'])
        self.assertTrue(res['intervention_required'])
        self.assertIn(res['critical_pair'], [(0, 1), (1, 2), (2, 3)])

        # Apply deconfliction
        deconflicted = apply_proprioceptive_deconfliction(clash_stations, res, splay_gain_deg=5.0)
        i1, i2 = res['critical_pair']
        self.assertLess(deconflicted[i1]['sweep_deg'], 90.0)   # Swept forward
        self.assertGreater(deconflicted[i2]['sweep_deg'], 90.0) # Swept aft

    def test_h_prop1_registered(self):
        """H-PROP-1 is registered in AERO_HYPOTHESES with valid proprioceptive config."""
        h = next((entry for entry in wl.AERO_HYPOTHESES if entry['id'] == 'H-PROP-1'), None)
        self.assertIsNotNone(h)
        cfg = h['config']
        self.assertTrue(cfg.get('_h_prop_1'))
        self.assertEqual(cfg.get('h_prop1_encoder_rate_hz'), 1000.0)
        self.assertEqual(cfg.get('h_prop1_phase_lock_tolerance_deg'), 2.0)
        self.assertEqual(cfg.get('h_prop1_fleet_angle_limit_deg'), 14.0)
        # Check alias lookup via discovery_hypothesis_select
        res_hyp = wl.discovery_hypothesis_select(focus='proprioception')
        self.assertIsNotNone(res_hyp)
        self.assertEqual(res_hyp['id'], 'H-PROP-1')



class TestImportSafety(unittest.TestCase):
    """Verifies that all modular entry points can be imported without crashing the process."""

    def test_import_panel(self):
        import firmware.mco_panel as mp
        self.assertTrue(hasattr(mp, 'Panel'))

    def test_import_api_proxy(self):
        import firmware.mco_gs_server as gs
        self.assertTrue(hasattr(gs, 'MCOGSServer'))

    def test_flight_test_protocol_constants(self):
        import firmware.mco_test_cards as tc
        self.assertTrue(hasattr(tc, 'MTOW_N'))


if __name__ == '__main__':
    unittest.main()
