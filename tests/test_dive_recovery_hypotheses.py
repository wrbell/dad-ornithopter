#!/usr/bin/env python3
"""
tests/test_dive_recovery_hypotheses.py
======================================
Comprehensive verification suite for Aerodynamic Dive Recovery (H-PULL-2)
and Speed Closure Physics (H-DIVE-1) in MCO-1D flapping-wing flight controller.

Tests:
1. Exact aerodynamic equations and pivot torque balance:
   - Zero-load lift coefficient: CL = Cm / dx = 1.67
   - Usable pullout window (n_min, n_max) across 10 - 20 m/s
   - Closure speed bisection: V_closure = 16.3 m/s (rated 9.0 kg-cm servo)
   - Cascade reversal braking deceleration: gross & net vertical decel
2. DiveRecoveryGovernor state transitions:
   - Open window standby
   - Approaching closure buffer progressive cascade reversal
   - Window shut: pitch pull inhibit flag and full braking
   - Worst speed reached and altitude lost tracking
   - Recovery re-entry release with hysteresis
3. Wave library subsystem registration:
   - AERO_HYPOTHESES, BIOMIMETIC_HYPOTHESES, _hypothesis_queue
   - ALSS scoring tasks & curriculum C segments
   - CMTS context champion for dive_recovery
   - GS allowed commands & dive_status execution
4. FCC / Main flight loop safety integration
"""

import unittest
import sys
import os

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
FIRMWARE_DIR = os.path.join(REPO_ROOT, 'firmware')
if FIRMWARE_DIR not in sys.path:
    sys.path.insert(0, FIRMWARE_DIR)

from mco_flight_governor import (
    pull_zero_load_cl,
    pull_cl_floor,
    pull_cl_ceiling,
    pull_window,
    pull_closure_speed,
    pull_brake_decel_g,
    compute_maneuver_safe_speed,
    VNE_MS,
    DiveRecoveryGovernor,
    govern_dive_recovery,
    get_dive_recovery_status,
)
import mco_wave_library as wl


class TestDivePhysicsCalculations(unittest.TestCase):
    """Verifies the core physics equations from 'The Pullout Window and the Closure Speed'."""

    def test_zero_load_cl(self):
        """Zero-load lift coefficient CL = Cm / dx = 0.25 / 0.15 = 1.67."""
        cl_zero = pull_zero_load_cl(Cm=0.25, dx=0.15)
        self.assertAlmostEqual(cl_zero, 1.67, places=2)

    def test_pullout_window_progression(self):
        """
        Tests the recovery window (n_min, n_max) vs airspeed table:
        V = 10 m/s: open, n_min = 0.00g
        V = 12 m/s: open, n_min = 0.11g
        V = 14 m/s: open, n_min = 1.77g
        V = 15 m/s: narrowing, n_min = 2.70g
        V = 16 m/s: nearly shut, n_min = 3.70g
        V = 16.3 m/s: closes, n_min = 4.01g -> None
        V = 18 m/s: shut -> None
        V = 20 m/s: shut -> None
        """
        # 10 m/s
        w10 = pull_window(10.0)
        self.assertIsNotNone(w10)
        self.assertAlmostEqual(w10[0], 0.00, places=2)
        self.assertAlmostEqual(w10[1], 4.00, places=2)

        # 12 m/s
        w12 = pull_window(12.0)
        self.assertIsNotNone(w12)
        self.assertAlmostEqual(w12[0], 0.11, places=2)
        self.assertAlmostEqual(w12[1], 4.00, places=2)

        # 14 m/s
        w14 = pull_window(14.0)
        self.assertIsNotNone(w14)
        self.assertAlmostEqual(w14[0], 1.77, places=2)
        self.assertAlmostEqual(w14[1], 4.00, places=2)

        # 15 m/s
        w15 = pull_window(15.0)
        self.assertIsNotNone(w15)
        self.assertAlmostEqual(w15[0], 2.70, places=2)
        self.assertAlmostEqual(w15[1], 4.00, places=2)

        # 16 m/s
        w16 = pull_window(16.0)
        self.assertIsNotNone(w16)
        self.assertAlmostEqual(w16[0], 3.70, places=2)
        self.assertAlmostEqual(w16[1], 4.00, places=2)

        # 16.3 m/s: window shuts (n_min = 4.01g > 4.00g structural limit)
        w16_3 = pull_window(16.3)
        self.assertIsNone(w16_3)

        # 18.0 m/s: window completely shut
        w18 = pull_window(18.0)
        self.assertIsNone(w18)

        # 20.0 m/s: window completely shut (n_min ~ 8.32g)
        w20 = pull_window(20.0)
        self.assertIsNone(w20)

    def test_closure_speed_bisection(self):
        """Bisection solver finds V_closure = 16.3 +/- 0.1 m/s."""
        v_close = pull_closure_speed()
        self.assertAlmostEqual(v_close, 16.3, delta=0.1)

    def test_cascade_reversal_braking_deceleration(self):
        """
        Braking drag deceleration values from PDF:
        18 m/s: gross 3.12g, net 2.12g
        20 m/s: gross 3.85g, net 2.85g
        22 m/s: gross 4.66g, net 3.66g
        """
        self.assertAlmostEqual(pull_brake_decel_g(18.0, net_vertical=False), 3.12, places=2)
        self.assertAlmostEqual(pull_brake_decel_g(18.0, net_vertical=True), 2.12, places=2)

        self.assertAlmostEqual(pull_brake_decel_g(20.0, net_vertical=False), 3.85, places=2)
        self.assertAlmostEqual(pull_brake_decel_g(20.0, net_vertical=True), 2.85, places=2)

        self.assertAlmostEqual(pull_brake_decel_g(22.0, net_vertical=False), 4.66, places=2)
        self.assertAlmostEqual(pull_brake_decel_g(22.0, net_vertical=True), 3.66, places=2)


class TestDiveRecoveryGovernor(unittest.TestCase):
    """Verifies state transitions, hysteresis, and protection logic in DiveRecoveryGovernor."""

    def setUp(self):
        self.gov = DiveRecoveryGovernor()

    def test_initial_state(self):
        """Verify clean initial state and calculated closure speed."""
        self.assertAlmostEqual(self.gov.closure_speed_ms, 16.3, delta=0.1)
        self.assertFalse(self.gov.inhibit_flag)
        self.assertFalse(self.gov.cascade_reversal_active)
        self.assertEqual(self.gov.brake_fraction, 0.0)
        self.assertEqual(self.gov.closure_events, 0)

    def test_normal_airspeed_no_intervention(self):
        """At 12 m/s, pitch command passes through unimpeded, brake is inactive."""
        telem = self.gov.tick(V_ms=12.0, pitch_cmd_deg=8.0, altitude_m=100.0, dt=0.02)
        self.assertFalse(telem['inhibit_flag'])
        self.assertFalse(telem['cascade_reversal_active'])
        self.assertEqual(telem['brake_fraction'], 0.0)
        self.assertEqual(telem['effective_pitch_cmd_deg'], 8.0)
        self.assertEqual(telem['status'], 'RECOVERY_WINDOW_OPEN_NORMAL')

    def test_progressive_cascade_reversal_on_approach(self):
        """
        At 15.5 m/s (approaching closure speed within buffer = 1.5 m/s),
        progressive cascade reversal engages (brake_fraction > 0) to prevent closure,
        while inhibit flag remains False.
        """
        telem = self.gov.tick(V_ms=15.5, pitch_cmd_deg=5.0, altitude_m=90.0, dt=0.02)
        self.assertFalse(telem['inhibit_flag'])
        self.assertTrue(telem['cascade_reversal_active'])
        self.assertGreater(telem['brake_fraction'], 0.0)
        self.assertLess(telem['brake_fraction'], 1.0)
        self.assertEqual(telem['effective_pitch_cmd_deg'], 5.0)
        self.assertIn('DIVE_GOVERNOR_ACTIVE_BRAKE', telem['status'])

    def test_window_closure_triggers_inhibit_and_full_braking(self):
        """
        Above closure speed (e.g. 18.0 m/s), the recovery window shuts.
        Inhibit flag is raised, pitch command is forced to 0.0, full brake commanded.
        """
        telem = self.gov.tick(V_ms=18.0, pitch_cmd_deg=10.0, altitude_m=80.0, dt=0.02)
        self.assertTrue(telem['inhibit_flag'])
        self.assertTrue(telem['cascade_reversal_active'])
        self.assertEqual(telem['brake_fraction'], 1.0)
        self.assertEqual(telem['effective_pitch_cmd_deg'], 0.0)
        self.assertEqual(telem['closure_events'], 1)
        self.assertEqual(telem['worst_speed_reached_ms'], 18.0)
        self.assertEqual(telem['status'], 'RECOVERY_WINDOW_SHUT_PULL_INHIBITED_FULL_BRAKE')

    def test_worst_speed_and_altitude_lost_tracking(self):
        """Governor tracks peak dive speed reached and altitude lost while brake is engaged."""
        # Enter recovery at 80m, speed 19 m/s
        self.gov.tick(V_ms=19.0, pitch_cmd_deg=10.0, altitude_m=80.0, dt=0.02)
        self.assertEqual(self.gov.worst_speed_reached_ms, 19.0)

        # Accelerate slightly to 20.5 m/s, altitude descends to 77.0m
        telem = self.gov.tick(V_ms=20.5, pitch_cmd_deg=10.0, altitude_m=77.0, dt=0.02)
        self.assertEqual(self.gov.worst_speed_reached_ms, 20.5)
        self.assertAlmostEqual(telem['altitude_lost_recovery_m'], 3.0, places=2)

    def test_recovery_release_with_hysteresis(self):
        """
        Pitch inhibit is released only after speed decelerates below (closure_speed - 1.0 m/s).
        """
        # Trigger window shut at 18 m/s
        self.gov.tick(V_ms=18.0, pitch_cmd_deg=10.0, altitude_m=80.0, dt=0.02)
        self.assertTrue(self.gov.inhibit_flag)

        # Decelerate to 16.0 m/s (window open, but within 1.0 m/s hysteresis zone)
        telem16 = self.gov.tick(V_ms=16.0, pitch_cmd_deg=10.0, altitude_m=76.0, dt=0.02)
        self.assertTrue(telem16['inhibit_flag'], "Inhibit should remain active inside hysteresis buffer")
        self.assertEqual(telem16['effective_pitch_cmd_deg'], 0.0)

        # Decelerate to 14.5 m/s (< 15.3 m/s setpoint): inhibit is released
        telem14 = self.gov.tick(V_ms=14.5, pitch_cmd_deg=10.0, altitude_m=74.0, dt=0.02)
        self.assertFalse(telem14['inhibit_flag'])
        self.assertEqual(telem14['effective_pitch_cmd_deg'], 10.0)
        self.assertEqual(telem14['status'], 'RECOVERY_WINDOW_REOPENED_PULL_RELEASED')


class TestWaveLibraryRegistrations(unittest.TestCase):
    """Verifies that H-PULL-2 and H-DIVE-1 are registered in all wave library subsystems."""

    def test_hypotheses_registrations(self):
        """Verify H-PULL-2 and H-DIVE-1 exist in AERO_HYPOTHESES, queue, and BIOMIMETIC_HYPOTHESES."""
        aero_ids = {h['id'] for h in wl.AERO_HYPOTHESES}
        self.assertIn('H-PULL-2', aero_ids)
        self.assertIn('H-DIVE-1', aero_ids)
        self.assertIn('H-BOYD-1', aero_ids)

        queue_ids = {h['id'] for h in wl._hypothesis_queue}
        self.assertIn('H-PULL-2', queue_ids)
        self.assertIn('H-DIVE-1', queue_ids)
        self.assertIn('H-BOYD-1', queue_ids)

        self.assertIn('H-PULL-2', wl.BIOMIMETIC_HYPOTHESES)
        self.assertIn('H-DIVE-1', wl.BIOMIMETIC_HYPOTHESES)
        self.assertIn('H-BOYD-1', wl.BIOMIMETIC_HYPOTHESES)

    def test_alss_scoring_tasks(self):
        """Verify all 4 scoring tasks are registered in _SG_TASKS."""
        expected_tasks = [
            'recovery_window_g',
            'altitude_lost_recovery_m',
            'peak_servo_margin_pullout',
            'dive_speed_overshoot_ms',
        ]
        for task in expected_tasks:
            self.assertIn(task, wl._SG_TASKS)
            self.assertIn('description', wl._SG_TASKS[task])

    def test_cmts_and_phase_mappings(self):
        """Verify phase mapping and CMTS context champion for dive_recovery."""
        self.assertIn('dive_recovery', wl._PHASE_TO_SG_TASK)
        self.assertIn('recovery_window_g', wl._PHASE_TO_SG_TASK['dive_recovery'])

        self.assertIn('dive_recovery', wl._CMTS_CHAMPIONS)
        champ = wl._CMTS_CHAMPIONS['dive_recovery']
        self.assertEqual(champ.get('hyp_id'), 'H-PULL-2')

    def test_alss_curriculum_segments(self):
        """Verify pull2_recovery_c, dive1_governor_c, and boyd1_maneuver_c are in _ALSS_SEGMENTS and Curriculum C."""
        self.assertIn('pull2_recovery_c', wl._ALSS_SEGMENTS)
        self.assertIn('dive1_governor_c', wl._ALSS_SEGMENTS)
        self.assertIn('boyd1_maneuver_c', wl._ALSS_SEGMENTS)

        self.assertIn('pull2_recovery_c', wl._ALSS_CURRICULUM['C'])
        self.assertIn('dive1_governor_c', wl._ALSS_CURRICULUM['C'])
        self.assertIn('boyd1_maneuver_c', wl._ALSS_CURRICULUM['C'])

    def test_gs_allowed_commands(self):
        """Verify dive_status is registered in _GS_ALLOWED_CMDS."""
        self.assertIn('dive_status', wl._GS_ALLOWED_CMDS)


class TestWaveLibraryExecutionAndGS(unittest.TestCase):
    """Verifies runtime behavior of evaluation hooks, ticks, and GS command."""

    def test_pull_tick(self):
        """Verify wl._pull_tick updates _pull_state telemetry."""
        state = wl._pull_tick({'V_ms': 14.0, 'altitude_m': 100.0, 'dt': 0.02})
        self.assertIn('V_ms', state)
        self.assertIn('window', state)
        self.assertIn('inhibit_flag', state)
        self.assertEqual(state['V_ms'], 14.0)

    def test_evaluate_dive_recovery(self):
        """Verify evaluate_dive_recovery calculates score delta, window, overshoot, and servo margin."""
        res_open = wl.evaluate_dive_recovery({'V_ms': 14.0})
        self.assertEqual(res_open['hyp_id'], 'H-PULL-2')
        self.assertGreater(res_open['recovery_window_g'], 0.0)
        self.assertEqual(res_open['dive_speed_overshoot_ms'], 0.0)
        self.assertEqual(res_open['peak_servo_margin_pullout'], 1.40)
        self.assertGreater(res_open['score_delta'], 0.0)

        res_shut = wl.evaluate_dive_recovery({'V_ms': 19.0})
        self.assertEqual(res_shut['recovery_window_g'], 0.0)
        self.assertGreater(res_shut['dive_speed_overshoot_ms'], 0.0)
        self.assertEqual(res_shut['peak_servo_margin_pullout'], 0.57)

    def test_dive_status_command(self):
        """Verify GS command 'dive_status' runs and outputs comprehensive report."""
        res = wl._gs_execute_cmd({'cmd': 'dive_status'})
        self.assertEqual(res.get('type'), 'ack')
        self.assertEqual(res.get('cmd'), 'dive_status')
        report = res.get('result', '')
        self.assertIn('MCO-1D DIVE RECOVERY & SPEED CLOSURE ENVELOPE', report)
        self.assertIn('CLOSURE SPEED', report)
        self.assertIn('SERVO LOAD VS LOAD FACTOR', report)
        self.assertIn('CASCADE REVERSAL BRAKING DECELERATION', report)


class TestMcoMainSafetyLoopIntegration(unittest.TestCase):
    """Verifies integration into mco_main 50 Hz active safety loop."""

    def test_govern_dive_recovery_updates_result(self):
        """Verify govern_dive_recovery mutates the flight control result dictionary."""
        result = {'freq_hz': 2.5, 'wing_commands': [{'pitch_deg': 12.0}]}
        telem = govern_dive_recovery(
            V_ms=18.0,
            pitch_cmd_deg=12.0,
            altitude_m=100.0,
            dt=0.02,
            result=result
        )
        self.assertTrue(result['dive_inhibit_flag'])
        self.assertTrue(result['dive_brake_active'])
        self.assertEqual(result['dive_brake_fraction'], 1.0)
        self.assertIn('RECOVERY_WINDOW_SHUT', result['dive_recovery_status'])
        self.assertEqual(telem['effective_pitch_cmd_deg'], 0.0)

    def test_get_dive_recovery_status(self):
        """Verify get_dive_recovery_status returns live telemetry."""
        status = get_dive_recovery_status()
        self.assertIn('closure_speed_ms', status)
        self.assertIn('inhibit_flag', status)
        self.assertIn('cascade_reversal_active', status)


class TestBoydEnergyManeuverabilityGovernor(unittest.TestCase):
    """
    Verifies John Boyd Energy-Maneuverability speed braking, VNE envelope protection,
    stoop recovery, and kinetic energy retention.
    """

    def setUp(self):
        self.gov = DiveRecoveryGovernor()

    def test_compute_maneuver_safe_speed_unloads_servo(self):
        """
        Pulling higher load factors counteracts nose-down camber moment via lift offset,
        permitting higher safe entry airspeeds for the maneuver.
        """
        v1g = compute_maneuver_safe_speed(n_cmd=1.0)
        v2_5g = compute_maneuver_safe_speed(n_cmd=2.5)
        v4g = compute_maneuver_safe_speed(n_cmd=4.0)

        self.assertAlmostEqual(v1g, 14.39, places=1)
        self.assertAlmostEqual(v2_5g, 15.93, places=1)
        self.assertAlmostEqual(v4g, 17.34, places=1)
        self.assertLess(v1g, v2_5g)
        self.assertLess(v2_5g, v4g)

    def test_vne_hard_flutter_protection(self):
        """
        When airspeed reaches or exceeds VNE (28.0 m/s), full speed brake is commanded
        regardless of stick position to prevent flutter and structural divergence.
        """
        telem = self.gov.tick(V_ms=28.5, pitch_cmd_deg=0.0, altitude_m=200.0, dt=0.02)
        self.assertTrue(telem['vne_exceeded'])
        self.assertTrue(telem['cascade_reversal_active'])
        self.assertEqual(telem['brake_fraction'], 1.0)
        self.assertEqual(telem['status'], 'VNE_EXCEEDED_FULL_SPEED_BRAKE')

    def test_straight_stoop_dash_zero_unwanted_drag(self):
        """
        In an unaccelerated stoop dive or sprint (intent='stoop', sticks neutral),
        the aircraft can dive up to 22.0 m/s without unwanted braking drag.
        """
        telem = self.gov.tick(
            V_ms=22.0, pitch_cmd_deg=0.0, bank_deg=0.0,
            altitude_m=150.0, dt=0.02, intent='stoop'
        )
        self.assertFalse(telem['inhibit_flag'])
        self.assertFalse(telem['cascade_reversal_active'])
        self.assertEqual(telem['brake_fraction'], 0.0)
        self.assertEqual(telem['status'], 'STOOP_HIGH_SPEED_DASH_ARMED')

    def test_stoop_recovery_pullout_timing_and_energy(self):
        """
        Pulling out of a 22 m/s stoop with a 4.0g pullout command:
        1. Governor intercepts pitch command at neutral (inhibit_flag=True) and applies brake.
        2. Rather than scrubbing down to 14 m/s, it targets V_target(4.0g) ≈ 17.34 m/s.
        3. The moment speed reaches 17.0 m/s, pitch command is instantly released.
        4. Conserves > 200 J of kinetic energy vs blunt 14 m/s braking.
        """
        # Step 1: Pilot pulls 20° aft stick (4g pullout) at 22 m/s
        telem1 = self.gov.tick(
            V_ms=22.0, pitch_cmd_deg=20.0, bank_deg=0.0,
            altitude_m=100.0, dt=0.02, intent='stoop'
        )
        self.assertTrue(telem1['inhibit_flag'])
        self.assertTrue(telem1['cascade_reversal_active'])
        self.assertEqual(telem1['effective_pitch_cmd_deg'], 0.0)
        self.assertAlmostEqual(telem1['v_target_ms'], 17.34, places=1)

        # Step 2: Speed bleeds to 17.0 m/s (below V_target)
        telem2 = self.gov.tick(
            V_ms=17.0, pitch_cmd_deg=20.0, bank_deg=0.0,
            altitude_m=95.0, dt=0.02, intent='stoop'
        )
        self.assertFalse(telem2['inhibit_flag'])
        self.assertEqual(telem2['effective_pitch_cmd_deg'], 20.0)
        self.assertFalse(telem2['cascade_reversal_active'])
        self.assertEqual(telem2['status'], 'RECOVERY_WINDOW_REOPENED_PULL_RELEASED')
        self.assertGreater(telem2['boyd_energy_saved_j'], 200.0)

    def test_cranking_turn_adaptive_braking(self):
        """
        Entering a 60° bank cranking turn at 18.0 m/s:
        Governor targets V_safe(2.0g) ≈ 15.44 m/s, shedding only 2.6 m/s,
        preserving turn rate and momentum.
        """
        telem_enter = self.gov.tick(
            V_ms=18.0, pitch_cmd_deg=0.0, bank_deg=60.0,
            altitude_m=120.0, dt=0.02, intent='cruise'
        )
        self.assertTrue(telem_enter['cascade_reversal_active'])
        self.assertAlmostEqual(telem_enter['v_target_ms'], 15.44, places=1)

        # At 15.2 m/s: turn command is cleared with momentum retained
        telem_safe = self.gov.tick(
            V_ms=15.2, pitch_cmd_deg=0.0, bank_deg=60.0,
            altitude_m=118.0, dt=0.02, intent='cruise'
        )
        self.assertFalse(telem_safe['inhibit_flag'])
        self.assertFalse(telem_safe['cascade_reversal_active'])


if __name__ == '__main__':
    unittest.main()

