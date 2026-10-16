#!/usr/bin/env python3
"""
Test Suite: Surface Taxiing & High-Wind Takeoff in Crosswinds, Headwinds, and Tailwinds
========================================================================================
Tests the aerodynamic physics models, flight governor safeguards, and wave library
hypotheses for surface wind handling on the MCO-1D 8-wing cascade ornithopter:
  - H-TAXI-WIND-1: Active Ground Suction Downforce
  - H-TAXI-REEF-1: Asymmetric Falcon Upwind Wing Reefing
  - H-TAXI-YAW-1: Autonomous Anti-Weathervaning Differential Flap
  - H-TAXI-TAIL-1: Tailwind Inverted Streamline Anti-Tumble
  - H-HEAD-TO-1: Strong Headwind Pop-Up Takeoff
  - H-CROSS-TO-1: Decoupled Lateral Thrust Crosswind Takeoff
  - H-VAULT-GUST-1: Severe Turbulence / Gust Vault Breakout
"""

import sys
import os
import unittest
import math

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'firmware'))

import mco_advanced_physics as ap
import mco_flight_governor as gov
from mco_taxi_park import taxi_commands
import mco_wave_library as wave


class TestTaxiHighWindCrosswindSuite(unittest.TestCase):

    def test_passive_crosswind_vulnerability(self):
        """Verify that passive uncompensated 8-wing cascade experiences rollover hazard in >= 18 kt crosswinds."""
        # 10 kt crosswind (5.14 m/s)
        _, _, gsf_10, diag_10 = ap.crosswind_ground_aerodynamics(V_wind_ms=5.14, wind_bearing_deg=90.0, heading_deg=0.0)
        self.assertGreater(gsf_10, 2.0, "10 kt crosswind should be passively stable")
        self.assertFalse(diag_10['rollover_hazard'])

        # 20 kt crosswind (10.29 m/s)
        _, _, gsf_20, diag_20 = ap.crosswind_ground_aerodynamics(V_wind_ms=10.29, wind_bearing_deg=90.0, heading_deg=0.0)
        self.assertLess(gsf_20, 1.0, "20 kt crosswind should produce passive rollover hazard (GSF < 1.0)")
        self.assertTrue(diag_20['rollover_hazard'])
        self.assertGreater(diag_20['M_overturn_passive_nm'], diag_20['M_restore_nm'])

        # 25 kt crosswind (12.86 m/s)
        _, _, gsf_25, diag_25 = ap.crosswind_ground_aerodynamics(V_wind_ms=12.86, wind_bearing_deg=90.0, heading_deg=0.0)
        self.assertLess(gsf_25, 0.5, "25 kt crosswind without active pitch will roll the craft over")

    def test_active_ground_suction_and_reefing(self):
        """Verify H-TAXI-WIND-1 and H-TAXI-REEF-1 convert hazardous crosswinds into stable GSF > 2.0."""
        # Test 20 kt crosswind from port (wind bearing 90°, heading 0°)
        pitch_20, (sw_port_20, sw_stbd_20), delta_amp_20, diag_20 = ap.ground_wind_active_compensator(
            V_wind_ms=10.29, wind_bearing_deg=90.0, heading_deg=0.0
        )
        # Port wing is upwind: should reef aft
        self.assertGreaterEqual(sw_port_20, 120.0, "Upwind port wings should sweep aft >= 120°")
        self.assertEqual(sw_stbd_20, 60.0, "Downwind starboard wings should remain at 60°")
        # Pitch should be negative to generate downforce
        self.assertLessEqual(pitch_20, -12.0, "Suction pitch should be <= -12° in 20 kt winds")
        self.assertGreater(diag_20['active_downforce_n'], 5.0, "Downforce should be positive")
        self.assertGreaterEqual(diag_20['gsf_compensated'], 2.0, "Compensated GSF must be >= 2.0 (unconditionally stable)")
        self.assertTrue(diag_20['wind_hazard_mitigated'])

        # Test 25 kt crosswind from starboard (wind bearing 270°, heading 0°)
        pitch_25, (sw_port_25, sw_stbd_25), delta_amp_25, diag_25 = ap.ground_wind_active_compensator(
            V_wind_ms=12.86, wind_bearing_deg=270.0, heading_deg=0.0
        )
        self.assertEqual(sw_port_25, 60.0, "Downwind port wings should remain at 60°")
        self.assertGreaterEqual(sw_stbd_25, 140.0, "Upwind starboard wings should reef to >= 140°")
        self.assertLessEqual(pitch_25, -18.0, "Suction pitch should reach ~ -20° in 25 kt winds")
        self.assertGreater(diag_25['apparent_weight_n'], 70.0, "Apparent weight should increase due to aerodynamic downforce")
        self.assertGreater(diag_25['gsf_compensated'], 3.0)

    def test_anti_weathervaning_differential_stroke(self):
        """Verify H-TAXI-YAW-1 counteracts weathercock yaw moment."""
        # 15 kt crosswind from port creates clockwise (right) yaw torque on fuselage aft of gear
        _, (sw_p, sw_s), delta_amp, diag = ap.ground_wind_active_compensator(
            V_wind_ms=7.72, wind_bearing_deg=90.0, heading_deg=0.0
        )
        self.assertGreater(delta_amp, 0.0, "Must command differential amplitude to counter weathervaning")
        self.assertLessEqual(delta_amp, 22.0, "Differential amplitude clamped to physical limits")

    def test_tailwind_anti_tumble(self):
        """Verify H-TAXI-TAIL-1 inverts wing pitch bias to prevent reverse-flow tail lift."""
        # 15 kt tailwind (wind bearing 180°, heading 0°)
        pitch_tail, sweeps, _, diag_tail = ap.ground_wind_active_compensator(
            V_wind_ms=7.72, wind_bearing_deg=180.0, heading_deg=0.0
        )
        # In tailwind, pitch must be positive relative to fuselage (+6° to +18°)
        self.assertGreaterEqual(pitch_tail, 6.0, "Tailwind requires positive pitch to ground tail against reverse flow")
        self.assertEqual(sweeps, (60.0, 60.0), "Pure tailwind has symmetric sweeps")

    def test_high_wind_takeoff_matrix(self):
        """Verify dispatch of high-wind takeoff modes under various wind vectors."""
        # Strong Headwind (15 kt = 7.72 m/s headwind) -> H-HEAD-TO-1
        mode_head, roll_head, diag_head = ap.high_wind_takeoff_matrix(
            V_wind_ms=7.72, wind_bearing_deg=0.0, heading_deg=0.0, runway_heading_deg=0.0
        )
        self.assertEqual(mode_head, 'H-HEAD-TO-1')
        self.assertLessEqual(roll_head, 2.0, "Strong headwind allows pop-up unstick in < 2.0 m")
        self.assertEqual(diag_head['rotation_pitch_deg'], 14.0)

        # Pure Crosswind (12 kt = 6.17 m/s crosswind) -> H-CROSS-TO-1
        mode_cross, roll_cross, diag_cross = ap.high_wind_takeoff_matrix(
            V_wind_ms=6.17, wind_bearing_deg=90.0, heading_deg=0.0, runway_heading_deg=0.0
        )
        self.assertEqual(mode_cross, 'H-CROSS-TO-1')
        self.assertNotEqual(diag_cross['decoupled_lateral_n'], 0.0, "Crosswind takeoff engages decoupled lateral thrust")

        # Severe Gusts / Intent Vault -> H-VAULT-GUST-1
        mode_vault, roll_vault, diag_vault = ap.high_wind_takeoff_matrix(
            V_wind_ms=10.0, wind_bearing_deg=45.0, heading_deg=0.0, runway_heading_deg=0.0, intent='vault_takeoff'
        )
        self.assertEqual(mode_vault, 'H-VAULT-GUST-1')
        self.assertLessEqual(roll_vault, 0.5, "Vault takeoff unstick is immediate (<= 0.5 m)")

        # Tailwind -> H-TAIL-ESC-1
        mode_tail, roll_tail, diag_tail = ap.high_wind_takeoff_matrix(
            V_wind_ms=5.0, wind_bearing_deg=180.0, heading_deg=0.0, runway_heading_deg=0.0
        )
        self.assertEqual(mode_tail, 'H-TAIL-ESC-1')

    def test_taxi_commands_wind_integration(self):
        """Verify taxi_commands() accepts wind_data and adjusts wing sweep, pitch, and amplitude."""
        # Calm baseline
        calm_cmd = taxi_commands(throttle_frac=0.3, yaw_cmd=0.0)
        self.assertEqual(calm_cmd['wing_cmds'][0]['sweep'], 60.0)
        self.assertEqual(calm_cmd['wing_cmds'][0]['pitch'], 3.0)

        # 20 kt crosswind from port
        wind_data = {'V_wind_ms': 10.29, 'wind_bearing_deg': 90.0, 'heading_deg': 0.0}
        wind_cmd = taxi_commands(throttle_frac=0.3, yaw_cmd=0.0, wind_data=wind_data)
        self.assertTrue(wind_cmd['wind_mitigated'])
        self.assertLess(wind_cmd['active_suction_pitch_deg'], -10.0)
        # Port wing (index 0) swept aft; starboard wing (index 1) at base sweep
        self.assertGreater(wind_cmd['wing_cmds'][0]['sweep'], 120.0)
        self.assertEqual(wind_cmd['wing_cmds'][1]['sweep'], 60.0)
        # Both wings have negative suction pitch
        self.assertLess(wind_cmd['wing_cmds'][0]['pitch'], -5.0)
        self.assertLess(wind_cmd['wing_cmds'][1]['pitch'], -5.0)

    def test_ground_wind_safety_governor(self):
        """Verify GroundWindSafetyGovernor state machine and telemetry."""
        governor = gov.GroundWindSafetyGovernor(track_width_m=0.220, default_mass_kg=5.367)

        # Calm taxi
        t_calm = governor.tick(on_ground=True, V_wind_ms=1.0)
        self.assertEqual(t_calm['state'], 'CALM_TAXIS')
        self.assertEqual(t_calm['takeoff_mode'], 'CONV-STOL-1')

        # Crosswind reefing
        t_cross = governor.tick(on_ground=True, V_wind_ms=10.0, wind_bearing_deg=90.0, heading_deg=0.0)
        self.assertEqual(t_cross['state'], 'CROSSWIND_REEFING_ACTIVE')
        self.assertTrue(t_cross['rollover_hazard_mitigated'])
        self.assertGreater(t_cross['gsf_compensated'], 2.0)

        # Disarmed when airborne
        t_air = governor.tick(on_ground=False, V_wind_ms=15.0)
        self.assertEqual(t_air['state'], 'AIRBORNE_DISARMED')
        self.assertEqual(t_air['active_suction_pitch_deg'], 0.0)

    def test_wave_library_hypotheses_and_evaluator(self):
        """Verify ground wind hypotheses are registered in wave library and evaluate cleanly."""
        expected_hyps = [
            'H-TAXI-WIND-1', 'H-TAXI-REEF-1', 'H-TAXI-YAW-1', 'H-TAXI-TAIL-1',
            'H-HEAD-TO-1', 'H-CROSS-TO-1', 'H-VAULT-GUST-1'
        ]
        registered_ids = {h['id'] for h in wave.AERO_HYPOTHESES}
        for hid in expected_hyps:
            self.assertIn(hid, registered_ids, f"Hypothesis {hid} must be in AERO_HYPOTHESES")

        # Test evaluator hook
        eval_res = wave.evaluate_ground_wind_mitigation({
            'V_wind_ms': 10.0, 'wind_bearing_deg': 90.0, 'heading_deg': 0.0
        })
        self.assertEqual(eval_res['hyp_id'], 'H-TAXI-WIND-1')
        self.assertTrue(eval_res['wind_hazard_mitigated'])
        self.assertGreater(eval_res['score_delta'], 0.0)

    def test_biomimetic_tendon_wind_inference(self):
        """Verify ground wind estimation using wing pitch servo holding currents (tendon mechanoreceptors)."""
        # Scenario 1: Port crosswind (port pitch servos heavily loaded, stbd lightly loaded)
        # Port servos (0, 2, 4, 6) = 0.85A; Stbd servos (1, 3, 5, 7) = 0.25A
        currents_port_cross = [0.85, 0.25, 0.85, 0.25, 0.85, 0.25, 0.85, 0.25]
        v_w, bearing, conf, diag = ap.estimate_ground_wind_from_wing_tendons(
            servo_currents_a=currents_port_cross, pitot_v_ms=0.5, heading_deg=0.0
        )
        self.assertGreater(v_w, 6.0, "Should infer ~7 m/s crosswind from differential servo currents")
        # Wind from port (left) at 0° heading corresponds to ~90° bearing
        self.assertAlmostEqual(bearing, 90.0, delta=15.0)
        self.assertGreater(conf, 0.6)

        # Scenario 2: Integration with GroundWindSafetyGovernor when external wind telemetry is 0.0
        governor = gov.GroundWindSafetyGovernor()
        telem = governor.tick(
            on_ground=True, V_wind_ms=0.0, heading_deg=0.0,
            servo_currents_a=currents_port_cross, pitot_v_ms=0.5
        )
        self.assertEqual(telem['state'], 'CROSSWIND_REEFING_ACTIVE')
        self.assertGreater(telem['sweep_override_port_deg'], 90.0)
        self.assertLess(telem['active_suction_pitch_deg'], -8.0)

    def test_adv_catalogs_and_scoring_tasks(self):
        """Verify ground wind hypotheses are registered across all 5 ALC modalities."""
        target_ids = [
            'H-TAXI-WIND-1', 'H-TAXI-REEF-1', 'H-TAXI-YAW-1', 'H-TAXI-TAIL-1',
            'H-HEAD-TO-1', 'H-CROSS-TO-1', 'H-VAULT-GUST-1'
        ]
        adv_ids = {h['id'] for h in wave._ADV_NEW_HYPS}
        q_ids = {h['id'] for h in wave._hypothesis_queue}
        aero_ids = {h['id'] for h in wave.AERO_HYPOTHESES}

        for hid in target_ids:
            self.assertIn(hid, adv_ids, f"{hid} missing from _ADV_NEW_HYPS")
            self.assertIn(hid, q_ids, f"{hid} missing from _hypothesis_queue")
            self.assertIn(hid, aero_ids, f"{hid} missing from AERO_HYPOTHESES")
            self.assertIn(hid, wave.BIOMIMETIC_HYPOTHESES, f"{hid} missing from BIOMIMETIC_HYPOTHESES")

        # Verify _SG_TASKS
        expected_tasks = [
            'gsf_compensated', 'active_downforce_n', 'anti_weathervane_yaw_error_deg',
            'tailwind_anti_tumble_margin_nm', 'high_wind_ground_roll_m'
        ]
        for task in expected_tasks:
            self.assertIn(task, wave._SG_TASKS, f"{task} missing from _SG_TASKS")

        # Verify _CMTS_CHAMPIONS
        expected_champs = [
            'ground_wind_suction', 'crosswind_reefing', 'anti_weathervaning',
            'tailwind_anti_tumble', 'headwind_popup_takeoff', 'crosswind_takeoff',
            'gust_vault_breakout'
        ]
        for champ in expected_champs:
            self.assertIn(champ, wave._CMTS_CHAMPIONS, f"{champ} missing from _CMTS_CHAMPIONS")

        # Verify _EXPANDED_DISCOVERY_BOUNDS
        expected_bounds = [
            'h_taxi_wind_pitch_bias_deg', 'h_taxi_reef_sweep_upwind_deg',
            'h_taxi_yaw_delta_amp_deg', 'h_taxi_tail_pitch_bias_deg',
            'h_head_to_rotation_pitch_deg', 'h_cross_to_lateral_bias_deg',
            'h_vault_gust_jump_pitch_deg'
        ]
        for b in expected_bounds:
            self.assertIn(b, wave._EXPANDED_DISCOVERY_BOUNDS, f"{b} missing from _EXPANDED_DISCOVERY_BOUNDS")

    def test_ai_observer_trigger_56_and_priority_queue(self):
        """Verify AI Observer Trigger #56 detects ground wind hazards and queues hypotheses."""
        wave._ALC_SHARED['priority_hyp_queue'].clear()

        # High crosswind on ground
        sensors_cross = {
            'on_ground': True,
            'wind_spd_ms': 10.0,
            'wind_ang_rel_deg': 90.0,
        }
        obs = wave.ai_observer_tick(sensors_cross, {})
        self.assertTrue(obs.get('ground_wind_hazard'))
        queued = [e['hyp_id'] for e in wave._ALC_SHARED['priority_hyp_queue']]
        self.assertIn('H-TAXI-WIND-1', queued)
        self.assertIn('H-TAXI-REEF-1', queued)
        self.assertIn('H-TAXI-YAW-1', queued)

        # Ground tailwind
        wave._ALC_SHARED['priority_hyp_queue'].clear()
        sensors_tail = {
            'on_ground': True,
            'wind_spd_ms': 6.0,
            'wind_ang_rel_deg': 160.0,
        }
        obs_tail = wave.ai_observer_tick(sensors_tail, {})
        self.assertTrue(obs_tail.get('tailwind_ground_hazard'))
        queued_tail = [e['hyp_id'] for e in wave._ALC_SHARED['priority_hyp_queue']]
        self.assertIn('H-TAXI-TAIL-1', queued_tail)

        # Takeoff in strong headwind
        wave._ALC_SHARED['priority_hyp_queue'].clear()
        sensors_head_to = {
            'on_ground': True,
            'takeoff_demanded': True,
            'wind_spd_ms': 7.0,
            'wind_ang_rel_deg': 15.0,
        }
        obs_head = wave.ai_observer_tick(sensors_head_to, {})
        self.assertTrue(obs_head.get('headwind_popup_takeoff_active'))
        queued_head = [e['hyp_id'] for e in wave._ALC_SHARED['priority_hyp_queue']]
        self.assertIn('H-HEAD-TO-1', queued_head)

        # Takeoff in severe gust / turbulence
        wave._ALC_SHARED['priority_hyp_queue'].clear()
        sensors_gust = {
            'on_ground': True,
            'takeoff_demanded': True,
            'gust_severity': 0.8,
        }
        obs_gust = wave.ai_observer_tick(sensors_gust, {})
        self.assertTrue(obs_gust.get('gust_vault_takeoff_active'))
        queued_gust = [e['hyp_id'] for e in wave._ALC_SHARED['priority_hyp_queue']]
        self.assertIn('H-VAULT-GUST-1', queued_gust)

    def test_discovery_hypothesis_select_focus_and_sensors(self):
        """Verify discovery_hypothesis_select dispatch via focus keywords and sensor triggers."""
        # Focus dispatch
        h_wind = wave.discovery_hypothesis_select(focus='ground_wind')
        self.assertIsNotNone(h_wind)
        self.assertEqual(h_wind['id'], 'H-TAXI-WIND-1')

        h_reef = wave.discovery_hypothesis_select(focus='crosswind_taxi')
        self.assertIsNotNone(h_reef)
        self.assertEqual(h_reef['id'], 'H-TAXI-REEF-1')

        h_yaw = wave.discovery_hypothesis_select(focus='anti_weathervaning')
        self.assertIsNotNone(h_yaw)
        self.assertEqual(h_yaw['id'], 'H-TAXI-YAW-1')

        h_tail = wave.discovery_hypothesis_select(focus='tailwind')
        self.assertIsNotNone(h_tail)
        self.assertEqual(h_tail['id'], 'H-TAXI-TAIL-1')

        h_head = wave.discovery_hypothesis_select(focus='headwind_takeoff')
        self.assertIsNotNone(h_head)
        self.assertEqual(h_head['id'], 'H-HEAD-TO-1')

        h_cross = wave.discovery_hypothesis_select(focus='crosswind_takeoff')
        self.assertIsNotNone(h_cross)
        self.assertEqual(h_cross['id'], 'H-CROSS-TO-1')

        h_vault = wave.discovery_hypothesis_select(focus='gust_vault')
        self.assertIsNotNone(h_vault)
        self.assertEqual(h_vault['id'], 'H-VAULT-GUST-1')

        # Direct sensor trigger dispatch (clear priority queue first so sensor rules are evaluated)
        wave._ALC_SHARED['priority_hyp_queue'].clear()
        h_sensor_wind = wave.discovery_hypothesis_select(sensors={'h_taxi_wind_1_active': True})
        self.assertEqual(h_sensor_wind['id'], 'H-TAXI-WIND-1')

        h_sensor_reef = wave.discovery_hypothesis_select(sensors={'h_taxi_reef_1_active': True})
        self.assertEqual(h_sensor_reef['id'], 'H-TAXI-REEF-1')

        h_sensor_yaw = wave.discovery_hypothesis_select(sensors={'h_taxi_yaw_1_active': True})
        self.assertEqual(h_sensor_yaw['id'], 'H-TAXI-YAW-1')

        h_sensor_tail = wave.discovery_hypothesis_select(sensors={'h_taxi_tail_1_active': True})
        self.assertEqual(h_sensor_tail['id'], 'H-TAXI-TAIL-1')

        h_sensor_vault = wave.discovery_hypothesis_select(sensors={'h_vault_gust_1_active': True})
        self.assertEqual(h_sensor_vault['id'], 'H-VAULT-GUST-1')

    def test_update_genome_from_trial(self):
        """Verify _update_genome_from_trial routes all 7 ground wind hypotheses correctly."""
        import copy
        orig_weights = copy.deepcopy(wave._lever_mode_weights)
        try:
            for hid in ['H-TAXI-WIND-1', 'H-TAXI-REEF-1', 'H-TAXI-YAW-1', 'H-TAXI-TAIL-1', 'H-HEAD-TO-1', 'H-CROSS-TO-1', 'H-VAULT-GUST-1']:
                wave._update_genome_from_trial(hid, 0.15, {})
        finally:
            wave._lever_mode_weights.clear()
            wave._lever_mode_weights.update(orig_weights)

    def test_status_reports(self):
        """Verify advanced_physics_status and ground_wind_status run cleanly."""
        rep_adv = wave.advanced_physics_status()
        self.assertIn("GROUND WIND SAFETY & TAKEOFF MATRIX", rep_adv)
        self.assertIn("H-TAXI-WIND-1", rep_adv)

        rep_gw = wave.ground_wind_status()
        self.assertIn("GROUND WIND DYNAMICS & TAKEOFF MATRIX", rep_gw)
        self.assertIn("H-TAXI-WIND-1 to H-VAULT-GUST-1", rep_gw)


if __name__ == '__main__':
    unittest.main()
