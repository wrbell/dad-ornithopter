"""
Tests for Dynamic CG Compensation via Asymmetric Chevron Sweep (H-CHEVRON-2) Integration with ALC.

Verifies:
  1. Complete registration across AERO_HYPOTHESES, _hypothesis_queue, lever mode weights,
     ALSS segments, curriculum, SG tasks, expanded discovery bounds, and CMTS champions.
  2. Modality 1: AI Observer Trigger 55 for H-CHEVRON-2.
  3. Modality 2: Discovery Selector focus keywords and sensor triggers.
  4. Modality 3: ALSS segment execution and phase task mapping.
  5. Modality 4: Genome update routing to pitch, stability, performance, and endurance.
  6. Modality 5: CMTS champion query.
  7. Forward-sweep aerodynamics and aeroelastic wash-in divergence safety (pre-bias washout).
  8. AC shift dynamic tracking and servo stall prevention for forward CG shifts up to 65mm.
  9. Evaluator hook and ground station diagnostic reporting (chevron_cg_status).
"""

import unittest
import math
import sys
import os

# Add repo root to path
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

import firmware.mco_wave_library as mco


class TestHChevron2Registration(unittest.TestCase):
    """Test full registration of H-CHEVRON-2 across all system registries."""

    def test_aero_hypotheses_registration(self):
        hyp = next((h for h in mco.AERO_HYPOTHESES if h['id'] == 'H-CHEVRON-2'), None)
        self.assertIsNotNone(hyp, "H-CHEVRON-2 must be registered in AERO_HYPOTHESES")
        self.assertEqual(hyp['config']['fore_sweep_deg'], -9.0)
        self.assertEqual(hyp['config']['pitch_reflex_deg'], -1.2)
        self.assertEqual(hyp['config']['pitch_washout_deg'], -1.2)
        self.assertEqual(hyp['config']['cg_shift_max_m'], 0.058)
        self.assertEqual(hyp['config']['h_chevron2_load_split_ratio'], 0.70)
        self.assertTrue(hyp['config']['_h_chevron_2'])
        self.assertEqual(hyp['config']['_scorer'], 'cg_compensation_margin')

    def test_hypothesis_queue_registration(self):
        hyp = next((h for h in mco._hypothesis_queue if h['id'] == 'H-CHEVRON-2'), None)
        self.assertIsNotNone(hyp, "H-CHEVRON-2 must be in _hypothesis_queue")

    def test_lever_zone_mapping_and_weights(self):
        self.assertEqual(mco._HYP_TO_LEVER_ZONE.get('H-CHEVRON-2'), 'MAX_THRUST')
        self.assertIn('H-CHEVRON-2', mco._lever_mode_weights['MAX_THRUST'])
        self.assertGreaterEqual(mco._lever_mode_weights['MAX_THRUST']['H-CHEVRON-2'], 1.0)
        self.assertIn('H-CHEVRON-2', mco._lever_mode_weights['CRUISE'])
        self.assertGreaterEqual(mco._lever_mode_weights['CRUISE']['H-CHEVRON-2'], 1.0)

    def test_alss_curriculum_and_segments(self):
        self.assertIn('chevron2_cg_c', mco._ALSS_SEGMENTS)
        seg = mco._ALSS_SEGMENTS['chevron2_cg_c']
        self.assertEqual(seg[0], 'cg_compensation')
        self.assertEqual(seg[1], 'H-CHEVRON-2')
        self.assertIn('chevron2_cg_c', mco._ALSS_CURRICULUM['C'])

    def test_sg_tasks_registered(self):
        self.assertIn('cg_compensation_margin', mco._SG_TASKS)
        task = mco._SG_TASKS['cg_compensation_margin']
        self.assertEqual(task['direction'], +1)
        self.assertEqual(task['threshold'], 1.5)

    def test_phase_to_sg_task_mapping(self):
        self.assertIn('cg_compensation', mco._PHASE_TO_SG_TASK)
        tasks = mco._PHASE_TO_SG_TASK['cg_compensation']
        self.assertIn('cg_compensation_margin', tasks)

    def test_expanded_discovery_bounds(self):
        self.assertIn('h_chevron2_fore_sweep_deg', mco._EXPANDED_DISCOVERY_BOUNDS)
        self.assertEqual(mco._EXPANDED_DISCOVERY_BOUNDS['h_chevron2_fore_sweep_deg'], (-10.0, 0.0))
        self.assertIn('h_chevron2_aft_sweep_deg', mco._EXPANDED_DISCOVERY_BOUNDS)
        self.assertEqual(mco._EXPANDED_DISCOVERY_BOUNDS['h_chevron2_aft_sweep_deg'], (20.0, 50.0))
        self.assertIn('h_chevron2_pitch_reflex_deg', mco._EXPANDED_DISCOVERY_BOUNDS)
        self.assertEqual(mco._EXPANDED_DISCOVERY_BOUNDS['h_chevron2_pitch_reflex_deg'], (-3.0, 0.0))
        self.assertIn('h_chevron2_load_split_ratio', mco._EXPANDED_DISCOVERY_BOUNDS)
        self.assertEqual(mco._EXPANDED_DISCOVERY_BOUNDS['h_chevron2_load_split_ratio'], (0.50, 0.80))

    def test_cmts_champions(self):
        self.assertIn('cg_compensation', mco._CMTS_CHAMPIONS)
        champ = mco._CMTS_CHAMPIONS['cg_compensation']
        self.assertEqual(champ['hyp_id'], 'H-CHEVRON-2')
        self.assertEqual(champ['fore_sweep_deg'], -9.0)
        self.assertEqual(champ['pitch_reflex_deg'], -1.2)
        self.assertEqual(champ['cg_shift_absorbed_m'], 0.058)

    def test_gs_allowed_commands(self):
        self.assertIn('chevron_cg_status', mco._GS_ALLOWED_CMDS)
        self.assertTrue(callable(getattr(mco, 'chevron_cg_status')))


class TestHChevron2ALCIntegration(unittest.TestCase):
    """Test 5-Modality ALC integration for H-CHEVRON-2."""

    def test_modality1_ai_observer_trigger55(self):
        sensors = {
            'V_ms': 16.0,
            'cg_excursion_mm': 35.0,
            'pitch_tuck_detected': True,
        }
        obs = mco.ai_observer_tick(sensors, {})
        self.assertTrue(obs.get('h_chevron2_active'))
        queue = mco._ALC_SHARED['priority_hyp_queue']
        entry = next((e for e in queue if e.get('hyp_id') == 'H-CHEVRON-2'), None)
        self.assertIsNotNone(entry, "H-CHEVRON-2 should be armed into priority_hyp_queue")
        self.assertIn("Dynamic CG compensation active", entry['reason'])

    def test_modality2_discovery_selector_focus_keywords(self):
        for kw in ['h_chevron_2', 'h_chevron2', 'cg_compensation', 'cg_sweep', 'asymmetric_chevron', 'pitch_trim_neutrality']:
            h = mco.discovery_hypothesis_select(focus=kw)
            self.assertIsNotNone(h, f"Discovery selector should find H-CHEVRON-2 with focus='{kw}'")
            self.assertEqual(h['id'], 'H-CHEVRON-2')

    def test_modality2_discovery_selector_sensor_triggers(self):
        for s_key in ['h_chevron2_active', 'chevron2_cg_active', 'cg_compensation_active']:
            sensors = {s_key: True}
            h = mco.discovery_hypothesis_select(sensors=sensors)
            self.assertIsNotNone(h, f"Failed for sensor key {s_key}")
            self.assertEqual(h['id'], 'H-CHEVRON-2')

        # Threshold trigger
        sensors = {'cg_excursion_mm': 25.0}
        h = mco.discovery_hypothesis_select(sensors=sensors)
        self.assertIsNotNone(h)
        self.assertEqual(h['id'], 'H-CHEVRON-2')

    def test_modality4_genome_routing(self):
        initial_pitch = mco._MW_PITCH.get('pitch_aoa', 0.0)
        initial_stab = mco._MW_STABILITY.get('proverse_yaw', 0.0)
        initial_perf = mco._MW_PERFORMANCE.get('cruise_efficiency', 0.0)
        initial_endurance = mco._MW_ENDURANCE.get('sustained_wh_per_km', 0.0)

        sensors = {'V_ms': 16.0, 'cg_compensation_margin': 2.4}
        mco._update_genome_from_trial('H-CHEVRON-2', +0.22, sensors)

        self.assertGreaterEqual(mco._MW_PITCH.get('pitch_aoa', 0.0), initial_pitch)
        self.assertGreaterEqual(mco._MW_STABILITY.get('proverse_yaw', 0.0), initial_stab)
        self.assertGreaterEqual(mco._MW_PERFORMANCE.get('cruise_efficiency', 0.0), initial_perf)
        self.assertGreaterEqual(mco._MW_ENDURANCE.get('sustained_wh_per_km', 0.0), initial_endurance)


class TestHChevron2AerodynamicsAndSafety(unittest.TestCase):
    """Test CG compensation kinematics, servo holding torque margins, and wash-in safety."""

    def test_kinematics_and_washout(self):
        wave = mco.wave_chevron_cg_compensation(0.0, {'airspeed_ms': 14.0, 'cg_excursion_mm': 58.0})
        self.assertEqual(wave['station_sweeps_deg']['station_1'], -9.0)
        self.assertEqual(wave['station_sweeps_deg']['station_2'], 0.0)
        self.assertAlmostEqual(wave['pitch_reflex_deg'], -1.2, places=2)
        # At 14 m/s, v_ratio = 1.0, pitch washout = -1.20 deg
        self.assertAlmostEqual(wave['fore_pitch_washout_deg'], -1.20, places=2)
        self.assertTrue(wave['capstan_fleet_safe'])
        self.assertLessEqual(wave['capstan_fleet_angle_deg'], 14.0)
        self.assertTrue(wave['static_margin_safe'])
        self.assertGreaterEqual(wave['static_margin_pct'], 7.5)

    def test_extreme_forward_cg_compensation(self):
        # 58mm forward CG shift at max dash 21.5 m/s
        wave = mco.wave_chevron_cg_compensation(0.0, {'airspeed_ms': 21.5, 'cg_excursion_mm': 58.0})
        self.assertTrue(wave['servo_stall_prevented'])
        self.assertLessEqual(wave['pitch_servo_holding_torque_nm'], 0.48)
        self.assertLessEqual(wave['sweep_servo_holding_torque_nm'], 1.35)
        self.assertGreaterEqual(wave['pitch_servo_torque_sf'], 3.75)
        self.assertGreaterEqual(wave['sweep_servo_torque_sf'], 1.77)
        self.assertGreaterEqual(wave['cg_compensation_margin'], 1.5)
        self.assertTrue(wave['sprint_burst_active'])
        self.assertTrue(wave['capstan_fleet_safe'])
        self.assertTrue(wave['static_margin_safe'])

    def test_evaluator_and_gs_status(self):
        ev = mco.evaluate_chevron_cg({
            'V_ms': 17.2,
            'cg_excursion_mm': 55.0,
            'cg_compensation_margin': 2.3,
            'pitch_servo_holding_torque_nm': 0.25,
            'sweep_servo_holding_torque_nm': 0.70,
            'capstan_fleet_angle_deg': 9.0,
            'static_margin_pct': 7.6,
            'washin_divergence_margin': 2.4,
        })
        self.assertEqual(ev['hyp_id'], 'H-CHEVRON-2')
        self.assertTrue(ev['compensation_safe'])
        self.assertTrue(ev['servo_stall_prevented'])
        self.assertTrue(ev['capstan_fleet_safe'])
        self.assertTrue(ev['static_margin_safe'])
        self.assertTrue(ev['washin_safe'])
        self.assertGreaterEqual(ev['score_delta'], 0.18)

        status_report = mco.chevron_cg_status()
        self.assertIsInstance(status_report, str)
        self.assertIn("H-CHEVRON-2", status_report)
        self.assertIn("DYNAMIC CG COMPENSATION", status_report)
        self.assertIn("Capstan Cable Fleet Angle", status_report)
        self.assertIn("Static Margin", status_report)


if __name__ == '__main__':
    unittest.main()
