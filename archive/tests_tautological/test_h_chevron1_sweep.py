"""
Tests for Tandem Forward-Aft Chevron Sweep (H-CHEVRON-1) Integration with ALC.

Verifies:
  1. Complete registration across AERO_HYPOTHESES, queue, lever mode weights,
     ALSS segments, curriculum, SG tasks, expanded discovery bounds, and CMTS champions.
  2. Modality 1: AI Observer Trigger 54 for H-CHEVRON-1.
  3. Modality 2: Discovery Selector focus keywords and sensor triggers.
  4. Modality 3: ALSS segment execution and phase task mapping.
  5. Modality 4: Genome update routing to endurance, performance, pitch, and stability.
  6. Modality 5: CMTS champion query.
  7. Forward-sweep aerodynamics and aeroelastic wash-in divergence safety (pre-bias washout).
  8. Neutral point zero-tuck pitch moment cancellation.
  9. Ram-air forced convective cooling governor back-off logic (V >= 11 m/s, I <= 5.5A).
 10. Kinematics wave generator, evaluator hook, and ground station diagnostic reporting.
"""

import unittest
import math
import sys
import os

# Add repo root to path
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

import firmware.mco_wave_library as mco


class TestHChevron1Registration(unittest.TestCase):
    """Test full registration of H-CHEVRON-1 across all system registries."""

    def test_aero_hypotheses_registration(self):
        hyp = next((h for h in mco.AERO_HYPOTHESES if h['id'] == 'H-CHEVRON-1'), None)
        self.assertIsNotNone(hyp, "H-CHEVRON-1 must be registered in AERO_HYPOTHESES")
        self.assertEqual(hyp['config']['fore_sweep_deg'], -14.0)
        self.assertEqual(hyp['config']['pitch_washout_deg'], -1.68)
        self.assertTrue(hyp['config']['ram_air_cooling_enabled'])

    def test_hypothesis_queue_registration(self):
        hyp = next((h for h in mco._hypothesis_queue if h['id'] == 'H-CHEVRON-1'), None)
        self.assertIsNotNone(hyp, "H-CHEVRON-1 must be in _hypothesis_queue")

    def test_lever_zone_mapping_and_weights(self):
        self.assertEqual(mco._HYP_TO_LEVER_ZONE.get('H-CHEVRON-1'), 'MAX_THRUST')
        self.assertIn('H-CHEVRON-1', mco._lever_mode_weights['MAX_THRUST'])
        self.assertGreaterEqual(mco._lever_mode_weights['MAX_THRUST']['H-CHEVRON-1'], 1.0)
        self.assertIn('H-CHEVRON-1', mco._lever_mode_weights['CRUISE'])
        self.assertGreaterEqual(mco._lever_mode_weights['CRUISE']['H-CHEVRON-1'], 1.0)

    def test_alss_curriculum_and_segments(self):
        self.assertIn('chevron1_sweep_c', mco._ALSS_SEGMENTS)
        seg = mco._ALSS_SEGMENTS['chevron1_sweep_c']
        self.assertEqual(seg[0], 'chevron_sweep')
        self.assertEqual(seg[1], 'H-CHEVRON-1')
        self.assertIn('chevron1_sweep_c', mco._ALSS_CURRICULUM['C'])

    def test_sg_tasks_registered(self):
        for task in ['chevron_cruise_efficiency', 'washin_divergence_margin', 'pitch_trim_neutrality_nm']:
            self.assertIn(task, mco._SG_TASKS, f"{task} must be in _SG_TASKS")

    def test_expanded_discovery_bounds(self):
        self.assertIn('h_chevron1_fore_sweep_deg', mco._EXPANDED_DISCOVERY_BOUNDS)
        self.assertEqual(mco._EXPANDED_DISCOVERY_BOUNDS['h_chevron1_fore_sweep_deg'], (-14.0, 0.0))
        self.assertIn('h_chevron1_aft_sweep_deg', mco._EXPANDED_DISCOVERY_BOUNDS)
        self.assertEqual(mco._EXPANDED_DISCOVERY_BOUNDS['h_chevron1_aft_sweep_deg'], (20.0, 50.0))

    def test_cmts_champions(self):
        self.assertIn('chevron_sweep', mco._CMTS_CHAMPIONS)
        champ = mco._CMTS_CHAMPIONS['chevron_sweep']
        self.assertEqual(champ['hyp_id'], 'H-CHEVRON-1')
        self.assertEqual(champ['fore_sweep_deg'], -14.0)

    def test_gs_allowed_commands(self):
        self.assertIn('chevron_status', mco._GS_ALLOWED_CMDS)
        self.assertTrue(callable(getattr(mco, 'chevron_status')))


class TestHChevron1ALCIntegration(unittest.TestCase):
    """Test 5-Modality ALC integration for H-CHEVRON-1."""

    def test_modality1_ai_observer_trigger54(self):
        sensors = {
            'V_ms': 12.5,
            'lever_pct': 75.0,
            'chevron_sweep_active': True,
        }
        obs = mco.ai_observer_tick(sensors, {})
        self.assertTrue(obs.get('h_chevron1_active'))
        queue = mco._ALC_SHARED['priority_hyp_queue']
        entry = next((e for e in queue if e.get('hyp_id') == 'H-CHEVRON-1'), None)
        self.assertIsNotNone(entry, "H-CHEVRON-1 should be armed into priority_hyp_queue")
        self.assertIn("Progressive chevron sweep", entry['reason'])

    def test_modality2_discovery_selector_focus_keywords(self):
        for kw in ['h_chevron_1', 'h_chevron1', 'chevron_sweep', 'diamond_sweep', 'vortex_ladder', 'forward_aft_sweep']:
            h = mco.discovery_hypothesis_select(focus=kw)
            self.assertIsNotNone(h, f"Discovery selector should find H-CHEVRON-1 with focus='{kw}'")
            self.assertEqual(h['id'], 'H-CHEVRON-1')

    def test_modality2_discovery_selector_sensor_trigger(self):
        sensors = {'h_chevron1_active': True}
        h = mco.discovery_hypothesis_select(sensors=sensors)
        self.assertIsNotNone(h)
        self.assertEqual(h['id'], 'H-CHEVRON-1')

    def test_modality4_genome_routing(self):
        initial_endurance = mco._MW_ENDURANCE.get('sustained_wh_per_km', 0.0)
        initial_perf = mco._MW_PERFORMANCE.get('cruise_efficiency', 0.0)
        initial_pitch = mco._MW_PITCH.get('pitch_aoa', 0.0)
        initial_stab = mco._MW_STABILITY.get('proverse_yaw', 0.0)

        # Apply positive trial delta
        sensors = {'V_ms': 13.0, 'wh_per_km': 1.95}
        mco._update_genome_from_trial('H-CHEVRON-1', +0.20, sensors)

        self.assertGreaterEqual(mco._MW_ENDURANCE.get('sustained_wh_per_km', 0.0), initial_endurance)
        self.assertGreaterEqual(mco._MW_PERFORMANCE.get('cruise_efficiency', 0.0), initial_perf)
        self.assertGreaterEqual(mco._MW_PITCH.get('pitch_aoa', 0.0), initial_pitch)
        self.assertGreaterEqual(mco._MW_STABILITY.get('proverse_yaw', 0.0), initial_stab)


class TestHChevron1AerodynamicsAndCooling(unittest.TestCase):
    """Test aerodynamic safety, wash-in divergence margin, and ram-air cooling back-off."""

    def test_station1_locked_sweep_and_washout(self):
        wave = mco.wave_chevron_sweep(0.0, {'airspeed_ms': 14.0})
        self.assertEqual(wave['station_sweeps_deg']['station_1'], -14.0)
        # At 14 m/s, pitch washout = -0.12 * 14 * 1.0 = -1.68 deg
        self.assertAlmostEqual(wave['fore_pitch_washout_deg'], -1.68, places=2)
        self.assertGreaterEqual(wave['washin_divergence_margin'], 1.8)

    def test_neutral_point_zero_tuck_pitch_trim(self):
        wave = mco.wave_chevron_sweep(0.0, {'airspeed_ms': 12.0})
        self.assertTrue(wave['zero_tuck_balanced'])
        self.assertLessEqual(abs(wave['net_pitch_trim_residual_nm']), 0.15)

    def test_tip_clearance_increase(self):
        wave = mco.wave_chevron_sweep(0.0, {'fore_sweep_deg': -14.0})
        # Baseline is 16.5cm, with -14 deg sweep should be ~27.4cm
        self.assertGreaterEqual(wave['tip_clearance_cm'], 26.0)
        self.assertAlmostEqual(wave['tip_clearance_cm'], 27.4, delta=0.5)

    def test_ram_air_cooling_backoff_at_high_speed(self):
        # High speed cruise V = 12.0 m/s >= 11.0 m/s, bus current 3.2A <= 5.5A
        wave = mco.wave_chevron_sweep(0.0, {'airspeed_ms': 12.0, 'bus_current_a': 3.2, 'hold_duration_s': 200.0})
        self.assertTrue(wave['ram_air_cooling_active'])
        self.assertTrue(math.isinf(wave['max_sustained_duration_s']))
        self.assertTrue(wave['holding_governor_backed_off'])
        self.assertFalse(wave['thermal_warning'])
        self.assertLessEqual(wave['casing_temp_c'], 48.0)

    def test_ram_air_cooling_static_governor_at_low_speed(self):
        # Low speed loiter V = 8.0 m/s < 11.0 m/s -> static 120s governor enforced
        wave = mco.wave_chevron_sweep(0.0, {'airspeed_ms': 8.0, 'bus_current_a': 3.2, 'hold_duration_s': 50.0})
        self.assertFalse(wave['ram_air_cooling_active'])
        self.assertEqual(wave['max_sustained_duration_s'], 120.0)
        self.assertFalse(wave['holding_governor_backed_off'])
        self.assertFalse(wave['thermal_warning'])

        # Over-duration triggers warning
        wave_over = mco.wave_chevron_sweep(0.0, {'airspeed_ms': 8.0, 'bus_current_a': 3.2, 'hold_duration_s': 130.0})
        self.assertTrue(wave_over['thermal_warning'])

    def test_ram_air_cooling_overcurrent_protection(self):
        # High speed V = 14.0 m/s but overcurrent bus current = 6.0A > 5.5A
        wave = mco.wave_chevron_sweep(0.0, {'airspeed_ms': 14.0, 'bus_current_a': 6.0, 'hold_duration_s': 10.0})
        self.assertFalse(wave['ram_air_cooling_active'])
        self.assertEqual(wave['max_sustained_duration_s'], 120.0)
        self.assertTrue(wave['thermal_warning'])

    def test_evaluator_and_gs_status(self):
        ev = mco.evaluate_chevron_sweep({
            'V_ms': 13.0,
            'wh_per_km': 1.95,
            'washin_divergence_margin': 2.4,
            'pitch_trim_neutrality_nm': 0.045,
            'casing_temp_c': 42.0,
            'bus_current_a': 3.2,
        })
        self.assertEqual(ev['hyp_id'], 'H-CHEVRON-1')
        self.assertTrue(ev['cooling_effective'])
        self.assertTrue(ev['washin_safe'])
        self.assertTrue(ev['trim_neutral'])
        self.assertGreaterEqual(ev['score_delta'], 0.15)

        status_report = mco.chevron_status()
        self.assertIsInstance(status_report, str)
        self.assertIn("H-CHEVRON-1", status_report)
        self.assertIn("UNLIMITED (Forced Convection)", status_report)


if __name__ == '__main__':
    unittest.main()
