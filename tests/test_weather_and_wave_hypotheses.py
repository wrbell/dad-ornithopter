"""
Unit and Integration Tests for Adverse-Weather and Orographic Wave Hypotheses:
  - H-RAIN-LEV-1: Unsteady LEV Rain Penetration & Boundary Layer Circulation Boost
  - H-WAVE-LOITER-1: Energy-Throttled Orographic Mountain Wave Loiter

Verifies:
  1. Complete 5-modality ALC registration across AERO_HYPOTHESES, _hypothesis_queue,
     _SG_TASKS, _EXPANDED_DISCOVERY_BOUNDS, and _CMTS_CHAMPIONS.
  2. Modality 1: AI Observer triggers (#57 and #58) and priority queueing.
  3. Modality 2: Discovery Selector focus keyword routing and sensor triggers.
  4. Modality 3, 4, 5: Scoring tasks, genome bounds, and CMTS context champions.
  5. Deterministic safety governors:
     - UnsteadyLEVRainGovernor (frequency boost, 2.65 Hz cap, -1.5° reflex pitch bias, 25-35% lift recovery).
     - OrographicWaveLoiterGovernor (w_atm calculation, ground lockout, 90° sweep lock,
       1.1 Hz frequency derate, 87.5% aero / >70% net power savings, hysteresis, pilot breakout).
  6. Firmware integration into mco_main.py flight control loop.
  7. Ground station telemetry and diagnostic reporting in advanced_physics_status().
"""

import math
import unittest
import sys
import os

# Ensure repo root and firmware dir are in python path
REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
FIRMWARE_DIR = os.path.join(REPO_ROOT, 'firmware')
if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)
if FIRMWARE_DIR not in sys.path:
    sys.path.insert(0, FIRMWARE_DIR)

import mco_wave_library as mco
from mco_flight_governor import (
    UnsteadyLEVRainGovernor, govern_unsteady_lev_rain,
    OrographicWaveLoiterGovernor, govern_orographic_wave_loiter,
)
import mco_main


class TestHypothesesRegistration(unittest.TestCase):
    """Test full registration of H-RAIN-LEV-1 and H-WAVE-LOITER-1 across all registries."""

    def test_h_rain_lev_1_registered(self):
        hyp = next((h for h in mco.AERO_HYPOTHESES if h['id'] == 'H-RAIN-LEV-1'), None)
        self.assertIsNotNone(hyp, "H-RAIN-LEV-1 must be registered in AERO_HYPOTHESES")
        self.assertEqual(hyp['config']['freq_hz'], 2.45)
        self.assertEqual(hyp['config']['h_rain_lev_freq_boost_hz'], 0.25)
        self.assertTrue(hyp['config']['_h_rain_lev_1'])
        self.assertEqual(hyp['config']['_scorer'], 'rain_lift_recovery_pct')
        self.assertIn('rain', hyp['eval_phases'])

        q_hyp = next((h for h in mco._hypothesis_queue if h['id'] == 'H-RAIN-LEV-1'), None)
        self.assertIsNotNone(q_hyp, "H-RAIN-LEV-1 must be registered in _hypothesis_queue")

    def test_h_wave_loiter_1_registered(self):
        hyp = next((h for h in mco.AERO_HYPOTHESES if h['id'] == 'H-WAVE-LOITER-1'), None)
        self.assertIsNotNone(hyp, "H-WAVE-LOITER-1 must be registered in AERO_HYPOTHESES")
        self.assertEqual(hyp['config']['freq_hz'], 1.1)
        self.assertEqual(hyp['config']['sweep_deg'], 90.0)
        self.assertEqual(hyp['config']['h_wave_loiter_flap_freq_hz'], 1.1)
        self.assertTrue(hyp['config']['_h_wave_loiter_1'])
        self.assertEqual(hyp['config']['_scorer'], 'updraft_power_savings_pct')
        self.assertIn('mountain_wave', hyp['eval_phases'])

        q_hyp = next((h for h in mco._hypothesis_queue if h['id'] == 'H-WAVE-LOITER-1'), None)
        self.assertIsNotNone(q_hyp, "H-WAVE-LOITER-1 must be registered in _hypothesis_queue")

    def test_scoring_tasks_registered(self):
        self.assertIn('rain_lift_recovery_pct', mco._SG_TASKS)
        task_rain = mco._SG_TASKS['rain_lift_recovery_pct']
        self.assertEqual(task_rain['axis'], 'resilience')
        self.assertEqual(task_rain['direction'], +1)
        self.assertEqual(task_rain['threshold'], 25.0)

        self.assertIn('updraft_power_savings_pct', mco._SG_TASKS)
        task_wave = mco._SG_TASKS['updraft_power_savings_pct']
        self.assertEqual(task_wave['axis'], 'endurance')
        self.assertEqual(task_wave['direction'], +1)
        self.assertEqual(task_wave['threshold'], 70.0)

    def test_discovery_genome_bounds(self):
        self.assertIn('h_rain_lev_freq_boost_hz', mco._EXPANDED_DISCOVERY_BOUNDS)
        self.assertEqual(mco._EXPANDED_DISCOVERY_BOUNDS['h_rain_lev_freq_boost_hz'], (0.10, 0.45))

        self.assertIn('h_wave_loiter_flap_freq_hz', mco._EXPANDED_DISCOVERY_BOUNDS)
        self.assertEqual(mco._EXPANDED_DISCOVERY_BOUNDS['h_wave_loiter_flap_freq_hz'], (0.80, 1.60))

    def test_cmts_champions(self):
        self.assertIn('rain_lev_circulation', mco._CMTS_CHAMPIONS)
        champ_rain = mco._CMTS_CHAMPIONS['rain_lev_circulation']
        self.assertEqual(champ_rain['hyp_id'], 'H-RAIN-LEV-1')
        self.assertGreaterEqual(champ_rain['rain_lift_recovery_pct'], 25.0)

        self.assertIn('mountain_wave_loiter', mco._CMTS_CHAMPIONS)
        champ_wave = mco._CMTS_CHAMPIONS['mountain_wave_loiter']
        self.assertEqual(champ_wave['hyp_id'], 'H-WAVE-LOITER-1')
        self.assertGreaterEqual(champ_wave['net_power_savings_pct'], 70.0)


class TestModality1AIObserver(unittest.TestCase):
    """Test AI Observer Triggers #57 and #58."""

    def setUp(self):
        mco._ALC_SHARED['priority_hyp_queue'].clear()

    def test_observer_57_rain_lev_trigger(self):
        sensors = {'rain_detected': True, 'rain_rate_mm_h': 3.5}
        obs = mco.ai_observer_tick(sensors, {})
        self.assertTrue(obs.get('unsteady_rain_lev_active'))
        queue_ids = [item['hyp_id'] for item in mco._ALC_SHARED['priority_hyp_queue']]
        self.assertIn('H-RAIN-LEV-1', queue_ids)

    def test_observer_57_rain_inactive(self):
        sensors = {'rain_detected': False, 'rain_rate_mm_h': 0.0}
        obs = mco.ai_observer_tick(sensors, {})
        self.assertFalse(obs.get('unsteady_rain_lev_active', False))

    def test_observer_58_wave_loiter_trigger(self):
        sensors = {'on_ground': False, 'h_agl_m': 50.0, 'w_atm_ms': 3.2}
        obs = mco.ai_observer_tick(sensors, {})
        self.assertTrue(obs.get('orographic_wave_loiter_active'))
        queue_ids = [item['hyp_id'] for item in mco._ALC_SHARED['priority_hyp_queue']]
        self.assertIn('H-WAVE-LOITER-1', queue_ids)

    def test_observer_58_wave_loiter_ground_lockout(self):
        sensors = {'on_ground': True, 'h_agl_m': 0.0, 'w_atm_ms': 4.0}
        obs = mco.ai_observer_tick(sensors, {})
        self.assertFalse(obs.get('orographic_wave_loiter_active', False))


class TestModality2DiscoverySelector(unittest.TestCase):
    """Test Discovery Hypothesis Selection via FOCUS keywords and sensor triggers."""

    def test_focus_keywords_rain(self):
        for kw in ['rain_boost', 'unsteady_rain', 'rain_penetration', 'rain_lev', 'h_rain_lev_1']:
            hyp = mco.discovery_hypothesis_select(focus=kw)
            self.assertIsNotNone(hyp, f"Focus '{kw}' must return a hypothesis")
            self.assertEqual(hyp['id'], 'H-RAIN-LEV-1', f"Focus '{kw}' must route to H-RAIN-LEV-1")

    def test_focus_keywords_wave(self):
        for kw in ['mountain_wave', 'orographic_loiter', 'wave_soaring', 'ridge_lift', 'h_wave_loiter_1']:
            hyp = mco.discovery_hypothesis_select(focus=kw)
            self.assertIsNotNone(hyp, f"Focus '{kw}' must return a hypothesis")
            self.assertEqual(hyp['id'], 'H-WAVE-LOITER-1', f"Focus '{kw}' must route to H-WAVE-LOITER-1")

    def test_sensor_triggers_rain_lev(self):
        sensors = {'rain_detected': True, 'rain_rate_mm_h': 2.5}
        hyp = mco.discovery_hypothesis_select(sensors=sensors)
        self.assertEqual(hyp['id'], 'H-RAIN-LEV-1')

    def test_sensor_triggers_wave_loiter(self):
        sensors = {'on_ground': False, 'h_agl_m': 100.0, 'w_atm_ms': 3.0}
        hyp = mco.discovery_hypothesis_select(sensors=sensors)
        self.assertEqual(hyp['id'], 'H-WAVE-LOITER-1')


class TestUnsteadyLEVRainGovernor(unittest.TestCase):
    """Test UnsteadyLEVRainGovernor deterministic physics & controls."""

    def test_rain_inactive(self):
        gov = UnsteadyLEVRainGovernor()
        telem = gov.update(dt_s=0.02, rain_detected=False, rain_rate_mm_h=0.0, base_freq_hz=2.2)
        self.assertFalse(telem['rain_lev_active'])
        self.assertEqual(telem['effective_freq_hz'], 2.2)
        self.assertEqual(telem['pitch_reflex_deg'], 0.0)
        self.assertEqual(telem['rain_lift_recovery_pct'], 0.0)
        self.assertEqual(telem['status'], 'RAIN_LEV_INACTIVE')

    def test_rain_active_boost_and_pitch_bias(self):
        gov = UnsteadyLEVRainGovernor(freq_boost_hz=0.25, max_freq_hz=2.65, reflex_bias_deg=-1.5)
        wing_cmds = [{'pitch_deg': 5.0}, {'pitch_deg': 5.0}]
        result = {}
        telem = gov.update(
            dt_s=0.02, rain_detected=True, rain_rate_mm_h=2.0,
            base_freq_hz=2.2, wing_commands=wing_cmds, result=result
        )
        self.assertTrue(telem['rain_lev_active'])
        self.assertEqual(telem['effective_freq_hz'], 2.45)
        self.assertEqual(telem['pitch_reflex_deg'], -1.5)
        self.assertGreaterEqual(telem['rain_lift_recovery_pct'], 25.0)
        self.assertLessEqual(telem['rain_lift_recovery_pct'], 35.0)
        self.assertEqual(wing_cmds[0]['pitch_deg'], 3.5)  # 5.0 - 1.5
        self.assertEqual(result['freq_hz'], 2.45)

    def test_rain_max_frequency_clamping(self):
        gov = UnsteadyLEVRainGovernor(freq_boost_hz=0.25, max_freq_hz=2.65)
        telem = gov.update(dt_s=0.02, rain_detected=True, base_freq_hz=2.50)
        self.assertEqual(telem['effective_freq_hz'], 2.65)  # 2.50 + 0.25 = 2.75 clamped to 2.65

    def test_helper_function_rain(self):
        result = {}
        telem = govern_unsteady_lev_rain(
            dt_s=0.02, rain_detected=True, rain_rate_mm_h=4.0,
            base_freq_hz=2.2, result=result
        )
        self.assertTrue(telem['rain_lev_active'])
        self.assertEqual(result['rain_lev_active'], True)
        self.assertEqual(result['rain_freq_boost_hz'], 0.25)


class TestOrographicWaveLoiterGovernor(unittest.TestCase):
    """Test OrographicWaveLoiterGovernor deterministic physics, sweep lock, power savings & breakout."""

    def test_w_atm_computation(self):
        gov = OrographicWaveLoiterGovernor()
        # Level flight (theta = 0): w_atm = vz
        w = gov.compute_w_atm(vz_ms=3.0, V_ias_ms=12.0, pitch_deg=0.0)
        self.assertAlmostEqual(w, 3.0, places=3)

        # Climbing at 5 deg pitch: vz = 2.0, V_ias = 15.0
        # w_atm = 2.0 - 15.0 * sin(5 deg) = 2.0 - 1.307 = 0.693 m/s
        theta_rad = math.radians(5.0)
        expected_w = 2.0 - 15.0 * math.sin(theta_rad)
        w = gov.compute_w_atm(vz_ms=2.0, V_ias_ms=15.0, pitch_deg=5.0)
        self.assertAlmostEqual(w, expected_w, places=3)

    def test_ground_lockout(self):
        gov = OrographicWaveLoiterGovernor()
        telem = gov.update(
            dt_s=0.02, V_ias_ms=0.0, vz_ms=4.0, pitch_deg=0.0,
            on_ground=True, base_freq_hz=2.2
        )
        self.assertFalse(telem['wave_loiter_active'])
        self.assertEqual(telem['status'], 'WAVE_LOITER_INACTIVE_ON_GROUND')

    def test_engagement_and_power_savings(self):
        gov = OrographicWaveLoiterGovernor(engage_updraft_ms=2.5, throttled_freq_hz=1.1, nominal_freq_hz=2.2)
        wing_cmds = [{'sweep_deg': 60.0}, {'sweep_deg': 70.0}]
        result = {}
        telem = gov.update(
            dt_s=0.02, V_ias_ms=10.0, vz_ms=3.5, pitch_deg=0.0,
            on_ground=False, base_freq_hz=2.2, wing_commands=wing_cmds, result=result
        )
        self.assertTrue(telem['wave_loiter_active'])
        self.assertEqual(telem['sweep_lock_deg'], 90.0)
        self.assertEqual(wing_cmds[0]['sweep_deg'], 90.0)
        self.assertEqual(wing_cmds[1]['sweep_deg'], 90.0)
        self.assertEqual(telem['flapping_freq_hz'], 1.1)

        # Aero power cut = 1 - (1.1 / 2.2)^3 = 1 - 0.125 = 87.5%
        self.assertAlmostEqual(telem['flapping_power_reduction_pct'], 87.5, delta=0.5)
        # Net electrical savings > 70%
        self.assertGreaterEqual(telem['net_power_savings_pct'], 70.0)
        self.assertEqual(result['wave_loiter_active'], True)
        self.assertEqual(result['freq_hz'], 1.1)

    def test_hysteresis_behavior(self):
        gov = OrographicWaveLoiterGovernor(engage_updraft_ms=2.5, disengage_updraft_ms=1.5)
        # 1. Engage at 2.8 m/s updraft
        t1 = gov.update(vz_ms=2.8, pitch_deg=0.0, on_ground=False)
        self.assertTrue(t1['wave_loiter_active'])

        # 2. Remains active in lull down to 2.0 m/s (above disengage 1.5 m/s)
        t2 = gov.update(vz_ms=2.0, pitch_deg=0.0, on_ground=False)
        self.assertTrue(t2['wave_loiter_active'])

        # 3. Disengages when updraft drops below 1.5 m/s
        t3 = gov.update(vz_ms=1.2, pitch_deg=0.0, on_ground=False)
        self.assertFalse(t3['wave_loiter_active'])

    def test_pilot_breakout_throttle(self):
        gov = OrographicWaveLoiterGovernor(engage_updraft_ms=2.5)
        # Engage first
        gov.update(vz_ms=3.0, pitch_deg=0.0, throttle_frac=0.4, on_ground=False)
        self.assertTrue(gov.active)

        # Pilot pushes throttle to 75%
        telem = gov.update(vz_ms=3.0, pitch_deg=0.0, throttle_frac=0.75, on_ground=False)
        self.assertFalse(telem['wave_loiter_active'])
        self.assertTrue(telem['pilot_breakout'])
        self.assertEqual(telem['status'], 'WAVE_LOITER_PILOT_BREAKOUT')

    def test_pilot_breakout_pitch_stick(self):
        gov = OrographicWaveLoiterGovernor(engage_updraft_ms=2.5)
        # Engage first
        gov.update(vz_ms=3.0, pitch_deg=0.0, pilot_pitch_stick=0.0, on_ground=False)
        self.assertTrue(gov.active)

        # Pilot pulls back on stick > 0.20
        telem = gov.update(vz_ms=3.0, pitch_deg=0.0, pilot_pitch_stick=0.35, on_ground=False)
        self.assertFalse(telem['wave_loiter_active'])
        self.assertTrue(telem['pilot_breakout'])
        self.assertEqual(telem['status'], 'WAVE_LOITER_PILOT_BREAKOUT')


class TestMcoMainIntegration(unittest.TestCase):
    """Test integration into firmware/mco_main.py flight control loop."""

    def test_imports_in_mco_main(self):
        self.assertTrue(hasattr(mco_main, 'govern_unsteady_lev_rain'))
        self.assertTrue(hasattr(mco_main, 'govern_orographic_wave_loiter'))

    def test_kinematic_flight_tick_with_governors(self):
        # Create synthetic flight control tick result dict
        res = {
            'freq_hz': 2.2,
            'amp_deg': 70.0,
            'sweep_deg': 60.0,
            'wing_commands': [{'pitch_deg': 4.0, 'sweep_deg': 60.0}]
        }

        # Step 9d: Rain governor tick
        rain_telem = mco_main.govern_unsteady_lev_rain(
            dt_s=0.02, rain_detected=True, rain_rate_mm_h=2.5,
            base_freq_hz=res['freq_hz'], wing_commands=res['wing_commands'],
            result=res
        )
        self.assertTrue(rain_telem['rain_lev_active'])
        self.assertEqual(res['freq_hz'], 2.45)
        self.assertEqual(res['wing_commands'][0]['pitch_deg'], 2.5)  # 4.0 - 1.5

        # Step 9e: Wave loiter governor tick
        wave_telem = mco_main.govern_orographic_wave_loiter(
            dt_s=0.02, V_ias_ms=11.0, vz_ms=3.5, pitch_deg=0.0,
            throttle_frac=0.5, pilot_pitch_stick=0.0, on_ground=False,
            base_freq_hz=res['freq_hz'], wing_commands=res['wing_commands'],
            result=res
        )
        self.assertTrue(wave_telem['wave_loiter_active'])
        self.assertEqual(res['wing_commands'][0]['sweep_deg'], 90.0)
        self.assertEqual(res['freq_hz'], 1.1)


class TestGroundStationReporting(unittest.TestCase):
    """Test ground station telemetry and diagnostic reporting."""

    def test_advanced_physics_status_contains_new_hypotheses(self):
        report = mco.advanced_physics_status()
        self.assertIn('H-RAIN-LEV-1', report)
        self.assertIn('H-WAVE-LOITER-1', report)
        self.assertIn('Unsteady LEV Rain Penetration', report)
        self.assertIn('Energy-Throttled Mountain Wave Loiter', report)


if __name__ == '__main__':
    unittest.main()
