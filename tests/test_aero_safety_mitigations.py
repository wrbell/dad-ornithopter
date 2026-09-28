"""
Unit Tests: Aerodynamic Safety, Dynamic Stall & Blind-Spot Mitigation Hypotheses
================================================================================
Comprehensive test suite verifying:
  1. H-DSH-1: Dynamic Stall Delay & Camber Reflex Damping
  2. H-AC-SHIFT-1: Aerodynamic Center Shift Pre-bias
  3. H-SHED-1: Post-Stall Wake Shedding Dither (with H-45 bus brownout interlock)
  4. H-VEX-1: Ground Vortex Fountain Suppression (with LiDAR AGL gating)
  5. Hardened H-LSB-1:
     - H-43 Wind Shear preemptive arbitration
     - CFIT safety clamp (h_AGL < 5m blocks nose-down pitch trim)
     - Inertial stroke acceleration tare decoupling in _LSBDetector
  6. ALSS baseline seeds, CMTS champions, and genome update routing
  7. Discovery keyword and sensor routing in discovery_hypothesis_select
  8. advanced_physics_status() telemetry status reporting
"""

import sys
import os
import unittest
import math

# Ensure firmware directory is in python path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'firmware'))

import mco_wave_library as wl
from mco_flight_governor import LSBDetector


class TestAeroSafetyRegistration(unittest.TestCase):
    """Verifies catalog registration, config schema, bounds, and queues."""

    def test_registered_in_adv_new_hyps(self):
        adv_ids = {h['id'] for h in wl._ADV_NEW_HYPS}
        expected = {'H-LSB-1', 'H-DSH-1', 'H-AC-SHIFT-1', 'H-SHED-1', 'H-VEX-1'}
        for hyp_id in expected:
            self.assertIn(hyp_id, adv_ids, f"{hyp_id} missing from _ADV_NEW_HYPS")

    def test_registered_in_aero_hypotheses(self):
        aero_ids = {h['id'] for h in wl.AERO_HYPOTHESES}
        expected = {'H-LSB-1', 'H-DSH-1', 'H-AC-SHIFT-1', 'H-SHED-1', 'H-VEX-1'}
        for hyp_id in expected:
            self.assertIn(hyp_id, aero_ids, f"{hyp_id} missing from AERO_HYPOTHESES")

    def test_config_fields(self):
        dsh = next(h for h in wl.AERO_HYPOTHESES if h['id'] == 'H-DSH-1')
        self.assertEqual(dsh['config']['_scorer'], 'dsh_hysteresis_area')
        self.assertEqual(dsh['target'], 'smooth_flight')

        ac = next(h for h in wl.AERO_HYPOTHESES if h['id'] == 'H-AC-SHIFT-1')
        self.assertEqual(ac['config']['_scorer'], 'ac_shift_trim_deg')
        self.assertEqual(ac['target'], 'top_speed')

        shed = next(h for h in wl.AERO_HYPOTHESES if h['id'] == 'H-SHED-1')
        self.assertEqual(shed['config']['_scorer'], 'shed_dither_recovery_ms')
        self.assertEqual(shed['target'], 'recovery')

        vex = next(h for h in wl.AERO_HYPOTHESES if h['id'] == 'H-VEX-1')
        self.assertEqual(vex['config']['_scorer'], 'vex_recirculation_jitter_g')
        self.assertEqual(vex['target'], 'hover_hold')

    def test_alss_segments(self):
        expected_segments = {
            'dsh_stall_hysteresis_b': ('manoeuvre', 'H-DSH-1'),
            'ac_shift_compensation_b': ('cruise', 'H-AC-SHIFT-1'),
            'shed_wake_dither_a': ('recovery', 'H-SHED-1'),
            'vex_ground_recirculation_a': ('hover', 'H-VEX-1'),
        }
        for seg_name, (phase, hyp_id) in expected_segments.items():
            self.assertIn(seg_name, wl._ALSS_SEGMENTS, f"{seg_name} missing from _ALSS_SEGMENTS")
            seg = wl._ALSS_SEGMENTS[seg_name]
            self.assertEqual(seg[0], phase)
            self.assertEqual(seg[1], hyp_id)

        # Check curriculum inclusion
        self.assertIn('dsh_stall_hysteresis_b', wl._ALSS_CURRICULUM.get('B', []))
        self.assertIn('ac_shift_compensation_b', wl._ALSS_CURRICULUM.get('B', []))
        self.assertIn('shed_wake_dither_a', wl._ALSS_CURRICULUM.get('A', []))
        self.assertIn('vex_ground_recirculation_a', wl._ALSS_CURRICULUM.get('A', []))

    def test_discovery_bounds(self):
        expected_bounds = [
            'h_dsh_pitch_rate_dps',
            'h_dsh_reflex_deg',
            'h_ac_shift_bias_deg',
            'h_ac_sprint_q_pa',
            'h_shed_dither_deg',
            'h_shed_dither_hz',
            'h_vex_dihedral_deg',
            'h_vex_agl_threshold_m',
        ]
        for key in expected_bounds:
            self.assertIn(key, wl._EXPANDED_DISCOVERY_BOUNDS, f"{key} missing from bounds")
            lo, hi = wl._EXPANDED_DISCOVERY_BOUNDS[key]
            self.assertLessEqual(lo, hi)

    def test_cmts_champions(self):
        expected_champions = {
            'dynamic_stall_mitigation': 'H-DSH-1',
            'ac_shift_trim_hold': 'H-AC-SHIFT-1',
            'deep_stall_wake_shed': 'H-SHED-1',
            'ground_fountain_suppression': 'H-VEX-1',
        }
        for ctx, hyp_id in expected_champions.items():
            self.assertIn(ctx, wl._CMTS_CHAMPIONS, f"{ctx} missing from _CMTS_CHAMPIONS")
            self.assertEqual(wl._CMTS_CHAMPIONS[ctx]['hyp_id'], hyp_id)


class TestGenomeAdaptationRouting(unittest.TestCase):
    """Verifies genome evolution updates across stability and control dictionaries."""

    def test_dsh_genome_update(self):
        orig_pitch = wl._MW_PITCH.get('pitch_aoa', 0.0)
        orig_dih = wl._MW_STABILITY.get('stroke_dihedral', 0.0)
        wl._update_genome_from_trial('H-DSH-1', 0.20, {'V_ms': 10.0})
        self.assertGreater(wl._MW_PITCH.get('pitch_aoa', 0.0), orig_pitch)
        self.assertGreater(wl._MW_STABILITY.get('stroke_dihedral', 0.0), orig_dih)

    def test_ac_shift_genome_update(self):
        orig_pitch = wl._MW_PITCH.get('pitch_aoa', 0.0)
        orig_top_speed = wl._MW_PERFORMANCE.get('top_speed', 0.0)
        wl._update_genome_from_trial('H-AC-SHIFT-1', 0.15, {'V_ms': 18.0})
        self.assertGreater(wl._MW_PITCH.get('pitch_aoa', 0.0), orig_pitch)
        self.assertGreater(wl._MW_PERFORMANCE.get('top_speed', 0.0), orig_top_speed)

    def test_shed_genome_update(self):
        orig_pitch = wl._MW_PITCH.get('pitch_aoa', 0.0)
        orig_roll = wl._MW_ROLL.get('roll_amp', 0.0)
        orig_loss = wl._MW_RESILIENCE.get('single_pair_loss', 0.0)
        wl._update_genome_from_trial('H-SHED-1', 0.25, {'V_ms': 2.0})
        self.assertGreater(wl._MW_PITCH.get('pitch_aoa', 0.0), orig_pitch)
        self.assertGreater(wl._MW_ROLL.get('roll_amp', 0.0), orig_roll)
        self.assertGreater(wl._MW_RESILIENCE.get('single_pair_loss', 0.0), orig_loss)

    def test_vex_genome_update(self):
        orig_dih = wl._MW_STABILITY.get('stroke_dihedral', 0.0)
        orig_geom = wl._MW_STABILITY.get('geom_dihedral', 0.0)
        wl._update_genome_from_trial('H-VEX-1', 0.30, {'h_agl_m': 1.0})
        self.assertGreater(wl._MW_STABILITY.get('stroke_dihedral', 0.0), orig_dih)
        self.assertGreater(wl._MW_STABILITY.get('geom_dihedral', 0.0), orig_geom)


class TestDiscoverySelectorRouting(unittest.TestCase):
    """Verifies keyword and sensor-driven discovery selection."""

    def test_dsh_discovery(self):
        for kw in ('dynamic_stall', 'camber_reflex', 'dsh', 'stall_vortex'):
            h = wl.discovery_hypothesis_select(focus=kw)
            self.assertIsNotNone(h)
            self.assertEqual(h['id'], 'H-DSH-1')

        h_sensor = wl.discovery_hypothesis_select(sensors={'dynamic_stall_warning': True})
        self.assertIsNotNone(h_sensor)
        self.assertEqual(h_sensor['id'], 'H-DSH-1')

    def test_ac_shift_discovery(self):
        for kw in ('ac_shift', 'aerodynamic_center', 'center_shift', 'reflex_drift'):
            h = wl.discovery_hypothesis_select(focus=kw)
            self.assertIsNotNone(h)
            self.assertEqual(h['id'], 'H-AC-SHIFT-1')

        h_sensor = wl.discovery_hypothesis_select(sensors={'ac_shift_warning': True})
        self.assertIsNotNone(h_sensor)
        self.assertEqual(h_sensor['id'], 'H-AC-SHIFT-1')

    def test_shed_discovery(self):
        for kw in ('wake_shedding', 'deep_stall', 'dither_shed', 'shed_dither'):
            h = wl.discovery_hypothesis_select(focus=kw)
            self.assertIsNotNone(h)
            self.assertEqual(h['id'], 'H-SHED-1')

        h_sensor = wl.discovery_hypothesis_select(sensors={'deep_stall_flag': True})
        self.assertIsNotNone(h_sensor)
        self.assertEqual(h_sensor['id'], 'H-SHED-1')

    def test_vex_discovery(self):
        for kw in ('ground_vortex', 'vortex_fountain', 'ground_effect_recirculation', 'fountain_effect'):
            h = wl.discovery_hypothesis_select(focus=kw)
            self.assertIsNotNone(h)
            self.assertEqual(h['id'], 'H-VEX-1')

        h_sensor = wl.discovery_hypothesis_select(sensors={'ground_vortex_risk': True})
        self.assertIsNotNone(h_sensor)
        self.assertEqual(h_sensor['id'], 'H-VEX-1')


class TestAIObserverAndSafetyArbitration(unittest.TestCase):
    """Verifies online AI Observer triggers, priority queues, and preemptive safety interlocks."""

    def setUp(self):
        wl._ALC_SHARED['alert_log'].clear()
        wl._ALC_SHARED['priority_hyp_queue'].clear()
        det = wl._get_lsb_detector()
        if det:
            det.reset()

    def test_lsb_wind_shear_preemption(self):
        """H-43 Wind Shear or high sink rate preempts LSB AoA trimming."""
        det = wl._get_lsb_detector()
        self.assertIsNotNone(det)
        det.recent_burst_flags = [True, True, True]
        det.last_burst_rate = 0.50
        det.last_mean_variance = 0.030

        # Scenario 1: Severe sink w_sink = -3.5 m/s (Microburst)
        sensors = {
            'V_ms': 10.0,
            'w_sink': -3.5,
            'motor_currents_a': [14.0] * 8,
            'is_stroke_end': True,
            'flight_phase': 'approach',
            'h_agl_m': 15.0,
        }
        obs = wl.ai_observer_tick(sensors, {})
        # LSB alert should be preempted by H-43 wind shear / sink recovery
        self.assertNotIn('h_lsb_burst_alert', obs)
        lsb_queue = [q for q in wl._ALC_SHARED['priority_hyp_queue'] if q.get('hyp_id') == 'H-LSB-1']
        self.assertEqual(len(lsb_queue), 0, "LSB should be preempted by severe sink rate")

        # Scenario 2: Normal sink w_sink = -0.5 m/s -> LSB triggers
        wl._ALC_SHARED['priority_hyp_queue'].clear()
        sensors['w_sink'] = -0.5
        obs2 = wl.ai_observer_tick(sensors, {})
        self.assertIn('h_lsb_burst_alert', obs2)
        lsb_queue2 = [q for q in wl._ALC_SHARED['priority_hyp_queue'] if q.get('hyp_id') == 'H-LSB-1']
        self.assertGreater(len(lsb_queue2), 0)

    def test_lsb_cfit_safety_clamp(self):
        """Within 10m AGL, LSB detector prohibits nose-down pitch trim (CFIT prevention)."""
        det = LSBDetector()
        det.recent_burst_flags = [True, True, True, True]
        det.last_burst_rate = 0.60
        det.last_mean_variance = 0.035

        # Approach at 3m AGL (dangerously close to ground) -> Prohibit pitch-down, boost frequency
        st_low = det.get_status(mean_drag_n=8.0, is_approach=True, h_agl_m=3.0)
        self.assertFalse(st_low['allow_pitch_down'])
        self.assertEqual(st_low['retrim_advisory'], 'FREQUENCY_BOOST')
        self.assertEqual(st_low['status'], 'WARN_LSB_BURST_APPROACH_FREQ_BOOST_NO_PITCH_DOWN')

        # Approach at 15m AGL (safe altitude for nose-down re-attachment trim)
        st_safe = det.get_status(mean_drag_n=8.0, is_approach=True, h_agl_m=15.0)
        self.assertTrue(st_safe['allow_pitch_down'])
        self.assertEqual(st_safe['retrim_advisory'], 'AOA_TRIM')
        self.assertEqual(st_safe['status'], 'WARN_LSB_BURST_APPROACH_LIFT_RISK')

    def test_lsb_inertial_decoupling(self):
        """Inertial acceleration tare compensates for angular acceleration."""
        det_raw = LSBDetector(window_strokes=6, samples_per_stroke=10)
        det_decoupled = LSBDetector(window_strokes=6, samples_per_stroke=10)

        # High current variation across strokes purely driven by wing angular acceleration
        for stroke in range(6):
            accel_stroke = float(stroke * 15.0)  # stroke-varying acceleration
            for s in range(10):
                # Current scales with accel: I = I_base + (J/Kt)*accel
                inertial_current = (0.0018 / 0.082) * accel_stroke
                curr = 12.0 + inertial_current
                is_end = (s == 9)
                det_raw.add_sample([curr] * 8, is_stroke_end=is_end)
                det_decoupled.add_sample([curr] * 8, is_stroke_end=is_end, accels_rad_s2=[accel_stroke] * 8)

        st_raw = det_raw.get_status(mean_drag_n=5.0)
        st_dec = det_decoupled.get_status(mean_drag_n=5.0)

        # Decoupled detector removes inertial current, yielding significantly lower variance
        self.assertLess(st_dec['lsb_stroke_variance'], st_raw['lsb_stroke_variance'])
        self.assertAlmostEqual(st_dec['lsb_stroke_variance'], 0.0, places=4)

    def test_dsh_dynamic_stall_trigger(self):
        """H-DSH-1 triggers on pitch rate > 35 dps."""
        sensors = {
            'V_ms': 10.0,
            'pitch_rate_dps': 42.0,
            'is_stroke_end': False,
        }
        obs = wl.ai_observer_tick(sensors, {})
        self.assertTrue(obs.get('h_dsh_dynamic_stall_active', False))
        dsh_queue = [q for q in wl._ALC_SHARED['priority_hyp_queue'] if q.get('hyp_id') == 'H-DSH-1']
        self.assertGreater(len(dsh_queue), 0)

    def test_ac_shift_dynamic_pressure_trigger(self):
        """H-AC-SHIFT-1 triggers on dynamic pressure > 180 Pa."""
        # 18 m/s airspeed -> q = 0.5 * 1.225 * 18^2 = 198.45 Pa
        sensors = {
            'V_ms': 18.0,
            'lever_pct': 75.0,
            'is_stroke_end': False,
        }
        obs = wl.ai_observer_tick(sensors, {})
        self.assertTrue(obs.get('h_ac_shift_active', False))
        ac_queue = [q for q in wl._ALC_SHARED['priority_hyp_queue'] if q.get('hyp_id') == 'H-AC-SHIFT-1']
        self.assertGreater(len(ac_queue), 0)

    def test_shed_deep_stall_and_brownout_interlock(self):
        """H-SHED-1 triggers under deep stall, but suppresses if V_bus < 22.2V (H-45 interlock)."""
        # Case A: Low bus voltage (21.5V) -> Brownout hazard! Must suppress!
        sensors_low_v = {
            'V_ms': 2.0,
            'aoa_deg': 34.0,
            'bus_voltage_v': 21.5,
            'is_stroke_end': False,
        }
        obs_a = wl.ai_observer_tick(sensors_low_v, {})
        self.assertFalse(obs_a.get('h_shed_wake_dither_active', False))
        # Verify Tier-2 CRM alert was raised
        brownout_alerts = [a for a in wl._ALC_SHARED['alert_log'] if a.get('hyp_id') == 'H-45']
        self.assertGreater(len(brownout_alerts), 0)
        self.assertEqual(brownout_alerts[-1]['tier'], 2)

        # Case B: Nominal bus voltage (24.0V) -> Dither fires safely
        sensors_nominal_v = {
            'V_ms': 2.0,
            'aoa_deg': 34.0,
            'bus_voltage_v': 24.0,
            'is_stroke_end': False,
        }
        obs_b = wl.ai_observer_tick(sensors_nominal_v, {})
        self.assertTrue(obs_b.get('h_shed_wake_dither_active', False))
        shed_queue = [q for q in wl._ALC_SHARED['priority_hyp_queue'] if q.get('hyp_id') == 'H-SHED-1']
        self.assertGreater(len(shed_queue), 0)

    def test_vex_lidar_gating(self):
        """H-VEX-1 triggers within 2m AGL only when LiDAR agl_valid is True."""
        # Case A: AGL < 2.0m but LiDAR is INVALID (e.g. sensor failure or over water)
        sensors_invalid_lidar = {
            'V_ms': 1.0,
            'h_agl_m': 1.2,
            'agl_valid': False,
            'is_stroke_end': False,
            'downwash_recirculation': True,
        }
        obs_a = wl.ai_observer_tick(sensors_invalid_lidar, {})
        self.assertFalse(obs_a.get('h_vex_ground_vortex_active', False))

        # Case B: AGL < 2.0m and LiDAR is VALID -> Canting activates
        sensors_valid_lidar = {
            'V_ms': 1.0,
            'h_agl_m': 1.2,
            'agl_valid': True,
            'is_stroke_end': False,
            'downwash_recirculation': True,
        }
        obs_b = wl.ai_observer_tick(sensors_valid_lidar, {})
        self.assertTrue(obs_b.get('h_vex_ground_vortex_active', False))
        vex_queue = [q for q in wl._ALC_SHARED['priority_hyp_queue'] if q.get('hyp_id') == 'H-VEX-1']
        self.assertGreater(len(vex_queue), 0)


class TestTelemetryStatusOutput(unittest.TestCase):
    """Verifies advanced_physics_status telemetry string formatter."""

    def test_advanced_physics_status_formatting(self):
        status_txt = wl.advanced_physics_status()
        self.assertIsInstance(status_txt, str)
        self.assertIn("AERODYNAMIC SAFETY, DYNAMIC STALL & BLIND-SPOT MITIGATION", status_txt)
        self.assertIn("H-LSB-1", status_txt)
        self.assertIn("H-DSH-1", status_txt)
        self.assertIn("H-AC-SHIFT-1", status_txt)
        self.assertIn("H-SHED-1", status_txt)
        self.assertIn("H-VEX-1", status_txt)
        self.assertIn("CFIT clamp", status_txt)
        self.assertIn("H-45 brownout voltage clamp", status_txt)
        self.assertIn("LiDAR ground proximity interlock", status_txt)


if __name__ == '__main__':
    unittest.main()
