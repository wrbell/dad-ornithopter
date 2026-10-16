"""
Test Suite for H-FAST-CRUISE-2 (Audited 4-Wing Core Fast Cruise & Level Dash).
Verifies:
  1. Hypothesis definitions and registration in AERO_HYPOTHESES and _ADV_NEW_HYPS.
  2. ALSS segment registration ('fc2_audited_cruise_c') and curriculum presence in phase 'C'.
  3. Discovery selection routing for 'h_fast_cruise_2', 'audited_fast_cruise', 'audited_cruise', 'audited_4wing_core'.
  4. Kinematic waveform generator wave_audited_fast_cruise with 5-point adversarial safeguards:
     - Outboard stations 1 & 4 stowed flush along fuselage (85.0° sweep, flap 0.0°, reflex clamped).
     - Symmetric propulsive core stations 2 & 3 centered on CG with neutral pitch moment (pitch_moment_nm = 0.0).
     - Flapping sweep clamped to <= 70.0° (Lambda <= 25.0°) avoiding >50% thrust cosine loss and collision.
     - Reflex pitch angle clamped to [-2.0°, -1.0°] with -1.5° sweet spot eliminating lower-surface S1223 pocket scooping.
     - Micro-stroke amplitude clamped to [22.0°, 26.0°] keeping phase current safely below 20 A.
  5. Red-team safety aborts:
     - Pitch stick (|pitch| > 0.35), roll stick (|roll| > 0.35), yaw stick (|yaw| > 0.15).
     - Low airspeed (V < 12.0 m/s) stall abort.
     - Ground lockout (on_ground=True).
     - Pitot saturation fallback to Tactical EKF / GPS airspeed at dynamic pressure >= 950 Pa (V >= 39.4 m/s).
  6. Evaluator hook evaluate_fast_cruise_audited.
  7. Diagnostic ground station status command fast_cruise_status.
"""

import sys
import os
import unittest
import math

# Ensure firmware directory is in python path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'firmware'))

import mco_wave_library as wl


class TestHFastCruise2Registration(unittest.TestCase):
    """Verifies H-FAST-CRUISE-2 hypothesis data structures and ALSS integration."""

    def setUp(self):
        self.hyp = next((h for h in wl._ADV_NEW_HYPS if h['id'] == 'H-FAST-CRUISE-2'), None)

    def test_hypothesis_registered_in_adv_new_hyps(self):
        self.assertIsNotNone(self.hyp, "H-FAST-CRUISE-2 not found in _ADV_NEW_HYPS")
        self.assertEqual(self.hyp['target'], 'fast_cruise')
        self.assertIn('cruise', self.hyp['eval_phases'])
        self.assertIn('sprint', self.hyp['eval_phases'])

    def test_hypothesis_registered_in_aero_hypotheses(self):
        aero_ids = [h['id'] for h in wl.AERO_HYPOTHESES]
        self.assertIn('H-FAST-CRUISE-2', aero_ids)
        h = next(h for h in wl.AERO_HYPOTHESES if h['id'] == 'H-FAST-CRUISE-2')
        self.assertIn('Audited 4-wing core fast cruise', h['label'])
        self.assertIn('S1223', h['rationale'])

    def test_alss_segment_registered(self):
        self.assertIn('fc2_audited_cruise_c', wl._ALSS_SEGMENTS)
        seg = wl._ALSS_SEGMENTS['fc2_audited_cruise_c']
        self.assertEqual(seg[0], 'feathered_fast_cruise')
        self.assertEqual(seg[1], 'H-FAST-CRUISE-2')
        self.assertIn('fc2_audited_cruise_c', wl._ALSS_CURRICULUM.get('C', []))

    def test_discovery_selection_by_focus_keywords(self):
        for kw in ['h_fast_cruise_2', 'audited_fast_cruise', 'audited_cruise', 'audited_4wing_core']:
            h = wl.discovery_hypothesis_select(focus=kw)
            self.assertIsNotNone(h, f"Focus '{kw}' returned None")
            self.assertEqual(h['id'], 'H-FAST-CRUISE-2', f"Focus '{kw}' did not return H-FAST-CRUISE-2")


class TestHFastCruise2WaveGenerator(unittest.TestCase):
    """Verifies kinematic waveform generator wave_audited_fast_cruise and adversarial limits."""

    def test_nominal_cruise_waveform(self):
        res = wl.wave_audited_fast_cruise(t_sec=2.0, v_ms=21.0)
        self.assertTrue(res['fast_cruise_active'])
        self.assertFalse(res['aborted'])
        self.assertEqual(res['abort_reason'], 'NONE')
        self.assertEqual(len(res['pairs']), 4)

        # Outboard stations 1 & 4 (pair_1 and pair_4) must be stowed flush along fuselage
        for p_key in ['pair_1', 'pair_4']:
            pair = res['pairs'][p_key]
            self.assertEqual(pair['mode'], 'STOWED_FUSELAGE')
            self.assertEqual(pair['sweep_deg'], 85.0)
            self.assertEqual(pair['flap_deg'], 0.0)
            self.assertFalse(pair['active_flapping'])

        # Inboard core stations 2 & 3 (pair_2 and pair_3) must flap within safe micro-stroke band
        for p_key in ['pair_2', 'pair_3']:
            pair = res['pairs'][p_key]
            self.assertEqual(pair['mode'], 'PROPULSIVE_CORE')
            self.assertLessEqual(pair['sweep_deg'], 70.0)
            self.assertTrue(pair['active_flapping'])

        # Pitching moment must be balanced (symmetric core centered on CG)
        self.assertAlmostEqual(res['pitch_moment_nm'], 0.0, places=2)
        # Cascade clearance margin preserved
        self.assertGreater(res['cascade_clearance_margin_mm'], 40.0)
        # Power accounting within nominal cruise envelope (~380 W @ 20-22 m/s)
        self.assertLess(res['total_power_w'], 420.0)
        self.assertGreater(res['total_power_w'], 300.0)

    def test_sweep_clamping_to_safe_boundary(self):
        # Even if commanded sweep is 78°, core sweep must clamp <= 70.0°
        res = wl.wave_audited_fast_cruise(t_sec=1.5, sweep_deg=78.0, v_ms=20.0)
        self.assertLessEqual(res['sweep_deg'], 70.0)
        self.assertLessEqual(res['pairs']['pair_2']['sweep_deg'], 70.0)
        self.assertLessEqual(res['pairs']['pair_3']['sweep_deg'], 70.0)
        self.assertFalse(res['aborted'])

    def test_stroke_amplitude_fence(self):
        # Commanded amplitude 15° clamped to 22.0° min, 35° clamped to 26.0° max
        res_low = wl.wave_audited_fast_cruise(t_sec=1.0, flap_amp_deg=15.0, v_ms=20.0)
        self.assertAlmostEqual(res_low['flap_amp_deg'], 22.0, places=1)

        res_high = wl.wave_audited_fast_cruise(t_sec=1.0, flap_amp_deg=35.0, v_ms=20.0)
        self.assertAlmostEqual(res_high['flap_amp_deg'], 26.0, places=1)

    def test_reflex_pitch_angle_clamped(self):
        # Reflex pitch angle clamped within [-2.0, -1.0]
        res_low = wl.wave_audited_fast_cruise(t_sec=1.0, reflex_pitch_deg=-6.0, v_ms=20.0)
        self.assertAlmostEqual(res_low['reflex_pitch_deg'], -2.0, places=1)

        res_high = wl.wave_audited_fast_cruise(t_sec=1.0, reflex_pitch_deg=2.0, v_ms=20.0)
        self.assertAlmostEqual(res_high['reflex_pitch_deg'], -1.0, places=1)


class TestHFastCruise2SafetyAborts(unittest.TestCase):
    """Verifies red-team safety abort triggers and sensor fallbacks."""

    def test_pitch_stick_abort(self):
        res = wl.wave_audited_fast_cruise(t_sec=1.0, pitch_stick=0.38, v_ms=20.0)
        self.assertTrue(res['aborted'])
        self.assertFalse(res['fast_cruise_active'])
        self.assertIn('pitch_stick', res['abort_reason'])

    def test_roll_stick_abort(self):
        res = wl.wave_audited_fast_cruise(t_sec=1.0, roll_stick=-0.42, v_ms=20.0)
        self.assertTrue(res['aborted'])
        self.assertFalse(res['fast_cruise_active'])
        self.assertIn('roll_stick', res['abort_reason'])

    def test_yaw_stick_abort(self):
        res = wl.wave_audited_fast_cruise(t_sec=1.0, yaw_stick=0.20, v_ms=20.0)
        self.assertTrue(res['aborted'])
        self.assertFalse(res['fast_cruise_active'])
        self.assertIn('yaw_stick', res['abort_reason'])

    def test_low_airspeed_stall_abort(self):
        res = wl.wave_audited_fast_cruise(t_sec=1.0, v_ms=11.5)
        self.assertTrue(res['aborted'])
        self.assertFalse(res['fast_cruise_active'])
        self.assertIn('below min', res['abort_reason'])

    def test_ground_lockout_abort(self):
        res = wl.wave_audited_fast_cruise(t_sec=1.0, on_ground=True, v_ms=20.0)
        self.assertTrue(res['aborted'])
        self.assertFalse(res['fast_cruise_active'])
        self.assertIn('on_ground', res['abort_reason'])

    def test_pitot_saturation_sensor_fallback(self):
        # Above V >= 39.4 m/s (950 Pa), Pitot transducer saturates and flags EKF fallback
        res_normal = wl.wave_audited_fast_cruise(t_sec=1.0, v_ms=25.0)
        self.assertFalse(res_normal['pitot_saturated'])
        self.assertEqual(res_normal['airspeed_source'], 'PITOT_AIRSPEED')

        res_sat = wl.wave_audited_fast_cruise(t_sec=1.0, v_ms=40.0)
        self.assertTrue(res_sat['pitot_saturated'])
        self.assertEqual(res_sat['airspeed_source'], 'TACTICAL_EKF_GPS_FUSION')


class TestHFastCruise2EvaluatorAndStatus(unittest.TestCase):
    """Verifies evaluate_fast_cruise_audited and diagnostic command fast_cruise_status."""

    def test_evaluate_fast_cruise_audited_nominal(self):
        sensors = {
            'V_ms': 21.0,
            't_sec': 5.0,
            'pitch_stick': 0.0,
            'roll_stick': 0.0,
            'yaw_stick': 0.0,
            'on_ground': False,
        }
        res = wl.evaluate_fast_cruise_audited(sensors=sensors)
        self.assertEqual(res['hyp_id'], 'H-FAST-CRUISE-2')
        self.assertFalse(res['aborted'])
        self.assertTrue(res['safeguards_verified'])
        self.assertGreater(res['wh_per_km'], 0.0)

    def test_fast_cruise_status_output(self):
        report = wl.fast_cruise_status()
        self.assertIn('H-FAST-CRUISE-2', report)
        self.assertIn('AUDITED PROPULSIVE CORE ARCHITECTURE', report)
        self.assertIn('Stations 1 & 4 (Fuselage Boundary): STOWED FLUSH', report)
        self.assertIn('Stations 2 & 3 (Propulsive Core):   BALANCED ON CG', report)
        self.assertIn('Pitot Transducer Guard', report)
        self.assertIn('Stroke Amplitude Fence', report)
        self.assertIn('Motor Current Limit', report)
        self.assertIn('~380 W', report)
        self.assertIn('~600 W', report)


if __name__ == '__main__':
    unittest.main()
