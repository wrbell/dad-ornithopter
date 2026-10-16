"""
Unit Tests: H-88 Dual Vectoring & Software-Only Recoil Suite
============================================================
Tests coverage for:
  - H-88A: Odonata Dynamic Harmonic Stroke-Plane Vectoring
  - H-88B: Odonata Quasi-Steady Differential Sweep Bias Vectoring
  - Software-Only Reversal Recoil Engine (Trajectory Smoothing & Aero-Spring Advance)
  - Salvaged Biomimetic Hypotheses: H-91 (Clap-Fling), H-108 (Flare), H-101 (Anti-Nodding)
"""

import sys
import os
import unittest
import math

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'firmware'))

import mco_wave_library as wl
import mco_flight_governor as fg


class TestH88CatalogAndRegistration(unittest.TestCase):
    """Verify hypothesis catalog registration and metadata for H-88A and H-88B."""

    def test_h88a_and_h88b_registered_in_adv_new_hyps(self):
        adv_ids = {h['id'] for h in wl._ADV_NEW_HYPS}
        self.assertIn('H-88A', adv_ids)
        self.assertIn('H-88B', adv_ids)

    def test_h88a_metadata(self):
        h = next(h for h in wl._ADV_NEW_HYPS if h['id'] == 'H-88A')
        self.assertEqual(h['target'], 'manoeuvre')
        self.assertEqual(h['config']['_scorer'], 'stroke_plane_yaw_moment_nm')
        self.assertTrue(h['config'].get('_h88a_dynamic_vector'))
        self.assertGreater(h['config'].get('h88a_yaw_moment_nm', 0), 4.0)

    def test_h88b_metadata(self):
        h = next(h for h in wl._ADV_NEW_HYPS if h['id'] == 'H-88B')
        self.assertEqual(h['target'], 'manoeuvre')
        self.assertEqual(h['config']['_scorer'], 'sweep_bias_yaw_moment_nm')
        self.assertTrue(h['config'].get('_h88b_sweep_bias'))
        self.assertGreater(h['config'].get('h88b_yaw_moment_nm', 0), 2.5)

    def test_alss_curriculum_b_enrollment(self):
        self.assertIn('h88a_dynamic_vector_b', wl._ALSS_CURRICULUM['B'])
        self.assertIn('h88b_sweep_bias_b', wl._ALSS_CURRICULUM['B'])

    def test_cmts_champions(self):
        self.assertEqual(wl._CMTS_CHAMPIONS['odonata_dynamic_vectoring']['hyp_id'], 'H-88A')
        self.assertEqual(wl._CMTS_CHAMPIONS['odonata_sweep_bias_trim']['hyp_id'], 'H-88B')

    def test_expanded_bounds(self):
        for k in ['h88a_stroke_plane_tilt_deg', 'h88a_sweep_harmonic_amp_deg', 'h88a_yaw_moment_nm',
                  'h88b_sweep_bias_deg', 'h88b_yaw_moment_nm']:
            self.assertIn(k, wl._EXPANDED_DISCOVERY_BOUNDS)
            lo, hi = wl._EXPANDED_DISCOVERY_BOUNDS[k]
            self.assertLess(lo, hi)


class TestH88KinematicsAndGovernor(unittest.TestCase):
    """Test flight governor kinematic functions for H-88A and H-88B."""

    def test_h88a_harmonic_vectoring_neutral(self):
        res = fg.compute_h88a_harmonic_vectoring(t_sec=0.1, f_hz=2.6, flap_amp_deg=24.0,
                                                 stroke_plane_tilt_deg=18.0, yaw_demand=0.0)
        self.assertEqual(res['eff_tilt_deg'], 0.0)
        self.assertFalse(res['decoupled_yaw_active'])
        self.assertEqual(res['sweep_port_deg'], 0.0)
        self.assertEqual(res['sweep_stbd_deg'], 0.0)

    def test_h88a_harmonic_vectoring_right_yaw(self):
        res = fg.compute_h88a_harmonic_vectoring(t_sec=0.1, f_hz=2.6, flap_amp_deg=24.0,
                                                 stroke_plane_tilt_deg=18.0, yaw_demand=1.0)
        self.assertEqual(res['eff_tilt_deg'], 18.0)
        self.assertTrue(res['decoupled_yaw_active'])
        # Port and starboard sweeps should be anti-phase for differential stroke-plane tilt
        self.assertAlmostEqual(res['sweep_port_deg'], -res['sweep_stbd_deg'])
        # Symmetric flapping maintains lift balance (zero roll couple)
        self.assertEqual(res['flap_port_deg'], res['flap_stbd_deg'])

    def test_h88b_sweep_bias_vectoring(self):
        res = fg.compute_h88b_sweep_bias_vectoring(yaw_demand=0.75, bias_max_deg=12.0)
        self.assertEqual(res['sweep_bias_port_deg'], 9.0)
        self.assertEqual(res['sweep_bias_stbd_deg'], -9.0)
        self.assertEqual(res['differential_sweep_deg'], 18.0)
        self.assertTrue(res['is_trim_active'])
        self.assertEqual(res['power_penalty_w'], 0.0)

    def test_h88b_sweep_bias_deadband(self):
        res = fg.compute_h88b_sweep_bias_vectoring(yaw_demand=0.01, bias_max_deg=12.0)
        self.assertFalse(res['is_trim_active'])


class TestSoftwareOnlyRecoilEngine(unittest.TestCase):
    """Test software-only turnaround recoil and aero-spring trajectory shaping."""

    def test_soft_recoil_trajectory_accel_reduction(self):
        f_hz = 2.8
        amp_deg = 24.0
        # Check turnaround point at quarter-cycle (t = 1/(4*f))
        t_turn = 1.0 / (4.0 * f_hz)
        res = fg.compute_soft_recoil_trajectory(t_sec=t_turn, f_hz=f_hz, amp_deg=amp_deg)
        self.assertIn('pos_deg', res)
        self.assertIn('vel_dps', res)
        self.assertIn('accel_dps2', res)
        self.assertGreater(res['peak_accel_reduction_pct'], 0.0)

    def test_aero_spring_pitch_advance_near_stroke_reversal(self):
        f_hz = 2.8
        T = 1.0 / f_hz
        t_top = 0.25 * T
        # 10 ms before top turnaround
        t_test = t_top - 0.010
        res = fg.compute_aero_spring_pitch_advance(t_sec=t_test, f_hz=f_hz,
                                                   nominal_pitch_deg=6.0, advance_lead_ms=18.0)
        self.assertTrue(res['pitch_advance_active'])
        self.assertEqual(res['pitch_offset_deg'], +8.0)
        self.assertEqual(res['effective_pitch_deg'], 14.0)

    def test_aero_spring_pitch_advance_mid_stroke(self):
        f_hz = 2.8
        # Mid-downstroke (far from turnaround)
        t_test = 0.0
        res = fg.compute_aero_spring_pitch_advance(t_sec=t_test, f_hz=f_hz,
                                                   nominal_pitch_deg=6.0, advance_lead_ms=18.0)
        self.assertFalse(res['pitch_advance_active'])
        self.assertEqual(res['effective_pitch_deg'], 6.0)


class TestBiomimeticSalvagedHypotheses(unittest.TestCase):
    """Test H-91 clap-fling, H-108 pitch flare, and H-101 anti-nodding."""

    def test_h91_clap_fling_trajectory(self):
        res = fg.compute_h91_clap_fling_trajectory(t_sec=0.1, f_hz=2.4)
        self.assertEqual(res['root_clearance_mm'], 94.0)
        self.assertTrue(res['collision_safe'])
        self.assertIn('dorsal_elevation_deg', res)

    def test_h108_pitch_flare_trigger(self):
        # Severe sink rate (-3.0 m/s <= -2.5 m/s threshold)
        res_emergency = fg.compute_h108_pitch_flare(w_sink_ms=-3.0)
        self.assertTrue(res_emergency['is_flare_active'])
        self.assertEqual(res_emergency['commanded_pitch_deg'], 18.0)
        self.assertEqual(res_emergency['transient_lift_boost_n'], 35.0)

        # Normal sink rate (-1.0 m/s > -2.5 m/s threshold)
        res_normal = fg.compute_h108_pitch_flare(w_sink_ms=-1.0)
        self.assertFalse(res_normal['is_flare_active'])
        self.assertEqual(res_normal['commanded_pitch_deg'], 6.0)

    def test_h101_anti_nodding_phases(self):
        res = fg.compute_h101_anti_nodding_phases(base_phase_deg=0.0)
        p = res['station_phases_deg']
        self.assertEqual(len(p), 4)
        # Station 1 (fore) and Station 4 (aft) must be 180 deg out of phase
        phase_diff = abs(p[3] - p[0])
        self.assertEqual(phase_diff, 180.0)
        self.assertGreater(res['pitch_nod_attenuation_pct'], 80.0)


if __name__ == '__main__':
    unittest.main()
