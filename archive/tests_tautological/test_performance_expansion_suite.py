#!/usr/bin/env python3
"""
tests/test_performance_expansion_suite.py
==========================================
Verifies performance-enhancing firmware expansions:
  1. H-49 Convective Katzmayr wake surfing with physical 0.185m station stagger.
  2. H-46 High-speed dash parasitic drag elimination (S1223 zero-lift -4.2° decambering)
     and 3.1 Hz / 22.5° kinematic frequency boost within governor limits.
  3. TurnCoordinationGovernor turn load-factor pitch compensation (Delta_alpha = k*(sec(phi)-1)).
  4. CascadeSlipstreamGovernor 0.185m physical stagger calibration.
"""

import unittest
import math
import sys
import os

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
FIRMWARE_DIR = os.path.join(REPO_ROOT, 'firmware')
if FIRMWARE_DIR not in sys.path:
    sys.path.insert(0, FIRMWARE_DIR)

import mco_advanced_physics as ap
import mco_flight_governor as fg


class TestPerformanceExpansionSuite(unittest.TestCase):

    def test_h49_convective_katzmayr_station_delays(self):
        """H-49 applies distinct convective spatial phase delays to stations 2, 3, and 4."""
        base_cmds = [{'pitch_deg': 6.0} for _ in range(8)]
        # At V = 10.0 m/s, master phase = 0.0
        cmds, active, boost = ap.h49_katzmayr_vortex_energy_ingestion(
            base_cmds, V_ms=10.0, master_phase_rad=0.0, lever_pct=55.0
        )
        self.assertTrue(active)
        self.assertGreater(boost, 3.0)  # Up to 3.4 N boost

        # Stn 1 (wings 0,1) unmodified by downstream wake ingestion
        self.assertEqual(cmds[0]['pitch_deg'], 6.0)
        self.assertEqual(cmds[1]['pitch_deg'], 6.0)

        # Stn 2 (wings 2,3), Stn 3 (wings 4,5), Stn 4 (wings 6,7) must have progressive convective delays
        pitch_stn2 = cmds[2]['pitch_deg']
        pitch_stn3 = cmds[4]['pitch_deg']
        pitch_stn4 = cmds[6]['pitch_deg']
        self.assertNotEqual(pitch_stn2, 6.0)
        self.assertNotEqual(pitch_stn2, pitch_stn3)
        self.assertNotEqual(pitch_stn3, pitch_stn4)

    def test_h46_dash_sprint_zero_lift_decambering(self):
        """H-46 feathers stowed stations 1 & 4 with airfoil-specific flutter safety gating."""
        base_cmds = [{'pitch_deg': 6.0, 'sweep_deg': 60.0, 'flap_amp_deg': 60.0} for _ in range(8)]

        # 1. SD7062 high-speed dash (certified to 28 m/s): full 3.1 Hz / 22.5 deg sprint, -0.5 deg reflex
        cmds_sd, active_sd, f_sd = ap.h46_micro_stroke_resonant_buzz(
            lever_pct=90.0, V_ms=22.0, commands=base_cmds, current_freq_hz=2.0, airfoil_type='SD7062'
        )
        self.assertTrue(active_sd)
        self.assertEqual(f_sd, 3.1)
        for idx in (0, 1, 6, 7):
            self.assertEqual(cmds_sd[idx]['pitch_deg'], -0.5)  # SD7062 alpha_L=0
            self.assertEqual(cmds_sd[idx]['flap_amp_deg'], 0.0)
            self.assertEqual(cmds_sd[idx]['sweep_deg'], 85.0)
        for idx in (2, 3, 4, 5):
            self.assertEqual(cmds_sd[idx]['flap_amp_deg'], 22.5)
            self.assertEqual(cmds_sd[idx]['pitch_deg'], 2.0)
            self.assertEqual(cmds_sd[idx]['sweep_deg'], 68.0)

        # Product f * amp <= 91.7 deg*Hz governor ceiling
        self.assertLessEqual(f_sd * 22.5, 91.7)

        # 2. S1223 high-lift STOL: restricted by flutter ceiling (Vne = 18 m/s), clamped to 2.5 Hz, -4.2 deg reflex
        cmds_s12, active_s12, f_s12 = ap.h46_micro_stroke_resonant_buzz(
            lever_pct=90.0, V_ms=22.0, commands=base_cmds, current_freq_hz=2.0, airfoil_type='S1223'
        )
        self.assertTrue(active_s12)
        self.assertEqual(f_s12, 2.5)  # Clamped to prevent flutter divergence
        for idx in (0, 1, 6, 7):
            self.assertEqual(cmds_s12[idx]['pitch_deg'], -4.2)  # S1223 alpha_L=0

    def test_turn_coordination_load_factor_pitch_compensation(self):
        """TurnCoordinationGovernor computes delta_pitch_cmd, freq boost, and station pitch taper."""
        gov = fg.TurnCoordinationGovernor()
        # Straight level flight (bank = 0): no pitch compensation
        t_level = gov.compute(bank_deg=0.0, pitch_deg=0.0, V_ms=12.0)
        self.assertEqual(t_level['delta_pitch_cmd'], 0.0)
        self.assertEqual(t_level['turn_freq_mult'], 1.0)

        # 30 deg bank: 1/cos(30) = 1.155 -> +0.62 deg pitch, 1.075x freq multiplier
        t_30 = gov.compute(bank_deg=30.0, pitch_deg=0.0, V_ms=12.0)
        self.assertGreater(t_30['delta_pitch_cmd'], 0.5)
        self.assertLess(t_30['delta_pitch_cmd'], 1.2)
        self.assertGreater(t_30['turn_freq_mult'], 1.05)

        # 45 deg bank: 1/cos(45) = 1.414 -> +1.66 deg pitch, 1.189x freq multiplier
        t_45 = gov.compute(bank_deg=45.0, pitch_deg=0.0, V_ms=12.0)
        self.assertGreater(t_45['delta_pitch_cmd'], 1.4)
        self.assertLess(t_45['delta_pitch_cmd'], 2.2)
        self.assertGreater(t_45['turn_freq_mult'], 1.15)

        # 60 deg bank: clamped to safe limit (<= 3.5 deg), 1.414x freq multiplier (n=2.0)
        t_60 = gov.compute(bank_deg=60.0, pitch_deg=0.0, V_ms=12.0)
        self.assertAlmostEqual(t_60['delta_pitch_cmd'], 3.5, delta=0.5)
        self.assertLessEqual(t_60['delta_pitch_cmd'], 3.5)
        self.assertAlmostEqual(t_60['turn_freq_mult'], 1.414, places=2)

        # Station-specific pitch taper: Stn 1 has lower pitch increment to preserve stall margin
        taper = t_60['pitch_taper_deg']
        self.assertEqual(len(taper), 4)
        self.assertLess(taper[0], taper[1])  # Stn 1 (clean air) < Stn 2
        self.assertGreater(taper[3], taper[2])  # Stn 4 (deep downwash) > Stn 3

    def test_blade_strike_smooth_lyapunov_barrier(self):
        """Dynamic blade clearance uses continuous Lyapunov sigmoid barrier."""
        # 1. Opposing crossing with 25 deg sweep divergence (hazard)
        res_hazard = fg.check_tandem_blade_clearance(
            pair_strokes_deg=[+35.0, -35.0, 0.0, 0.0],
            pair_sweeps_deg=[60.0, 85.0, 60.0, 60.0]
        )
        self.assertFalse(res_hazard['clearance_safe'])
        self.assertGreater(res_hazard['barrier_penalty'], 0.9)
        self.assertEqual(res_hazard['sweep_rate_scale'], 0.1)

        # 2. Opposing crossing with mild 8 deg sweep divergence (smooth deceleration, safe)
        res_safe = fg.check_tandem_blade_clearance(
            pair_strokes_deg=[+20.0, -20.0, 0.0, 0.0],
            pair_sweeps_deg=[60.0, 68.0, 60.0, 60.0]
        )
        self.assertTrue(res_safe['clearance_safe'])
        self.assertLess(res_safe['barrier_penalty'], 0.1)
        self.assertGreater(res_safe['sweep_rate_scale'], 0.9)

    def test_cascade_slipstream_physical_stagger_calibration(self):
        """CascadeSlipstreamGovernor defaults to 0.185m physical tandem station stagger."""
        gov = fg.CascadeSlipstreamGovernor()
        self.assertEqual(gov.delta_x_m, 0.185)

        # Convective delay at 10 m/s slipstream: 0.185 / 11.5 = 16.1 ms
        telem = gov.compute(V_ias_ms=10.0, freq_hz=2.0)
        self.assertAlmostEqual(telem['vortex_delay_s'], 0.185 / 11.5, places=3)


if __name__ == '__main__':
    unittest.main()
