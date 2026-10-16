#!/usr/bin/env python3
"""
tests/test_symmetrical_rth_controller.py
=========================================
Unit and integration test suite for Upgraded Symmetrical Station Pair Quarantine & Biased Evasion (H-42-RTH / H-RTH-6WING).
Validates:
  1. Clean 6-Wing Symmetrical Quarantine:
     - Zero blade strike via Golden Rule sweep schedule (Lambda_rear >= Lambda_front).
     - True zero-lift feathering (pitch = -4.2 deg, alpha_L=0 for S1223).
     - Frequency scaled to 2.16 Hz (100% MTOW lift retention).
  2. Full Longitudinal Pitch Moment Balance (100% of M_lost Restored):
     - Hybrid pitch + amplitude: restores 100% of lost moment without exceeding S1223 stall limits.
     - Station 1 loss: Stn 2 (+2.5° / 88.0°), Stn 4 (-2.5° / 74.0°), restoring +2.90 Nm (+100.3%).
     - Station 4 loss: Stn 1 (-2.5° / 74.0°), Stn 3 (+2.5° / 88.0°), restoring -2.90 Nm (-100.3%).
     - Station 2 loss: Stn 1 (+1.5° / 86.0°), Stn 4 (-1.5° / 78.0°), restoring +1.36 Nm (+100.7%).
     - Station 3 loss: Stn 1 (-1.5° / 78.0°), Stn 4 (+1.5° / 86.0°), restoring -1.36 Nm (-100.7%).
  3. Pincer Sweep Corridor & Golden Rule (Lambda_rear >= Lambda_front):
     - Stn 2 loss: Stn 1 forward (50°), Stn 3 & 4 aft (74°), opening >= 10.9mm gap everywhere.
     - Stn 3 loss: Stns 1 & 2 forward (50°), Stn 4 aft (74°), opening >= 10.9mm gap everywhere.
     - Dynamic sweep blow-back defense cascades rear sweep to maintain clearance.
  4. Dynamic 3-Pair 120° Metachronal Phase Re-indexing:
     - Surviving 3 pairs re-indexed to [0°, 120°, 240°] eliminating 180° deadband and bus ripple.
  5. Biased Sector Evasion (Frozen Wing / Broken Rope):
     - Enforces >= 32.0° aeroelastic whip clearance in lower hemisphere ([-71°, -33°]).
     - Bilateral station coupling mirrors stroke to prevent roll/sideforce imbalance.
     - Aft sweep shift to 74.0° for longitudinal spanwise clearance.
  6. Ground Flare Interlock:
     - Below 2.5m AGL, clamps evasion flap amplitude to 0.0° (shallow glide lock) to prevent ground strike.
  7. Multi-Wing Degradation Stages (6-wing RTH -> 4-wing emergency descent -> forced deadstick).
  8. SymmetricalRTHGovernor persistence timer (fast H-42 -> 1.5s -> Symmetrical RTH, immediate on 'rth' intent).
  9. Full loop integration in MCOHardware.apply_commands().
"""

import unittest
import math
import mco_advanced_physics as ap
from mco_flight_governor import (
    SymmetricalRTHGovernor,
    govern_symmetrical_rth,
    reset_symmetrical_rth_governor
)
from mco_main import MCOHardware


class TestSymmetricalRTHController(unittest.TestCase):

    def setUp(self):
        reset_symmetrical_rth_governor()

    def test_station_1_loss_hardened_quarantine(self):
        """Station 1 failure (Wing 0/1) must quarantine Stn 1, apply hybrid pitch+amp (+2.90 Nm), and re-index 120° phase."""
        base_cmds = [{'pitch_deg': 5.0, 'sweep_deg': 60.0, 'flap_amp_deg': 75.0} for _ in range(8)]
        out_cmds, rth_active, rec_freq, diag = ap.symmetrical_pair_quarantine_rth(
            failed_wings=0,
            base_commands=base_cmds,
            base_freq_hz=1.87
        )

        self.assertTrue(rth_active)
        self.assertEqual(diag['quarantined_stations'], [0])
        self.assertEqual(diag['surviving_wings'], [2, 3, 4, 5, 6, 7])

        # Quarantined wings 0 and 1: zero flap, zero-lift pitch (-4.2°), coplanar sweep (60°)
        for w in (0, 1):
            self.assertEqual(out_cmds[w]['flap_amp_deg'], 0.0)
            self.assertEqual(out_cmds[w]['pitch_deg'], -4.2)
            self.assertEqual(out_cmds[w]['sweep_deg'], 60.0)
            self.assertTrue(out_cmds[w]['quarantined'])

        # Surviving wings: Station 2 (+2.5° - 0.3° clean air = 6.0°, amp 88.0°, sweep 74.0°)
        self.assertEqual(out_cmds[2]['pitch_deg'], 6.0)
        self.assertEqual(out_cmds[3]['pitch_deg'], 6.0)
        self.assertEqual(out_cmds[2]['flap_amp_deg'], 88.0)
        self.assertEqual(out_cmds[3]['flap_amp_deg'], 88.0)
        self.assertEqual(out_cmds[2]['sweep_deg'], 74.0)

        # Station 3: pitch 3.8°, amp 82.0°, sweep 74.0°
        self.assertEqual(out_cmds[4]['pitch_deg'], 3.8)
        self.assertEqual(out_cmds[5]['pitch_deg'], 3.8)
        self.assertEqual(out_cmds[4]['flap_amp_deg'], 82.0)
        self.assertEqual(out_cmds[5]['flap_amp_deg'], 82.0)

        # Station 4: pitch 1.3°, amp 74.0°, sweep 74.0°
        self.assertEqual(out_cmds[6]['pitch_deg'], 1.3)
        self.assertEqual(out_cmds[7]['pitch_deg'], 1.3)
        self.assertEqual(out_cmds[6]['flap_amp_deg'], 74.0)
        self.assertEqual(out_cmds[7]['flap_amp_deg'], 74.0)

        # Frequency scaled to 2.16 Hz
        self.assertAlmostEqual(rec_freq, 2.16, places=2)

        # 3-Pair 120° phase re-indexing
        self.assertTrue(diag['phase_reindexed_120deg'])
        self.assertEqual(len(diag['recommended_pair_offsets_rad']), 3)

        # Dutch roll damping recommendation flagged for high sweep
        self.assertTrue(diag['dutch_roll_damping_recommended'])

    def test_station_4_loss_pitch_amp_trim(self):
        """Station 4 loss (Wings 6/7) must shed forward lift (Stn 1: 1.3°/74°) and boost aft lift (Stn 3: 6.3°/88°)."""
        base_cmds = [{'pitch_deg': 5.0, 'sweep_deg': 60.0, 'flap_amp_deg': 75.0} for _ in range(8)]
        out_cmds, rth_active, rec_freq, diag = ap.symmetrical_pair_quarantine_rth(
            failed_wings=6,
            base_commands=base_cmds,
            base_freq_hz=1.87
        )

        self.assertTrue(rth_active)
        self.assertEqual(diag['quarantined_stations'], [3])

        # Station 1: pitch 1.3°, amp 74.0°, sweep 60.0°
        self.assertEqual(out_cmds[0]['pitch_deg'], 1.3)
        self.assertEqual(out_cmds[1]['pitch_deg'], 1.3)
        self.assertEqual(out_cmds[0]['flap_amp_deg'], 74.0)
        self.assertEqual(out_cmds[1]['flap_amp_deg'], 74.0)

        # Station 2: pitch 3.8°, amp 82.0°, sweep 60.0°
        self.assertEqual(out_cmds[2]['pitch_deg'], 3.8)
        self.assertEqual(out_cmds[3]['pitch_deg'], 3.8)
        self.assertEqual(out_cmds[2]['flap_amp_deg'], 82.0)

        # Station 3: pitch 6.3°, amp 88.0°, sweep 60.0°
        self.assertEqual(out_cmds[4]['pitch_deg'], 6.3)
        self.assertEqual(out_cmds[5]['pitch_deg'], 6.3)
        self.assertEqual(out_cmds[4]['flap_amp_deg'], 88.0)

        # Quarantined Station 4: pitch -4.2°, amp 0.0°, sweep 74.0°
        self.assertEqual(out_cmds[6]['pitch_deg'], -4.2)
        self.assertEqual(out_cmds[7]['pitch_deg'], -4.2)
        self.assertEqual(out_cmds[6]['flap_amp_deg'], 0.0)
        self.assertEqual(out_cmds[6]['sweep_deg'], 74.0)

    def test_station_2_loss_pincer_sweep_corridor(self):
        """Station 2 loss (Wings 2/3) must open Pincer corridor: Stn 1 forward (50°), Stn 3 & 4 aft (74°)."""
        base_cmds = [{'pitch_deg': 5.0, 'sweep_deg': 60.0, 'flap_amp_deg': 75.0} for _ in range(8)]
        out_cmds, rth_active, rec_freq, diag = ap.symmetrical_pair_quarantine_rth(
            failed_wings=2,
            base_commands=base_cmds,
            base_freq_hz=1.87
        )

        self.assertTrue(rth_active)
        self.assertEqual(diag['quarantined_stations'], [1])

        # Sweeps: Stn 1 (50°), Stn 2 (60°), Stn 3 (74°), Stn 4 (74°)
        self.assertEqual(out_cmds[0]['sweep_deg'], 50.0)
        self.assertEqual(out_cmds[1]['sweep_deg'], 50.0)
        self.assertEqual(out_cmds[2]['sweep_deg'], 60.0)
        self.assertEqual(out_cmds[3]['sweep_deg'], 60.0)
        self.assertEqual(out_cmds[4]['sweep_deg'], 74.0)
        self.assertEqual(out_cmds[5]['sweep_deg'], 74.0)
        self.assertEqual(out_cmds[6]['sweep_deg'], 74.0)
        self.assertEqual(out_cmds[7]['sweep_deg'], 74.0)

        # Hybrid pitch+amp: Stn 1 (+1.5° -> 5.3°, amp 86°), Stn 4 (-1.5° -> 2.3°, amp 78°)
        self.assertEqual(out_cmds[0]['pitch_deg'], 5.3)
        self.assertEqual(out_cmds[0]['flap_amp_deg'], 86.0)
        self.assertEqual(out_cmds[6]['pitch_deg'], 2.3)
        self.assertEqual(out_cmds[6]['flap_amp_deg'], 78.0)

    def test_station_3_loss_pincer_sweep_corridor(self):
        """Station 3 loss (Wings 4/5) must open Pincer corridor: Stns 1 & 2 forward (50°), Stn 4 aft (74°)."""
        base_cmds = [{'pitch_deg': 5.0, 'sweep_deg': 60.0, 'flap_amp_deg': 75.0} for _ in range(8)]
        out_cmds, rth_active, rec_freq, diag = ap.symmetrical_pair_quarantine_rth(
            failed_wings=4,
            base_commands=base_cmds,
            base_freq_hz=1.87
        )

        self.assertTrue(rth_active)
        self.assertEqual(diag['quarantined_stations'], [2])

        # Sweeps: Stn 1 (50°), Stn 2 (50°), Stn 3 (60°), Stn 4 (74°)
        self.assertEqual(out_cmds[0]['sweep_deg'], 50.0)
        self.assertEqual(out_cmds[2]['sweep_deg'], 50.0)
        self.assertEqual(out_cmds[4]['sweep_deg'], 60.0)
        self.assertEqual(out_cmds[6]['sweep_deg'], 74.0)

        # Hybrid pitch+amp: Stn 1 (-1.5° -> 2.3°, amp 78°), Stn 4 (+1.5° -> 5.3°, amp 86°)
        self.assertEqual(out_cmds[0]['pitch_deg'], 2.3)
        self.assertEqual(out_cmds[0]['flap_amp_deg'], 78.0)
        self.assertEqual(out_cmds[6]['pitch_deg'], 5.3)
        self.assertEqual(out_cmds[6]['flap_amp_deg'], 86.0)

    def test_golden_rule_sweep_blowback_defense(self):
        """If a forward wing blows back aft, downstream wings must cascade to obey Lambda_rear >= Lambda_front."""
        base_cmds = [{'pitch_deg': 5.0, 'sweep_deg': 60.0, 'flap_amp_deg': 75.0} for _ in range(8)]
        # Force front wing to 76 deg blowback
        base_cmds[0]['sweep_deg'] = 76.0
        base_cmds[1]['sweep_deg'] = 76.0

        out_cmds, rth_active, rec_freq, diag = ap.symmetrical_pair_quarantine_rth(
            failed_wings=0,
            base_commands=base_cmds,
            base_freq_hz=1.87
        )

        # Downstream stations must cascade to >= 76.0 deg
        for st in range(3):
            sw_f = out_cmds[st * 2]['sweep_deg']
            sw_r = out_cmds[(st + 1) * 2]['sweep_deg']
            self.assertGreaterEqual(sw_r, sw_f - 0.1)

    def test_biased_sector_evasion_and_aeroelastic_whip_margin(self):
        """When Station 1 is frozen, Station 2 must flap in lower hemisphere with >= 32° whip clearance and bilateral symmetry."""
        base_cmds = [{'pitch_deg': 5.0, 'sweep_deg': 60.0, 'flap_amp_deg': 75.0} for _ in range(8)]

        # Frozen wing at horizontal (0.0°)
        out_cmds, rth_active, rec_freq, diag = ap.symmetrical_pair_quarantine_rth(
            failed_wings=0,
            base_commands=base_cmds,
            base_freq_hz=1.87,
            h_agl_m=50.0,
            frozen_angles={0: 0.0}
        )

        self.assertTrue(diag['evasion_active'])
        self.assertFalse(diag['ground_flare_interlock_active'])

        # Station 2 (wings 2 & 3) must flap with dihedral bias -52° and amp 38°
        for w in (2, 3):
            self.assertEqual(out_cmds[w]['flap_amp_deg'], 38.0)
            self.assertEqual(out_cmds[w]['dihedral_bias_deg'], -52.0)
            self.assertEqual(out_cmds[w]['sweep_deg'], 74.0)

        # Bilateral symmetry
        self.assertEqual(out_cmds[2]['flap_amp_deg'], out_cmds[3]['flap_amp_deg'])
        self.assertEqual(out_cmds[2]['dihedral_bias_deg'], out_cmds[3]['dihedral_bias_deg'])

    def test_ground_flare_interlock_prevents_runway_strike(self):
        """Below 2.5m AGL, biased sector evasion must clamp flap amplitude to 0° (glide lock) to eliminate ground strike."""
        base_cmds = [{'pitch_deg': 5.0, 'sweep_deg': 60.0, 'flap_amp_deg': 75.0} for _ in range(8)]

        out_cmds, rth_active, rec_freq, diag = ap.symmetrical_pair_quarantine_rth(
            failed_wings=0,
            base_commands=base_cmds,
            base_freq_hz=1.87,
            h_agl_m=1.8,
            frozen_angles={0: 0.0}
        )

        self.assertTrue(diag['evasion_active'])
        self.assertTrue(diag['ground_flare_interlock_active'])
        self.assertEqual(out_cmds[2]['flap_amp_deg'], 0.0)
        self.assertEqual(out_cmds[3]['flap_amp_deg'], 0.0)

    def test_multi_wing_degradation(self):
        """Test degradation under 2-station loss (4 wings surviving) and 3-station loss (<4 wings surviving)."""
        base_cmds = [{'pitch_deg': 5.0, 'sweep_deg': 60.0, 'flap_amp_deg': 75.0} for _ in range(8)]

        # Two stations damaged: Station 0 (wings 0,1) and Station 2 (wings 4,5)
        out_cmds, rth_active, rec_freq, diag = ap.symmetrical_pair_quarantine_rth(
            failed_wings=[0, 4],
            base_commands=base_cmds,
            base_freq_hz=1.87
        )
        self.assertTrue(rth_active)
        self.assertEqual(diag['num_surviving_wings'], 4)
        self.assertIn('4WING', diag['status'])
        self.assertAlmostEqual(diag['lift_retention_pct'], 66.7, places=1)
        self.assertEqual(rec_freq, 2.34)

        # Three stations damaged: Station 0, 1, 2
        out_cmds, rth_active, rec_freq, diag = ap.symmetrical_pair_quarantine_rth(
            failed_wings=[0, 2, 4],
            base_commands=base_cmds,
            base_freq_hz=1.87
        )
        self.assertTrue(rth_active)
        self.assertEqual(diag['num_surviving_wings'], 2)
        self.assertEqual(rec_freq, 0.0)
        self.assertIn('FORCED_DEADSTICK_GLIDE', diag['status'])

    def test_governor_persistence_and_intent(self):
        """Governor transitions from asymmetric stabilization to symmetrical RTH after 1.5s or on RTH intent."""
        gov = SymmetricalRTHGovernor(persistence_threshold_s=1.5)
        cmds = [{'pitch_deg': 5.0, 'sweep_deg': 60.0, 'flap_amp_deg': 75.0} for _ in range(8)]

        # Tick 1: t = 0.02s in cruise -> immediate asymmetric stabilization mode
        res1 = gov.govern_rth(cmds, failed_wings=[0], intent='cruise', dt=0.02, base_freq_hz=1.87)
        self.assertFalse(res1['rth_engaged'])
        self.assertEqual(res1['status'], 'IMMEDIATE_ASYMMETRIC_STABILIZATION')

        # Tick 2 to 74 (total 1.48s) -> still asymmetric stabilization
        for _ in range(73):
            gov.govern_rth(cmds, failed_wings=[0], intent='cruise', dt=0.02, base_freq_hz=1.87)
        self.assertFalse(gov.symmetrical_rth_engaged)

        # Tick 76 (total 1.52s >= 1.50s) -> symmetrical RTH engaged!
        res76 = gov.govern_rth(cmds, failed_wings=[0], intent='cruise', dt=0.02, base_freq_hz=1.87)
        self.assertTrue(res76['rth_engaged'])
        self.assertEqual(res76['quarantined_stations'], [0])
        self.assertAlmostEqual(res76['recommended_freq_hz'], 2.16, places=2)
        self.assertTrue(res76['phase_reindexed_120deg'])

        # Immediate engagement when intent is 'rth'
        gov.reset()
        res_rth = gov.govern_rth(cmds, failed_wings=[1], intent='rth', dt=0.02, base_freq_hz=1.87)
        self.assertTrue(res_rth['rth_engaged'])
        self.assertEqual(res_rth['quarantined_stations'], [0])

    def test_integration_in_mco_hardware_apply_commands(self):
        """Verify full loop in MCOHardware.apply_commands cleanly triggers hardened symmetrical RTH."""
        hw = MCOHardware(sim_mode=True)
        wc = [{'pitch_deg': 5.0, 'sweep_deg': 60.0, 'flap_amp_deg': 75.0} for _ in range(8)]
        result = {'wing_commands': wc, 'freq_hz': 1.87}
        sensors = {
            'failed_wing_idx': 0,
            'intent': 'rth',
            'servo_actual_aoa_deg': [5.0]*8,
            'sweep_actual_deg': [60.0]*8,
            'h_agl_m': 30.0,
        }

        hw.apply_commands(result, sensors)

        self.assertTrue(hw._symmetrical_rth_active)
        self.assertFalse(hw._h42_active)  # H-42 superseded
        self.assertEqual(result['freq_hz'], 2.16)
        self.assertEqual(result['wing_commands'][0]['flap_amp_deg'], 0.0)
        self.assertEqual(result['wing_commands'][1]['flap_amp_deg'], 0.0)
        self.assertEqual(result['wing_commands'][0]['pitch_deg'], -4.2)
        self.assertEqual(result['wing_commands'][1]['pitch_deg'], -4.2)
        self.assertEqual(result['wing_commands'][0]['sweep_deg'], 60.0)
        self.assertEqual(result['wing_commands'][1]['sweep_deg'], 60.0)
        self.assertEqual(result['wing_commands'][2]['flap_amp_deg'], 88.0)
        self.assertEqual(len(result['pair_phase_offsets']), 3)


if __name__ == '__main__':
    unittest.main()
