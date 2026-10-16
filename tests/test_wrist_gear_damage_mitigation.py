#!/usr/bin/env python3
"""
tests/test_wrist_gear_damage_mitigation.py
==========================================
Unit test suite verifying the Progressive Wrist Gear Damage Monitor & Mitigation
architecture (H-GEAR-COPE-1) implemented in mco_flight_governor.py and integrated
into mco_main.py.

Verifies:
  1. Nominal gear tracking (error < 2.0°): No derating, all tiers at 0.
  2. Tier 1 Incipient Backlash Expansion (error > 2.5°):
     - Softens stroke amplitude by 10% on damaged wing to cushion tooth reversal shock.
     - Sets gear_tier1_soften flag and emits telemetry advisory.
  3. Tier 2 Active Tooth Slipping (error > 5.0° for >= 6 cycles):
     - Freezes pitch actuation on damaged wing, locking to +4.0° neutral cruise trim.
     - Offloads lift to healthy tandem partner wing (+12% flap amplitude boost).
     - Sets gear_tier2_locked flag and emits RTB advisory.
  4. Tier 3 Catastrophic Tooth Stripping (error > 15.0° for >= 5 cycles):
     - Immediately zeros flapping amplitude (flap_amp_deg = 0.0) on damaged wing.
     - Sweeps damaged wing aft (95.0°) to minimize drag.
     - Identifies primary_failed_wing and automatically engages H-42 N-1 Fail-Operational Retrim.
  5. Reset functionality across battery swaps or after replacing sacrificial gears.
"""

import unittest
import copy
import sys
import os

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'firmware')))

from mco_flight_governor import (
    WristGearHealthMonitor,
    govern_wrist_gear_damage,
    reset_wrist_gear_monitor
)
import mco_advanced_physics as ap


class TestWristGearDamageMitigation(unittest.TestCase):

    def setUp(self):
        reset_wrist_gear_monitor()
        self.base_commands = [
            {'flap_amp_deg': 24.0, 'pitch_deg': 6.0, 'sweep_deg': 90.0}
            for _ in range(8)
        ]

    def test_nominal_tracking(self):
        """When actual pitch angles match commanded pitch within 1.0°, all tiers remain 0."""
        actual_pitches = [6.2, 5.8, 6.1, 5.9, 6.0, 6.2, 5.8, 6.1]
        cmds = copy.deepcopy(self.base_commands)
        res = {}

        telem = govern_wrist_gear_damage(
            wing_commands=cmds,
            actual_pitch_degs=actual_pitches,
            dt=0.02,
            result=res
        )

        self.assertEqual(telem['tier1_count'], 0)
        self.assertEqual(telem['tier2_count'], 0)
        self.assertEqual(telem['tier3_count'], 0)
        self.assertEqual(telem['status'], 'NOMINAL_GEAR_HEALTH')
        self.assertIsNone(telem['primary_failed_wing'])
        # Verify commands are untouched
        for i in range(8):
            self.assertEqual(cmds[i]['flap_amp_deg'], 24.0)
            self.assertEqual(cmds[i]['pitch_deg'], 6.0)

    def test_tier1_incipient_backlash_softening(self):
        """When pitch tracking error exceeds 2.5°, Tier 1 softens amplitude by 10%."""
        # Wing 0 has 3.5° backlash error (cmd=6.0°, actual=2.5°)
        actual_pitches = [2.5, 6.0, 6.0, 6.0, 6.0, 6.0, 6.0, 6.0]
        cmds = copy.deepcopy(self.base_commands)
        res = {}

        telem = govern_wrist_gear_damage(
            wing_commands=cmds,
            actual_pitch_degs=actual_pitches,
            dt=0.02,
            result=res
        )

        self.assertEqual(telem['health_tiers'][0], 1)
        self.assertEqual(telem['tier1_count'], 1)
        self.assertIn(0, telem['tier1_wings'])
        self.assertTrue(cmds[0].get('gear_tier1_soften'))
        # 24.0 * 0.90 = 21.6°
        self.assertAlmostEqual(cmds[0]['flap_amp_deg'], 21.6, places=1)
        # Other wings remain at 24.0°
        for i in range(1, 8):
            self.assertEqual(cmds[i]['flap_amp_deg'], 24.0)

    def test_tier2_active_slip_locking_and_partner_boost(self):
        """When pitch error exceeds 5.0° for 6 cycles, Tier 2 locks pitch to +4° and boosts tandem partner."""
        # Wing 2 slips persistently (cmd=10.0°, actual=2.0° -> error 8.0°)
        for c in self.base_commands:
            c['pitch_deg'] = 10.0

        for cycle in range(6):
            cmds = copy.deepcopy(self.base_commands)
            actual_pitches = [10.0, 10.0, 2.0, 10.0, 10.0, 10.0, 10.0, 10.0]
            res = {}
            telem = govern_wrist_gear_damage(
                wing_commands=cmds,
                actual_pitch_degs=actual_pitches,
                dt=0.02,
                result=res
            )

        self.assertEqual(telem['health_tiers'][2], 2)
        self.assertEqual(telem['tier2_count'], 1)
        self.assertIn(2, telem['tier2_wings'])
        # Wing 2 pitch must be locked to neutral cruise trim (+4.0°)
        self.assertEqual(cmds[2]['pitch_deg'], 4.0)
        self.assertTrue(cmds[2].get('gear_tier2_locked'))
        # Partner wing (Wing 4: (2 + 2) % 8 = 4) must receive +12% lift boost (24.0 * 1.12 = 26.88°)
        self.assertAlmostEqual(cmds[4]['flap_amp_deg'], 26.88, places=1)
        self.assertTrue(cmds[4].get('partner_boost_active'))

    def test_tier3_catastrophic_stripping_triggers_h42(self):
        """When pitch error exceeds 15.0° for 5 cycles, Tier 3 zeros flapping, stows wing, and triggers H-42."""
        # Wing 1 gear shears off completely (cmd=12.0°, actual=-8.0° -> error 20.0°)
        for c in self.base_commands:
            c['pitch_deg'] = 12.0

        for cycle in range(5):
            cmds = copy.deepcopy(self.base_commands)
            actual_pitches = [12.0, -8.0, 12.0, 12.0, 12.0, 12.0, 12.0, 12.0]
            res = {}
            telem = govern_wrist_gear_damage(
                wing_commands=cmds,
                actual_pitch_degs=actual_pitches,
                dt=0.02,
                result=res
            )

        self.assertEqual(telem['health_tiers'][1], 3)
        self.assertEqual(telem['tier3_count'], 1)
        self.assertIn(1, telem['tier3_wings'])
        self.assertEqual(telem['primary_failed_wing'], 1)
        # Damaged Wing 1 must be zeroed in flapping and stowed aft
        self.assertEqual(cmds[1]['flap_amp_deg'], 0.0)
        self.assertEqual(cmds[1]['sweep_deg'], 95.0)
        self.assertTrue(cmds[1].get('gear_tier3_stripped'))
        self.assertEqual(res.get('failed_wing_idx'), 1)

        # Verify H-42 seamlessly takes over with the declared failed_wing_idx
        retrim_cmds, h42_active, diag = ap.h42_asymmetric_retrim_allocation(1, cmds)
        self.assertTrue(h42_active)
        self.assertEqual(diag['failed_wing'], 1)
        self.assertGreater(diag['lift_retention_pct'], 95.0)

    def test_monitor_reset_after_gear_swap(self):
        """Verifies that reset clears all health tiers and error history."""
        # Trip Tier 1 on Wing 3
        actual_pitches = [6.0, 6.0, 6.0, 2.0, 6.0, 6.0, 6.0, 6.0]
        cmds = copy.deepcopy(self.base_commands)
        govern_wrist_gear_damage(cmds, actual_pitches)

        # Reset after physical gear replacement
        reset_wrist_gear_monitor()

        # Next cycle with good tracking should be completely nominal
        good_pitches = [6.0] * 8
        telem = govern_wrist_gear_damage(cmds, good_pitches)
        self.assertEqual(telem['tier1_count'], 0)
        self.assertEqual(telem['health_tiers'][3], 0)
        self.assertEqual(telem['status'], 'NOMINAL_GEAR_HEALTH')


if __name__ == '__main__':
    unittest.main()
