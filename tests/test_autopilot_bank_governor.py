#!/usr/bin/env python3
"""
tests/test_autopilot_bank_governor.py
======================================
Verifies the Autopilot Bank & Knife-Edge Governor:
1. Under manual pilot flight, pilot authority is 100% unimpeded (no bank clamping).
2. Under autopilot autonomous transit/altitude hold, sustained bank is clamped to <= 65°.
3. Under autopilot autonomous transit, transient knife-edge passes (>75°) expire after 3.0s.
4. Pilot override on sticks instantly clears autopilot limits.
"""

import unittest
import sys
import os

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
FIRMWARE_DIR = os.path.join(REPO_ROOT, 'firmware')
if FIRMWARE_DIR not in sys.path:
    sys.path.insert(0, FIRMWARE_DIR)

from mco_flight_governor import AutopilotBankGovernor, BANK_SUSTAINED_MAX_DEG, KNIFE_EDGE_BUDGET_S
import mco_wave_library as wl


class TestAutopilotBankGovernor(unittest.TestCase):
    def setUp(self):
        self.gov = AutopilotBankGovernor()

    def test_pilot_authority_unimpeded(self):
        """Human pilot can pull 90° knife edge or steep bank without FCC clamping."""
        res = self.gov.update(dt_s=0.02, bank_deg=90.0, hold_altitude=True, is_pilot_flying=True)
        self.assertEqual(res['commanded_bank_deg'], 90.0)
        self.assertFalse(res['recovery_active'])
        self.assertIn('PILOT_AUTHORITY_UNIMPEDED', res['status'])

        # Sustained for 5 seconds — pilot is never overridden
        for _ in range(250):
            res = self.gov.update(dt_s=0.02, bank_deg=-85.0, hold_altitude=True, is_pilot_flying=True)
        self.assertEqual(res['commanded_bank_deg'], -85.0)
        self.assertFalse(res['recovery_active'])

    def test_autopilot_sustained_bank_clamped_to_65(self):
        """Autopilot in altitude hold clamps sustained bank to <= 65° to preserve vertical lift."""
        res = self.gov.update(dt_s=0.02, bank_deg=70.0, hold_altitude=True, is_pilot_flying=False)
        self.assertEqual(res['commanded_bank_deg'], BANK_SUSTAINED_MAX_DEG)
        self.assertFalse(res['recovery_active'])

        res_neg = self.gov.update(dt_s=0.02, bank_deg=-72.0, hold_altitude=True, is_pilot_flying=False)
        self.assertEqual(res_neg['commanded_bank_deg'], -BANK_SUSTAINED_MAX_DEG)

    def test_autopilot_knife_edge_transient_budget(self):
        """Autopilot transient knife-edge (>75°) is allowed for <= 3.0s before rolling back."""
        # First 2.0s in knife-edge (within 3.0s budget)
        for _ in range(100):
            res = self.gov.update(dt_s=0.02, bank_deg=85.0, hold_altitude=True, is_pilot_flying=False)
        self.assertTrue(res['knife_edge_active'])
        self.assertFalse(res['recovery_active'])
        self.assertEqual(res['commanded_bank_deg'], 85.0)

        # Exceed 3.0s budget (another 60 ticks = 1.2s -> total 3.2s)
        for _ in range(60):
            res = self.gov.update(dt_s=0.02, bank_deg=85.0, hold_altitude=True, is_pilot_flying=False)
        self.assertTrue(res['recovery_active'])
        self.assertEqual(res['commanded_bank_deg'], BANK_SUSTAINED_MAX_DEG)
        self.assertIn('WARN_AUTOPILOT_KNIFE_EDGE_BUDGET_EXPIRED', res['status'])

    def test_autopilot_transit_integration(self):
        """Verifies integration with wl.transit_tick and pilot stick override."""
        wl.transit_to(40.7128, -74.0060, alt_m=50.0)

        sensors_ap = {
            'lat': 40.7000, 'lon': -74.0000, 'heading_deg': 0.0,
            'V_ms': 12.0, 'h_agl_m': 50.0, 'roll_deg_imu': 30.0,
            'roll_stick': 0.0, 'pitch_stick': 0.0, 'yaw_stick': 0.0,
            'dt_s': 0.02,
        }
        res = wl.transit_tick(sensors_ap, {})
        self.assertTrue(res.get('transit_active'))
        self.assertFalse(res.get('transit_pilot_override'))
        self.assertIn('ap_bank_gov_status', res)

        # Pilot moves stick -> autopilot yields immediately
        sensors_pilot = dict(sensors_ap)
        sensors_pilot['roll_stick'] = 0.50
        res_pilot = wl.transit_tick(sensors_pilot, {})
        self.assertTrue(res_pilot.get('transit_pilot_override'))

        wl.transit_abort()


if __name__ == '__main__':
    unittest.main()
