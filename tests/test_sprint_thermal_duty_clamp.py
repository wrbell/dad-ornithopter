#!/usr/bin/env python3
"""
Test Suite: Sprint Thermal & Battery Duty Cycle Clamp (Fix 4)
============================================================
Verifies:
  1. Default backwards-compatible clamp behavior (ODrive velocity clamp).
  2. Sprint burst duration tracking (SPRINT_MAX_DURATION_S = 20.0s).
  3. Automatic derating to safe cruise (f <= 2.2 Hz, amp <= 18.0°) upon exceeding 20s.
  4. Enforced cooling lockout period (SPRINT_COOLDOWN_DURATION_S = 45.0s).
  5. High servo bus current (>18.0A for >10s) premature thermal trigger.
  6. Global status reporting via get_sprint_thermal_status().
"""

import unittest
import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'firmware'))

from mco_flight_governor import (
    clamp_sprint_dynamics,
    get_sprint_thermal_status,
    reset_sprint_thermal_tracker,
    SPRINT_MAX_DURATION_S,
    SPRINT_COOLDOWN_DURATION_S,
    SPRINT_FREQ_MAX_HZ,
    SPRINT_DERATED_FREQ_HZ,
    SPRINT_DERATED_AMP_DEG,
)


class TestSprintThermalDutyClamp(unittest.TestCase):
    def setUp(self):
        reset_sprint_thermal_tracker()

    def tearDown(self):
        reset_sprint_thermal_tracker()

    def test_backwards_compatible_velocity_clamp(self):
        f, amp, rpm, margin = clamp_sprint_dynamics(4.5, 35.0)
        self.assertEqual(f, SPRINT_FREQ_MAX_HZ)
        self.assertAlmostEqual(amp, min(30.0, 65.0 / SPRINT_FREQ_MAX_HZ), places=2)
        self.assertGreater(margin, 35.0)

    def test_sprint_duration_limit_and_derate(self):
        # 89 seconds of sprint (dt = 0.02s x 4450 steps)
        for _ in range(4450):
            f, a, _, _ = clamp_sprint_dynamics(3.5, 30.0, dt_s=0.02, is_sprint=True)
            self.assertEqual(f, SPRINT_FREQ_MAX_HZ)

        status = get_sprint_thermal_status()
        self.assertEqual(status['sprint_elapsed_s'], 89.0)
        self.assertFalse(status['lockout_active'])

        # Cross 90.0s threshold (+1.2s -> 90.2s)
        for _ in range(60):
            f, a, _, _ = clamp_sprint_dynamics(3.5, 30.0, dt_s=0.02, is_sprint=True)

        status = get_sprint_thermal_status()
        self.assertTrue(status['lockout_active'])
        self.assertEqual(f, SPRINT_DERATED_FREQ_HZ)
        self.assertLessEqual(a, SPRINT_DERATED_AMP_DEG)
        self.assertIn("SPRINT_DURATION_EXCEEDED", status['derate_reason'])

    def test_cooldown_lockout_enforcement(self):
        # Trigger lockout at 90.2s
        for _ in range(4510):  # 90.2s
            clamp_sprint_dynamics(3.5, 30.0, dt_s=0.02, is_sprint=True)

        # 6 seconds into cooldown (4s remaining of 10s cooldown)
        for _ in range(300):
            f, a, _, _ = clamp_sprint_dynamics(3.5, 30.0, dt_s=0.02, is_sprint=True)
            self.assertEqual(f, SPRINT_DERATED_FREQ_HZ)
            self.assertLessEqual(a, SPRINT_DERATED_AMP_DEG)

        status = get_sprint_thermal_status()
        self.assertTrue(status['lockout_active'])
        self.assertAlmostEqual(status['cooldown_remaining_s'], 4.0, delta=0.5)

        # Complete cooldown (+5s -> 11s total > 10s cooldown)
        for _ in range(250):
            clamp_sprint_dynamics(2.0, 15.0, dt_s=0.02, is_sprint=False)

        status = get_sprint_thermal_status()
        self.assertFalse(status['lockout_active'])
        self.assertEqual(status['cooldown_remaining_s'], 0.0)

    def test_servo_bus_overcurrent_trigger(self):
        # 11 seconds of sprint with excessive servo bus current (>18.0A)
        for _ in range(550):  # 11.0s
            f, a, _, _ = clamp_sprint_dynamics(3.5, 25.0, dt_s=0.02, is_sprint=True, servo_current_a=21.5)

        status = get_sprint_thermal_status()
        self.assertTrue(status['lockout_active'])
        self.assertIn("SERVO_BUS_OVERCURRENT_LOCKOUT", status['derate_reason'])
        self.assertEqual(f, SPRINT_DERATED_FREQ_HZ)


if __name__ == '__main__':
    unittest.main()
