"""
Unit Tests: Active Ride Quality, Heave Damping & Metachronal Pitch Moment Cancellation (H-41)
=============================================================================================
Verifies:
  1. Metachronal pitch reaction moment cancellation across the 4 longitudinal stations
     (x = [+0.207, +0.069, -0.069, -0.207] m).
  2. Active heave damping response to vertical acceleration spikes (a_z) and climb rates (vsi).
  3. Passenger comfort criteria (ISO 2631 < 0.03g RMS cabin vibration).
  4. Sensitive/fragile cargo G-limit protection envelope (<= 1.15g peak).
  5. Intent arbitration into RIDE_COMFORT and CARGO_STABILIZE states.
  6. Behaviour nodes: RideComfortBehaviour and CargoStabilizeBehaviour execution.
  7. Pilot stick priority preemption (FAR Part 23 manual stick override).
  8. MAVLink bridge ride comfort status reporting and mode toggling.
"""

import math
import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'firmware'))

import mco_advanced_physics as ap
import mco_intent_control as ic
import mco_mavlink_bridge as mb


class TestMetachronalPitchMomentCancellation(unittest.TestCase):
    """Test metachronal pitch reaction moment cancellation across 4 wing pairs."""

    def setUp(self):
        self.stabilizer = ap.ActiveRideStabilizer()
        self.base_commands = [
            {'pitch_deg': 12.0, 'flap_amp_deg': 96.0, 'sweep_deg': 90.0} for _ in range(8)
        ]

    def test_anti_phase_flapping_moment_balance(self):
        """Verify that [0, pi, pi, 0] anti-phase offsets eliminate flapping pitch nod couple."""
        offsets, active, vib_red = ap.h41_heave_pitch_cancellation(comfort_mode=True)
        self.assertTrue(active)
        self.assertEqual(offsets, [0.0, math.pi, math.pi, 0.0])
        self.assertGreater(vib_red, 80.0)

        # Flapping lift modulation at quarter stroke (t = T/4):
        # Pairs 1 & 4 at phase pi/2 (peak downstroke lift: 13.95 + 4.88 N)
        # Pairs 2 & 3 at phase 3pi/2 (trough upstroke lift: 13.95 - 4.88 N)
        delta_l = 4.88
        pair_lifts = [
            13.95 + delta_l,  # Station 1 (+0.207m)
            13.95 - delta_l,  # Station 2 (+0.069m)
            13.95 - delta_l,  # Station 3 (-0.069m)
            13.95 + delta_l,  # Station 4 (-0.207m)
        ]

        net_moment = self.stabilizer.calculate_pitch_moment(pair_lifts)
        # Net pitching moment around CG must be identically zero
        self.assertAlmostEqual(net_moment, 0.0, places=5)

    def test_dynamic_residual_pitch_moment_cancellation(self):
        """Verify dynamic differential pitch trim counters measured residual moment."""
        sensors = {
            'comfort_mode': True,
            'pitch_moment_meas_Nm': +0.40,  # 0.40 Nm nose-up pitch disturbance
            'q_rad_s': +0.05,               # nose-up pitch rate
            'az_g': 1.0,
            'vsi_ms': 0.0,
            'pilot_pitch': 0.0,
        }

        cmds, offsets, active, diag = self.stabilizer.update(sensors, self.base_commands)
        self.assertTrue(active)
        pitch_couples = diag['pitch_couples_deg']

        # Foremost pair (Station 1, x = +0.207m) must trim nose-down (negative delta)
        # Rearmost pair (Station 4, x = -0.207m) must trim nose-up (positive delta)
        self.assertLess(pitch_couples[0], 0.0)
        self.assertGreater(pitch_couples[3], 0.0)
        self.assertAlmostEqual(pitch_couples[0], -pitch_couples[3], places=2)

        # Middle stations (+0.069m, -0.069m) must have exactly 1/3 the magnitude of the outer stations
        self.assertAlmostEqual(pitch_couples[1], pitch_couples[0] / 3.0, places=2)
        self.assertAlmostEqual(pitch_couples[2], pitch_couples[3] / 3.0, places=2)

        # Net collective lift trim from the differential moment couple must sum to zero
        self.assertAlmostEqual(sum(pitch_couples), 0.0, places=4)


class TestActiveHeaveDamping(unittest.TestCase):
    """Test active closed-loop heave damping for ride quality and cargo protection."""

    def setUp(self):
        self.stabilizer = ap.ActiveRideStabilizer()
        self.base_commands = [
            {'pitch_deg': 12.0, 'flap_amp_deg': 96.0, 'sweep_deg': 90.0} for _ in range(8)
        ]

    def test_updraft_positive_g_heave_rejection(self):
        """Updraft (az = 1.30g) commands negative collective pitch trim to suppress vertical spike."""
        sensors = {
            'comfort_mode': True,
            'az_g': 1.30,      # +0.30g vertical bump
            'vsi_ms': +0.8,    # climbing at 0.8 m/s
            'pilot_pitch': 0.0,
        }

        cmds, _, active, diag = self.stabilizer.update(sensors, self.base_commands)
        self.assertTrue(active)
        self.assertLess(diag['delta_alpha_heave_deg'], -0.5, "Should command negative collective pitch")

        # Flap amplitude should also be reduced on upward heave spike
        for cmd in cmds:
            self.assertLess(cmd['flap_amp_deg'], 96.0)
            self.assertLess(cmd['pitch_deg'], 12.0)

    def test_downdraft_negative_g_heave_rejection(self):
        """Downdraft (az = 0.75g) commands positive collective pitch trim to support airframe."""
        sensors = {
            'comfort_mode': True,
            'az_g': 0.75,      # -0.25g drop
            'vsi_ms': -0.6,    # sinking
            'pilot_pitch': 0.0,
        }

        cmds, _, active, diag = self.stabilizer.update(sensors, self.base_commands)
        self.assertTrue(active)
        self.assertGreater(diag['delta_alpha_heave_deg'], +0.5, "Should command positive collective pitch")

        for cmd in cmds:
            self.assertGreater(cmd['pitch_deg'], 12.0)

    def test_iso_2631_passenger_comfort_compliance(self):
        """Turbulence of 0.12g RMS is attenuated below the 0.031g ISO 2631 passenger comfort target."""
        sensors = {
            'comfort_mode': True,
            'imu_rms_g': 0.120,   # Significant turbulence
            'az_g': 1.0,
            'pilot_pitch': 0.0,
        }

        _, _, active, diag = self.stabilizer.update(sensors, self.base_commands)
        self.assertTrue(active)
        self.assertTrue(diag['iso_2631_compliant'])
        self.assertLess(diag['cabin_rms_g'], 0.031)
        self.assertGreater(diag['vibration_reduction_pct'], 85.0)

    def test_sensitive_cargo_g_envelope_enforcement(self):
        """Sensitive cargo mode restricts peak G envelope to <= 1.15g."""
        sensors = {
            'sensitive_cargo': True,
            'az_g': 1.25,
            'vsi_ms': 0.5,
            'pilot_pitch': 0.0,
        }

        _, _, active, diag = self.stabilizer.update(sensors, self.base_commands)
        self.assertTrue(active)
        self.assertTrue(diag['sensitive_cargo'])
        self.assertLessEqual(diag['cargo_peak_g'], 1.15)


class TestIntentAndBehaviours(unittest.TestCase):
    """Test Intent arbitration and Behaviour execution for Ride Comfort & Sensitive Cargo."""

    def setUp(self):
        self.interp = ic.IntentInterpreter()

    def test_intent_arbitration_ride_comfort(self):
        """Comfort mode or high vibration maps to Intent.RIDE_COMFORT."""
        intent, conf, params = self.interp.interpret(
            {'pitch': 0.0, 'throttle': 0.50, 'roll': 0.0, 'yaw': 0.0},
            {},
            {'on_ground': False, 'comfort_mode': True, 'V_ms': 9.0}
        )
        self.assertEqual(intent, ic.Intent.RIDE_COMFORT)
        self.assertTrue(params.get('heave_damping'))
        self.assertTrue(params.get('anti_phase_pitch'))

    def test_intent_arbitration_cargo_stabilize(self):
        """Sensitive / fragile payload maps to Intent.CARGO_STABILIZE."""
        intent, conf, params = self.interp.interpret(
            {'pitch': 0.0, 'throttle': 0.50, 'roll': 0.0, 'yaw': 0.0},
            {},
            {'on_ground': False, 'sensitive_cargo': True, 'V_ms': 9.0}
        )
        self.assertEqual(intent, ic.Intent.CARGO_STABILIZE)
        self.assertEqual(params.get('peak_g_limit'), 1.15)

    def test_pilot_stick_preemption_override(self):
        """Pilot positive pitch stick excursion (>0.12) overrides automated comfort trimming."""
        # Pilot commands strong aft stick (climb pitch = +0.65)
        sensors = {
            'comfort_mode': True,
            'pilot_pitch': 0.65,
            'az_g': 1.10,
        }
        stabilizer = ap.ActiveRideStabilizer()
        base_cmds = [{'pitch_deg': 12.0, 'flap_amp_deg': 96.0}] * 8

        cmds, _, active, diag = stabilizer.update(sensors, base_cmds)
        self.assertFalse(active, "Automated comfort stabilizer must yield when pilot stick active")
        self.assertTrue(diag.get('pilot_override'))

    def test_ride_comfort_behaviour_execution(self):
        """Verify RideComfortBehaviour executes and trims wing pitches."""
        class MockCtrl:
            _tgt_freq = 0.0
            _tgt_pitches = []
            _tgt_sweeps = []
            _tgt_dihedrals = []

        ctrl = MockCtrl()
        behaviour = ic.RideComfortBehaviour()
        ctx = {'az_g': 1.15, 'vsi_ms': 0.4, 'q_rad_s': 0.02}

        res = behaviour.tick(ctx, ctrl)
        self.assertEqual(res, ic.RUNNING)
        self.assertEqual(ctrl._tgt_freq, 2.0)
        self.assertEqual(len(ctrl._tgt_pitches), 8)
        # Under positive az (1.15g), collective pitch should trim down from base (11.0°)
        self.assertLess(ctrl._tgt_pitches[0], 11.0)


class TestMavlinkRideComfortBridge(unittest.TestCase):
    """Test MAVLink telemetry broadcasting and mode toggling."""

    def test_mavlink_comfort_mode_toggle_and_telemetry(self):
        bridge = mb.PixhawkBridge(port="/dev/null", baud=57600)
        state_init = bridge.get_state()
        self.assertFalse(state_init['comfort_mode'])
        self.assertFalse(state_init['sensitive_cargo'])

        # Enable comfort mode
        bridge.set_comfort_mode(True)
        self.assertTrue(bridge.get_state()['comfort_mode'])

        # Enable sensitive cargo mode
        bridge.set_sensitive_cargo(True)
        self.assertTrue(bridge.get_state()['sensitive_cargo'])

        # Broadcast telemetry text (should not throw)
        bridge.send_ride_comfort_status(active=True, vib_red_pct=88.5, cabin_rms_g=0.025, residual_my_Nm=0.02)
        bridge.close()


if __name__ == '__main__':
    unittest.main()
