"""
Unit Tests: Conventional Fixed-Wing Takeoff Roll & Dual-Cascade Rotation Suite
=============================================================================
Verifies:
  1. Cracked-throttle taxiing with rudder pedal yaw steering (ch4).
  2. Direct sideways ground translation (crab taxi) via lateral roll stick (ch1).
  3. Ground roll acceleration (Intent.TAKEOFF_ROLL) with neutral stick and full throttle.
  4. Dual-cascade rotation (Intent.ROTATE) via aft stick across all 5 operational regimes:
     - Standard CTOL Takeoff
     - High Density Altitude Takeoff
     - Overweight Takeoff
     - Short Field Takeoff
     - Soft Field Takeoff
  5. Clean handoff from rotation to climb upon unstick.
  6. Isolation of VTOL launch triggers (Dragonfly/Pterosaur) from standard runway operations.
"""

import math
import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'firmware'))

import mco_advanced_physics as ap
import mco_intent_control as ic
import mco_taxi_park as tp


class TestTaxiSteeringAndTranslation(unittest.TestCase):
    """Test taxi commands, rudder pedal steering, and lateral stick translation."""

    def test_cracked_throttle_taxi_speed_and_clearance(self):
        """Cracked throttle (0.15 to 0.35) scales frequency (0.2 to 0.5 Hz) and clamps downstroke to -4°."""
        cmd_idle = tp.taxi_commands(throttle_frac=0.10, yaw_cmd=0.0)
        self.assertAlmostEqual(cmd_idle['freq_hz'], 0.28, places=2)
        self.assertEqual(cmd_idle['wing_cmds'][0]['amp_down'], tp.TAXI_AMP_DOWN_DEG)

        cmd_cruise = tp.taxi_commands(throttle_frac=0.35, yaw_cmd=0.0)
        self.assertAlmostEqual(cmd_cruise['freq_hz'], 0.48, places=2)
        for wc in cmd_cruise['wing_cmds']:
            self.assertEqual(wc['amp_down'], tp.TAXI_AMP_DOWN_DEG)
            self.assertEqual(wc['sweep'], 60.0)

    def test_rudder_pedal_yaw_steering(self):
        """Rudder pedals (yaw_cmd) create differential upstroke amplitude between port and starboard."""
        # Right pedal: port wings stroke higher to push nose right
        cmd_right = tp.taxi_commands(throttle_frac=0.30, yaw_cmd=0.8)
        port_up = cmd_right['wing_cmds'][0]['amp_up']
        stbd_up = cmd_right['wing_cmds'][1]['amp_up']
        self.assertGreater(port_up, stbd_up, "Port upstroke must be greater than starboard for right turn")

        # Left pedal: starboard wings stroke higher to push nose left
        cmd_left = tp.taxi_commands(throttle_frac=0.30, yaw_cmd=-0.8)
        port_up_l = cmd_left['wing_cmds'][0]['amp_up']
        stbd_up_l = cmd_left['wing_cmds'][1]['amp_up']
        self.assertGreater(stbd_up_l, port_up_l, "Starboard upstroke must be greater than port for left turn")

    def test_direct_sideways_translation_via_lateral_stick(self):
        """Roll stick (lateral_cmd) commands differential AoA and dihedral for direct sideways crab."""
        # Stick deflected right (lateral_cmd = +0.75)
        cmd_lat_r = tp.taxi_commands(throttle_frac=0.25, yaw_cmd=0.0, lateral_cmd=0.75)
        self.assertTrue(cmd_lat_r['lateral_active'])
        port_wc = cmd_lat_r['wing_cmds'][0]
        stbd_wc = cmd_lat_r['wing_cmds'][1]

        # Port wings should have higher pitch and higher dihedral than starboard
        self.assertGreater(port_wc['pitch'], stbd_wc['pitch'])
        self.assertGreater(port_wc['dihedral_deg'], stbd_wc['dihedral_deg'])

        # Stick centered (within deadband)
        cmd_center = tp.taxi_commands(throttle_frac=0.25, yaw_cmd=0.0, lateral_cmd=0.02)
        self.assertFalse(cmd_center['lateral_active'])
        self.assertEqual(cmd_center['wing_cmds'][0]['pitch'], cmd_center['wing_cmds'][1]['pitch'])


class TestTakeoffRollAndIsolation(unittest.TestCase):
    """Test runway ground roll acceleration and VTOL isolation."""

    def setUp(self):
        self.interp = ic.IntentInterpreter()

    def test_runway_full_throttle_triggers_takeoff_roll_not_vtol(self):
        """Advancing throttle to full with neutral stick initiates TAKEOFF_ROLL, NOT Dragonfly jump."""
        sticks = {'roll': 0.0, 'pitch': 0.0, 'throttle': 0.95, 'yaw': 0.0}
        switches = {'sw_arm': 1, 'sw_mode': 0, 'sw_launch': False}
        context = {'on_ground': True, 'h_agl_m': 0.0, 'V_ms': 1.0}

        intent, conf, params = self.interp.interpret(sticks, switches, context)
        self.assertEqual(intent, ic.Intent.TAKEOFF_ROLL, "Standard throttle advancement must initiate TAKEOFF_ROLL")
        self.assertGreaterEqual(conf, 0.90)

    def test_takeoff_roll_kinematics_progression(self):
        """Takeoff roll relaxes taxi constraint and builds stroke amplitude with speed."""
        cmds_0, freq_0, diag_0 = ap.ctol_takeoff_roll_kinematics(throttle_frac=1.0, V_ms=0.5)
        self.assertEqual(diag_0['status'], 'TAKEOFF_ROLL_ACCELERATING')
        self.assertFalse(diag_0['ready_to_rotate'])
        self.assertAlmostEqual(cmds_0[0]['flap_amp_deg'], 77.3, places=1)

        cmds_fast, freq_fast, diag_fast = ap.ctol_takeoff_roll_kinematics(throttle_frac=1.0, V_ms=4.6)
        self.assertTrue(diag_fast['ready_to_rotate'])
        self.assertAlmostEqual(cmds_fast[0]['flap_amp_deg'], 96.0, places=1)

    def test_vtol_requires_explicit_demand(self):
        """Dragonfly super-lift triggers ONLY when explicit demand or launch switch is set."""
        sticks = {'roll': 0.0, 'pitch': 0.0, 'throttle': 0.95, 'yaw': 0.0}
        switches = {'sw_arm': 1, 'sw_mode': 0, 'sw_launch': False}
        context = {'on_ground': True, 'h_agl_m': 0.0, 'V_ms': 1.0, 'dragonfly_launch_demand': True}

        intent, _, params = self.interp.interpret(sticks, switches, context)
        self.assertEqual(intent, ic.Intent.DRAGONFLY_SUPER_LIFT)


class TestDualCascadeRotation(unittest.TestCase):
    """Test dual-cascade rotation across all 5 operational regimes."""

    def setUp(self):
        self.interp = ic.IntentInterpreter()

    def test_rotation_intent_trigger(self):
        """At rotation speed (V >= 3.5 m/s), aft pitch stick (> 0.15) triggers Intent.ROTATE."""
        sticks = {'roll': 0.0, 'pitch': 0.45, 'throttle': 0.90, 'yaw': 0.0}
        switches = {'sw_arm': 1, 'sw_mode': 0}
        context = {'on_ground': True, 'h_agl_m': 0.0, 'V_ms': 4.5}

        intent, conf, params = self.interp.interpret(sticks, switches, context)
        self.assertEqual(intent, ic.Intent.ROTATE)
        self.assertAlmostEqual(params['pitch'], 0.45)

    def test_standard_rotation_canard_elevon_blend(self):
        """Standard rotation applies canard direct lift on front pairs and elevon trim on rear pairs."""
        cmds, freq, diag = ap.ctol_dual_cascade_rotation(pitch_stick=0.80, V_ms=4.5, regime='standard')
        self.assertEqual(diag['status'], 'DUAL_CASCADE_ROTATION_ACTIVE')
        self.assertTrue(diag['canard_lift_priority'])
        self.assertTrue(diag['tail_strike_prevented'])

        # Front pairs (0..3) have higher AoA than rear pairs (4..7)
        front_pitch = cmds[0]['pitch_deg']
        rear_pitch = cmds[4]['pitch_deg']
        self.assertGreater(front_pitch, rear_pitch, "Front canard AoA must exceed rear elevon AoA")
        self.assertGreater(cmds[0]['flap_amp_deg'], cmds[4]['flap_amp_deg'], "Front amplitude boosted for direct lift")

    def test_high_density_altitude_rotation(self):
        """High DA regime uses higher stroke frequency (2.6 Hz) and stronger front canard lift."""
        cmds, freq, diag = ap.ctol_dual_cascade_rotation(pitch_stick=0.80, V_ms=5.2, regime='high_da')
        self.assertEqual(freq, 2.6)
        self.assertGreaterEqual(diag['front_aoa_deg'], 14.0)

    def test_overweight_rotation(self):
        """Overweight regime delays rotation to 6.8 m/s and uses progressive moderate AoA."""
        _, _, diag = ap.ctol_dual_cascade_rotation(pitch_stick=0.50, V_ms=6.8, regime='overweight')
        self.assertAlmostEqual(diag['unstick_target_m'], 0.45)
        self.assertLessEqual(diag['front_aoa_deg'], 12.0)

    def test_short_field_rotation(self):
        """Short field uses 55° forward sweep for high static thrust and rotates at 3.8 m/s."""
        cmds, freq, diag = ap.ctol_dual_cascade_rotation(pitch_stick=1.0, V_ms=3.8, regime='short_field')
        self.assertEqual(cmds[0]['sweep_deg'], 55.0)
        self.assertEqual(freq, 2.5)

    def test_soft_field_takeoff_profile(self):
        """Soft field allows early unstick at 1.15 Vs_IGE (V >= 6.0 m/s) into low ground effect (0.25 m)."""
        cmds, freq, diag = ap.ctol_dual_cascade_rotation(pitch_stick=0.70, V_ms=6.0, regime='soft_field')
        self.assertEqual(diag['unstick_target_m'], 0.25)


class MockFlightController:
    def __init__(self):
        self._tgt_freq = 2.0
        self._tgt_sweeps = [90.0] * 8
        self._tgt_dihedrals = [0.0] * 8
        self._tgt_pitches = [12.0] * 8


class TestRotateLifecycleHandoff(unittest.TestCase):
    """Test RotateBehaviour tick lifecycle and handoff to climb."""

    def test_rotate_handoff_to_climb_on_unstick(self):
        """RotateBehaviour returns SUCCESS once height reaches unstick target or positive climb."""
        beh = ic.RotateBehaviour()
        ctrl = MockFlightController()

        beh.on_enter({'t': 0.0})

        # Tick 1: on ground, no climb -> RUNNING
        ctx_gnd = {'t': 0.1, 'pitch': 0.5, 'V_ms': 4.5, 'h_agl_m': 0.1, 'climb_rate_ms': 0.0}
        status1 = beh.tick(ctx_gnd, ctrl)
        self.assertEqual(status1, ic.RUNNING)

        # Tick 2: unstick height achieved (0.38 m > 0.37 m) -> SUCCESS
        ctx_air = {'t': 0.3, 'pitch': 0.5, 'V_ms': 5.0, 'h_agl_m': 0.38, 'climb_rate_ms': 0.8}
        status2 = beh.tick(ctx_air, ctrl)
        self.assertEqual(status2, ic.SUCCESS, "Should declare SUCCESS on unstick to hand off to CLIMB")


if __name__ == '__main__':
    unittest.main()
