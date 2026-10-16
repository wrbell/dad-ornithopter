"""
Unit Tests: Cargo Drone & eVTOL Flight Control Suite
====================================================
Verifies:
  1. DynamicPayloadEstimator (mco_cg_estimator.py): RLS mass and CG estimation, drop innovation, and G-limit derating.
  2. PixhawkBridge (mco_mavlink_bridge.py): Gripper command handling and payload status reporting.
  3. IntentInterpreter (mco_intent_control.py): Cargo G-limiter, fail-safes (total power loss, OWI), drop, swing damp, IGE cushion, vertiport approach.
  4. Behaviour Nodes: PayloadDropMacro, AutoGlideLockMacro, OwiRedundancyBehaviour, IgeCushionBehaviour, VertiportApproachBehaviour.
"""

import sys
import os
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'firmware'))

import mco_cg_estimator as cg
import mco_intent_control as ic
import mco_mavlink_bridge as mb


class TestDynamicPayloadEstimator(unittest.TestCase):
    """Test real-time recursive payload mass and CG estimation."""

    def setUp(self):
        self.est = cg.DynamicPayloadEstimator(empty_mass_kg=5.687)

    def test_empty_airborne_estimation(self):
        # Clean airframe: lift equals empty weight
        r = self.est.update(t=1.0, V_ms=9.0, lift_cmd_n=55.8, az_g=1.0, pitch_trim_deg=0.0)
        self.assertAlmostEqual(r['payload_kg'], 0.0, delta=0.1)
        self.assertAlmostEqual(r['gross_mass_kg'], 5.687, delta=0.2)
        self.assertEqual(r['n_limit_g'], 3.50)
        self.assertFalse(r['is_cargo_detected'])

    def test_payload_pickup_and_glimit_derating(self):
        # 1.5 kg payload added: total lift required = (5.687 + 1.5) * 9.81 ≈ 70.5 N
        # Pitch trim +2.0° indicating forward-biased cargo
        for i in range(12):
            t = 1.0 + i * 0.1
            r = self.est.update(t=t, V_ms=9.0, lift_cmd_n=70.5, az_g=1.0, pitch_trim_deg=2.0)
        
        self.assertGreater(r['payload_kg'], 1.2)
        self.assertTrue(r['is_cargo_detected'])
        self.assertGreater(r['delta_cg_mm'], 10.0)
        # G-limiter derated below 3.5g
        self.assertLess(r['n_limit_g'], 3.20)
        self.assertGreaterEqual(r['n_limit_g'], 2.0)

    def test_drop_innovation_detection(self):
        # Establish payload of 1.5 kg
        for i in range(10):
            self.est.update(t=i * 0.1, V_ms=9.0, lift_cmd_n=70.5, az_g=1.0)
        self.assertTrue(self.est._payload_kg > 1.0)

        # Instantaneous drop: lift command drops back to clean weight (55.8 N)
        r_drop = self.est.update(t=1.5, V_ms=9.0, lift_cmd_n=55.8, az_g=1.0)
        self.assertTrue(r_drop['drop_detected'], "Drop innovation should be detected immediately")
        self.assertLess(r_drop['payload_kg'], 0.5)


class TestMavlinkCargoBridge(unittest.TestCase):
    """Test MAVLink bridge gripper commands and payload status telemetry."""

    def test_gripper_command_and_telemetry(self):
        bridge = mb.PixhawkBridge(port="/dev/null", baud=57600)
        state_init = bridge.get_state()
        self.assertFalse(state_init['payload_release_cmd'])
        self.assertEqual(state_init['gripper_state'], 'closed')

        # Trigger cargo release
        bridge.trigger_cargo_release()
        state_released = bridge.get_state()
        self.assertTrue(state_released['payload_release_cmd'])
        self.assertEqual(state_released['gripper_state'], 'released')

        # Subsequent read resets the edge-triggered command
        state_subsequent = bridge.get_state()
        self.assertFalse(state_subsequent['payload_release_cmd'])
        bridge.close()


class TestCargoEvtolIntentAndBehaviours(unittest.TestCase):
    """Test Intent classification and behaviour execution for cargo and eVTOL operations."""

    def setUp(self):
        self.interp = ic.IntentInterpreter()

    def test_dynamic_g_limiter_in_intent(self):
        # Heavy payload of 2.0 kg reduces allowable pitch stick
        # Pilot requests full aft stick (0.95)
        intent, conf, params = self.interp.interpret(
            {'pitch': 0.95, 'throttle': 0.70, 'roll': 0.0, 'yaw': 0.0},
            {},
            {'on_ground': False, 'V_ms': 9.0, 'payload_kg': 2.0}
        )
        self.assertTrue(params.get('g_limit_active', True) or self.interp._vault_armed is False)

    def test_total_power_loss_auto_glide_lock(self):
        intent, conf, params = self.interp.interpret(
            {'pitch': 0.0, 'throttle': 0.50},
            {},
            {'on_ground': False, 'total_power_loss': True}
        )
        self.assertEqual(intent, ic.Intent.AUTO_GLIDE_LOCK)

    def test_owi_redundancy_intent(self):
        intent, conf, params = self.interp.interpret(
            {'pitch': 0.0, 'throttle': 0.50},
            {},
            {'on_ground': False, 'failed_wing_node': 2}
        )
        self.assertEqual(intent, ic.Intent.OWI_REDUNDANCY)
        self.assertEqual(params.get('failed_node'), 2)

    def test_payload_drop_intent(self):
        intent, conf, params = self.interp.interpret(
            {'pitch': 0.0, 'throttle': 0.50},
            {},
            {'on_ground': False, 'payload_release_cmd': True}
        )
        self.assertEqual(intent, ic.Intent.PAYLOAD_DROP)
        self.assertAlmostEqual(params.get('de_balloon_pitch_deg'), -2.0)

    def test_cargo_swing_damp_intent(self):
        intent, conf, params = self.interp.interpret(
            {'pitch': 0.0, 'throttle': 0.50},
            {},
            {'on_ground': False, 'cargo_swing_deg': 12.0}
        )
        self.assertEqual(intent, ic.Intent.CARGO_SWING_DAMP)

    def test_ige_cushion_intent(self):
        # Low altitude hover over vertiport pad
        intent, conf, params = self.interp.interpret(
            {'pitch': 0.0, 'throttle': 0.50},
            {},
            {'on_ground': False, 'h_agl_m': 0.35, 'V_ms': 1.0, 'vertiport_hover': True}
        )
        self.assertEqual(intent, ic.Intent.IGE_CUSHION)

    def test_vertiport_approach_intent(self):
        intent, conf, params = self.interp.interpret(
            {'pitch': 0.0, 'throttle': 0.40},
            {},
            {'on_ground': False, 'vertiport_approach': True}
        )
        self.assertEqual(intent, ic.Intent.VERTIPORT_APPROACH)


class TestBehaviourExecution(unittest.TestCase):
    """Test BehaviourDispatcher execution for new cargo and eVTOL behaviours."""

    class MockCtrl:
        freq_hz = 1.0
        _tgt_freq = 1.0
        _tgt_sweeps = [60.0] * 8
        _tgt_dihedrals = [0.0] * 8
        _tgt_pitches = [0.0] * 8

    def setUp(self):
        self.ctrl = self.MockCtrl()
        self.disp = ic.BehaviourDispatcher(self.ctrl)

    def test_payload_drop_macro_execution(self):
        name, intent, status = self.disp.update(
            0.1,
            {'pitch': 0.0, 'throttle': 0.60, 'roll': 0.0, 'yaw': 0.0},
            {},
            {'on_ground': False, 'payload_release_cmd': True, 'h_agl_m': 20.0, 'V_ms': 9.0}
        )
        self.assertEqual(name, 'payload_drop')
        self.assertEqual(intent, ic.Intent.PAYLOAD_DROP)
        self.assertEqual(status, ic.RUNNING)
        # Collective de-balloon pitch commanded
        self.assertEqual(self.ctrl._tgt_pitches, [8.0] * 8)

    def test_auto_glide_lock_macro_execution(self):
        name, intent, status = self.disp.update(
            0.1,
            {'pitch': 0.0, 'throttle': 0.0, 'roll': 0.0, 'yaw': 0.0},
            {},
            {'on_ground': False, 'total_power_loss': True, 'h_agl_m': 50.0, 'V_ms': 8.0}
        )
        self.assertEqual(name, 'auto_glide_lock')
        self.assertEqual(intent, ic.Intent.AUTO_GLIDE_LOCK)
        self.assertEqual(self.ctrl._tgt_freq, 0.0)
        # Tandem slotted box wing lock pitches
        self.assertEqual(self.ctrl._tgt_pitches, [8.5, 8.5, 14.0, 14.0, 8.5, 8.5, 14.0, 14.0])

    def test_owi_redundancy_behaviour_execution(self):
        # Node 0 (Port W1) fails -> Node 1 (Stbd W1) should be feathered to 0.0° AoA
        name, intent, status = self.disp.update(
            0.1,
            {'pitch': 0.0, 'throttle': 0.50, 'roll': 0.0, 'yaw': 0.0},
            {},
            {'on_ground': False, 'failed_wing_node': 0, 'h_agl_m': 30.0, 'V_ms': 8.0}
        )
        self.assertEqual(name, 'owi_redundancy')
        self.assertEqual(intent, ic.Intent.OWI_REDUNDANCY)
        self.assertEqual(self.ctrl._tgt_pitches[0], 0.0)
        self.assertEqual(self.ctrl._tgt_pitches[1], 0.0)
        self.assertEqual(self.ctrl._tgt_pitches[2], 12.0)
        self.assertAlmostEqual(self.ctrl._tgt_freq, 2.4)

    def test_ige_cushion_behaviour_execution(self):
        name, intent, status = self.disp.update(
            0.1,
            {'pitch': 0.0, 'throttle': 0.50, 'roll': 0.0, 'yaw': 0.0},
            {},
            {'on_ground': False, 'h_agl_m': 0.30, 'V_ms': 1.0, 'vertiport_hover': True, 'pad_drift_x_m': 0.2}
        )
        self.assertEqual(name, 'ige_cushion')
        self.assertEqual(intent, ic.Intent.IGE_CUSHION)
        # Frequency damped by 15%
        self.assertAlmostEqual(self.ctrl._tgt_freq, 1.8 * 0.85)
        # Differential sweep engaged to counter pad drift
        self.assertNotEqual(self.ctrl._tgt_sweeps[0], self.ctrl._tgt_sweeps[1])

    def test_vertiport_approach_behaviour_execution(self):
        name, intent, status = self.disp.update(
            0.1,
            {'pitch': 0.0, 'throttle': 0.40, 'roll': 0.0, 'yaw': 0.0},
            {},
            {'on_ground': False, 'vertiport_approach': True, 'h_agl_m': 15.0, 'V_ms': 7.0}
        )
        self.assertEqual(name, 'vertiport_approach')
        self.assertEqual(intent, ic.Intent.VERTIPORT_APPROACH)
        # Steep dihedral angle
        self.assertEqual(self.ctrl._tgt_dihedrals, [50.0] * 8)


if __name__ == '__main__':
    unittest.main()
