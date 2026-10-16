"""
Tests for Universal Go-Around & Balked Landing Suite across all landing regimes.
Validates:
  1. Cram (Power): immediate ramp to max climb frequency (2.8 to 3.5 Hz)
  2. Climb (Pitch): Canard Direct Lift pulse (W1-W2) arresting sink rate in <= 0.20s
  3. Clean (Configuration): instant airbrake dihedral collapse and forward rake sweep
  4. Anti-Tail-Strike Protection: W3-W4 elevator trim maintaining >= 150mm clearance
  5. Priority Preemption: Priority 3 preemption across approach, flare, rollout, and glide
  6. Smooth handoff to steady-state CLIMB once positive climb is established
"""

import unittest
import math
import sys
import os

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../firmware')))

import mco_advanced_physics as ap
import mco_intent_control as ic


class MockController:
    def __init__(self):
        self.freq_hz = 0.0
        self._tgt_freq = 0.0
        self._tgt_sweeps = [50.0] * 8
        self._tgt_dihedrals = [6.0] * 8
        self._tgt_pitches = [6.0] * 8
        self._intent_name = 'approach_normal'
        self.flare_diag = {}
        self.rollout_diag = {}
        self.go_around_diag = {}


class TestGoAroundSuite(unittest.TestCase):

    def setUp(self):
        self.ctrl = MockController()
        self.interp = ic.IntentInterpreter()

    def test_01_balked_landing_kinematics_physics(self):
        """Test balked_landing_go_around_kinematics core aerodynamics and structural bounds."""
        cmds, freq, diag = ap.balked_landing_go_around_kinematics(
            pitch_stick=0.20, throttle=1.0, h_agl_m=0.80, V_ms=6.0,
            landing_regime='standard', vz_ms=-0.75
        )

        # 1. Power: frequency should ramp to maximum climb power (>= 3.2 Hz)
        self.assertGreaterEqual(freq, 3.20)
        self.assertLessEqual(freq, 3.55)

        # 2. Canard Direct Lift pulse on W1-W2 (+12° to +16° AoA)
        for i in range(4):
            self.assertGreaterEqual(cmds[i]['pitch_deg'], 12.0)
            self.assertLessEqual(cmds[i]['pitch_deg'], 16.0)

        # 3. Configuration clean-up: clean climb sweep (52°) and dihedral (6°)
        for i in range(8):
            self.assertAlmostEqual(cmds[i]['sweep_deg'], 52.0, delta=1.0)
            self.assertAlmostEqual(cmds[i]['dihedral_deg'], 6.0, delta=1.0)

        # 4. Tail clearance protection: rear wing W4 clearance must satisfy >= 150mm margin
        self.assertTrue(diag['tail_strike_safe'])
        self.assertGreaterEqual(diag['tail_clearance_w4_m'], 0.150)
        self.assertTrue(diag['sink_rate_arrested'])
        self.assertGreater(diag['climb_thrust_n'], 30.0)

    def test_02_dynamic_q_aoa_protection_during_go_around(self):
        """Verify dynamic pressure q-limiter bounds AoA at high go-around airspeeds (nz <= 2.2g)."""
        cmds_fast, freq_fast, diag_fast = ap.balked_landing_go_around_kinematics(
            pitch_stick=0.50, throttle=1.0, h_agl_m=10.0, V_ms=13.5,
            landing_regime='standard', vz_ms=-0.20
        )
        # At 13.5 m/s, AoA must be throttled back to prevent airframe over-G
        for c in cmds_fast:
            self.assertLessEqual(c['pitch_deg'], 11.5)

    def test_03_intent_interpreter_recognizes_go_around_from_all_landing_contexts(self):
        """Verify IntentInterpreter correctly identifies Intent.GO_AROUND across landing contexts."""
        # A. From Approach Corridor
        ctx_app = {
            'regime': 'standard_power_off',
            'h_agl_m': 20.0,
            'V_ms': 9.0,
            'landing_approach': True,
        }
        self.interp._last_intent = ic.Intent.APPROACH_NORMAL
        intent, conf, _ = self.interp.interpret(
            {'ch1': 0.0, 'ch2': 0.15, 'ch3': 0.85, 'ch4': 0.0},
            {}, ctx_app
        )
        self.assertEqual(intent, ic.Intent.GO_AROUND)
        self.assertGreaterEqual(conf, 0.95)

        # B. From Low Flare (h = 0.8m)
        ctx_flare = {
            'regime': 'commercial_powered',
            'h_agl_m': 0.8,
            'V_ms': 6.2,
            'landing_flare': True,
        }
        self.interp._last_intent = ic.Intent.LANDING_FLARE
        intent, conf, _ = self.interp.interpret(
            {'ch1': 0.0, 'ch2': 0.25, 'ch3': 0.90, 'ch4': 0.0},
            {}, ctx_flare
        )
        self.assertEqual(intent, ic.Intent.GO_AROUND)

        # C. From Runway Rollout with explicit go-around demand
        ctx_roll = {
            'regime': 'short_field',
            'h_agl_m': 0.0,
            'V_ms': 4.8,
            'on_ground': True,
            'touchdown_rollout': True,
            'go_around_demand': True,
        }
        self.interp._last_intent = ic.Intent.TOUCHDOWN_ROLLOUT
        intent, conf, _ = self.interp.interpret(
            {'ch1': 0.0, 'ch2': 0.20, 'ch3': 0.80, 'ch4': 0.0},
            {}, ctx_roll
        )
        self.assertEqual(intent, ic.Intent.GO_AROUND)

    def test_04_normal_takeoff_roll_not_confused_with_go_around(self):
        """Verify standard takeoff roll from standstill on the runway is NOT misidentified as go-around."""
        ctx_to = {
            'regime': 'standard',
            'takeoff_regime': 'standard',
            'h_agl_m': 0.0,
            'V_ms': 2.0,
            'on_ground': True,
        }
        self.interp._last_intent = ic.Intent.TAKEOFF_ROLL
        intent, conf, _ = self.interp.interpret(
            {'ch1': 0.0, 'ch2': 0.0, 'ch3': 0.85, 'ch4': 0.0},
            {}, ctx_to
        )
        self.assertEqual(intent, ic.Intent.TAKEOFF_ROLL)

    def test_05_high_approach_go_around_preemption(self):
        """Verify GoAroundBehaviour instantly preempts ApproachBehaviour at 25m AGL."""
        disp = ic.BehaviourDispatcher(self.ctrl)
        ctx_app = {
            'throttle': 0.30, 'pitch': 0.0, 'V_ms': 9.5, 'h_agl_m': 25.0,
            'regime': 'standard_power_off', 'on_ground': False, 'landing_approach': True,
        }
        disp.update(t=1.0, sticks={'pitch': 0.0, 'throttle': 0.30}, switches={}, context=ctx_app)
        self.assertIsInstance(disp.active, ic.ApproachBehaviour)

        # Pilot initiates go-around: throttle slammed to 95%
        ctx_ga = dict(ctx_app)
        ctx_ga['throttle'] = 0.95
        ctx_ga['pitch'] = 0.20
        disp.update(t=1.02, sticks={'pitch': 0.20, 'throttle': 0.95}, switches={}, context=ctx_ga)
        self.assertIsInstance(disp.active, ic.GoAroundBehaviour)
        self.assertGreaterEqual(self.ctrl._tgt_freq, 3.20)

    def test_06_in_flare_balked_landing_preemption(self):
        """Verify GoAroundBehaviour instantly preempts FlareBehaviour at 0.9m AGL in ground effect."""
        disp = ic.BehaviourDispatcher(self.ctrl)
        ctx_flare = {
            'throttle': 0.05, 'pitch': 0.25, 'V_ms': 6.0, 'h_agl_m': 0.9,
            'regime': 'standard_power_off', 'on_ground': False, 'landing_flare': True, 'vz_ms': -0.70,
        }
        disp.update(t=5.0, sticks={'pitch': 0.25, 'throttle': 0.05}, switches={}, context=ctx_flare)
        self.assertIsInstance(disp.active, ic.FlareBehaviour)

        # Pilot calls go-around in the flare: full TOGA power
        ctx_balk = dict(ctx_flare)
        ctx_balk['throttle'] = 1.0
        ctx_balk['pitch'] = 0.20
        disp.update(t=5.02, sticks={'pitch': 0.20, 'throttle': 1.0}, switches={}, context=ctx_balk)
        self.assertIsInstance(disp.active, ic.GoAroundBehaviour)
        self.assertGreaterEqual(self.ctrl._tgt_freq, 3.20)
        self.assertTrue(self.ctrl.go_around_diag['tail_strike_safe'])
        self.assertGreaterEqual(self.ctrl.go_around_diag['tail_clearance_w4_m'], 0.150)

    def test_07_soft_field_flare_go_around(self):
        """Verify soft-field nosewheel hold-off go-around maintains tail clearance and stops sink."""
        disp = ic.BehaviourDispatcher(self.ctrl)
        ctx_soft = {
            'throttle': 0.15, 'pitch': 0.40, 'V_ms': 5.2, 'h_agl_m': 0.50,
            'regime': 'soft_field', 'on_ground': False, 'landing_flare': True, 'vz_ms': -0.45,
        }
        disp.update(t=10.0, sticks={'pitch': 0.40, 'throttle': 0.15}, switches={}, context=ctx_soft)
        self.assertIsInstance(disp.active, ic.FlareBehaviour)

        # Punch TOGA power to go around from soft-field flare
        ctx_soft_ga = dict(ctx_soft)
        ctx_soft_ga['throttle'] = 0.95
        ctx_soft_ga['pitch'] = 0.25
        disp.update(t=10.02, sticks={'pitch': 0.25, 'throttle': 0.95}, switches={}, context=ctx_soft_ga)
        self.assertIsInstance(disp.active, ic.GoAroundBehaviour)
        self.assertTrue(self.ctrl.go_around_diag['tail_strike_safe'])

    def test_08_short_field_backside_balked_landing(self):
        """Verify go-around behind the power curve spools max thrust and accelerates out of backside."""
        disp = ic.BehaviourDispatcher(self.ctrl)
        ctx_short = {
            'throttle': 0.35, 'pitch': 0.10, 'V_ms': 5.8, 'h_agl_m': 4.5,
            'regime': 'short_field', 'on_ground': False, 'landing_approach': True, 'vz_ms': -0.90,
        }
        disp.update(t=15.0, sticks={'pitch': 0.10, 'throttle': 0.35}, switches={}, context=ctx_short)
        self.assertIsInstance(disp.active, ic.ApproachBehaviour)

        # Balked landing behind the power curve
        ctx_short_balk = dict(ctx_short)
        ctx_short_balk['throttle'] = 1.0
        ctx_short_balk['pitch'] = 0.15
        disp.update(t=15.02, sticks={'pitch': 0.15, 'throttle': 1.0}, switches={}, context=ctx_short_balk)
        self.assertIsInstance(disp.active, ic.GoAroundBehaviour)
        self.assertGreaterEqual(self.ctrl._tgt_freq, 3.30)
        self.assertGreater(self.ctrl.go_around_diag['lift_surge_n'], 45.0)

    def test_09_stol_bush_chop_and_drop_balked_landing(self):
        """Verify last-second go-around after STOL throttle chop instantly re-engages flapping lift."""
        disp = ic.BehaviourDispatcher(self.ctrl)
        # STOL flare after throttle chopped to idle (0.0 Hz)
        ctx_chop = {
            'throttle': 0.0, 'pitch': 0.30, 'V_ms': 5.2, 'h_agl_m': 0.70,
            'regime': 'stol', 'on_ground': False, 'landing_flare': True, 'vz_ms': -0.60,
        }
        disp.update(t=20.0, sticks={'pitch': 0.30, 'throttle': 0.0}, switches={}, context=ctx_chop)
        self.assertIsInstance(disp.active, ic.FlareBehaviour)
        self.assertEqual(self.ctrl._tgt_freq, 0.0)

        # Pilot calls balked landing: slam throttle to 100%
        ctx_chop_ga = dict(ctx_chop)
        ctx_chop_ga['throttle'] = 1.0
        ctx_chop_ga['pitch'] = 0.20
        disp.update(t=20.02, sticks={'pitch': 0.20, 'throttle': 1.0}, switches={}, context=ctx_chop_ga)
        self.assertIsInstance(disp.active, ic.GoAroundBehaviour)
        # Flapping must immediately restart at >= 3.2 Hz, canceling the chop lift-dump
        self.assertGreaterEqual(self.ctrl._tgt_freq, 3.20)
        self.assertTrue(self.ctrl.go_around_diag['tail_strike_safe'])

    def test_10_rejected_rollout_touch_and_go_balked_landing(self):
        """Verify rejected rollout with active Shuttlecock bluff brake immediately cleans up and unsticks."""
        disp = ic.BehaviourDispatcher(self.ctrl)
        # On runway in Shuttlecock brake rollout (lever = 20%)
        ctx_roll = {
            'lever_pct': 20.0, 'throttle': 0.0, 'V_ms': 5.0, 'h_agl_m': 0.0,
            'regime': 'standard_power_off', 'on_ground': True, 'touchdown_rollout': True,
        }
        disp.update(t=25.0, sticks={'throttle': 0.0, 'lever_pct': 20.0}, switches={}, context=ctx_roll)
        self.assertIsInstance(disp.active, ic.LandingRolloutBehaviour)
        self.assertIn('SHUTTLECOCK', self.ctrl.rollout_diag['modality'])

        # Pilot pushes TOGA power to abort rollout and go around
        ctx_roll_ga = dict(ctx_roll)
        ctx_roll_ga['throttle'] = 0.90
        ctx_roll_ga['lever_pct'] = 88.0
        ctx_roll_ga['pitch'] = 0.20
        ctx_roll_ga['go_around_demand'] = True
        disp.update(t=25.02, sticks={'pitch': 0.20, 'throttle': 0.90, 'lever_pct': 88.0}, switches={}, context=ctx_roll_ga)
        self.assertIsInstance(disp.active, ic.GoAroundBehaviour)
        # Bluff cup must be collapsed: dihedrals reset from +70° to clean climb 6°
        for d in self.ctrl._tgt_dihedrals:
            self.assertAlmostEqual(d, 6.0, delta=1.0)
        self.assertGreaterEqual(self.ctrl._tgt_freq, 3.20)

    def test_11_tailwheel_3point_balked_landing(self):
        """Verify tailwheel 3-point landing go-around protects tail and un-stalls wings smoothly."""
        disp = ic.BehaviourDispatcher(self.ctrl)
        ctx_tw = {
            'throttle': 0.05, 'pitch': 0.35, 'V_ms': 5.4, 'h_agl_m': 0.35,
            'regime': 'tailwheel_3point', 'on_ground': False, 'landing_flare': True, 'vz_ms': -0.30,
        }
        disp.update(t=30.0, sticks={'pitch': 0.35, 'throttle': 0.05}, switches={}, context=ctx_tw)
        self.assertIsInstance(disp.active, ic.FlareBehaviour)

        # Balked landing
        ctx_tw_balk = dict(ctx_tw)
        ctx_tw_balk['throttle'] = 0.95
        ctx_tw_balk['pitch'] = 0.20
        disp.update(t=30.02, sticks={'pitch': 0.20, 'throttle': 0.95}, switches={}, context=ctx_tw_balk)
        self.assertIsInstance(disp.active, ic.GoAroundBehaviour)
        self.assertTrue(self.ctrl.go_around_diag['tail_strike_safe'])

    def test_12_go_around_to_climb_handoff(self):
        """Verify GoAroundBehaviour exits to SUCCESS and hands off to CLIMB once positive climb is established."""
        disp = ic.BehaviourDispatcher(self.ctrl)
        ctx_balk = {
            'throttle': 1.0, 'pitch': 0.25, 'V_ms': 7.5, 'h_agl_m': 2.0,
            'regime': 'standard_power_off', 'on_ground': False, 'landing_flare': True, 'vz_ms': -0.5,
        }
        disp.update(t=35.0, sticks={'pitch': 0.25, 'throttle': 1.0}, switches={}, context=ctx_balk)
        self.assertIsInstance(disp.active, ic.GoAroundBehaviour)

        # After 1.6s, climb rate is +1.8 m/s and altitude is 7.5m
        ctx_climb = dict(ctx_balk)
        ctx_climb['h_agl_m'] = 7.5
        ctx_climb['vz_ms'] = 1.8
        ctx_climb['landing_flare'] = False
        ctx_climb['landing_approach'] = False
        status = disp.active.tick(ctx_climb, self.ctrl)
        self.assertEqual(status, ic.SUCCESS)


if __name__ == '__main__':
    unittest.main()
