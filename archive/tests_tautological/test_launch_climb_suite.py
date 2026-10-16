"""
Unit Tests: Dragonfly & Pigeon Biological Launch & High-Angle Ascent Suite
===========================================================================
Verifies:
  1. H-LA-1: Dragonfly 0° In-Phase Synchronized Downstroke Super-Lift (Sun & Wang 2007).
  2. H-LA-3: Pigeon Dorsal Clap-and-Peel Vortex Ring Steep Climb Corridor (Provini et al. 2012).
  3. IntentInterpreter: Autonomous arbitration for VTOL unstick and steep obstacle climb.
  4. Macro & Behaviour Lifecycles: Lifecycle states, parameter targeting, and auto-breakout.
  5. Safety Overrides: Pilot manual stick override and macro abort boundaries.
"""

import math
import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'firmware'))

import mco_advanced_physics as ap
import mco_intent_control as ic


class TestDragonflySuperLiftHLA1(unittest.TestCase):
    """Test H-LA-1: Dragonfly 0° In-Phase Synchronized Downstroke Super-Lift."""

    def setUp(self):
        self.base_commands = [
            {'pitch_deg': 12.0, 'flap_amp_deg': 90.0, 'sweep_deg': 90.0} for _ in range(8)
        ]

    def test_dragonfly_super_lift_trigger(self):
        """At low airspeed and ground proximity, generates >2.4x MTOW (>135 N) vertical lift spike."""
        V_ms = 0.5
        h_agl = 0.2

        boosted_cmds, active, total_lift_N, diag = ap.h_la1_dragonfly_inphase_super_lift(
            V_ms=V_ms, h_agl_m=h_agl, base_commands=self.base_commands, armed=True, cycle_count=1
        )

        self.assertTrue(active, "Dragonfly super-lift should fire within launch envelope")
        self.assertEqual(diag['status'], 'DRAGONFLY_INPHASE_SUPER_LIFT_FIRED')
        self.assertTrue(diag['constructive_lev_interference'])
        self.assertEqual(diag['phase_offsets_deg'], [0.0, 0.0, 0.0, 0.0])
        self.assertEqual(diag['frequency_hz'], 2.50)
        self.assertEqual(diag['stroke_amp_deg'], 112.0)
        self.assertAlmostEqual(diag['lift_multiplier'], 2.42)
        self.assertGreaterEqual(total_lift_N, 135.0, "Should generate >= 135 N on 55.8 N MTOW")

        # All 8 wings commanded to 112° amplitude and 18° peak LEV AoA
        for cmd in boosted_cmds:
            self.assertEqual(cmd['flap_amp_deg'], 112.0)
            self.assertEqual(cmd['pitch_deg'], 18.0)
            self.assertEqual(cmd['sweep_deg'], 90.0)

    def test_disarmed_when_airspeed_exceeds_launch_bound(self):
        """When airspeed exceeds 4.0 m/s, super-lift does not fire."""
        V_ms = 6.5
        h_agl = 0.2

        cmds, active, total_lift_N, diag = ap.h_la1_dragonfly_inphase_super_lift(
            V_ms=V_ms, h_agl_m=h_agl, base_commands=self.base_commands, armed=True, cycle_count=1
        )

        self.assertFalse(active)
        self.assertEqual(diag['status'], 'ARMED_AWAITING_LAUNCH_WINDOW')
        self.assertEqual(total_lift_N, ap.H_LA1_NOMINAL_MTOW_N)

    def test_disarmed_when_cycle_count_exceeded(self):
        """Clamps burst to maximum 2 stroke cycles to prevent motor thermal overload."""
        V_ms = 0.5
        h_agl = 0.2

        _, active, _, diag = ap.h_la1_dragonfly_inphase_super_lift(
            V_ms=V_ms, h_agl_m=h_agl, base_commands=self.base_commands, armed=True, cycle_count=3
        )
        self.assertFalse(active, "Burst should disarm after 2 cycles")

    def test_disarmed_when_not_armed(self):
        """When safety disarmed, launch burst must not trigger."""
        _, active, _, diag = ap.h_la1_dragonfly_inphase_super_lift(
            V_ms=0.0, h_agl_m=0.0, base_commands=self.base_commands, armed=False
        )
        self.assertFalse(active)
        self.assertEqual(diag['status'], 'DISARMED')


class TestPigeonSteepClimbHLA3(unittest.TestCase):
    """Test H-LA-3: Pigeon Dorsal Clap-and-Peel Vortex Ring Steep Climb Corridor."""

    def setUp(self):
        self.base_commands = [
            {'pitch_deg': 12.0, 'flap_amp_deg': 90.0, 'sweep_deg': 60.0} for _ in range(8)
        ]

    def test_pigeon_steep_climb_trigger(self):
        """At low airspeed with high climb angle demand (>= 70°), vectors force at 75°-85°."""
        climb_angle = 80.0
        V_ms = 4.5

        cmds, active, achieved_deg, diag = ap.h_la3_pigeon_clap_and_peel(
            climb_angle_deg=climb_angle, V_ms=V_ms, base_commands=self.base_commands, peel_phase=0.5
        )

        self.assertTrue(active)
        self.assertEqual(diag['status'], 'PIGEON_CLAP_PEEL_CLIMB_ACTIVE')
        self.assertAlmostEqual(achieved_deg, 80.0)
        self.assertEqual(diag['peel_lead_ms'], 18.0)
        self.assertTrue(diag['starting_vortex_ring_ejected'])
        self.assertGreaterEqual(diag['vertical_force_vector_pct'], 95.0)

        # Verify dynamic sweep and spanwise pitch lead
        # Root pairs (0,1 and 6,7) have 18.0° pitch; mid pairs (2,3 and 4,5) have 15.0° pitch
        self.assertEqual(cmds[0]['pitch_deg'], 18.0)
        self.assertEqual(cmds[1]['pitch_deg'], 18.0)
        self.assertEqual(cmds[2]['pitch_deg'], 15.0)
        self.assertEqual(cmds[3]['pitch_deg'], 15.0)
        self.assertEqual(cmds[4]['pitch_deg'], 15.0)
        self.assertEqual(cmds[5]['pitch_deg'], 15.0)
        self.assertEqual(cmds[6]['pitch_deg'], 18.0)
        self.assertEqual(cmds[7]['pitch_deg'], 18.0)

    def test_dynamic_peel_sweep_progression(self):
        """Sweep transitions from 42° at dorsal clap (phase 0) to 65° at downstroke (phase 1)."""
        cmds_clap, _, _, diag_clap = ap.h_la3_pigeon_clap_and_peel(
            climb_angle_deg=80.0, V_ms=3.0, base_commands=self.base_commands, peel_phase=0.0
        )
        self.assertAlmostEqual(cmds_clap[0]['sweep_deg'], 42.0)

        cmds_down, _, _, diag_down = ap.h_la3_pigeon_clap_and_peel(
            climb_angle_deg=80.0, V_ms=3.0, base_commands=self.base_commands, peel_phase=1.0
        )
        self.assertAlmostEqual(cmds_down[0]['sweep_deg'], 65.0)

    def test_inactive_outside_steep_envelope(self):
        """Inactive when airspeed is too high (> 7.0 m/s) or climb angle is shallow (< 70°)."""
        _, active_speed, _, diag_speed = ap.h_la3_pigeon_clap_and_peel(
            climb_angle_deg=80.0, V_ms=12.0, base_commands=self.base_commands
        )
        self.assertFalse(active_speed)
        self.assertEqual(diag_speed['status'], 'OUTSIDE_STEEP_CLIMB_ENVELOPE')

        _, active_angle, _, diag_angle = ap.h_la3_pigeon_clap_and_peel(
            climb_angle_deg=45.0, V_ms=4.0, base_commands=self.base_commands
        )
        self.assertFalse(active_angle)


class TestLaunchClimbIntentArbitration(unittest.TestCase):
    """Test IntentInterpreter arbitration for Dragonfly launch and Pigeon climb."""

    def setUp(self):
        self.interp = ic.IntentInterpreter()

    def test_dragonfly_launch_detection_on_takeoff(self):
        """Full throttle on ground with deliberate launch trigger engages DRAGONFLY_SUPER_LIFT."""
        sticks = {'ch1': 0.0, 'ch2': 0.0, 'ch3': 0.90, 'ch4': 0.0}
        switches = {'sw_arm': 1, 'sw_mode': 0, 'sw_launch': 1}
        context = {'on_ground': True, 'h_agl_m': 0.1, 'V_ms': 0.5, 'dragonfly_launch_demand': True}

        intent, conf, params = self.interp.interpret(sticks, switches, context)
        self.assertEqual(intent, ic.Intent.DRAGONFLY_SUPER_LIFT)
        self.assertGreaterEqual(conf, 0.95)
        self.assertAlmostEqual(params['lift_boost_multiplier'], 2.42)
        self.assertEqual(params['freq_hz'], 2.50)

    def test_pigeon_steep_climb_detection_airborne(self):
        """Airborne low airspeed with steep climb demand triggers PIGEON_STEEP_CLIMB."""
        sticks = {'ch1': 0.0, 'ch2': 0.0, 'ch3': 0.85, 'ch4': 0.0}
        switches = {'sw_arm': 1, 'sw_mode': 0}
        context = {
            'on_ground': False,
            'h_agl_m': 5.0,
            'V_ms': 4.2,
            'steep_climb_demand': True,
            'climb_angle_deg': 78.0,
        }

        intent, conf, params = self.interp.interpret(sticks, switches, context)
        self.assertEqual(intent, ic.Intent.PIGEON_STEEP_CLIMB)
        self.assertGreaterEqual(conf, 0.90)
        self.assertEqual(params['climb_angle_deg'], 80.0)

    def test_safety_preemption_over_launch_modes(self):
        """Safety triggers (e.g. emergency recover) immediately preempt launch/climb intents."""
        sticks = {'ch1': 0.0, 'ch2': 0.0, 'ch3': 0.90, 'ch4': 0.0}
        switches = {'sw_arm': 1, 'sw_mode': 0}
        # Emergency land switch
        context = {
            'on_ground': False,
            'h_agl_m': 10.0,
            'V_ms': 3.0,
            'emergency_land': True,
        }

        intent, _, _ = self.interp.interpret(sticks, switches, context)
        self.assertEqual(intent, ic.Intent.EMERGENCY_LAND)


class MockFlightController:
    """Mock controller for testing behaviour node tick actions."""
    def __init__(self):
        self._tgt_freq = 2.0
        self._tgt_sweeps = [90.0] * 8
        self._tgt_dihedrals = [0.0] * 8
        self._tgt_pitches = [12.0] * 8


class TestLaunchClimbMacroLifecycle(unittest.TestCase):
    """Test lifecycle, parameter targets, and auto-breakout for macros."""

    def test_dragonfly_super_lift_macro_execution(self):
        """Macro applies 2.5 Hz and 18° pitch, and completes upon climb breakout."""
        macro = ic.DragonflySuperLiftMacro()
        ctrl = MockFlightController()

        # Enter at t=10.0s
        macro.on_enter({'t': 10.0})

        # Tick 1: t=10.1s, low climb rate -> RUNNING
        ctx1 = {'t': 10.1, 'climb_rate_ms': 0.5, 'h_agl_m': 0.4}
        status1 = macro.tick(ctx1, ctrl)
        self.assertEqual(status1, ic.RUNNING)
        self.assertEqual(ctrl._tgt_freq, 2.50)
        self.assertEqual(ctrl._tgt_pitches, [18.0] * 8)
        self.assertEqual(ctrl._tgt_sweeps, [90.0] * 8)

        # Tick 2: t=10.25s, climb rate breaks out to 1.8 m/s -> SUCCESS
        ctx2 = {'t': 10.25, 'climb_rate_ms': 1.8, 'h_agl_m': 1.3}
        status2 = macro.tick(ctx2, ctrl)
        self.assertEqual(status2, ic.SUCCESS, "Should declare SUCCESS on positive climb breakout")

    def test_dragonfly_macro_hard_duration_clamp(self):
        """Macro terminates at 0.45s even if climb rate does not exceed threshold."""
        macro = ic.DragonflySuperLiftMacro()
        ctrl = MockFlightController()

        macro.on_enter({'t': 0.0})
        ctx = {'t': 0.46, 'climb_rate_ms': 0.2, 'h_agl_m': 0.3}
        status = macro.tick(ctx, ctrl)
        self.assertEqual(status, ic.SUCCESS, "Hard duration clamp must fire at 0.45s")

    def test_dragonfly_macro_abort_triggers(self):
        """Check abort trigger on overspeed or altitude ceiling."""
        abort, reason = ic.check_macro_abort('dragonfly_super_lift', {'V_ms': 10.5, 'h_agl_m': 0.5})
        self.assertTrue(abort)
        self.assertIn('airspeed too high', reason)

        abort_h, reason_h = ic.check_macro_abort('dragonfly_super_lift', {'V_ms': 2.0, 'h_agl_m': 3.0})
        self.assertTrue(abort_h)
        self.assertIn('altitude breakout reached', reason_h)

    def test_pigeon_steep_climb_behaviour_targets(self):
        """Pigeon climb sets 2.3 Hz, root pitch lead (+3° on pairs 0 & 3), and dynamic sweep."""
        beh = ic.PigeonSteepClimbBehaviour()
        ctrl = MockFlightController()

        beh.on_enter({'t': 0.0})
        ctx = {'t': 0.5, 'peel_phase': 0.5, 'climb_angle_deg': 80.0}
        status = beh.tick(ctx, ctrl)

        self.assertEqual(status, ic.RUNNING)
        self.assertEqual(ctrl._tgt_freq, 2.30)
        # Root pairs (0, 1 and 6, 7) have 18°, mid have 15°
        self.assertEqual(ctrl._tgt_pitches, [18.0, 18.0, 15.0, 15.0, 15.0, 15.0, 18.0, 18.0])
        # Sweep at phase 0.5: 42 + 0.5*(65-42) = 53.5
        self.assertAlmostEqual(ctrl._tgt_sweeps[0], 53.5)


if __name__ == '__main__':
    unittest.main()
