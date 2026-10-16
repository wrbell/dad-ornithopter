#!/usr/bin/env python3
"""
tests/test_universal_launch_suite.py
====================================
Comprehensive verification suite for:
  - Universal "Crouch-and-Spring" pilot launch gesture
  - Active pilot steering (rudder pedals ch4, roll stick ch1, pitch stick ch2) during launch burst
  - Minimal macro duration and transparent unstick handoff to flight control
  - Autonomous Learning Curriculum (ALC) genetic interbreeding of launch modality chromosomes
  - Active pilot steering across all launch macros (LaunchBurstMacro, QuadrupedalVaultMacro, DragonflySuperLiftMacro)
"""

import unittest
import math
import sys
import os

# Add firmware to path
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'firmware')))

import mco_intent_control as ic
import mco_advanced_physics as ap
import mco_wave_library as wl


class MockFlightController:
    def __init__(self):
        self._tgt_freq = 2.0
        self._tgt_sweeps = [60.0] * 8
        self._tgt_dihedrals = [6.0] * 8
        self._tgt_pitches = [12.0] * 8


class TestUniversalLaunchGesture(unittest.TestCase):
    """Verifies the universal pilot intent gesture ('Crouch-and-Spring')."""

    def setUp(self):
        self.interp = ic.IntentInterpreter()

    def test_crouch_and_spring_fires_launch_burst(self):
        """Hold brakes + full throttle + full aft stick (crouch) -> rapid center (spring) -> LAUNCH_BURST."""
        # Step 1: Pilot crouches: pedals depressed (brakes held), full throttle (0.90), full aft stick (0.80)
        intent1, conf1, params1 = self.interp.interpret(
            {'yaw': 0.0, 'roll': 0.0, 'pitch': 0.80, 'throttle': 0.90, 'pedals_depressed': True},
            {},
            {'t': 0.0, 'V_ms': 0.0, 'h_agl_m': 0.0, 'on_ground': True, 'universal_launch': True}
        )
        self.assertEqual(intent1, ic.Intent.TAXI_FWD)
        self.assertTrue(self.interp._vault_armed, "Launch sequence should be armed in crouch phase")

        # Step 2: Pilot rapidly centers stick (0.02) at t = 0.20s (dt = 0.20s <= 0.45s)
        intent2, conf2, params2 = self.interp.interpret(
            {'yaw': 0.0, 'roll': 0.0, 'pitch': 0.02, 'throttle': 0.90, 'pedals_depressed': True},
            {},
            {'t': 0.20, 'V_ms': 0.0, 'h_agl_m': 0.0, 'on_ground': True, 'universal_launch': True}
        )
        self.assertEqual(intent2, ic.Intent.LAUNCH_BURST)
        self.assertGreaterEqual(conf2, 0.95)
        self.assertEqual(params2.get('trigger'), 'rapid_center')
        self.assertIn('chromosome', params2)
        self.assertGreaterEqual(params2.get('vault_impulse_g', 0.0), 1.5)
        self.assertFalse(self.interp._vault_armed, "Armed flag must reset after firing")

    def test_crouch_cancellation_on_pedal_release(self):
        """Releasing pedals during crouch cancels launch arming."""
        # Arm
        self.interp.interpret(
            {'pitch': 0.80, 'throttle': 0.90, 'pedals_depressed': True},
            {},
            {'t': 0.0, 'on_ground': True, 'h_agl_m': 0.0}
        )
        self.assertTrue(self.interp._vault_armed)

        # Release pedals
        self.interp.interpret(
            {'pitch': 0.80, 'throttle': 0.90, 'pedals_depressed': False},
            {},
            {'t': 0.10, 'on_ground': True, 'h_agl_m': 0.0}
        )
        self.assertFalse(self.interp._vault_armed, "Should disarm on pedal release")

        # Centering after disarm does not fire
        intent, _, _ = self.interp.interpret(
            {'pitch': 0.0, 'throttle': 0.90, 'pedals_depressed': False},
            {},
            {'t': 0.20, 'on_ground': True, 'h_agl_m': 0.0}
        )
        self.assertNotEqual(intent, ic.Intent.LAUNCH_BURST)

    def test_crouch_cancellation_on_throttle_backoff(self):
        """Backing off throttle (<0.80) cancels arming."""
        # Arm
        self.interp.interpret(
            {'pitch': 0.80, 'throttle': 0.90, 'pedals_depressed': True},
            {},
            {'t': 0.0, 'on_ground': True, 'h_agl_m': 0.0}
        )
        self.assertTrue(self.interp._vault_armed)

        # Throttle backoff
        self.interp.interpret(
            {'pitch': 0.80, 'throttle': 0.65, 'pedals_depressed': True},
            {},
            {'t': 0.10, 'on_ground': True, 'h_agl_m': 0.0}
        )
        self.assertFalse(self.interp._vault_armed, "Should disarm on throttle backoff")

    def test_crouch_cancellation_on_slow_centering(self):
        """Slow centering (>0.45s) cancels arming without firing."""
        # Arm
        self.interp.interpret(
            {'pitch': 0.80, 'throttle': 0.90, 'pedals_depressed': True},
            {},
            {'t': 0.0, 'on_ground': True, 'h_agl_m': 0.0}
        )
        self.assertTrue(self.interp._vault_armed)

        # Center slowly at dt = 0.50s > 0.45s
        intent, _, _ = self.interp.interpret(
            {'pitch': 0.0, 'throttle': 0.90, 'pedals_depressed': True},
            {},
            {'t': 0.50, 'on_ground': True, 'h_agl_m': 0.0}
        )
        self.assertNotEqual(intent, ic.Intent.LAUNCH_BURST)
        self.assertFalse(self.interp._vault_armed)


class TestActivePilotSteeringKinematics(unittest.TestCase):
    """Verifies that pilot controls actively steer the aircraft during launch burst."""

    def setUp(self):
        self.chrom = ap.PTEROSAUR_VAULT_GENOME

    def test_rudder_pedal_yaw_differential_steering(self):
        """Rudder pedal input produces differential stroke amplitude and sweep across port and stbd."""
        # Full right rudder: ch4 = +1.0
        pilot_ctrl = {'ch1': 0.0, 'ch2': 0.0, 'ch3': 1.0, 'ch4': 1.0}
        commands, freq, diag = ap.launch_burst_kinematics(self.chrom, t_rel_s=0.20, pilot_controls=pilot_ctrl)

        self.assertTrue(diag['pilot_steering_active'])
        self.assertAlmostEqual(diag['diff_amp_deg'], 12.0)
        self.assertAlmostEqual(diag['diff_sweep_deg'], 10.0)

        # Port wings (0, 2, 4, 6) have lower amplitude and lower sweep than stbd (1, 3, 5, 7)
        for port_i, stbd_i in [(0, 1), (2, 3), (4, 5), (6, 7)]:
            self.assertLess(commands[port_i]['flap_amp_deg'], commands[stbd_i]['flap_amp_deg'])
            self.assertLess(commands[port_i]['sweep_deg'], commands[stbd_i]['sweep_deg'])

    def test_roll_stick_lateral_steering(self):
        """Roll stick input produces differential dihedral and differential pitch."""
        # Roll right: ch1 = +0.8
        pilot_ctrl = {'ch1': 0.8, 'ch2': 0.0, 'ch3': 1.0, 'ch4': 0.0}
        commands, freq, diag = ap.launch_burst_kinematics(self.chrom, t_rel_s=0.20, pilot_controls=pilot_ctrl)

        self.assertTrue(diag['pilot_steering_active'])
        # Port dihedral lower than stbd dihedral
        self.assertLess(commands[0]['dihedral_deg'], commands[1]['dihedral_deg'])
        # Port pitch lower than stbd pitch
        self.assertLess(commands[0]['pitch_deg'], commands[1]['pitch_deg'])

    def test_pitch_stick_climb_gradient_modulation(self):
        """Pitch stick aft increases climb pitch AoA; forward decreases it."""
        pilot_neutral = {'ch1': 0.0, 'ch2': 0.0, 'ch3': 1.0, 'ch4': 0.0}
        pilot_pull = {'ch1': 0.0, 'ch2': 0.7, 'ch3': 1.0, 'ch4': 0.0}
        pilot_push = {'ch1': 0.0, 'ch2': -0.4, 'ch3': 1.0, 'ch4': 0.0}

        cmd_neutral, _, _ = ap.launch_burst_kinematics(self.chrom, t_rel_s=0.20, pilot_controls=pilot_neutral)
        cmd_pull, _, _ = ap.launch_burst_kinematics(self.chrom, t_rel_s=0.20, pilot_controls=pilot_pull)
        cmd_push, _, _ = ap.launch_burst_kinematics(self.chrom, t_rel_s=0.20, pilot_controls=pilot_push)

        self.assertGreater(cmd_pull[0]['pitch_deg'], cmd_neutral[0]['pitch_deg'])
        self.assertLess(cmd_push[0]['pitch_deg'], cmd_neutral[0]['pitch_deg'])


class TestMinimalMacroDurationAndHandoff(unittest.TestCase):
    """Verifies that the launch macro is minimal in duration and cleanly hands off upon unstick."""

    def test_macro_breakout_upon_unstick(self):
        """Macro terminates with SUCCESS once positive unstick and climb rate are verified."""
        macro = ic.LaunchBurstMacro()
        chrom = ap.PTEROSAUR_VAULT_GENOME
        context = {
            't': 0.0,
            'chromosome': chrom,
            'on_ground': True,
            'h_agl_m': 0.0,
            'V_ms': 0.0,
            'vz_ms': 0.0,
            'sticks': {'ch1': 0.0, 'ch2': 0.0, 'ch3': 1.0, 'ch4': 0.0},
        }
        macro.on_enter(context)

        # Initial tick at t = 0.10s -> still running
        context['t'] = 0.10
        ctrl = MockFlightController()
        state = macro.tick(context, ctrl)
        self.assertEqual(state, ic.RUNNING)

        # Tick at t = 0.30s with positive climb and unstick (h = 0.9m, vz = 1.4 m/s) -> breakout to SUCCESS!
        context['t'] = 0.30
        context['h_agl_m'] = 0.9
        context['vz_ms'] = 1.4
        context['V_ms'] = 4.5
        state = macro.tick(context, ctrl)
        self.assertEqual(state, ic.SUCCESS, "Should cleanly hand off to flight control upon unstick")

    def test_macro_hard_duration_ceiling(self):
        """Macro terminates with SUCCESS if hard duration clamp is reached."""
        macro = ic.LaunchBurstMacro()
        chrom = ap.PTEROSAUR_VAULT_GENOME
        context = {
            't': 0.0,
            'chromosome': chrom,
            'on_ground': True,
            'h_agl_m': 0.2,
            'V_ms': 3.0,
            'vz_ms': 0.5,
            'sticks': {'ch1': 0.0, 'ch2': 0.0, 'ch3': 1.0, 'ch4': 0.0},
        }
        macro.on_enter(context)
        ctrl = MockFlightController()

        # Exceed macro duration max
        context['t'] = chrom.macro_duration_max_s + 0.05
        state = macro.tick(context, ctrl)
        self.assertEqual(state, ic.SUCCESS)

    def test_macro_abort_on_dive_stick(self):
        """Macro immediately aborts with FAILURE if pilot pushes forward into dive (ch2 < -0.60)."""
        macro = ic.LaunchBurstMacro()
        chrom = ap.PTEROSAUR_VAULT_GENOME
        context = {
            't': 0.0,
            'chromosome': chrom,
            'on_ground': True,
            'h_agl_m': 0.0,
            'V_ms': 0.0,
            'vz_ms': 0.0,
            'sticks': {'ch1': 0.0, 'ch2': 0.0, 'ch3': 1.0, 'ch4': 0.0},
        }
        macro.on_enter(context)
        ctrl = MockFlightController()

        # Forward dive push
        context['t'] = 0.15
        context['pitch'] = -0.70
        state = macro.tick(context, ctrl)
        self.assertEqual(state, ic.FAILURE)


class TestALCLaunchInterbreeding(unittest.TestCase):
    """Verifies ALC genetic interbreeding of launch modality traits."""

    def test_interbreed_chromosomes(self):
        """Crossbreeding Pterosaur and Dragonfly creates a valid hybrid with blended traits."""
        hybrid = ap.interbreed_launch_chromosomes(
            ap.PTEROSAUR_VAULT_GENOME, ap.DRAGONFLY_SUPERLIFT_GENOME, rng_seed=42
        )
        self.assertIsInstance(hybrid, ap.LaunchChromosome)
        self.assertTrue(hybrid.name.startswith("hybrid_"))
        # Blended frequency between 2.0 and 3.5
        self.assertGreaterEqual(hybrid.freq_hz, 2.0)
        self.assertLessEqual(hybrid.freq_hz, 3.5)
        # Blended phase collapse between 0° and 75°
        self.assertGreaterEqual(hybrid.phase_collapse_deg, 0.0)
        self.assertLessEqual(hybrid.phase_collapse_deg, 75.0)

    def test_evaluate_launch_fitness(self):
        """Evaluates multi-objective fitness under standard and obstacle conditions."""
        fit, metrics = ap.evaluate_launch_fitness(
            ap.DRAGONFLY_SUPERLIFT_GENOME,
            {'gross_weight_kg': 5.687, 'obstacle_dist_m': 12.0, 'obstacle_height_m': 2.5}
        )
        self.assertGreater(metrics['peak_g'], 1.8)
        self.assertLess(metrics['t_unstick_s'], 0.40)
        self.assertIn('fitness', metrics)

    def test_select_optimal_launch_chromosome(self):
        """ALC tournament selection returns optimal chromosome for given environment."""
        # High density altitude scenario
        chrom, metrics = ap.select_optimal_launch_chromosome({'density_rho': 0.95})
        self.assertIsInstance(chrom, ap.LaunchChromosome)
        self.assertGreater(metrics['fitness'], 50.0)


class TestActiveSteeringInLegacyMacros(unittest.TestCase):
    """Verifies that active pilot steering works inside QuadrupedalVaultMacro and DragonflySuperLiftMacro."""

    def test_quadrupedal_vault_macro_active_steering(self):
        """QuadrupedalVaultMacro steers with rudders (ch4) and roll stick (ch1)."""
        macro = ic.QuadrupedalVaultMacro()
        ctrl = MockFlightController()
        context = {
            't': 0.20,
            'on_ground': True,
            'h_agl_m': 0.2,
            'V_ms': 2.0,
            'pedals_depressed': True,
            'throttle': 0.90,
            'yaw': 0.70,   # Right rudder
            'roll': -0.50, # Left roll
        }
        macro.on_enter(context)
        context['t'] = 0.30
        macro.tick(context, ctrl)

        # Stbd sweep greater than port sweep due to right rudder
        self.assertGreater(ctrl._tgt_sweeps[1], ctrl._tgt_sweeps[0])
        # Port dihedral greater than stbd dihedral due to left roll
        self.assertGreater(ctrl._tgt_dihedrals[0], ctrl._tgt_dihedrals[1])

    def test_dragonfly_super_lift_macro_active_steering(self):
        """DragonflySuperLiftMacro steers with rudders (ch4) and roll stick (ch1)."""
        macro = ic.DragonflySuperLiftMacro()
        ctrl = MockFlightController()
        context = {
            't': 0.0,
            'h_agl_m': 0.1,
            'V_ms': 0.5,
            'climb_rate_ms': 0.2,
            'yaw': -0.80, # Left rudder
            'roll': 0.60, # Right roll
        }
        macro.on_enter(context)
        context['t'] = 0.10
        macro.tick(context, ctrl)

        # Port sweep greater than stbd sweep due to left rudder
        self.assertGreater(ctrl._tgt_sweeps[0], ctrl._tgt_sweeps[1])
        # Stbd dihedral greater than port dihedral due to right roll
        self.assertGreater(ctrl._tgt_dihedrals[1], ctrl._tgt_dihedrals[0])


if __name__ == '__main__':
    unittest.main()
