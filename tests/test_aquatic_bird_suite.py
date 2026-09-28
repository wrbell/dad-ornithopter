"""
Unit Tests: Aquatic Bird Flight Biomechanics Suite (Top 4)
===========================================================
Verifies:
  1. H-WD-1: Gull Clap-and-Fling × Water Surface Synergy Burst (Hashimoto et al. 2024 JRM).
  2. H-WD-2: Mallard Decoupled Front-Thrust / Rear-Lift Coordination (Taylor-Burt & Biewener 2020 JEB).
  3. H-WD-3: Seagull Asymmetric High-Q Water Departure Kinematics (Laridae water departure).
  4. WIG-Pelican: Wave-Following Wing-in-Ground (WIG) Compression Gliding (Pelecanus ocean swell skimming).
  5. IntentInterpreter: Autonomous arbitration across all 4 aquatic bird flight regimes.
  6. Behaviour Nodes: Execution, target kinematics dispatch, and macro lifecycle completion.
"""

import math
import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'firmware'))

import mco_advanced_physics as ap
import mco_intent_control as ic


class TestGullClapSynergyHWD1(unittest.TestCase):
    """Test H-WD-1: Gull Clap-and-Fling × Water Surface Synergy Burst."""

    def setUp(self):
        self.base_commands = [
            {'pitch_deg': 12.0, 'flap_amp_deg': 96.0, 'sweep_deg': 90.0} for _ in range(8)
        ]

    def test_water_unstick_clap_synergy_trigger(self):
        """At rotation speed in water surface effect zone, triggers +22% transient lift burst."""
        h_water = 0.30  # 30 cm above water (h/b = 0.15 < 0.25)
        V_ms = 9.2      # At rotation/unstick airspeed

        boosted_cmds, fired, boost_N, diag = ap.h_wd1_water_clap_synergy(
            h_water_m=h_water, V_ms=V_ms, base_commands=self.base_commands, armed=True
        )

        self.assertTrue(fired, "Clap synergy burst should fire within unstick window")
        self.assertEqual(diag['status'], 'CLAP_SURFACE_SYNERGY_FIRED')
        self.assertTrue(diag['mirror_vortex_trapped'])
        self.assertAlmostEqual(diag['lift_boost_pct'], 22.0)
        self.assertGreaterEqual(boost_N, 12.0, "Should generate >12 N lift boost on 55.8 N MTOW")

        # All 8 wings commanded to high dorsal clap amplitude
        for cmd in boosted_cmds:
            self.assertEqual(cmd['flap_amp_deg'], 108.0)
            self.assertEqual(cmd['pitch_deg'], 15.0)

    def test_disarmed_outside_surface_effect(self):
        """Above 0.54m (clear of WIG surface effect), clap synergy does not fire."""
        h_water = 0.85  # Outside surface effect layer
        V_ms = 9.5

        _, fired, boost_N, diag = ap.h_wd1_water_clap_synergy(
            h_water_m=h_water, V_ms=V_ms, base_commands=self.base_commands, armed=True
        )

        self.assertFalse(fired)
        self.assertEqual(boost_N, 0.0)
        self.assertFalse(diag['in_wig'])


class TestMallardDecoupledHumpHWD2(unittest.TestCase):
    """Test H-WD-2: Mallard Decoupled Front-Thrust / Rear-Lift Coordination."""

    def setUp(self):
        self.base_commands = [
            {'pitch_deg': 12.0, 'flap_amp_deg': 96.0, 'sweep_deg': 90.0} for _ in range(8)
        ]

    def test_hump_zone_thrust_lift_separation(self):
        """In hump resistance zone (3.5 - 8.0 m/s), decouples front thrust vs rear lift."""
        V_ms = 5.2  # Hump speed peak

        cmds, active, split_pct, diag = ap.h_wd2_mallard_thrust_lift_decoupling(
            V_ms=V_ms, base_commands=self.base_commands, on_water=True
        )

        self.assertTrue(active)
        self.assertEqual(diag['status'], 'MALLARD_DECOUPLED_HUMP_PENETRATION')
        self.assertEqual(diag['front_work'], 'FORWARD_THRUST_FEET')
        self.assertEqual(diag['rear_work'], 'VERTICAL_LIFT_WINGS')

        # Front pairs W1-W2 (indices 0..3): forward sweep, low AoA (thrust)
        for i in range(4):
            self.assertEqual(cmds[i]['sweep_deg'], 55.0)
            self.assertEqual(cmds[i]['pitch_deg'], 10.0)
            self.assertEqual(cmds[i]['flap_amp_deg'], 102.0)

        # Rear pairs W3-W4 (indices 4..7): neutral sweep, high AoA (lift)
        for i in range(4, 8):
            self.assertEqual(cmds[i]['sweep_deg'], 90.0)
            self.assertEqual(cmds[i]['pitch_deg'], 16.0)
            self.assertEqual(cmds[i]['flap_amp_deg'], 96.0)

    def test_inactive_outside_hump(self):
        """At high planing speed (V = 10.5 m/s), decoupled hump mode is inactive."""
        cmds, active, _, _ = ap.h_wd2_mallard_thrust_lift_decoupling(
            V_ms=10.5, base_commands=self.base_commands, on_water=True
        )
        self.assertFalse(active)


class TestGullAsymmetricStrokeHWD3(unittest.TestCase):
    """Test H-WD-3: Seagull Asymmetric High-Q Water Departure Kinematics."""

    def test_asymmetric_stroke_velocity_scaling(self):
        """Downstroke is rapid (1.31x), upstroke is slow and feathered (0.69x)."""
        # Downstroke: phase = pi / 2
        vel_down, is_dn, red_pct = ap.h_wd3_gull_asymmetric_stroke(
            master_phase_rad=math.pi / 2.0, Q_ratio=1.90
        )
        self.assertTrue(is_dn)
        self.assertGreater(vel_down, 1.25)
        self.assertAlmostEqual(red_pct, 36.5)

        # Upstroke: phase = 3*pi / 2
        vel_up, is_dn_up, _ = ap.h_wd3_gull_asymmetric_stroke(
            master_phase_rad=3.0 * math.pi / 2.0, Q_ratio=1.90
        )
        self.assertFalse(is_dn_up)
        self.assertLess(vel_up, 0.75)

        # Ratio of downstroke velocity to upstroke velocity equals Q
        self.assertAlmostEqual(vel_down / vel_up, 1.90, places=4)


class TestPelicanWigCruise(unittest.TestCase):
    """Test Pelican Wave-Following Wing-in-Ground (WIG) Compression Gliding."""

    def setUp(self):
        self.base_commands = [
            {'pitch_deg': 6.0, 'flap_amp_deg': 45.0, 'sweep_deg': 50.0} for _ in range(8)
        ]

    def test_wig_compression_drag_reduction(self):
        """At 20cm above water, induced drag drops by >85%."""
        h_lidar = 0.20  # 20 cm AGL (h/b = 0.10)
        cmds, active, drag_red_pct, diag = ap.pelican_wig_wave_following(
            h_lidar_m=h_lidar, wave_slope_deg=0.0, V_ms=9.0, base_commands=self.base_commands
        )

        self.assertTrue(active)
        self.assertEqual(diag['status'], 'PELICAN_WAVE_FOLLOWING_WIG_ACTIVE')
        self.assertGreater(drag_red_pct, 85.0, "Induced drag should drop by >85% at h/b = 0.10")

    def test_wave_slope_updraft_power_shedding(self):
        """Surfing a 4° wave slope harvests wave updrafts and sheds active flapping amplitude."""
        cmds, active, _, diag = ap.pelican_wig_wave_following(
            h_lidar_m=0.25, wave_slope_deg=4.5, V_ms=9.0, base_commands=self.base_commands
        )

        self.assertTrue(active)
        self.assertGreater(diag['updraft_harvest_ms'], 0.5)
        self.assertGreater(diag['power_shed_pct'], 10.0)
        # Flap amplitude derated due to updraft support
        for cmd in cmds:
            self.assertLess(cmd['flap_amp_deg'], 45.0)


class TestAquaticBirdIntentArbitration(unittest.TestCase):
    """Test Intent arbitration and behaviour dispatch for all 4 aquatic bird regimes."""

    def setUp(self):
        self.interp = ic.IntentInterpreter()

    def test_intent_water_clap_burst(self):
        """Unstick conditions map to Intent.WATER_CLAP_BURST."""
        intent, conf, params = self.interp.interpret(
            {'pitch': 0.0, 'throttle': 0.85, 'roll': 0.0, 'yaw': 0.0},
            {},
            {'on_water': True, 'h_water_m': 0.28, 'V_ms': 9.2}
        )
        self.assertEqual(intent, ic.Intent.WATER_CLAP_BURST)
        self.assertEqual(params.get('lift_boost_pct'), 22.0)

    def test_intent_mallard_hump_breakout(self):
        """Hump speed on water maps to Intent.MALLARD_HUMP_BREAKOUT."""
        intent, conf, params = self.interp.interpret(
            {'pitch': 0.0, 'throttle': 0.80, 'roll': 0.0, 'yaw': 0.0},
            {},
            {'on_water': True, 'h_water_m': 0.10, 'V_ms': 5.5}
        )
        self.assertEqual(intent, ic.Intent.MALLARD_HUMP_BREAKOUT)
        self.assertEqual(params.get('front_sweep_deg'), 55.0)

    def test_intent_gull_asymmetric_stroke(self):
        """Low speed water taxi maps to Intent.GULL_ASYMMETRIC_STROKE."""
        intent, conf, params = self.interp.interpret(
            {'pitch': 0.0, 'throttle': 0.40, 'roll': 0.0, 'yaw': 0.0},
            {},
            {'on_water': True, 'h_water_m': 0.05, 'V_ms': 2.0}
        )
        self.assertEqual(intent, ic.Intent.GULL_ASYMMETRIC_STROKE)
        self.assertEqual(params.get('Q_ratio'), 1.90)

    def test_intent_pelican_wig_cruise(self):
        """Low AGL flight over water maps to Intent.WIG_CRUISE."""
        intent, conf, params = self.interp.interpret(
            {'pitch': 0.0, 'throttle': 0.50, 'roll': 0.0, 'yaw': 0.0},
            {},
            {'on_ground': False, 'over_water': True, 'h_water_m': 0.25, 'V_ms': 8.5}
        )
        self.assertEqual(intent, ic.Intent.WIG_CRUISE)
        self.assertEqual(params.get('induced_drag_reduction_pct'), 86.0)

    def test_water_clap_macro_execution_and_completion(self):
        """Verify WaterClapBurstMacro execution and exit condition."""
        macro = ic.WaterClapBurstMacro()
        macro.on_enter({'t': 1.0})

        class MockCtrl:
            _tgt_freq = 0.0
            _tgt_pitches = []
            _tgt_sweeps = []
            _tgt_dihedrals = []

        ctrl = MockCtrl()
        # Mid-burst at t = 1.15s (duration 0.15s < 0.40s)
        status = macro.tick({'t': 1.15, 'h_agl_m': 0.35}, ctrl)
        self.assertEqual(status, ic.RUNNING)
        self.assertEqual(ctrl._tgt_freq, 2.4)
        self.assertEqual(ctrl._tgt_pitches[0], 15.0)

        # Completed at t = 1.45s (duration 0.45s >= 0.40s)
        status_done = macro.tick({'t': 1.45, 'h_agl_m': 0.45}, ctrl)
        self.assertEqual(status_done, ic.SUCCESS)


if __name__ == '__main__':
    unittest.main()
