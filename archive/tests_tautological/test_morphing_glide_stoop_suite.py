#!/usr/bin/env python3
"""
tests/test_morphing_glide_stoop_suite.py
========================================
Comprehensive verification suite for the Falcon Morphing Polar (Glide-Dive-Stoop):
  1. Four continuous flight zones (Float -> Penetration -> Dive -> Peregrine Stoop)
  2. Continuous, reversible pitch-stick modulation (forward to stoop, aft to zoom)
  3. Active pilot steering (rudder pedals ch4 differential sweep, roll stick ch1 differential dihedral)
  4. Active envelope protection (dynamic +2.5g limiter and 25m AGL terrain proximity floor)
  5. Autonomous Learning Curriculum (ALC) integration (MacCready speed-to-fly bias and kinetic zoom energy tracking)
  6. IntentInterpreter and GlideBehaviour full integration
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
        self._tgt_freq = 0.0
        self._tgt_sweeps = [60.0] * 8
        self._tgt_dihedrals = [6.0] * 8
        self._tgt_pitches = [6.0] * 8
        self.falcon_morph_diag = {}


class TestFalconMorphingPolarZones(unittest.TestCase):
    """Verifies the aerodynamic geometries and metrics across all 4 morphing zones."""

    def test_zone_1_best_ld_float(self):
        """Zone 1: Best L/D Float (stick >= -0.05). High L/D (~15), low sweep, positive dihedral."""
        cmds, freq, diag = ap.falcon_morphing_glide_polar(pitch_stick=0.0, V_ms=11.0, h_agl_m=150.0)
        self.assertEqual(diag['zone'], "ZONE_1_BEST_LD_FLOAT")
        self.assertAlmostEqual(freq, 0.0, places=2)
        self.assertGreaterEqual(diag['est_ld'], 14.0)
        self.assertLessEqual(diag['base_sweep_deg'], 60.5)
        self.assertAlmostEqual(diag['base_dihedral_deg'], 6.0, places=1)
        self.assertGreaterEqual(diag['base_aoa_deg'], 5.5)
        # All 8 wings should have identical baseline float geometry
        for c in cmds:
            self.assertAlmostEqual(c['sweep_deg'], diag['base_sweep_deg'], places=1)
            self.assertAlmostEqual(c['dihedral_deg'], 6.0, places=1)

    def test_zone_2_fast_penetration(self):
        """Zone 2: Fast Penetration (-0.05 > stick >= -0.40). Sweep 60°->74°, dihedral +6°->+1°."""
        cmds, freq, diag = ap.falcon_morphing_glide_polar(pitch_stick=-0.25, V_ms=15.0, h_agl_m=150.0)
        self.assertEqual(diag['zone'], "ZONE_2_FAST_PENETRATION")
        self.assertGreater(diag['base_sweep_deg'], 62.0)
        self.assertLess(diag['base_sweep_deg'], 74.0)
        self.assertLess(diag['base_dihedral_deg'], 6.0)
        self.assertGreater(diag['base_dihedral_deg'], 0.5)
        self.assertLess(diag['est_ld'], 14.0)
        self.assertGreater(diag['est_ld'], 8.0)
        self.assertGreater(diag['target_v_ms'], 12.0)

    def test_zone_3_steep_dive(self):
        """Zone 3: Steep Aerodynamic Dive (-0.40 > stick >= -0.75). Gull tuck dihedral (-8°)."""
        cmds, freq, diag = ap.falcon_morphing_glide_polar(pitch_stick=-0.60, V_ms=24.0, h_agl_m=150.0)
        self.assertEqual(diag['zone'], "ZONE_3_STEEP_DIVE")
        self.assertGreaterEqual(diag['base_sweep_deg'], 74.0)
        self.assertLessEqual(diag['base_sweep_deg'], 85.0)
        self.assertLess(diag['base_dihedral_deg'], 0.0)  # Gull tuck anhedral
        self.assertGreater(diag['base_dihedral_deg'], -8.0)
        # Station 4 (W4) sweeps further back for tailplane authority
        self.assertGreater(diag['w4_sweep_deg'], diag['base_sweep_deg'])
        self.assertLess(diag['est_ld'], 8.0)

    def test_zone_4_full_peregrine_stoop(self):
        """Zone 4: Full Peregrine Stoop (-0.75 > stick >= -1.0). Flush delta fold W1-W3 (90°), W4 tail (135°)."""
        cmds, freq, diag = ap.falcon_morphing_glide_polar(pitch_stick=-1.0, V_ms=36.0, h_agl_m=200.0)
        self.assertEqual(diag['zone'], "ZONE_4_PEREGRINE_STOOP")
        self.assertAlmostEqual(diag['base_sweep_deg'], 90.0, places=1)
        self.assertAlmostEqual(diag['w4_sweep_deg'], 135.0, places=1)
        self.assertAlmostEqual(diag['base_dihedral_deg'], -14.0, places=1)
        self.assertLessEqual(diag['est_ld'], 2.5)
        self.assertGreaterEqual(diag['target_v_ms'], 38.0)
        # W4 (index 6, 7) should have negative pitch trim for longitudinal stability
        w1_pitch = cmds[0]['pitch_deg']
        w4_pitch = cmds[6]['pitch_deg']
        self.assertLess(w4_pitch, w1_pitch)


class TestReversibleMorphingContinuum(unittest.TestCase):
    """Verifies seamless continuous reversibility with no modal lockouts or discontinuities."""

    def test_monotonic_sweep_and_dihedral_progression(self):
        """As stick transitions from float (0.0) down to full stoop (-1.0), sweep increases and dihedral drops monotonically."""
        sticks = [0.0, -0.15, -0.30, -0.45, -0.60, -0.75, -0.90, -1.0]
        sweeps = []
        dihedrals = []
        lds = []

        for st in sticks:
            _, _, diag = ap.falcon_morphing_glide_polar(pitch_stick=st, V_ms=20.0, h_agl_m=100.0)
            sweeps.append(diag['base_sweep_deg'])
            dihedrals.append(diag['base_dihedral_deg'])
            lds.append(diag['est_ld'])

        # Check monotonic progression forward
        for i in range(len(sticks) - 1):
            self.assertLessEqual(sweeps[i], sweeps[i+1], f"Sweep should increase or hold from {sticks[i]} to {sticks[i+1]}")
            self.assertGreaterEqual(dihedrals[i], dihedrals[i+1], f"Dihedral should decrease or hold from {sticks[i]} to {sticks[i+1]}")
            self.assertGreaterEqual(lds[i], lds[i+1], f"L/D should decrease or hold from {sticks[i]} to {sticks[i+1]}")

    def test_reversible_pullout_returns_to_float(self):
        """Moving stick aft (-1.0 -> -0.4 -> 0.0) smoothly recovers high L/D float."""
        # Stoop
        _, _, d_stoop = ap.falcon_morphing_glide_polar(pitch_stick=-0.95, V_ms=35.0, h_agl_m=100.0)
        self.assertEqual(d_stoop['zone'], "ZONE_4_PEREGRINE_STOOP")

        # Ease stick to penetration
        _, _, d_pen = ap.falcon_morphing_glide_polar(pitch_stick=-0.30, V_ms=25.0, h_agl_m=100.0)
        self.assertEqual(d_pen['zone'], "ZONE_2_FAST_PENETRATION")

        # Return to neutral stick float
        _, _, d_float = ap.falcon_morphing_glide_polar(pitch_stick=0.0, V_ms=12.0, h_agl_m=100.0)
        self.assertEqual(d_float['zone'], "ZONE_1_BEST_LD_FLOAT")
        self.assertAlmostEqual(d_float['base_dihedral_deg'], 6.0, places=1)


class TestActivePilotSteering(unittest.TestCase):
    """Verifies that pilot steering (rudder pedals ch4, roll stick ch1) remains fully active in dive and stoop."""

    def test_rudder_pedal_yaw_differential_in_stoop(self):
        """In full stoop, right rudder (ch4 = +0.50) creates differential sweep across port and starboard wings."""
        cmds, _, diag = ap.falcon_morphing_glide_polar(
            pitch_stick=-0.90,
            V_ms=35.0,
            h_agl_m=100.0,
            pilot_controls={'ch4': 0.50, 'ch1': 0.0}
        )
        self.assertTrue(diag['pilot_steering_active'])
        # Port wings (even indices 0, 2, 4, 6) vs Starboard wings (odd indices 1, 3, 5, 7)
        port_w1_sw = cmds[0]['sweep_deg']
        stbd_w1_sw = cmds[1]['sweep_deg']
        self.assertNotEqual(port_w1_sw, stbd_w1_sw)
        # Right yaw command (ch4 > 0): stbd sweep > port sweep
        self.assertGreater(stbd_w1_sw, port_w1_sw)

    def test_roll_stick_differential_dihedral_in_dive(self):
        """In dive, right roll stick (ch1 = +0.60) creates differential dihedral across port and starboard wings."""
        cmds, _, diag = ap.falcon_morphing_glide_polar(
            pitch_stick=-0.55,
            V_ms=25.0,
            h_agl_m=100.0,
            pilot_controls={'ch4': 0.0, 'ch1': 0.60}
        )
        self.assertTrue(diag['pilot_steering_active'])
        port_dih = cmds[0]['dihedral_deg']
        stbd_dih = cmds[1]['dihedral_deg']
        self.assertNotEqual(port_dih, stbd_dih)
        self.assertGreater(stbd_dih, port_dih)


class TestEnvelopeProtectionAndSafety(unittest.TestCase):
    """Verifies active envelope protection: Dynamic G-Limiter and Terrain Proximity Floor."""

    def test_ground_floor_tripwire_overrides_dive(self):
        """When AGL < 25m and pilot is diving, the ground floor automatically overrides to pullout."""
        _, _, diag = ap.falcon_morphing_glide_polar(
            pitch_stick=-0.80,
            V_ms=28.0,
            h_agl_m=18.0  # Under 25m safety tripwire
        )
        self.assertTrue(diag['ground_floor_active'])
        self.assertEqual(diag['zone'], "ZONE_1_BEST_LD_FLOAT")
        self.assertGreater(diag['effective_stick'], 0.0)

    def test_dynamic_g_limiter_caps_pullout_aoa(self):
        """At high airspeed (e.g. 35 m/s) and aft stick (+0.80), G-limiter clamps AoA to prevent exceeding +2.5g."""
        cmds, _, diag = ap.falcon_morphing_glide_polar(
            pitch_stick=0.80,
            V_ms=35.0,
            h_agl_m=150.0
        )
        self.assertTrue(diag['g_limited'])
        # Normal +0.80 stick would command AoA ~ 7.5°, but at 35 m/s it should be clamped lower
        self.assertLessEqual(diag['base_aoa_deg'], 5.0)


class TestALCIntegration(unittest.TestCase):
    """Verifies Autonomous Learning Curriculum (ALC) integration: MacCready bias & Kinetic Zoom Energy tracking."""

    def test_alc_maccready_sink_speed_bias(self):
        """In strong atmospheric sink (w_netto = -2.5 m/s), MacCready increases target speed and biases stick forward."""
        bias_sink = ap.alc_maccready_glide_bias(w_netto_ms=-2.5)
        self.assertGreater(bias_sink['v_target_ms'], bias_sink['v_best_ld_ms'])
        self.assertLess(bias_sink['pitch_bias'], 0.0)  # Pushes stick forward for faster penetration

    def test_alc_maccready_lift_speed_bias(self):
        """In strong thermal lift (w_netto = +2.0 m/s), MacCready reduces target speed and biases stick aft."""
        bias_lift = ap.alc_maccready_glide_bias(w_netto_ms=2.0)
        self.assertLess(bias_lift['v_target_ms'], bias_lift['v_best_ld_ms'])
        self.assertGreater(bias_lift['pitch_bias'], 0.0)  # Biases stick aft to float/soar

    def test_alc_kinetic_zoom_efficiency_calculation(self):
        """Evaluates potential energy recovered vs kinetic energy lost in zoom pullout."""
        # Entry at 35 m/s, exit at 15 m/s, zoom climb delta_h = 38 m, mass = 5.687 kg
        res = ap.alc_kinetic_zoom_efficiency(v_entry_ms=35.0, v_exit_ms=15.0, delta_h_m=38.0, gross_weight_kg=5.687)
        self.assertGreater(res['eta_zoom'], 0.70)
        self.assertLessEqual(res['eta_zoom'], 1.0)
        self.assertGreater(res['delta_pe_J'], 2000.0)
        self.assertGreater(res['delta_ke_J'], 2500.0)

    def test_alss_curriculum_registration(self):
        """Verifies that falcon_morphing_glide_stoop_a is registered in ALSS segments, curriculum A, and task dictionary."""
        self.assertIn('falcon_morphing_glide_stoop_a', wl._ALSS_SEGMENTS)
        seg = wl._ALSS_SEGMENTS['falcon_morphing_glide_stoop_a']
        self.assertEqual(seg[0], 'descent')
        self.assertEqual(seg[1], 'FALCON-MORPH')
        self.assertIn('falcon_morphing_glide_stoop_a', wl._ALSS_CURRICULUM['A'])
        self.assertIn('falcon_morphing_glide', wl._PHASE_TO_SG_TASK)


class TestIntentInterpreterGlideIntegration(unittest.TestCase):
    """Verifies that IntentInterpreter and GlideBehaviour seamlessly execute falcon morphing polar."""

    def setUp(self):
        self.interp = ic.IntentInterpreter()
        self.ctrl = MockFlightController()

    def test_glide_behaviour_ticks_falcon_morph_on_pitch_stick(self):
        """When in glide mode and pilot pitches stick forward (ch2 = -0.65), GlideBehaviour applies falcon polar."""
        context = {
            'lever_pct': 30.0,
            'V_ms': 22.0,
            'h_agl_m': 100.0,
            'sticks': {'pitch': -0.65, 'roll': 0.0, 'yaw': 0.0},
            'pitch_deg': -12.0,
        }
        b = ic.GlideBehaviour()
        res = b.tick(context, self.ctrl)
        self.assertEqual(res, ic.RUNNING)
        self.assertTrue(hasattr(self.ctrl, 'falcon_morph_diag'))
        self.assertEqual(self.ctrl.falcon_morph_diag['zone'], "ZONE_3_STEEP_DIVE")
        self.assertGreaterEqual(self.ctrl._tgt_sweeps[0], 74.0)

    def test_interpret_routes_to_glide_during_forward_pitch_stick(self):
        """IntentInterpreter.interpret routes to GLIDE (not SLOTTED_FLAP_GLIDE) when ch2 < -0.05."""
        intent, conf, params = self.interp.interpret(
            sticks={'pitch': -0.45, 'roll': 0.0, 'yaw': 0.0, 'throttle': 0.20, 'gl_lever': -0.40},
            switches={},
            context={'lever_pct': 30.0, 'V_ms': 12.0, 'on_ground': False, 'h_agl_m': 100.0}
        )
        self.assertEqual(intent, ic.Intent.GLIDE)
        self.assertTrue(params.get('falcon_morph', False))


if __name__ == '__main__':
    unittest.main()
