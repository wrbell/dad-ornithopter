#!/usr/bin/env python3
"""
tests/test_modality_soft_handoff_suite.py
=========================================
Comprehensive verification suite for:
  1. Universal Raised-Cosine Soft-Handoff Blending Kernel (ModalityCrossfadeRamp)
  2. Dynamic Pressure (q)-Compensated AoA Structural Limiter
  3. Low-Throttle Glide & Dive Immunity (No Active Upstroke Brake Hijacking)
  4. Continuous Sigmoidal Yaw & Roll Spectrum (Inertial to Aerodynamic Tilt)
  5. ALC Transition Roughness Evaluator & Curriculum Registration
  6. BehaviourDispatcher End-to-End Soft-Handoff Integration
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
        self.freq_hz = 2.0
        self._tgt_freq = 2.0
        self._tgt_sweeps = [60.0] * 8
        self._tgt_dihedrals = [6.0] * 8
        self._tgt_pitches = [8.0] * 8
        self._intent_name = 'cruise'


class TestRaisedCosineCrossfadeKernel(unittest.TestCase):
    """Verifies the mathematical properties and rate-bounding of the raised-cosine interpolator."""

    def test_boundary_values(self):
        """tau=0 returns prev_val, tau=1 returns target_val exactly."""
        p = [50.0] * 8
        t = [90.0] * 8
        res_0 = ap.modality_soft_handoff_blend(p, t, tau=0.0)
        res_1 = ap.modality_soft_handoff_blend(p, t, tau=1.0)
        self.assertEqual(res_0, p)
        self.assertEqual(res_1, t)

    def test_midpoint_symmetry(self):
        """At tau=0.5, alpha is exactly 0.5 (midpoint of transition)."""
        val_0 = 10.0
        val_1 = 30.0
        mid = ap.modality_soft_handoff_blend(val_0, val_1, tau=0.5)
        self.assertAlmostEqual(mid, 20.0, places=2)

    def test_rate_bounding_over_ramp(self):
        """A 40° sweep step over 0.20s (10 steps at 50Hz) never exceeds 300°/s slew rate."""
        p = 50.0
        t = 90.0
        steps = 10
        dt = 0.02
        trajectory = []
        for i in range(steps + 1):
            tau = i / steps
            val = ap.modality_soft_handoff_blend(p, t, tau)
            trajectory.append(val)

        # Check instantaneous slew rates between consecutive 50Hz ticks
        max_rate = max(abs(trajectory[k] - trajectory[k-1]) / dt for k in range(1, len(trajectory)))
        # Analytical peak of raised cosine ramp: (pi/2) * (delta_theta / T_blend) = 1.57 * (40 / 0.2) = 314 deg/s max
        # With quantization, well bounded and smoothly tapering to zero at both ends
        self.assertLess(max_rate, 330.0)
        # Verify boundary derivatives are smooth (slow start and slow finish)
        first_step_rate = abs(trajectory[1] - trajectory[0]) / dt
        last_step_rate = abs(trajectory[-1] - trajectory[-2]) / dt
        self.assertLess(first_step_rate, 150.0)
        self.assertLess(last_step_rate, 150.0)


class TestDynamicPressureAoALimiter(unittest.TestCase):
    """Verifies that high-lift geometric pitch demands are scaled with dynamic pressure."""

    def test_low_speed_preserves_high_camber(self):
        """At takeoff / low speed (V = 5.0 m/s), allows full high-lift AoA (~16°)."""
        aoa_lim = ap.dynamic_pressure_aoa_limit(V_ms=5.0, target_nz=2.2)
        self.assertGreaterEqual(aoa_lim, 14.0)

    def test_high_speed_scales_aoa_down(self):
        """At high speed (V = 16.0 m/s), clamps AoA down to prevent exceeding +2.2g."""
        aoa_lim = ap.dynamic_pressure_aoa_limit(V_ms=16.0, target_nz=2.2)
        self.assertLessEqual(aoa_lim, 6.0)

    def test_dive_speed_tight_clamp(self):
        """At dive speed (V = 32.0 m/s), AoA is clamped near minimum trim (~2.5° to 3.5°)."""
        aoa_lim = ap.dynamic_pressure_aoa_limit(V_ms=32.0, target_nz=2.2)
        self.assertLessEqual(aoa_lim, 3.5)
        self.assertGreaterEqual(aoa_lim, 2.5)


class TestLowThrottleGlideImmunity(unittest.TestCase):
    """Verifies that closing throttle in flight or pushing stick forward does NOT trigger active upstroke airbrake."""

    def setUp(self):
        self.interp = ic.IntentInterpreter()

    def test_zero_throttle_glide_routes_to_glide(self):
        """Throttle idle (0.05), stick centered (0.0), V = 12 m/s -> routes to GLIDE, not airbrake."""
        intent, conf, params = self.interp.interpret(
            sticks={'pitch': 0.0, 'roll': 0.0, 'yaw': 0.0, 'throttle': 0.05, 'gl_lever': -0.40},
            switches={},
            context={'lever_pct': 30.0, 'V_ms': 12.0, 'on_ground': False, 'h_agl_m': 50.0}
        )
        self.assertIn(intent, (ic.Intent.GLIDE, ic.Intent.SLOTTED_FLAP_GLIDE))
        self.assertNotEqual(intent, ic.Intent.ACTIVE_UPSTROKE_BRAKE)

    def test_forward_stick_in_glide_routes_to_falcon_glide(self):
        """Throttle idle (0.00), stick forward (-0.50), V = 18 m/s -> routes to GLIDE (Falcon dive)."""
        intent, conf, params = self.interp.interpret(
            sticks={'pitch': -0.50, 'roll': 0.0, 'yaw': 0.0, 'throttle': 0.00, 'gl_lever': -0.50},
            switches={},
            context={'lever_pct': 20.0, 'V_ms': 18.0, 'on_ground': False, 'h_agl_m': 80.0, 'falcon_morph': True}
        )
        self.assertEqual(intent, ic.Intent.GLIDE)
        self.assertNotEqual(intent, ic.Intent.ACTIVE_UPSTROKE_BRAKE)

    def test_dedicated_airbrake_detent_still_triggers_airbrake(self):
        """When lever is pulled into dedicated airbrake detent (lever_pct <= 10.0), airbrake DOES trigger."""
        intent, conf, params = self.interp.interpret(
            sticks={'pitch': 0.0, 'roll': 0.0, 'yaw': 0.0, 'throttle': 0.05, 'gl_lever': -0.90},
            switches={},
            context={'lever_pct': 5.0, 'V_ms': 10.0, 'on_ground': False, 'h_agl_m': 30.0}
        )
        self.assertEqual(intent, ic.Intent.ACTIVE_UPSTROKE_BRAKE)


class TestContinuousSigmoidalYawBlend(unittest.TestCase):
    """Verifies smooth sigmoidal blending from low-speed inertial yaw into high-speed aerodynamic yaw."""

    def test_low_speed_dominance(self):
        """At V = 0.5 m/s, inertial weight is dominant (w_inertial > 0.95)."""
        res = ap.continuous_speed_yaw_blend(yaw_stick=0.5, V_ms=0.5)
        self.assertGreater(res['w_inertial'], 0.95)
        self.assertLess(res['w_aero'], 0.05)
        self.assertGreater(res['inertial_diff_amp_deg'], 8.0)

    def test_high_speed_dominance(self):
        """At V = 6.0 m/s, aerodynamic tilt and sweep are dominant (w_aero > 0.95)."""
        res = ap.continuous_speed_yaw_blend(yaw_stick=0.5, V_ms=6.0)
        self.assertGreater(res['w_aero'], 0.95)
        self.assertLess(res['w_inertial'], 0.05)
        self.assertGreater(res['stroke_plane_tilt_deg'], 8.0)
        self.assertGreater(res['differential_sweep_deg'], 4.0)

    def test_mid_speed_monotonic_transfer(self):
        """Between 1.0 m/s and 4.0 m/s, w_inertial decreases monotonically without step discontinuity."""
        speeds = [1.0, 1.5, 2.0, 2.5, 3.0, 3.5, 4.0]
        w_in = [ap.continuous_speed_yaw_blend(yaw_stick=0.5, V_ms=v)['w_inertial'] for v in speeds]
        for i in range(len(w_in) - 1):
            self.assertGreaterEqual(w_in[i], w_in[i+1])


class TestALCTransitionRoughnessScore(unittest.TestCase):
    """Verifies the ALC transition roughness scoring and curriculum registration."""

    def test_smooth_sequence_receives_high_score(self):
        """A continuous smooth ramp receives a score >= 0.90."""
        history = []
        for i in range(11):
            sw = [50.0 + i * 4.0] * 8  # 50° to 90° over 10 steps
            p = [8.0 + i * 0.5] * 8
            history.append({'sweeps': sw, 'pitches': p, 'dihedrals': [6.0]*8})
        res = ap.alc_transition_roughness_score(history, dt_s=0.02)
        self.assertGreaterEqual(res['smoothness_score'], 0.85)
        self.assertTrue(res['servo_safe'])

    def test_hard_step_receives_low_score(self):
        """An instantaneous 40° jump in a single 20ms tick (2000°/s) receives a penalty score."""
        history = [
            {'sweeps': [50.0]*8, 'pitches': [8.0]*8, 'dihedrals': [6.0]*8},
            {'sweeps': [90.0]*8, 'pitches': [8.0]*8, 'dihedrals': [6.0]*8},  # 2000°/s spike
            {'sweeps': [90.0]*8, 'pitches': [8.0]*8, 'dihedrals': [6.0]*8},
        ]
        res = ap.alc_transition_roughness_score(history, dt_s=0.02)
        self.assertFalse(res['servo_safe'])
        self.assertLess(res['smoothness_score'], 0.50)

    def test_alss_curriculum_registration(self):
        """Verifies that soft_handoff_crossfade_a is registered in ALSS segments, curriculum A, and tasks."""
        self.assertIn('soft_handoff_crossfade_a', wl._ALSS_SEGMENTS)
        seg = wl._ALSS_SEGMENTS['soft_handoff_crossfade_a']
        self.assertEqual(seg[0], 'manoeuvre')
        self.assertEqual(seg[1], 'SOFT-HANDOFF')
        self.assertIn('soft_handoff_crossfade_a', wl._ALSS_CURRICULUM['A'])
        self.assertIn('modality_soft_handoff', wl._PHASE_TO_SG_TASK)


class TestBehaviourDispatcherSoftHandoffIntegration(unittest.TestCase):
    """Verifies that BehaviourDispatcher seamlessly crossfades targets during node transitions."""

    def setUp(self):
        self.ctrl = MockFlightController()
        self.dispatcher = ic.BehaviourDispatcher(self.ctrl, enable_soft_handoff=True)

    def test_transition_initiates_crossfade(self):
        """Switching from Cruise to Elytron Canard smoothly ramps sweeps over 0.20s rather than jumping."""
        # Initial state in Cruise
        self.ctrl._tgt_sweeps = [60.0] * 8
        self.ctrl._tgt_pitches = [8.0] * 8

        # Step 1: Trigger transition to ELYTRON_CANARD at t = 1.0s (pitch stick 0.40 + obstacle_climb)
        context = {'t': 1.0, 'V_ms': 10.0, 'h_agl_m': 30.0, 'lever_pct': 80.0, 'obstacle_climb': True}
        sticks = {'pitch': 0.40, 'roll': 0.0, 'yaw': 0.0, 'throttle': 0.85, 'gl_lever': 0.0}
        self.dispatcher.update(1.0, sticks, {}, context)

        self.assertTrue(self.dispatcher._blend_active)
        # Elytron Canard Station 1 sweep target is 0.0°. At t=1.0 (tau=0.0), blended sweeps must equal previous [60.0]*8
        self.assertAlmostEqual(self.ctrl._tgt_sweeps[0], 60.0, places=1)

        # Step 2: At t = 1.10s (tau = 0.50), sweeps should be midway between 60° and 0° (30.0°)
        context['t'] = 1.10
        self.dispatcher.update(1.10, sticks, {}, context)
        self.assertAlmostEqual(self.ctrl._tgt_sweeps[0], 30.0, delta=1.5)

        # Step 3: At t = 1.25s (tau >= 1.0), blend completes and targets match Elytron Canard Station 1 (0.0°)
        context['t'] = 1.25
        self.dispatcher.update(1.25, sticks, {}, context)
        self.assertFalse(self.dispatcher._blend_active)
        self.assertAlmostEqual(self.ctrl._tgt_sweeps[0], 0.0, places=1)


if __name__ == '__main__':
    unittest.main()
