"""
test_conventional_landing_suite.py
===================================
Validation Maneuver T23 / Open Item O-CONV-LANDING.

Comprehensive test suite verifying the Conventional Fixed-Wing Approach, Flare,
and Runway Rollout Braking Suite with full ALC Integration:
  1. 6 Landing Regimes (Power-Off Idle, Commercial Powered, Short-Field, Soft-Field, Tailwheel 3-Point, Tailwheel Wheel)
  2. Dual-Cascade Flare Kinematics & Anti-Tail-Strike Ground Clearance (h_w4 >= 0.15m)
  3. Ground-Effect Induced Drag Collapse (IGE sigma)
  4. Crosswind Slip Decrab Alignment
  5. Aerodynamic Rollout Braking on Wings (Aircraft has NO wheel brakes):
     - Shuttlecock Bluff-Body Airbrake (Cd ~ 1.65, -0.65g)
     - Active Reverse Thrust Flapping (-25N, -0.85g)
     - Compound Max Stop (-45N + bluff drag, > -1.25g)
     - Freewheel Low-Drag Coasting (Cd ~ 0.04)
  6. Anti-Nose-Over Longitudinal Relief on Front Wings W1-W2
  7. Touch-and-Go / Go-Around Re-acceleration (throttle >= 50% or lever >= 78%)
  8. Auto-Taxi Handoff (V < 4.5 m/s, lever >= 40%)
  9. ALC Touchdown Quality Scoring (grease landing, sink impact, G-shock, tail clearance)
  10. IntentInterpreter transitions: APPROACH_NORMAL -> LANDING_FLARE -> TOUCHDOWN_ROLLOUT -> TAXI_FWD / TAKEOFF_ROLL
  11. BehaviourDispatcher and ALSS Curriculum Class A registration
"""

import math
import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'firmware'))

import mco_advanced_physics as ap
import mco_intent_control as ic
import mco_wave_library as wl


class MockControlState:
    """Mock ControlState conforming to BehaviourDispatcher expectations."""
    def __init__(self):
        self.freq_hz = 0.0
        self._tgt_freq = 0.0
        self._tgt_pitches = [0.0] * 8
        self._tgt_sweeps = [0.0] * 8
        self._tgt_dihedrals = [0.0] * 8
        self._intent_name = 'idle'
        self.flare_diag = {}
        self.rollout_diag = {}


class TestConventionalLandingSuite(unittest.TestCase):
    """Validation test suite for Validation Maneuver T23."""

    def test_detect_landing_phase(self):
        """Verify automatic phase detection across approach, flare, and rollout."""
        # 1. High approach
        p1 = ap.detect_landing_phase(h_agl_m=12.0, V_ms=8.0, vz_ms=-0.8, throttle=0.30, on_ground=False)
        self.assertEqual(p1, 'approach')

        # 2. Flare window (h <= 2.0m, sinking)
        p2 = ap.detect_landing_phase(h_agl_m=1.2, V_ms=6.2, vz_ms=-0.6, throttle=0.10, on_ground=False)
        self.assertEqual(p2, 'flare')

        # 3. Touchdown rollout (on ground)
        p3 = ap.detect_landing_phase(h_agl_m=0.05, V_ms=5.5, vz_ms=0.0, throttle=0.0, on_ground=True)
        self.assertEqual(p3, 'rollout')

    def test_landing_regimes_defined(self):
        """Verify all 6 landing regimes are defined with correct physical parameters."""
        expected_regimes = [
            'standard_power_off',
            'commercial_powered',
            'short_field',
            'soft_field',
            'tailwheel_3point',
            'tailwheel_wheel_landing',
        ]
        for reg in expected_regimes:
            self.assertIn(reg, ap.LANDING_REGIMES, f"Missing landing regime: {reg}")
            cfg = ap.LANDING_REGIMES[reg]
            self.assertIn('target_touchdown_pitch_deg', cfg)
            self.assertIn('max_sink_rate_contact_ms', cfg)
            self.assertIn('gear_type', cfg)

    def test_flare_dual_cascade_kinematics_and_tail_clearance(self):
        """Verify Front Cascade (W1-W2) provides canard lift while W4 clearance >= 0.15m."""
        # Test standard power-off flare with aft pitch stick
        pitch_stick = 0.35
        throttle = 0.05
        h_agl = 0.80
        V_ms = 6.0
        cmds, freq_hz, diag = ap.conventional_landing_flare_kinematics(
            pitch_in=pitch_stick,
            throttle=throttle,
            h_agl_m=h_agl,
            V_ms=V_ms,
            regime='standard_power_off'
        )

        self.assertEqual(len(cmds), 8)
        # Power-off flare has idle flapping freq
        self.assertEqual(freq_hz, 0.0)

        # Verify canard lift on front wings W1-W2 (higher AoA than rear)
        front_pitches = [cmds[i]['pitch_deg'] for i in range(4)]
        rear_pitches = [cmds[i]['pitch_deg'] for i in range(4, 8)]
        front_mean = sum(front_pitches) / len(front_pitches)
        rear_mean = sum(rear_pitches) / len(rear_pitches)
        self.assertGreater(front_mean, rear_mean)
        self.assertGreaterEqual(front_mean, 10.0)

        # Verify anti-tail-strike clearance >= 150mm
        tail_clearance = diag['tail_clearance_w4_m']
        self.assertGreaterEqual(tail_clearance, 0.15, f"Tail clearance too low: {tail_clearance:.3f}m")
        self.assertTrue(diag['tail_strike_safe'])

        # Verify IGE drag reduction factor sigma < 1.0 near ground
        self.assertLess(diag['sigma_ige'], 1.0)
        self.assertGreater(diag['sigma_ige'], 0.4)

    def test_flare_commercial_powered_approach(self):
        """Verify commercial powered flare carries active throttle & flatter attitude."""
        cmds, freq_hz, diag = ap.conventional_landing_flare_kinematics(
            pitch_in=0.15,
            throttle=0.35,
            h_agl_m=1.0,
            V_ms=7.5,
            regime='commercial_powered'
        )
        # Powered approach retains active flapping thrust
        self.assertGreater(freq_hz, 1.0)
        self.assertAlmostEqual(freq_hz, 1.80, places=1)
        self.assertAlmostEqual(diag['touchdown_pitch_deg'], 4.5, places=1)
        self.assertTrue(diag['tail_strike_safe'])

    def test_flare_tailwheel_profiles(self):
        """Verify tailwheel 3-point full stall vs tailwheel wheel landing."""
        # 1. 3-point full stall landing: high pitch attitude (15.5 deg) at touchdown
        cmds_3pt, freq_3pt, diag_3pt = ap.conventional_landing_flare_kinematics(
            pitch_in=0.85,
            throttle=0.0,
            h_agl_m=0.30,
            V_ms=4.8,
            regime='tailwheel_3point'
        )
        self.assertGreaterEqual(diag_3pt['touchdown_pitch_deg'], 14.0)
        self.assertGreaterEqual(diag_3pt['tail_clearance_w4_m'], 0.15)

        # 2. Wheel landing: level pitch attitude (3.5 deg), tail held up
        cmds_wl, freq_wl, diag_wl = ap.conventional_landing_flare_kinematics(
            pitch_in=0.05,
            throttle=0.15,
            h_agl_m=0.30,
            V_ms=6.8,
            regime='tailwheel_wheel_landing'
        )
        self.assertLessEqual(diag_wl['touchdown_pitch_deg'], 5.0)
        self.assertGreater(diag_wl['tail_clearance_w4_m'], 0.20)

    def test_flare_crosswind_slip_decrab(self):
        """Verify crosswind decrab applies differential dihedral to counter drift."""
        # 5 m/s crosswind
        crosswind_ms = 5.0
        cmds, freq_hz, diag = ap.conventional_landing_flare_kinematics(
            pitch_in=0.20,
            throttle=0.10,
            h_agl_m=1.0,
            V_ms=7.0,
            regime='standard_power_off',
            crosswind_ms=crosswind_ms
        )
        self.assertGreater(abs(diag['decrab_residual_beta_deg']), 0.0)
        # Check port vs stbd dihedral differential
        port_dih = cmds[0]['dihedral_deg']
        stbd_dih = cmds[1]['dihedral_deg']
        self.assertNotEqual(port_dih, stbd_dih)

    def test_rollout_braking_modalities_defined(self):
        """Verify all 6 rollout modalities are mapped to the wave function lever."""
        expected_modalities = [
            'TOUCH_AND_GO',
            'FREEWHEEL_COAST',
            'AUTO_TAXI',
            'SHUTTLECOCK_AIRBRAKE',
            'ACTIVE_REVERSE_FLAP',
            'COMPOUND_MAX_STOP',
        ]
        for m in expected_modalities:
            self.assertIn(m, ap.ROLLOUT_MODALITIES)

    def test_rollout_braking_shuttlecock_airbrake(self):
        """Verify passive shuttlecock bluff-body airbrake cup (lever 16-28%)."""
        lever_pct = 22.0  # In shuttlecock zone
        cmds, freq_hz, diag = ap.aerodynamic_rollout_braking_kinematics(
            lever_pct=lever_pct,
            throttle=0.0,
            V_ms=10.0,
            regime='standard_power_off'
        )
        self.assertEqual(diag['modality'], 'SHUTTLECOCK_AIRBRAKE')
        self.assertEqual(freq_hz, 0.0)  # Passive aerodynamic cup
        # High drag coefficient
        self.assertGreaterEqual(diag['retarding_force_n'], 30.0)
        self.assertGreater(diag['decel_g'], 0.45)

        # Outboard wings (4..7) have steep +70° dihedral cup
        outboard_dihs = [cmds[i]['dihedral_deg'] for i in range(4, 8)]
        for dih in outboard_dihs:
            self.assertGreaterEqual(dih, 60.0)

        front_pitches = [cmds[i]['pitch_deg'] for i in range(4)]
        rear_pitches = [cmds[i]['pitch_deg'] for i in range(4, 8)]
        front_mean = sum(front_pitches) / len(front_pitches)
        rear_mean = sum(rear_pitches) / len(rear_pitches)
        self.assertGreater(front_mean, rear_mean)

    def test_rollout_braking_active_reverse_flap(self):
        """Verify active reverse thrust flapping on ground (lever 6-16%)."""
        lever_pct = 12.0  # In reverse flap zone
        cmds, freq_hz, diag = ap.aerodynamic_rollout_braking_kinematics(
            lever_pct=lever_pct,
            throttle=0.0,
            V_ms=6.0,
            regime='standard_power_off'
        )
        self.assertEqual(diag['modality'], 'ACTIVE_REVERSE_FLAP')
        self.assertAlmostEqual(freq_hz, 2.20, places=1)
        self.assertGreaterEqual(diag['retarding_force_n'], 24.0)

        # Negative AoA (-26°) for reverse thrust flapping (with canard relief on front)
        rear_pitches = [cmds[i]['pitch_deg'] for i in range(4, 8)]
        for p in rear_pitches:
            self.assertLessEqual(p, -20.0)

    def test_rollout_braking_compound_max_stop(self):
        """Verify compound stop (<6% lever) produces maximum retarding force."""
        lever_pct = 2.0  # Compound max stop zone
        cmds, freq_hz, diag = ap.aerodynamic_rollout_braking_kinematics(
            lever_pct=lever_pct,
            throttle=0.0,
            V_ms=10.0,
            regime='short_field'
        )
        self.assertEqual(diag['modality'], 'COMPOUND_MAX_STOP')
        self.assertGreater(freq_hz, 2.0)
        # Combined reverse thrust + bluff drag produces > 1.0g deceleration
        self.assertGreaterEqual(diag['decel_g'], 1.0)
        self.assertGreaterEqual(diag['retarding_force_n'], 60.0)

    def test_rollout_freewheel_coast(self):
        """Verify freewheel coasting (48-78% lever) produces minimal drag."""
        lever_pct = 55.0
        cmds, freq_hz, diag = ap.aerodynamic_rollout_braking_kinematics(
            lever_pct=lever_pct,
            throttle=0.0,
            V_ms=8.0,
            regime='standard_power_off'
        )
        self.assertEqual(diag['modality'], 'FREEWHEEL_COAST')
        self.assertEqual(freq_hz, 0.0)
        self.assertLessEqual(diag['decel_g'], 0.10)
        self.assertLessEqual(diag['retarding_force_n'], 5.0)

    def test_rollout_touch_and_go_preemption(self):
        """Verify pushing throttle or lever forward triggers touch-and-go mode."""
        # 1. Throttle >= 0.50
        cmds1, freq1, diag1 = ap.aerodynamic_rollout_braking_kinematics(
            lever_pct=50.0,
            throttle=0.65,
            V_ms=6.0,
            regime='standard_power_off'
        )
        self.assertEqual(diag1['modality'], 'TOUCH_AND_GO')
        self.assertTrue(diag1['touch_and_go_ready'])

        # 2. Lever >= 78%
        cmds2, freq2, diag2 = ap.aerodynamic_rollout_braking_kinematics(
            lever_pct=85.0,
            throttle=0.0,
            V_ms=6.0,
            regime='standard_power_off'
        )
        self.assertEqual(diag2['modality'], 'TOUCH_AND_GO')
        self.assertTrue(diag2['touch_and_go_ready'])

    def test_rollout_auto_taxi_handoff(self):
        """Verify automatic transition to taxi as rollout speed drops below 4.5 m/s."""
        lever_pct = 42.0  # Within taxi detent
        cmds, freq, diag = ap.aerodynamic_rollout_braking_kinematics(
            lever_pct=lever_pct,
            throttle=0.15,
            V_ms=3.2,
            regime='standard_power_off'
        )
        self.assertEqual(diag['modality'], 'AUTO_TAXI')
        self.assertTrue(diag['taxi_ready'])

    def test_alc_touchdown_quality_scoring(self):
        """Verify ALC touchdown quality score rewards grease landing and penalizes impacts."""
        # 1. Perfect grease touchdown
        telemetry_grease = {
            'vz_impact_ms': -0.22,
            'peak_nz_g': 1.10,
            'min_tail_clearance_m': 0.25,
            'stopping_distance_m': 14.2,
        }
        res_grease = ap.alc_touchdown_quality_score(telemetry_grease)
        self.assertAlmostEqual(res_grease['touchdown_score'], 1.0, places=2)
        self.assertTrue(res_grease['grease_touchdown'])
        self.assertTrue(res_grease['safe_tail_clearance'])

        # 2. Hard landing with tail proximity
        telemetry_hard = {
            'vz_impact_ms': -1.15,
            'peak_nz_g': 2.30,
            'min_tail_clearance_m': 0.08,
            'stopping_distance_m': 28.5,
        }
        res_hard = ap.alc_touchdown_quality_score(telemetry_hard)
        self.assertLess(res_hard['touchdown_score'], 0.40)
        self.assertFalse(res_hard['grease_touchdown'])
        self.assertFalse(res_hard['safe_tail_clearance'])

    def test_intent_interpreter_landing_suite(self):
        """Verify IntentInterpreter routes approach, flare, rollout, touch-and-go, and auto-taxi."""
        interp = ic.IntentInterpreter()

        # 1. Approach on final descent (h = 15m, low throttle)
        sticks_app = {'roll': 0.0, 'pitch': 0.0, 'throttle': 0.30, 'yaw': 0.0}
        switches = {}
        ctx_app = {'V_ms': 8.0, 'h_agl_m': 15.0, 'on_ground': False, 'landing_approach': True}
        intent, conf, params = interp.interpret(sticks_app, switches, ctx_app)
        self.assertEqual(intent, ic.Intent.APPROACH_NORMAL)

        # 2. Flare near ground (h = 1.2m, positive aft stick)
        interp._last_intent = ic.Intent.APPROACH_NORMAL
        sticks_flr = {'roll': 0.0, 'pitch': 0.25, 'throttle': 0.05, 'yaw': 0.0}
        ctx_flr = {'V_ms': 6.2, 'h_agl_m': 1.2, 'on_ground': False}
        intent, conf, params = interp.interpret(sticks_flr, switches, ctx_flr)
        self.assertEqual(intent, ic.Intent.LANDING_FLARE)

        # 3. Touchdown rollout on ground without wheel brakes (h = 0.05m, on_ground=True, V = 7.0 m/s)
        interp._last_intent = ic.Intent.LANDING_FLARE
        sticks_roll = {'roll': 0.0, 'pitch': 0.0, 'throttle': 0.0, 'yaw': 0.0}
        ctx_roll = {'V_ms': 7.0, 'h_agl_m': 0.05, 'on_ground': True, 'lever_pct': 20.0}
        intent, conf, params = interp.interpret(sticks_roll, switches, ctx_roll)
        self.assertEqual(intent, ic.Intent.TOUCHDOWN_ROLLOUT)
        self.assertEqual(params['lever_pct'], 20.0)

        # 4. Touch-and-Go re-acceleration during rollout (throttle pushed forward to 0.70)
        interp._last_intent = ic.Intent.TOUCHDOWN_ROLLOUT
        sticks_tng = {'roll': 0.0, 'pitch': 0.0, 'throttle': 0.70, 'yaw': 0.0}
        ctx_tng = {'V_ms': 6.0, 'h_agl_m': 0.05, 'on_ground': True, 'lever_pct': 70.0}
        intent, conf, params = interp.interpret(sticks_tng, switches, ctx_tng)
        self.assertEqual(intent, ic.Intent.TAKEOFF_ROLL)

        # 5. Low-speed auto-taxi handoff (V drops to 3.5 m/s, lever >= 40%)
        interp._last_intent = ic.Intent.TOUCHDOWN_ROLLOUT
        sticks_taxi = {'roll': 0.0, 'pitch': 0.0, 'throttle': 0.15, 'yaw': 0.0}
        ctx_taxi = {'V_ms': 3.5, 'h_agl_m': 0.05, 'on_ground': True, 'lever_pct': 50.0}
        intent, conf, params = interp.interpret(sticks_taxi, switches, ctx_taxi)
        self.assertEqual(intent, ic.Intent.TAXI_FWD)

    def test_behaviour_dispatcher_landing_execution(self):
        """Verify ApproachBehaviour, FlareBehaviour, and LandingRolloutBehaviour execute in dispatcher."""
        ctrl = MockControlState()
        disp = ic.BehaviourDispatcher(ctrl, enable_soft_handoff=False)

        # 1. Approach
        ctx_app = {
            'V_ms': 8.0,
            'h_agl_m': 12.0,
            'on_ground': False,
            'landing_approach': True,
            'throttle': 0.35,
            'pitch': 0.0,
            'yaw': 0.0,
        }
        disp.update(t=1.0, sticks={'pitch': 0.0, 'throttle': 0.35}, switches={}, context=ctx_app)
        self.assertIsInstance(disp.active, ic.ApproachBehaviour)
        self.assertGreater(ctrl._tgt_freq, 0.5)

        # 2. Flare
        ctx_flr = {
            'V_ms': 6.0,
            'h_agl_m': 1.2,
            'on_ground': False,
            'pitch': 0.25,
            'throttle': 0.05,
            'regime': 'standard_power_off',
        }
        disp.update(t=1.5, sticks={'pitch': 0.25, 'throttle': 0.05}, switches={}, context=ctx_flr)
        self.assertIsInstance(disp.active, ic.FlareBehaviour)
        self.assertGreaterEqual(ctrl.flare_diag.get('tail_clearance_w4_m', 0.0), 0.15)

        # 3. Rollout braking
        ctx_roll = {
            'V_ms': 8.0,
            'h_agl_m': 0.05,
            'on_ground': True,
            'lever_pct': 22.0,  # Shuttlecock airbrake
            'throttle': 0.0,
            'regime': 'standard_power_off',
        }
        disp.update(t=2.0, sticks={'pitch': 0.0, 'throttle': 0.0}, switches={}, context=ctx_roll)
        self.assertIsInstance(disp.active, ic.LandingRolloutBehaviour)
        self.assertEqual(ctrl.rollout_diag.get('modality'), 'SHUTTLECOCK_AIRBRAKE')
        self.assertGreater(ctrl.rollout_diag.get('retarding_force_n', 0.0), 20.0)

    def test_alss_curriculum_registration(self):
        """Verify ALSS curriculum Class A includes all conventional landing segments."""
        expected_segments = [
            'landing_approach_a',
            'landing_flare_touchdown_a',
            'landing_rollout_braking_a',
        ]
        for seg in expected_segments:
            self.assertIn(seg, wl._ALSS_SEGMENTS, f"Missing segment in _ALSS_SEGMENTS: {seg}")
            self.assertIn(seg, wl._ALSS_CURRICULUM['A'], f"Missing segment in _ALSS_CURRICULUM['A']: {seg}")

        # Check phase tasks
        self.assertIn('conventional_landing', wl._PHASE_TO_SG_TASK)
        tasks = wl._PHASE_TO_SG_TASK['conventional_landing']
        self.assertIn('touchdown_sink_rate_ms', tasks)
        self.assertIn('tail_clearance_margin_m', tasks)
        self.assertIn('shuttlecock_decel_g', tasks)
        self.assertIn('reverse_thrust_n', tasks)


if __name__ == '__main__':
    unittest.main()
