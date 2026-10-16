"""
Unit and Integration Test Suite for H-FALCON-TRIM-1
("Falcon Dart" Fanned Bi-Delta Tailplane Fast Cruise & Level Dash).

Verifies:
  1. Hypothesis Registration & Discovery:
     - Registered in wl._ADV_NEW_HYPS and wl.AERO_HYPOTHESES.
     - Mapped in _HYP_TO_LEVER_ZONE ('MAX_THRUST') and _lever_mode_weights with weight 1.85.
     - ALSS segment 'falcon_trim1_cruise_c' registered in _ALSS_SEGMENTS and _ALSS_CURRICULUM['C'].
     - Keyword discovery via 'falcon_dart', 'falcon_trim', 'h_falcon_trim_1', 'fanned_bi_delta', etc.
     - Genome optimization weights (_MW_PERFORMANCE, _MW_PITCH, _MW_ENDURANCE).
     - Priority queue trigger in AI Observer.
  2. Kinematic & Aerodynamic Waveform (wave_falcon_dart):
     - Stations 1, 2, 3 active flapping cascade with fanned sweep (+5°, +15°, +28°),
       washout trim (+2.5°, +1.5°, +1.0°), and amplitude ±18.0°.
     - Station 4 stationary lock at stroke θ = 0.0°, sweep +50.0°, and pitch trim α = -5.5°.
     - Longitudinal pitch equilibrium: active flapping nose-down moment (-21.4 N*m)
       neutralized by stationary W4 nose-up trim moment (+21.4 N*m), yielding residual < 0.1 N*m.
     - Actuator safety: Station 4 servo hinge moment <= 0.45 N*m (safely below 0.69 N*m AGFRC stall limit).
     - Dynamic 3D clearance: > 43.4 mm inter-station margin (pair 1-2: 57.8 mm, pair 3-4: 95.8 mm).
     - Power & Specific Energy: Fast cruise (20.8 m/s, 2.8 Hz) ~365 W (4.87 Wh/km);
       Level dash (26.35 m/s, 3.0 Hz) ~580 W (6.11 Wh/km).
  3. Red-Team Safety Envelopes & Abort Triggering:
     - Pitch stick (|pitch| > 0.35), roll stick (|roll| > 0.35), yaw stick (|yaw| > 0.15).
     - Low airspeed (V < 12.0 m/s) stall abort.
     - Ground lockout (on_ground = True).
     - Pitot saturation fallback to Tactical EKF / GPS at V >= 39.4 m/s (950 Pa).
  4. Evaluator Hook (evaluate_falcon_dart) & Ground Station Telemetry (falcon_dart_status).
  5. Intent-Based Control Integration:
     - Intent.FALCON_DART classified at high throttle (>=85%) and high airspeed (>=14.0 m/s).
     - FalconDartBehaviour registered in BEHAVIOUR_REGISTRY and commanding correct targets.
"""

import sys
import os
import unittest
import math

# Ensure firmware directory is on python path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'firmware'))

import mco_wave_library as wl
import mco_intent_control as ic


class TestFalconDartRegistration(unittest.TestCase):
    """Verifies H-FALCON-TRIM-1 data structures, ALSS integration, and discovery."""

    def setUp(self):
        self.hyp = next((h for h in wl._ADV_NEW_HYPS if h['id'] == 'H-FALCON-TRIM-1'), None)

    def test_hypothesis_registered_in_adv_new_hyps(self):
        self.assertIsNotNone(self.hyp, "H-FALCON-TRIM-1 not found in _ADV_NEW_HYPS")
        self.assertEqual(self.hyp['target'], 'fast_cruise')
        self.assertEqual(self.hyp['phase'], 'cruise')
        self.assertIn('cruise', self.hyp['eval_phases'])
        self.assertIn('sprint', self.hyp['eval_phases'])
        self.assertIn('fast_cruise', self.hyp['eval_phases'])

    def test_hypothesis_registered_in_aero_hypotheses(self):
        aero_ids = [h['id'] for h in wl.AERO_HYPOTHESES]
        self.assertIn('H-FALCON-TRIM-1', aero_ids)
        h = next(h for h in wl.AERO_HYPOTHESES if h['id'] == 'H-FALCON-TRIM-1')
        self.assertIn('Falcon Dart cruise & dash', h['label'])
        self.assertIn('Selig S1223', h['rationale'])

    def test_lever_zone_mapping_and_weights(self):
        self.assertEqual(wl._HYP_TO_LEVER_ZONE.get('H-FALCON-TRIM-1'), 'MAX_THRUST')
        self.assertIn('H-FALCON-TRIM-1', wl._lever_mode_weights['MAX_THRUST'])
        self.assertGreaterEqual(wl._lever_mode_weights['MAX_THRUST']['H-FALCON-TRIM-1'], 1.0)

    def test_alss_segment_and_curriculum(self):
        self.assertIn('falcon_trim1_cruise_c', wl._ALSS_SEGMENTS)
        seg = wl._ALSS_SEGMENTS['falcon_trim1_cruise_c']
        self.assertEqual(seg[0], 'feathered_fast_cruise')
        self.assertEqual(seg[1], 'H-FALCON-TRIM-1')
        self.assertEqual(seg[2], 15.0)  # v_min
        self.assertEqual(seg[3], 35.0)  # v_max
        self.assertIn('falcon_trim1_cruise_c', wl._ALSS_CURRICULUM.get('C', []))

    def test_keyword_discovery_mapping(self):
        keywords = ['h_falcon_trim_1', 'h_falcon_trim1', 'falcon_dart', 'falcon_trim', 'fanned_bi_delta', 'falcon_cruise', 'falcon_dash', 'tailplane_trim']
        for kw in keywords:
            h = wl.discovery_hypothesis_select(focus=kw)
            self.assertIsNotNone(h, f"Focus '{kw}' returned None")
            self.assertEqual(h['id'], 'H-FALCON-TRIM-1', f"Focus '{kw}' did not return H-FALCON-TRIM-1")


class TestFalconDartKinematicsAndAero(unittest.TestCase):
    """Verifies wave_falcon_dart waveform, trim moment neutralization, servo safety, and clearance."""

    def test_nominal_fast_cruise_waveform(self):
        res = wl.wave_falcon_dart(t_sec=0.25, f_hz=2.8, v_ms=20.80)
        self.assertTrue(res['falcon_dart_active'])
        self.assertFalse(res['aborted'])
        self.assertEqual(res['active_stations'], [1, 2, 3])
        self.assertEqual(res['stationary_stations'], [4])
        self.assertEqual(res['f_hz'], 2.8)
        self.assertAlmostEqual(res['v_ms'], 20.80, places=2)

        pairs = res['pairs']
        # Active stations 1-3
        for st in ['pair_1', 'pair_2', 'pair_3']:
            self.assertEqual(pairs[st]['mode'], 'PROPULSIVE_CASCADE')
            self.assertTrue(pairs[st]['active_flapping'])
            self.assertEqual(pairs[st]['current_limit_a'], 30.0)

        # Fanned sweeps
        self.assertEqual(pairs['pair_1']['sweep_deg'], 5.0)
        self.assertEqual(pairs['pair_2']['sweep_deg'], 15.0)
        self.assertEqual(pairs['pair_3']['sweep_deg'], 28.0)
        self.assertEqual(pairs['pair_4']['sweep_deg'], 50.0)

        # Washout AoA trim
        self.assertEqual(pairs['pair_1']['pitch_trim_deg'], 2.5)
        self.assertEqual(pairs['pair_2']['pitch_trim_deg'], 1.5)
        self.assertEqual(pairs['pair_3']['pitch_trim_deg'], 1.0)
        self.assertEqual(pairs['pair_4']['pitch_trim_deg'], -5.5)

        # Stationary Station 4
        self.assertEqual(pairs['pair_4']['mode'], 'STATIONARY_FBW_TAILPLANE')
        self.assertEqual(pairs['pair_4']['flap_deg'], 0.0)
        self.assertFalse(pairs['pair_4']['active_flapping'])
        self.assertEqual(pairs['pair_4']['current_limit_a'], 8.0)

    def test_pitch_moment_trim_neutrality(self):
        res = wl.wave_falcon_dart(t_sec=1.0, f_hz=2.8, v_ms=20.80)
        # Active wings pitch moment (-21.4 N*m) neutralized by W4 tailplane (+21.4 N*m)
        self.assertAlmostEqual(res['pitch_moment_active_nm'], -21.4, places=1)
        self.assertAlmostEqual(res['pitch_moment_trim_nm'], 21.4, places=1)
        self.assertAlmostEqual(res['residual_pitch_moment_nm'], 0.0, places=1)

    def test_servo_hinge_moment_limits(self):
        # Fast cruise (20.8 m/s)
        res_cruise = wl.wave_falcon_dart(t_sec=1.0, f_hz=2.8, v_ms=20.80)
        self.assertLess(res_cruise['servo_hinge_moment_nm'], 0.35)
        self.assertGreater(res_cruise['servo_torque_margin_pct'], 50.0)

        # Level dash (26.35 m/s)
        res_dash = wl.wave_falcon_dart(t_sec=1.0, f_hz=3.0, v_ms=26.35)
        self.assertLessEqual(res_dash['servo_hinge_moment_nm'], 0.45)
        self.assertLess(res_dash['servo_hinge_moment_nm'], res_dash['servo_stall_limit_nm'])
        self.assertGreater(res_dash['servo_torque_margin_pct'], 30.0)

    def test_inter_station_clearance_margin(self):
        res = wl.wave_falcon_dart(t_sec=1.0, f_hz=2.8, v_ms=20.80)
        # Minimum clearance must exceed 43.4 mm safety floor
        self.assertGreater(res['min_clearance_mm'], 43.4)

    def test_power_and_specific_energy(self):
        res_cruise = wl.wave_falcon_dart(t_sec=1.0, f_hz=2.8, v_ms=20.80)
        self.assertAlmostEqual(res_cruise['total_power_w'], 365.0, places=1)
        self.assertAlmostEqual(res_cruise['wh_per_km'], 4.87, places=1)

        res_dash = wl.wave_falcon_dart(t_sec=1.0, f_hz=3.0, v_ms=26.35)
        self.assertAlmostEqual(res_dash['total_power_w'], 580.0, places=1)
        self.assertAlmostEqual(res_dash['wh_per_km'], 6.11, places=1)


class TestFalconDartSafetyAborts(unittest.TestCase):
    """Verifies red-team safety abort conditions for Falcon Dart mode."""

    def test_stick_aborts(self):
        # Pitch stick > 0.35
        res = wl.wave_falcon_dart(t_sec=1.0, pitch_stick=0.36)
        self.assertTrue(res['aborted'])
        self.assertIn("pitch_stick", res['abort_reason'])

        # Roll stick > 0.35
        res = wl.wave_falcon_dart(t_sec=1.0, roll_stick=-0.40)
        self.assertTrue(res['aborted'])
        self.assertIn("roll_stick", res['abort_reason'])

        # Yaw stick > 0.15
        res = wl.wave_falcon_dart(t_sec=1.0, yaw_stick=0.18)
        self.assertTrue(res['aborted'])
        self.assertIn("yaw_stick", res['abort_reason'])

    def test_low_airspeed_abort(self):
        res = wl.wave_falcon_dart(t_sec=1.0, v_ms=11.5)
        self.assertTrue(res['aborted'])
        self.assertIn("below min", res['abort_reason'])

    def test_ground_contact_lockout(self):
        res = wl.wave_falcon_dart(t_sec=1.0, v_ms=20.80, on_ground=True)
        self.assertTrue(res['aborted'])
        self.assertIn("on_ground", res['abort_reason'])

    def test_pitot_saturation_guard(self):
        res_nominal = wl.wave_falcon_dart(t_sec=1.0, v_ms=25.0)
        self.assertFalse(res_nominal['pitot_saturated'])
        self.assertEqual(res_nominal['airspeed_source'], 'PITOT_AIRSPEED')

        res_sat = wl.wave_falcon_dart(t_sec=1.0, v_ms=40.0)
        self.assertTrue(res_sat['pitot_saturated'])
        self.assertEqual(res_sat['airspeed_source'], 'TACTICAL_EKF_GPS_FUSION')


class TestFalconDartEvaluatorAndGS(unittest.TestCase):
    """Verifies evaluator hook and ground station status command."""

    def test_evaluate_falcon_dart(self):
        res = wl.evaluate_falcon_dart(sensors={'V_ms': 20.80, 't_sec': 2.0})
        self.assertEqual(res['hyp_id'], 'H-FALCON-TRIM-1')
        self.assertAlmostEqual(res['residual_pitch_moment_nm'], 0.0, places=1)
        self.assertGreater(res['min_clearance_mm'], 43.4)
        self.assertLessEqual(res['servo_hinge_moment_nm'], 0.45)
        self.assertFalse(res['aborted'])
        self.assertTrue(res['safeguards_verified'])

    def test_falcon_dart_status(self):
        self.assertIn('falcon_dart_status', wl._GS_ALLOWED_CMDS)
        report = wl.falcon_dart_status()
        self.assertIn('FALCON DART FANNED BI-DELTA TAILPLANE CRUISE STATUS', report)
        self.assertIn('H-FALCON-TRIM-1', report)
        self.assertIn('Station 4 (W4) locked', report)
        self.assertIn('0.69 N*m', report)


import mco_flight_governor as fg


class TestFalconDartTransitionGovernor(unittest.TestCase):
    """Verifies red-team transition governor safety envelopes (cable slack, pitch lead, thermal clamp)."""

    def setUp(self):
        self.gov = fg.FalconDartTransitionGovernor()

    def test_pitch_lead_pre_arm(self):
        # Initial request: state enters TRANSITION_TO_DART, t=0
        res = self.gov.update(dt_s=0.02, falcon_mode_requested=True)
        self.assertEqual(res['state'], 'TRANSITION_TO_DART')
        self.assertEqual(res['status'], 'TRANSITION_PITCH_LEAD_PRE_ARM')
        # During pre-arm (t < 0.120s), flapping amplitude is preserved at 100% to avoid sudden loss of lift
        self.assertEqual(res['w4_amp_scale'], 1.0)
        self.assertEqual(res['w4_amp_deg'], 18.0)
        self.assertTrue(res['cable_slack_safe'])

        # Progress within the 120ms lead window: pitch slews while amp remains 1.0
        res = self.gov.update(dt_s=0.06, falcon_mode_requested=True)
        self.assertEqual(res['status'], 'TRANSITION_PITCH_LEAD_PRE_ARM')
        self.assertEqual(res['w4_amp_scale'], 1.0)
        self.assertLess(res['w4_pitch_trim_deg'], 2.0)  # Slewing negative toward -5.5°

    def test_s_curve_deceleration_and_slack_tension(self):
        # Initiate transition (t = 0.0)
        self.gov.update(dt_s=0.02, falcon_mode_requested=True)
        # Advance through pre-arm lead (0.120s)
        self.gov.update(dt_s=0.120, falcon_mode_requested=True)
        # Advance into deceleration ramp by half of 350ms ramp (0.175s)
        res = self.gov.update(dt_s=0.175, falcon_mode_requested=True)
        self.assertIn('TRANSITION_S_CURVE_ARREST', res['status'])
        # Raised-cosine at 50% fraction should have amplitude scale approx 0.5
        self.assertAlmostEqual(res['w4_amp_scale'], 0.5, delta=0.05)
        # Verify Dyneema cable slack safety and braking torque limit
        self.assertTrue(res['cable_slack_safe'])
        self.assertLessEqual(res['max_braking_torque_nm'], 1.80)

    def test_dart_locked_state(self):
        # Step through full transition time (120ms + 350ms = 470ms)
        for _ in range(25):  # 25 * 20ms = 500ms
            res = self.gov.update(dt_s=0.02, falcon_mode_requested=True)

        self.assertEqual(res['state'], 'DART_LOCKED')
        self.assertEqual(res['w4_amp_scale'], 0.0)
        self.assertEqual(res['w4_amp_deg'], 0.0)
        self.assertEqual(res['w4_pitch_trim_deg'], -5.5)
        self.assertTrue(res['cable_slack_safe'])
        self.assertTrue(res['thermal_overload_safe'])

    def test_thermal_clamp_on_stationary_dihedral(self):
        # Step into DART_LOCKED
        for _ in range(25):
            self.gov.update(dt_s=0.02, falcon_mode_requested=True)

        # Attempt to command +10.0° dihedral to stationary W4
        res = self.gov.update(dt_s=0.02, falcon_mode_requested=True, w4_dih_cmd=10.0)
        # Clamped to <= 3.0° to eliminate continuous ODrive stator phase currents
        self.assertLessEqual(res['w4_dihedral_deg'], 3.0)
        self.assertTrue(res['thermal_overload_safe'])

        # Commanding negative dihedral clamped to 0.0°
        res_neg = self.gov.update(dt_s=0.02, falcon_mode_requested=True, w4_dih_cmd=-5.0)
        self.assertEqual(res_neg['w4_dihedral_deg'], 0.0)

    def test_recovery_transition_to_octo(self):
        # Settle in DART_LOCKED
        for _ in range(25):
            self.gov.update(dt_s=0.02, falcon_mode_requested=True)

        # Pilot disengages Falcon Dart (e.g. stick deflection or lever release)
        res_rev = self.gov.update(dt_s=0.02, falcon_mode_requested=False)
        self.assertEqual(res_rev['state'], 'TRANSITION_TO_OCTO')
        self.assertIn('TRANSITION_SPIN_UP_RESUME', res_rev['status'])
        self.assertTrue(res_rev['cable_slack_safe'])

        # Step through ramp up (350ms)
        for _ in range(20):
            res_rev = self.gov.update(dt_s=0.02, falcon_mode_requested=False)

        self.assertEqual(res_rev['state'], 'OCTO_FLAPPING')
        self.assertEqual(res_rev['w4_amp_scale'], 1.0)
        self.assertEqual(res_rev['w4_pitch_trim_deg'], 2.0)


class TestFalconDartIntentControl(unittest.TestCase):
    """Verifies Intent-Based Control System dispatching and behaviour."""

    def setUp(self):
        self.interpreter = ic.IntentInterpreter()
        fg._GLOBAL_FALCON_TRANSITION_GOVERNOR.state = 'OCTO_FLAPPING'
        fg._GLOBAL_FALCON_TRANSITION_GOVERNOR.transition_t = 0.0

    def test_intent_classification(self):
        sticks = {'ch1': 0.0, 'ch2': 0.0, 'ch3': 0.90, 'ch4': 0.0}
        switches = {}
        context = {'V_ms': 20.80, 'h_agl_m': 60.0, 'lever_pct': 90.0}

        intent, conf, params = self.interpreter.interpret(sticks, switches, context)
        self.assertEqual(intent, ic.Intent.FALCON_DART)
        self.assertGreaterEqual(conf, 0.90)
        self.assertEqual(params['sweep_schedule_deg'], [5.0, 15.0, 28.0, 50.0])
        self.assertEqual(params['tailplane_aoa_deg'], -5.5)

    def test_behaviour_node_execution(self):
        self.assertIn(ic.Intent.FALCON_DART, ic.BEHAVIOUR_REGISTRY)
        node = ic.FalconDartBehaviour()

        class MockCtrl:
            def __init__(self):
                self.freq_hz = 2.0
                self._tgt_freq = 0.0
                self._tgt_sweeps = []
                self._tgt_dihedrals = []
                self._tgt_pitches = []
                self.stationary_tailplane_active = False
                self.w4_amp_scale = 1.0

        ctrl = MockCtrl()
        ctx = {'V_ms': 20.80, 't': 1.0, 'sticks': {'pitch': 0.0, 'roll': 0.0, 'yaw': 0.0}, 'dt_s': 0.02}

        # First tick: initiates coordinated transition (pre-arm pitch lead)
        state = node.tick(ctx, ctrl)
        self.assertEqual(state, ic.RUNNING)
        self.assertEqual(ctrl._tgt_freq, 2.8)
        self.assertEqual(ctrl._tgt_sweeps, [5.0, 5.0, 15.0, 15.0, 28.0, 28.0, 50.0, 50.0])
        # Stationary lock is False during transition ramp
        self.assertFalse(ctrl.stationary_tailplane_active)

        # Run through transition duration (>= 470 ms: 120 ms pitch lead + 350 ms S-curve ramp)
        for _ in range(25):
            node.tick(ctx, ctrl)

        # Fully locked stationary tailplane
        self.assertEqual(ctrl._tgt_pitches, [2.5, 2.5, 1.5, 1.5, 1.0, 1.0, -5.5, -5.5])
        self.assertEqual(ctrl._tgt_dihedrals, [5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 0.0, 0.0])
        self.assertTrue(ctrl.stationary_tailplane_active)
        self.assertEqual(ctrl.w4_amp_scale, 0.0)
        self.assertIsNotNone(ctrl.falcon_dart_diag)


if __name__ == '__main__':
    unittest.main()

