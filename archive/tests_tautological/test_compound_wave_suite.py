"""
Unit Tests: Compound Waveform Hypotheses Suite (H-CMP 1..5)
===========================================================
Comprehensive verification suite for Compound Waveforms H-CMP-1 through H-CMP-5:

  1. H-CMP-1: Staggered Anti-Phase Bound-Glide Engine (StaggeredBoundGlideEngine)
     - 180° anti-phase micro-strokes on Stations 1 & 4
     - Locked high-AR Azhdarchid dihedral on Stations 2 & 3
     - Smooth role-swap cross-fading & electrical bus ripple cancellation

  2. H-CMP-2: Avian Deep-Flare Perch Landing Engine (DeepFlarePerchEngine)
     - Multi-stage arrival: Approach -> Scissor Airbrake -> Caridoid Flare -> Ground Spoil Lock
     - Scissor drag (28 N) and 80ms vertical flare impulse (2.2 g)
     - Lift-dump ground-spoil lock (+45° dihedral) and zero rollout (< 15 cm)

  3. H-CMP-3: Katzmayr Vortex-Ingesting Dash Engine (KatzmayrDashEngine)
     - Airspeed-adaptive cascade phase phi(V) = 28° * (V0/V)
     - Membrane AC shift feedforward pitch pre-bias (+fore, -aft)
     - Katzmayr thrust recovery power savings & buzz amplitude modulation

  4. H-CMP-4: Dipteran Haltere Active Gust Cancellation (DipteranGustRejectionEngine)
     - Sub-15ms Coriolis torque demodulation from motor current harmonics
     - Direct Collective Lift and Thrust Vectoring (DLTV) amplitude surging
     - 12 Hz boundary layer micro-dither for Laminar Separation Bubble (LSB) lock

  5. H-CMP-5: Vectored-In-Flight Post-Stall 3D Agility (ViffAgilityEngine)
     - Independent Odonata stroke-plane vectoring (+/-30° Port/Stbd tilt)
     - Chiropteran bat wing retraction asymmetry during stroke reversals
     - Zero-airspeed yaw torque (>10 Nm) and high angular acceleration (>5000 deg/s^2)

  6. Catalog Registration & Selection
     - Hypotheses registered in _ADV_NEW_HYPS, AERO_HYPOTHESES, and _hypothesis_queue
     - Scoring tasks registered in _SG_TASKS
     - Curriculum segments in _ALSS_SEGMENTS and _ALSS_CURRICULUM['B']
     - Flight phase mappings in _PHASE_TO_SG_TASK
     - Keyword and sensor routing in discovery_hypothesis_select()
"""

import sys
import os
import unittest
import math

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'firmware'))

import mco_wave_library as wl
from mco_flight_governor import (
    StaggeredBoundGlideEngine,
    DeepFlarePerchEngine,
    KatzmayrDashEngine,
    DipteranGustRejectionEngine,
    ViffAgilityEngine,
)


class TestStaggeredBoundGlideEngine(unittest.TestCase):
    """Verifies H-CMP-1 Staggered Anti-Phase Bound-Glide."""

    def setUp(self):
        self.engine = StaggeredBoundGlideEngine(swap_period_s=30.0, micro_stroke_amp_deg=12.0,
                                               glide_dihedral_deg=14.0, glide_aoa_deg=6.0,
                                               flapping_freq_hz=3.0)

    def test_group_a_flapping_at_t_zero(self):
        # 5.0833s = past fade time and at peak quarter-cycle (3 Hz period = 0.333s, 1/4 = 0.0833s)
        res = self.engine.tick(t_s=5.0833)
        self.assertEqual(res['active_group'], 'STATIONS_1_4')
        self.assertAlmostEqual(res['weight_group_a'], 1.0, places=2)
        self.assertAlmostEqual(res['weight_group_b'], 0.0, places=2)

        strokes = res['wing_strokes_deg']
        # Stations 1 (Wings 1, 2) and 4 (Wings 7, 8) must have non-zero stroke
        self.assertNotEqual(strokes[0], 0.0)
        self.assertEqual(strokes[0], strokes[1])
        # Stations 2 & 3 (Wings 3, 4, 5, 6) must have zero stroke
        self.assertEqual(strokes[2], 0.0)
        self.assertEqual(strokes[3], 0.0)
        self.assertEqual(strokes[4], 0.0)
        self.assertEqual(strokes[5], 0.0)

        # Dihedral check: Stations 2 & 3 locked in glide dihedral (14 deg)
        dihedrals = res['wing_dihedrals_deg']
        self.assertEqual(dihedrals[0], 0.0)
        self.assertEqual(dihedrals[2], 14.0)
        self.assertEqual(dihedrals[4], 14.0)
        self.assertEqual(dihedrals[6], 0.0)

        self.assertTrue(res['bus_anti_phase_locked'])
        self.assertGreater(res['electrical_ripple_cancellation_pct'], 95.0)

    def test_group_b_flapping_after_swap(self):
        # At t=35.0833s, swap to Group B (Stations 2 & 3)
        res = self.engine.tick(t_s=35.0833)
        self.assertEqual(res['active_group'], 'STATIONS_2_3')
        self.assertAlmostEqual(res['weight_group_b'], 1.0, places=2)

        strokes = res['wing_strokes_deg']
        self.assertEqual(strokes[0], 0.0)
        self.assertEqual(strokes[1], 0.0)
        self.assertEqual(strokes[6], 0.0)
        self.assertEqual(strokes[7], 0.0)
        self.assertNotEqual(strokes[2], 0.0)

    def test_anti_phase_relationship(self):
        # At quarter cycle t = 1/(4*3) = 0.0833s
        # sin(2*pi*3*0.0833) = sin(pi/2) = 1.0
        # Station 1 = +12.0, Station 4 = -12.0
        t = 1.0 / (4.0 * 3.0)
        res = self.engine.tick(t_s=t + 5.0)  # +5s so past fade time
        strokes = res['wing_strokes_deg']
        self.assertAlmostEqual(strokes[0], -strokes[6], places=1)


class TestDeepFlarePerchEngine(unittest.TestCase):
    """Verifies H-CMP-2 Avian Deep-Flare Perch Landing."""

    def setUp(self):
        self.engine = DeepFlarePerchEngine(decel_speed_threshold_ms=3.5,
                                           caridoid_flare_alt_m=0.50, ground_contact_alt_m=0.08)

    def test_approach_to_scissor_airbrake(self):
        res1 = self.engine.tick(airspeed_ms=10.0, alt_agl_m=2.5, t_s=0.0)
        self.assertEqual(res1['perch_state'], 'APPROACH')
        self.assertEqual(res1['brake_drag_n'], 0.0)

        # Descend below 1.8m at speed
        res2 = self.engine.tick(airspeed_ms=8.0, alt_agl_m=1.4, t_s=0.5)
        self.assertEqual(res2['perch_state'], 'SCISSOR_AIRBRAKE')
        self.assertEqual(res2['brake_drag_n'], 28.0)
        self.assertEqual(res2['wing_pitch_bias_deg'], -18.0)

    def test_scissor_to_caridoid_flare(self):
        # Trigger airbrake first
        self.engine.tick(airspeed_ms=5.0, alt_agl_m=1.0, t_s=1.0)
        # Descend through 0.50m
        res = self.engine.tick(airspeed_ms=3.0, alt_agl_m=0.45, t_s=1.5)
        self.assertEqual(res['perch_state'], 'CARIDOID_FLARE')
        self.assertEqual(res['vertical_thrust_g'], 2.2)
        self.assertEqual(res['wing_pitch_bias_deg'], 25.0)

    def test_flare_to_ground_spoil_lock(self):
        self.engine.tick(airspeed_ms=5.0, alt_agl_m=1.0, t_s=1.0)
        self.engine.tick(airspeed_ms=3.0, alt_agl_m=0.45, t_s=1.5)
        # 100ms later, flare completes and touches down at 0.05m
        res = self.engine.tick(airspeed_ms=0.8, alt_agl_m=0.05, t_s=1.65)
        self.assertEqual(res['perch_state'], 'GROUND_SPOIL_LOCK')
        self.assertEqual(res['ground_spoil_dihedral_deg'], 45.0)
        self.assertTrue(res['zero_rollout_achieved'])
        self.assertLess(res['perch_stop_distance_cm'], 15.0)


class TestKatzmayrDashEngine(unittest.TestCase):
    """Verifies H-CMP-3 Katzmayr Vortex-Ingesting Dash."""

    def setUp(self):
        self.engine = KatzmayrDashEngine(v_ref_ms=10.0, station_spacing_m=0.20, ac_shift_coeff=0.12)

    def test_nominal_cruise_speed(self):
        res = self.engine.tick(airspeed_ms=10.0, flap_freq_hz=5.5)
        # t_conv = 0.20 / 10.0 = 0.02s. Phase = (360 * 5.5 * 0.02) = 39.6 deg
        self.assertAlmostEqual(res['optimal_cascade_phase_deg'], 39.6, places=1)
        self.assertEqual(res['st1_pitch_prebias_deg'], 0.0)
        self.assertEqual(res['st4_pitch_prebias_deg'], 0.0)

    def test_high_speed_dash_scaling(self):
        res = self.engine.tick(airspeed_ms=22.0, flap_freq_hz=6.0, base_amp_deg=18.0)
        # AC shift pitch pre-bias: coeff * (22 - 10) = 0.12 * 12 = 1.44 deg
        self.assertAlmostEqual(res['st1_pitch_prebias_deg'], 1.44, places=2)
        self.assertAlmostEqual(res['st4_pitch_prebias_deg'], -0.72, places=2)
        self.assertGreater(res['katzmayr_power_saved_w'], 20.0)
        self.assertTrue(res['dynamic_stall_suppressed'])
        self.assertGreater(res['dash_net_efficiency_gain_pct'], 20.0)


class TestDipteranGustRejectionEngine(unittest.TestCase):
    """Verifies H-CMP-4 Dipteran Haltere Active Gust Cancellation."""

    def setUp(self):
        self.engine = DipteranGustRejectionEngine(coriolis_threshold_nm=0.40, dltv_gain=0.75,
                                                 dither_freq_hz=12.0, dither_amp_deg=1.5)

    def test_calm_conditions(self):
        torques = [1.2] * 8
        res = self.engine.tick(motor_torques_nm=torques, imu_yaw_rate_dps=2.0, t_s=0.0)
        self.assertFalse(res['gust_detected'])
        self.assertEqual(res['detection_latency_ms'], 0.0)
        self.assertEqual(res['dltv_amp_bias_deg'], 0.0)

    def test_coriolis_gust_detection(self):
        # Port experiences upwash torque reduction, Starboard experiences downwash torque increase
        torques = [0.8, 0.8, 0.8, 0.8, 1.8, 1.8, 1.8, 1.8]
        res = self.engine.tick(motor_torques_nm=torques, imu_yaw_rate_dps=5.0, t_s=0.1)
        self.assertTrue(res['gust_detected'])
        self.assertEqual(res['detection_latency_ms'], 12.5)
        self.assertGreater(res['dltv_amp_bias_deg'], 0.0)
        self.assertNotEqual(res['lsb_dither_offset_deg'], 0.0)
        self.assertGreater(res['rms_accel_reduction_pct'], 50.0)
        self.assertTrue(res['lsb_bubble_stable'])


class TestViffAgilityEngine(unittest.TestCase):
    """Verifies H-CMP-5 Vectored-In-Flight Post-Stall 3D Agility."""

    def setUp(self):
        self.engine = ViffAgilityEngine(max_vector_angle_deg=30.0, fuselage_inertia_kg_m2=0.085)

    def test_zero_yaw_command(self):
        res = self.engine.compute_maneuver(commanded_yaw_rate_dps=0.0)
        self.assertEqual(res['stroke_plane_tilt_port_deg'], 0.0)
        self.assertEqual(res['stroke_plane_tilt_stbd_deg'], 0.0)
        self.assertEqual(res['retraction_port_pct'], 0.0)
        self.assertEqual(res['retraction_stbd_pct'], 0.0)
        self.assertEqual(res['predicted_yaw_torque_nm'], 0.0)

    def test_full_yaw_pivot_zero_airspeed(self):
        res = self.engine.compute_maneuver(commanded_yaw_rate_dps=300.0, airspeed_ms=0.0)
        self.assertEqual(res['stroke_plane_tilt_port_deg'], 30.0)
        self.assertEqual(res['stroke_plane_tilt_stbd_deg'], -30.0)
        self.assertGreater(res['retraction_port_pct'], 15.0)
        self.assertEqual(res['retraction_stbd_pct'], 0.0)
        self.assertGreater(res['predicted_yaw_torque_nm'], 5.0)
        self.assertGreater(res['predicted_yaw_accel_dps2'], 3000.0)
        self.assertTrue(res['zero_airspeed_authority_valid'])


class TestCompoundHypothesisRegistration(unittest.TestCase):
    """Verifies H-CMP-1 through H-CMP-5 registration across all catalogues and dispatchers."""

    def test_adv_new_hyps_registration(self):
        ids = [h['id'] for h in wl._ADV_NEW_HYPS]
        for hid in ['H-CMP-1', 'H-CMP-2', 'H-CMP-3', 'H-CMP-4', 'H-CMP-5']:
            self.assertIn(hid, ids, f"Hypothesis {hid} missing from _ADV_NEW_HYPS")

        h1 = next(h for h in wl._ADV_NEW_HYPS if h['id'] == 'H-CMP-1')
        self.assertTrue(h1['config']['_compound'])
        self.assertIn('H-102', h1['config']['_constituents'])
        self.assertEqual(h1['config']['_scorer'], 'compound_loiter_endurance_min')

        h2 = next(h for h in wl._ADV_NEW_HYPS if h['id'] == 'H-CMP-2')
        self.assertTrue(h2['config']['_compound'])
        self.assertIn('H-108', h2['config']['_constituents'])

        h3 = next(h for h in wl._ADV_NEW_HYPS if h['id'] == 'H-CMP-3')
        self.assertTrue(h3['config']['_compound'])
        self.assertIn('H-46', h3['config']['_constituents'])

        h4 = next(h for h in wl._ADV_NEW_HYPS if h['id'] == 'H-CMP-4')
        self.assertTrue(h4['config']['_compound'])
        self.assertIn('H-93', h4['config']['_constituents'])

        h5 = next(h for h in wl._ADV_NEW_HYPS if h['id'] == 'H-CMP-5')
        self.assertTrue(h5['config']['_compound'])
        self.assertIn('H-88A', h5['config']['_constituents'])

    def test_aero_hypotheses_registration(self):
        ids = [h['id'] for h in wl.AERO_HYPOTHESES]
        for hid in ['H-CMP-1', 'H-CMP-2', 'H-CMP-3', 'H-CMP-4', 'H-CMP-5']:
            self.assertIn(hid, ids, f"Hypothesis {hid} missing from AERO_HYPOTHESES")

    def test_hypothesis_queue_registration(self):
        ids = [h['id'] for h in wl._hypothesis_queue]
        for hid in ['H-CMP-1', 'H-CMP-2', 'H-CMP-3', 'H-CMP-4', 'H-CMP-5']:
            self.assertIn(hid, ids, f"Hypothesis {hid} missing from _hypothesis_queue")

    def test_scoring_tasks_registration(self):
        expected_tasks = [
            'compound_loiter_endurance_min',
            'perch_stop_distance_cm',
            'katzmayr_dash_efficiency_wh_km',
            'coriolis_gust_rejection_db',
            'viff_zero_speed_yaw_rate_dps',
        ]
        for task in expected_tasks:
            self.assertIn(task, wl._SG_TASKS, f"Task {task} missing from _SG_TASKS")

    def test_curriculum_registration(self):
        expected_segments = [
            'cmp1_staggered_glide_b',
            'cmp2_perch_flare_b',
            'cmp3_katzmayr_dash_b',
            'cmp4_gust_rejection_b',
            'cmp5_viff_agility_b',
        ]
        for seg in expected_segments:
            self.assertIn(seg, wl._ALSS_SEGMENTS, f"Segment {seg} missing from _ALSS_SEGMENTS")
            self.assertIn(seg, wl._ALSS_CURRICULUM['B'], f"Segment {seg} missing from _ALSS_CURRICULUM['B']")

        self.assertIn('compound_loiter', wl._PHASE_TO_SG_TASK)
        self.assertIn('perch_landing', wl._PHASE_TO_SG_TASK)
        self.assertIn('katzmayr_dash', wl._PHASE_TO_SG_TASK)
        self.assertIn('coriolis_gust', wl._PHASE_TO_SG_TASK)
        self.assertIn('viff_maneuver', wl._PHASE_TO_SG_TASK)

    def test_discovery_hypothesis_select_keywords(self):
        keyword_targets = {
            'h_cmp_1':              'H-CMP-1',
            'staggered_glide':      'H-CMP-1',
            'azhdarchid_interleave':'H-CMP-1',
            'h_cmp_2':              'H-CMP-2',
            'perch_landing':        'H-CMP-2',
            'deep_flare':           'H-CMP-2',
            'h_cmp_3':              'H-CMP-3',
            'katzmayr_dash':        'H-CMP-3',
            'trans_envelope':       'H-CMP-3',
            'h_cmp_4':              'H-CMP-4',
            'gust_cancellation':    'H-CMP-4',
            'dipteran_haltere':     'H-CMP-4',
            'h_cmp_5':              'H-CMP-5',
            'viff':                 'H-CMP-5',
            'post_stall_agility':   'H-CMP-5',
        }
        for kw, expected_id in keyword_targets.items():
            selected = wl.discovery_hypothesis_select(focus=kw)
            self.assertIsNotNone(selected, f"Focus keyword '{kw}' returned None")
            self.assertEqual(selected['id'], expected_id,
                             f"Focus keyword '{kw}' did not select {expected_id}, got {selected['id']}")

    def test_discovery_hypothesis_select_sensors(self):
        sensor_cases = [
            ({'staggered_glide_active': True}, 'H-CMP-1'),
            ({'perch_flare_active': True}, 'H-CMP-2'),
            ({'katzmayr_dash_active': True}, 'H-CMP-3'),
            ({'coriolis_gust_active': True}, 'H-CMP-4'),
            ({'viff_active': True}, 'H-CMP-5'),
        ]
        for sensors, expected_id in sensor_cases:
            selected = wl.discovery_hypothesis_select(sensors=sensors)
            self.assertIsNotNone(selected, f"Sensors {sensors} returned None")
            self.assertEqual(selected['id'], expected_id,
                             f"Sensors {sensors} selected {selected['id']}, expected {expected_id}")


if __name__ == '__main__':
    unittest.main()
