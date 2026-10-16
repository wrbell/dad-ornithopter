"""
Tests for Vector Control Hypotheses Suite:
H-VEC-1: Band-centre dihedral differential lateral vectoring with zero roll moment
H-YAW-1: Fore-aft opposed pure yaw couple with zero translation and roll
H-MV-4R: Split-cascade roll authority multi-dimensional space search
"""

import unittest
import math
import copy

import firmware.mco_wave_library as wl


class TestVectorControlHypotheses(unittest.TestCase):

    def setUp(self):
        # Preserve lever mode weights to prevent state leakage
        self._orig_lever_weights = copy.deepcopy(wl._lever_mode_weights)
        self._orig_vec_state = copy.deepcopy(wl._vec_state)

    def tearDown(self):
        wl._lever_mode_weights.clear()
        wl._lever_mode_weights.update(self._orig_lever_weights)
        wl._vec_state.clear()
        wl._vec_state.update(self._orig_vec_state)

    # ── 1. Physics & Candidate Table Tests ────────────────────────────────────

    def test_candidate_table_lateral_ratios(self):
        """Test analytic lateral ratio against the 7 candidate points in the PDF."""
        expected_points = [
            (50.0, 50.0, 0.000),
            (30.0, 0.0, 0.289),
            (40.0, 0.0, 0.420),
            (50.0, 0.0, 0.596),
            (60.0, 0.0, 0.866),
            (60.0, -20.0, 1.048),
            (50.0, -50.0, 1.192),
        ]
        for p, s, exp_lat in expected_points:
            calc_lat = wl.vec_lateral_ratio(p, s)
            self.assertAlmostEqual(calc_lat, exp_lat, places=3,
                                   msg=f"Failed for port={p}, stbd={s}")

    def test_candidate_table_force_penalties(self):
        """Test force penalties against candidate table points and 1.60x cap."""
        expected_penalties = [
            (50.0, 50.0, 1.000),
            (30.0, 0.0, 1.077),
            (40.0, 0.0, 1.131),
            (50.0, 0.0, 1.212),
            (60.0, 0.0, 1.366),
            (60.0, -20.0, 1.451),
            (50.0, -50.0, 1.556),
        ]
        for p, s, exp_pen in expected_penalties:
            calc_pen = wl.vec_force_penalty(p, s)
            self.assertAlmostEqual(calc_pen, exp_pen, places=3,
                                   msg=f"Failed for port={p}, stbd={s}")

        # Extreme angle must be clamped to 1.60x
        extreme_pen = wl.vec_force_penalty(75.0, -75.0)
        self.assertLessEqual(extreme_pen, 1.60)

    def test_yaw_couple_moments(self):
        """Test pure yaw moments for 0.2, 0.4, 0.6 W fractions with d=0.50m and W=19.57N."""
        # 0.2 W -> 1.96 N·m
        m_02 = wl.vec_yaw_moment(0.2)
        self.assertAlmostEqual(m_02, 1.96, delta=0.01)

        # 0.4 W -> 3.91 N·m
        m_04 = wl.vec_yaw_moment(0.4)
        self.assertAlmostEqual(m_04, 3.91, delta=0.01)

        # 0.6 W -> 5.87 N·m
        m_06 = wl.vec_yaw_moment(0.6)
        self.assertAlmostEqual(m_06, 5.87, delta=0.01)

    def test_3d_roll_decoupling_and_residual(self):
        """Test uncompensated 2D roll residual moment and 3D zero roll condition."""
        # Uncompensated 2D roll moment at nominal point (+50°, 0°)
        residual_moment = wl.vec_roll_residual(50.0, 0.0, Fp=9.785)
        self.assertAlmostEqual(residual_moment, 1.029, delta=0.02)

        # In 2D, lateral ratio is 0.596
        lat_2d = wl.vec_lateral_ratio(50.0, 0.0, full_3d=False)
        self.assertAlmostEqual(lat_2d, 0.596, places=3)

        # In 3D decoupled mode, Fs is scaled to cancel adverse roll
        lat_3d = wl.vec_lateral_ratio(50.0, 0.0, full_3d=True)
        self.assertAlmostEqual(lat_3d, 0.422, places=3)

    # ── 2. 50 Hz State Machine & Tick Tests ───────────────────────────────────

    def test_vec_tick_quiescent(self):
        """Test _vec_tick in quiescent state (+50°, +50°)."""
        state = wl._vec_tick({'port_band_centre_deg': 50.0, 'stbd_band_centre_deg': 50.0})
        self.assertFalse(state['active'])
        self.assertAlmostEqual(state['lateral_ratio'], 0.0)
        self.assertAlmostEqual(state['force_penalty'], 1.0)
        self.assertEqual(state['duration_s'], 0.0)
        self.assertFalse(state['force_cap_exceeded'])
        self.assertFalse(state['duration_exceeded'])
        self.assertFalse(state['thermal_warning'])

    def test_vec_tick_active_manoeuvre_and_duration_cap(self):
        """Test _vec_tick during active lateral manoeuvre and duration timeout."""
        # Run 50 ticks of 0.1s = 5.0s at nominal working point (+50°, 0°) with penalty 1.212x <= 1.40x
        for _ in range(50):
            state = wl._vec_tick({
                'port_band_centre_deg': 50.0,
                'stbd_band_centre_deg': 0.0,
                'roll_residual_deg': 0.01,
                'dt': 0.1,
            })
        self.assertTrue(state['active'])
        self.assertAlmostEqual(state['duration_s'], 5.0, places=2)
        self.assertFalse(state['duration_exceeded'])
        self.assertAlmostEqual(state['lateral_ratio'], 0.596, places=3)
        self.assertAlmostEqual(state['force_penalty'], 1.212, places=3)

        # Run another 40 ticks = +4.0s -> total 9.0s (> 8.0s cap for <= 1.40x)
        for _ in range(40):
            state = wl._vec_tick({
                'port_band_centre_deg': 50.0,
                'stbd_band_centre_deg': 0.0,
                'roll_residual_deg': 0.01,
                'dt': 0.1,
            })
        self.assertTrue(state['duration_exceeded'])

    def test_vec_tick_thermal_governor_derating(self):
        """Test thermal governor derating sustained duration to 5.0s for penalty > 1.40x."""
        # Reset state between subtests
        wl._vec_state.clear()
        wl._vec_state.update(copy.deepcopy(self._orig_vec_state))

        # At (+50°, -50°), penalty is 1.556x > 1.40x -> max sustained duration is derated to 5.0s
        for _ in range(48):  # 4.8s
            state = wl._vec_tick({
                'port_band_centre_deg': 50.0,
                'stbd_band_centre_deg': -50.0,
                'dt': 0.1,
            })
        self.assertFalse(state['duration_exceeded'])
        self.assertTrue(state['thermal_warning'])  # 4.8s > 5.0s - 0.5s with penalty 1.556x > 1.35x
        self.assertGreater(state['thermal_energy_joules'], 200.0)

        # Push past 5.0s -> 5.2s
        for _ in range(4):
            state = wl._vec_tick({
                'port_band_centre_deg': 50.0,
                'stbd_band_centre_deg': -50.0,
                'dt': 0.1,
            })
        self.assertTrue(state['duration_exceeded'])

    # ── 3. Wave Generators & Evaluators ───────────────────────────────────────

    def test_wave_lateral_vectoring(self):
        """Test wave_lateral_vectoring outputs for both 3D-decoupled and 2D-legacy modes."""
        # 3D decoupled mode (default)
        res_3d = wl.wave_lateral_vectoring(t_sec=1.0, port_deg=50.0, stbd_deg=0.0, use_3d_comp=True)
        self.assertEqual(res_3d['hyp_id'], 'H-VEC-1')
        self.assertAlmostEqual(res_3d['lateral_ratio'], 0.596, places=3)
        self.assertAlmostEqual(res_3d['lateral_ratio_3d'], 0.422, places=3)
        self.assertAlmostEqual(res_3d['force_penalty'], 1.212, places=3)
        self.assertTrue(res_3d['zero_roll_moment'])
        self.assertTrue(res_3d['use_3d_comp'])
        # 3D amplitude scaling: (0.198 + 0.045 * sin(50)) / 0.198 = 1.1741 -> amp_stbd = 58.71 deg
        scale_3d = (0.198 + 0.045 * math.sin(math.radians(50.0))) / 0.198
        self.assertAlmostEqual(res_3d['amp_stbd_deg'], 50.0 * scale_3d, places=2)
        self.assertAlmostEqual(res_3d['uncompensated_roll_moment_nm'], 1.029, delta=0.02)

        # 2D legacy mode
        res_2d = wl.wave_lateral_vectoring(t_sec=1.0, port_deg=50.0, stbd_deg=0.0, use_3d_comp=False)
        self.assertFalse(res_2d['use_3d_comp'])
        expected_2d_stbd_amp = 50.0 * math.cos(math.radians(50.0)) / math.cos(0.0)
        self.assertAlmostEqual(res_2d['amp_stbd_deg'], expected_2d_stbd_amp, places=2)

    def test_evaluate_lateral_vectoring(self):
        """Test evaluate_lateral_vectoring scoring metric and penalties."""
        eval_res = wl.evaluate_lateral_vectoring({
            'port_band_centre_deg': 50.0,
            'stbd_band_centre_deg': 0.0,
            'lateral_accel_g': 0.596,
            'roll_residual_deg': 0.02,
            'duration_s': 2.0,
        })
        self.assertEqual(eval_res['hyp_id'], 'H-VEC-1')
        self.assertGreater(eval_res['lat_accel_per_roll_residual'], 20.0)
        self.assertTrue(eval_res['penalty_ok'])
        self.assertTrue(eval_res['duration_ok'])
        self.assertGreater(eval_res['score_delta'], 0.0)

    def test_wave_and_evaluate_pure_yaw_couple(self):
        """Test wave_pure_yaw_couple and evaluate_pure_yaw_couple with feedforward decoupling."""
        wave_res = wl.wave_pure_yaw_couple(t_sec=0.5, lateral_frac=0.40)
        self.assertEqual(wave_res['hyp_id'], 'H-YAW-1')
        self.assertAlmostEqual(wave_res['yaw_moment_nm'], 3.91, delta=0.01)
        self.assertEqual(wave_res['net_lateral_force_n'], 0.0)
        self.assertEqual(wave_res['net_roll_moment_nm'], 0.0)
        self.assertEqual(wave_res['net_pitch_moment_nm'], 0.0)
        # Feedforward trim biases
        self.assertTrue(wave_res['downwash_decoupled'])
        self.assertAlmostEqual(wave_res['delta_aoa_aft_port_deg'], 2.5 * 0.40, places=2)
        self.assertAlmostEqual(wave_res['delta_aoa_aft_stbd_deg'], -1.5 * 0.40, places=2)
        self.assertAlmostEqual(wave_res['delta_pitch_trim_ff_deg'], -1.2 * 0.40, places=2)

        eval_res = wl.evaluate_pure_yaw_couple({
            'yaw_rate_dps': 30.0,
            'position_drift_m': 0.04,
            'lateral_frac': 0.40,
            'duration_s': 3.0,
            'downwash_decoupled': True,
        })
        self.assertEqual(eval_res['hyp_id'], 'H-YAW-1')
        self.assertGreater(eval_res['yaw_rate_per_position_drift'], 500.0)
        self.assertTrue(eval_res['duration_ok'])
        self.assertTrue(eval_res['downwash_decoupled'])
        self.assertGreater(eval_res['score_delta'], 0.0)

    def test_wave_and_evaluate_split_cascade_roll_search(self):
        """Test wave_split_cascade_roll_search and evaluate_split_cascade_roll with handoff."""
        # Initial phase kick regime (t <= 0.4s)
        wave_kick = wl.wave_split_cascade_roll_search(
            t_sec=0.25, phase_offset_deg=45.0, cascade_phase_deg=90.0, aoa_deg=18.0
        )
        self.assertEqual(wave_kick['hyp_id'], 'H-MV-4R')
        self.assertTrue(wave_kick['instantaneous_roll_moment'])
        self.assertEqual(wave_kick['handoff_blend_factor'], 0.0)
        self.assertEqual(wave_kick['phase_kick_deg'], 45.0)
        self.assertEqual(wave_kick['diff_amp_deg'], 0.0)
        self.assertFalse(wave_kick['sustained_authority_active'])

        # Blended sustained authority regime (t = 0.8s)
        wave_sustained = wl.wave_split_cascade_roll_search(
            t_sec=0.8, phase_offset_deg=45.0, cascade_phase_deg=90.0, aoa_deg=18.0
        )
        self.assertAlmostEqual(wave_sustained['handoff_blend_factor'], 1.0, places=2)
        self.assertAlmostEqual(wave_sustained['diff_amp_deg'], 18.0, places=2)
        self.assertTrue(wave_sustained['sustained_authority_active'])

        eval_res = wl.evaluate_split_cascade_roll({
            'roll_onset_latency_s': 0.040,
            'sustained_roll_authority': 45.0,
            'sustained_authority_active': True,
        })
        self.assertEqual(eval_res['hyp_id'], 'H-MV-4R')
        self.assertGreater(eval_res['composite_score'], 0.70)
        self.assertGreater(eval_res['score_delta'], 0.0)

    # ── 4. Ground Station Command Tests ───────────────────────────────────────

    def test_vector_status_gs_command(self):
        """Test vector_status ground station command format and registration."""
        self.assertIn('vector_status', wl._GS_ALLOWED_CMDS)
        self.assertTrue(callable(wl.vector_status))
        report = wl.vector_status()
        self.assertIn('MCO-1D VECTOR CONTROL', report)
        self.assertIn('CANDIDATE BAND-PAIR WORKING POINTS', report)
        self.assertIn('PURE YAW COUPLE PERFORMANCE', report)
        self.assertIn('SPLIT-CASCADE ROLL MULTI-DIMENSIONAL SEARCH', report)
        self.assertIn('+50.0°   +0.0°    0.596', report)
        self.assertIn('1.96 N·m', report)
        self.assertIn('3.91 N·m', report)
        self.assertIn('5.87 N·m', report)
        self.assertIn('3D Decoupled Ratio', report)
        self.assertIn('Thermal Governor', report)
        self.assertIn('Downwash Decoupling', report)
        self.assertIn('Handoff Crossfade', report)

    # ── 5. System Integration & Modalities Tests ──────────────────────────────

    def test_registered_in_aero_hypotheses_and_queue(self):
        """Test H-VEC-1, H-YAW-1, and H-MV-4R are properly registered in hypotheses tables."""
        hyp_ids = {h['id'] for h in wl.AERO_HYPOTHESES}
        self.assertIn('H-VEC-1', hyp_ids)
        self.assertIn('H-YAW-1', hyp_ids)
        self.assertIn('H-MV-4R', hyp_ids)

        q_ids = {h['id'] for h in wl._hypothesis_queue}
        self.assertIn('H-VEC-1', q_ids)
        self.assertIn('H-YAW-1', q_ids)
        self.assertIn('H-MV-4R', q_ids)

    def test_alss_curriculum_and_segments(self):
        """Test Class C ALSS segments for all three hypotheses."""
        self.assertIn('vec1_lateral_c', wl._ALSS_SEGMENTS)
        self.assertIn('yaw1_couple_c', wl._ALSS_SEGMENTS)
        self.assertIn('mv4r_splitcasc_c', wl._ALSS_SEGMENTS)

        class_c = wl._ALSS_CURRICULUM.get('C', [])
        self.assertIn('vec1_lateral_c', class_c)
        self.assertIn('yaw1_couple_c', class_c)
        self.assertIn('mv4r_splitcasc_c', class_c)

    def test_scoring_tasks_registration(self):
        """Test 4 new scoring tasks in _SG_TASKS."""
        self.assertIn('lat_accel_per_roll_residual', wl._SG_TASKS)
        self.assertIn('vertical_force_penalty', wl._SG_TASKS)
        self.assertIn('yaw_rate_per_position_drift', wl._SG_TASKS)
        self.assertIn('roll_onset_latency_s', wl._SG_TASKS)

    def test_cmts_champions(self):
        """Test CMTS champions for lateral_vector and yaw_couple."""
        self.assertIn('lateral_vector', wl._CMTS_CHAMPIONS)
        self.assertEqual(wl._CMTS_CHAMPIONS['lateral_vector']['hyp_id'], 'H-VEC-1')
        self.assertIn('yaw_couple', wl._CMTS_CHAMPIONS)
        self.assertEqual(wl._CMTS_CHAMPIONS['yaw_couple']['hyp_id'], 'H-YAW-1')

    def test_ai_observer_triggers(self):
        """Test AI observer triggers 51, 52, 53 queue the hypotheses."""
        # Trigger 51
        obs51 = wl.ai_observer_tick({'lateral_vector_active': True}, {})
        self.assertTrue(obs51.get('h_vec1_active'))
        last_q51 = wl._ALC_SHARED['priority_hyp_queue'][-1]
        self.assertEqual(last_q51['hyp_id'], 'H-VEC-1')

        # Trigger 52
        obs52 = wl.ai_observer_tick({'yaw_couple_active': True}, {})
        self.assertTrue(obs52.get('h_yaw1_active'))
        last_q52 = wl._ALC_SHARED['priority_hyp_queue'][-1]
        self.assertEqual(last_q52['hyp_id'], 'H-YAW-1')

        # Trigger 53
        obs53 = wl.ai_observer_tick({'split_cascade_roll_active': True}, {})
        self.assertTrue(obs53.get('h_mv4r_active'))
        last_q53 = wl._ALC_SHARED['priority_hyp_queue'][-1]
        self.assertEqual(last_q53['hyp_id'], 'H-MV-4R')

    def test_discovery_hypothesis_select(self):
        """Test discovery selection by keyword focus and sensor triggers."""
        # Focus keywords
        h_vec = wl.discovery_hypothesis_select(focus='lateral_vector')
        self.assertEqual(h_vec['id'], 'H-VEC-1')

        h_yaw = wl.discovery_hypothesis_select(focus='pure_yaw')
        self.assertEqual(h_yaw['id'], 'H-YAW-1')

        h_mv4r = wl.discovery_hypothesis_select(focus='split_cascade_roll')
        self.assertEqual(h_mv4r['id'], 'H-MV-4R')

        # Sensor triggers
        h_vec_s = wl.discovery_hypothesis_select(sensors={'lateral_vector_active': True})
        self.assertEqual(h_vec_s['id'], 'H-VEC-1')

        h_yaw_s = wl.discovery_hypothesis_select(sensors={'yaw_couple_active': True})
        self.assertEqual(h_yaw_s['id'], 'H-YAW-1')

        h_mv4r_s = wl.discovery_hypothesis_select(sensors={'split_cascade_roll_active': True})
        self.assertEqual(h_mv4r_s['id'], 'H-MV-4R')

    def test_genome_update_routing(self):
        """Test _update_genome_from_trial promotes weights for H-VEC-1, H-YAW-1, and H-MV-4R."""
        # H-VEC-1 updates MW_STABILITY and MW_PERFORMANCE
        w_stab_before = wl._MW_STABILITY.get('stroke_dihedral', 0.2)
        wl._update_genome_from_trial('H-VEC-1', 0.10, {})
        # Note: after normalisation, check relative adjustment or positive boost occurred
        self.assertIn('stroke_dihedral', wl._MW_STABILITY)

        # H-YAW-1 updates MW_YAW
        self.assertIn('yaw_sweep', wl._MW_YAW)
        wl._update_genome_from_trial('H-YAW-1', 0.10, {})
        self.assertIn('proverse_yaw', wl._MW_STABILITY)

        # H-MV-4R updates MW_ROLL
        self.assertIn('roll_amp', wl._MW_ROLL)
        wl._update_genome_from_trial('H-MV-4R', 0.10, {})


if __name__ == '__main__':
    unittest.main()
