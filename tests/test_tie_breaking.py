"""
Unit & Integration Tests: 5-Tier Hierarchical Tie-Breaking & Composite Selection
================================================================================
Verifies that when aerodynamic hypotheses exhibit equivalent primary performance
(|Δw| <= 2.0% in _lv_select), the selection cascades down through 5 physical tiers:
  Tier 1: Electrical & Aero Efficiency (Wh/km or J/m)
  Tier 2: Actuator Stress & Mechanical Margin (Crest Factor / Slew Rate)
  Tier 3: Control Robustness & Disturbance Rejection (Dynamic Stall Margin)
  Tier 4: Structural Vibration & Sensor Floor Degradation (IMU RMS g)
  Tier 5: Algorithmic Parsimony & Simplicity (Active DOF Complexity Count)
  Fallback: Deterministic Lexicographical Sorting

Also tests:
  - Multi-candidate tournament sorting via break_hypothesis_tie
  - Multi-objective composite score J calculation (compute_hypothesis_composite_score)
  - Full end-to-end integration into _lv_select and lever mapping
"""

import sys
import os
import unittest
import math

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'firmware'))

import mco_wave_library as wl


class TestTieBreakerTiers(unittest.TestCase):
    """Verifies that each tier of the 5-tier tie breaker resolves ties correctly."""

    def test_tier1_efficiency_decides(self):
        """Tier 1: Lower energy metric (Wh/km) wins when delta > 1.5%."""
        cand_a = {
            'mode_id': 'TEST_MODE_A',
            'wh_per_km': 4.5,
            'freq_hz': 2.0,
            'amp_port_deg': 80.0,
            'aoa_deg': 6.0,
            'cascade_phase_deg': 25.0,
        }
        cand_b = {
            'mode_id': 'TEST_MODE_B',
            'wh_per_km': 5.2,  # > 1.5% higher energy consumption
            'freq_hz': 2.0,
            'amp_port_deg': 80.0,
            'aoa_deg': 6.0,
            'cascade_phase_deg': 25.0,
        }
        res = wl.evaluate_tie_breaker(cand_a, cand_b, zone_id='CRUISE', sensors={'V_ms': 8.0})
        self.assertEqual(res['winning_id'], 'TEST_MODE_A')
        self.assertEqual(res['deciding_tier'], 'Tier 1: Efficiency')
        self.assertTrue(res['delta_pct'] > 1.5)
        self.assertTrue(any('Tier 1 (Efficiency)' in log for log in res['audit_log']))

    def test_tier2_actuator_stress_decides(self):
        """Tier 2: When efficiency tied within 1.5%, lower stress index wins when delta > 3.0%."""
        # Both candidates have identical Wh/km
        cand_a = {
            'mode_id': 'TEST_MODE_A',
            'wh_per_km': 4.5,
            'freq_hz': 2.0,
            'downstroke_ratio': 0.50,  # Symmetric duty cycle -> lower crest factor
            'amp_port_deg': 80.0,
            'aoa_deg': 6.0,
            'cascade_phase_deg': 25.0,
            'bus_shuttle_enabled': True,  # 10% back-EMF smoothing discount
        }
        cand_b = {
            'mode_id': 'TEST_MODE_B',
            'wh_per_km': 4.5,
            'freq_hz': 2.8,  # Higher flap frequency -> higher stress
            'downstroke_ratio': 0.65,  # Asymmetric duty cycle -> crest factor penalty
            'amp_port_deg': 80.0,
            'aoa_deg': 6.0,
            'cascade_phase_deg': 25.0,
            'bus_shuttle_enabled': False,
        }
        res = wl.evaluate_tie_breaker(cand_a, cand_b, zone_id='CRUISE', sensors={'V_ms': 8.0})
        self.assertEqual(res['winning_id'], 'TEST_MODE_A')
        self.assertEqual(res['deciding_tier'], 'Tier 2: Actuator Stress')
        self.assertTrue(res['delta_pct'] > 3.0)
        self.assertTrue(any('Tier 2 (Actuator Stress)' in log for log in res['audit_log']))

    def test_tier3_control_robustness_decides(self):
        """Tier 3: When T1 and T2 tied, higher dynamic stall margin wins when delta > 0.5 deg."""
        # Exact same efficiency and actuator stress kinematics
        cand_a = {
            'mode_id': 'TEST_MODE_A_LEV',
            'wh_per_km': 4.5,
            'freq_hz': 2.0,
            'downstroke_ratio': 0.50,
            'amp_port_deg': 80.0,
            'aoa_deg': 6.0,
            'cascade_phase_deg': 25.0,
            'bus_shuttle_enabled': False,
            'front_aoa_deg': 8.0,
            'rear_aoa_deg': 4.0,  # Differential pitch schedule (+2.0 deg stall bonus)
        }
        cand_b = {
            'mode_id': 'TEST_MODE_B_CONV',
            'wh_per_km': 4.5,
            'freq_hz': 2.0,
            'downstroke_ratio': 0.50,
            'amp_port_deg': 80.0,
            'aoa_deg': 6.0,
            'cascade_phase_deg': 25.0,
            'bus_shuttle_enabled': False,
            # No differential pitch
        }
        res = wl.evaluate_tie_breaker(cand_a, cand_b, zone_id='CRUISE', sensors={'V_ms': 8.0})
        self.assertEqual(res['winning_id'], 'TEST_MODE_A_LEV')
        self.assertEqual(res['deciding_tier'], 'Tier 3: Control Robustness')
        self.assertTrue(res['delta_val'] > 0.5)
        self.assertTrue(any('Tier 3 (Robustness)' in log for log in res['audit_log']))

    def test_tier4_vibration_decides(self):
        """Tier 4: When T1-T3 tied, lower vibration index wins when delta > 5.0%."""
        # Matched efficiency, stress, and stall margin
        # Mode A uses metachronal cascade phase (45 deg) -> heave cancellation
        # Mode B uses in-phase flapping (0 deg) -> full heave reaction force
        cand_a = {
            'mode_id': 'TEST_MODE_A_CASC',
            'wh_per_km': 4.5,
            'freq_hz': 2.0,
            'downstroke_ratio': 0.50,
            'amp_port_deg': 80.0,
            'aoa_deg': 6.0,
            'cascade_phase_deg': 45.0,
            'bus_shuttle_enabled': False,
        }
        cand_b = {
            'mode_id': 'TEST_MODE_B_INPHASE',
            'wh_per_km': 4.5,
            'freq_hz': 2.0,
            'downstroke_ratio': 0.50,
            'amp_port_deg': 80.0,
            'aoa_deg': 6.0,
            'cascade_phase_deg': 0.0,
            'bus_shuttle_enabled': False,
        }
        res = wl.evaluate_tie_breaker(cand_a, cand_b, zone_id='CRUISE', sensors={'V_ms': 8.0})
        self.assertEqual(res['winning_id'], 'TEST_MODE_A_CASC')
        self.assertEqual(res['deciding_tier'], 'Tier 4: Vibration')
        self.assertTrue(res['delta_pct'] > 5.0)
        self.assertTrue(any('Tier 4 (Vibration)' in log for log in res['audit_log']))

    def test_tier5_algorithmic_parsimony_decides(self):
        """Tier 5: When T1-T4 tied, candidate with fewer active DOFs / lower complexity wins."""
        # Both candidates have identical physical performance metrics
        cand_a = {
            'mode_id': 'TEST_MODE_SIMPLE',
            'wh_per_km': 4.5,
            'freq_hz': 2.0,
            'downstroke_ratio': 0.50,
            'amp_port_deg': 80.0,
            'aoa_deg': 6.0,
            'cascade_phase_deg': 25.0,
            'bus_shuttle_enabled': False,
            # Base DOF count = 6
        }
        cand_b = {
            'mode_id': 'TEST_MODE_COMPLEX',
            'wh_per_km': 4.5,
            'freq_hz': 2.0,
            'downstroke_ratio': 0.50,
            'amp_port_deg': 80.0,
            'aoa_deg': 6.0,
            'cascade_phase_deg': 25.0,
            'bus_shuttle_enabled': False,
            'h_adaptive_trigger': True,  # Adds +2 DOFs for trigger logic
        }
        res = wl.evaluate_tie_breaker(cand_a, cand_b, zone_id='CRUISE', sensors={'V_ms': 8.0})
        self.assertEqual(res['winning_id'], 'TEST_MODE_SIMPLE')
        self.assertEqual(res['deciding_tier'], 'Tier 5: Algorithmic Parsimony')
        self.assertTrue(any('Tier 5 (Parsimony)' in log for log in res['audit_log']))

    def test_deterministic_fallback_lexicographical(self):
        """Fallback: When all 5 tiers are identical, fall back to alphabetical mode ID."""
        cand_a = {
            'mode_id': 'ALPHA_MODE',
            'wh_per_km': 4.5,
            'freq_hz': 2.0,
            'downstroke_ratio': 0.50,
            'amp_port_deg': 80.0,
            'aoa_deg': 6.0,
            'cascade_phase_deg': 25.0,
            'bus_shuttle_enabled': False,
        }
        cand_b = {
            'mode_id': 'BETA_MODE',
            'wh_per_km': 4.5,
            'freq_hz': 2.0,
            'downstroke_ratio': 0.50,
            'amp_port_deg': 80.0,
            'aoa_deg': 6.0,
            'cascade_phase_deg': 25.0,
            'bus_shuttle_enabled': False,
        }
        res = wl.evaluate_tie_breaker(cand_a, cand_b, zone_id='CRUISE', sensors={'V_ms': 8.0})
        self.assertEqual(res['winning_id'], 'ALPHA_MODE')
        self.assertEqual(res['deciding_tier'], 'Deterministic Lexicographical Fallback')
        self.assertTrue(any('Deterministic Fallback' in log for log in res['audit_log']))

    def test_usage_balanced_exploration_fallback(self):
        """When candidates are identical across all tiers, least-recently-evaluated candidate wins."""
        wl.reset_tie_breaker_state()
        cand_a = ('MODE_ALPHA', 0.0, 30.0, 0.0, lambda: {'mode_id': 'MODE_ALPHA', 'wh_per_km': 5.0, 'freq_hz': 2.0, 'amp_port_deg': 80.0, 'aoa_deg': 6.0, 'cascade_phase_deg': 25.0})
        cand_b = ('MODE_BETA', 0.0, 30.0, 0.0, lambda: {'mode_id': 'MODE_BETA', 'wh_per_km': 5.0, 'freq_hz': 2.0, 'amp_port_deg': 80.0, 'aoa_deg': 6.0, 'cascade_phase_deg': 25.0})

        # Pre-seed MODE_ALPHA with 5 evaluations and MODE_BETA with 1
        wl._TIE_BREAKER_STATE['eval_counts']['MODE_ALPHA'] = 5
        wl._TIE_BREAKER_STATE['eval_counts']['MODE_BETA'] = 1

        res = wl.evaluate_tie_breaker(cand_a, cand_b, zone_id='CRUISE', sensors={'V_ms': 8.0})
        self.assertEqual(res['winning_id'], 'MODE_BETA')
        self.assertEqual(res['deciding_tier'], 'Exploration Usage Balancing')
        self.assertTrue(any('Usage Balancing' in log for log in res['audit_log']))

    def test_malformed_candidate_defensive_handling(self):
        """Candidates returning None, non-dict, or throwing exceptions must be handled safely."""
        corrupt_cand = ('CORRUPT_CAND', 0.0, 30.0, 0.0, lambda: None)
        valid_cand = ('VALID_CAND', 0.0, 30.0, 0.0, lambda: {'mode_id': 'VALID_CAND', 'wh_per_km': 4.5})
        res = wl.evaluate_tie_breaker(corrupt_cand, valid_cand, zone_id='CRUISE')
        self.assertIsNotNone(res['winner'])
        self.assertIn('deciding_tier', res)


class TestMultiCandidateTournament(unittest.TestCase):
    """Tests tournament tie-breaking across 3+ competing hypotheses."""

    def test_tournament_multiple_candidates(self):
        candidates = [
            {'mode_id': 'CAND_C', 'wh_per_km': 6.0, 'freq_hz': 2.0, 'amp_port_deg': 80.0, 'aoa_deg': 6.0, 'cascade_phase_deg': 25.0},
            {'mode_id': 'CAND_B', 'wh_per_km': 5.0, 'freq_hz': 2.0, 'amp_port_deg': 80.0, 'aoa_deg': 6.0, 'cascade_phase_deg': 25.0},
            {'mode_id': 'CAND_A', 'wh_per_km': 4.0, 'freq_hz': 2.0, 'amp_port_deg': 80.0, 'aoa_deg': 6.0, 'cascade_phase_deg': 25.0},
        ]
        winner, audit = wl.break_hypothesis_tie(candidates, zone_id='CRUISE', sensors={'V_ms': 8.0})
        self.assertEqual(winner['mode_id'], 'CAND_A')
        self.assertGreaterEqual(len(audit), 2)

    def test_single_candidate_tournament(self):
        cand = {'mode_id': 'SOLO_CAND', 'wh_per_km': 4.0}
        winner, audit = wl.break_hypothesis_tie([cand], zone_id='CRUISE')
        self.assertEqual(winner['mode_id'], 'SOLO_CAND')
        self.assertEqual(len(audit), 1)

    def test_empty_candidate_tournament(self):
        winner, audit = wl.break_hypothesis_tie([], zone_id='CRUISE')
        self.assertIsNone(winner)
        self.assertEqual(audit, [])


class TestCompositeScore(unittest.TestCase):
    """Tests multi-objective composite score J calculation across all 5 tiers."""

    def test_composite_score_bounds_and_structure(self):
        cand = {
            'mode_id': 'H-DA-5',
            'wh_per_km': 4.2,
            'freq_hz': 1.8,
            'amp_port_deg': 70.0,
            'aoa_deg': 4.8,
            'cascade_phase_deg': 28.0,
            'outboard_reflex_deg': -2.5,
        }
        score_data = wl.compute_hypothesis_composite_score(cand, zone_id='CRUISE', sensors={'V_ms': 8.0})
        self.assertIn('composite_score', score_data)
        self.assertIn('utility_breakdown', score_data)
        self.assertIn('metrics', score_data)

        j = score_data['composite_score']
        self.assertGreaterEqual(j, 0.0)
        self.assertLessEqual(j, 1.0)

        ub = score_data['utility_breakdown']
        for k in ('u_efficiency', 'u_actuator_stress', 'u_robustness', 'u_vibration', 'u_parsimony'):
            self.assertIn(k, ub)
            self.assertGreaterEqual(ub[k], 0.0)
            self.assertLessEqual(ub[k], 1.0)


class TestLvSelectIntegration(unittest.TestCase):
    """Tests integration of tie breaker within _lv_select and unified_lever_to_params."""

    def test_tie_break_trigger_in_lv_select(self):
        """When two candidates have weights within 2%, tie breaker must activate and tag params."""
        # Create mock candidate tuples: (mode_id, v_min, v_max, h_min, params_fn)
        cand_a_params = {
            'mode_id': 'CAND_ALPHA',
            'freq_hz': 2.0,
            'wh_per_km': 4.0,  # Lower energy -> winner at Tier 1
            'amp_port_deg': 80.0,
            'aoa_deg': 6.0,
            'cascade_phase_deg': 25.0,
        }
        cand_b_params = {
            'mode_id': 'CAND_BETA',
            'freq_hz': 2.0,
            'wh_per_km': 5.0,
            'amp_port_deg': 80.0,
            'aoa_deg': 6.0,
            'cascade_phase_deg': 25.0,
        }

        cands = [
            ('CAND_ALPHA', 0.0, 30.0, 0.0, lambda: dict(cand_a_params)),
            ('CAND_BETA', 0.0, 30.0, 0.0, lambda: dict(cand_b_params)),
        ]

        # Inject weights into wl._lever_mode_weights['CRUISE'] that are tied within 1%
        orig_weights = wl._lever_mode_weights.get('CRUISE', {}).copy()
        try:
            wl._lever_mode_weights['CRUISE'] = {
                'CAND_ALPHA': 1.00,
                'CAND_BETA': 0.99,  # Within 1.0% (< 2.0% parity threshold)
            }
            res = wl._lv_select('CRUISE', cands, {'V_ms': 8.0, 'h_agl_m': 100.0, 'lever_pct': 50.0})

            # Assert tie breaker was invoked
            self.assertTrue(res.get('_tie_break_active', False))
            self.assertIn('_tie_break_audit', res)
            self.assertEqual(res.get('mode_id'), 'CAND_ALPHA')
            self.assertTrue(any('Tier 1 (Efficiency)' in log for log in res['_tie_break_audit']))

        finally:
            wl._lever_mode_weights['CRUISE'] = orig_weights

    def test_no_tie_break_when_distinct_weights(self):
        """When candidates have weights with > 2% gap, tie breaker must NOT activate."""
        cand_a_params = {'mode_id': 'CAND_ALPHA', 'freq_hz': 2.0, 'wh_per_km': 6.0}
        cand_b_params = {'mode_id': 'CAND_BETA', 'freq_hz': 2.0, 'wh_per_km': 3.0}

        cands = [
            ('CAND_ALPHA', 0.0, 30.0, 0.0, lambda: dict(cand_a_params)),
            ('CAND_BETA', 0.0, 30.0, 0.0, lambda: dict(cand_b_params)),
        ]

        orig_weights = wl._lever_mode_weights.get('CRUISE', {}).copy()
        try:
            wl._lever_mode_weights['CRUISE'] = {
                'CAND_ALPHA': 1.00,
                'CAND_BETA': 0.85,  # 15% gap (> 2% parity threshold)
            }
            res = wl._lv_select('CRUISE', cands, {'V_ms': 8.0, 'h_agl_m': 100.0, 'lever_pct': 50.0})

            self.assertFalse(res.get('_tie_break_active', False))
            self.assertEqual(res.get('mode_id'), 'CAND_ALPHA')

        finally:
            wl._lever_mode_weights['CRUISE'] = orig_weights

    def test_tier2_dynamic_pitch_vs_static_trim_slew(self):
        """Tier 2: Static trim AoA demands 0 deg/s slew, while dynamic pitch demands 2*pi*f*theta."""
        cand_static = {
            'mode_id': 'STATIC_TRIM_CLIMB',
            'wh_per_km': 4.5,
            'freq_hz': 2.0,
            'aoa_deg': 15.0,  # Steep trim AoA
            'has_dynamic_pitch': False,
            'downstroke_ratio': 0.50,
            'amp_port_deg': 80.0,
            'cascade_phase_deg': 25.0,
        }
        cand_dynamic = {
            'mode_id': 'ACTIVE_PITCH_CLIMB',
            'wh_per_km': 4.5,
            'freq_hz': 2.0,
            'aoa_deg': 15.0,
            'has_dynamic_pitch': True,
            'pitch_amp_deg': 15.0,  # 15 deg amplitude oscillation at 2 Hz
            'downstroke_ratio': 0.50,
            'amp_port_deg': 80.0,
            'cascade_phase_deg': 25.0,
        }
        m_stat = wl._extract_cand_metrics(cand_static, 'CRUISE')
        m_dyn = wl._extract_cand_metrics(cand_dynamic, 'CRUISE')

        # Static trim should have 444.0 deg/s full headroom, dynamic pitch has 255.5 deg/s headroom
        self.assertEqual(m_stat['slew_headroom_degs'], 444.0)
        self.assertEqual(m_dyn['slew_headroom_degs'], 255.5)
        self.assertGreater(m_stat['slew_headroom_degs'], m_dyn['slew_headroom_degs'])
        self.assertLess(m_stat['stress_index'], m_dyn['stress_index'])

        res = wl.evaluate_tie_breaker(cand_static, cand_dynamic, 'CRUISE')
        self.assertEqual(res['winning_id'], 'STATIC_TRIM_CLIMB')
        self.assertEqual(res['deciding_tier'], 'Tier 2: Actuator Stress')

    def test_tier3_low_speed_hover_inflow_and_margin(self):
        """Tier 3: Low-speed flight (V <= 1.0 m/s) blends into hover thrust margin without singularity."""
        cand_std = {
            'mode_id': 'HOVER_STD',
            'freq_hz': 3.0,
            'amp_port_deg': 85.0,
            'aoa_deg': 10.0,
            'wh_per_km': 5.0,
        }
        cand_lev = {
            'mode_id': 'HOVER_LEV',
            'freq_hz': 3.0,
            'amp_port_deg': 85.0,
            'aoa_deg': 10.0,
            'wh_per_km': 5.0,
            'front_aoa_deg': 12.0,
            'rear_aoa_deg': 8.0,  # Differential pitch (+2.0 deg bonus)
        }
        # Test at V = 0.0 m/s (zero airspeed hover)
        m_std_0 = wl._extract_cand_metrics(cand_std, 'CRUISE', sensors={'V_ms': 0.0})
        m_lev_0 = wl._extract_cand_metrics(cand_lev, 'CRUISE', sensors={'V_ms': 0.0})

        self.assertGreater(m_std_0['stall_margin_deg'], 0.0)
        self.assertGreater(m_lev_0['stall_margin_deg'], m_std_0['stall_margin_deg'])
        self.assertFalse(math.isinf(m_std_0['stall_margin_deg']))
        self.assertFalse(math.isnan(m_std_0['stall_margin_deg']))

        # Test at V = 0.5 m/s
        m_std_05 = wl._extract_cand_metrics(cand_std, 'CRUISE', sensors={'V_ms': 0.5})
        self.assertGreater(m_std_05['stall_margin_deg'], 0.0)

    def test_tier5_turbulence_inversion_rewards_control_authority(self):
        """Tier 5: Rewards simpler model in calm air, but rewards control authority in severe turbulence."""
        cand_simple = {
            'mode_id': 'SIMPLE_MODE',
            'wh_per_km': 4.5,
            'freq_hz': 2.0,
            'downstroke_ratio': 0.50,
            'amp_port_deg': 80.0,
            'aoa_deg': 6.0,
            'cascade_phase_deg': 25.0,
        }
        cand_authority = {
            'mode_id': 'AUTHORITY_MODE',
            'wh_per_km': 4.5,
            'freq_hz': 2.0,
            'downstroke_ratio': 0.50,
            'amp_port_deg': 80.0,
            'aoa_deg': 6.0,
            'cascade_phase_deg': 25.0,
            'extra_dofs': 4,
            'h_gust_suppression_trigger': True,
        }
        # In calm conditions, SIMPLE_MODE wins via Occam's razor
        res_calm = wl.evaluate_tie_breaker(
            cand_simple, cand_authority, 'CRUISE',
            sensors={'V_ms': 8.0, 'turbulence_level': 'LIGHT', 'gust_ms': 0.2}
        )
        self.assertEqual(res_calm['winning_id'], 'SIMPLE_MODE')
        self.assertEqual(res_calm['deciding_tier'], 'Tier 5: Algorithmic Parsimony')

        # In severe turbulence, AUTHORITY_MODE wins for disturbance rejection
        res_turb = wl.evaluate_tie_breaker(
            cand_simple, cand_authority, 'CRUISE',
            sensors={'V_ms': 8.0, 'turbulence_level': 'SEVERE', 'gust_ms': 2.8}
        )
        self.assertEqual(res_turb['winning_id'], 'AUTHORITY_MODE')
        self.assertEqual(res_turb['deciding_tier'], 'Tier 5: Control Authority (Turbulence)')
        self.assertTrue(any('Control Authority - Turbulence' in log for log in res_turb['audit_log']))

    def test_schmitt_trigger_hysteresis_and_dwell(self):
        """Verifies Schmitt-trigger (2.0% enter / 3.5% exit) and dwell-time (1.5s) chatter suppression."""
        wl.reset_tie_breaker_state()

        cand_a_params = {'mode_id': 'CAND_A', 'freq_hz': 2.0, 'wh_per_km': 5.5}
        cand_b_params = {'mode_id': 'CAND_B', 'freq_hz': 2.0, 'wh_per_km': 4.0}  # Better efficiency -> wins tie-break

        cands = [
            ('CAND_A', 0.0, 30.0, 0.0, lambda: dict(cand_a_params)),
            ('CAND_B', 0.0, 30.0, 0.0, lambda: dict(cand_b_params)),
        ]

        orig_weights = wl._lever_mode_weights.get('CRUISE', {}).copy()
        try:
            # 1. Enter tie-breaking at delta = 1.5% (<= 2.0% entry threshold)
            wl._lever_mode_weights['CRUISE'] = {'CAND_A': 1.00, 'CAND_B': 0.985}
            r1 = wl._lv_select('CRUISE', cands, {'V_ms': 8.0, 'time_s': 10.0})
            self.assertEqual(r1['mode_id'], 'CAND_B')
            self.assertTrue(r1['_tie_break_active'])

            # 2. Perturb delta to 2.5% (> 2.0% entry, but <= 3.5% exit). Latch must hold CAND_B!
            wl._lever_mode_weights['CRUISE'] = {'CAND_A': 1.00, 'CAND_B': 0.975}
            r2 = wl._lv_select('CRUISE', cands, {'V_ms': 8.0, 'time_s': 10.5})
            self.assertEqual(r2['mode_id'], 'CAND_B')
            self.assertTrue(r2['_tie_break_active'])

            # 3. Increase delta to 5.0% (> 3.5% exit threshold). Primary weight favours CAND_A,
            # but time elapsed (11.0 - 10.0 = 1.0s) is LESS than mode_dwell_time_s (1.5s).
            # Dwell protection must latch and retain CAND_B!
            wl._lever_mode_weights['CRUISE'] = {'CAND_A': 1.00, 'CAND_B': 0.950}
            r3 = wl._lv_select('CRUISE', cands, {'V_ms': 8.0, 'time_s': 11.0})
            self.assertEqual(r3['mode_id'], 'CAND_B')
            self.assertTrue(r3.get('_tie_break_dwell_locked', False))

            # 4. Advance time past dwell threshold (10.0 + 1.6s = 11.6s).
            # Switch to CAND_A must now successfully execute!
            r4 = wl._lv_select('CRUISE', cands, {'V_ms': 8.0, 'time_s': 11.6})
            self.assertEqual(r4['mode_id'], 'CAND_A')
            self.assertFalse(r4.get('_tie_break_dwell_locked', False))
            self.assertFalse(r4.get('_tie_break_active', False))

        finally:
            wl._lever_mode_weights['CRUISE'] = orig_weights
            wl.reset_tie_breaker_state()


if __name__ == '__main__':
    unittest.main()
