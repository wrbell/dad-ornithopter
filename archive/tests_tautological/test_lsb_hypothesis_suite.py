"""
Unit Tests: Laminar Separation Bubble (LSB) Stability & Burst Suite (H-LSB-1)
=============================================================================
Comprehensive test suite verifying:
  1. Hypothesis definitions and metadata in AERO_HYPOTHESES, _ADV_NEW_HYPS, and BIOMIMETIC_HYPOTHESES
  2. ALSS scoring tasks in _SG_TASKS (lsb_stroke_variance, lsb_cross_correlation, lsb_burst_rate)
  3. Curriculum enrollment in _ALSS_SEGMENTS, _ALSS_CURRICULUM['B'], and _PHASE_TO_SG_TASK
  4. Expanded parameter discovery bounds in _EXPANDED_DISCOVERY_BOUNDS
  5. CMTS Context Champion in _CMTS_CHAMPIONS (lsb_boundary_stability)
  6. Genome evolution weight vector routing in _update_genome_from_trial
  7. Discovery keyword and sensor routing in discovery_hypothesis_select
  8. AI Observer Trigger 34 integration in ai_observer_tick (Tier-1 and Tier-2 alerts)
  9. LSBDetector in mco_flight_governor:
     - Multi-channel stroke variance tracking
     - Gust vs. Burst spatial discrimination (coherent gust rejection via cross-wing correlation)
     - Tier-1 and Tier-2 alerting thresholds
     - Pairwise Pearson correlation computation across 8 independent motor channels
"""

import sys
import os
import unittest
import math

# Ensure firmware directory is in python path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'firmware'))

import mco_wave_library as wl
from mco_flight_governor import (
    LSBDetector,
    compute_pairwise_correlation,
    LSB_CORRELATION_GUST_THRESH,
    LSB_CORRELATION_BURST_THRESH,
    LSB_VARIANCE_CLEAN_REF,
    LSB_VARIANCE_BURST_THRESH,
    LSB_APPROACH_ALERT_RATE,
)


class TestLSBHypothesisRegistration(unittest.TestCase):
    """Verifies H-LSB-1 registration and metadata across library catalogs."""

    def test_registered_in_adv_new_hyps(self):
        adv_ids = {h['id'] for h in wl._ADV_NEW_HYPS}
        self.assertIn('H-LSB-1', adv_ids, "H-LSB-1 missing from _ADV_NEW_HYPS")

    def test_registered_in_aero_hypotheses(self):
        aero_ids = {h['id'] for h in wl.AERO_HYPOTHESES}
        self.assertIn('H-LSB-1', aero_ids, "H-LSB-1 missing from AERO_HYPOTHESES")
        h = next(h for h in wl.AERO_HYPOTHESES if h['id'] == 'H-LSB-1')
        self.assertEqual(h['target'], 'smooth_flight')
        self.assertIn('lsb_stroke_variance', h['config']['_scorer'])
        self.assertIn('cruise', h['eval_phases'])
        self.assertIn('glide', h['eval_phases'])

    def test_registered_in_queue(self):
        queue_ids = {h['id'] for h in wl._hypothesis_queue}
        self.assertIn('H-LSB-1', queue_ids, "H-LSB-1 missing from _hypothesis_queue")

    def test_registered_in_biomimetic(self):
        self.assertIn('H-LSB-1', wl.BIOMIMETIC_HYPOTHESES, "H-LSB-1 missing from BIOMIMETIC_HYPOTHESES")


class TestLSBScoringAndCurriculum(unittest.TestCase):
    """Verifies ALSS tasks, curricula, bounds, and champions."""

    def test_sg_tasks_metrics(self):
        expected_metrics = [
            'lsb_stroke_variance',
            'lsb_cross_correlation',
            'lsb_burst_rate',
        ]
        for m in expected_metrics:
            self.assertIn(m, wl._SG_TASKS, f"{m} missing from _SG_TASKS")
            task = wl._SG_TASKS[m]
            self.assertIn('axis', task)
            self.assertIn('sensor', task)
            self.assertIn('direction', task)

        # Variance: lower is better (-1)
        self.assertEqual(wl._SG_TASKS['lsb_stroke_variance']['direction'], -1)
        # Correlation: higher is better (+1)
        self.assertEqual(wl._SG_TASKS['lsb_cross_correlation']['direction'], +1)
        # Burst rate: lower is better (-1)
        self.assertEqual(wl._SG_TASKS['lsb_burst_rate']['direction'], -1)

    def test_curriculum_enrolment(self):
        self.assertIn('lsb_stability_b', wl._ALSS_SEGMENTS)
        seg = wl._ALSS_SEGMENTS['lsb_stability_b']
        self.assertEqual(seg[0], 'cruise')
        self.assertEqual(seg[1], 'H-LSB-1')
        self.assertIn('lsb_stability_b', wl._ALSS_CURRICULUM.get('B', []))

    def test_phase_to_sg_task_mapping(self):
        self.assertIn('lsb_stability', wl._PHASE_TO_SG_TASK)
        tasks = wl._PHASE_TO_SG_TASK['lsb_stability']
        self.assertIn('lsb_stroke_variance', tasks)
        self.assertIn('lsb_cross_correlation', tasks)
        self.assertIn('lsb_burst_rate', tasks)

    def test_discovery_bounds(self):
        expected_bounds = [
            'h_lsb_aoa_deg',
            'h_lsb_freq_hz',
            'h_lsb_amp_deg',
            'h_lsb_cascade_deg',
        ]
        for key in expected_bounds:
            self.assertIn(key, wl._EXPANDED_DISCOVERY_BOUNDS, f"{key} missing from bounds")
            lo, hi = wl._EXPANDED_DISCOVERY_BOUNDS[key]
            self.assertLessEqual(lo, hi)

    def test_cmts_champion(self):
        self.assertIn('lsb_boundary_stability', wl._CMTS_CHAMPIONS)
        champ = wl._CMTS_CHAMPIONS['lsb_boundary_stability']
        self.assertEqual(champ['hyp_id'], 'H-LSB-1')
        self.assertIn('burst_rate', champ)


class TestLSBDiscoveryAndRouting(unittest.TestCase):
    """Verifies keyword, sensor, detent, and genome routing."""

    def test_discovery_by_focus_keywords(self):
        keywords = ['lsb', 'separation_bubble', 'bubble_burst', 'boundary_stability', 'low_reynolds', 'chord_separation']
        for kw in keywords:
            h = wl.discovery_hypothesis_select(focus=kw)
            self.assertIsNotNone(h, f"Failed to select hypothesis for focus='{kw}'")
            self.assertEqual(h['id'], 'H-LSB-1', f"Focus '{kw}' did not route to H-LSB-1")

    def test_discovery_by_sensor_flags(self):
        for flag in ('lsb_alert', 'bubble_burst', 'lsb_active'):
            sensors = {flag: True, 'V_ms': 12.0}
            h = wl.discovery_hypothesis_select(sensors=sensors)
            self.assertIsNotNone(h, f"Failed to select hypothesis with sensor flag '{flag}'")
            self.assertEqual(h['id'], 'H-LSB-1')

    def test_genome_update_routing(self):
        orig_yaw = wl._MW_STABILITY.get('proverse_yaw', 0.0)
        orig_pitch = wl._MW_PITCH.get('pitch_aoa', 0.0)

        sensors = {'V_ms': 12.0, 'h_agl_m': 20.0}
        wl._update_genome_from_trial('H-LSB-1', 0.25, sensors)

        self.assertGreater(wl._MW_STABILITY.get('proverse_yaw', 0.0), orig_yaw)
        self.assertGreater(wl._MW_PITCH.get('pitch_aoa', 0.0), orig_pitch)


class TestLSBDetectorUnit(unittest.TestCase):
    """Direct unit tests for LSBDetector online algorithm and spatial filtering."""

    def setUp(self):
        self.detector = LSBDetector(window_strokes=10, samples_per_stroke=20)

    def test_nominal_attached_flow(self):
        """Under attached flow, variance is low and joint score is high."""
        # 10 strokes of clean, highly repeatable current profiles (approx 12A with micro-noise)
        for stroke in range(12):
            for sample in range(20):
                # Small intra-stroke sinusoidal variation, identical between strokes
                curr = 12.0 + math.sin(sample / 20.0 * math.pi) * 1.5
                currents = [curr + 0.02 * (i % 2) for i in range(8)]
                self.detector.add_sample(currents, is_stroke_end=(sample == 19))

        status = self.detector.get_status(mean_drag_n=5.0, is_approach=False)
        self.assertLess(status['lsb_stroke_variance'], LSB_VARIANCE_BURST_THRESH)
        self.assertEqual(status['tier_alert'], 0)
        self.assertEqual(status['status'], 'ATTACHED_LAMINAR_BUBBLE_STABLE')
        self.assertGreater(status['joint_score'], 0.70)

    def test_gust_spatial_discrimination(self):
        """A coherent gust perturbs all 8 wings simultaneously (r_cross >= 0.55), NOT an LSB burst."""
        self.detector.reset()
        # Settle baseline cycle expectation
        for stroke in range(6):
            for sample in range(20):
                curr = 12.0 + math.sin(sample / 20.0 * math.pi) * 1.5
                self.detector.add_sample([curr] * 8, is_stroke_end=(sample == 19))

        # Coherent dynamic gust across all 8 wings
        for stroke in range(6):
            for sample in range(20):
                gust = math.sin(sample / 3.0) * 2.5
                curr = 12.0 + math.sin(sample / 20.0 * math.pi) * 1.5 + gust
                self.detector.add_sample([curr] * 8, is_stroke_end=(sample == 19))

        status = self.detector.get_status(mean_drag_n=6.0, is_approach=False)
        # Cross correlation should be near 1.0 (coherent)
        self.assertGreaterEqual(status['lsb_cross_correlation'], LSB_CORRELATION_GUST_THRESH)
        # Even with elevated variance, it should NOT flag as LSB burst because it's a gust
        self.assertFalse(status['is_burst_detected'])

    def test_localized_bubble_burst_detection(self):
        """Localized burst on wing 0 causes high variance with low cross-wing correlation."""
        self.detector.reset()
        import random
        rng = random.Random(42)

        # Baseline strokes to settle cycle expectation
        for stroke in range(8):
            for sample in range(20):
                curr = 10.0 + math.sin(sample / 20.0 * math.pi)
                currents = [curr + rng.uniform(-0.05, 0.05) for _ in range(8)]
                self.detector.add_sample(currents, is_stroke_end=(sample == 19))

        # Burst strokes: wing 0 experiences intermittent boundary layer detachment
        for stroke in range(8):
            burst_noise = rng.uniform(-4.0, 4.0)
            for sample in range(20):
                curr_base = 10.0 + math.sin(sample / 20.0 * math.pi)
                currents = [curr_base + (burst_noise if i == 0 else rng.uniform(-0.05, 0.05)) for i in range(8)]
                self.detector.add_sample(currents, is_stroke_end=(sample == 19))

        status = self.detector.get_status(mean_drag_n=8.0, is_approach=False)
        self.assertGreaterEqual(status['lsb_stroke_variance'], LSB_VARIANCE_BURST_THRESH)
        self.assertLessEqual(status['lsb_cross_correlation'], LSB_CORRELATION_BURST_THRESH)
        self.assertTrue(status['is_burst_detected'])
        self.assertGreaterEqual(status['tier_alert'], 1)

    def test_tier_2_approach_safety_alert(self):
        """Approach phase with burst rate >= 15% triggers Tier-2 alert."""
        self.detector.reset()
        # Force burst flags
        self.detector.recent_burst_flags = [True, True, True, False, False]
        self.detector.last_burst_rate = 0.60
        self.detector.last_mean_variance = 0.030

        status = self.detector.get_status(mean_drag_n=9.0, is_approach=True)
        self.assertEqual(status['tier_alert'], 2)
        self.assertEqual(status['status'], 'WARN_LSB_BURST_APPROACH_LIFT_RISK')

    def test_pairwise_correlation_math(self):
        """Verifies Pearson correlation helper across matrix channels."""
        # 3 perfectly correlated channels
        matrix_corr = [
            [1.0, 2.0, 3.0, 4.0, 5.0],
            [2.0, 4.0, 6.0, 8.0, 10.0],
            [10.0, 20.0, 30.0, 40.0, 50.0],
        ]
        r = compute_pairwise_correlation(matrix_corr)
        self.assertAlmostEqual(r, 1.0, places=4)

        # Uncorrelated / anti-correlated channels
        matrix_anti = [
            [1.0, 2.0, 3.0, 4.0, 5.0],
            [5.0, 4.0, 3.0, 2.0, 1.0],
        ]
        r_anti = compute_pairwise_correlation(matrix_anti)
        self.assertAlmostEqual(r_anti, -1.0, places=4)


class TestAIObserverLSBIntegration(unittest.TestCase):
    """Verifies AI Observer Trigger 34 in ai_observer_tick."""

    def setUp(self):
        wl._ALC_SHARED['alert_log'].clear()
        wl._ALC_SHARED['priority_hyp_queue'].clear()
        det = wl._get_lsb_detector()
        if det:
            det.reset()

    def test_observer_processes_currents(self):
        # Send stroke-end reading
        sensors = {
            'V_ms': 11.0,
            'w_sink': -0.2,
            'motor_currents_a': [12.0] * 8,
            'is_stroke_end': True,
            'is_approach': False,
            'h_agl_m': 30.0,
        }
        obs = wl.ai_observer_tick(sensors, {})
        self.assertIsInstance(obs, dict)

    def test_observer_dispatches_tier2_alert(self):
        det = wl._get_lsb_detector()
        self.assertIsNotNone(det)
        det.recent_burst_flags = [True, True, True, True]
        det.last_burst_rate = 0.50
        det.last_mean_variance = 0.025

        sensors = {
            'V_ms': 9.0,
            'w_sink': -0.8,
            'motor_currents_a': [14.0] * 8,
            'is_stroke_end': True,
            'flight_phase': 'approach',
            'h_agl_m': 10.0,
        }
        obs = wl.ai_observer_tick(sensors, {})
        self.assertIn('h_lsb_burst_alert', obs)
        self.assertIn('TIER-2', obs['h_lsb_burst_alert'])

        # Check ALC alert log and priority queue
        tier2_alerts = [a for a in wl._ALC_SHARED['alert_log'] if a.get('hyp_id') == 'H-LSB-1']
        self.assertTrue(len(tier2_alerts) > 0)
        self.assertEqual(tier2_alerts[-1]['tier'], 2)

        priority_items = [q for q in wl._ALC_SHARED['priority_hyp_queue'] if q.get('hyp_id') == 'H-LSB-1']
        self.assertTrue(len(priority_items) > 0)


if __name__ == '__main__':
    unittest.main()
