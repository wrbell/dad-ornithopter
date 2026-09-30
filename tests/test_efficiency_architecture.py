#!/usr/bin/env python3
"""
Test Suite: Unified Multi-Regime Flight Efficiency Architecture
===============================================================
Validates:
  1. Universal Always-On Baseline: H-SERVO-1, H-DIST-1, H-SWEEP-1, H-DUTY-1, H-SHUTTLE-1.
  2. Cruise / Loiter Maximum Endurance Specialization (H-GLIDER-1, H-MICRO-1, H-PULSE-1).
  3. Maximum-Efficiency Climb Regime (metachronal downwash harvesting, pitch trim feedforward).
  4. Maximum-Efficiency Sprint Regime (progressive wing stowing, Strouhal-governed micro-strokes).
  5. Wave Function Lever Continuum: full 0% to 100% lever efficiency progression.
  6. Performance Model Comparative Accounting.
  7. MCOFlightController 50 Hz loop integration.
  8. Ground Station Telemetry diagnostics.
"""

import unittest
import math
import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'firmware'))

import mco_wave_library as w
from mco_flight_governor import (
    UnifiedEfficiencySupervisor,
    govern_flight_efficiency,
    compute_h_dist_pitch_compensation,
    compute_h_duty_asymmetric_profile,
    compute_h_sweep_upstroke_trajectory,
    _servo_tick,
    _GLOBAL_SERVO_TRACKER,
)
from mco_performance_model import PerformanceModel


class TestUniversalAlwaysOnBaseline(unittest.TestCase):
    """Verifies that all 5 baseline efficiency strategies operate continuously."""

    def test_servo_twist_unloading(self):
        st = _servo_tick({'servo_rail_v': 7.4, 'servo_rail_a': 2.7, 'passive_twist_deg': 4.0})
        self.assertIn('power_savings_w', st)
        self.assertGreaterEqual(st['power_savings_w'], 30.0)

    def test_streamwise_distribution_compensation(self):
        comp = compute_h_dist_pitch_compensation(front_bias=1.15, base_aoa_deg=14.0)
        self.assertTrue(comp['longitudinal_balanced'])
        self.assertGreater(comp['pitch_trim_feedforward_deg'], 0.0)
        self.assertLess(comp['aft_aoa_compensated_deg'], 14.0)

    def test_upstroke_sweep_hardware_clamp(self):
        traj_up = compute_h_sweep_upstroke_trajectory(t_sec=0.25, f_hz=2.0, flap_amp_deg=90.0, sweep_target_deg=35.0)
        self.assertLessEqual(traj_up['sweep_deg'], 12.0)
        self.assertTrue(traj_up['clamp_enforced'])

    def test_asymmetric_duty_cycle_current_protection(self):
        profile = compute_h_duty_asymmetric_profile(t_sec=0.1, f_hz=2.0, flap_amp_deg=90.0, downstroke_pct=55.0)
        self.assertTrue(profile['current_limit_35a_safe'])
        self.assertLess(profile['est_motor_current_a'], 35.0)
        self.assertGreaterEqual(profile['duty_downstroke_pct'], 52.0)
        self.assertLessEqual(profile['duty_downstroke_pct'], 57.0)

    def test_supervisor_baseline_inclusion(self):
        sup = UnifiedEfficiencySupervisor()
        wing_cmds = [{'sweep_deg': 60.0, 'flap_deg': 40.0} for _ in range(8)]
        res = sup.govern_efficiency(
            intent='cruise', lever_pct=50.0, V_ms=9.0, freq_hz=2.0, flap_amp_deg=70.0,
            wing_commands=wing_cmds, sensors={'bus_voltage_v': 24.0, 'bus_ripple_v': 0.5}
        )
        for expected_strat in ['H-SERVO-1', 'H-DIST-1', 'H-SWEEP-1', 'H-DUTY-1', 'H-SHUTTLE-1']:
            self.assertIn(expected_strat, res['active_strategies'])
        self.assertGreaterEqual(res['power_savings_w'], 70.0)


class TestRegimeSpecificEfficiency(unittest.TestCase):
    """Verifies that climb, sprint, and cruise activate their optimal efficiency strategies."""

    def setUp(self):
        self.sup = UnifiedEfficiencySupervisor()

    def test_cruise_motorglider_loiter(self):
        wing_cmds = [{'sweep_deg': 60.0, 'flap_deg': 40.0} for _ in range(8)]
        res = self.sup.govern_efficiency(
            intent='cruise', lever_pct=55.0, V_ms=10.5, freq_hz=1.8, flap_amp_deg=60.0,
            wing_commands=wing_cmds, sensors={'motorglider_eligible': True}
        )
        self.assertEqual(res['efficiency_regime'], 'CRUISE')
        self.assertIn('H-GLIDER-1_SUSTAINER', res['active_strategies'])
        self.assertIn('H-MICRO-1', res['active_strategies'])
        self.assertGreaterEqual(res['power_savings_w'], 220.0)

    def test_maximum_efficiency_climb(self):
        wing_cmds = [{'sweep_deg': 60.0, 'flap_deg': 80.0} for _ in range(8)]
        res = self.sup.govern_efficiency(
            intent='climb', lever_pct=75.0, V_ms=9.0, freq_hz=2.6, flap_amp_deg=80.0,
            wing_commands=wing_cmds, sensors={'climb_rate_ms': 1.8}
        )
        self.assertEqual(res['efficiency_regime'], 'CLIMB')
        self.assertIn('H-65_DOWNWASH_HARVESTING', res['active_strategies'])
        self.assertIn('H-DIST-1_CLIMB_TRIM', res['active_strategies'])
        self.assertGreater(res['climb_efficiency_j_per_m'], 0.0)
        self.assertLessEqual(res['climb_efficiency_j_per_m'], 240.0)

    def test_maximum_efficiency_sprint(self):
        wing_cmds = [{'sweep_deg': 60.0, 'flap_deg': 90.0} for _ in range(8)]
        res = self.sup.govern_efficiency(
            intent='sprint', lever_pct=95.0, V_ms=15.0, freq_hz=3.5, flap_amp_deg=90.0,
            wing_commands=wing_cmds, sensors={}
        )
        self.assertEqual(res['efficiency_regime'], 'SPRINT')
        self.assertIn('H-MICRO-1_STROUHAL', res['active_strategies'])
        self.assertIn('PROGRESSIVE_WING_STOW', res['active_strategies'])
        self.assertGreater(res['parasite_drag_reduction_pct'], 30.0)
        # Verify rear pairs stowed
        self.assertTrue(wing_cmds[4].get('stowed', False))
        self.assertTrue(wing_cmds[7].get('stowed', False))


class TestWaveFunctionLeverContinuum(unittest.TestCase):
    """Verifies that the entire 0% to 100% lever carries efficiency strategies."""

    def test_lever_sample_points(self):
        test_points = [
            (5.0,   'DESCENT_BRAKE', 'EMERGENCY_STOP'),
            (20.0,  'DESCENT_BRAKE', 'DESCENT'),
            (35.0,  'DESCENT_BRAKE', 'GLIDE_SOAR'),
            (42.0,  'DESCENT_BRAKE', 'APPROACH'),
            (50.0,  'CRUISE',        'TRIM_DETENT'),
            (60.0,  'CRUISE',        'CRUISE'),
            (72.0,  'CLIMB',         'POWERED_CLIMB'),
            (82.0,  'CLIMB',         'HIGH_THRUST'),
            (95.0,  'SPRINT',        'MAX_THRUST'),
            (100.0, 'SPRINT',        'MAX_THRUST'),
        ]

        for lev_pct, expected_regime, expected_zone in test_points:
            params = w.unified_lever_to_params(lev_pct, V_ms=10.0, h_agl_m=100.0)
            self.assertEqual(params['zone'], expected_zone, f"Zone mismatch at {lev_pct}%")
            self.assertEqual(params['efficiency_regime'], expected_regime, f"Regime mismatch at {lev_pct}%")
            self.assertTrue(params['efficiency_governed'])

            # Verify universal strategies are present on EVERY lever position
            for u_strat in ['H-SERVO-1', 'H-DIST-1', 'H-SWEEP-1', 'H-DUTY-1', 'H-SHUTTLE-1']:
                self.assertIn(u_strat, params['universal_strategies'], f"Missing {u_strat} at {lev_pct}%")
                self.assertIn(u_strat, params['active_efficiency_strategies'], f"Missing {u_strat} in active at {lev_pct}%")

    def test_sprint_strouhal_derating(self):
        # At high airspeed (15 m/s), full throttle lever 95% should derate amplitude to keep Strouhal optimal
        p_fast = w.unified_lever_to_params(95.0, V_ms=15.0)
        self.assertLessEqual(p_fast['amp_port_deg'], 65.0)
        self.assertGreaterEqual(p_fast['strouhal_number'], 0.20)
        self.assertLessEqual(p_fast['strouhal_number'], 0.35)

    def test_climb_downwash_harvesting(self):
        p_climb = w.unified_lever_to_params(75.0, V_ms=9.0)
        self.assertEqual(p_climb['efficiency_regime'], 'CLIMB')
        self.assertGreater(p_climb['climb_efficiency_j_per_m'], 0.0)
        self.assertGreater(p_climb['cascade_phase_deg'], 10.0)
        self.assertLessEqual(p_climb['cascade_phase_deg'], 30.0)


class TestPerformanceModelAccounting(unittest.TestCase):
    """Verifies comparative power and energy accounting in PerformanceModel."""

    def test_unified_efficiency_evaluation(self):
        pm = PerformanceModel()
        eval_res = pm.evaluate_unified_efficiency_architecture()

        self.assertEqual(eval_res['universal_baseline_savings_w'], 91.0)
        self.assertIn('H-SERVO-1', eval_res['universal_strategies'])

        # Climb savings
        self.assertGreaterEqual(eval_res['climb']['energy_savings_pct'], 20.0)
        self.assertLessEqual(eval_res['climb']['optimized_joules_per_m'], 220.0)

        # Sprint savings
        self.assertGreaterEqual(eval_res['sprint']['energy_savings_pct'], 30.0)
        self.assertLess(eval_res['sprint']['strouhal_optimized'], eval_res['sprint']['strouhal_baseline'])

        # Motorglider cruise
        self.assertEqual(eval_res['cruise']['motorglider_power_w'], 125.0)
        self.assertGreaterEqual(eval_res['cruise']['motorglider_endurance_min'], 90.0)


class TestFlightControllerTickIntegration(unittest.TestCase):
    """Verifies that MCOFlightController.tick attaches efficiency telemetry."""

    def test_fcc_tick_efficiency_output(self):
        fcc = w.MCOFlightController()
        sensors = {
            'V_ms': 10.5, 'freq_hz': 2.0, 'h_agl_m': 50.0,
            'lever_pct': 55.0, 'intent': 'cruise',
            'flap_phases': [0.5]*8, 'stroke_dirs': [+1]*8,
            'sweeps_deg': [60.0]*8, 'pitches_deg': [0.0]*8,
            'flap_degs': [30.0]*8, 'bus_voltage_v': 24.0,
        }
        res = fcc.tick(sensors)
        self.assertTrue(res.get('efficiency_governor_active', False))
        self.assertIn('active_strategies', res)
        self.assertIn('power_savings_w', res)
        self.assertGreater(res['power_savings_w'], 50.0)


class TestGroundStationTelemetryOutput(unittest.TestCase):
    """Verifies that ground station diagnostics run and include efficiency markers."""

    def test_efficiency_status_callable(self):
        report = w.efficiency_architecture_status()
        self.assertIn("UNIFIED MULTI-REGIME FLIGHT EFFICIENCY ARCHITECTURE", report)
        self.assertIn("H-SERVO-1", report)
        self.assertIn("H-GLIDER-1", report)
        self.assertIn("Downwash Harvesting", report)
        self.assertIn("Strouhal-Governed Micro-Stroke", report)


if __name__ == '__main__':
    unittest.main()
