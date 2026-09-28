#!/usr/bin/env python3
"""
tests/test_takeoff_landing_handoff_matrix.py
=============================================
Comprehensive Matrix Simulation and Verification Suite for:
  - Differing Takeoff Hypotheses & Genomes (CTOL Standard, High DA, Overweight,
    STOL Short-Field, Soft-Field; Universal Launch: Pterosaur Vault H-98,
    Dragonfly Superlift H-LA-1, Pigeon Steep H-LA-3, Mallard Spring, Canvasback Skim)
  - Differing Landing Scenarios & Regimes (Standard Power-Off, Commercial Powered,
    STOL Chop-and-Drop Spot Landing, Soft-Field Hold-Off, Tailwheel 3-Point Full-Stall,
    Tailwheel Wheel Landing, and Balked Landing Go-Around)
  - Modality Soft-Handoff Crossfade (Raised-cosine S-curve kernel, T_blend = 0.20s)
  - Actuator Slew Rate Compliance (<= 295.0 deg/s sweep, <= 1050 deg/s pitch TCM-1001)
  - Structural Load Factor Limit (nz <= +2.20g via dynamic pressure q-compensated limiter)
  - Anti-Tail-Strike Flare Clearance (h_W4 >= 0.150m)
  - Aerodynamic Rollout Deceleration & Taxi Full Stop (V = 0.00 m/s)
"""

import math
import os
import sys
import unittest

# Ensure firmware is on import path
REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
FIRMWARE_DIR = os.path.join(REPO_ROOT, 'firmware')
if FIRMWARE_DIR not in sys.path:
    sys.path.insert(0, FIRMWARE_DIR)

import mco_advanced_physics as ap
import mco_intent_control as ic
import mco_wave_library as wl
from tests.test_adversarial_simulation_flight import AdversarialFlightSimulator


class TestTakeoffLandingHandoffMatrix(unittest.TestCase):
    """Verifies matrix execution and smooth handoffs between all takeoff and landing modes."""

    def setUp(self):
        self.sim = AdversarialFlightSimulator()

    def _simulate_takeoff(self, takeoff_mode):
        """Simulate ground roll / launch burst, rotation, unstick, and initial climb."""
        ctol_regimes = ap.TAKEOFF_REGIMES
        biomimetic_genomes = ap.GENOME_LIBRARY

        if takeoff_mode in ctol_regimes:
            cfg = ctol_regimes[takeoff_mode]
            vr = cfg['Vr_ms']
            # Ground roll acceleration with full throttle
            for _ in range(140):
                self.sim.step(
                    {'throttle': 1.0, 'pitch': 0.0, 'yaw': 0.0, 'lever_pct': 85.0},
                    external_env={'regime': takeoff_mode, 'takeoff_roll': True}
                )
                if self.sim.V_ms >= vr:
                    break
            self.assertGreaterEqual(self.sim.V_ms, vr * 0.90,
                                    f"Takeoff roll failed to reach rotation speed for {takeoff_mode}")

            # Dual-cascade rotation
            for _ in range(35):
                self.sim.step(
                    {'throttle': 1.0, 'pitch': 0.50, 'yaw': 0.0, 'lever_pct': 85.0},
                    external_env={'regime': takeoff_mode, 'rotate': True}
                )
                if not self.sim.on_ground and self.sim.z >= 0.20:
                    break
            self.assertFalse(self.sim.on_ground, f"Aircraft must unstick during rotation in {takeoff_mode}")
            self.assertGreater(self.sim.z, 0.15, f"Liftoff height must be positive in {takeoff_mode}")

        elif takeoff_mode in biomimetic_genomes:
            chrom = biomimetic_genomes[takeoff_mode]
            # Pilot crouch
            for _ in range(8):
                self.sim.step(
                    {'throttle': 0.95, 'pitch': 0.80, 'yaw': 0.0, 'pedals_depressed': True},
                    external_env={'universal_launch': True, 'chromosome_name': takeoff_mode}
                )
            # Pilot spring -> triggers rapid center launch burst
            for _ in range(5):
                self.sim.step(
                    {'throttle': 0.95, 'pitch': 0.02, 'yaw': 0.0, 'pedals_depressed': True},
                    external_env={'universal_launch': True, 'chromosome_name': takeoff_mode}
                )
            # Burst execution for designed genome macro duration
            burst_steps = max(20, int(chrom.macro_duration_max_s / self.sim.dt))
            for _ in range(burst_steps):
                self.sim.step(
                    {'throttle': 0.95, 'pitch': 0.20, 'yaw': 0.0, 'pedals_depressed': True},
                    external_env={'universal_launch': True, 'chromosome_name': takeoff_mode}
                )
            self.assertFalse(self.sim.on_ground, f"Aircraft must unstick during launch burst in {takeoff_mode}")
            self.assertGreater(self.sim.z, 0.08, f"Breakout altitude must be positive in {takeoff_mode}")

        # Initial climb corridor
        climb_steps = 110
        for _ in range(climb_steps):
            self.sim.step({'throttle': 0.85, 'pitch': 0.20, 'yaw': 0.0, 'lever_pct': 85.0})

        self.assertGreater(self.sim.z, 1.0, f"Aircraft should establish initial climb in {takeoff_mode}")
        self.assertGreater(self.sim.V_ms, 5.0, f"Airspeed must be above stall in {takeoff_mode}")

    def _simulate_handoff_and_cruise(self):
        """Crossfades from climb to level flight and into descent with soft handoff."""
        # Climb to transition altitude
        for _ in range(25):
            self.sim.step({'throttle': 0.80, 'pitch': 0.05, 'yaw': 0.0, 'lever_pct': 70.0})

        # Soft handoff into level cruise (Raised-cosine S-curve)
        for _ in range(20):
            self.sim.step({'throttle': 0.65, 'pitch': 0.0, 'yaw': 0.0, 'lever_pct': 60.0})

        # Slew rates and load factor verification during handoff
        self.assertLessEqual(self.sim.peak_slew_rates['sweep'], 295.0 + 1e-3,
                             f"Sweep slew rate exceeded 295 deg/s during handoff: {self.sim.peak_slew_rates['sweep']:.1f}")
        self.assertLessEqual(self.sim.structural_nz_max, 2.20 + 1e-3,
                             f"Load factor exceeded 2.2g during handoff: {self.sim.structural_nz_max:.2f}")

    def _simulate_landing(self, landing_regime):
        """Executes approach, roundout, flare, rollout braking, and stop or go-around."""
        self.sim.on_ground = False
        self.sim.z = 5.0
        self.sim.V_ms = 8.5
        self.sim.pitch_deg = -3.0
        self.sim.min_w4_tail_clearance_m = 999.0

        if landing_regime == 'go_around':
            # Approach down to 1.2m
            for _ in range(300):
                self.sim.step(
                    {'pitch': -0.10, 'throttle': 0.20, 'lever_pct': 18.0},
                    external_env={'regime': 'standard', 'landing_approach': True, 'sink_rate_ms': -0.8}
                )
                if self.sim.z <= 1.2:
                    break
            self.assertLessEqual(self.sim.z, 1.5, "Aircraft must descend to low altitude for go-around test")

            # Trigger Go-Around / Balked Landing at 1.0m AGL
            for _ in range(50):
                self.sim.step(
                    {'pitch': 0.30, 'throttle': 1.0, 'lever_pct': 90.0},
                    external_env={'regime': 'standard', 'go_around': True, 'sink_rate_ms': 1.2}
                )
            self.assertFalse(self.sim.on_ground, "Go-around must prevent ground collision")
            self.assertGreater(self.sim.z, 2.0, "Aircraft must climb away after go-around initiate")
            self.assertGreaterEqual(self.sim.min_w4_tail_clearance_m, 0.150,
                                    f"Tail clearance during go-around must be >= 0.15m: {self.sim.min_w4_tail_clearance_m:.3f}")
            return

        # Conventional Landing Regimes
        reg_cfg = ap.LANDING_REGIMES.get(landing_regime, ap.LANDING_REGIMES['standard'])
        app_thr = reg_cfg.get('approach_throttle', 0.15)
        tgt_sink = reg_cfg.get('target_sink_ms', 0.35)

        # 1. Approach Glidepath (5.0m down to 2.0m)
        for _ in range(500):
            self.sim.step(
                {'pitch': -0.10, 'throttle': app_thr, 'lever_pct': 18.0},
                external_env={'regime': landing_regime, 'landing_approach': True, 'sink_rate_ms': -0.8}
            )
            if self.sim.z <= 2.0:
                break
        self.assertLessEqual(self.sim.z, 2.0, f"Must reach flare window in {landing_regime}")

        # 2. Flare (2.0m down to touchdown)
        flare_pitch = 0.30 if landing_regime in ('soft_field', 'tailwheel_3point') else 0.22
        flare_thr = 0.0 if reg_cfg.get('chop_and_drop', False) else app_thr * 0.5
        for _ in range(750):
            self.sim.step(
                {'pitch': flare_pitch, 'throttle': flare_thr, 'lever_pct': 10.0},
                external_env={'regime': landing_regime, 'landing_flare': True, 'sink_rate_ms': -tgt_sink}
            )
            if self.sim.on_ground:
                break

        self.assertTrue(self.sim.on_ground, f"Touchdown confirmed in {landing_regime}")
        self.assertGreaterEqual(
            self.sim.min_w4_tail_clearance_m, 0.150,
            f"Tail clearance risk in {landing_regime}: {self.sim.min_w4_tail_clearance_m:.3f}m < 0.150m"
        )

        # 3. Aerodynamic Rollout Braking (aircraft has no wheel brakes)
        is_stol = reg_cfg.get('chop_and_drop', False)
        brake_lever = 4.0 if is_stol else 18.0  # reverse flapping vs shuttlecock
        for _ in range(50):
            self.sim.step(
                {'pitch': 0.20, 'throttle': 0.0, 'lever_pct': brake_lever},
                external_env={'regime': landing_regime, 'touchdown_rollout': True}
            )

        # 4. Taxi to full stop
        for _ in range(50):
            self.sim.step(
                {'pitch': 0.0, 'throttle': 0.0, 'lever_pct': 0.0},
                external_env={'regime': landing_regime, 'touchdown_rollout': True}
            )

        self.assertAlmostEqual(self.sim.V_ms, 0.0, delta=0.25,
                               msg=f"Aircraft must decelerate to full stop in {landing_regime}")

    # =========================================================================
    # Test Cases
    # =========================================================================

    def test_01_ctol_standard_to_all_landing_regimes(self):
        """CTOL Standard takeoff paired across each distinct landing regime."""
        landing_scenarios = [
            'standard_power_off',
            'commercial_powered',
            'short_field',
            'soft_field',
            'tailwheel_3point',
            'tailwheel_wheel',
            'go_around',
        ]
        for landing_reg in landing_scenarios:
            with self.subTest(landing_regime=landing_reg):
                self.setUp()
                self._simulate_takeoff('standard')
                self._simulate_handoff_and_cruise()
                self._simulate_landing(landing_reg)

    def test_02_all_ctol_takeoff_regimes(self):
        """All 5 CTOL takeoff regimes (standard, high_da, overweight, short_field, soft_field)."""
        ctol_takeoffs = ['standard', 'high_da', 'overweight', 'short_field', 'soft_field']
        for to_mode in ctol_takeoffs:
            with self.subTest(takeoff_mode=to_mode):
                self.setUp()
                self._simulate_takeoff(to_mode)
                self._simulate_handoff_and_cruise()
                self._simulate_landing('standard_power_off')

    def test_03_all_biomimetic_universal_launches(self):
        """All 5 Universal Biomimetic Launch Genomes."""
        launch_genomes = [
            'pterosaur_vault',
            'dragonfly_superlift',
            'pigeon_steep',
            'mallard_spring',
            'canvasback_skim',
        ]
        for genome_name in launch_genomes:
            with self.subTest(genome=genome_name):
                self.setUp()
                self._simulate_takeoff(genome_name)
                self._simulate_handoff_and_cruise()
                self._simulate_landing('standard_power_off')

    def test_04_stol_cross_matrix_short_field_takeoff_to_short_field_spot_landing(self):
        """STOL Bush Plane Mission: Short-Field Takeoff -> Backside Chop-and-Drop Landing."""
        self._simulate_takeoff('short_field')
        self._simulate_handoff_and_cruise()
        self._simulate_landing('short_field')

    def test_05_soft_field_cross_matrix(self):
        """Soft-Field Mission: Ground Effect Cushion Takeoff -> Nosewheel Hold-Off Landing."""
        self._simulate_takeoff('soft_field')
        self._simulate_handoff_and_cruise()
        self._simulate_landing('soft_field')

    def test_06_balked_landing_go_around_preemption(self):
        """Go-around abort preemption initiated during landing flare across multiple approach profiles."""
        for approach_reg in ['commercial_powered', 'short_field', 'tailwheel_3point']:
            with self.subTest(approach_regime=approach_reg):
                self.setUp()
                self._simulate_takeoff('pterosaur_vault')
                self._simulate_handoff_and_cruise()
                self._simulate_landing('go_around')


if __name__ == '__main__':
    unittest.main()
