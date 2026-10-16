#!/usr/bin/env python3
"""
tests/test_physical_discovery.py
================================
Unit tests for the MCO-1D Autonomous Physical Plant Self-Discovery Engine.
"""

import unittest
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'firmware'))

from mco_physical_discovery import (
    AutonomousPhysicalDiscoveryEngine,
    PhysicalPlantManifest,
    load_calibrated_profile,
)


class TestPhysicalDiscovery(unittest.TestCase):
    """Tests autonomous discovery routines, gate checks, and discrepancy detection."""

    def setUp(self):
        self.engine = AutonomousPhysicalDiscoveryEngine(sim_mode=True)

    def test_transmission_ratio_discovery(self):
        """Verifies 10:1 gear ratio discovery across all 8 drives."""
        ratios = self.engine.discover_capstan_gear_ratios()
        self.assertEqual(len(ratios), 8)
        for r in ratios:
            self.assertAlmostEqual(r, 10.0, delta=0.15)

    def test_cable_tension_compliance(self):
        """Verifies Dyneema cable tension discovery around 180N baseline."""
        stiffness, tensions = self.engine.discover_cable_tension_compliance()
        self.assertEqual(len(tensions), 8)
        for t in tensions:
            self.assertGreaterEqual(t, 160.0)
            self.assertLessEqual(t, 200.0)

    def test_wing_inertia_discovery(self):
        """Verifies wing rotational inertia identification."""
        inertias, frictions = self.engine.discover_wing_inertias()
        self.assertEqual(len(inertias), 8)
        self.assertAlmostEqual(inertias[0], 0.0206, delta=0.003)

    def test_reconciliation_pass(self):
        """Verifies nominal build passes all discovery gates."""
        manifest = PhysicalPlantManifest()
        res = self.engine.run_full_discovery_phase(manifest)
        self.assertTrue(res['all_gates_passed'])
        self.assertEqual(res['discrepancy_count'], 0)

    def test_discrepancy_detection(self):
        """Verifies engine flags critical discrepancies if physical hardware deviates."""
        bad_manifest = PhysicalPlantManifest(
            capstan_nominal_ratio=7.0,      # Wrong ratio!
            nominal_cable_tension_n=250.0,   # Way too tight!
            station_stagger_mm=100.0,       # Dangerous clash spacing!
        )
        self.engine.discover_capstan_gear_ratios()
        self.engine.discover_cable_tension_compliance()
        self.engine.discover_wing_inertias()
        self.engine.discover_servo_travel_limits()
        self.engine.discover_cascade_kinematic_clearance(stagger_mm=100.0, chord_mm=165.0)

        res = self.engine.cross_check_and_reconcile(bad_manifest)
        self.assertFalse(res['all_gates_passed'])
        self.assertGreater(res['discrepancy_count'], 0)
        discrepancy_text = " ".join(res['discrepancies'])
        self.assertIn("CRITICAL", discrepancy_text)
        self.assertIn("WARNING", discrepancy_text)


if __name__ == '__main__':
    unittest.main()
