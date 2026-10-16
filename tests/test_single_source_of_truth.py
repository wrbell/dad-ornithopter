#!/usr/bin/env python3
"""
tests/test_single_source_of_truth.py
====================================
Verifies that `config/airframe_spec.json`, `firmware/mco_config.py` (`MCOConfig`),
`bom/bom.csv`, and all active firmware modules share a single, non-contradictory
set of physical constants, GPIO pin assignments, and drivetrain parameters.
"""

from __future__ import annotations

import csv
import json
import sys
import unittest
from pathlib import Path
from unittest.mock import patch

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "firmware"))
sys.path.insert(0, str(REPO_ROOT))

import mco_config as cfg
from mco_config import MCOConfig


class TestSingleSourceOfTruth(unittest.TestCase):
    def setUp(self) -> None:
        self.spec_path = REPO_ROOT / "config" / "airframe_spec.json"
        self.bom_csv_path = REPO_ROOT / "bom" / "bom.csv"
        self.spec = json.loads(self.spec_path.read_text(encoding="utf-8"))

    def test_airframe_spec_matches_mco_config(self) -> None:
        self.assertAlmostEqual(MCOConfig.MASS_KG, self.spec["mass_and_balance"]["design_installed_mass_kg"], places=6)
        self.assertAlmostEqual(MCOConfig.INSTALLED_MASS_G, self.spec["mass_and_balance"]["design_installed_mass_g"], places=3)
        self.assertAlmostEqual(MCOConfig.MTOW_LIMIT_KG, self.spec["mass_and_balance"]["max_takeoff_mass_limit_kg"], places=6)
        self.assertAlmostEqual(MCOConfig.WINGSPAN_M, self.spec["geometry"]["tip_to_tip_wingspan_m"], places=6)
        self.assertAlmostEqual(MCOConfig.CHORD_M, self.spec["geometry"]["chord_m"], places=6)
        self.assertAlmostEqual(MCOConfig.WING_AREA_TOTAL_M2, self.spec["geometry"]["total_wing_area_m2"], places=6)
        self.assertAlmostEqual(MCOConfig.STAGGER_MM, self.spec["geometry"]["tandem_stagger_mm"], places=6)
        self.assertAlmostEqual(MCOConfig.GEAR_RATIO, self.spec["propulsion"]["capstan_reduction_ratio"], places=6)
        self.assertEqual(MCOConfig.MOTOR_KV, self.spec["propulsion"]["motor_kv"])
        self.assertEqual(MCOConfig.MOTOR_POLE_PAIRS, self.spec["propulsion"]["motor_pole_pairs"])

    def test_gpio_pins_are_disjoint(self) -> None:
        self.assertEqual(len(MCOConfig.PITCH_PINS), 8)
        self.assertEqual(len(MCOConfig.SWEEP_PINS), 8)
        overlap = set(MCOConfig.PITCH_PINS) & set(MCOConfig.SWEEP_PINS)
        self.assertEqual(overlap, set(), f"GPIO collision between PITCH_PINS and SWEEP_PINS: {overlap}")

    def test_bom_csv_exists_and_matches_bom_tool_installed_mass(self) -> None:
        from scripts import mco_bom_tool

        self.assertTrue(self.bom_csv_path.exists(), "bom/bom.csv must exist")
        with self.bom_csv_path.open("r", encoding="utf-8") as f:
            rows = list(csv.DictReader(f))
        self.assertEqual(len(rows), len(mco_bom_tool.BOM_DATABASE))
        summary = mco_bom_tool.compute_bom_summary()
        self.assertAlmostEqual(summary["installed_total_mass_g"], self.spec["mass_and_balance"]["design_installed_mass_g"], places=1)
        self.assertLess(summary["installed_total_mass_kg"], self.spec["mass_and_balance"]["max_takeoff_mass_limit_kg"])

    def test_mco_main_uses_ssot_constants_and_disjoint_pins(self) -> None:
        import mco_main

        self.assertEqual(list(mco_main.PITCH_PINS), list(MCOConfig.PITCH_PINS))
        self.assertEqual(list(mco_main.SWEEP_PINS), list(MCOConfig.SWEEP_PINS))
        self.assertEqual(set(mco_main.PITCH_PINS) & set(mco_main.SWEEP_PINS), set())
        self.assertAlmostEqual(float(mco_main.REDUCER_RATIO), cfg.GEAR_RATIO, places=6)

    def test_odrive_interface_uses_reciprocating_velocity_not_unidirectional_runaway(self) -> None:
        import mco_odrive_interface as od

        self.assertAlmostEqual(od.GEAR_RATIO, 10.0, places=6)
        s1 = od.ODriveInterface()
        recorded: list[float] = []
        with patch.object(s1, "set_velocity", side_effect=lambda w, v, torque_ff=0.0: recorded.append(v)):
            with patch("time.monotonic", return_value=s1._osc_t0):
                s1.set_flap_freq(0, freq_hz=2.0, phase_offset_deg=0.0)
                s1.set_flap_freq(0, freq_hz=2.0, phase_offset_deg=90.0)
                s1.set_flap_freq(0, freq_hz=2.0, phase_offset_deg=180.0)
        self.assertEqual(len(recorded), 3)
        self.assertGreater(recorded[0], 0.0)
        self.assertAlmostEqual(recorded[1], 0.0, places=5)
        self.assertAlmostEqual(recorded[2], -recorded[0], places=5)

    def test_secondary_firmware_modules_match_ssot(self) -> None:
        import mco_cg_estimator
        import mco_odrive_setup
        import mco_performance_model
        import mco_rc_verify
        import mco_taxi_park

        self.assertEqual(mco_odrive_setup.POLE_PAIRS, cfg.MOTOR_POLE_PAIRS)
        self.assertEqual(mco_odrive_setup.MOTOR_KV, cfg.MOTOR_KV)
        self.assertAlmostEqual(mco_performance_model.MTOW_KG, cfg.DESIGN_MASS_KG, places=6)
        self.assertAlmostEqual(mco_performance_model.S_TOTAL_M2, cfg.TOTAL_WING_AREA_M2, places=6)
        self.assertAlmostEqual(mco_cg_estimator.MAC_MM, cfg.MAC_MM, places=3)
        self.assertAlmostEqual(mco_taxi_park.B_W4_MM, cfg.WING_SPAN_MM, places=3)


if __name__ == "__main__":
    unittest.main()
