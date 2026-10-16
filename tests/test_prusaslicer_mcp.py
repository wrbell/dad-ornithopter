#!/usr/bin/env python3
"""
tests/test_prusaslicer_mcp.py
=============================
Unit and integration tests for the macOS PrusaSlicer 2.9.6 MCP Server
(`scripts/mcp/prusaslicer_mcp_server.py`) and `./mco` workflow tooling.
"""

import os
import tempfile
import unittest
from pathlib import Path

from scripts.mcp import prusaslicer_mcp_server as ps

try:
    ps.find_prusaslicer_bin()
    HAS_PRUSASLICER = True
except FileNotFoundError:
    HAS_PRUSASLICER = False


@unittest.skipUnless(HAS_PRUSASLICER, "PrusaSlicer CLI binary required for MCP tests")
class TestPrusaSlicerMCP(unittest.TestCase):
    def test_01_prusaslicer_version_and_status(self):
        status = ps.prusaslicer_version_and_status()
        self.assertEqual(status["status"], "ok")
        self.assertIn("PrusaSlicer", status["version_banner"])
        self.assertTrue(os.path.isfile(status["binary_path"]))
        self.assertGreaterEqual(len(status["mco_repo_profiles"]), 4)
        self.assertIn("pa12cf_structural", status["profile_aliases"])
        self.assertIn("pa12cf_solid_gears", status["profile_aliases"])
        self.assertIn("lwpla_aero_fairing", status["profile_aliases"])

    def test_02_list_printer_and_filament_profiles(self):
        profiles = ps.list_printer_and_filament_profiles("COREONE")
        self.assertEqual(profiles["filter"], "COREONE")
        self.assertGreaterEqual(len(profiles["mco_repo_profiles"]), 4)

    def test_03_inspect_model_capstan_pinion(self):
        info = ps.inspect_model("cad/print_package_wing_root/05_v10_stepped_capstan_pinion.stl")
        self.assertTrue(info["is_manifold"])
        self.assertEqual(info["number_of_parts"], 1)
        self.assertAlmostEqual(info["dimensions_mm"]["x"], 15.0, places=1)
        self.assertAlmostEqual(info["dimensions_mm"]["z"], 44.0, places=1)
        self.assertGreater(info["volume_cm3"], 2.0)
        self.assertTrue(info["bed_check"]["fits_on_bed"])

    def test_04_slice_model_and_parse_gcode_telemetry(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            res = ps.slice_model(
                input_paths=["cad/print_package_wing_root/05_v10_stepped_capstan_pinion.stl"],
                output_dir=tmpdir,
                profile="pa12cf_solid_gears",
                export_3mf=True,
            )
            self.assertEqual(res["status"], "sliced")
            self.assertTrue(Path(res["gcode_path"]).exists())
            self.assertTrue(Path(res["project_3mf_path"]).exists())

            tel = res["telemetry"]
            self.assertGreater(tel["filament_used_g"], 2.0)
            self.assertGreater(tel["filament_used_mm"], 500.0)
            self.assertGreater(tel["estimated_printing_time_seconds"], 600)
            self.assertGreater(tel["layer_count"], 100)
            self.assertEqual(tel["nozzle_temperature_c"], 290.0)
            self.assertEqual(tel["bed_temperature_c"], 105.0)

    def test_05_transform_or_pack_models(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            out_stl = os.path.join(tmpdir, "duplicated_pinion.stl")
            res = ps.transform_or_pack_models(
                input_paths=["cad/print_package_wing_root/05_v10_stepped_capstan_pinion.stl"],
                output_path=out_stl,
                duplicate=2,
            )
            self.assertEqual(res["status"], "exported")
            self.assertTrue(os.path.exists(out_stl))
            self.assertTrue(res["info"]["is_manifold"])
            self.assertEqual(res["info"]["number_of_parts"], 2)

    def test_06_validate_mco_print_package(self):
        report = ps.validate_mco_print_package("cad/print_package_wing_root")
        self.assertEqual(report["total_stl_files"], 9)
        self.assertTrue(report["all_manifold"])
        self.assertTrue(report["all_fit_bed"])


if __name__ == "__main__":
    unittest.main()
