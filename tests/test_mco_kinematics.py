#!/usr/bin/env python3
"""
tests/test_mco_kinematics.py
============================
Comprehensive unit and integration tests for `firmware/mco_kinematics.py`
and its 50 Hz closed-loop wiring in `firmware/mco_main.py` (`MCOHardware`).

Verifies:
  1. 4-station metachronal phase progression (`90°` quarter-wave lag across W1..W4)
     and bilateral port/starboard phase symmetry across all 8 wings.
  2. 90° pitch-feathering phase advance (`cos(phase)` coupling between downstroke
     velocity and nose-down propulsive pitch angle, passing through trim at reversal).
  3. Roll, pitch, and yaw control allocation mixer gains and saturation clamps
     (`[0.0, MAX_WING_STROKE_DEG]`, `[-45.0, +45.0]` pitch, `[15.0, 165.0]` sweep).
  4. Reciprocating ODrive capstan position (`motor_pos_turns`) and velocity
     (`motor_vel_rps`) synthesis using `MCOConfig.GEAR_RATIO` (`10.0:1`).
  5. `MCOHardware.apply_commands()` and `get_wing_kinematics()` integration with
     `MetachronalSynthesizer`.
"""

from __future__ import annotations

import math
import sys
import unittest
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "firmware"))
sys.path.insert(0, str(REPO_ROOT))

import mco_config as cfg
from mco_kinematics import (
    ControlAllocationMixer,
    MetachronalPhaseOscillator,
    MetachronalSynthesizer,
    WingPairKinematics,
)
from mco_main import MCOHardware


class TestMetachronalPhaseOscillator(unittest.TestCase):
    """Tests 4-station metachronal phase progression and 8-wing mapping."""

    def test_quarter_wave_90deg_station_lag(self) -> None:
        osc = MetachronalPhaseOscillator(cascade_phase_deg=90.0, initial_phase_rad=0.0)
        station_phases = osc.get_station_phases_rad()
        self.assertEqual(len(station_phases), 4)
        expected = [
            0.0,
            (0.0 - math.pi / 2.0) % (2.0 * math.pi),
            (0.0 - math.pi) % (2.0 * math.pi),
            (0.0 - 3.0 * math.pi / 2.0) % (2.0 * math.pi),
        ]
        for actual, exp in zip(station_phases, expected):
            self.assertAlmostEqual(actual, exp, places=6)

        # Verify consecutive stations lag by exactly pi/2 rad (90 deg)
        for k in range(3):
            diff = (station_phases[k] - station_phases[k + 1]) % (2.0 * math.pi)
            self.assertAlmostEqual(diff, math.pi / 2.0, places=6)

    def test_wing_phase_mapping_port_starboard_symmetry(self) -> None:
        osc = MetachronalPhaseOscillator(cascade_phase_deg=90.0, initial_phase_rad=0.8)
        station_phases = osc.get_station_phases_rad()
        wing_phases = osc.get_wing_phases_rad()
        self.assertEqual(len(wing_phases), 8)
        for k in range(4):
            self.assertAlmostEqual(wing_phases[2 * k], station_phases[k], places=6)
            self.assertAlmostEqual(wing_phases[2 * k + 1], station_phases[k], places=6)

    def test_phase_advance_over_one_full_period(self) -> None:
        osc = MetachronalPhaseOscillator(cascade_phase_deg=90.0, initial_phase_rad=0.0)
        freq_hz = 2.0
        # 25 steps of 20 ms = 0.5 s = 1 full cycle at 2.0 Hz
        for _ in range(25):
            osc.advance(dt_s=0.02, freq_hz=freq_hz)
        # Master phase wraps back to 0.0 (mod 2*pi)
        self.assertAlmostEqual(osc.phi_0 % (2.0 * math.pi), 0.0, places=5)


class TestPitchFeatheringPhaseAdvance(unittest.TestCase):
    """Tests 90° phase-advanced pitch feathering relative to flapping stroke."""

    def test_cosine_velocity_and_pitch_feathering_coupling(self) -> None:
        synth = MetachronalSynthesizer(cascade_phase_deg=90.0, initial_phase_rad=0.0)

        # Step with dt_s=0.0 so master phase phi_0 remains exactly at cardinal angles
        cmd = {
            "freq_hz": 2.0,
            "stroke_amp_deg": 28.0,
            "aoa_cmd_deg": 4.0,
            "pitch_feather_amp_deg": 10.0,
            "sweep_cmd_deg": 60.0,
        }

        # 1. phi = 0.0 rad: mid-downstroke (flap = 0°, peak positive velocity cos(0) = +1)
        synth.reset(phi_0=0.0)
        frame_0 = synth.step(dt_s=0.0, commands=cmd, sticks={"roll": 0.0, "pitch": 0.0, "yaw": 0.0, "throttle": 0.5})
        self.assertAlmostEqual(frame_0["flap_angle_deg"][0], 0.0, places=5)
        self.assertEqual(frame_0["stroke_dir"][0], 1)
        self.assertGreater(frame_0["motor_vel_rps"][0], 0.0)
        # Pitch is nose-down (trim - feather_amp = 4.0 - 10.0 = -6.0°) for forward thrust
        self.assertAlmostEqual(frame_0["pitch_servo_deg"][0], -6.0, places=5)

        # 2. phi = pi/2 rad: bottom/top stroke reversal (flap = +28°, velocity cos(pi/2) = 0)
        synth.reset(phi_0=math.pi / 2.0)
        frame_90 = synth.step(dt_s=0.0, commands=cmd, sticks={"roll": 0.0, "pitch": 0.0, "yaw": 0.0, "throttle": 0.5})
        self.assertAlmostEqual(frame_90["flap_angle_deg"][0], 28.0, places=5)
        self.assertAlmostEqual(frame_90["motor_vel_rps"][0], 0.0, places=5)
        # At stroke reversal, pitch feathering passes right through trim (+4.0°)
        self.assertAlmostEqual(frame_90["pitch_servo_deg"][0], 4.0, places=5)

        # 3. phi = pi rad: mid-upstroke (flap = 0°, peak negative velocity cos(pi) = -1)
        synth.reset(phi_0=math.pi)
        frame_180 = synth.step(dt_s=0.0, commands=cmd, sticks={"roll": 0.0, "pitch": 0.0, "yaw": 0.0, "throttle": 0.5})
        self.assertAlmostEqual(frame_180["flap_angle_deg"][0], 0.0, places=5)
        self.assertEqual(frame_180["stroke_dir"][0], -1)
        self.assertLess(frame_180["motor_vel_rps"][0], 0.0)
        # Pitch feathers nose-up (trim + feather_amp = 4.0 + 10.0 = +14.0°)
        self.assertAlmostEqual(frame_180["pitch_servo_deg"][0], 14.0, places=5)

    def test_odrive_reciprocating_turns_and_velocity_scaling(self) -> None:
        kin = WingPairKinematics()
        mixer = ControlAllocationMixer()
        alloc = mixer.allocate(
            roll=0.0,
            pitch=0.0,
            yaw=0.0,
            throttle=0.5,
            freq_hz=2.0,
            stroke_amp_deg=36.0,
        )
        # At phase = pi/2, flap_angle = 36.0 deg -> motor_pos_turns = (36 / 360) * 10.0 = 1.0 turn
        frame_peak = kin.evaluate([math.pi / 2.0] * 8, freq_hz=2.0, allocation=alloc)
        self.assertAlmostEqual(frame_peak["motor_pos_turns"][0], (36.0 / 360.0) * cfg.GEAR_RATIO, places=6)
        self.assertAlmostEqual(frame_peak["motor_vel_rps"][0], 0.0, places=6)

        # At phase = 0.0, cos(0) = 1.0 -> motor_vel_rps = 2*pi * 2.0 * (36/360) * 10.0 = 4*pi rps
        frame_mid = kin.evaluate([0.0] * 8, freq_hz=2.0, allocation=alloc)
        expected_vel = 2.0 * math.pi * 2.0 * (36.0 / 360.0) * cfg.GEAR_RATIO
        self.assertAlmostEqual(frame_mid["motor_vel_rps"][0], expected_vel, places=6)


class TestControlAllocationMixer(unittest.TestCase):
    """Tests roll, pitch, and yaw control allocation and saturation clamps."""

    def setUp(self) -> None:
        self.mixer = ControlAllocationMixer()

    def test_roll_differential_stroke_allocation(self) -> None:
        # Positive roll (+1.0): port wings decrease stroke by 8.0°, starboard increase by 8.0°
        alloc = self.mixer.allocate(roll=1.0, pitch=0.0, yaw=0.0, throttle=0.5, stroke_amp_deg=28.0)
        for i in range(8):
            if i % 2 == 0:  # Port
                self.assertAlmostEqual(alloc["stroke_half_angle_deg"][i], 20.0, places=5)
            else:  # Starboard
                self.assertAlmostEqual(alloc["stroke_half_angle_deg"][i], 36.0, places=5)

    def test_longitudinal_pitch_fore_aft_allocation(self) -> None:
        # Positive pitch (+1.0): fore pairs W1/W2 (0..3) get +6.0° pitch trim and -5.0° forward sweep bias;
        # aft pairs W3/W4 (4..7) get -6.0° pitch trim and +5.0° aft sweep bias.
        alloc = self.mixer.allocate(
            roll=0.0,
            pitch=1.0,
            yaw=0.0,
            throttle=0.5,
            aoa_cmd_deg=4.0,
            sweep_cmd_deg=60.0,
        )
        for i in range(4):  # Fore pairs W1, W2
            self.assertAlmostEqual(alloc["pitch_trim_deg"][i], 10.0, places=5)
            self.assertAlmostEqual(alloc["sweep_deg"][i], 55.0, places=5)
        for i in range(4, 8):  # Aft pairs W3, W4
            self.assertAlmostEqual(alloc["pitch_trim_deg"][i], -2.0, places=5)
            self.assertAlmostEqual(alloc["sweep_deg"][i], 65.0, places=5)

    def test_yaw_differential_pitch_and_sweep_allocation(self) -> None:
        # Positive yaw (+1.0): port gets +4.0° pitch trim and -6.0° sweep; starboard gets -4.0° pitch and +6.0° sweep
        alloc = self.mixer.allocate(
            roll=0.0,
            pitch=0.0,
            yaw=1.0,
            throttle=0.5,
            aoa_cmd_deg=4.0,
            sweep_cmd_deg=60.0,
        )
        for i in range(8):
            if i % 2 == 0:  # Port
                self.assertAlmostEqual(alloc["pitch_trim_deg"][i], 8.0, places=5)
                self.assertAlmostEqual(alloc["sweep_deg"][i], 54.0, places=5)
            else:  # Starboard
                self.assertAlmostEqual(alloc["pitch_trim_deg"][i], 0.0, places=5)
                self.assertAlmostEqual(alloc["sweep_deg"][i], 66.0, places=5)

    def test_saturation_clamps(self) -> None:
        # Extreme inputs must clamp to [0.0, MAX_WING_STROKE_DEG], [-45.0, 45.0], and [15.0, 165.0]
        alloc_high = self.mixer.allocate(
            roll=1.0,
            pitch=1.0,
            yaw=1.0,
            throttle=1.0,
            freq_hz=99.0,
            stroke_amp_deg=85.0,
            aoa_cmd_deg=50.0,
            sweep_cmd_deg=180.0,
        )
        self.assertAlmostEqual(alloc_high["freq_hz"], cfg.MAX_FLAP_FREQ_HZ, places=5)
        for i in range(8):
            self.assertLessEqual(alloc_high["stroke_half_angle_deg"][i], cfg.MAX_WING_STROKE_DEG)
            self.assertGreaterEqual(alloc_high["stroke_half_angle_deg"][i], 0.0)
            self.assertLessEqual(alloc_high["pitch_trim_deg"][i], cfg.SERVO_TRAVEL_DEG / 2.0)
            self.assertGreaterEqual(alloc_high["pitch_trim_deg"][i], -cfg.SERVO_TRAVEL_DEG / 2.0)
            self.assertLessEqual(alloc_high["sweep_deg"][i], 165.0)
            self.assertGreaterEqual(alloc_high["sweep_deg"][i], 15.0)

        alloc_low = self.mixer.allocate(
            roll=1.0,
            pitch=-1.0,
            yaw=-1.0,
            throttle=0.5,
            stroke_amp_deg=3.0,
            aoa_cmd_deg=-50.0,
            sweep_cmd_deg=0.0,
        )
        # Port stroke (3.0 - 8.0 = -5.0) must clamp at 0.0, and sweep must clamp at 15.0
        self.assertEqual(alloc_low["stroke_half_angle_deg"][0], 0.0)
        self.assertEqual(alloc_low["sweep_deg"][0], 15.0)
        self.assertEqual(alloc_low["sweep_deg"][5], 15.0)


class TestMCOHardwareKinematicsIntegration(unittest.TestCase):
    """Tests MCOHardware.apply_commands() integration with MetachronalSynthesizer."""

    def test_hardware_apply_commands_updates_synthesizer_and_wing_kinematics(self) -> None:
        hw = MCOHardware(sim_mode=True)
        self.assertIsInstance(hw.kinematics, MetachronalSynthesizer)

        wing_cmds = [
            {"flap_amp_deg": 28.0, "pitch_deg": 5.0, "sweep_deg": 62.0}
            for _ in range(8)
        ]
        result = {"wing_commands": wing_cmds, "freq_hz": 2.0}
        sensors = {"dt_s": 0.02, "h_agl_m": 40.0}

        hw.apply_commands(result, sensors)
        kin = hw.get_wing_kinematics()

        self.assertEqual(len(kin["flap_degs"]), 8)
        self.assertEqual(len(kin["flap_phases"]), 8)
        self.assertEqual(len(kin["stroke_dirs"]), 8)
        self.assertEqual(len(kin["pitch_servo_degs"]), 8)
        self.assertEqual(len(kin["sweep_servo_degs"]), 8)
        self.assertEqual(len(kin["motor_pos_turns"]), 8)
        self.assertEqual(len(kin["motor_vel_rps"]), 8)

        # Verify 4 tandem pairs have distinct phases separated by quarter-wave (0.25 normalized cycle)
        p0 = kin["flap_phases"][0]
        p1 = kin["flap_phases"][2]
        p2 = kin["flap_phases"][4]
        p3 = kin["flap_phases"][6]
        self.assertAlmostEqual((p0 - p1) % 1.0, 0.25, places=3)
        self.assertAlmostEqual((p1 - p2) % 1.0, 0.25, places=3)
        self.assertAlmostEqual((p2 - p3) % 1.0, 0.25, places=3)

        # Verify per-wing command dicts are enriched with synthesized actuator targets
        for i in range(8):
            self.assertIn("pitch_servo_deg", result["wing_commands"][i])
            self.assertIn("sweep_servo_deg", result["wing_commands"][i])
            self.assertIn("motor_pos_turns", result["wing_commands"][i])
            self.assertIn("motor_vel_rps", result["wing_commands"][i])


if __name__ == "__main__":
    unittest.main()
