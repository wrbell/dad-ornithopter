#!/usr/bin/env python3
"""
Unit Test Suite: Military Modernization & ACL Integration
=========================================================
Tests the three software-defined defense capabilities for MCO-1D:
  1. Tactical Cursor-on-Target (CoT) Bridge & ATAK Parser (mco_cot_bridge.py)
  2. Tactical 15-State ES-EKF & Wind Vector Estimator (mco_tactical_ekf.py)
  3. Contested-EW Autonomy & Stealth Loiter Behaviours (mco_intent_control.py)
  4. Ground Station Access Control List (ACL) Whitelist & Command Execution

Author: MCO-1D Defense Modernization Initiative (September 2026)
"""

import sys
import os
import math
import time
import unittest

# Ensure firmware is in search path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'firmware'))

import mco_cot_bridge as cot
import mco_tactical_ekf as ekf_mod
import mco_intent_control as ic
import mco_wave_library as w


class TestCoTBridge(unittest.TestCase):
    """Test MIL-STD Cursor-on-Target (CoT) generation, parsing, and bridge lifecycle."""

    def test_cot_xml_generation(self):
        xml_str = cot.generate_cot_xml(
            lat=34.0522, lon=-118.2437, hae_m=125.4, speed_ms=9.2, course_deg=45.0,
            battery_pct=88.0, flight_mode="CRUISE", callsign="MCO-1D-TEST",
            uid="MCO-TEST-UID-01", extra_remarks="STEALTH_ACTIVE"
        )
        self.assertIn("<event", xml_str)
        self.assertIn("version=\"2.0\"", xml_str)
        self.assertIn("type=\"a-f-A-M-F-Q\"", xml_str)
        self.assertIn("uid=\"MCO-TEST-UID-01\"", xml_str)
        self.assertIn("lat=\"34.052200\"", xml_str)
        self.assertIn("lon=\"-118.243700\"", xml_str)
        self.assertIn("hae=\"125.4\"", xml_str)
        self.assertIn("callsign=\"MCO-1D-TEST\"", xml_str)
        self.assertIn("STEALTH_ACTIVE", xml_str)

    def test_cot_xml_parsing(self):
        sample_atak_msg = """<?xml version="1.0" encoding="UTF-8"?>
        <event version="2.0" uid="ATAK-TEAM-LEADER" type="b-m-p-w" how="m-g"
               time="2026-09-17T11:00:00.000Z" start="2026-09-17T11:00:00.000Z" stale="2026-09-17T11:05:00.000Z">
            <point lat="34.123456" lon="-118.654321" hae="250.0" ce="2.0" le="2.0"/>
            <detail>
                <contact callsign="LEADER-1"/>
                <track course="90.0" speed="0.0"/>
                <remarks>WAYPOINT: TARGET_OBSERVATION_POINT</remarks>
            </detail>
        </event>"""
        parsed = cot.parse_cot_xml(sample_atak_msg)
        self.assertIsNotNone(parsed)
        self.assertEqual(parsed["uid"], "ATAK-TEAM-LEADER")
        self.assertEqual(parsed["callsign"], "LEADER-1")
        self.assertAlmostEqual(parsed["lat"], 34.123456, places=5)
        self.assertAlmostEqual(parsed["lon"], -118.654321, places=5)
        self.assertAlmostEqual(parsed["hae_m"], 250.0, places=1)
        self.assertTrue(parsed["is_waypoint"])

    def test_cot_bridge_lifecycle_and_waypoint_queue(self):
        bridge = cot.MCOCoTBridge(enabled=True)
        self.assertFalse(bridge._running)
        bridge.start()
        self.assertTrue(bridge._running)

        # Update telemetry
        bridge.update_telemetry(
            lat=37.7749, lon=-122.4194, alt_m=100.0, speed_ms=9.0, course_deg=180.0,
            battery_pct=95.0, flight_mode="STEALTH_LOITER"
        )
        status = bridge.get_status()
        self.assertEqual(status["flight_mode"], "STEALTH_LOITER")
        self.assertAlmostEqual(status["last_lat"], 37.7749, places=4)

        # Broadcast telemetry
        broadcast_ok = bridge.broadcast_telemetry()
        self.assertTrue(broadcast_ok)
        self.assertGreaterEqual(bridge.n_packets_sent, 1)

        # Inbound waypoint queueing
        bridge.received_waypoints.append((37.8000, -122.4000, 150.0, "ATAK-DROP"))
        wp = bridge.pop_latest_waypoint()
        self.assertIsNotNone(wp)
        self.assertEqual(wp[3], "ATAK-DROP")
        self.assertAlmostEqual(wp[0], 37.8000, places=4)

        # Close
        bridge.close()
        self.assertFalse(bridge._running)


class TestTacticalEKF(unittest.TestCase):
    """Test 15-State Tactical ES-EKF and Wind Vector Estimation."""

    def test_ekf_prediction_and_propagation(self):
        ekf = ekf_mod.TacticalEKF(ref_lat=37.7749, ref_lon=-122.4194, ref_alt_m=50.0)
        self.assertAlmostEqual(ekf.pos_n, 0.0)
        self.assertAlmostEqual(ekf.pos_e, 0.0)

        # Propagate 50 Hz steps forward with forward acceleration
        for _ in range(50):   # 1 second at 50 Hz
            ekf.predict(
                dt=0.02,
                ax_mps2=1.0, ay_mps2=0.0, az_mps2=-9.80665,
                gx_rads=0.0, gy_rads=0.0, gz_rads=0.0
            )

        sol = ekf.get_navigation_solution()
        # With 1 m/s^2 forward accel for 1s, pos_n ~ 0.5m, vel_n ~ 1.0 m/s
        self.assertGreater(sol["vel_n_ms"], 0.8)
        self.assertGreater(sol["pos_n_m"], 0.3)

    def test_wind_vector_estimation_learning(self):
        ekf = ekf_mod.TacticalEKF(ref_lat=37.7749, ref_lon=-122.4194, ref_alt_m=50.0)
        ekf.update_pitot_airspeed(10.0)   # 10 m/s heading North
        ekf.update_heading(0.0)           # Yaw = 0° (North)

        # Actual GPS Ground speed is 10 m/s North + 4 m/s East (due to 4 m/s East crosswind)
        for _ in range(30):
            ekf.update_gps(
                lat=37.7750, lon=-122.4190, alt_m=50.0,
                vn_ms=10.0, ve_ms=4.0, vd_ms=0.0,
                hdop=0.8, n_sats=12
            )

        sol = ekf.get_navigation_solution()
        # Wind East component should have converged towards +4.0 m/s
        self.assertGreater(sol["wind_e_ms"], 2.5)
        self.assertGreater(sol["wind_speed_ms"], 2.5)
        self.assertGreater(sol["wind_confidence"], 0.5)

    def test_gps_denied_wind_compensated_dead_reckoning(self):
        ekf = ekf_mod.TacticalEKF(ref_lat=37.7749, ref_lon=-122.4194, ref_alt_m=50.0)
        # Manually inject known 5 m/s wind from West (blowing East)
        ekf.inject_wind(wind_speed_ms=5.0, wind_dir_from_deg=270.0)
        ekf.set_gps_denied(True)
        ekf.update_pitot_airspeed(10.0)
        ekf.update_heading(0.0)   # Flying North

        # Propagate 10 seconds of flight at 50 Hz (500 steps)
        for _ in range(500):
            ekf.predict(dt=0.02, ax_mps2=0.0, ay_mps2=0.0, az_mps2=-9.80665, gx_rads=0.0, gy_rads=0.0, gz_rads=0.0)
            ekf.update_pitot_airspeed(10.0)

        sol = ekf.get_navigation_solution()
        # Drone should have progressed North (~100m) and drifted East with wind (~50m)
        self.assertGreater(sol["pos_n_m"], 80.0)
        self.assertGreater(sol["pos_e_m"], 35.0)
        self.assertTrue(sol["gps_denied"])


class TestContestedEWAndStealthLoiter(unittest.TestCase):
    """Test Intent arbitration and behaviours under Contested EW and Stealth Loiter."""

    def test_intent_contested_ew_trigger(self):
        interp = ic.IntentInterpreter()
        sticks = {'roll': 0.0, 'pitch': 0.0, 'throttle': 0.5, 'yaw': 0.0}
        switches = {'mode_sw': 0}

        # Normal flight -> Cruise
        context_normal = {'V_ms': 9.0, 'h_agl_m': 50.0}
        intent, conf, params = interp.interpret(sticks, switches, context_normal)
        self.assertIn(intent, (ic.Intent.CRUISE, ic.Intent.LOCUST_TORSIONAL_CRUISE))

        # Electronic Warfare conditions: RC signal lost + GPS jammed
        context_ew = {
            'V_ms': 9.0,
            'h_agl_m': 50.0,
            'rc_lost': True,
            'gps_jammed': True,
        }
        intent_ew, conf_ew, params_ew = interp.interpret(sticks, switches, context_ew)
        self.assertEqual(intent_ew, ic.Intent.CONTESTED_EW)
        self.assertEqual(params_ew.get('target_h_m'), 30.0)

    def test_intent_stealth_loiter_trigger(self):
        interp = ic.IntentInterpreter()
        sticks = {'roll': 0.0, 'pitch': 0.0, 'throttle': 0.5, 'yaw': 0.0}
        switches = {'stealth_sw': True}
        context = {'V_ms': 9.0, 'h_agl_m': 50.0}

        intent, conf, params = interp.interpret(sticks, switches, context)
        self.assertEqual(intent, ic.Intent.STEALTH_LOITER)

    def test_stealth_loiter_behaviour_execution(self):
        behaviour = ic.StealthLoiterBehaviour()

        class DummyController:
            _tgt_freq = 0.0
            _tgt_sweeps = []
            _tgt_dihedrals = []
            _tgt_pitches = []
            _stealth_active = False

        ctrl = DummyController()
        status = behaviour.tick({'h_agl_m': 40.0}, ctrl)
        self.assertEqual(status, ic.RUNNING)
        self.assertAlmostEqual(ctrl._tgt_freq, 1.25)
        self.assertEqual(len(ctrl._tgt_sweeps), 8)
        self.assertTrue(ctrl._stealth_active)

    def test_contested_ew_behaviour_execution(self):
        behaviour = ic.ContestedEWBehaviour()

        class DummyController:
            _tgt_freq = 0.0
            _tgt_sweeps = []
            _tgt_dihedrals = []
            _tgt_pitches = []
            _stealth_active = False
            _contested_ew_active = False

        ctrl = DummyController()
        # High altitude -> descent to terrain mask
        status = behaviour.tick({'h_agl_m': 60.0, 'bus_voltage_v': 23.5}, ctrl)
        self.assertEqual(status, ic.RUNNING)
        self.assertAlmostEqual(ctrl._tgt_freq, 1.10)
        self.assertTrue(ctrl._contested_ew_active)

        # Low battery -> emergency perch landing
        status_crit = behaviour.tick({'h_agl_m': 10.0, 'bus_voltage_v': 20.5}, ctrl)
        self.assertEqual(status_crit, ic.SUCCESS)
        self.assertAlmostEqual(ctrl._tgt_freq, 0.5)


class TestGroundStationACLCommands(unittest.TestCase):
    """Test Ground Station Access Control List (ACL) whitelisting and command execution."""

    def test_acl_command_whitelist_membership(self):
        expected_commands = {
            'cot_status', 'cot_broadcast', 'cot_inject_waypoint', 'cot_start', 'cot_stop',
            'ekf_status', 'ekf_reset', 'ekf_inject_wind',
            'ew_status', 'ew_trigger', 'ew_clear', 'stealth_loiter_start'
        }
        for cmd in expected_commands:
            self.assertIn(cmd, w._GS_ALLOWED_CMDS, f"Command {cmd} not in _GS_ALLOWED_CMDS whitelist (ACL)")

    def test_acl_command_execution(self):
        # 1. CoT Status
        res_cot = w._execute_gs_base_cmd({'cmd': 'cot_status'})
        self.assertEqual(res_cot['type'], 'ack')
        self.assertIn('running', res_cot['result'])

        # 2. EKF Status
        res_ekf = w._execute_gs_base_cmd({'cmd': 'ekf_status'})
        self.assertEqual(res_ekf['type'], 'ack')
        self.assertIn('ground_speed_ms', res_ekf['result'])

        # 3. EKF Wind Injection
        res_wind = w._execute_gs_base_cmd({'cmd': 'ekf_inject_wind', 'speed_ms': 7.5, 'dir_deg': 180.0})
        self.assertEqual(res_wind['type'], 'ack')
        self.assertEqual(res_wind['result']['wind_speed_ms'], 7.5)

        # 4. EW Trigger and Clear
        res_ew_trig = w._execute_gs_base_cmd({'cmd': 'ew_trigger', 'reason': 'sim_radar_jamming'})
        self.assertEqual(res_ew_trig['type'], 'ack')
        self.assertTrue(res_ew_trig['result']['ew_active'])

        res_ew_status = w._execute_gs_base_cmd({'cmd': 'ew_status'})
        self.assertTrue(res_ew_status['result']['active'])

        res_ew_clear = w._execute_gs_base_cmd({'cmd': 'ew_clear'})
        self.assertEqual(res_ew_clear['type'], 'ack')
        self.assertFalse(res_ew_clear['result']['ew_active'])

        # 5. Stealth Loiter Start
        res_stealth = w._execute_gs_base_cmd({'cmd': 'stealth_loiter_start'})
        self.assertEqual(res_stealth['type'], 'ack')
        self.assertTrue(res_stealth['result']['stealth_loiter'])

        # 6. CoT Waypoint Injection
        res_wp = w._execute_gs_base_cmd({
            'cmd': 'cot_inject_waypoint',
            'lat': 34.05,
            'lon': -118.25,
            'alt_m': 100.0,
            'callsign': 'ATAK-ALPHA'
        })
        self.assertEqual(res_wp['type'], 'ack')
        self.assertEqual(res_wp['result']['lat'], 34.05)


if __name__ == '__main__':
    unittest.main()
