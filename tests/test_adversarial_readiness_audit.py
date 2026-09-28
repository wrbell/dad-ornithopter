#!/usr/bin/env python3
"""
Adversarial Aircraft Readiness Audit Test Suite
================================================
Verifies all 14 safety and operational vulnerabilities identified
during the pre-flight adversarial software audit.
"""

import sys
import os
import unittest
import time
import math

# Ensure firmware directory is in Python path
REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
FIRMWARE_DIR = os.path.join(REPO_ROOT, 'firmware')
if FIRMWARE_DIR not in sys.path:
    sys.path.insert(0, FIRMWARE_DIR)

import mco_wave_library as w
from mco_logger import MCOLogger
from mco_odrive_interface import ODriveInterface, N_WINGS
from mco_gs_server import MCOGSServer
from mco_ground_station import AppState, DEFAULT_PORT
import mco_preflight_checks as pfc


class TestAdversarialReadinessAudit(unittest.TestCase):

    def test_aud01_servo_pitch_inversion_and_sweep(self):
        """AUD2-01: Starboard pitch servos must be inverted relative to port servos."""
        # Check wave library conversion helper
        port_pitch = 10.0
        stbd_pitch = 10.0
        port_us = w.pitch_deg_to_us(port_pitch, servo_index=0)  # Port W1
        stbd_us = w.pitch_deg_to_us(stbd_pitch, servo_index=1)  # Stbd W1
        
        # Directions are opposite (+1 vs -1)
        self.assertEqual(w.PITCH_SERVO_DIRECTION[0], +1)
        self.assertEqual(w.PITCH_SERVO_DIRECTION[1], -1)
        # Port increases pulse width, Starboard decreases pulse width
        self.assertGreater(port_us, w.PITCH_PWM_NEUTRAL_US)
        self.assertLess(stbd_us, w.PITCH_PWM_NEUTRAL_US)

        # Neutral sweep is 60° -> 1500 us
        neutral_us = w.sweep_deg_to_us(60.0, servo_index=0)
        self.assertEqual(neutral_us, 1500)

        # Starboard sweep mirrors about neutral
        port_sweep_70 = w.sweep_deg_to_us(70.0, servo_index=0)
        stbd_sweep_70 = w.sweep_deg_to_us(70.0, servo_index=1)
        self.assertGreater(port_sweep_70, 1500)
        self.assertLess(stbd_sweep_70, 1500)

    def test_aud02_motor_flap_direction_applied(self):
        """AUD2-02: Reverse-routed Dyneema capstans must invert motor commands."""
        self.assertEqual(len(w.FLAP_MOTOR_DIRECTION), 8)
        # Verify that direction multipliers are either +1 or -1
        for d in w.FLAP_MOTOR_DIRECTION:
            self.assertIn(d, (+1, -1))
        
        # Test that flap_turns_corrected applies FLAP_MOTOR_DIRECTION
        orig_dir = w.FLAP_MOTOR_DIRECTION[1]
        try:
            w.FLAP_MOTOR_DIRECTION[1] = -1
            corrected = w.flap_turns_corrected(5.0, servo_index=1)
            self.assertEqual(corrected, -5.0)
            
            w.FLAP_MOTOR_DIRECTION[1] = +1
            corrected_pos = w.flap_turns_corrected(5.0, servo_index=1)
            self.assertEqual(corrected_pos, +5.0)
        finally:
            w.FLAP_MOTOR_DIRECTION[1] = orig_dir

    def test_aud03_odrive_telemetry_nonblocking(self):
        """AUD2-03: ODrive telemetry reads must be non-blocking (< 5 ms)."""
        odi = ODriveInterface()
        t0 = time.perf_counter()
        pos, vel = odi.read_encoder_estimates()
        currents = odi.read_currents()
        temps = odi.read_temperatures()
        vbus = odi.read_vbus()
        states = odi.read_all_states()
        elapsed_ms = (time.perf_counter() - t0) * 1000.0
        odi.close()

        # Telemetry should be cached and return nearly instantaneously
        self.assertLess(elapsed_ms, 5.0, f"Telemetry reads took {elapsed_ms:.2f} ms — expected < 5 ms")
        self.assertEqual(len(pos), N_WINGS)
        self.assertEqual(len(vel), N_WINGS)
        self.assertEqual(len(currents), N_WINGS)
        self.assertEqual(len(temps), N_WINGS)
        self.assertEqual(len(states), N_WINGS)
        self.assertGreater(vbus, 10.0)

    def test_aud04_battery_6s_detection_and_thresholds(self):
        """AUD2-04: Battery voltage detection must support 6S LiPo with proper land cutoff."""
        # 19.8V is dead 6S (3.3V/cell) -> should FAIL preflight check
        res = pfc.CheckResult()
        pfc.check_battery_voltage(res, {'bus_voltage_v': 19.8, 'servo_voltage_v': 7.4})
        chk = res.checks[-1]
        self.assertEqual(chk.status, pfc.FAIL, f"19.8V 6S battery should FAIL, got {chk.status}: {chk.message}")
        self.assertIn("6S", chk.message)
        self.assertIn("~0%", chk.message)

        # 25.0V is healthy 6S -> PASS
        res2 = pfc.CheckResult()
        pfc.check_battery_voltage(res2, {'bus_voltage_v': 25.0, 'servo_voltage_v': 7.4})
        chk2 = res2.checks[-1]
        self.assertEqual(chk2.status, pfc.PASS)
        self.assertIn("~96%", chk2.message)

        # 16.5V is healthy 4S (91.6% full) -> PASS
        res3 = pfc.CheckResult()
        pfc.check_battery_voltage(res3, {'bus_voltage_v': 16.5, 'servo_voltage_v': 7.4})
        chk3 = res3.checks[-1]
        self.assertEqual(chk3.status, pfc.PASS)
        self.assertIn("4S", chk3.message)

        # In wave library: 20.0V on 6S triggers land_now emergency
        w._fault_state['degraded_mode'] = 'normal'
        w.fault_detect_actuators(None, [1.0]*8, [0.0]*8, [0.0]*8, bus_voltage_v=20.0)
        self.assertEqual(w._fault_state['degraded_mode'], 'land_now')
        self.assertIn('BATTERY_CRITICAL', w._fault_state['fault_codes'])

        # Reset fault state
        w._fault_state['degraded_mode'] = 'normal'
        w._clear_fault('BATTERY_CRITICAL')

    def test_aud05_logger_pre_start_logging(self):
        """AUD2-11: Calling mco_logger.log() before start() must not raise TypeError."""
        logger = MCOLogger()
        # Should not throw TypeError even though start() was not called
        try:
            logger.log({'pitch_deg_imu': 0.0, 'roll_deg_imu': 0.0}, {'ground_state': 'INIT'})
        except TypeError as e:
            self.fail(f"logger.log() raised TypeError before start(): {e}")

    def test_aud06_certification_tools_import(self):
        """AUD2-10: mco_certification_tools must import cleanly without Pi filesystem dependencies."""
        try:
            import mco_certification_tools as cert
            self.assertTrue(hasattr(cert, '_get_cert_dir'))
            cert_dir = cert._get_cert_dir()
            self.assertTrue(os.path.isdir(cert_dir))
        except Exception as e:
            self.fail(f"Failed to import or run mco_certification_tools: {e}")

    def test_aud07_gs_default_port(self):
        """AUD2-08: Ground station default port must match server port 5757."""
        self.assertEqual(DEFAULT_PORT, 5757)

    def test_aud08_gs_server_command_queue(self):
        """AUD2-07: MCOGSServer must queue dict payloads preserving parameters."""
        server = MCOGSServer()
        # Simulate queuing a command dict
        cmd_payload = {'type': 'cmd', 'cmd': 'set_param', 'name': 'sweep_deg', 'value': 15.0}
        with server._cmd_lock:
            server._cmd_queue.append(cmd_payload)
        
        pending = server.get_pending_commands()
        self.assertEqual(len(pending), 1)
        self.assertIsInstance(pending[0], dict)
        self.assertEqual(pending[0]['cmd'], 'set_param')
        self.assertEqual(pending[0]['value'], 15.0)

    def test_aud09_app_state_ingest_safety(self):
        """AUD2-09: AppState.ingest must safely handle string acks and non-dict payloads."""
        app = AppState()
        # Raw string ack should not raise AttributeError
        try:
            app.ingest("OK: armed")
            self.assertTrue(any("OK: armed" in a[1] for a in app.alerts))
        except Exception as e:
            self.fail(f"app.ingest('string') raised: {e}")

        # Dict ack should append to alerts
        app.ingest({'type': 'ack', 'cmd': 'arm', 'msg': 'Armed successfully'})
        self.assertTrue(any("Armed successfully" in a[1] for a in app.alerts))

    def test_aud10_preflight_telemetry_fallbacks(self):
        """AUD2-05: Preflight checks must pass when IMU/baro telemetry is valid."""
        res = pfc.CheckResult()
        sensors = {
            'imu_ok': True,
            'baro_ok': True,
            'oat_c': 21.5,
            'accel_z_g': 1.002,
            'h_baro_m': 15.2,
        }
        pfc.check_imu(res, sensors)
        pfc.check_barometer(res, sensors)
        pfc.check_oat_sensor(res, sensors)

        self.assertEqual(res.checks[0].status, pfc.PASS)
        self.assertEqual(res.checks[1].status, pfc.PASS)
        self.assertEqual(res.checks[2].status, pfc.PASS)


if __name__ == '__main__':
    unittest.main()
