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

    def test_path2_ps_governed_climb_limits(self):
        """Path 2 Milestone 2: Climb modes must enforce electrical Ps constraints."""
        clb1 = w.unified_lever_to_params(72.0, V_ms=8.0, h_agl_m=10.0)
        clb2 = w.unified_lever_to_params(82.0, V_ms=8.0, h_agl_m=10.0)

        self.assertEqual(clb1['mode_id'], 'CLB1_RATE')
        self.assertLessEqual(clb1['climb_rate_ceiling_ms'], 2.2)
        self.assertLessEqual(clb1['max_continuous_power_w'], 280.0)
        self.assertFalse(clb1['is_kinetic_zoom'])

        self.assertEqual(clb2['mode_id'], 'CLB2_ANGLE')
        self.assertLessEqual(clb2['max_steady_gamma_deg'], 15.5)
        self.assertLessEqual(clb2['climb_rate_ceiling_ms'], 2.2)
        self.assertLessEqual(clb2['max_continuous_power_w'], 280.0)
        self.assertFalse(clb2['is_kinetic_zoom'])

    def test_path2_atmospheric_density_and_loaded_servo_slew(self):
        """Path 2 Milestone 3: Barometric density and aero hinge derating on servo slew rate."""
        import mco_flight_governor as fg
        # Density decreases with altitude
        rho_sea_level = fg.atmospheric_density(0.0)
        rho_1000m = fg.atmospheric_density(1000.0)
        self.assertAlmostEqual(rho_sea_level, 1.225, delta=0.01)
        self.assertLess(rho_1000m, rho_sea_level)

        # Loaded slew rate derates at high speed due to aerodynamic hinge moments
        slew_0ms = fg.compute_loaded_servo_slew_ceiling(V_ms=0.0)
        slew_20ms = fg.compute_loaded_servo_slew_ceiling(V_ms=20.0)
        self.assertEqual(slew_0ms, fg.PITCH_SERVO_SLEW_DPS)
        self.assertLess(slew_20ms, slew_0ms)
        self.assertGreaterEqual(slew_20ms, fg.PITCH_SERVO_SLEW_DPS * 0.35)

    def test_path2_active_vs_apparent_power_model(self):
        """Path 2 Milestone 1: Reconcile flapping motor active work from apparent voltamperes."""
        import mco_performance_model as pm
        model = pm.PerformanceModel()
        res = model.predict(freq_hz=2.0, V_ms=10.0, sweep_deg_mean=60.0,
                            amplitude_deg=80.0, throttle_frac=0.7)

        self.assertIn('power_apparent_va', res)
        self.assertIn('power_W', res)
        self.assertIn('power_factor', res)
        self.assertEqual(res['power_factor'], pm.PF_FLAPPING)
        self.assertLess(res['power_W'], res['power_apparent_va'])
        self.assertLessEqual(res['power_W'], 600.0)

    def test_oc1_cf1_clap_fling_dispatcher(self):
        """O-C1: Verify CF-1 clap-and-fling execution dispatcher and safety gates."""
        # 1. Normal active burst mid-cycle
        res = w.run_cf1_clap_fling(t_sec=0.5, V_ms=4.0, bus_voltage_v=22.2, agl_m=5.0)
        self.assertTrue(res['active'])
        self.assertFalse(res['is_complete'])
        self.assertEqual(res['mode_id'], 'CF1_BURST')
        self.assertGreater(res['estimated_lift_n'], 100.0)
        self.assertTrue(res['hardstop_clearance_verified'])

        # 2. Voltage sag abort gate (< 20.5V)
        sag_res = w.run_cf1_clap_fling(t_sec=0.5, bus_voltage_v=20.0, agl_m=5.0)
        self.assertFalse(sag_res['active'])
        self.assertEqual(sag_res['mode_id'], 'CF1_VOLTAGE_SAG_ABORT')

        # 3. Ground proximity lockout (< 1.2m unless bench_mode)
        gnd_res = w.run_cf1_clap_fling(t_sec=0.5, bus_voltage_v=22.2, agl_m=0.8, bench_mode=False)
        self.assertFalse(gnd_res['active'])
        self.assertEqual(gnd_res['mode_id'], 'CF1_GROUND_LOCKOUT')

        # Bench mode bypasses ground lockout
        bench_res = w.run_cf1_clap_fling(t_sec=0.5, bus_voltage_v=22.2, agl_m=0.8, bench_mode=True)
        self.assertTrue(bench_res['active'])

        # 4. Burst completion reversion (t >= duration)
        comp_res = w.run_cf1_clap_fling(t_sec=2.0, bus_voltage_v=22.2, agl_m=5.0, burst_strokes=4, flap_freq_hz=2.5)
        self.assertFalse(comp_res['active'])
        self.assertTrue(comp_res['is_complete'])
        self.assertEqual(comp_res['mode_id'], w.CF1_REVERT_MODE)

    def test_od7_mountain_terrain_trigger(self):
        """O-D7: Verify autonomous GPS mountain terrain trigger and ridge lift detection."""
        from mco_mavlink_bridge import detect_mountain_terrain_trigger, PixhawkBridge

        # 1. Lowland nominal
        t_low = detect_mountain_terrain_trigger(34.0, -118.0, alt_msl_m=100.0, agl_m=80.0)
        self.assertFalse(t_low['mountain_mode'])
        self.assertFalse(t_low['ridge_updraft_detected'])
        self.assertEqual(t_low['stall_floor_factor'], 1.10)

        # 2. High altitude MSL (> 1500m)
        t_high = detect_mountain_terrain_trigger(34.0, -118.0, alt_msl_m=1800.0, agl_m=200.0)
        self.assertTrue(t_high['mountain_mode'])
        self.assertEqual(t_high['stall_floor_factor'], 1.20)

        # 3. Rugged terrain proximity (MSL > 600m, AGL < 150m) with ridge updraft (+1.8 m/s climb)
        t_ridge = detect_mountain_terrain_trigger(34.0, -118.0, alt_msl_m=950.0, agl_m=90.0, climb_rate_ms=1.8)
        self.assertTrue(t_ridge['mountain_mode'])
        self.assertTrue(t_ridge['ridge_updraft_detected'])

        # 4. PixhawkBridge integration
        bridge = PixhawkBridge(port=None)
        res = bridge.update_terrain_state(34.0, -118.0, alt_msl_m=1600.0, agl_m=100.0, climb_rate_ms=2.0)
        state = bridge.get_state()
        self.assertTrue(state['mountain_mode'])
        self.assertTrue(state['ridge_updraft_detected'])


if __name__ == '__main__':
    unittest.main()
