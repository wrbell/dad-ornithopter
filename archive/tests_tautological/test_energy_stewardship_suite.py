"""
Unit Test Suite: ALC Energy Stewardship & Joules Optimization
============================================================
Verifies:
  1. energy_stewardship ALC modality registration and share dictionary
  2. Demanding pilot input override (100% Ps, no down-throttling or efficiency restriction)
  3. Pareto multi-objective scoring (CoT, RMS form factor, Strouhal sweet spot)
  4. Reactive hypothesis queuing (H-30 on peaky current, H-32 on Strouhal mismatch, H-SOR-3 on updraft)
  5. Carson's best-range speed and Cost of Transport in PerformanceModel
  6. Per-stroke kinematic reversal back-EMF recovery in powered flight
  7. Telemetry reporting and Ground Station commands
"""

import unittest
import math
import firmware.mco_wave_library as wl
import firmware.mco_performance_model as pm
import firmware.mco_mavlink_bridge as mb


class TestEnergyStewardshipModality(unittest.TestCase):

    def setUp(self):
        wl._ALC_SHARED['priority_hyp_queue'].clear()
        wl._STEWARDSHIP_STATE['ambient_soaring_active'] = False
        wl._STEWARDSHIP_STATE['energy_saved_est_wh'] = 0.0
        wl._STEWARDSHIP_STATE['ambient_soaring_wh'] = 0.0

    def test_modality_registered(self):
        """Verify energy_stewardship is registered in ALC modalities."""
        self.assertIn('energy_stewardship', wl._ALC_SHARED['modalities'])
        mod = wl._ALC_SHARED['modalities']['energy_stewardship']
        self.assertTrue(callable(mod['observe']))
        self.assertTrue(callable(mod['learn']))
        self.assertTrue(callable(mod['share']))
        self.assertTrue(callable(mod['explain']))

    def test_modality_share_keys(self):
        """Verify energy_stewardship share returns all required metrics."""
        share = wl._energy_stewardship_share()
        expected_keys = [
            'cot_current', 'cot_rolling', 'cot_best',
            'wh_per_km', 'wh_per_km_rolling', 'wh_per_km_best',
            'form_factor_kf', 'strouhal', 'strouhal_optimal',
            'ambient_soaring_active', 'ambient_soaring_wh',
            'energy_saved_wh', 'carsons_speed_ms',
            'joules_total', 'joules_ambient'
        ]
        for key in expected_keys:
            self.assertIn(key, share, f"Missing key in share dict: {key}")

    def test_demanding_input_never_throttles(self):
        """
        Verify that whenever pilot commands demanding inputs (full stick, extreme lever,
        or abrupt stick rates), precision/efficiency scoring is 100% bypassed and raw Ps is returned.
        """
        # Full pitch stick excursion
        sensors_pitch_demand = {
            'pitch_stick': 0.85,
            'Ps_measured': 3.45,
            'V_ms': 8.0,
            'motor_I_A': 18.0,
            'bus_voltage_v': 22.2,
        }
        score, metric, detail = wl.phase_objective_score(sensors_pitch_demand, {}, phase='cruise')
        self.assertEqual(metric, 'ps_demand')
        self.assertEqual(score, 3.45)
        self.assertEqual(detail['phase'], 'demand_override')

        # Full roll stick excursion
        sensors_roll_demand = {
            'roll_stick': 0.90,
            'Ps_measured': 2.80,
            'V_ms': 7.0,
        }
        score, metric, detail = wl.phase_objective_score(sensors_roll_demand, {}, phase='cruise')
        self.assertEqual(metric, 'ps_demand')
        self.assertEqual(score, 2.80)

        # Abrupt stick rate
        sensors_abrupt = {
            'pitch_stick_rate': 250.0,
            'Ps_measured': 4.10,
        }
        score, metric, detail = wl.phase_objective_score(sensors_abrupt, {}, phase='cruise')
        self.assertEqual(metric, 'ps_demand')
        self.assertEqual(score, 4.10)

    def test_pareto_form_factor_penalty(self):
        """
        Verify that during steady cruise, a peaky current form factor (high I²R dissipation)
        scores lower than a smooth form factor for identical average power.
        """
        # Smooth current: kf = 1.0
        sensors_smooth = {
            'pitch_stick': 0.0, 'roll_stick': 0.0, 'yaw_stick': 0.0,
            'lever_pct': 50.0, 'pitch_stick_rate': 0.0,
            'V_ms': 8.0, 'motor_I_A': 2.0, 'motor_I_rms_A': 2.0,
            'bus_voltage_v': 22.2, 'sink_rate_ms': 0.0,
        }
        res_nominal = {'freq_hz_cmd': 1.87, 'amp_port_deg': 90.0}
        score_smooth, _, detail_smooth = wl.phase_objective_score(sensors_smooth, res_nominal, phase='cruise')

        # Peaky current: kf = 1.60
        sensors_peaky = dict(sensors_smooth)
        sensors_peaky['motor_I_rms_A'] = 3.2  # 3.2 / 2.0 = 1.6
        score_peaky, _, detail_peaky = wl.phase_objective_score(sensors_peaky, res_nominal, phase='cruise')

        self.assertGreater(score_smooth, score_peaky,
                           "Smooth current form factor must score higher than peaky current")
        self.assertAlmostEqual(detail_smooth['form_factor_kf'], 1.0, places=2)
        self.assertAlmostEqual(detail_peaky['form_factor_kf'], 1.6, places=2)

    def test_pareto_strouhal_sweet_spot(self):
        """
        Verify that flapping kinematics operating within the bio-optimal Strouhal window (0.20 - 0.35)
        receive an efficiency bonus over highly sub-optimal Strouhal numbers.
        """
        sensors = {
            'pitch_stick': 0.0, 'roll_stick': 0.0, 'yaw_stick': 0.0,
            'lever_pct': 50.0, 'pitch_stick_rate': 0.0,
            'V_ms': 10.0, 'motor_I_A': 2.0, 'bus_voltage_v': 22.2,
            'sink_rate_ms': 0.0,
        }
        # Optimal Strouhal: freq=1.87Hz, amp=60deg, V=10m/s -> St ≈ 0.30
        res_opt = {'freq_hz_cmd': 1.87, 'amp_port_deg': 60.0}
        score_opt, _, detail_opt = wl.phase_objective_score(sensors, res_opt, phase='cruise')

        # Excessive Strouhal: freq=3.5Hz, amp=90deg, V=5m/s -> St ≈ 1.70
        sensors_sub = dict(sensors)
        sensors_sub['V_ms'] = 5.0
        res_sub = {'freq_hz_cmd': 3.5, 'amp_port_deg': 90.0}
        score_sub, _, detail_sub = wl.phase_objective_score(sensors_sub, res_sub, phase='cruise')

        self.assertTrue(0.20 <= detail_opt['strouhal'] <= 0.35)
        self.assertGreater(score_opt, score_sub)

    def test_reactive_dispatch_on_peaky_current(self):
        """Verify H-30 (power-smoothing cascade) is queued when form factor kf > 1.35."""
        findings = {
            'cot': 4.2, 'wh_km': 65.0, 'form_factor_kf': 1.50,
            'strouhal': 0.28, 'strouhal_optimal': True,
            'P_watts': 44.0, 'V_ms': 8.0, 'vsi': 0.0,
            'updraft_conf': 0.1,
        }
        wl.alc_set_phase('cruise')
        wl._energy_stewardship_learn(findings)

        queued_ids = [q.get('hyp_id') for q in wl._ALC_SHARED['priority_hyp_queue']]
        self.assertIn('H-30', queued_ids, "H-30 must be queued upon detecting peaky current form factor")

    def test_reactive_dispatch_on_updraft(self):
        """Verify H-SOR-3 (thermal soaring assist) is queued and ambient_soaring_active set when updraft detected."""
        findings = {
            'cot': 3.0, 'wh_km': 50.0, 'form_factor_kf': 1.1,
            'strouhal': 0.28, 'strouhal_optimal': True,
            'P_watts': 25.0, 'V_ms': 7.0, 'vsi': +0.4,
            'updraft_conf': 0.75,
        }
        wl.alc_set_phase('cruise')
        wl._energy_stewardship_learn(findings)

        self.assertTrue(wl._STEWARDSHIP_STATE['ambient_soaring_active'])
        queued_ids = [q.get('hyp_id') for q in wl._ALC_SHARED['priority_hyp_queue']]
        self.assertIn('H-SOR-3', queued_ids, "H-SOR-3 must be queued upon sustained updraft detection")
        self.assertGreater(wl._STEWARDSHIP_STATE['ambient_soaring_wh'], 0.0)


class TestPerformanceModelStewardship(unittest.TestCase):

    def setUp(self):
        self.model = pm.PerformanceModel()

    def test_cost_of_transport_function(self):
        """Verify dimensionless Cost of Transport calculation."""
        weight_N = 5.687 * 9.81
        cot = pm.cost_of_transport(power_W=50.0, weight_N=weight_N, V_ms=8.0)
        expected = 50.0 / (weight_N * 8.0)
        self.assertAlmostEqual(cot, expected, places=4)

        # Zero speed safety
        self.assertEqual(pm.cost_of_transport(50.0, weight_N, V_ms=0.0), 99.0)

    def test_carsons_speed_calculation(self):
        """Verify Carson's speed calculation (V_br ≈ 1.316 * V_md ≈ 11.67 m/s at MTOW)."""
        v_br = pm.carsons_speed(weight_N=5.687 * 9.81, altitude_m=0.0)
        self.assertGreater(v_br, 10.0)
        self.assertLess(v_br, 13.0)
        self.assertAlmostEqual(v_br, 11.67, delta=0.5)

    def test_predict_outputs_energy_metrics(self):
        """Verify predict() includes cost_of_transport, carsons_speed_ms, wh_per_km, joules_per_meter."""
        res = self.model.predict(
            freq_hz=1.87, V_ms=8.0, sweep_deg_mean=60.0,
            amplitude_deg=90.0, throttle_frac=0.65
        )
        self.assertIn('cost_of_transport', res)
        self.assertIn('carsons_speed_ms', res)
        self.assertIn('wh_per_km', res)
        self.assertIn('joules_per_meter', res)
        self.assertGreater(res['cost_of_transport'], 0.0)
        self.assertGreater(res['carsons_speed_ms'], 0.0)


class TestStrokeReversalRegen(unittest.TestCase):

    def setUp(self):
        wl._REGEN_STATE['energy_regen_wh'] = 0.0
        wl._REGEN_STATE['energy_reversal_wh'] = 0.0

    def test_powered_flight_stroke_reversal_recovery(self):
        """Verify that in powered cruise, stroke reversal decel harvests back-EMF energy."""
        sensors = {
            'lever_pct': 65.0,     # Powered cruise zone
            'V_ms': 8.0,
            'bus_voltage': 22.2,
            'vsi_smooth_ms': 0.0,
        }
        result = {
            'freq_hz_cmd': 2.0,
            'amp_port_deg': 90.0,
            'cascade_phase_deg': 28.0,
        }
        res = wl._regen_tick(sensors, result)
        self.assertTrue(res['regen_active'])
        self.assertEqual(res['regen_context'], 'stroke_reversal')
        self.assertGreater(res['regen_P_W'], 0.0)
        self.assertGreater(wl._REGEN_STATE['energy_reversal_wh'], 0.0)

    def test_reversal_regen_overvoltage_protection(self):
        """Verify regen is inhibited when bus voltage reaches battery protection ceiling (>=25.0V)."""
        sensors = {
            'lever_pct': 65.0,
            'V_ms': 8.0,
            'bus_voltage': 25.1,   # Near full 6S charge
            'vsi_smooth_ms': 0.0,
        }
        result = {
            'freq_hz_cmd': 2.0,
            'amp_port_deg': 90.0,
        }
        res = wl._regen_tick(sensors, result)
        self.assertFalse(res.get('regen_active', False))


class TestTelemetryAndCommands(unittest.TestCase):

    def test_ground_station_energy_status(self):
        """Verify energy_status() and stewardship_status() execute without error."""
        import io, sys
        captured = io.StringIO()
        old_stdout = sys.stdout
        try:
            sys.stdout = captured
            wl.energy_status()
            output = captured.getvalue()
            self.assertIn("ENERGY STEWARDSHIP", output)
            self.assertIn("Cost of Transport", output)
            self.assertIn("Carson's best-range speed", output)

            # Test alias
            captured.seek(0)
            captured.truncate(0)
            wl.stewardship_status()
            output_alias = captured.getvalue()
            self.assertIn("ENERGY STEWARDSHIP", output_alias)
        finally:
            sys.stdout = old_stdout

    def test_mavlink_energy_stewardship_broadcast(self):
        """Verify MAVLink PixhawkBridge send_energy_stewardship does not error."""
        bridge = mb.PixhawkBridge()
        # Simulation mode or connected
        bridge.send_energy_stewardship(cot=3.85, wh_per_km=62.0, carsons_speed_ms=11.7)
        bridge.close()


if __name__ == '__main__':
    unittest.main()
