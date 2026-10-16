"""
Unit Tests: Electrical Spring & DC Bus Architecture Suite (H-BUS 1..4)
======================================================================
Comprehensive verification suite for Hypotheses H-BUS-1 through H-BUS-4
and the Adaptive Learning Coordinator (ALC) Electrical Bus Architect modality:

  1. Hypothesis H-BUS-1: Shared DC Bus Inertial Ripple Cancellation
     - Analytic ripple fraction engine (bus_ripple_fraction)
     - ElectricalBusMonitor 50 Hz observer engine (deque windowing, ripple metrics,
       exchange fraction, symmetry gate, Wh saved)

  2. Hypothesis H-BUS-2: Split-Bus Fault-Tolerant Isolation
     - SplitBusGuard dual-rail observer & simulation model
     - 2.5 us microsecond eFuse trip on short-circuit dV/dt or undervoltage
     - Healthy rail isolation retention & fail-operational trigger
     - Fuse reset capability

  3. Hypothesis H-BUS-3: Dynamic Thermal Load-Balancing in Downwash
     - ThermalLoadBalancer aft-motor downwash observer & controller
     - Differential temperature thresholding (T_aft > 65°C and delta_T > 3°C)
     - Asymmetric stroke bias forward (+delta fore, -delta aft)
     - Pitch trim compensation and clamping

  4. Hypothesis H-BUS-4: In-Flight Flapping EIS Battery SOH Observer
     - FlappingEISObserver rolling impedance extraction
     - Natural 5.2 Hz stroke ripple perturbation validation
     - Dynamic internal resistance R_int = dV_fund / dI_fund
     - SOH calculation and tiered degradation alerts

  5. ALC Integration: electrical_bus_architect Modality
     - Modality registration in _ALC_SHARED['modalities']
     - Pipeline execution: observe, learn, share, explain
     - CRM 3-Tier Alert hierarchy (Tier 1 ripple, Tier 2 thermal/EIS, Tier 3 eFuse)
     - Priority hypothesis queue dispatch
     - Ground Station command: bus_health_status()

  6. Firmware Catalog Registration & Selection
     - Hypotheses registered in _ADV_NEW_HYPS, AERO_HYPOTHESES, and _hypothesis_queue
     - Scoring tasks registered in _SG_TASKS
     - Curriculum segments in _ALSS_SEGMENTS and _ALSS_CURRICULUM['B']
     - Flight phase mappings in _PHASE_TO_SG_TASK
     - Keyword and sensor routing in discovery_hypothesis_select()
"""

import sys
import os
import io
import unittest
import math

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'firmware'))

import mco_wave_library as wl
from mco_flight_governor import (
    bus_ripple_fraction,
    ElectricalBusMonitor,
    SplitBusGuard,
    ThermalLoadBalancer,
    FlappingEISObserver,
)


class TestBusRippleAnalytic(unittest.TestCase):
    """Verifies the analytic harmonic ripple fraction across N=8 wings (H-BUS-1)."""

    def test_zero_crossing_at_22_5_deg(self):
        frac = bus_ripple_fraction(22.5, n=8)
        self.assertAlmostEqual(frac, 0.0, places=6,
                               msg="bus_ripple_fraction must be zero at phi=22.5 deg")

    def test_zero_crossing_at_45_deg(self):
        frac = bus_ripple_fraction(45.0, n=8)
        self.assertAlmostEqual(frac, 0.0, places=6,
                               msg="bus_ripple_fraction must be zero at phi=45.0 deg")

    def test_zero_limit_guard(self):
        frac = bus_ripple_fraction(0.0, n=8)
        self.assertEqual(frac, 1.0, "bus_ripple_fraction at phi=0 must return 1.0 guard")

    def test_nominal_28_deg_cascade(self):
        frac = bus_ripple_fraction(28.0, n=8)
        # Expected: |sin(8*28°) / sin(28°)| / 8 = |-0.69466 / 0.46947| / 8 ≈ 0.18496
        self.assertAlmostEqual(frac, 0.18496, places=4,
                               msg="bus_ripple_fraction at 28 deg should be ~0.185 (81.5% cancellation)")


class TestElectricalBusMonitor(unittest.TestCase):
    """Verifies the 50 Hz ElectricalBusMonitor observer engine (H-BUS-1)."""

    def setUp(self):
        self.monitor = ElectricalBusMonitor(n_wings=8, wing_inertia_kg_m2=0.027,
                                             window_sec=5.0, sample_rate_hz=50.0)

    def test_initial_state(self):
        self.assertEqual(self.monitor.window_samples, 250)
        self.assertEqual(self.monitor.cumulative_wh_saved, 0.0)

    def test_single_sample_tick(self):
        res = self.monitor.tick(v_bus=24.0, i_bus=10.0, timestamp_s=0.0,
                                commanded_phi_deg=28.0, commanded_freq_hz=2.59,
                                commanded_amp_deg=96.0, symmetry_delta_deg=0.0)
        self.assertEqual(res['bus_voltage_v'], 24.0)
        self.assertEqual(res['bus_current_a'], 10.0)
        self.assertEqual(res['bus_power_mean_w'], 240.0)
        self.assertEqual(res['bus_voltage_ripple_v'], 0.0)
        self.assertEqual(res['bus_power_ripple_w'], 0.0)
        self.assertTrue(res['symmetry_gate_pass'])
        self.assertTrue(res['voltage_ripple_healthy'])
        self.assertAlmostEqual(res['predicted_ripple_fraction'], 0.185, places=3)
        self.assertGreater(res['uncancelled_swing_w'], 300.0)

    def test_symmetry_gate(self):
        res_pass = self.monitor.tick(24.0, 10.0, 0.02, symmetry_delta_deg=2.5)
        self.assertTrue(res_pass['symmetry_gate_pass'])

        res_fail = self.monitor.tick(24.0, 10.0, 0.04, symmetry_delta_deg=3.5)
        self.assertFalse(res_fail['symmetry_gate_pass'])

    def test_exchange_fraction_with_ripple(self):
        t = 0.0
        for _ in range(50):
            t += 0.02
            v = 24.0
            i = 5.0
            self.monitor.tick(v, i, t, commanded_phi_deg=28.0)
            t += 0.02
            i = 25.0
            res = self.monitor.tick(v, i, t, commanded_phi_deg=28.0)

        self.assertAlmostEqual(res['bus_power_ripple_w'], 480.0, places=1)
        self.assertLess(res['bus_exchange_fraction'], 1.0)
        self.assertGreater(res['bus_exchange_fraction'], 0.0)

    def test_wh_saved_accumulation(self):
        t = 0.0
        for _ in range(100):
            t += 0.1
            res = self.monitor.tick(24.0, 10.0, t, commanded_phi_deg=28.0)
        self.assertGreater(res['cumulative_wh_saved'], 0.0)


class TestSplitBusGuard(unittest.TestCase):
    """Verifies the Split-Bus eFuse Fault Isolation Engine (H-BUS-2)."""

    def setUp(self):
        self.guard = SplitBusGuard(brownout_thresh_v=16.0, trip_current_a=40.0,
                                   trip_dv_dt_v_per_s=-500.0, hardware_trip_time_us=2.5)

    def test_normal_operation(self):
        res = self.guard.tick(v_port=24.0, i_port=10.0, v_stbd=24.0, i_stbd=10.0, timestamp_s=0.0)
        self.assertTrue(res['port_rail_active'])
        self.assertTrue(res['stbd_rail_active'])
        self.assertFalse(res['port_efuse_tripped'])
        self.assertFalse(res['stbd_efuse_tripped'])
        self.assertTrue(res['bus_healthy'])
        self.assertFalse(res['fail_operational_required'])
        self.assertEqual(res['split_bus_margin_v'], 24.0)
        self.assertEqual(res['isolation_time_us'], 2.5)

    def test_port_undervoltage_trip(self):
        # Normal first tick
        self.guard.tick(24.0, 10.0, 24.0, 10.0, 0.0)
        # Port experiences short: voltage drops to 14.0 V with 50.0 A current
        res = self.guard.tick(v_port=14.0, i_port=50.0, v_stbd=23.8, i_stbd=12.0, timestamp_s=0.02)
        self.assertFalse(res['port_rail_active'])
        self.assertTrue(res['stbd_rail_active'])
        self.assertTrue(res['port_efuse_tripped'])
        self.assertFalse(res['stbd_efuse_tripped'])
        self.assertFalse(res['bus_healthy'])
        self.assertTrue(res['fail_operational_required'])
        self.assertEqual(res['split_bus_margin_v'], 23.8)

    def test_stbd_dv_dt_trip(self):
        # Step 1: Baseline at t=0.0
        self.guard.tick(24.0, 10.0, 24.0, 10.0, 0.0)
        # Step 2: Severe dV/dt on Starboard: drops 6.0 V in 10 ms (-600 V/s < -500 V/s threshold)
        res = self.guard.tick(v_port=24.0, i_port=10.0, v_stbd=18.0, i_stbd=20.0, timestamp_s=0.01)
        self.assertTrue(res['port_rail_active'])
        self.assertFalse(res['stbd_rail_active'])
        self.assertTrue(res['stbd_efuse_tripped'])
        self.assertTrue(res['fail_operational_required'])

    def test_reset_fuses(self):
        self.guard.tick(12.0, 55.0, 24.0, 10.0, 0.02)
        self.assertTrue(self.guard.port_efuse_tripped)
        self.guard.reset_fuses()
        self.assertFalse(self.guard.port_efuse_tripped)
        self.assertFalse(self.guard.stbd_efuse_tripped)


class TestThermalLoadBalancer(unittest.TestCase):
    """Verifies Dynamic Thermal Load-Balancing in Downwash (H-BUS-3)."""

    def setUp(self):
        self.balancer = ThermalLoadBalancer(temp_warning_c=65.0, temp_critical_c=80.0,
                                            max_amp_bias_deg=8.0, pitch_comp_ratio=-0.15)

    def test_nominal_cooling(self):
        temps = [35.0] * 8
        res = self.balancer.balance(temps, base_amp_deg=96.0)
        self.assertFalse(res['thermal_throttling_active'])
        self.assertEqual(res['fore_amp_deg'], 96.0)
        self.assertEqual(res['aft_amp_deg'], 96.0)
        self.assertEqual(res['amp_bias_deg'], 0.0)
        self.assertEqual(res['pitch_trim_offset_deg'], 0.0)
        self.assertEqual(res['max_motor_temp_c'], 35.0)
        self.assertEqual(res['aft_motor_temp_margin_c'], 45.0)

    def test_uniform_high_temp_no_asymmetry(self):
        # All motors at 70°C -> delta_T = 0 <= 3.0, so no asymmetric downwash shift
        temps = [70.0] * 8
        res = self.balancer.balance(temps, base_amp_deg=96.0)
        self.assertFalse(res['thermal_throttling_active'])
        self.assertEqual(res['amp_bias_deg'], 0.0)

    def test_aft_downwash_overheating_trigger(self):
        # Fore motors cool (45°C), Aft motors hot (73°C)
        temps = [45.0, 45.0, 45.0, 45.0, 73.0, 73.0, 73.0, 73.0]
        res = self.balancer.balance(temps, base_amp_deg=96.0)
        self.assertTrue(res['thermal_throttling_active'])
        self.assertGreater(res['amp_bias_deg'], 0.0)
        self.assertGreater(res['fore_amp_deg'], 96.0)
        self.assertLess(res['aft_amp_deg'], 96.0)
        self.assertLess(res['pitch_trim_offset_deg'], 0.0)  # Counteracts nose-up pitch moment
        self.assertEqual(res['delta_t_c'], 28.0)
        self.assertEqual(res['max_motor_temp_c'], 73.0)

    def test_max_bias_clamping(self):
        # Extreme aft thermal load
        temps = [30.0, 30.0, 30.0, 30.0, 95.0, 95.0, 95.0, 95.0]
        res = self.balancer.balance(temps, base_amp_deg=96.0)
        self.assertTrue(res['thermal_throttling_active'])
        self.assertEqual(res['amp_bias_deg'], 8.0)  # Clamped to max_amp_bias_deg
        self.assertEqual(res['fore_amp_deg'], 104.0)
        self.assertEqual(res['aft_amp_deg'], 88.0)
        self.assertAlmostEqual(res['pitch_trim_offset_deg'], -1.2, places=2)


class TestFlappingEISObserver(unittest.TestCase):
    """Verifies In-Flight Flapping EIS Battery SOH Observer (H-BUS-4)."""

    def setUp(self):
        self.eis = FlappingEISObserver(r_ref_mohm=22.0, window_samples=20)

    def test_initial_warmup(self):
        res = self.eis.tick(v_bus=24.0, i_bus=10.0)
        self.assertEqual(res['r_int_rolling_mohm'], 22.0)
        self.assertEqual(res['eis_soh_pct'], 100.0)
        self.assertFalse(res['soh_warning'])
        self.assertFalse(res['perturbation_valid'])

    def test_impedance_extraction_under_stroke_ripple(self):
        # Inject 5.2 Hz alternating stroke ripple:
        # At peak stroke: current 35 A, voltage 23.6 V
        # At stroke reversal: current 15 A, voltage 24.1 V
        # delta_V = 0.5 V, delta_I = 20 A -> R = (0.5 / 20) * 1000 = 25.0 mΩ
        res = None
        for i in range(25):
            if i % 2 == 0:
                res = self.eis.tick(v_bus=23.6, i_bus=35.0)
            else:
                res = self.eis.tick(v_bus=24.1, i_bus=15.0)

        self.assertTrue(res['perturbation_valid'])
        self.assertAlmostEqual(res['r_int_live_mohm'], 25.0, places=1)
        self.assertGreater(res['eis_snr_db'], 20.0)
        self.assertFalse(res['soh_warning'])
        self.assertGreater(res['eis_soh_pct'], 80.0)

    def test_battery_degradation_alert(self):
        # Inject degraded cell perturbation:
        # delta_V = 0.9 V, delta_I = 20 A -> R = 45.0 mΩ (> 35% rise over 22 mΩ)
        res = None
        for i in range(80):
            if i % 2 == 0:
                res = self.eis.tick(v_bus=23.2, i_bus=35.0)
            else:
                res = self.eis.tick(v_bus=24.1, i_bus=15.0)

        self.assertTrue(res['soh_warning'])
        self.assertLess(res['eis_soh_pct'], 80.0)
        self.assertGreater(res['r_int_rolling_mohm'], 30.0)


class TestALCElectricalBusArchitectModality(unittest.TestCase):
    """Verifies the ALC electrical_bus_architect modality and CRM alerts."""

    def setUp(self):
        self.saved_phase = wl._ALC_SHARED.get('flight_phase', 'cruise')
        self.saved_alerts = list(wl._ALC_SHARED.get('alert_log', []))
        self.saved_queue = list(wl._ALC_SHARED.get('priority_hyp_queue', []))
        wl.alc_set_phase('cruise')

    def tearDown(self):
        wl.alc_set_phase(self.saved_phase)
        wl._ALC_SHARED['alert_log'] = self.saved_alerts
        wl._ALC_SHARED['priority_hyp_queue'] = self.saved_queue

    def test_modality_registration(self):
        self.assertIn('electrical_bus_architect', wl._ALC_SHARED['modalities'])
        mod = wl._ALC_SHARED['modalities']['electrical_bus_architect']
        self.assertTrue(callable(mod['observe']))
        self.assertTrue(callable(mod['learn']))
        self.assertTrue(callable(mod['share']))
        self.assertTrue(callable(mod['explain']))

    def test_observe_and_learn_pipeline(self):
        sensors = {
            'bus_voltage_v': 24.2,
            'bus_current_a': 14.5,
            'bus_power_ripple_w': 35.0,
            'bus_voltage_ripple_v': 0.18,
            'bus_exchange_fraction': 0.88,
            'cumulative_wh_saved': 1.45,
            'port_rail_active': True,
            'stbd_rail_active': True,
            'motor_temps': [38.0] * 8,
            'r_int_rolling_mohm': 21.5,
            'eis_soh_pct': 100.0,
        }
        findings = wl._bus_architect_observe(sensors, {})
        self.assertEqual(findings['v_bus'], 24.2)
        self.assertEqual(findings['exchange_frac'], 0.88)
        self.assertEqual(findings['aft_margin_c'], 42.0)

        wl._bus_architect_learn(findings)
        st = wl._BUS_ARCHITECT_STATE
        self.assertEqual(st['bus_voltage_ripple_v'], 0.18)
        self.assertEqual(st['bus_exchange_fraction'], 0.88)
        self.assertEqual(st['r_int_rolling_mohm'], 21.5)

    def test_crm_tier3_on_split_bus_fault(self):
        wl._ALC_SHARED['priority_hyp_queue'].clear()
        findings = {
            'port_active': False,
            'stbd_active': True,
            'margin_v': 23.5,
        }
        wl._bus_architect_learn(findings)
        self.assertIn('pending_tier3', wl._ALC_SHARED)
        self.assertTrue(any(e.get('hyp_id') == 'H-BUS-2' for e in wl._ALC_SHARED['priority_hyp_queue']))

    def test_crm_tier2_on_aft_thermal_overheat(self):
        wl._ALC_SHARED['priority_hyp_queue'].clear()
        findings = {
            'port_active': True,
            'stbd_active': True,
            'aft_margin_c': 8.0,
            'delta_t': 14.0,
            'max_t': 72.0,
            'aft_t': 72.0,
        }
        wl._bus_architect_learn(findings)
        self.assertTrue(wl._BUS_ARCHITECT_STATE['thermal_throttling_active'])
        self.assertGreater(wl._BUS_ARCHITECT_STATE['thermal_amp_bias_deg'], 0.0)
        self.assertTrue(any(e.get('hyp_id') == 'H-BUS-3' for e in wl._ALC_SHARED['priority_hyp_queue']))

    def test_crm_tier2_on_eis_degradation(self):
        wl._ALC_SHARED['priority_hyp_queue'].clear()
        findings = {
            'port_active': True,
            'stbd_active': True,
            'r_int': 36.5,
            'soh_pct': 68.0,
            'soh_warn': True,
        }
        wl._bus_architect_learn(findings)
        self.assertTrue(wl._BUS_ARCHITECT_STATE['eis_degradation_detected'])
        self.assertTrue(any(e.get('hyp_id') == 'H-BUS-4' for e in wl._ALC_SHARED['priority_hyp_queue']))

    def test_share_and_explain(self):
        shared = wl._bus_architect_share()
        self.assertIn('bus_power_ripple_w', shared)
        self.assertIn('bus_exchange_fraction', shared)
        self.assertIn('split_bus_healthy', shared)
        self.assertIn('battery_r_int_mohm', shared)

        summary = wl._bus_architect_explain()
        self.assertIsInstance(summary, str)
        self.assertIn("Electrical Bus Intelligence", summary)

    def test_gs_bus_health_status(self):
        old_stdout = sys.stdout
        sys.stdout = io.StringIO()
        try:
            wl.bus_health_status()
            out = sys.stdout.getvalue()
            self.assertIn("ELECTRICAL BUS & POWER INTELLIGENCE", out)
            self.assertIn("Split-Bus eFuse Rails", out)
            self.assertIn("Downwash Thermal Balance", out)
            self.assertIn("Live EIS Battery SOH", out)
        finally:
            sys.stdout = old_stdout


class TestHypothesisRegistration(unittest.TestCase):
    """Verifies H-BUS-1 through H-BUS-4 registration across all catalogues and dispatchers."""

    def test_adv_new_hyps_registration(self):
        ids = [h['id'] for h in wl._ADV_NEW_HYPS]
        for hid in ['H-BUS-1', 'H-BUS-2', 'H-BUS-3', 'H-BUS-4']:
            self.assertIn(hid, ids, f"Hypothesis {hid} missing from _ADV_NEW_HYPS")

        h2 = next(h for h in wl._ADV_NEW_HYPS if h['id'] == 'H-BUS-2')
        self.assertEqual(h2['target'], 'fail_operational')
        self.assertIn('bus_fault_isolation_time_us', h2['config']['_scorer'])

        h3 = next(h for h in wl._ADV_NEW_HYPS if h['id'] == 'H-BUS-3')
        self.assertEqual(h3['target'], 'motor_temperature')
        self.assertIn('aft_motor_temp_margin_c', h3['config']['_scorer'])

        h4 = next(h for h in wl._ADV_NEW_HYPS if h['id'] == 'H-BUS-4')
        self.assertEqual(h4['target'], 'battery_health')
        self.assertIn('battery_internal_resistance_mohm', h4['config']['_scorer'])

    def test_aero_hypotheses_registration(self):
        ids = [h['id'] for h in wl.AERO_HYPOTHESES]
        for hid in ['H-BUS-1', 'H-BUS-2', 'H-BUS-3', 'H-BUS-4']:
            self.assertIn(hid, ids, f"Hypothesis {hid} missing from AERO_HYPOTHESES")

    def test_hypothesis_queue_registration(self):
        ids = [h['id'] for h in wl._hypothesis_queue]
        for hid in ['H-BUS-1', 'H-BUS-2', 'H-BUS-3', 'H-BUS-4']:
            self.assertIn(hid, ids, f"Hypothesis {hid} missing from _hypothesis_queue")

    def test_scoring_tasks_registration(self):
        expected_tasks = [
            'bus_power_ripple_w',
            'bus_exchange_fraction',
            'bus_voltage_ripple_v',
            'bus_fault_isolation_time_us',
            'aft_motor_temp_margin_c',
            'battery_internal_resistance_mohm',
        ]
        for task in expected_tasks:
            self.assertIn(task, wl._SG_TASKS, f"Task {task} missing from _SG_TASKS")

    def test_curriculum_registration(self):
        expected_segments = ['bus1_exchange_b', 'bus2_split_b', 'bus3_thermal_b', 'bus4_eis_b']
        for seg in expected_segments:
            self.assertIn(seg, wl._ALSS_SEGMENTS, f"Segment {seg} missing from _ALSS_SEGMENTS")
            self.assertIn(seg, wl._ALSS_CURRICULUM['B'], f"Segment {seg} missing from _ALSS_CURRICULUM['B']")

        self.assertIn('bus_ripple', wl._PHASE_TO_SG_TASK)
        self.assertIn('bus_fault_isolation', wl._PHASE_TO_SG_TASK)
        self.assertIn('thermal_throttling', wl._PHASE_TO_SG_TASK)
        self.assertIn('battery_impedance', wl._PHASE_TO_SG_TASK)

    def test_discovery_hypothesis_select_keywords(self):
        keyword_targets = {
            'electrical_spring':   'H-BUS-1',
            'h_bus':               'H-BUS-1',
            'bus_ripple':          'H-BUS-1',
            'shared_bus':          'H-BUS-1',
            'inertial_exchange':   'H-BUS-1',
            'split_bus':           'H-BUS-2',
            'efuse':               'H-BUS-2',
            'bus_isolation':       'H-BUS-2',
            'h_bus_2':             'H-BUS-2',
            'thermal_balance':     'H-BUS-3',
            'downwash_cooling':    'H-BUS-3',
            'aft_motor_heat':      'H-BUS-3',
            'h_bus_3':             'H-BUS-3',
            'eis':                 'H-BUS-4',
            'battery_eis':         'H-BUS-4',
            'internal_resistance': 'H-BUS-4',
            'battery_soh':         'H-BUS-4',
            'h_bus_4':             'H-BUS-4',
        }
        for kw, expected_id in keyword_targets.items():
            selected = wl.discovery_hypothesis_select(focus=kw)
            self.assertIsNotNone(selected, f"Focus keyword '{kw}' returned None")
            self.assertEqual(selected['id'], expected_id,
                             f"Focus keyword '{kw}' did not select {expected_id}, got {selected['id']}")

    def test_discovery_hypothesis_select_sensors(self):
        sensor_cases = [
            ({'bus_ripple_active': True}, 'H-BUS-1'),
            ({'efuse_tripped': True}, 'H-BUS-2'),
            ({'split_bus_fault': True}, 'H-BUS-2'),
            ({'thermal_throttling_active': True}, 'H-BUS-3'),
            ({'aft_motor_hot': True}, 'H-BUS-3'),
            ({'soh_warning': True}, 'H-BUS-4'),
            ({'eis_active': True}, 'H-BUS-4'),
        ]
        for sensors, expected_id in sensor_cases:
            selected = wl.discovery_hypothesis_select(sensors=sensors)
            self.assertIsNotNone(selected, f"Sensors {sensors} returned None")
            self.assertEqual(selected['id'], expected_id,
                             f"Sensors {sensors} selected {selected['id']}, expected {expected_id}")


if __name__ == '__main__':
    unittest.main()
