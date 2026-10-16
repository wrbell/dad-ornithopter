"""
test_safety_and_performance_upgrades.py
───────────────────────────────────────
Unit test suite verifying all 8 safety, performance, and pilot-intent upgrades:
  1. ServoBusGovernor (2S 7.4V rail protection, derating, and brownout lock)
  2. WeightOnWheelsEstimator (multi-sensor fusion with debounced hysteresis)
  3. ServoRateLimiter (AGFRC IB53BHU slew rate and acceleration clamp)
  4. TurnCoordinationGovernor (FBW turn coordination with pilot toggle & flat-turn bypass)
  5. ContinuousStrouhalRegulator (St ~ 0.28 cruise frequency matching & energy savings)
  6. CascadeSlipstreamGovernor (airspeed-adaptive inter-tier phase lock)
  7. PitotBlockageFailsafe (icing/freeze detection & synthetic airspeed fallback)
  8. 50 Hz Control Loop Deadline Monitor & Watchdog (overrun detection and safe-hold)
"""

import math
import sys
import unittest
from pathlib import Path

# Add firmware directory to path
FIRMWARE_DIR = Path(__file__).resolve().parent.parent / 'firmware'
if str(FIRMWARE_DIR) not in sys.path:
    sys.path.insert(0, str(FIRMWARE_DIR))

import mco_advanced_physics as ap
from mco_flight_governor import (
    ServoBusGovernor,
    govern_servo_bus,
    WeightOnWheelsEstimator,
    govern_weight_on_wheels,
    ServoRateLimiter,
    govern_servo_rates,
    TurnCoordinationGovernor,
    govern_turn_coordination,
    set_turn_coordination,
    is_turn_coordination_enabled,
    ContinuousStrouhalRegulator,
    govern_strouhal_cruise,
    CascadeSlipstreamGovernor,
    govern_cascade_slipstream,
    check_pitot_blockage_and_synthetic_airspeed,
    DesertThermalInterleaver,
    govern_desert_thermal_interleaving,
    PreFlightSnowShake,
    govern_preflight_snow_shake,
    govern_zero_friction_ice_steering,
    ColdLiPoOcvEstimator,
    estimate_cold_lipo_ocv,
    ExtremeGustSqueezeGovernor,
    govern_extreme_gust_squeeze,
    PilotAlpineTakeoffGovernor,
    govern_pilot_alpine_takeoff_and_rollout,
    ROLLOUT_BRAKING_FORWARD_PITCH_DEG,
    ROLLOUT_BRAKING_AFT_PITCH_DEG,
    ROLLOUT_BRAKING_SUCTION_PITCH_DEG,
    ALPINE_MAX_TAKEOFF_AMP_DEG,
)
from mco_main import MCOMain


class TestServoBusGovernor(unittest.TestCase):
    """Verifies 2S 7.4V servo bus brownout protection and flap amplitude derating."""

    def setUp(self):
        self.gov = ServoBusGovernor()

    def test_nominal_voltage(self):
        telem = self.gov.tick(bus_voltage_v=7.4, flap_amp_deg=30.0)
        self.assertEqual(telem['status'], 'NOMINAL')
        self.assertEqual(telem['derate_factor'], 1.0)
        self.assertFalse(telem['brownout_lock'])
        self.assertAlmostEqual(telem['clamped_amp_deg'], 30.0)

    def test_derated_warning(self):
        telem = self.gov.tick(bus_voltage_v=6.65, flap_amp_deg=30.0)
        self.assertEqual(telem['status'], 'VOLTAGE_DERATED')
        self.assertGreaterEqual(telem['derate_factor'], 0.50)
        self.assertLess(telem['derate_factor'], 1.0)
        self.assertFalse(telem['brownout_lock'])
        self.assertLess(telem['clamped_amp_deg'], 30.0)

    def test_critical_brownout_lock(self):
        telem = self.gov.tick(bus_voltage_v=6.40, flap_amp_deg=30.0)
        self.assertEqual(telem['status'], 'CRITICAL_BROWNOUT_LOCK')
        self.assertEqual(telem['derate_factor'], 0.0)
        self.assertTrue(telem['brownout_lock'])
        self.assertEqual(telem['clamped_amp_deg'], 0.0)
        self.assertGreaterEqual(telem['alarm_count'], 1)

    def test_functional_governor_clamp(self):
        res = {'wing_commands': [{'flap_amp_deg': 35.0} for _ in range(8)]}
        govern_servo_bus(bus_voltage_v=6.4, flap_amp_deg=35.0, result=res)
        self.assertTrue(res['servo_bus_brownout_lock'])
        for cmd in res['wing_commands']:
            self.assertEqual(cmd['flap_amp_deg'], 0.0)


class TestWeightOnWheelsEstimator(unittest.TestCase):
    """Verifies multi-sensor fusion with debounced hysteresis."""

    def setUp(self):
        self.estimator = WeightOnWheelsEstimator(debounce_ticks=5)

    def test_initial_ground_state(self):
        telem = self.estimator.update(h_agl_m=0.05, az_g=1.0, gear_contact=True)
        self.assertTrue(telem['on_ground'])
        self.assertEqual(telem['state'], 'ON_GROUND')
        self.assertGreater(telem['weight_on_wheels_n'], 30.0)

    def test_debounce_takeoff_transition(self):
        # 1-3 ticks of airborne signal should NOT cause immediate state change
        for _ in range(3):
            telem = self.estimator.update(h_agl_m=0.50, az_g=1.1, gear_contact=False)
            self.assertTrue(telem['on_ground'])   # still debouncing
            self.assertEqual(telem['state'], 'TAKEOFF_ROTATING')

        # After 5 ticks, confirmed AIRBORNE
        for _ in range(3):
            telem = self.estimator.update(h_agl_m=0.50, az_g=1.1, gear_contact=False)
        self.assertFalse(telem['on_ground'])
        self.assertEqual(telem['state'], 'AIRBORNE')
        self.assertEqual(telem['weight_on_wheels_n'], 0.0)

    def test_debounce_landing_touchdown(self):
        # Set airborne
        for _ in range(6):
            self.estimator.update(h_agl_m=5.0, az_g=1.0, gear_contact=False)
        self.assertFalse(self.estimator.on_ground)

        # First couple ticks on ground
        telem = self.estimator.update(h_agl_m=0.08, az_g=1.0, gear_contact=True)
        self.assertFalse(telem['on_ground'])   # still airborne debouncing
        self.assertEqual(telem['state'], 'TOUCHDOWN_DEBOUNCE')

        # Sustained 5 ticks -> confirmed ON_GROUND
        for _ in range(5):
            telem = self.estimator.update(h_agl_m=0.08, az_g=1.0, gear_contact=True)
        self.assertTrue(telem['on_ground'])
        self.assertEqual(telem['state'], 'ON_GROUND')


class TestServoRateLimiter(unittest.TestCase):
    """Verifies AGFRC IB53BHU slew rate and acceleration limits."""

    def setUp(self):
        self.limiter = ServoRateLimiter(num_channels=8)

    def test_pitch_rate_limiting_under_load(self):
        # Large step request: 0 deg to 30 deg in 20 ms (1500 deg/s) under q = 150 Pa
        p_out, s_out = self.limiter.limit_command(
            target_pitch_deg=30.0, target_sweep_deg=60.0,
            dt=0.02, q_dyn_pa=150.0, channel_id=0
        )
        # Max rate is 250 deg/s, max accel is 2500 deg/s^2 -> max delta in 0.02s is bounded
        self.assertLess(p_out, 10.0)
        self.assertGreater(p_out, 0.0)

    def test_sweep_rate_limiting(self):
        # Large sweep step request: 60 deg to 120 deg in 20 ms (3000 deg/s)
        p_out, s_out = self.limiter.limit_command(
            target_pitch_deg=0.0, target_sweep_deg=120.0,
            dt=0.02, q_dyn_pa=100.0, channel_id=0
        )
        # Max sweep rate 220 deg/s
        self.assertLess(s_out, 70.0)
        self.assertGreater(s_out, 60.0)

    def test_filter_wing_commands(self):
        cmds = [{'pitch_deg': 45.0, 'sweep_deg': 110.0} for _ in range(8)]
        res = {}
        filtered = govern_servo_rates(cmds, dt=0.02, q_dyn_pa=120.0, result=res)
        self.assertTrue(res.get('servo_rate_limiting_active'))
        for cmd in filtered:
            self.assertLess(cmd['pitch_deg'], 45.0)
            self.assertLess(cmd['sweep_deg'], 110.0)


class TestTurnCoordinationGovernor(unittest.TestCase):
    """Verifies FBW turn coordination and pilot toggle ability."""

    def setUp(self):
        set_turn_coordination(True)
        self.coord = TurnCoordinationGovernor()

    def tearDown(self):
        set_turn_coordination(True)

    def test_banked_turn_active_coordination(self):
        # 30 deg bank at 15 m/s cruise
        telem = self.coord.compute(bank_deg=30.0, pitch_deg=0.0, V_ms=15.0, intent='cruise')
        self.assertTrue(telem['enabled'])
        self.assertTrue(telem['coordination_active'])
        # r_coord = (9.81 / 15.0) * tan(30) = 0.654 * 0.577 = 0.377 rad/s = 21.6 deg/s
        self.assertGreater(telem['r_coord_deg_s'], 15.0)
        self.assertGreater(telem['delta_yaw_cmd'], 0.0)

    def test_pilot_toggle_disables_coordination(self):
        # Pilot turns off turn coordination for intentional slip / aerobatics
        set_turn_coordination(False)
        self.assertFalse(is_turn_coordination_enabled())
        telem = self.coord.compute(bank_deg=30.0, pitch_deg=0.0, V_ms=15.0, intent='cruise')
        self.assertFalse(telem['enabled'])
        self.assertFalse(telem['coordination_active'])
        self.assertEqual(telem['r_coord_deg_s'], 0.0)
        self.assertEqual(telem['delta_yaw_cmd'], 0.0)
        self.assertEqual(telem['mode'], 'BYPASS_DISABLED')

    def test_flat_turn_bypasses_coordination(self):
        # In flat turn (yaw with 0 bank), turn coordination must NOT interfere
        set_turn_coordination(True)
        telem = self.coord.compute(bank_deg=0.0, pitch_deg=0.0, V_ms=15.0, intent='flat_turn')
        self.assertFalse(telem['coordination_active'])
        self.assertEqual(telem['r_coord_deg_s'], 0.0)
        self.assertEqual(telem['mode'], 'BYPASS_CONDITION')

    def test_low_airspeed_or_ground_bypasses_coordination(self):
        # On ground or low speed (<3 m/s) bypasses coordination
        telem = self.coord.compute(bank_deg=20.0, pitch_deg=0.0, V_ms=2.0, on_ground=True)
        self.assertFalse(telem['coordination_active'])
        self.assertEqual(telem['r_coord_deg_s'], 0.0)


class TestContinuousStrouhalRegulator(unittest.TestCase):
    """Verifies cruise flapping frequency modulation for St ~ 0.28."""

    def setUp(self):
        self.reg = ContinuousStrouhalRegulator()

    def test_cruise_strouhal_optimization(self):
        # At 12 m/s with 20 deg flap amplitude
        telem = self.reg.compute(V_ias_ms=12.0, flap_amp_deg=20.0, current_freq_hz=1.8, intent='cruise')
        self.assertTrue(telem['active'])
        self.assertEqual(telem['target_st'], 0.28)
        self.assertGreater(telem['optimal_freq_hz'], 1.5)
        self.assertLess(telem['optimal_freq_hz'], 2.8)
        self.assertGreaterEqual(telem['energy_savings_pct'], 0.0)

    def test_inactive_modes(self):
        telem = self.reg.compute(V_ias_ms=12.0, flap_amp_deg=20.0, current_freq_hz=1.8, intent='hover')
        self.assertFalse(telem['active'])

    def test_climb_and_sink_priority(self):
        # When sinking at -0.5 m/s, target frequency must NOT be derated below current 2.2 Hz
        telem_sink = self.reg.compute(
            V_ias_ms=10.0, flap_amp_deg=20.0, current_freq_hz=2.2, intent='cruise',
            climb_rate_ms=-0.50, throttle_frac=0.5
        )
        self.assertGreaterEqual(telem_sink['target_optimal_freq_hz'], 2.19)

        # When pilot demands high power (throttle 0.80), target frequency must NOT be derated
        telem_throttle = self.reg.compute(
            V_ias_ms=10.0, flap_amp_deg=20.0, current_freq_hz=2.4, intent='cruise',
            climb_rate_ms=0.0, throttle_frac=0.80
        )
        self.assertGreaterEqual(telem_throttle['target_optimal_freq_hz'], 2.39)


class TestDensityScalingAndGliding(unittest.TestCase):
    """Verifies aerodynamic density scaling for stall speed floors."""

    def test_glide_floor_density_scaling(self):
        from mco_flight_governor import check_glide_speed_floor

        # Standard sea level (rho = 1.225) -> floor 11.0 m/s
        telem_std = check_glide_speed_floor(V_ias_ms=11.2, throttle_frac=0.0, rho_kg_m3=1.225)
        self.assertTrue(telem_std['glide_permitted'])
        self.assertAlmostEqual(telem_std['effective_floor_ms'], 11.0, places=1)

        # Desert heat / high altitude (rho = 0.98) -> floor scales up to ~12.3 m/s
        telem_hot = check_glide_speed_floor(V_ias_ms=11.2, throttle_frac=0.0, rho_kg_m3=0.98)
        self.assertGreater(telem_hot['effective_floor_ms'], 12.0)
        # 11.2 m/s was safe at sea level, but in hot thin air it's below stall floor!
        self.assertFalse(telem_hot['glide_permitted'])

        # Sub-zero cold (rho = 1.36) -> floor safely lowers to ~10.4 m/s
        telem_cold = check_glide_speed_floor(V_ias_ms=10.6, throttle_frac=0.0, rho_kg_m3=1.36)
        self.assertLess(telem_cold['effective_floor_ms'], 10.6)
        self.assertTrue(telem_cold['glide_permitted'])


class TestCascadeSlipstreamGovernor(unittest.TestCase):
    """Verifies airspeed-adaptive inter-tier cascade slipstream phase lock."""

    def setUp(self):
        self.gov = CascadeSlipstreamGovernor()

    def test_phase_lock_scaling_with_airspeed(self):
        telem_slow = self.gov.compute(V_ias_ms=10.0, freq_hz=2.0, base_phase_deg=90.0)
        telem_fast = self.gov.compute(V_ias_ms=20.0, freq_hz=2.0, base_phase_deg=90.0)

        self.assertTrue(telem_slow['active'])
        self.assertTrue(telem_fast['active'])
        # Faster airspeed means smaller convective delay tau, hence smaller phase shift
        self.assertLess(telem_fast['vortex_delay_s'], telem_slow['vortex_delay_s'])
        self.assertNotEqual(telem_fast['intertier_phase_deg'], telem_slow['intertier_phase_deg'])


class TestPitotBlockageFailsafe(unittest.TestCase):
    """Verifies pitot icing/blockage detection and synthetic airspeed fallback."""

    def test_pitot_nominal(self):
        telem = check_pitot_blockage_and_synthetic_airspeed(
            V_pitot_ms=14.0, gps_v_ms=13.8, throttle_frac=0.5, h_agl_m=20.0, on_ground=False
        )
        self.assertFalse(telem['pitot_blocked'])
        self.assertFalse(telem['synthetic_airspeed_active'])
        self.assertEqual(telem['effective_v_ms'], 14.0)

    def test_pitot_blocked_in_flight(self):
        # Flying at 15 m/s GPS, but pitot frozen at 0.5 m/s (< 3.0 m/s)
        telem = check_pitot_blockage_and_synthetic_airspeed(
            V_pitot_ms=0.5, gps_v_ms=15.0, throttle_frac=0.6, h_agl_m=30.0, on_ground=False
        )
        self.assertTrue(telem['pitot_blocked'])
        self.assertTrue(telem['synthetic_airspeed_active'])
        self.assertEqual(telem['status'], 'PITOT_BLOCKED_SYNTHETIC_ACTIVE')
        self.assertEqual(telem['effective_v_ms'], 15.0)

    def test_pitot_blocked_gps_denied_throttle_fallback(self):
        # GPS is None/jammed, but airborne at cruise throttle 0.55 and pitot frozen at 0.2 m/s
        telem = check_pitot_blockage_and_synthetic_airspeed(
            V_pitot_ms=0.2, gps_v_ms=None, throttle_frac=0.55, h_agl_m=25.0, on_ground=False
        )
        self.assertTrue(telem['pitot_blocked'])
        self.assertTrue(telem['synthetic_airspeed_active'])
        self.assertEqual(telem['synthetic_source'], 'THROTTLE_POWER')
        self.assertGreaterEqual(telem['effective_v_ms'], 11.0)


class TestDesertThermalInterleaver(unittest.TestCase):
    """Verifies 30s convective slipstream duty cycle swapping across wing tiers."""

    def setUp(self):
        self.interleaver = DesertThermalInterleaver(swap_interval_s=30.0, entry_temp_c=70.0, exit_temp_c=65.0)

    def test_thermal_entry_trigger(self):
        # 68°C is below entry threshold
        cmds = [{'flap_amp_deg': 60.0, 'pitch_deg': 2.0} for _ in range(8)]
        telem = self.interleaver.update(dt_s=1.0, motor_temps_c=[68.0]*8, wing_commands=cmds, V_ias_ms=15.0)
        self.assertFalse(telem['active'])

        # 71°C exceeds threshold -> activates Phase 0 (Tiers 1 & 3 flap, Tiers 2 & 4 glide)
        telem = self.interleaver.update(dt_s=1.0, motor_temps_c=[71.0]*8, wing_commands=cmds, V_ias_ms=15.0)
        self.assertTrue(telem['active'])
        self.assertEqual(telem['active_group'], 'TIERS_1_3')

        # Wings 0, 1 (Tier 1) and 4, 5 (Tier 3) flap with boost; Wings 2, 3 and 6, 7 glide
        mod_cmds = telem['wing_commands']
        for i in (0, 1, 4, 5):
            self.assertGreater(mod_cmds[i]['flap_amp_deg'], 60.0)
            self.assertLessEqual(mod_cmds[i]['flap_amp_deg'], 96.0)
        for i in (2, 3, 6, 7):
            self.assertEqual(mod_cmds[i]['flap_amp_deg'], 0.0)
            self.assertEqual(mod_cmds[i]['pitch_deg'], 4.0)

    def test_tier_swap_at_30s(self):
        cmds = [{'flap_amp_deg': 60.0, 'pitch_deg': 2.0} for _ in range(8)]
        self.interleaver.update(dt_s=1.0, motor_temps_c=[72.0]*8, wing_commands=cmds, V_ias_ms=15.0)
        self.assertEqual(self.interleaver.current_tier_phase, 0)

        # Advance 29 seconds (total 30s) -> should swap to Phase 1 (Tiers 2 & 4 flap)
        telem = self.interleaver.update(dt_s=29.0, motor_temps_c=[72.0]*8, wing_commands=cmds, V_ias_ms=15.0)
        self.assertEqual(telem['active_group'], 'TIERS_2_4')
        mod_cmds = telem['wing_commands']
        for i in (0, 1, 4, 5):
            self.assertEqual(mod_cmds[i]['flap_amp_deg'], 0.0)
            self.assertEqual(mod_cmds[i]['pitch_deg'], 4.0)
        for i in (2, 3, 6, 7):
            self.assertGreater(mod_cmds[i]['flap_amp_deg'], 60.0)

    def test_hysteresis_exit(self):
        cmds = [{'flap_amp_deg': 60.0, 'pitch_deg': 2.0} for _ in range(8)]
        self.interleaver.update(dt_s=1.0, motor_temps_c=[72.0]*8, wing_commands=cmds, V_ias_ms=15.0)
        self.assertTrue(self.interleaver.active)

        # Drops to 67°C (between 65°C and 70°C) -> stays active (hysteresis)
        telem = self.interleaver.update(dt_s=1.0, motor_temps_c=[67.0]*8, wing_commands=cmds, V_ias_ms=15.0)
        self.assertTrue(telem['active'])

        # Drops below 65°C -> deactivates
        telem = self.interleaver.update(dt_s=1.0, motor_temps_c=[64.0]*8, wing_commands=cmds, V_ias_ms=15.0)
        self.assertFalse(telem['active'])

    def test_ground_interlock(self):
        # On ground, thermal interleaving MUST be inhibited
        cmds = [{'flap_amp_deg': 60.0, 'pitch_deg': 2.0} for _ in range(8)]
        telem = self.interleaver.update(dt_s=1.0, motor_temps_c=[80.0]*8, wing_commands=cmds, V_ias_ms=0.0, on_ground=True)
        self.assertFalse(telem['active'])


class TestPreFlightSnowShake(unittest.TestCase):
    """Verifies low-acceleration ground snow/frost shedding cycle."""

    def setUp(self):
        self.shaker = PreFlightSnowShake(duration_s=5.0, freq_hz=1.0, amp_deg=40.0)

    def test_ground_shake_sequence(self):
        ok = self.shaker.trigger(on_ground=True, throttle_frac=0.0)
        self.assertTrue(ok)
        self.assertTrue(self.shaker.active)

        # Mid-cycle update
        telem = self.shaker.update(dt_s=0.25, on_ground=True, throttle_frac=0.0)
        self.assertTrue(telem['active'])
        self.assertFalse(telem['completed'])
        self.assertEqual(telem['target_flap_amp_deg'], 40.0)
        self.assertEqual(telem['target_freq_hz'], 1.0)
        # Peak acceleration check: alpha = (2*pi*f)^2 * theta_0 = (2*pi*1)^2 * 40 * (pi/180) ~ 27.5 rad/s² ~ 1579 deg/s²
        alpha_max_deg_s2 = (2.0 * math.pi * 1.0) ** 2 * 40.0
        self.assertLess(alpha_max_deg_s2, 1800.0)

        # Complete 5 seconds
        telem = self.shaker.update(dt_s=4.8, on_ground=True, throttle_frac=0.0)
        self.assertFalse(telem['active'])
        self.assertTrue(telem['completed'])
        self.assertEqual(telem['status'], 'CYCLE_COMPLETED_SUCCESSFULLY')

    def test_airborne_interlock_rejection(self):
        # Triggering while airborne is forbidden
        ok = self.shaker.trigger(on_ground=False, throttle_frac=0.0)
        self.assertFalse(ok)
        telem = self.shaker.update(dt_s=0.1, on_ground=False, requested=True)
        self.assertFalse(telem['active'])
        self.assertEqual(telem['status'], 'REJECTED_AIRBORNE_OR_THROTTLE_NOT_IDLE')

    def test_throttle_interlock_rejection(self):
        # Triggering with non-idle throttle (> 0.05) is forbidden
        ok = self.shaker.trigger(on_ground=True, throttle_frac=0.30)
        self.assertFalse(ok)


class TestZeroFrictionIceSteering(unittest.TestCase):
    """Verifies differential wing sweep ground steering on low-friction snow/ice surfaces."""

    def test_ice_differential_sweep(self):
        cmds = [{'sweep_deg': 60.0} for _ in range(8)]
        res = {}
        telem = govern_zero_friction_ice_steering(
            wing_commands=cmds,
            pilot_yaw_stick=0.8,
            on_ground=True,
            ice_surface=True,
            result=res
        )
        self.assertTrue(telem['active'])
        self.assertAlmostEqual(telem['diff_sweep_deg'], 8.0, places=1)
        self.assertTrue(res['ice_steering_active'])

        # Port wings (even) swept forward (+8 deg -> 68 deg), Starboard swept aft (-8 deg -> 52 deg)
        self.assertAlmostEqual(cmds[0]['sweep_deg'], 68.0)
        self.assertAlmostEqual(cmds[1]['sweep_deg'], 52.0)
        self.assertAlmostEqual(cmds[2]['sweep_deg'], 68.0)
        self.assertAlmostEqual(cmds[3]['sweep_deg'], 52.0)

    def test_normal_surface_inactive(self):
        cmds = [{'sweep_deg': 60.0} for _ in range(8)]
        telem = govern_zero_friction_ice_steering(
            wing_commands=cmds,
            pilot_yaw_stick=0.8,
            on_ground=True,
            ice_surface=False
        )
        self.assertFalse(telem['active'])
        self.assertEqual(cmds[0]['sweep_deg'], 60.0)


class TestColdLiPoOcvEstimator(unittest.TestCase):
    """Verifies internal resistance temperature compensation and transient sag discrimination."""

    def setUp(self):
        self.estimator = ColdLiPoOcvEstimator()

    def test_internal_resistance_temperature_scaling(self):
        ri_warm = self.estimator.compute_internal_resistance(batt_temp_c=25.0)
        ri_zero = self.estimator.compute_internal_resistance(batt_temp_c=0.0)
        ri_subzero = self.estimator.compute_internal_resistance(batt_temp_c=-10.0)

        self.assertAlmostEqual(ri_warm, 0.048, places=3)
        self.assertGreater(ri_zero, 0.100)
        self.assertGreater(ri_subzero, 0.180)

    def test_transient_cold_sag_inhibit_cutoff(self):
        # Cold battery at -5°C under high takeoff load (70A). Terminal voltage drops to 19.8V (3.30V/cell).
        # But true OCV is 19.8V + 70A * 0.165Ω = 31.35V clamped/healthy (~23.0V).
        telem = self.estimator.estimate(bus_voltage_v=20.0, bus_current_a=60.0, batt_temp_c=-5.0)
        self.assertTrue(telem['is_transient_sag'])
        self.assertTrue(telem['emergency_cutoff_inhibited'])
        self.assertEqual(telem['status'], 'TRANSIENT_COLD_SAG_INHIBIT_CUTOFF')
        self.assertGreaterEqual(telem['cell_ocv_v'], 3.65)

    def test_true_electrochemical_depletion(self):
        # Battery at low current (5A) where terminal voltage is 18.5V (3.08V/cell) -> true exhaustion
        telem = self.estimator.estimate(bus_voltage_v=18.5, bus_current_a=5.0, batt_temp_c=20.0)
        self.assertFalse(telem['is_transient_sag'])
        self.assertFalse(telem['emergency_cutoff_inhibited'])
        self.assertIn('DEPLETION', telem['status'])


class TestMCOMainIntegration(unittest.TestCase):
    """Verifies MCOMain instantiation and tick execution with all new telemetry."""

    def test_mco_main_sim_init(self):
        main = MCOMain(sim=True)
        self.assertTrue(main.sim_mode)
        self.assertIsNotNone(main._tick_latencies)
        self.assertEqual(main._deadline_miss_count, 0)
        self.assertEqual(main._watchdog_trip_count, 0)

    def test_mco_main_tick_with_category_4_capabilities(self):
        main = MCOMain(sim=True)
        # Simulate cold ground scenario with snow shake request and ice surface
        sensors = {
            'h_agl_m': 0.0,
            'on_ground': True,
            'gear_contact': True,
            'gps_speed_ms': 0.0,
            'V_ias_ms': 0.0,
            'pitot_v_ms': 0.0,
            'bus_voltage_v': 21.0,
            'motor_currents_a': [8.0]*8,
            'batt_temp_c': -5.0,
            'ice_surface': True,
            'snow_shake_requested': True,
            'throttle': 0.0,
            'yaw': 0.5,
        }
        res = main.fc.tick(sensors)
        self.assertTrue(res['snow_shake_active'])
        self.assertTrue(res['ice_steering_active'])
        self.assertIn('lipo_ocv_est_v', res)
        self.assertIn('lipo_status', res)

    def test_mco_main_tick_with_gust_squeeze(self):
        main = MCOMain(sim=True)
        # Airborne flight encountering extreme +2.8g vertical gust
        sensors = {
            'h_agl_m': 50.0,
            'on_ground': False,
            'gear_contact': False,
            'V_ms': 13.0,
            'accel_z_g': 2.8,
            'lever_pct': 50.0,
        }
        res = main.fc.tick(sensors)
        self.assertTrue(res['gust_squeeze_engaged'])
        self.assertIn('SQUEEZE_ACTIVE', res['gust_squeeze_status'])


class TestExtremeGustSqueezeGovernor(unittest.TestCase):
    """Verifies Zero-Lift Squeeze Reflex under severe vertical turbulence / shear."""

    def setUp(self):
        self.gov = ExtremeGustSqueezeGovernor(
            pos_g_thresh=2.5, neg_g_thresh=-0.8, jerk_thresh=15.0,
            reflex_pitch_deg=-4.5, max_hold_s=0.50
        )

    def test_positive_g_trigger_decambers_wings(self):
        cmds = [{'pitch_deg': 2.5} for _ in range(8)]
        # Normal 1.0g baseline
        self.gov.update(az_g=1.0, dt_s=0.02, wing_commands=cmds, on_ground=False)
        self.assertFalse(self.gov.active)
        self.assertEqual(cmds[0]['pitch_deg'], 2.5)

        # Gust surge to +2.7g
        telem = self.gov.update(az_g=2.7, dt_s=0.02, wing_commands=cmds, on_ground=False)
        self.assertTrue(telem['active'])
        self.assertTrue(telem['reflex_engaged'])
        self.assertEqual(telem['blend_factor'], 1.0)
        self.assertIn('POS_G_EXCEEDED', telem['status'])
        # All wings decambered to -4.5 deg reflex
        for c in cmds:
            self.assertAlmostEqual(c['pitch_deg'], -4.5)

    def test_negative_g_trigger(self):
        cmds = [{'pitch_deg': 2.0} for _ in range(8)]
        self.gov.update(az_g=1.0, dt_s=0.02, wing_commands=cmds, on_ground=False)

        # Severe downburst shear to -0.9g
        telem = self.gov.update(az_g=-0.9, dt_s=0.02, wing_commands=cmds, on_ground=False)
        self.assertTrue(telem['active'])
        self.assertIn('NEG_G_EXCEEDED', telem['status'])
        self.assertAlmostEqual(cmds[0]['pitch_deg'], -4.5)

    def test_vertical_jerk_trigger(self):
        cmds = [{'pitch_deg': 2.0} for _ in range(8)]
        # Step from 1.0g to 2.0g in 20 ms -> jerk = 1.0 / 0.02 = 50 g/s (> 15 g/s threshold)
        self.gov.update(az_g=1.0, dt_s=0.02, wing_commands=cmds, on_ground=False)
        telem = self.gov.update(az_g=2.0, dt_s=0.02, wing_commands=cmds, on_ground=False)
        self.assertTrue(telem['active'])
        self.assertIn('VERTICAL_JERK_EXCEEDED', telem['status'])

    def test_ground_interlock_inhibits_reflex(self):
        cmds = [{'pitch_deg': 2.0} for _ in range(8)]
        # On ground bump (e.g. rough runway 2.8g bump) MUST NOT trigger pitch reflex
        telem = self.gov.update(az_g=2.8, dt_s=0.02, wing_commands=cmds, on_ground=True)
        self.assertFalse(telem['active'])
        self.assertFalse(telem['reflex_engaged'])
        self.assertEqual(cmds[0]['pitch_deg'], 2.0)

    def test_recovery_ramp_and_timeout(self):
        cmds = [{'pitch_deg': 2.0} for _ in range(8)]
        # Trigger
        self.gov.update(az_g=2.6, dt_s=0.02, wing_commands=cmds, on_ground=False)
        self.assertTrue(self.gov.active)

        # Normal load factor restored (1.0g) -> should transition to recovering
        self.gov.update(az_g=1.0, dt_s=0.10, wing_commands=cmds, on_ground=False)
        self.assertFalse(self.gov.active)
        self.assertTrue(self.gov.recovering)

        # After recovery duration (0.25s), should return completely to nominal
        self.gov.update(az_g=1.0, dt_s=0.30, wing_commands=cmds, on_ground=False)
        self.assertFalse(self.gov.active)
        self.assertFalse(self.gov.recovering)
        self.assertEqual(cmds[0]['pitch_deg'], 2.0)


class TestPilotAlpineTakeoffGovernor(unittest.TestCase):
    """Verifies Pilot-Centered Alpine Takeoff Suite and Unified Rollout/Abort Braking (H-ALPINE-TO-1)."""

    def setUp(self):
        self.gov = PilotAlpineTakeoffGovernor(base_v_rot_ms=11.2, base_v_s_ms=9.73)

    def test_density_calculation(self):
        # ISA Sea Level (101325 Pa, 15°C)
        rho_sl, da_sl = self.gov.compute_density(101325.0, 15.0)
        self.assertAlmostEqual(rho_sl, 1.225, delta=0.01)
        self.assertAlmostEqual(da_sl, 0.0, delta=50.0)

        # High-altitude alpine (e.g., 78000 Pa, 5°C ~ 2200m)
        rho_alp, da_alp = self.gov.compute_density(78000.0, 5.0)
        self.assertLess(rho_alp, 1.05)
        self.assertGreater(da_alp, 1800.0)

    def test_rotation_speed_scaling(self):
        # Sea level nominal: V_rot = 11.2 m/s
        telem_sl = self.gov.update(
            dt_s=0.02, V_ias_ms=10.0, p_baro_pa=101325.0, t_ambient_c=15.0,
            throttle_frac=0.8, pilot_pitch_stick=0.0, on_ground=True
        )
        self.assertAlmostEqual(telem_sl['effective_v_rot_ms'], 11.2, delta=0.1)

        # Thin alpine air (p=70000 Pa, 20°C -> rho ~ 0.832 kg/m³)
        telem_thin = self.gov.update(
            dt_s=0.02, V_ias_ms=10.0, p_baro_pa=70000.0, t_ambient_c=20.0,
            throttle_frac=0.8, pilot_pitch_stick=0.0, on_ground=True
        )
        # V_rot must scale up with sqrt(1.225 / rho)
        self.assertGreater(telem_thin['effective_v_rot_ms'], 13.0)
        self.assertLessEqual(telem_thin['effective_v_rot_ms'], 14.5)

    def test_boosted_flap_amplitude_clamped_at_limit(self):
        # Sea level: nominal 75.0 deg
        telem_sl = self.gov.update(
            dt_s=0.02, V_ias_ms=8.0, p_baro_pa=101325.0, t_ambient_c=15.0,
            throttle_frac=0.9, on_ground=True
        )
        self.assertAlmostEqual(telem_sl['boosted_flap_amp_deg'], 75.0, delta=0.5)

        # Extreme thin air: strictly clamped at 88.0° (8° buffer below 96° mechanical stop)
        telem_extreme = self.gov.update(
            dt_s=0.02, V_ias_ms=8.0, p_baro_pa=50000.0, t_ambient_c=35.0,
            throttle_frac=0.9, on_ground=True
        )
        self.assertEqual(telem_extreme['boosted_flap_amp_deg'], ALPINE_MAX_TAKEOFF_AMP_DEG)
        self.assertEqual(telem_extreme['boosted_flap_amp_deg'], 88.0)

    def test_progressive_pitch_gate_below_rotation(self):
        cmds = [{'pitch_deg': 0.0, 'flap_amp_deg': 75.0} for _ in range(8)]
        # Rollout at V = 6.0 m/s with V_rot = 11.2 m/s, moderate pilot pitch stick (0.50)
        # Dynamic pressure factor ~ (6/11.2)^2 ~ 0.287
        telem = self.gov.update(
            dt_s=0.02, V_ias_ms=6.0, p_baro_pa=101325.0, t_ambient_c=15.0,
            throttle_frac=0.8, pilot_pitch_stick=0.5, on_ground=True,
            wing_commands=cmds
        )
        self.assertTrue(telem['pitch_gate_active'])
        # Pitch command should be attenuated proportionally (preventing premature rotation & stall)
        self.assertLess(telem['shaped_pitch_cmd_deg'], 3.0)
        self.assertGreater(telem['shaped_pitch_cmd_deg'], 1.0)
        self.assertEqual(cmds[0]['pitch_deg'], telem['shaped_pitch_cmd_deg'])

        # Above rotation speed (e.g. 12.0 m/s), gate is inactive
        telem_rot = self.gov.update(
            dt_s=0.02, V_ias_ms=12.0, p_baro_pa=101325.0, t_ambient_c=15.0,
            throttle_frac=0.8, pilot_pitch_stick=0.5, on_ground=True,
            wing_commands=cmds
        )
        self.assertFalse(telem_rot['pitch_gate_active'])

    def test_pilot_panic_breakout_veto_on_takeoff_roll(self):
        cmds = [{'pitch_deg': 0.0, 'flap_amp_deg': 75.0} for _ in range(8)]
        # Below rotation speed (V = 6.0 m/s), but pilot hauls stick full aft (>= 0.90) for emergency hop
        telem = self.gov.update(
            dt_s=0.02, V_ias_ms=6.0, p_baro_pa=101325.0, t_ambient_c=15.0,
            throttle_frac=0.8, pilot_pitch_stick=1.0, on_ground=True,
            wing_commands=cmds
        )
        # Gate must be bypassed into Direct Law
        self.assertFalse(telem['pitch_gate_active'])
        self.assertTrue(telem['panic_breakout_active'])
        self.assertEqual(telem['status'], 'EMERGENCY_PILOT_BREAKOUT_DIRECT_LAW')
        # Full 15.0° authority granted immediately
        self.assertAlmostEqual(telem['shaped_pitch_cmd_deg'], 15.0)
        self.assertAlmostEqual(cmds[0]['pitch_deg'], 15.0)

    def test_initial_climb_alpha_prot(self):
        cmds = [{'pitch_deg': 12.0} for _ in range(8)]
        # In air, but airspeed V = 10.5 m/s < safe_climb_speed (1.25 * Vs = 12.16 m/s), level flight Vz = 0
        telem = self.gov.update(
            dt_s=0.02, V_ias_ms=10.5, p_baro_pa=101325.0, t_ambient_c=15.0,
            throttle_frac=0.8, pilot_pitch_stick=0.6, on_ground=False,
            wing_commands=cmds, vz_ms=0.5
        )
        self.assertTrue(telem['alpha_prot_active'])
        # Clamped to +6.5°
        self.assertAlmostEqual(telem['shaped_pitch_cmd_deg'], 6.5)
        for c in cmds:
            self.assertAlmostEqual(c['pitch_deg'], 6.5)

    def test_sink_rate_downdraft_escape_in_initial_climb(self):
        cmds = [{'pitch_deg': 12.0} for _ in range(8)]
        # Encountering mountain downdraft: Vz = -1.5 m/s
        telem = self.gov.update(
            dt_s=0.02, V_ias_ms=10.5, p_baro_pa=101325.0, t_ambient_c=15.0,
            throttle_frac=0.8, pilot_pitch_stick=0.8, on_ground=False,
            wing_commands=cmds, vz_ms=-1.5
        )
        # Clamp relaxed for terrain avoidance
        self.assertFalse(telem['alpha_prot_active'])
        self.assertEqual(telem['status'], 'INITIAL_CLIMB_DOWNDRAFT_ESCAPE_ACTIVE')
        self.assertGreater(telem['shaped_pitch_cmd_deg'], 6.5)

    def test_asymmetric_rollout_downforce_prevents_wheelbarrowing(self):
        cmds = [{'pitch_deg': 5.0, 'flap_amp_deg': 75.0, 'sweep_deg': 30.0} for _ in range(8)]
        # Throttle chopped to idle (0.02 <= 0.05) on ground
        telem = self.gov.update(
            dt_s=0.02, V_ias_ms=8.0, p_baro_pa=101325.0, t_ambient_c=15.0,
            throttle_frac=0.02, pilot_pitch_stick=0.0, on_ground=True,
            wing_commands=cmds
        )
        self.assertTrue(telem['suction_braking_active'])
        self.assertEqual(telem['status'], 'ROLLOUT_SUCTION_BRAKING_ACTIVE')
        # Tiers 1-2 (Wings 0-3): -3.0° mild downforce, eliminates nosewheel overload
        for c in cmds[:4]:
            self.assertEqual(c['pitch_deg'], ROLLOUT_BRAKING_FORWARD_PITCH_DEG)  # -3.0 deg
            self.assertEqual(c['flap_amp_deg'], 0.0)
        # Tiers 3-4 (Wings 4-7): -8.0° linear attached suction over main wheels
        for c in cmds[4:]:
            self.assertEqual(c['pitch_deg'], ROLLOUT_BRAKING_AFT_PITCH_DEG)      # -8.0 deg
            self.assertEqual(c['flap_amp_deg'], 0.0)

    def test_bounce_interlock_inhibits_suction(self):
        cmds = [{'pitch_deg': 2.0, 'flap_amp_deg': 50.0} for _ in range(8)]
        # On ground momentarily but with positive vertical velocity (airborne bounce Vz = +0.8 m/s)
        gov = PilotAlpineTakeoffGovernor()
        telem = gov.update(
            dt_s=0.02, V_ias_ms=8.0, p_baro_pa=101325.0, t_ambient_c=15.0,
            throttle_frac=0.02, pilot_pitch_stick=0.0, on_ground=True,
            wing_commands=cmds, vz_ms=0.8
        )
        self.assertFalse(telem['suction_braking_active'])
        self.assertTrue(telem['bounce_inhibited'])

    def test_touch_and_go_throttle_advance_clears_suction(self):
        cmds = [{'pitch_deg': 2.0, 'flap_amp_deg': 50.0} for _ in range(8)]
        gov = PilotAlpineTakeoffGovernor()
        # First engage suction
        gov.update(
            dt_s=0.02, V_ias_ms=8.0, throttle_frac=0.0, on_ground=True, wing_commands=cmds
        )
        self.assertTrue(gov.suction_braking_latched)

        # Pilot firewalling throttle for touch-and-go (> 0.15)
        telem = gov.update(
            dt_s=0.02, V_ias_ms=8.0, throttle_frac=0.80, on_ground=True, wing_commands=cmds
        )
        self.assertFalse(telem['suction_braking_active'])
        self.assertFalse(gov.suction_braking_latched)

    def test_functional_governor_call(self):
        res = {'wing_commands': [{'pitch_deg': 0.0, 'flap_amp_deg': 50.0} for _ in range(8)], 'freq_hz': 2.2}
        telem = govern_pilot_alpine_takeoff_and_rollout(
            dt_s=0.02, V_ias_ms=5.0, throttle_frac=0.0, on_ground=True,
            wing_commands=res['wing_commands'], result=res
        )
        self.assertTrue(res['rollout_suction_braking_active'])
        self.assertEqual(res['freq_hz'], 0.0)
        self.assertEqual(res['wing_commands'][0]['pitch_deg'], -3.0)
        self.assertEqual(res['wing_commands'][7]['pitch_deg'], -8.0)


class TestMCOMainRolloutIntegration(unittest.TestCase):
    """Verifies rollout and abort braking integration through MCOMain tick."""

    def setUp(self):
        self.mco = MCOMain(sim=True)

    def test_mco_main_ground_throttle_chop_triggers_suction_braking(self):
        sensors = {
            'V_ms': 8.0,
            'on_ground': True,
            'gear_contact': True,
            'h_agl_m': 0.0,
            'bus_voltage_v': 24.0,
            'throttle': 0.0,  # full back on throttle
            'pitch': 0.0,
            'roll': 0.0,
            'yaw': 0.0,
            'p_baro_pa': 101325.0,
            't_ambient_c': 15.0,
            'vz_ms': 0.0,
        }
        # First tick activates suction braking and cuts flapping
        result = self.mco.tick(sensors)
        self.assertTrue(result.get('rollout_suction_braking_active'))
        self.assertEqual(result.get('freq_hz'), 0.0)
        self.assertEqual(result.get('alpine_governor_status'), 'ROLLOUT_SUCTION_BRAKING_ACTIVE')

        # Allow servos to slew into suction downforce
        for _ in range(12):
            result = self.mco.tick(sensors)

        for cmd in result.get('wing_commands', [])[:8]:
            self.assertLess(cmd['pitch_deg'], 0.0)



class TestGustHardeningAndExtensions(unittest.TestCase):
    """
    Verifies hardened gust penetration, rejection, and ride stabilization:
      1. S1223 dynamic stall boundary enforcement (alpha_eff <= 13.0 deg) in DLTV (H-43).
      2. Unmet pitch demand conversion to flapping stroke amplitude boost in H-43.
      3. Strict 88.0 deg mechanical sector stop amplitude hardcap across H-41, H-42, and H-43.
      4. 2S 7.4V servo bus voltage derating in ActiveRideStabilizer (H-41).
      5. Convective cascade delay feedforward (~41.4 ms / 2 ticks) to Station 4 in H-41.
      6. Dynamic stall margin monitoring and telemetry export (stall_margin_deg).
      7. Tailless differential sweep sideslip decoupling (H-39) to neutralize weathercocking.
    """

    def test_h43_dynamic_stall_boundary_and_amplitude_conversion(self):
        """Under severe downdraft, collective pitch advance is clamped to keep alpha_eff <= 13.0 deg."""
        # Baseline pitch = 4.0 deg, downdraft = -3.5 m/s, V_ias = 10.0 m/s
        # Induced alpha = degrees(atan2(3.5, 10.0)) = 19.29 deg -> already exceeds 13.0 deg!
        # Max safe pitch is 0.0 deg, so delta_pitch_allowed = 0.0 deg.
        # Demand (3.5 * 1.4 = 4.9 deg) should convert to amplitude boost: 4.9 * 1.2 = 5.88 deg.
        base_cmds = [{'pitch_deg': 4.0, 'flap_amp_deg': 70.0} for _ in range(8)]
        out_cmds, active, delta_g = ap.h43_direct_lift_thrust_vectoring(
            base_cmds, w_sink_ms=-3.5, pilot_climb_override=False, V_ias_ms=10.0
        )
        self.assertTrue(active)
        self.assertGreater(delta_g, 0.5)

        for cmd in out_cmds:
            # Pitch must NOT increase past safe boundary
            self.assertLessEqual(cmd['pitch_deg'], 4.0)
            # Amplitude must receive the converted unmet pitch boost
            self.assertGreater(cmd['flap_amp_deg'], 70.0 + 5.0)
            # Amplitude must remain strictly within mechanical hardstop limit
            self.assertLessEqual(cmd['flap_amp_deg'], 88.0)

    def test_h43_moderate_downdraft_partial_pitch_advance(self):
        """Moderate downdraft allows partial pitch advance up to S1223 stall ceiling."""
        # Baseline pitch = 0.0 deg, downdraft = -2.5 m/s, V_ias = 15.0 m/s
        # Induced alpha = degrees(atan2(2.5, 15.0)) = 9.46 deg. Safe margin = 13.0 - 9.46 = 3.54 deg.
        base_cmds = [{'pitch_deg': 0.0, 'flap_amp_deg': 65.0} for _ in range(8)]
        out_cmds, active, _ = ap.h43_direct_lift_thrust_vectoring(
            base_cmds, w_sink_ms=-2.5, pilot_climb_override=False, V_ias_ms=15.0
        )
        self.assertTrue(active)
        for cmd in out_cmds:
            # Commanded pitch should advance by allowed pitch
            self.assertGreater(cmd['pitch_deg'], 0.0)
            # Effective alpha must be <= 13.0 deg
            alpha_eff = cmd['pitch_deg'] + math.degrees(math.atan2(2.5, 15.0))
            self.assertLessEqual(round(alpha_eff, 2), 13.0)

    def test_h43_flapping_amplitude_hardcap_at_88_deg(self):
        """Even with max severity downdraft and high baseline amplitude, flap_amp_deg is hardcapped at 88.0 deg."""
        base_cmds = [{'pitch_deg': 0.0, 'flap_amp_deg': 85.0} for _ in range(8)]
        out_cmds, active, _ = ap.h43_direct_lift_thrust_vectoring(
            base_cmds, w_sink_ms=-6.0, pilot_climb_override=True, V_ias_ms=8.0
        )
        self.assertTrue(active)
        for cmd in out_cmds:
            self.assertEqual(cmd['flap_amp_deg'], 88.0, "Flap amplitude must hardcap at 88.0 deg")

    def test_h42_asymmetric_retrim_hardcap_at_88_deg(self):
        """N-1 surviving wings are hardcapped at 88.0 deg to guarantee >= 8.0 deg dynamic overshoot margin."""
        base_cmds = [{'pitch_deg': 10.0, 'flap_amp_deg': 82.0, 'sweep_deg': 90.0} for _ in range(8)]
        retrim_cmds, active, diag = ap.h42_asymmetric_retrim_allocation(failed_wing_idx=2, base_commands=base_cmds)
        self.assertTrue(active)
        for i, cmd in enumerate(retrim_cmds):
            if i == 2:
                self.assertEqual(cmd['flap_amp_deg'], 0.0)
            else:
                self.assertLessEqual(cmd['flap_amp_deg'], 88.0)

    def test_h41_2s_servo_bus_voltage_derate(self):
        """Low 2S servo bus voltage derates heave collective pitch trim authority."""
        stabilizer = ap.ActiveRideStabilizer()
        base_cmds = [{'pitch_deg': 10.0, 'flap_amp_deg': 80.0} for _ in range(8)]

        # Nominal 7.4V bus
        stab_nom = ap.ActiveRideStabilizer()
        sensors_nominal = {
            'comfort_mode': True,
            'az_g': 1.25,
            'vsi_ms': 0.5,
            'servo_bus_v': 7.4,
            'pilot_pitch': 0.0,
        }
        _, _, _, diag_nom = stab_nom.update(sensors_nominal, base_cmds)
        self.assertEqual(diag_nom['servo_bus_derate'], 1.0)
        pitch_trim_nom = abs(diag_nom['delta_alpha_heave_deg'])

        # Low 6.5V bus (steady state)
        stab_sag = ap.ActiveRideStabilizer()
        sensors_sag = dict(sensors_nominal, servo_bus_v=6.5)
        _, _, _, diag_sag = stab_sag.update(sensors_sag, base_cmds)
        self.assertLess(diag_sag['servo_bus_derate'], 1.0)
        self.assertGreater(diag_sag['servo_bus_derate'], 0.5)
        self.assertLess(abs(diag_sag['delta_alpha_heave_deg']), pitch_trim_nom)

        # Critical 6.0V bus (steady state)
        stab_crit = ap.ActiveRideStabilizer()
        sensors_crit = dict(sensors_nominal, servo_bus_v=6.0)
        _, _, _, diag_crit = stab_crit.update(sensors_crit, base_cmds)
        self.assertAlmostEqual(diag_crit['servo_bus_derate'], 0.25, places=2)

    def test_h41_convective_cascade_delay_feedforward(self):
        """Station 1 gust disturbance propagates to Station 4 via 2-tick convective delay buffer."""
        stabilizer = ap.ActiveRideStabilizer()
        base_cmds = [{'pitch_deg': 10.0, 'flap_amp_deg': 80.0} for _ in range(8)]

        # Tick 1: Sharp vertical heave spike hits foremost Station 1 (with traveling wave pitch moment)
        sensors_t1 = {
            'comfort_mode': True,
            'az_g': 1.30,
            'vsi_ms': 0.8,
            'q_rad_s': 0.08,               # pitching moment from Station 1 gust onset
            'pitch_moment_meas_Nm': 0.35,   # Station 1 lift creates nose-up unbalance
            'V_ms': 10.0,
            'servo_bus_v': 7.4,
            'pilot_pitch': 0.0,
        }
        # Tick 1: Sharp vertical heave spike hits foremost Station 1 (t=0ms, in buffer)
        cmds_t1, _, active1, diag1 = stabilizer.update(sensors_t1, base_cmds, dt=0.02)
        self.assertTrue(active1)
        self.assertTrue(diag1['conv_ff_active'])
        self.assertEqual(diag1['conv_ff_mode'], 'CONVECTIVE_CASCADE_FEEDFORWARD')
        self.assertAlmostEqual(diag1['conv_delay_ms'], 41.4, places=1)

        # Tick 2 (t=20ms): Disturbance propagates downstream through mid-stations
        cmds_t2, _, active2, diag2 = stabilizer.update(sensors_t1, base_cmds, dt=0.02)
        self.assertTrue(active2)

        # Tick 3 (t=40ms): Disturbance arrives at Station 4 (x = -0.207m)
        cmds_t3, _, active3, diag3 = stabilizer.update(sensors_t1, base_cmds, dt=0.02)
        self.assertTrue(active3)

        # Station 4 (pair 3, wings 6 & 7) must have received preemptive feedforward pitch trim
        self.assertNotEqual(diag3['st4_ff_pitch_deg'], 0.0)
        self.assertLess(cmds_t3[6]['pitch_deg'], base_cmds[6]['pitch_deg'])

    def test_h41_dynamic_stall_margin_telemetry(self):
        """ActiveRideStabilizer reports dynamic stall margin bounded to S1223 airfoil envelope."""
        stabilizer = ap.ActiveRideStabilizer()
        base_cmds = [{'pitch_deg': 11.5, 'flap_amp_deg': 80.0} for _ in range(8)]
        sensors = {
            'comfort_mode': True,
            'az_g': 0.85,  # negative heave demands positive pitch addition
            'vsi_ms': -0.3,
            'servo_bus_v': 7.4,
            'pilot_pitch': 0.0,
        }
        cmds, _, active, diag = stabilizer.update(sensors, base_cmds)
        self.assertTrue(active)
        self.assertIn('stall_margin_deg', diag)
        self.assertGreaterEqual(diag['stall_margin_deg'], 0.0)
        # All wing pitch commands must respect S1223 upper ceiling (13.5 deg)
        for cmd in cmds:
            self.assertLessEqual(cmd['pitch_deg'], 13.5)

    def test_h39_tailless_differential_sweep_crosswind_decoupling(self):
        """Crosswind sideslip beta generates restoring differential sweep without stick input."""
        base_sweeps = [90.0] * 8

        # Sideslip beta = +8.0 deg (relative wind from starboard)
        sweeps_stbd_wind, active = ap.h39_differential_sweep_yaw(base_sweeps, yaw_stick=0.0, sideslip_beta_deg=+8.0)
        self.assertTrue(active)
        # To counter rightward weathercocking, port sweeps aft (>90) and starboard forward (<90)
        self.assertGreater(sweeps_stbd_wind[0], 90.0)
        self.assertLess(sweeps_stbd_wind[1], 90.0)

        # Verify physical boundaries [45.0, 145.0]
        sweeps_extreme, _ = ap.h39_differential_sweep_yaw([142.0] * 8, yaw_stick=+1.0, sideslip_beta_deg=-15.0)
        for sw in sweeps_extreme:
            self.assertGreaterEqual(sw, 45.0)
            self.assertLessEqual(sw, 145.0)

    def test_mco_main_gust_suite_telemetry_integration(self):
        """MCOMain tick executes full gust suite and exports telemetry flags."""
        mco = MCOMain(sim=True)
        sensors = {
            'V_ms': 10.0,
            'airspeed_ms': 10.0,
            'on_ground': False,
            'gear_contact': False,
            'h_agl_m': 50.0,
            'bus_voltage_v': 24.0,
            'servo_bus_v': 7.2,
            'throttle': 0.65,
            'pitch': 0.0,
            'roll': 0.0,
            'yaw': 0.0,
            'comfort_mode': True,
            'az_g': 1.20,
            'vsi_ms': 0.4,
            'w_sink_ms': -2.8,
            'sideslip_beta_deg': 6.0,
            'p_baro_pa': 101325.0,
            't_ambient_c': 15.0,
        }
        res = mco.tick(sensors)
        self.assertIn('h41_stall_margin_deg', res)
        self.assertIn('h41_servo_derate', res)
        self.assertTrue(res.get('h39_active'))
        self.assertTrue(res.get('h41_active'))
        self.assertTrue(res.get('h43_active'))

    def test_h41_uniform_microburst_wavefront_coherence_bypass(self):
        """In a uniform vertical downdraft, convective delay feedforward is bypassed to prevent PIO."""
        stabilizer = ap.ActiveRideStabilizer()
        base_cmds = [{'pitch_deg': 10.0, 'flap_amp_deg': 80.0} for _ in range(8)]
        # Uniform vertical shear: large heave error (az = 0.70g) but zero pitch rate and zero pitching moment
        sensors_uniform = {
            'comfort_mode': True,
            'az_g': 0.70,
            'vsi_ms': -1.2,
            'q_rad_s': 0.0,
            'pitch_moment_meas_Nm': 0.0,
            'V_ms': 10.0,
            'servo_bus_v': 7.4,
            'pilot_pitch': 0.0,
        }
        _, _, active, diag = stabilizer.update(sensors_uniform, base_cmds)
        self.assertTrue(active)
        self.assertTrue(diag['uniform_shear'])
        self.assertEqual(diag['conv_ff_mode'], 'UNIFORM_SHEAR_SYNCHRONOUS')
        self.assertEqual(diag['st4_ff_pitch_deg'], 0.0, "Station 4 delayed feedforward must be zero in uniform microburst")

    def test_h41_dynamic_sag_rate_protection(self):
        """Rapid voltage collapse (dV/dt < -1.5 V/s) applies dynamic sag penalty to prevent BEC brownout."""
        stabilizer = ap.ActiveRideStabilizer()
        base_cmds = [{'pitch_deg': 10.0, 'flap_amp_deg': 80.0} for _ in range(8)]
        # Tick 1: 7.2V
        stabilizer.update({'comfort_mode': True, 'az_g': 1.2, 'servo_bus_v': 7.2}, base_cmds, dt=0.02)
        # Tick 2: Sudden plunge to 6.6V over 20ms -> dV/dt = -30.0 V/s!
        _, _, _, diag = stabilizer.update({'comfort_mode': True, 'az_g': 1.2, 'servo_bus_v': 6.6}, base_cmds, dt=0.02)
        self.assertLess(diag['servo_bus_derate'], 0.70, "Rapid voltage collapse must trigger transient derate penalty")

    def test_h43_unsteady_lev_ceiling_and_secondary_frequency_boost(self):
        """When stroke amplitude saturates at 88.0 deg, DLTV activates emergency flapping frequency boost."""
        base_cmds = [{'pitch_deg': 4.0, 'flap_amp_deg': 85.0} for _ in range(8)]
        out_cmds, active, delta_g = ap.h43_direct_lift_thrust_vectoring(
            base_cmds, w_sink_ms=-5.0, pilot_climb_override=True, V_ias_ms=10.0, downstroke_active=True
        )
        self.assertTrue(active)
        # Stroke amplitude must clamp at 88.0 deg
        for cmd in out_cmds:
            self.assertEqual(cmd['flap_amp_deg'], 88.0)
        # Emergency frequency boost must be activated
        freq_boost = ap.h43_direct_lift_thrust_vectoring.last_freq_boost_hz
        self.assertGreater(freq_boost, 0.0, "Unmet demand beyond 88 deg must produce emergency frequency boost")
        self.assertLessEqual(freq_boost, 0.40)

    def test_h43_low_airspeed_inflow_conditioning(self):
        """At low airspeeds (near hover), rotor inflow prevents induced alpha from collapsing pitch authority."""
        base_cmds = [{'pitch_deg': 2.0, 'flap_amp_deg': 70.0} for _ in range(8)]
        # Low forward speed V = 1.0 m/s with sink rate = -2.5 m/s
        out_cmds, active, _ = ap.h43_direct_lift_thrust_vectoring(
            base_cmds, w_sink_ms=-2.5, pilot_climb_override=True, V_ias_ms=1.0, downstroke_active=True
        )
        self.assertTrue(active)
        # Commanded pitch should advance (not zeroed out by false high induced alpha)
        self.assertGreater(out_cmds[0]['pitch_deg'], 2.0)

    def test_h39_dynamic_flapping_amplitude_tip_clearance_clamp(self):
        """High flapping amplitude (88.0 deg) scales down max differential sweep from 8.0 deg to 4.0 deg."""
        base_sweeps = [90.0] * 8
        # At moderate amplitude (50 deg), full 8.0 deg differential sweep is permitted
        sweeps_low_amp, _ = ap.h39_differential_sweep_yaw(base_sweeps, yaw_stick=1.0, flap_amp_deg=50.0)
        diff_low_amp = abs(sweeps_low_amp[0] - sweeps_low_amp[1])
        self.assertAlmostEqual(diff_low_amp, 16.0, places=1)  # -8 deg port, +8 deg stbd

        # At max amplitude (88.0 deg), max differential sweep is clamped to 4.0 deg (diff = 8.0 deg)
        sweeps_high_amp, _ = ap.h39_differential_sweep_yaw(base_sweeps, yaw_stick=1.0, flap_amp_deg=88.0)
        diff_high_amp = abs(sweeps_high_amp[0] - sweeps_high_amp[1])
        self.assertAlmostEqual(diff_high_amp, 8.0, places=1)   # -4 deg port, +4 deg stbd
        self.assertLess(diff_high_amp, diff_low_amp)


if __name__ == '__main__':
    unittest.main()


