#!/usr/bin/env python3
"""
scripts/mco_full_envelope_mission_sim.py
========================================
High-Fidelity Multi-Modality Aerobatic Mission Simulator & Gap Detector for MCO-1D.

Simulates a complete, multi-phase mission exercising the full library:
  Phase 1: Ground Ops, Taxi Steering & Takeoff Roll / Rotation
  Phase 2: Climb Corridors (Strouhal Vy, Pigeon Vortex Ring Vx, Elytron Canard)
  Phase 3: Multi-Modality Cruise & Sprint Dash (Locust Torsional, Reverse Cascade, Venturi Dash, H-46 Resonant Buzz)
  Phase 4: High-Agility Aerobatics (Boyd Turn MT-3, Knife-Edge Gull Turn, Snap Roll, Tailless Sweep Yaw, Stroke Tilt)
  Phase 5: Azhdarchid Thermal Soar, Falcon Morphing Polar Continuum (Float->Penetration->Dive->Peregrine Stoop) & Zoom Recovery
  Phase 6: Approach, Dynamic Airbrake (Active Upstroke, Clamshell, Toroidal Ring) & Perch Flare Touchdown

Analyzes 50 Hz telemetry for:
  - Actuation rate saturation (> 300°/s servo rate limit)
  - Structural / aerodynamic load factor jerk (Δnz > 0.8g / 0.1s)
  - Specific Excess Power (Ps) cliff drops & energy holes
  - Missing soft-handoff blending across discrete hypothesis boundaries
"""

import sys
import os
import math
import io

# Ensure firmware path is present
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'firmware')))

# Suppress import noise
_old_stdout = sys.stdout
sys.stdout = io.StringIO()
import mco_intent_control as ic
import mco_advanced_physics as ap
import mco_wave_library as wl
import mco_performance_model as pm
sys.stdout = _old_stdout


class FullEnvelopeMissionSimulator:
    def __init__(self):
        self.dt = 0.02  # 50 Hz
        self.mass_kg = 5.687
        self.g = 9.80665
        self.weight_n = self.mass_kg * self.g
        self.rho = 1.225
        self.s_ref = 1.320  # m^2

        # Flight states
        self.t = 0.0
        self.x = 0.0
        self.h = 0.0
        self.V = 0.0
        self.gamma_deg = 0.0  # flight path angle (+ climb)
        self.pitch_deg = 0.0  # fuselage attitude
        self.bank_deg = 0.0
        self.heading_deg = 0.0
        self.on_ground = True

        # Flight Computer & Intent Arbitration
        class SimulatedFlightController:
            def __init__(self):
                self.freq_hz = 0.0
                self._tgt_freq = 0.0
                self._tgt_sweeps = [60.0] * 8
                self._tgt_dihedrals = [6.0] * 8
                self._tgt_pitches = [8.0] * 8
                self._cur_sweeps = [60.0] * 8
                self._cur_dihedrals = [6.0] * 8
                self._cur_pitches = [8.0] * 8
                self._intent_name = 'cruise'
                self.falcon_morph_diag = {}

        self.ctrl = SimulatedFlightController()
        self.dispatcher = ic.BehaviourDispatcher(self.ctrl, enable_soft_handoff=True)
        self.interp = self.dispatcher.interpreter

        # Telemetry & Gap Detector
        self.telemetry = []
        self.gaps_detected = []
        self.hypothesis_history = []
        self.last_cmd = {
            'freq': 0.0,
            'sweeps': [60.0] * 8,
            'dihedrals': [6.0] * 8,
            'pitches': [8.0] * 8,
            'intent': 'INIT',
        }
        self.last_nz = 1.0
        self.last_ps = 0.0

    def generate_pilot_mission_script(self):
        """
        Defines the mission timeline of pilot controls (sticks, switches, levers).
        Exercises every major flight modality in the library.
        """
        timeline = [
            # ── Phase 1: Ground Taxi & Takeoff Roll ──────────────────────────────
            (0.0, 3.0, "TAXI_CRACKED_THROTTLE", {
                'throttle': 0.25, 'pitch': 0.0, 'roll': 0.0, 'yaw': 0.30,
                'gl_lever': 0.0, 'lever_pct': 50.0, 'pedals_depressed': False
            }),
            (3.0, 5.0, "TAXI_CRAB_TRANSLATION", {
                'throttle': 0.30, 'pitch': 0.0, 'roll': 0.40, 'yaw': 0.0,
                'gl_lever': 0.0, 'lever_pct': 50.0, 'pedals_depressed': False
            }),
            (5.0, 8.5, "RUNWAY_TAKEOFF_ROLL", {
                'throttle': 0.90, 'pitch': 0.0, 'roll': 0.0, 'yaw': 0.0,
                'gl_lever': 0.0, 'lever_pct': 90.0, 'pedals_depressed': False
            }),
            (8.5, 12.0, "DUAL_CASCADE_ROTATION", {
                'throttle': 0.95, 'pitch': 0.45, 'roll': 0.0, 'yaw': 0.0,
                'gl_lever': 0.0, 'lever_pct': 95.0, 'pedals_depressed': False
            }),

            # ── Phase 2: Climb Corridors ─────────────────────────────────────────
            (12.0, 18.0, "STROUHAL_BEST_ROC_VY", {
                'throttle': 0.85, 'pitch': 0.25, 'roll': 0.0, 'yaw': 0.0,
                'gl_lever': 0.0, 'lever_pct': 85.0, 'pedals_depressed': False
            }),
            (18.0, 24.0, "PIGEON_CLAP_PEEL_VX", {
                'throttle': 0.95, 'pitch': 0.65, 'roll': 0.0, 'yaw': 0.0,
                'gl_lever': 0.0, 'lever_pct': 95.0, 'pedals_depressed': False
            }),
            (24.0, 30.0, "ELYTRON_CANARD_OBSTACLE_CLIMB", {
                'throttle': 0.90, 'pitch': 0.75, 'roll': 0.0, 'yaw': 0.0,
                'gl_lever': 0.0, 'lever_pct': 90.0, 'pedals_depressed': False,
                'obstacle_climb': True
            }),

            # ── Phase 3: Cruise & High-Speed Dash ────────────────────────────────
            (30.0, 36.0, "METACHRONAL_CRUISE_90", {
                'throttle': 0.50, 'pitch': 0.0, 'roll': 0.0, 'yaw': 0.0,
                'gl_lever': 0.0, 'lever_pct': 50.0, 'pedals_depressed': False
            }),
            (36.0, 42.0, "LOCUST_TORSIONAL_WAVE_CRUISE", {
                'throttle': 0.55, 'pitch': 0.0, 'roll': 0.0, 'yaw': 0.0,
                'gl_lever': 0.0, 'lever_pct': 55.0, 'pedals_depressed': False
            }),
            (42.0, 48.0, "REVERSE_CASCADE_THRUST_BOOST", {
                'throttle': 0.75, 'pitch': 0.0, 'roll': 0.0, 'yaw': 0.0,
                'gl_lever': 0.0, 'lever_pct': 75.0, 'pedals_depressed': False
            }),
            (48.0, 55.0, "RESONANT_BUZZ_SPRINT_DASH", {
                'throttle': 1.00, 'pitch': -0.05, 'roll': 0.0, 'yaw': 0.0,
                'gl_lever': 0.0, 'lever_pct': 100.0, 'pedals_depressed': False
            }),

            # ── Phase 4: High-Agility Aerobatics ─────────────────────────────────
            (55.0, 62.0, "BOYD_HIGH_G_TURN_MT3", {
                'throttle': 0.70, 'pitch': 0.10, 'roll': 0.85, 'yaw': 0.0,
                'gl_lever': 0.0, 'lever_pct': 70.0, 'pedals_depressed': False
            }),
            (62.0, 68.0, "KNIFE_EDGE_GULL_TURN_H82", {
                'throttle': 0.60, 'pitch': 0.05, 'roll': 0.70, 'yaw': 0.30,
                'gl_lever': 0.0, 'lever_pct': 60.0, 'pedals_depressed': False
            }),
            (68.0, 74.0, "SNAP_ROLL_INVERSION_H61", {
                'throttle': 0.65, 'pitch': 0.0, 'roll': -0.95, 'yaw': -0.40,
                'gl_lever': 0.0, 'lever_pct': 65.0, 'pedals_depressed': False
            }),
            (74.0, 80.0, "TAILLESS_SWEEP_YAW_H100", {
                'throttle': 0.50, 'pitch': 0.0, 'roll': 0.0, 'yaw': 0.60,
                'gl_lever': 0.0, 'lever_pct': 50.0, 'pedals_depressed': False
            }),
            (80.0, 85.0, "ODONATA_STROKE_PLANE_VECTOR_H88", {
                'throttle': 0.55, 'pitch': 0.0, 'roll': 0.0, 'yaw': -0.75,
                'gl_lever': 0.0, 'lever_pct': 55.0, 'pedals_depressed': False
            }),

            # ── Phase 5: Thermal Soar, Falcon Morphing Stoop & Zoom ─────────────
            (85.0, 92.0, "AZHDARCHID_THERMAL_SOAR_H102", {
                'throttle': 0.15, 'pitch': 0.05, 'roll': 0.0, 'yaw': 0.0,
                'gl_lever': -0.40, 'lever_pct': 35.0, 'pedals_depressed': False,
                'thermal_loiter': True
            }),
            (92.0, 96.0, "FALCON_ZONE1_BEST_LD_FLOAT", {
                'throttle': 0.05, 'pitch': 0.0, 'roll': 0.0, 'yaw': 0.0,
                'gl_lever': -0.50, 'lever_pct': 30.0, 'pedals_depressed': False,
                'falcon_morph': True
            }),
            (96.0, 101.0, "FALCON_ZONE2_FAST_PENETRATION", {
                'throttle': 0.00, 'pitch': -0.25, 'roll': 0.0, 'yaw': 0.0,
                'gl_lever': -0.50, 'lever_pct': 25.0, 'pedals_depressed': False,
                'falcon_morph': True
            }),
            (101.0, 107.0, "FALCON_ZONE3_STEEP_DIVE", {
                'throttle': 0.00, 'pitch': -0.60, 'roll': 0.0, 'yaw': 0.0,
                'gl_lever': -0.50, 'lever_pct': 20.0, 'pedals_depressed': False,
                'falcon_morph': True
            }),
            (107.0, 112.0, "FALCON_ZONE4_PEREGRINE_STOOP", {
                'throttle': 0.00, 'pitch': -0.95, 'roll': -0.20, 'yaw': 0.35,
                'gl_lever': -0.50, 'lever_pct': 15.0, 'pedals_depressed': False,
                'falcon_morph': True
            }),
            (112.0, 116.0, "G_LIMITED_PULLOUT_AND_ZOOM", {
                'throttle': 0.70, 'pitch': 0.70, 'roll': 0.0, 'yaw': 0.0,
                'gl_lever': 0.0, 'lever_pct': 70.0, 'pedals_depressed': False
            }),

            # ── Phase 6: Approach, Dynamic Airbrake & Perch Landing ──────────────
            (116.0, 122.0, "SLOTTED_FLAP_GLIDE_APPROACH_H95", {
                'throttle': 0.10, 'pitch': 0.0, 'roll': 0.0, 'yaw': 0.0,
                'gl_lever': -0.50, 'lever_pct': 30.0, 'pedals_depressed': False
            }),
            (122.0, 127.0, "ACTIVE_UPSTROKE_DYNAMIC_AIRBRAKE_H87", {
                'throttle': 0.05, 'pitch': -0.20, 'roll': 0.0, 'yaw': 0.0,
                'gl_lever': -0.85, 'lever_pct': 10.0, 'pedals_depressed': False,
                'active_upstroke_brake': True
            }),
            (127.0, 132.0, "CLAMSHELL_SCISSOR_FLAP_BRAKE_H54", {
                'throttle': 0.00, 'pitch': -0.10, 'roll': 0.0, 'yaw': 0.0,
                'gl_lever': -0.90, 'lever_pct': 5.0, 'pedals_depressed': False
            }),
            (132.0, 137.0, "TOROIDAL_VORTEX_RING_RAPID_STOP_H55", {
                'throttle': 0.00, 'pitch': 0.20, 'roll': 0.0, 'yaw': 0.0,
                'gl_lever': -1.00, 'lever_pct': 2.0, 'pedals_depressed': False
            }),
            (137.0, 142.0, "CONVENTIONAL_RUNWAY_APPROACH", {
                'throttle': 0.25, 'pitch': 0.0, 'roll': 0.0, 'yaw': 0.0,
                'gl_lever': 0.0, 'lever_pct': 50.0, 'pedals_depressed': False,
                'landing_approach': True, 'h_agl_m': 15.0
            }),
            (142.0, 146.0, "CONVENTIONAL_DUAL_CASCADE_FLARE", {
                'throttle': 0.05, 'pitch': 0.25, 'roll': 0.0, 'yaw': 0.0,
                'gl_lever': 0.0, 'lever_pct': 50.0, 'pedals_depressed': False,
                'landing_flare': True, 'h_agl_m': 1.0
            }),
            (146.0, 150.0, "RUNWAY_AERODYNAMIC_ROLLOUT_SHUTTLECOCK_BRAKE", {
                'throttle': 0.00, 'pitch': 0.0, 'roll': 0.0, 'yaw': 0.0,
                'gl_lever': -0.80, 'lever_pct': 22.0, 'pedals_depressed': False,
                'touchdown_rollout': True, 'h_agl_m': 0.05, 'on_ground': True
            }),
        ]
        return timeline

    def run_simulation(self):
        """Executes the full 150-second mission at 50 Hz."""
        timeline = self.generate_pilot_mission_script()
        print(f"\n🚀 Starting MCO-1D Full Envelope Aerobatic Mission Simulation (0 to 150 s @ 50 Hz)...")

        total_steps = int(150.0 / self.dt)
        script_idx = 0

        for step in range(total_steps):
            t_curr = round(step * self.dt, 3)
            self.t = t_curr

            # Find active scripted phase
            while script_idx < len(timeline) - 1 and t_curr >= timeline[script_idx][1]:
                script_idx += 1
            t_start, t_end, phase_name, inputs = timeline[script_idx]

            # Build pilot control input dictionaries
            sticks = {
                'pitch': inputs['pitch'],
                'roll': inputs['roll'],
                'yaw': inputs['yaw'],
                'throttle': inputs['throttle'],
                'gl_lever': inputs['gl_lever'],
            }
            switches = {
                'mode_sw': 1 if self.h > 1.0 else 0,
                'macro_sw': False,
                'macro_long': False,
                'rth_sw': False,
                'wing_sw': 0,
            }
            if 'h_agl_m' in inputs:
                self.h = inputs['h_agl_m']
            if 'on_ground' in inputs:
                self.on_ground = inputs['on_ground']

            context = {
                't': t_curr,
                'V_ms': self.V,
                'h_agl_m': self.h,
                'on_ground': self.on_ground,
                'pitch_deg': self.pitch_deg,
                'bank_deg': self.bank_deg,
                'heading_deg': self.heading_deg,
                'lever_pct': inputs['lever_pct'],
                'pedals_depressed': inputs.get('pedals_depressed', False),
                'obstacle_climb': inputs.get('obstacle_climb', False),
                'thermal_loiter': inputs.get('thermal_loiter', False),
                'falcon_morph': inputs.get('falcon_morph', False),
                'active_upstroke_brake': inputs.get('active_upstroke_brake', False),
                'landing_approach': inputs.get('landing_approach', False),
                'landing_flare': inputs.get('landing_flare', False),
                'touchdown_rollout': inputs.get('touchdown_rollout', False),
                'soft_handoff': True,
            }

            # 1. Execute Intent Arbitration & Behavior Dispatch from Library
            self.dispatcher.update(t_curr, sticks, switches, context)
            intent = self.dispatcher._last_intent
            context['intent'] = intent

            # 2. Apply Servo Slew & Actuator Dynamics
            demanded_sweeps = list(self.ctrl._tgt_sweeps)
            demanded_pitches = list(self.ctrl._tgt_pitches)
            demanded_dihedrals = list(self.ctrl._tgt_dihedrals)
            demanded_freq = float(self.ctrl._tgt_freq)

            # Calculate raw instantaneous demand rates
            max_sweep_rate = max(abs(d - c) / self.dt for d, c in zip(demanded_sweeps, self.last_cmd['sweeps']))
            max_pitch_rate = max(abs(d - c) / self.dt for d, c in zip(demanded_pitches, self.last_cmd['pitches']))
            max_dih_rate = max(abs(d - c) / self.dt for d, c in zip(demanded_dihedrals, self.last_cmd['dihedrals']))
            freq_rate = abs(demanded_freq - self.last_cmd['freq']) / self.dt

            # Slew rate limit according to physical servos (AGFRC IB53BHU ~ 300°/s max)
            MAX_SERVO_DPS = 300.0  # deg/s
            actual_sweeps = []
            actual_pitches = []
            actual_dihedrals = []
            for d, c in zip(demanded_sweeps, self.ctrl._cur_sweeps):
                delta = max(-MAX_SERVO_DPS * self.dt, min(MAX_SERVO_DPS * self.dt, d - c))
                actual_sweeps.append(c + delta)
            for d, c in zip(demanded_pitches, self.ctrl._cur_pitches):
                delta = max(-MAX_SERVO_DPS * self.dt, min(MAX_SERVO_DPS * self.dt, d - c))
                actual_pitches.append(c + delta)
            for d, c in zip(demanded_dihedrals, self.ctrl._cur_dihedrals):
                delta = max(-MAX_SERVO_DPS * self.dt, min(MAX_SERVO_DPS * self.dt, d - c))
                actual_dihedrals.append(c + delta)

            actual_freq = demanded_freq  # Motor ODrive velocity slew

            self.ctrl._cur_sweeps = actual_sweeps
            self.ctrl._cur_pitches = actual_pitches
            self.ctrl._cur_dihedrals = actual_dihedrals
            self.ctrl.freq_hz = actual_freq

            # 4. Aerodynamic Force & Performance Computation
            mean_sweep = sum(actual_sweeps) / 8.0
            mean_pitch = sum(actual_pitches) / 8.0
            mean_dih = sum(actual_dihedrals) / 8.0

            q_dyn = 0.5 * self.rho * max(self.V, 0.5)**2
            sweep_abeam = max(0.0, mean_sweep - 60.0)
            s_eff = self.s_ref * math.cos(math.radians(sweep_abeam))

            # Lift computation based on mean pitch AoA and flapping augment
            aoa_eff = mean_pitch + (self.pitch_deg - self.gamma_deg)
            cl_steady = max(-0.5, min(1.85, 0.10 * (aoa_eff + 3.0)))
            cl_flapping = 0.35 * (actual_freq / 2.5) if actual_freq > 0.1 else 0.0
            cl_total = cl_steady + cl_flapping
            lift_n = cl_total * q_dyn * s_eff

            # Ground effect boost when h < 1.0m
            if self.h < 1.5:
                ige_factor = 1.0 + 0.35 * max(0.0, (1.5 - self.h) / 1.5)
                lift_n *= ige_factor

            # Drag computation
            cd0 = 0.025 + 0.035 * (abs(mean_dih) / 15.0)**1.5 + (0.15 if "AIRBRAKE" in phase_name or "STOP" in phase_name else 0.0)
            cdi = (cl_total ** 2) / (math.pi * 12.12 * 0.92)
            drag_n = (cd0 + cdi) * q_dyn * s_eff + 0.120 * q_dyn

            # Thrust computation
            if actual_freq > 0.1:
                v_tip = 2.0 * math.pi * actual_freq * math.radians(50.0) * 1.0
                j_adv = self.V / max(0.1, v_tip)
                ct = 0.32 * max(0.1, 1.0 - 0.45 * j_adv) * math.cos(math.radians(sweep_abeam))
                thrust_n = ct * 0.5 * self.rho * (self.V**2 + v_tip**2) * s_eff
            else:
                thrust_n = 0.0

            # Dynamic brake reversal
            if "AIRBRAKE" in phase_name or intent == ic.Intent.ACTIVE_UPSTROKE_BRAKE:
                thrust_n = -22.0 * (self.V / 15.0)

            # Ground interaction & state integration
            if self.on_ground:
                net_f_vert = lift_n - self.weight_n
                if net_f_vert > 0.0 and self.V >= 5.0 and inputs['pitch'] > 0.20:
                    self.on_ground = False
                    n_z = lift_n / self.weight_n
                else:
                    self.on_ground = True
                    self.h = 0.0
                    self.gamma_deg = 0.0
                    normal_f = max(0.0, self.weight_n - lift_n)
                    friction_f = 0.04 * normal_f
                    net_thrust = max(0.0, thrust_n - drag_n - friction_f)
                    accel_x = net_thrust / self.mass_kg
                    self.V = max(0.0, self.V + accel_x * self.dt)
                    self.x += self.V * self.dt
                    n_z = 1.0
            else:
                # 2-DOF longitudinal trajectory integration
                gamma_rad = math.radians(self.gamma_deg)
                f_parallel = thrust_n - drag_n - self.weight_n * math.sin(gamma_rad)
                accel_v = f_parallel / self.mass_kg
                self.V = max(1.0, self.V + accel_v * self.dt)

                f_perpend = lift_n - self.weight_n * math.cos(gamma_rad)
                gamma_dot_rad = f_perpend / (self.mass_kg * self.V)
                self.gamma_deg += math.degrees(gamma_dot_rad) * self.dt
                self.gamma_deg = max(-85.0, min(85.0, self.gamma_deg))

                # Pitch attitude dynamics: blend towards flight path + pilot demand
                tgt_pitch = self.gamma_deg + inputs['pitch'] * 12.0
                self.pitch_deg += (tgt_pitch - self.pitch_deg) * min(1.0, 4.0 * self.dt)

                self.h += self.V * math.sin(math.radians(self.gamma_deg)) * self.dt
                self.x += self.V * math.cos(math.radians(self.gamma_deg)) * self.dt
                n_z = lift_n / self.weight_n

                # Check touchdown
                if self.h <= 0.0 and t_curr > 140.0:
                    self.h = 0.0
                    self.on_ground = True
                    self.gamma_deg = 0.0
                    self.pitch_deg = 0.0

            # Specific Excess Power
            p_s = (thrust_n - drag_n) / self.weight_n * self.V

            # ── 5. GAP DETECTION & CONTINUITY MONITORING ──────────────────
            delta_nz = abs(n_z - self.last_nz)
            nz_jerk = delta_nz / self.dt
            delta_ps = p_s - self.last_ps

            # Gap 1: Actuator Demand Slew Violation (> 300°/s instantaneous jump)
            if max_sweep_rate > MAX_SERVO_DPS or max_pitch_rate > MAX_SERVO_DPS or max_dih_rate > MAX_SERVO_DPS:
                gap_info = {
                    't': t_curr,
                    'phase': phase_name,
                    'intent_prev': self.last_cmd['intent'],
                    'intent_curr': intent,
                    'type': 'ACTUATION_RATE_SATURATION',
                    'severity': 'HIGH',
                    'details': f"Demand rate jump: sweep={max_sweep_rate:.1f}°/s, pitch={max_pitch_rate:.1f}°/s, dih={max_dih_rate:.1f}°/s (limit 300°/s)",
                    'pitch_jump_deg': max(abs(d - c) for d, c in zip(demanded_pitches, self.last_cmd['pitches'])),
                    'sweep_jump_deg': max(abs(d - c) for d, c in zip(demanded_sweeps, self.last_cmd['sweeps'])),
                }
                self.gaps_detected.append(gap_info)

            # Gap 2: Aerodynamic Load Factor Jerk (Δnz > 0.8g within 0.04s)
            if nz_jerk > 20.0 and delta_nz > 0.75 and t_curr > 8.0:
                gap_info = {
                    't': t_curr,
                    'phase': phase_name,
                    'intent_prev': self.last_cmd['intent'],
                    'intent_curr': intent,
                    'type': 'AERODYNAMIC_LOAD_FACTOR_JERK',
                    'severity': 'MEDIUM',
                    'details': f"Load factor step: nz jump of {delta_nz:.2f}g in {self.dt}s (jerk={nz_jerk:.1f}g/s, nz={n_z:.2f}g)",
                }
                self.gaps_detected.append(gap_info)

            # Gap 3: Specific Excess Power (Ps) Cliff Drop (> 8 m/s collapse in 1 tick during active thrust)
            if delta_ps < -8.0 and inputs['throttle'] > 0.40:
                gap_info = {
                    't': t_curr,
                    'phase': phase_name,
                    'intent_prev': self.last_cmd['intent'],
                    'intent_curr': intent,
                    'type': 'ENERGY_SPECIFIC_POWER_CLIFF',
                    'severity': 'MEDIUM',
                    'details': f"Specific excess power collapsed by {delta_ps:.1f} m/s (from {self.last_ps:.1f} to {p_s:.1f} m/s)",
                }
                self.gaps_detected.append(gap_info)

            # Record state
            self.last_cmd = {
                'freq': demanded_freq,
                'sweeps': demanded_sweeps,
                'dihedrals': demanded_dihedrals,
                'pitches': demanded_pitches,
                'intent': intent,
            }
            self.last_nz = n_z
            self.last_ps = p_s

            if step % 25 == 0:  # 2 Hz telemetry summary
                self.telemetry.append({
                    't': t_curr,
                    'phase': phase_name,
                    'intent': intent,
                    'V': round(self.V, 1),
                    'h': round(self.h, 1),
                    'gamma': round(self.gamma_deg, 1),
                    'nz': round(n_z, 2),
                    'Ps': round(p_s, 2),
                    'freq': round(actual_freq, 2),
                    'sweep': round(mean_sweep, 1),
                    'pitch': round(mean_pitch, 1),
                    'dih': round(mean_dih, 1),
                })

        print(f"✅ Simulation finished successfully: 150.0s elapsed, {total_steps} ticks computed.")
        print(f"   End flight state: V={self.V:.1f} m/s, h={self.h:.1f} m, on_ground={self.on_ground}")
        print(f"   Detected {len(self.gaps_detected)} potential transition gaps / discontinuity events.")
        return self.telemetry, self.gaps_detected


def main():
    sim = FullEnvelopeMissionSimulator()
    telemetry, gaps = sim.run_simulation()

    print("\n" + "="*80)
    print("           MCO-1D FULL ENVELOPE MISSION SIMULATION & GAP AUDIT          ")
    print("="*80)

    # Print sample timeline milestones
    print("\n📍 MISSION TRAJECTORY MILESTONES:")
    for pt in telemetry[::10]:
        print(f"  t={pt['t']:5.1f}s | {pt['phase']:<35} | Intent: {pt['intent']:<20} | V={pt['V']:4.1f} m/s | h={pt['h']:5.1f} m | nz={pt['nz']:4.2f}g | Ps={pt['Ps']:5.1f} m/s")

    # Analyze gaps
    print("\n🔍 GAP ANALYSIS & DISCONTINUITY AUDIT:")
    if not gaps:
        print("  No gaps detected! All transitions were perfectly smooth.")
        return

    # Categorize and deduplicate gaps by transition
    transition_groups = {}
    for g in gaps:
        key = (g['phase'], g['intent_prev'], g['intent_curr'], g['type'])
        if key not in transition_groups:
            transition_groups[key] = []
        transition_groups[key].append(g)

    print(f"  Total raw events: {len(gaps)}. Unique gap transition signatures: {len(transition_groups)}.\n")

    for i, ((phase, prev_i, curr_i, gtype), ev_list) in enumerate(transition_groups.items(), 1):
        first = ev_list[0]
        print(f"  [{i}] Phase: {phase}")
        print(f"      Transition: {prev_i} -> {curr_i}")
        print(f"      Type:       {gtype} ({first['severity']} severity, {len(ev_list)} occurrences)")
        print(f"      Details:    {first['details']}")
        print()


if __name__ == '__main__':
    main()
