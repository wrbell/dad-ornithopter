#!/usr/bin/env python3
"""
scripts/calculate_wrist_servo_flight_requirements.py
===================================================
Rigorous aerodynamic and dynamic flight calculation to size the two servos
for the 2-DOF merged sector-wheel wrist joint of MCO-1D.

Degrees of Freedom:
  DOF 1: Flapping (driven by Big Drum / capstan motor)
  DOF 2: Wing Feathering / Pitch AoA (-10° to +40°, dynamic reciprocating)
  DOF 3: Wing Sweep / Folding (-10° forward to +90° aft stow)
"""

import math

def calculate_flight_requirements():
    # Aircraft & Atmospheric Parameters
    rho = 1.225            # kg/m^3 (sea level air density)
    V_stall = 10.5         # m/s
    V_cruise = 15.0        # m/s
    V_dash = 20.0          # m/s (dive / dash envelope)
    f_flap = 2.2           # Hz (max continuous flapping frequency)
    omega_flap = 2 * math.pi * f_flap # rad/s (~13.82 rad/s)
    
    # Wing Blade Parameters (per wing, 8 wings total)
    span_blade = 0.380     # m (380mm active blade span)
    root_chord = 0.120     # m (120mm root chord, S1223 airfoil)
    tip_chord = 0.080      # m (80mm tip chord)
    mean_chord = 0.100     # m (100mm mean aerodynamic chord MAC)
    area_blade = span_blade * mean_chord # 0.038 m^2
    
    # Mass & Inertia Properties of Wing Blade
    m_blade = 0.048        # kg (48g lightweight carbon spar + LW-PLA ribs + Oracover)
    r_cg_chord = 0.032     # m (CG is 32mm aft of leading edge)
    r_spar = 0.030         # m (main spar pivot is at 25% MAC = 30mm aft of LE)
    delta_cg_pitch = abs(r_cg_chord - r_spar) # 0.002 m offset
    
    # Moment of inertia about spar axis (Pitch)
    I_pitch = m_blade * ((mean_chord ** 2) / 12.0 + delta_cg_pitch ** 2) # ~ 4.0e-5 kg*m^2
    
    # Moment of inertia about sweep pivot
    I_sweep = (1.0 / 3.0) * m_blade * (span_blade ** 2) # ~ 2.31e-3 kg*m^2
    
    # --------------------------------------------------------------------------
    # 1. PITCH (FEATHERING / ANGLE OF ATTACK) TORQUE CALCULATION
    # --------------------------------------------------------------------------
    # Aerodynamic pitching moment:
    # M_aero = 0.5 * rho * V^2 * S * c * Cm
    # S1223 has high camber, Cm0 ≈ -0.12 at zero lift, up to Cm ≈ -0.16 at high alpha.
    Cm_pitch = 0.15
    q_dash = 0.5 * rho * (V_dash ** 2) # Dynamic pressure at 20 m/s = 245 Pa
    M_aero_pitch = q_dash * area_blade * mean_chord * Cm_pitch # in N*m
    
    # Dynamic gust & flow separation buffer (gust factor 1.5x)
    M_aero_pitch_peak = M_aero_pitch * 1.50
    
    # Dynamic inertial torque for pitch reversal at 2.2 Hz flapping
    # Pitch amplitude theta_pitch_amp = ±15° = 0.262 rad
    theta_pitch_amp = math.radians(15.0)
    alpha_pitch_accel = theta_pitch_amp * (omega_flap ** 2) # rad/s^2 (50.0 rad/s^2)
    tau_inertial_pitch = I_pitch * alpha_pitch_accel
    
    # Total required peak pitch torque
    tau_total_pitch = (M_aero_pitch_peak + tau_inertial_pitch) * 1.50 # 1.5x safety margin
    
    # Speed requirement for pitch
    # Must traverse 30° in 1/4 flapping cycle = 1 / (4 * 2.2) = 0.114 s
    # Angular velocity = 30° / 0.114 s = 264 °/s (unloaded rating should be >= 450 °/s, i.e. <= 0.13 s / 60°)
    slew_speed_pitch_req = 60.0 / (264.0) # s/60° = 0.22 s/60° max allowable
    
    # --------------------------------------------------------------------------
    # 2. SWEEP (FOLDING & DRAG TRIM) TORQUE CALCULATION
    # --------------------------------------------------------------------------
    # Aerodynamic drag moment about vertical hinge:
    # Drag force at max dash speed V_dash = 20 m/s with Cd = 0.08
    Cd_dash = 0.08
    D_dash = q_dash * area_blade * Cd_dash # ~ 0.745 N
    r_drag_cp = span_blade * 0.45          # Center of pressure at 45% span ~ 0.171 m
    M_aero_drag = D_dash * r_drag_cp       # in N*m
    
    # Folding against airflow during emergency stow or high-speed dive:
    # Wing folded back 60°..85°, lift vector projects into sweep moment
    Cl_stow = 0.40
    L_dash = q_dash * area_blade * Cl_stow # ~ 3.72 N
    # When swept at 30°, component along chord:
    M_sweep_coupled = (L_dash * math.sin(math.radians(20.0)) + D_dash) * r_drag_cp
    
    # Flapping cross-inertial Coriolis and chordwise reversal loads:
    # During 2.2 Hz flapping stroke (amplitude ±45°), chordwise acceleration
    # a_chord = omega_flap^2 * r_cg * sin(theta_flap)
    tau_inertial_sweep = I_sweep * 12.0 # rad/s^2 dynamic chordwise vibration
    
    tau_total_sweep = (M_sweep_coupled + tau_inertial_sweep) * 1.60 # 1.6x safety margin
    
    # Convert N*m to kg*cm: 1 N*m = 10.197 kg*cm
    pitch_kgcm = tau_total_pitch * 10.197
    sweep_kgcm = tau_total_sweep * 10.197
    
    print("=" * 72)
    print("  MCO-1D 2-DOF MERGED WRIST SERVO FLIGHT REQUIREMENTS AUDIT")
    print("=" * 72)
    print(f"Airspeed Envelope: V_stall={V_stall:.1f} m/s, V_cruise={V_cruise:.1f} m/s, V_dash={V_dash:.1f} m/s")
    print(f"Flapping Dynamics: f={f_flap:.2f} Hz, omega={omega_flap:.2f} rad/s")
    print(f"Blade Properties : Span={span_blade*1e3:.0f} mm, MAC={mean_chord*1e3:.0f} mm, Mass={m_blade*1e3:.1f} g")
    print("-" * 72)
    print(f"PITCH (FEATHERING) DEMAND:")
    print(f"  Aerodynamic Pitch Moment (Dash+Gust): {M_aero_pitch_peak:.3f} N*m")
    print(f"  Inertial Reversal Torque           : {tau_inertial_pitch:.4f} N*m")
    print(f"  Total Factored Pitch Torque (1.5x) : {tau_total_pitch:.3f} N*m ({pitch_kgcm:.2f} kg*cm)")
    print(f"  Minimum Dynamic Slew Speed         : <= 0.13 s / 60° (unloaded)")
    print("-" * 72)
    print(f"SWEEP (FOLD / TRIM) DEMAND:")
    print(f"  Aerodynamic Drag Moment (Dash)     : {M_aero_drag:.3f} N*m")
    print(f"  Coupled Folding Moment (Dive Stow) : {M_sweep_coupled:.3f} N*m")
    print(f"  Flap-Coriolis Dynamic Inertia      : {tau_inertial_sweep:.3f} N*m")
    print(f"  Total Factored Sweep Torque (1.6x) : {tau_total_sweep:.3f} N*m ({sweep_kgcm:.2f} kg*cm)")
    print(f"  Stow / Hold Authority Limit        : >= 85° mechanical sweep")
    print("=" * 72)

    # --------------------------------------------------------------------------
    # 3. EVALUATION OF CANDIDATE SERVOS
    # --------------------------------------------------------------------------
    servos = [
        {
            "name": "AGFRC B24CLM (Micro Coreless)",
            "mass_g": 21.0,
            "torque_8_4v_kgcm": 7.0,
            "speed_8_4v_s60": 0.065,
            "dims_mm": (23.0, 12.0, 27.5),
            "gear_type": "Steel / Titanium",
            "motor": "Coreless"
        },
        {
            "name": "AGFRC B44BLS (Mini Brushless)",
            "mass_g": 44.0,
            "torque_8_4v_kgcm": 15.0,
            "speed_8_4v_s60": 0.055,
            "dims_mm": (35.5, 15.0, 28.5),
            "gear_type": "Hardened Steel",
            "motor": "Brushless"
        },
        {
            "name": "KST X10 Mini (Thin Wing)",
            "mass_g": 23.0,
            "torque_8_4v_kgcm": 7.5,
            "speed_8_4v_s60": 0.080,
            "dims_mm": (30.0, 10.0, 29.6),
            "gear_type": "Hardened Steel",
            "motor": "Coreless"
        },
        {
            "name": "AGFRC A50BHM (Low-Profile Standard)",
            "mass_g": 52.0,
            "torque_8_4v_kgcm": 22.0,
            "speed_8_4v_s60": 0.065,
            "dims_mm": (40.8, 20.2, 26.5),
            "gear_type": "Steel",
            "motor": "Brushless"
        }
    ]

    print("\nCANDIDATE SERVO COMPARISON & MARGIN EVALUATION (With 2:1 Differential Bevel Reduction):")
    print(f"{'Servo Model':<32} | {'Mass (ea)':<9} | {'Stall Torque':<12} | {'Joint Torque (2:1)':<18} | {'Pitch SF':<8} | {'Sweep SF':<8}")
    print("-" * 98)
    for s in servos:
        joint_torque_kgcm = 2.0 * s["torque_8_4v_kgcm"] * 2.0 # 2 servos in differential + 2:1 bevel ratio
        sf_pitch = joint_torque_kgcm / pitch_kgcm
        sf_sweep = joint_torque_kgcm / sweep_kgcm
        print(f"{s['name']:<32} | {s['mass_g']:>6.1f} g  | {s['torque_8_4v_kgcm']:>6.1f} kg*cm | {joint_torque_kgcm:>8.1f} kg*cm    | {sf_pitch:>6.1f}x  | {sf_sweep:>6.1f}x")
    print("=" * 98)

if __name__ == '__main__':
    calculate_flight_requirements()
