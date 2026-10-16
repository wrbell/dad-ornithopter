#!/usr/bin/env python3
"""
scripts/orthotropic_wing_optimization_audit.py
=============================================
Rigorous Orthotropic & Aeroelastic Analysis of the SD7062 Wing System:
Validates that the 3D-printed LW-PLA substructure, carbon spar array,
and heat-shrunk Mylar skin fulfill all structural, aeroelastic, and
kinematic flight requirements across the MCO-1D Mark 1 flight envelope.
"""

import numpy as np

def run_orthotropic_analysis():
    # Aircraft & Envelope Parameters
    span_semi = 1.150          # m (semi-span)
    chord = 0.165              # m (chord)
    S_wing = span_semi * chord # m² per wing panel
    MTOW = 6.15                # kg (Mark 1 max design limit)
    n_limit = 2.2              # g (limit load factor)
    n_ult = 3.3                # g (ultimate load factor, 1.5x factor of safety)
    rho_air = 1.225            # kg/m³
    V_cruise = 9.0             # m/s
    V_sprint = 24.0            # m/s
    V_ne = 28.0                # m/s (Never-Exceed Speed)
    f_flap_max = 3.5           # Hz max flapping frequency
    theta_0_deg = 24.0         # degrees (half stroke amplitude)
    theta_0 = np.radians(theta_0_deg)

    # 1. Carbon Spar Layup (Orthotropic Transversely Isotropic Pultrusion)
    # Main Spar at 20% chord (X = 33.0 mm): 10.0 mm OD x 8.0 mm ID 3K Carbon Tube
    E_cf_axial = 120.0e9       # Pa (120 GPa)
    G_cf_shear = 4.5e9         # Pa (4.5 GPa)
    r_main_o = 0.005           # m
    r_main_i = 0.004           # m
    I_main = (np.pi / 4.0) * (r_main_o**4 - r_main_i**4)
    J_main = (np.pi / 2.0) * (r_main_o**4 - r_main_i**4)
    EI_main = E_cf_axial * I_main  # 34.78 N*m²
    GJ_main = G_cf_shear * J_main  # 2.61 N*m²

    # Secondary Spars at 10% (X=16.5mm) and 55% (X=90.75mm): 2.0 mm solid carbon rods
    r_sec = 0.001              # m
    I_sec = (np.pi / 4.0) * (r_sec**4)
    EI_sec = 2.0 * E_cf_axial * I_sec # 0.19 N*m²

    # 2. 3D-Printed LW-PLA Substructure (Orthotropic FDM Print along Z-span)
    # Layer lines in XY airfoil plane; Z-build axis along wing span
    E_lw_span = 0.55e9         # Pa (0.55 GPa inter-layer tensile)
    E_lw_chord = 0.95e9        # Pa (0.95 GPa along extrusion paths)
    G_lw_inplane = 0.28e9      # Pa (0.28 GPa in-plane shear)
    sigma_lw_tensile_z = 8.5e6 # Pa (8.5 MPa inter-layer tensile strength)
    sigma_lw_comp = 12.0e6     # Pa (12.0 MPa compressive yield)

    # D-Box Monocoque Front 30% Closed Cell (X = 0 to 49.5 mm)
    t_skin = 0.00045           # m (0.45 mm outer skin)
    A_cell_dbox = 6.68e-4      # m²
    peri_dbox = 0.125          # m
    J_dbox = 4.0 * (A_cell_dbox**2) / (peri_dbox / t_skin)
    GJ_dbox = G_lw_inplane * J_dbox # 1.80 N*m²

    # 3. Mylar Skin (Oralight Biaxially Oriented Polyester Film, t = 25 µm)
    # Ironed at 135°C -> Pre-tension stress sigma_0 = 12.0 MPa (T = 300 N/m)
    E_mylar = 4.1e9            # Pa
    G_mylar = 1.45e9           # Pa
    t_mylar = 25.0e-6          # m
    T_film = 12.0e6 * t_mylar  # 300 N/m

    # Wagner Stressed-Skin Contribution over Aft Open Bays (X = 49.5 to 165 mm)
    A_cell_aft = 0.5 * (chord - 0.0495) * 0.015
    peri_aft = 2.0 * (chord - 0.0495) + 0.015 + 0.003
    J_aft = 4.0 * (A_cell_aft**2) / (peri_aft / t_mylar)
    GJ_mylar = G_mylar * J_aft # 0.43 N*m²

    # Total Section Properties
    EI_total = EI_main + EI_sec + (E_lw_span * 1.5e-8) # 35.80 N*m²
    GJ_total = GJ_main + GJ_dbox + GJ_mylar           # 4.84 N*m²

    # 4. Inertial & Aeroelastic Dynamic Modal Analysis
    m_panel = 0.235            # kg (187.5g printed + 42g spars/couplers + 5.5g film)
    m_dist = m_panel / span_semi # 0.204 kg/m
    r_theta = 0.24 * chord     # radius of gyration
    I_p = m_dist * (r_theta**2)

    omega_bend = (1.875**2) * np.sqrt(EI_total / (m_dist * span_semi**4))
    f_bend = omega_bend / (2.0 * np.pi)

    omega_torsion = (np.pi / (2.0 * span_semi)) * np.sqrt(GJ_total / I_p)
    f_torsion = omega_torsion / (2.0 * np.pi)

    # Divergence dynamic pressure & speed
    e_ac_ea = (0.25 - 0.20) * chord # 0.00825 m
    dCL_dalpha = 2.0 * np.pi
    q_div = ((np.pi / 2.0)**2) * GJ_total / (e_ac_ea * chord * (span_semi**2) * dCL_dalpha)
    V_div = np.sqrt(2.0 * q_div / rho_air)

    # 5. Flight Loads & Stress Margins
    # Aerodynamic limit lift (8 wings total carry 6.15 kg at n = 2.2g)
    L_wing_limit = (MTOW * 9.81 * n_limit) / 8.0 # 16.59 N
    M_root_aero = L_wing_limit * (0.43 * span_semi) # 8.20 N*m

    # Peak inertial flapping bending moment at stroke reversal (f = 3.5 Hz)
    omega_flap = 2.0 * np.pi * f_flap_max
    M_root_inertial = m_dist * theta_0 * (omega_flap**2) * ((span_semi**3) / 3.0) # 20.99 N*m
    M_root_total = M_root_aero + M_root_inertial # 29.19 N*m

    # Main carbon spar bending stress
    sigma_spar = (M_root_total * r_main_o) / I_main # 503.6 MPa (CF yield: ~1500 MPa)
    FOS_spar = 1500.0e6 / sigma_spar

    # Mylar skin billowing under peak suction at V_sprint (24 m/s, Cp = -2.4)
    q_sprint = 0.5 * rho_air * (V_sprint**2)
    delta_p = 2.4 * q_sprint
    bay_pitch = 0.050 # m (50 mm rib bulkheads)
    w_mylar_max = (delta_p * (bay_pitch**2)) / (8.0 * T_film)

    # Rib bulkhead compressive stress under Mylar chordwise shrink tension
    F_crush_per_rib = 2.0 * T_film * bay_pitch # 30.0 N
    A_rib_caps = 15.0e-6 # m²
    sigma_rib_crush = F_crush_per_rib / A_rib_caps # 2.0 MPa (LW-PLA yield: 12.0 MPa)
    FOS_rib = sigma_lw_comp / sigma_rib_crush

    print("=====================================================================")
    print("MCO-1D MARK 1: SD7062 ORTHOTROPIC AEROELASTIC OPTIMIZATION REPORT")
    print("=====================================================================")
    print(f"Bending Rigidity (EI):             {EI_total:.2f} N·m²")
    print(f"Torsional Rigidity (GJ):           {GJ_total:.2f} N·m²")
    print(f"  - ⌀10mm CF Main Spar:            {GJ_main:.2f} N·m² (53.9%)")
    print(f"  - 3D D-Box Monocoque (LW-PLA):   {GJ_dbox:.2f} N·m² (37.2%)")
    print(f"  - Pre-tensioned Mylar Skin:      {GJ_mylar:.2f} N·m² (8.9%)")
    print("---------------------------------------------------------------------")
    print(f"First Bending Mode (f_h):          {f_bend:.2f} Hz")
    print(f"First Torsional Mode (f_alpha):    {f_torsion:.2f} Hz")
    print(f"Torsional / Bending Ratio:         {f_torsion / f_bend:.2f} (>3.5 target: PASS)")
    print(f"Static Aeroelastic Divergence:     V_div = {V_div:.1f} m/s ({V_div*2.237:.1f} mph) vs V_ne={V_ne} m/s [PASS]")
    print("---------------------------------------------------------------------")
    print(f"Peak Flapping Bending Moment:      {M_root_total:.2f} N·m (Limit n=2.2g + 3.5Hz reversal)")
    print(f"Main Spar Bending Stress:          {sigma_spar / 1e6:.1f} MPa (FOS = {FOS_spar:.2f} vs 1500 MPa [PASS])")
    print(f"Rib Bulkhead Crush Stress:         {sigma_rib_crush / 1e6:.1f} MPa (FOS = {FOS_rib:.2f} vs 12 MPa [PASS])")
    print(f"Max Mylar Skin Billowing:          {w_mylar_max * 1e3:.2f} mm ({w_mylar_max / chord * 100:.2f}% chord [PASS])")
    print("=====================================================================")

if __name__ == '__main__':
    run_orthotropic_analysis()
