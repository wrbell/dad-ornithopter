#!/usr/bin/env python3
"""
plot_flight_envelope.py — MCO-1D Mark 1 Flight Envelope Generation Script

Calculates and renders publication-quality figures for:
  1. V-n Diagram (Velocity - Load Factor Structural Envelope)
  2. Altitude vs Airspeed Operational Corridor (Service & Transition Envelope)
  3. Combined Multi-Panel Flight Performance Envelope

Saves vector PDF and high-res PNG formats to:
  docs/mark1/worksheets/
"""

import os
import math
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon

# Output directory
OUTPUT_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "docs", "mark1", "worksheets"))
os.makedirs(OUTPUT_DIR, exist_ok=True)

# ─── Aircraft Baseline Physics ────────────────────────────────────────────────
MTOW_KG       = 5.687
W_NEWTONS     = MTOW_KG * 9.80665     # 55.77 N
WING_AREA_M2  = 0.7124                # 8 wings total projected area
RHO_0         = 1.225                 # Sea level density kg/m^3

# Aerodynamic Lift Limits
CL_MAX_PASSIVE   = 1.45               # Unpowered fixed-wing clean stall
CL_MAX_ACTIVE    = 1.85               # Active flapping with LEV & wake capture
CL_MAX_IGE       = 2.15               # Flapping with ground effect
CL_MIN_REFLEX    = -0.60              # Inverted / negative AoA reflex
CL_CRUISE_CLEAN  = 1.05               # Nominal clean cruise polar

# Structural Load Factor Limits
N_LIMIT_POS      = 2.20               # +2.2g Limit Load (8.6 SF against spar flexure)
N_ULTIMATE_POS   = N_LIMIT_POS * 1.5  # +3.3g Ultimate Load
N_LIMIT_NEG      = -0.80              # -0.8g Negative Limit Load

# Airspeed Limits (m/s)
V_STALL_PASSIVE  = math.sqrt((2 * W_NEWTONS) / (RHO_0 * WING_AREA_M2 * CL_MAX_PASSIVE)) # 9.38 m/s
V_STALL_ACTIVE   = math.sqrt((2 * W_NEWTONS) / (RHO_0 * WING_AREA_M2 * CL_MAX_ACTIVE))  # 8.30 m/s
V_UNSTICK_IGE    = 6.20               # Ground roll unstick with IGE
V_CRUISE_NOMINAL = 9.00               # Nominal STOL cruise
V_FAST_CRUISE    = 12.00              # Fast cruise
V_A_CORNER_ACT   = math.sqrt((2 * N_LIMIT_POS * W_NEWTONS) / (RHO_0 * WING_AREA_M2 * CL_MAX_ACTIVE)) # 12.31 m/s
V_A_CORNER_PAS   = math.sqrt((2 * N_LIMIT_POS * W_NEWTONS) / (RHO_0 * WING_AREA_M2 * CL_MAX_PASSIVE)) # 13.91 m/s
V_NE             = 16.00              # 16.0 m/s (Never-Exceed / ODrive 40 rps ceiling)
V_GUST_LIMIT     = 18.00              # Structural design dive limit

# Ceiling Limits
CEILING_SERVICE_M  = 2500.0           # Service ceiling (100 fpm / 0.5 m/s climb)
CEILING_ABSOLUTE_M = 3200.0           # Absolute ceiling

def std_density(h_m):
    """ICAO standard atmosphere density at altitude h."""
    temp_ratio = 1.0 - 2.25577e-5 * h_m
    if temp_ratio <= 0:
        return 0.1
    return RHO_0 * (temp_ratio ** 4.25588)

# ─── 1. Plot V-n Diagram ──────────────────────────────────────────────────────
def plot_vn_diagram():
    fig, ax = plt.subplots(figsize=(10, 6.5), dpi=300)
    
    # Airspeed range for positive stall curves
    v_stall_curve_act = np.linspace(0.01, V_A_CORNER_ACT, 200)
    n_stall_act = 0.5 * RHO_0 * (v_stall_curve_act ** 2) * WING_AREA_M2 * CL_MAX_ACTIVE / W_NEWTONS

    v_stall_curve_pas = np.linspace(0.01, V_A_CORNER_PAS, 200)
    n_stall_pas = 0.5 * RHO_0 * (v_stall_curve_pas ** 2) * WING_AREA_M2 * CL_MAX_PASSIVE / W_NEWTONS

    # Negative stall curve
    v_neg_curve = np.linspace(0.01, math.sqrt((2 * abs(N_LIMIT_NEG) * W_NEWTONS) / (RHO_0 * WING_AREA_M2 * abs(CL_MIN_REFLEX))), 200)
    n_neg = -0.5 * RHO_0 * (v_neg_curve ** 2) * WING_AREA_M2 * abs(CL_MIN_REFLEX) / W_NEWTONS

    # Shaded Safe Operational Envelope Polygon
    poly_v = []
    poly_n = []
    
    # 0 m/s to 3 m/s Hover Column (Special Ornithopter Feature: n=1.0g at V=0!)
    poly_v.extend([0.0, 0.0, 3.0])
    poly_n.extend([0.0, 1.35, 1.35])

    # Along active stall curve to corner speed
    for v, n in zip(v_stall_curve_act, n_stall_act):
        if v >= 3.0:
            poly_v.append(v)
            poly_n.append(n)
            
    # Positive limit load flat top
    poly_v.extend([V_A_CORNER_ACT, V_NE, V_NE])
    poly_n.extend([N_LIMIT_POS, N_LIMIT_POS, 0.0])

    # Negative limit boundary
    v_neg_end = v_neg_curve[-1]
    poly_v.extend([V_NE, v_neg_end])
    poly_n.extend([N_LIMIT_NEG, N_LIMIT_NEG])

    for v, n in reversed(list(zip(v_neg_curve, n_neg))):
        poly_v.append(v)
        poly_n.append(n)
        
    poly_v.append(0.0)
    poly_n.append(0.0)

    # Fill Envelope
    ax.fill(poly_v, poly_n, color='#EBF8FF', alpha=0.8, label='Normal Safe Flight Envelope (STOL & Cruise)')

    # Hover / Weis-Fogh Envelope Extension
    hover_v = [0.0, 0.0, 3.0, 3.0]
    hover_n = [0.0, 1.35, 1.35, 0.0]
    ax.fill(hover_v, hover_n, color='#C6F6D5', alpha=0.6, label='Active Hover / Weis-Fogh Regime (0 to 3 m/s)')

    # Curves
    ax.plot(v_stall_curve_act, n_stall_act, 'b-', lw=2.2, label='Active Flapping Stall Curve ($C_{L,eff} = 1.85$)')
    ax.plot(v_stall_curve_pas, n_stall_pas, 'b--', lw=1.5, alpha=0.7, label='Passive Fixed-Wing Stall ($C_{L} = 1.45$)')
    ax.plot(v_neg_curve, n_neg, 'purple', lw=2.0, label='Negative Stall Curve ($C_{L,min} = -0.60$)')

    # Structural Boundaries
    ax.hlines(N_LIMIT_POS, V_A_CORNER_ACT, V_NE, colors='#C53030', linestyles='-', lw=2.5, label='Positive Structural Limit (+2.2g)')
    ax.hlines(N_ULTIMATE_POS, 0, V_NE, colors='#E53E3E', linestyles=':', lw=1.5, label='Ultimate Load Boundary (+3.3g)')
    ax.hlines(N_LIMIT_NEG, v_neg_end, V_NE, colors='#C53030', linestyles='-', lw=2.0, label='Negative Structural Limit (-0.8g)')
    ax.vlines(V_NE, N_LIMIT_NEG, N_LIMIT_POS, colors='#9B2C2C', linestyles='-', lw=2.5, label='Never-Exceed Airspeed $V_{NE}$ (16.0 m/s)')

    # Key Operating Points
    ax.plot([0.0], [1.0], 'go', markersize=8)
    ax.annotate('Stationary Hover\n(V=0, n=1.0g)', xy=(0.0, 1.0), xytext=(0.6, 1.15),
                arrowprops=dict(arrowstyle="->", color="green", lw=1.2), fontsize=8.5, fontweight='bold', color='green')

    ax.plot([V_UNSTICK_IGE], [1.0], 'bo', markersize=8)
    ax.annotate('STOL Unstick\n(6.2 m/s, IGE)', xy=(V_UNSTICK_IGE, 1.0), xytext=(V_UNSTICK_IGE - 2.2, 1.45),
                arrowprops=dict(arrowstyle="->", color="blue", lw=1.2), fontsize=8.5, fontweight='bold', color='#2B6CB0')

    ax.plot([V_STALL_PASSIVE], [1.0], 'ro', markersize=7)
    ax.annotate('Passive 1g Stall\n(9.38 m/s, No Flap)', xy=(V_STALL_PASSIVE, 1.0), xytext=(V_STALL_PASSIVE - 0.5, 0.45),
                arrowprops=dict(arrowstyle="->", color="red", lw=1.0), fontsize=8, color='#C53030')

    ax.plot([V_CRUISE_NOMINAL], [1.0], 'ks', markersize=8)
    ax.annotate('Nominal Cruise\n(9.0 m/s, f=2.4 Hz)', xy=(V_CRUISE_NOMINAL, 1.0), xytext=(V_CRUISE_NOMINAL - 0.2, 1.55),
                arrowprops=dict(arrowstyle="->", color="black", lw=1.2), fontsize=8.5, fontweight='bold')

    ax.plot([V_A_CORNER_ACT], [N_LIMIT_POS], 'mo', markersize=8)
    ax.annotate('Active Corner Speed $V_A$\n(12.31 m/s, +2.2g)', xy=(V_A_CORNER_ACT, N_LIMIT_POS), xytext=(V_A_CORNER_ACT - 3.5, 2.35),
                arrowprops=dict(arrowstyle="->", color="purple", lw=1.2), fontsize=8.5, fontweight='bold', color='purple')

    # Formatting
    ax.set_title("MCO-1D Mark 1 — $V$-$n$ Structural Load Factor Flight Envelope\n(MTOW = 5.687 kg, 8 Wings, S = 0.7124 m², 10:1 Capstan Reduction)",
                 fontsize=12, fontweight='bold', color='#1A365D', pad=12)
    ax.set_xlabel("True Airspeed $V$ (m/s)  [Bottom]  /  (mph)  [Reference]", fontsize=10, fontweight='bold')
    ax.set_ylabel("Load Factor $n$ ($g$ units: $L / W$)", fontsize=10, fontweight='bold')
    ax.set_xlim(-0.5, 18.0)
    ax.set_ylim(-1.2, 3.6)
    ax.axhline(0, color='gray', lw=0.8, ls='--')
    ax.axhline(1.0, color='gray', lw=0.8, ls=':')
    ax.grid(True, linestyle='--', alpha=0.5)

    # Secondary X-axis for mph
    ax2 = ax.twiny()
    ax2.set_xlim(ax.get_xlim()[0] * 2.23694, ax.get_xlim()[1] * 2.23694)
    ax2.set_xlabel("Airspeed (mph)", fontsize=9, color='#4A5568')

    ax.legend(loc='lower right', fontsize=8, framealpha=0.95)
    plt.tight_layout()
    
    png_path = os.path.join(OUTPUT_DIR, "MCO-1D_V_n_Diagram.png")
    pdf_path = os.path.join(OUTPUT_DIR, "MCO-1D_V_n_Diagram.pdf")
    plt.savefig(png_path, dpi=300)
    plt.savefig(pdf_path)
    plt.close()
    print(f"[OK] Generated: {png_path} & {pdf_path}")

# ─── 2. Plot Altitude vs Airspeed Operating Corridor ──────────────────────────
def plot_altitude_airspeed_envelope():
    fig, ax = plt.subplots(figsize=(10, 6.5), dpi=300)
    
    h_arr = np.linspace(0, CEILING_ABSOLUTE_M, 150)
    
    # Calculate stall boundaries as function of altitude
    v_stall_pas_h = []
    v_stall_act_h = []
    v_unstick_h   = []
    v_max_power_h = []
    
    for h in h_arr:
        rho = std_density(h)
        # Passive stall
        v_pas = math.sqrt((2 * W_NEWTONS) / (rho * WING_AREA_M2 * CL_MAX_PASSIVE))
        v_stall_pas_h.append(v_pas)
        
        # Active stall
        v_act = math.sqrt((2 * W_NEWTONS) / (rho * WING_AREA_M2 * CL_MAX_ACTIVE))
        v_stall_act_h.append(v_act)
        
        # Unstick (IGE at ground, fading with altitude)
        ige_factor = 2.15 - (2.15 - 1.85) * min(1.0, h / 20.0)
        v_unst = math.sqrt((2 * W_NEWTONS) / (rho * WING_AREA_M2 * ige_factor))
        v_unstick_h.append(v_unst)
        
        # High speed power ceiling (flapping thrust decreases with density)
        # Max motor power = 680W peak, available thrust shrinks with density lapse
        density_ratio = rho / RHO_0
        v_max = V_NE * math.sqrt(density_ratio) * (1.0 - 0.25 * (h / CEILING_ABSOLUTE_M))
        v_max_power_h.append(max(v_act + 1.0, v_max))

    v_stall_pas_h = np.array(v_stall_pas_h)
    v_stall_act_h = np.array(v_stall_act_h)
    v_max_power_h = np.array(v_max_power_h)

    # Operational Corridor Polygon
    # Left edge: V=0 up to hover ceiling (1500m), then active stall curve to absolute ceiling
    corr_v = [0.0]
    corr_h = [0.0]
    
    # Hover ceiling up to 1,500m
    corr_v.extend([0.0, 0.0])
    corr_h.extend([0.0, 1500.0])
    
    # Active stall up to ceiling
    for v, h in zip(v_stall_act_h, h_arr):
        if h >= 1500.0 and h <= CEILING_SERVICE_M:
            corr_v.append(v)
            corr_h.append(h)
            
    # Service ceiling flat
    idx_svc = np.argmin(np.abs(h_arr - CEILING_SERVICE_M))
    v_max_svc = v_max_power_h[idx_svc]
    corr_v.append(v_max_svc)
    corr_h.append(CEILING_SERVICE_M)
    
    # Down high speed boundary
    for v, h in reversed(list(zip(v_max_power_h[:idx_svc], h_arr[:idx_svc]))):
        corr_v.append(v)
        corr_h.append(h)
        
    corr_v.append(0.0)
    corr_h.append(0.0)

    # Shaded Main Flight Corridor
    ax.fill(corr_v, corr_h, color='#EBF8FF', alpha=0.85, label='Normal Operational Flight Corridor')

    # Shaded Hover Zone
    hover_poly_v = [0.0, 0.0, 3.0, 3.0]
    hover_poly_h = [0.0, 1500.0, 1500.0, 0.0]
    ax.fill(hover_poly_v, hover_poly_h, color='#C6F6D5', alpha=0.6, label='Hover / VTOL Capability Zone (0 to 1,500 m MSL)')

    # Shaded Passive Glide Zone (No Flapping Possible)
    glide_poly_v = []
    glide_poly_h = []
    for v, h in zip(v_stall_pas_h, h_arr):
        if h <= CEILING_SERVICE_M:
            glide_poly_v.append(v)
            glide_poly_h.append(h)
    for v, h in reversed(list(zip(v_max_power_h[:idx_svc], h_arr[:idx_svc]))):
        glide_poly_v.append(v)
        glide_poly_h.append(h)
    ax.fill(glide_poly_v, glide_poly_h, color='#FEFCBF', alpha=0.45, label=r'Unpowered Passive Glide Sub-Corridor ($V \geq 11.0$ m/s)')

    # Boundary lines
    ax.plot(v_stall_act_h, h_arr, 'b-', lw=2.2, label='Active Flapping Stall Boundary ($C_{L,eff}=1.85$)')
    ax.plot(v_stall_pas_h, h_arr, 'r--', lw=1.8, label='Passive Glide Stall Boundary ($C_L=1.45$)')
    ax.plot(v_max_power_h, h_arr, 'k-', lw=2.2, label='High-Speed / Power Limit Boundary')
    ax.axhline(CEILING_SERVICE_M, color='#C53030', ls='-', lw=2.0, label=f'Service Ceiling ({int(CEILING_SERVICE_M)} m / 8,200 ft)')
    ax.axhline(CEILING_ABSOLUTE_M, color='#E53E3E', ls=':', lw=1.5, label=f'Absolute Aerodynamic Ceiling ({int(CEILING_ABSOLUTE_M)} m)')
    ax.vlines(V_NE, 0, CEILING_SERVICE_M, color='#9B2C2C', ls='--', lw=1.5, label='$V_{NE}$ Never-Exceed Clamp (16.0 m/s)')

    # Key Operating Points
    ax.plot([0.0], [0.0], 'go', markersize=8)
    ax.annotate('Ground VTOL Hover\n(V=0, SL)', xy=(0.0, 0.0), xytext=(0.5, 200),
                arrowprops=dict(arrowstyle="->", color="green", lw=1.2), fontsize=8.5, fontweight='bold', color='green')

    ax.plot([V_UNSTICK_IGE], [0.0], 'bo', markersize=8)
    ax.annotate('STOL Liftoff\n(6.2 m/s)', xy=(V_UNSTICK_IGE, 0.0), xytext=(V_UNSTICK_IGE - 2.5, 350),
                arrowprops=dict(arrowstyle="->", color="blue", lw=1.2), fontsize=8.5, fontweight='bold', color='#2B6CB0')

    ax.plot([V_CRUISE_NOMINAL], [500.0], 'ks', markersize=8)
    ax.annotate('Design Cruise Operating Point\n(9.0 m/s, 500 m AGL, f=2.4 Hz)', xy=(V_CRUISE_NOMINAL, 500.0), xytext=(V_CRUISE_NOMINAL - 1.5, 950),
                arrowprops=dict(arrowstyle="->", color="black", lw=1.2), fontsize=8.5, fontweight='bold')

    ax.plot([V_FAST_CRUISE], [1000.0], 'k^', markersize=8)
    ax.annotate('Fast Transit\n(12.0 m/s, 1000 m)', xy=(V_FAST_CRUISE, 1000.0), xytext=(V_FAST_CRUISE - 2.0, 1400),
                arrowprops=dict(arrowstyle="->", color="black", lw=1.2), fontsize=8.5)

    # Formatting
    ax.set_title("MCO-1D Mark 1 — Altitude vs Airspeed Operational Service Envelope\n(Metachronal Cascade Flight Corridor & VTOL / Glide Sub-Envelopes)",
                 fontsize=12, fontweight='bold', color='#1A365D', pad=12)
    ax.set_xlabel("True Airspeed $V$ (m/s)  [Bottom]  /  (mph)  [Reference]", fontsize=10, fontweight='bold')
    ax.set_ylabel("Altitude (meters MSL)  [Left]  /  (feet MSL)  [Right]", fontsize=10, fontweight='bold')
    ax.set_xlim(-0.5, 18.0)
    ax.set_ylim(-50, 3500)
    ax.grid(True, linestyle='--', alpha=0.5)

    # Secondary Y-axis for Feet
    ax_alt_ft = ax.twinx()
    ax_alt_ft.set_ylim(ax.get_ylim()[0] * 3.28084, ax.get_ylim()[1] * 3.28084)
    ax_alt_ft.set_ylabel("Altitude (feet MSL)", fontsize=9, color='#4A5568')

    ax.legend(loc='upper right', fontsize=7.8, framealpha=0.95)
    plt.tight_layout()

    png_path = os.path.join(OUTPUT_DIR, "MCO-1D_Altitude_Airspeed_Envelope.png")
    pdf_path = os.path.join(OUTPUT_DIR, "MCO-1D_Altitude_Airspeed_Envelope.pdf")
    plt.savefig(png_path, dpi=300)
    plt.savefig(pdf_path)
    plt.close()
    print(f"[OK] Generated: {png_path} & {pdf_path}")

# ─── 3. Combined Multi-Panel Figure ───────────────────────────────────────────
def plot_combined_envelope():
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(16, 6.8), dpi=300)
    
    # ── Panel 1: V-n Diagram ──
    v_stall_curve_act = np.linspace(0.01, V_A_CORNER_ACT, 200)
    n_stall_act = 0.5 * RHO_0 * (v_stall_curve_act ** 2) * WING_AREA_M2 * CL_MAX_ACTIVE / W_NEWTONS
    v_stall_curve_pas = np.linspace(0.01, V_A_CORNER_PAS, 200)
    n_stall_pas = 0.5 * RHO_0 * (v_stall_curve_pas ** 2) * WING_AREA_M2 * CL_MAX_PASSIVE / W_NEWTONS
    v_neg_curve = np.linspace(0.01, math.sqrt((2 * abs(N_LIMIT_NEG) * W_NEWTONS) / (RHO_0 * WING_AREA_M2 * abs(CL_MIN_REFLEX))), 200)
    n_neg = -0.5 * RHO_0 * (v_neg_curve ** 2) * WING_AREA_M2 * abs(CL_MIN_REFLEX) / W_NEWTONS

    poly_v = [0.0, 0.0, 3.0]
    poly_n = [0.0, 1.35, 1.35]
    for v, n in zip(v_stall_curve_act, n_stall_act):
        if v >= 3.0:
            poly_v.append(v)
            poly_n.append(n)
    poly_v.extend([V_A_CORNER_ACT, V_NE, V_NE])
    poly_n.extend([N_LIMIT_POS, N_LIMIT_POS, 0.0])
    v_neg_end = v_neg_curve[-1]
    poly_v.extend([V_NE, v_neg_end])
    poly_n.extend([N_LIMIT_NEG, N_LIMIT_NEG])
    for v, n in reversed(list(zip(v_neg_curve, n_neg))):
        poly_v.append(v)
        poly_n.append(n)
    poly_v.append(0.0)
    poly_n.append(0.0)

    ax1.fill(poly_v, poly_n, color='#EBF8FF', alpha=0.85, label='Normal Flight Envelope')
    ax1.fill([0.0, 0.0, 3.0, 3.0], [0.0, 1.35, 1.35, 0.0], color='#C6F6D5', alpha=0.6, label='Hover Regime (0-3 m/s)')
    ax1.plot(v_stall_curve_act, n_stall_act, 'b-', lw=2.2, label='Active Stall ($C_{L,eff}=1.85$)')
    ax1.plot(v_stall_curve_pas, n_stall_pas, 'b--', lw=1.5, alpha=0.7, label='Passive Stall ($C_L=1.45$)')
    ax1.plot(v_neg_curve, n_neg, 'purple', lw=1.8, label='Negative Stall (-0.60)')
    ax1.hlines(N_LIMIT_POS, V_A_CORNER_ACT, V_NE, colors='#C53030', lw=2.5, label='+2.2g Limit Load')
    ax1.hlines(N_LIMIT_NEG, v_neg_end, V_NE, colors='#C53030', lw=2.0, label='-0.8g Limit Load')
    ax1.vlines(V_NE, N_LIMIT_NEG, N_LIMIT_POS, colors='#9B2C2C', lw=2.2, label='$V_{NE} = 16.0$ m/s')
    ax1.plot([V_CRUISE_NOMINAL], [1.0], 'ks', markersize=7)
    ax1.annotate('Cruise (9 m/s, 1g)', xy=(V_CRUISE_NOMINAL, 1.0), xytext=(V_CRUISE_NOMINAL - 1.0, 1.3),
                 arrowprops=dict(arrowstyle="->", lw=1), fontsize=8, fontweight='bold')
    ax1.set_title("(A) Structural Load Factor ($V$-$n$) Envelope", fontsize=11, fontweight='bold', color='#1A365D')
    ax1.set_xlabel("True Airspeed (m/s)", fontsize=9, fontweight='bold')
    ax1.set_ylabel("Load Factor $n$ (g)", fontsize=9, fontweight='bold')
    ax1.set_xlim(-0.5, 18.0)
    ax1.set_ylim(-1.2, 3.0)
    ax1.grid(True, linestyle='--', alpha=0.5)
    ax1.legend(loc='lower right', fontsize=7.5)

    # ── Panel 2: Altitude vs Airspeed ──
    h_arr = np.linspace(0, CEILING_ABSOLUTE_M, 150)
    v_act_h = [math.sqrt((2 * W_NEWTONS) / (std_density(h) * WING_AREA_M2 * CL_MAX_ACTIVE)) for h in h_arr]
    v_pas_h = [math.sqrt((2 * W_NEWTONS) / (std_density(h) * WING_AREA_M2 * CL_MAX_PASSIVE)) for h in h_arr]
    v_max_h = [max(v_act + 1.0, V_NE * math.sqrt(std_density(h)/RHO_0) * (1.0 - 0.25 * (h/CEILING_ABSOLUTE_M)))
               for h, v_act in zip(h_arr, v_act_h)]

    ax2.fill_betweenx(h_arr, 0, v_max_h, where=[h <= CEILING_SERVICE_M for h in h_arr], color='#EBF8FF', alpha=0.85, label='Service Corridor')
    ax2.fill_betweenx([0, 1500], [0, 0], [3.0, 3.0], color='#C6F6D5', alpha=0.6, label='Hover Zone')
    ax2.fill_betweenx(h_arr, v_pas_h, v_max_h, where=[h <= CEILING_SERVICE_M for h in h_arr], color='#FEFCBF', alpha=0.45, label='Glide Zone')
    ax2.plot(v_act_h, h_arr, 'b-', lw=2.2, label='Active Stall')
    ax2.plot(v_pas_h, h_arr, 'r--', lw=1.8, label='Passive Stall')
    ax2.plot(v_max_h, h_arr, 'k-', lw=2.0, label='Max Speed')
    ax2.axhline(CEILING_SERVICE_M, color='#C53030', lw=2.0, label='Service Ceiling (2500 m)')
    ax2.set_title("(B) Altitude vs Airspeed Operational Corridor", fontsize=11, fontweight='bold', color='#1A365D')
    ax2.set_xlabel("True Airspeed (m/s)", fontsize=9, fontweight='bold')
    ax2.set_ylabel("Altitude (meters MSL)", fontsize=9, fontweight='bold')
    ax2.set_xlim(-0.5, 18.0)
    ax2.set_ylim(-50, 3500)
    ax2.grid(True, linestyle='--', alpha=0.5)
    ax2.legend(loc='upper right', fontsize=7.5)

    plt.suptitle("MCO-1D Mark 1 — Complete Master Flight Performance Envelope", fontsize=13, fontweight='bold', color='#1A365D', y=0.98)
    plt.tight_layout()

    comb_png = os.path.join(OUTPUT_DIR, "MCO-1D_Mark1_Flight_Envelope.png")
    comb_pdf = os.path.join(OUTPUT_DIR, "MCO-1D_Mark1_Flight_Envelope.pdf")
    plt.savefig(comb_png, dpi=300)
    plt.savefig(comb_pdf)
    plt.close()
    print(f"[OK] Generated Combined Master Envelope: {comb_png} & {comb_pdf}")

if __name__ == '__main__':
    plot_vn_diagram()
    plot_altitude_airspeed_envelope()
    plot_combined_envelope()
