#!/usr/bin/env python3
"""
generate_spring_installation_drawing.py
Generates a multi-panel engineering blueprint drawing of the resonant torsion spring
integrated into the V2 Capstan Shoulder Assembly for the MCO-1D Ornithopter.
"""

import os
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.patches import FancyArrowPatch, Arc, Rectangle, Circle, Polygon
import numpy as np

os.makedirs("cad/drawings", exist_ok=True)
output_path = "cad/drawings/v2_spring_installation_schematic.png"

fig = plt.figure(figsize=(20, 12), dpi=200, facecolor="#0b0f19")

# Define grid
# Top left: Cross Section (A-A)
# Top right: Top Plan View (Kinematics & Anchors)
# Bottom: Exploded Stack Order & Engineering Specs
gs = fig.add_gridspec(2, 2, width_ratios=[1.1, 0.9], height_ratios=[1.1, 0.9],
                       left=0.05, right=0.96, top=0.92, bottom=0.06, wspace=0.15, hspace=0.22)

ax_section  = fig.add_subplot(gs[0, 0])
ax_plan     = fig.add_subplot(gs[0, 1])
ax_exploded = fig.add_subplot(gs[1, 0])
ax_specs    = fig.add_subplot(gs[1, 1])

# Styling helper
for ax in [ax_section, ax_plan, ax_exploded, ax_specs]:
    ax.set_facecolor("#111827")
    ax.tick_params(colors="#9ca3af")
    for spine in ax.spines.values():
        spine.set_color("#374151")

# =============================================================================
# PANEL 1: CROSS-SECTION A-A (PEDESTAL + BIG DRUM + TORSION SPRING)
# =============================================================================
ax_section.set_title("SECTION A-A: COAXIAL SPRING INSTALLATION & BEARING STACK", 
                      color="#38bdf8", fontsize=13, fontweight="bold", pad=12, loc="left")
ax_section.set_xlim(-55, 55)
ax_section.set_ylim(-15, 65)
ax_section.set_aspect("equal")
ax_section.grid(True, linestyle="--", alpha=0.15, color="#6b7280")

# Centerline
ax_section.axvline(0, color="#ef4444", linestyle="-.", linewidth=1.2, alpha=0.7, label="Main Flap Pivot Axis")

# 1. Base Plate (Z: -10 to 0)
base_plate = Rectangle((-45, -8), 90, 8, facecolor="#334155", edgecolor="#64748b", linewidth=1.5, hatch="//")
ax_section.add_patch(base_plate)

# 2. Conical/Cylindrical Pedestal Post (Fixed to Base)
# OD: 40mm at bearing journals (R=20), taper at base
pedestal_poly = Polygon([
    (-30, -8), (-25, 0), (-20, 0), (-20, 48), 
    (20, 48), (20, 0), (25, 0), (30, -8)
], closed=True, facecolor="#1e293b", edgecolor="#0ea5e9", linewidth=2.0)
ax_section.add_patch(pedestal_poly)

# Inner bore of Pedestal (Hollow core for M4 screw tower / weight saving)
pedestal_bore = Rectangle((-12, 0), 24, 48, facecolor="#0b0f19", edgecolor="#475569", linestyle=":")
ax_section.add_patch(pedestal_bore)

# 3. Dual 6708-2RS Bearings (ID 40, OD 50, W 6) -> R_in=20, R_out=25, W=6
# Lower Bearing at Z=5 to 11
b_low_l = Rectangle((-25, 5), 5, 6, facecolor="#e2e8f0", edgecolor="#0284c7", linewidth=1.2)
b_low_r = Rectangle((20, 5), 5, 6, facecolor="#e2e8f0", edgecolor="#0284c7", linewidth=1.2)
ax_section.add_patch(b_low_l)
ax_section.add_patch(b_low_r)
ax_section.plot([-22.5, 22.5], [8, 8], 'ko', markersize=3) # balls

# Upper Bearing at Z=37 to 43
b_up_l = Rectangle((-25, 37), 5, 6, facecolor="#e2e8f0", edgecolor="#0284c7", linewidth=1.2)
b_up_r = Rectangle((20, 37), 5, 6, facecolor="#e2e8f0", edgecolor="#0284c7", linewidth=1.2)
ax_section.add_patch(b_up_l)
ax_section.add_patch(b_up_r)
ax_section.plot([-22.5, 22.5], [40, 40], 'ko', markersize=3) # balls

# 4. Big Drum Outer Shell (Rotates on 6708 bearings)
# Drum OD = 70mm (R=35), Bearing seats at R=25, Relief at R=27
drum_l = Polygon([
    (-45, 2), (-25, 2), (-25, 11), (-27, 11), (-27, 37), (-25, 37), (-25, 45), (-45, 45)
], closed=True, facecolor="#1e3a5f", edgecolor="#38bdf8", linewidth=1.8, alpha=0.85)
drum_r = Polygon([
    (45, 2), (25, 2), (25, 11), (27, 11), (27, 37), (25, 37), (25, 45), (45, 45)
], closed=True, facecolor="#1e3a5f", edgecolor="#38bdf8", linewidth=1.8, alpha=0.85)
ax_section.add_patch(drum_l)
ax_section.add_patch(drum_r)

# Cable Grooves on Drum outer perimeter
for zg in [15, 21, 27, 33]:
    ax_section.plot([-45, -45], [zg-1.5, zg+1.5], color="#f59e0b", linewidth=2.5)
    ax_section.plot([45, 45], [zg-1.5, zg+1.5], color="#f59e0b", linewidth=2.5)

# 5. Helical Torsion Spring (Coaxially situated in the internal annular chamber or on top)
# Coils: d=3.5mm wire, Dm=30mm (R=15, cross-section wire circles at R=+/-15)
spring_z_start = 14
n_coils = 5
pitch = 4.2
for i in range(n_coils):
    zc = spring_z_start + i * pitch
    c_l = Circle((-15, zc), 1.75, facecolor="#f59e0b", edgecolor="#fef08a", linewidth=1.2)
    c_r = Circle((15, zc), 1.75, facecolor="#f59e0b", edgecolor="#fef08a", linewidth=1.2)
    ax_section.add_patch(c_l)
    ax_section.add_patch(c_r)
    # Helical slant line
    ax_section.plot([-15, 15], [zc, zc + pitch/2], color="#d97706", linewidth=1.5, alpha=0.7)

# 6. Stationary Anchor Leg (Fixed to Pedestal / Bridge)
ax_section.plot([15, 15, 8], [spring_z_start + 4*pitch, 48, 48], color="#e11d48", linewidth=3.5, solid_capstyle="round")
ax_section.plot(8, 48, marker='o', markersize=6, color="#ef4444")

# 7. Moving Anchor Leg (Engages Big Drum drive pin / slot)
ax_section.plot([-15, -28], [spring_z_start, spring_z_start], color="#10b981", linewidth=3.5, solid_capstyle="round")
ax_section.plot(-28, spring_z_start, marker='s', markersize=6, color="#10b981")

# 8. Top Dog-Bone Bridge (Z: 48 to 56)
bridge = Rectangle((-35, 48), 70, 8, facecolor="#475569", edgecolor="#94a3b8", linewidth=1.5)
ax_section.add_patch(bridge)
# 4x M4 Bolts through bridge into pedestal
ax_section.plot([-8, -8], [42, 57], color="#cbd5e1", linewidth=3.0)
ax_section.plot([8, 8], [42, 57], color="#cbd5e1", linewidth=3.0)

# Annotations / Callouts
ax_section.annotate("Top Bridge (Stationary Tie)", xy=(-30, 52), xytext=(-52, 60),
                    arrowprops=dict(arrowstyle="->", color="#94a3b8", lw=1.2), color="#e2e8f0", fontsize=9)
ax_section.annotate("Upper 6708 Bearing (ID 40, OD 50)", xy=(-25, 40), xytext=(-53, 44),
                    arrowprops=dict(arrowstyle="->", color="#38bdf8", lw=1.2), color="#38bdf8", fontsize=9)
ax_section.annotate("Torsion Spring (d=3.5mm, 5 Coils)\nCoaxial around Pedestal Core", xy=(15, 24), xytext=(22, 28),
                    arrowprops=dict(arrowstyle="->", color="#f59e0b", lw=1.4), color="#fef08a", fontsize=9, fontweight="bold")
ax_section.annotate("Moving Leg Anchor\n(Keyed to Big Drum)", xy=(-28, spring_z_start), xytext=(-52, 20),
                    arrowprops=dict(arrowstyle="->", color="#10b981", lw=1.4), color="#34d399", fontsize=9, fontweight="bold")
ax_section.annotate("Stationary Anchor Leg\n(Pinned to Bridge/Pedestal)", xy=(8, 48), xytext=(12, 56),
                    arrowprops=dict(arrowstyle="->", color="#ef4444", lw=1.4), color="#f87171", fontsize=9, fontweight="bold")
ax_section.annotate("Lower 6708 Bearing", xy=(-25, 8), xytext=(-50, 6),
                    arrowprops=dict(arrowstyle="->", color="#38bdf8", lw=1.2), color="#38bdf8", fontsize=9)
ax_section.annotate("Base Mounting Plate", xy=(-35, -4), xytext=(-50, -12),
                    arrowprops=dict(arrowstyle="->", color="#94a3b8", lw=1.2), color="#94a3b8", fontsize=9)

# =============================================================================
# PANEL 2: TOP PLAN VIEW & KINEMATICS
# =============================================================================
ax_plan.set_title("TOP PLAN VIEW: PRELOAD & FLAPPING KINEMATICS", 
                  color="#38bdf8", fontsize=13, fontweight="bold", pad=12, loc="left")
ax_plan.set_xlim(-60, 60)
ax_plan.set_ylim(-60, 60)
ax_plan.set_aspect("equal")
ax_plan.grid(True, linestyle="--", alpha=0.15, color="#6b7280")

# Pivot center
ax_plan.plot(0, 0, 'r+', markersize=10, markeredgewidth=2)

# Big Drum Circle
c_drum = Circle((0, 0), 45, facecolor="#1e293b", edgecolor="#38bdf8", linewidth=2.0, linestyle="--")
ax_plan.add_patch(c_drum)

# Spring Coil Profile
c_spring_od = Circle((0, 0), 16.75, fill=False, edgecolor="#f59e0b", linewidth=3.0, alpha=0.8)
c_spring_id = Circle((0, 0), 13.25, fill=False, edgecolor="#f59e0b", linewidth=1.5, linestyle=":")
ax_plan.add_patch(c_spring_od)
ax_plan.add_patch(c_spring_id)

# Wing Spar Extension Line (Neutral Dihedral = +4 deg)
neutral_deg = 4.0
rad_neu = np.radians(neutral_deg)
ax_plan.plot([0, 52 * np.cos(rad_neu)], [0, 52 * np.sin(rad_neu)], color="#10b981", linewidth=3.5, label="Neutral Dihedral (+4°)")

# Stroke Limit Lines (+/- 45 deg from neutral)
up_deg = neutral_deg + 45.0
dn_deg = neutral_deg - 45.0
rad_up = np.radians(up_deg)
rad_dn = np.radians(dn_deg)

ax_plan.plot([0, 50 * np.cos(rad_up)], [0, 50 * np.sin(rad_up)], color="#eab308", linestyle="--", linewidth=2.0)
ax_plan.plot([0, 50 * np.cos(rad_dn)], [0, 50 * np.sin(rad_dn)], color="#eab308", linestyle="--", linewidth=2.0)

# Stroke arc
stroke_arc = Arc((0, 0), 70, 70, angle=0, theta1=dn_deg, theta2=up_deg, color="#facc15", linewidth=2.5)
ax_plan.add_patch(stroke_arc)
ax_plan.text(38, 22, "90° Total Stroke\n(±45° Amplitude)", color="#facc15", fontsize=10, fontweight="bold")

# Fixed anchor point on bridge (at X=0, Y=20)
ax_plan.plot(0, 20, marker="o", markersize=9, color="#ef4444")
ax_plan.plot([0, 0], [15, 22], color="#ef4444", linewidth=3.5)
ax_plan.annotate("Fixed Anchor Pin\n(On Top Bridge)", xy=(0, 20), xytext=(-45, 42),
                arrowprops=dict(arrowstyle="->", color="#ef4444", lw=1.2), color="#f87171", fontsize=9, fontweight="bold")

# Moving anchor pin on Big Drum (at neutral position)
moving_pin_x = 28 * np.cos(rad_neu)
moving_pin_y = 28 * np.sin(rad_neu)
ax_plan.plot(moving_pin_x, moving_pin_y, marker="s", markersize=9, color="#10b981")
ax_plan.plot([15 * np.cos(rad_neu), moving_pin_x], [15 * np.sin(rad_neu), moving_pin_y], color="#10b981", linewidth=3.5)
ax_plan.annotate("Moving Anchor Key\n(Locked into Big Drum)", xy=(moving_pin_x, moving_pin_y), xytext=(15, -45),
                arrowprops=dict(arrowstyle="->", color="#10b981", lw=1.2), color="#34d399", fontsize=9, fontweight="bold")

# Torque Return Direction Indicators
arr_up = FancyArrowPatch((30*np.cos(rad_up), 30*np.sin(rad_up)), 
                         (28*np.cos(rad_up-np.radians(15)), 28*np.sin(rad_up-np.radians(15))),
                         connectionstyle="arc3,rad=-0.2", color="#f43f5e", arrowstyle="->", mutation_scale=15, lw=2.5)
arr_dn = FancyArrowPatch((30*np.cos(rad_dn), 30*np.sin(rad_dn)), 
                         (28*np.cos(rad_dn+np.radians(15)), 28*np.sin(rad_dn+np.radians(15))),
                         connectionstyle="arc3,rad=0.2", color="#f43f5e", arrowstyle="->", mutation_scale=15, lw=2.5)
ax_plan.add_patch(arr_up)
ax_plan.add_patch(arr_dn)
ax_plan.text(12, 34, "Elastic\nRestoring\nTorque", color="#f43f5e", fontsize=8, fontweight="bold")

# =============================================================================
# PANEL 3: EXPLODED STACK-UP SEQUENCE
# =============================================================================
ax_exploded.set_title("MECHANICAL INTEGRATION & INSTALLATION STACK ORDER", 
                      color="#38bdf8", fontsize=13, fontweight="bold", pad=12, loc="left")
ax_exploded.set_xlim(0, 100)
ax_exploded.set_ylim(0, 100)
ax_exploded.axis("off")

steps = [
    ("Step 1: Base & Lower Bearing", "Press lower 6708-2RS bearing (ID 40, OD 50, W 6) over conical pedestal shoulder.", "#38bdf8"),
    ("Step 2: Drum Installation", "Slide Big Drum over pedestal. Lower bearing seats into internal bottom bore.", "#0ea5e9"),
    ("Step 3: Spring Insertion", "Lower Torsion Spring into drum bore. Insert Moving Leg into pre-machined radial slot in drum.", "#f59e0b"),
    ("Step 4: Upper Bearing", "Press upper 6708-2RS bearing over pedestal neck to clamp Big Drum axially without binding.", "#38bdf8"),
    ("Step 5: Top Bridge & Fixed Anchor", "Engage Stationary Leg into anchor hole of Top Bridge; secure Bridge with 4x M4 screws to 3.0 N·m.", "#ef4444"),
    ("Step 6: Cable Rigging & Preload Check", "Set wing to +4° neutral dihedral glide trim before tensioning Dyneema drive cable.", "#10b981")
]

y_pos = 86
for i, (title, desc, col) in enumerate(steps):
    # Step box
    rect = Rectangle((2, y_pos - 10), 96, 12, facecolor="#1e293b", edgecolor=col, linewidth=1.5)
    ax_exploded.add_patch(rect)
    ax_exploded.text(5, y_pos - 1, title, color=col, fontsize=9.5, fontweight="bold")
    ax_exploded.text(5, y_pos - 7, desc, color="#cbd5e1", fontsize=8.5)
    y_pos -= 15

# =============================================================================
# PANEL 4: TECHNICAL SPECIFICATIONS & PROCUREMENT
# =============================================================================
ax_specs.set_title("ENGINEERING SPECIFICATION & OFF-THE-SHELF PROCUREMENT", 
                   color="#38bdf8", fontsize=13, fontweight="bold", pad=12, loc="left")
ax_specs.set_xlim(0, 100)
ax_specs.set_ylim(0, 100)
ax_specs.axis("off")

specs_data = [
    ("Target Resonant Flapping Freq (f₀)", "1.95 Hz (Hypothesis Median)"),
    ("Wing Flap Inertia per Joint (I_wing)", "0.0220 kg·m²"),
    ("Required Torsional Stiffness (K_spring)", "3.30 N·m/rad (0.058 N·m/deg)"),
    ("Wire Diameter (d)", "3.50 mm (ASTM A228 Music Wire / 302 SS)"),
    ("Mean Coil Diameter (D_m)", "30.0 mm (OD = 33.5 mm, ID = 26.5 mm)"),
    ("Number of Active Coils (N_a)", "4.9 coils (Solid Height ≈ 17.2 mm)"),
    ("Max Operational Deflection (θ_max)", "±45° (±0.785 rad)"),
    ("Max Spring Torque at Peak (τ_max)", "±2.59 N·m (Wing axis) / 0.37 N·m (Motor)"),
    ("Peak Bending Stress (σ_max)", "1,220 MPa (68% of 1,800 MPa Yield)"),
    ("Calculated Fatigue Life", "> 2.5 × 10⁶ cycles (Infinite in flight)"),
    ("Total Spring Weight", "35.5 grams"),
    ("Recommended Vendor Part", "McMaster-Carr #9271K series / Lee Spring LTL")
]

table_y = 92
for label, val in specs_data:
    ax_specs.text(3, table_y, label, color="#9ca3af", fontsize=8.5)
    ax_specs.text(62, table_y, val, color="#f3f4f6", fontsize=8.5, fontweight="bold", family="monospace")
    ax_specs.plot([3, 97], [table_y - 2, table_y - 2], color="#374151", linewidth=0.6)
    table_y -= 7.4

# Main Figure Title
fig.suptitle("MCO-1D SHOULDER FLAP JOINT: RESONANT TORSION SPRING INTEGRATION (V2 CAPSTAN)", 
             color="#ffffff", fontsize=15, fontweight="heavy", y=0.97)

plt.savefig(output_path, dpi=200, bbox_inches="tight")
plt.close()
print(f"Generated {output_path}")
