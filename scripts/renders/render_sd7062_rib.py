#!/usr/bin/env python3
"""
Renders orthographic and isometric previews of the generated SD7062 wing rib STL.
"""
import matplotlib.pyplot as plt
import numpy as np

# Load coordinate text file
pts = []
for line in open('cad/coordinates/SD7062_165mm.txt'):
    if line.strip().startswith('#') or not line.strip():
        continue
    toks = line.split()
    pts.append([float(toks[0]), float(toks[1])])

pts = np.array(pts)

fig, ax = plt.subplots(figsize=(12, 4), dpi=150)
ax.plot(pts[:, 0], pts[:, 1], 'b-', lw=2, label='SD7062 Airfoil Profile (c = 165 mm)')
ax.fill(pts[:, 0], pts[:, 1], 'lightblue', alpha=0.3)

# Plot chord line
ax.plot([0, 165], [0, 0], 'k--', lw=1, alpha=0.6, label='Chord Line (Y = 0)')

# Spar Bores
# Fwd (10%, x=16.5, dia=2.4)
circle_fwd = plt.Circle((16.5, 4.11), 1.2, color='red', fill=True, label='Fwd Spar (⌀2.4 mm, 10% c)')
ax.add_patch(circle_fwd)

# Mid (20%, x=33.0, dia=10.5)
circle_mid = plt.Circle((33.0, 5.68), 5.25, color='darkred', fill=True, label='Main Spar (⌀10.5 mm, 20% c)')
ax.add_patch(circle_mid)

# Aft (55%, x=90.75, dia=2.4)
circle_aft = plt.Circle((90.75, 5.99), 1.2, color='red', fill=True, label='Aft Spar (⌀2.4 mm, 55% c)')
ax.add_patch(circle_aft)

# Annotations
ax.text(16.5, 8.0, 'Fwd ⌀2.4', ha='center', fontsize=9, fontweight='bold', color='red')
ax.text(33.0, 13.0, 'Main Spar ⌀10.5', ha='center', fontsize=9, fontweight='bold', color='darkred')
ax.text(90.75, 9.5, 'Aft ⌀2.4', ha='center', fontsize=9, fontweight='bold', color='red')

ax.set_aspect('equal')
ax.set_xlim(-5, 175)
ax.set_ylim(-15, 25)
ax.set_xlabel('Chordwise Position X (mm)')
ax.set_ylabel('Height Y (mm)')
ax.set_title('MCO-1.5D Wing Rib — Selig-Donovan SD7062 (Chord = 165.0 mm)')
ax.grid(True, linestyle=':', alpha=0.6)
ax.legend(loc='lower center', bbox_to_anchor=(0.5, -0.32), ncol=4, fontsize=8)

plt.tight_layout()
out_png = 'cad/manufactured/sd7062_wing_rib_profile_preview.png'
plt.savefig(out_png, dpi=200)
print(f'Rendered preview to: {out_png}')
