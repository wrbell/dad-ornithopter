#!/usr/bin/env python3
"""
Renders comprehensive multi-angle orthographic and isometric previews
of the generated SD7062 3D-printable wing system parts.
"""
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import numpy as np

def load_stl(filename):
    with open(filename, 'rb') as f:
        f.read(84)
        data = f.read()
    arr = np.frombuffer(data, dtype=np.dtype([('n', 'f4', 3), ('v1', 'f4', 3), ('v2', 'f4', 3), ('v3', 'f4', 3), ('a', 'u2')]))
    return np.stack([arr['v1'], arr['v2'], arr['v3']], axis=1)

mount_tris = load_stl('cad/manufactured/sd7062_airfoil_mount_starboard_2.4mm.stl')
seg_tris = load_stl('cad/manufactured/sd7062_wing_segment_25cm_2.4mm.stl')
tip_tris = load_stl('cad/manufactured/sd7062_wingtip_smooth_starboard_2.4mm.stl')

fig = plt.figure(figsize=(16, 10), dpi=150)

# Subplot 1: Full Starboard Assembly Isometric
ax1 = fig.add_subplot(2, 2, 1, projection='3d')
# Subsample triangles for fast rendering
skip_m = max(1, len(mount_tris) // 600)
skip_s = max(1, len(seg_tris) // 1200)
skip_t = max(1, len(tip_tris) // 800)

m_poly = Poly3DCollection(mount_tris[::skip_m], alpha=0.8, edgecolor='k', linewidths=0.2, facecolor='royalblue')
s_poly = Poly3DCollection(seg_tris[::skip_s], alpha=0.7, edgecolor='k', linewidths=0.2, facecolor='lightseagreen')
t_poly = Poly3DCollection(tip_tris[::skip_t] + np.array([0, 0, -250.0]), alpha=0.8, edgecolor='k', linewidths=0.2, facecolor='coral')

ax1.add_collection3d(m_poly)
ax1.add_collection3d(s_poly)
ax1.add_collection3d(t_poly)

ax1.set_xlim(0, 165)
ax1.set_ylim(-30, 40)
ax1.set_zlim(-380, 30)
ax1.view_init(elev=25, azim=-60)
ax1.set_xlabel('X (Chord, mm)')
ax1.set_ylabel('Y (Height, mm)')
ax1.set_zlabel('Z (Span, mm)')
ax1.set_title('SD7062 Wing Assembly (Mount + 25cm Segment + Wingtip)')

# Subplot 2: Mount Detail Isometric
ax2 = fig.add_subplot(2, 2, 2, projection='3d')
m_detail = Poly3DCollection(mount_tris[::skip_m], alpha=0.85, edgecolor='darkblue', linewidths=0.3, facecolor='cornflowerblue')
ax2.add_collection3d(m_detail)
ax2.set_xlim(0, 165)
ax2.set_ylim(-25, 45)
ax2.set_zlim(-5, 25)
ax2.view_init(elev=35, azim=-45)
ax2.set_xlabel('X (mm)')
ax2.set_ylabel('Y (mm)')
ax2.set_zlabel('Z (mm)')
ax2.set_title('SD7062 Airfoil Mount Detail (Z=0 Female Socket, Z=20 Flange)')

# Subplot 3: 25cm Segment Detail
ax3 = fig.add_subplot(2, 2, 3, projection='3d')
s_detail = Poly3DCollection(seg_tris[::skip_s], alpha=0.75, edgecolor='teal', linewidths=0.3, facecolor='mediumaquamarine')
ax3.add_collection3d(s_detail)
ax3.set_xlim(0, 165)
ax3.set_ylim(-20, 25)
ax3.set_zlim(-260, 10)
ax3.view_init(elev=20, azim=-55)
ax3.set_xlabel('X (mm)')
ax3.set_ylabel('Y (mm)')
ax3.set_zlabel('Z (mm)')
ax3.set_title('SD7062 25cm Segment (D-Box, Open Bays & Oval Lightening Ribs)')

# Subplot 4: Wingtip Detail
ax4 = fig.add_subplot(2, 2, 4, projection='3d')
t_detail = Poly3DCollection(tip_tris[::skip_t], alpha=0.85, edgecolor='brown', linewidths=0.3, facecolor='salmon')
ax4.add_collection3d(t_detail)
ax4.set_xlim(0, 165)
ax4.set_ylim(-20, 25)
ax4.set_zlim(-135, 10)
ax4.view_init(elev=25, azim=-65)
ax4.set_xlabel('X (mm)')
ax4.set_ylabel('Y (mm)')
ax4.set_zlabel('Z (mm)')
ax4.set_title('SD7062 Aerodynamic Wingtip (Swept-Taper Loft, Blind Sockets)')

plt.tight_layout()
out_png = 'cad/manufactured/sd7062_wing_system_assembly_preview.png'
plt.savefig(out_png, dpi=180)
print(f'Rendered system assembly preview: {out_png}')
