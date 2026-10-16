import math

# Aircraft Parameters
M_AUW = 3.55  # kg
G = 9.81
W_N = M_AUW * G  # 34.8 N

# Flight / Maneuver Loads
N_ULT = 4.0  # 4g limit load factor
F_LIFT_MAX = W_N * N_ULT  # 139.3 N total lift
# Peak flapping torque / roll moment ~ 20 N*m
# Forward thrust per motor ~ 3.5 N * 8 = 28 N
# Landing touchdown: 4g vertical impact on main gear (X=400mm) = 139 N
# Taildragger 3-point / tailskid touchdown load = ~0.5g = 17.5 N

# Corner 1 (Top-Front):
# Connects vertical post (14mm) in tension/compression to spine (20mm) in bending/torque
# Post tension during positive G: post pulls down on Corner 1, or pushes up on touchdown
# Max axial post load: ~140 N
# Required wall thickness for PETG (yield stress 50 MPa, PA-CF yield 75 MPa)
# Minimum carbon hoop clamp wall: 2.2 - 2.5 mm is structurally sufficient to generate 400 N clamping force!
print(f"Total AUW: {M_AUW} kg | 4g limit load: {F_LIFT_MAX:.1f} N")
