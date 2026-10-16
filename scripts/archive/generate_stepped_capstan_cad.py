import cadquery as cq
import math
import os

os.makedirs("cad/manufactured", exist_ok=True)

# ==============================================================================
# PARAMETERS FOR 16:1 STEPPED CAPSTAN TRANSMISSION
# ==============================================================================
# Stage 1: Motor Pinion (r1 = 6mm) -> Intermediate Wheel (R2 = 24mm) -> Ratio 4:1
# Stage 2: Intermediate Spool (r3 = 6mm) -> Wing Sector Drum (R4 = 24mm) -> Ratio 4:1
# Total Reduction: 4.0 * 4.0 = 16.0 : 1
# Cable: 0.8mm Dyneema SK99 (groove depth 0.6mm, groove pitch 1.1mm)
# ==============================================================================

# ------------------------------------------------------------------------------
# 1. MOTOR PINION SPOOL (Stage 1 Input)
# ------------------------------------------------------------------------------
r1_pitch = 6.0
r1_flange = 8.5
l1_barrel = 11.0 # accommodates 9.0 turns of 0.8mm Dyneema
motor_bore = 5.0 # 5mm shaft bore for 2207 / 2807 motor

motor_pinion = (
    cq.Workplane("XY")
    .circle(r1_flange).extrude(1.5) # bottom flange
    .faces(">Z").circle(r1_pitch).extrude(l1_barrel) # wrapping drum
    .faces(">Z").circle(r1_flange).extrude(1.5) # top flange
    # Center shaft bore
    .faces(">Z").workplane().circle(motor_bore / 2.0).extrude(-(l1_barrel + 3.0), combine="cut")
    # Motor hub collar on bottom
    .faces("<Z").workplane().circle(8.0).extrude(3.0)
    .faces("<Z").workplane().circle(motor_bore / 2.0).extrude(-3.0, combine="cut")
)

# M3 set-screw cross hole in hub collar
set_screw_cut = (
    cq.Workplane("YZ")
    .transformed(offset=(0, -1.5, 0))
    .circle(1.25)
    .extrude(15.0)
)
motor_pinion = motor_pinion.cut(set_screw_cut)

# Dyneema anchor through-hole across barrel
anchor_hole1 = (
    cq.Workplane("XZ")
    .transformed(offset=(0, 6.0, 0))
    .circle(0.7)
    .extrude(12.0)
)
motor_pinion = motor_pinion.cut(anchor_hole1)

cq.exporters.export(motor_pinion, "cad/manufactured/stepped_capstan_motor_pinion.stl")
print("Generated cad/manufactured/stepped_capstan_motor_pinion.stl")

# ------------------------------------------------------------------------------
# 2. INTERMEDIATE STEPPED PULLEY (Compound Wheel & Spool)
# ------------------------------------------------------------------------------
# Large wheel (driven by motor): R2 = 24mm (dia 48mm), flange = 26mm, width = 5mm
# Small spool (driving wing): r3 = 6mm (dia 12mm), flange = 8.5mm, width = 6mm
# Bearing bores: 685-ZZ (11mm OD, 5mm ID, 5mm depth) at each end
# ------------------------------------------------------------------------------
R2_pitch = 24.0
R2_flange = 26.5
l2_rim = 4.0

r3_pitch = 6.0
r3_flange = 8.5
l3_spool = 6.0

bearing_od = 11.0
bearing_id = 5.0
bearing_depth = 5.0

# Base monolithic compound body
inter_pulley = (
    cq.Workplane("XY")
    # Stage 1 Driven Large Wheel: bottom flange + rim + top flange
    .circle(R2_flange).extrude(1.2)
    .faces(">Z").circle(R2_pitch).extrude(l2_rim)
    .faces(">Z").circle(R2_flange).extrude(1.2)
    # Transition boss
    .faces(">Z").circle(9.0).extrude(2.0)
    # Stage 2 Output Small Spool: barrel + outer flange
    .faces(">Z").circle(r3_pitch).extrude(l3_spool)
    .faces(">Z").circle(r3_flange).extrude(1.5)
)

# Central through-hole for 5mm stationary axle pin
total_len = 1.2 + l2_rim + 1.2 + 2.0 + l3_spool + 1.5
inter_pulley = (
    inter_pulley.faces(">Z").workplane()
    .circle(bearing_id / 2.0 + 0.3)
    .extrude(-total_len, combine="cut")
)

# Bearing pocket top (for 685-ZZ bearing: 11mm OD, 4.0mm deep)
inter_pulley = (
    inter_pulley.faces(">Z").workplane()
    .circle(bearing_od / 2.0)
    .extrude(-4.0, combine="cut")
)

# Bearing pocket bottom (for 685-ZZ bearing: 11mm OD, 4.0mm deep)
inter_pulley = (
    inter_pulley.faces("<Z").workplane()
    .circle(bearing_od / 2.0)
    .extrude(-4.0, combine="cut")
)

# Lightening pockets in large wheel (6-spoke pattern)
for i in range(6):
    ang = i * (360.0 / 6)
    pocket = (
        cq.Workplane("XY")
        .transformed(offset=(16.0 * math.cos(math.radians(ang)), 16.0 * math.sin(math.radians(ang)), 0))
        .circle(4.5)
        .extrude(1.2 + l2_rim + 1.2)
    )
    inter_pulley = inter_pulley.cut(pocket)

# Cable anchor holes for stage 1 and stage 2
anchor_hole2 = (
    cq.Workplane("XZ")
    .transformed(offset=(0, 2.5, 0))
    .circle(0.8)
    .extrude(R2_flange + 2.0)
)
inter_pulley = inter_pulley.cut(anchor_hole2)

cq.exporters.export(inter_pulley, "cad/manufactured/stepped_capstan_intermediate_pulley.stl")
print("Generated cad/manufactured/stepped_capstan_intermediate_pulley.stl")

# ------------------------------------------------------------------------------
# 3. WING SPAR OUTPUT SECTOR DRUM (220° Stroke Range, Final Stage)
# ------------------------------------------------------------------------------
# Radius: R4 = 24.0mm (dia 48mm)
# Arc: 220° (covers -100° to +100° flapping plus 20° safety detent)
# Center hub: 10.0mm bore for carbon spar tube with split clamping collar & pinch bolts
# ------------------------------------------------------------------------------
R4_pitch = 24.0
R4_flange = 26.5
spar_od = 10.0
sector_thickness = 10.0
arc_deg = 220.0

# Build 220° sector profile
half_arc = arc_deg / 2.0
pts_arc = []
steps = 60
d_th = math.radians(arc_deg) / steps
st_th = math.radians(-half_arc)

pts_arc.append((0.0, 0.0))
for s in range(steps + 1):
    th = st_th + s * d_th
    pts_arc.append((R4_flange * math.cos(th), R4_flange * math.sin(th)))
pts_arc.append((0.0, 0.0))

# Extrude base 220° sector
wing_sector = (
    cq.Workplane("XY")
    .polyline(pts_arc).close()
    .extrude(sector_thickness)
)

# Cable channel groove along outer rim (pitch radius R4 = 24.0mm, depth 2.5mm, width 4mm)
pts_groove = []
for s in range(steps + 1):
    th = st_th + s * d_th
    pts_groove.append(((R4_flange + 0.5) * math.cos(th), (R4_flange + 0.5) * math.sin(th)))
for s in range(steps, -1, -1):
    th = st_th + s * d_th
    pts_groove.append((R4_pitch * math.cos(th), R4_pitch * math.sin(th)))

groove_cut = (
    cq.Workplane("XY")
    .transformed(offset=(0, 0, 3.0))
    .polyline(pts_groove).close()
    .extrude(4.0)
)
wing_sector = wing_sector.cut(groove_cut)

# Main Spar Bore: 10.05 mm diameter through center
wing_sector = (
    wing_sector.faces(">Z").workplane()
    .circle(spar_od / 2.0 + 0.05)
    .extrude(-sector_thickness, combine="cut")
)

# Clamping split slot (1.2mm slit) on the back side (180° away from mid-stroke)
clamp_slot = (
    cq.Workplane("XY")
    .transformed(offset=(0, -R4_flange/2.0, 0))
    .rect(1.2, R4_flange)
    .extrude(sector_thickness)
)
wing_sector = wing_sector.cut(clamp_slot)

# Clamping pinch bolt holes (2x M3 holes across split slot)
pinch_bolt1 = (
    cq.Workplane("XZ")
    .transformed(offset=(0, 3.0, 0))
    .circle(1.6) # 3.2mm clearance
    .extrude(15.0)
)
wing_sector = wing_sector.cut(pinch_bolt1)

pinch_bolt2 = (
    cq.Workplane("XZ")
    .transformed(offset=(0, 7.0, 0))
    .circle(1.6)
    .extrude(15.0)
)
wing_sector = wing_sector.cut(pinch_bolt2)

# Dyneema dual dead-end anchor pinch screw holes (M2.5) near sector ends
for side in [-1, 1]:
    ang = side * math.radians(half_arc - 8.0)
    ax = (R4_pitch - 4.0) * math.cos(ang)
    ay = (R4_pitch - 4.0) * math.sin(ang)
    anchor_hole = (
        cq.Workplane("XY")
        .transformed(offset=(ax, ay, 0))
        .circle(1.05) # M2.5 tap bore
        .extrude(sector_thickness)
    )
    wing_sector = wing_sector.cut(anchor_hole)

cq.exporters.export(wing_sector, "cad/manufactured/stepped_capstan_wing_sector_drum.stl")
print("Generated cad/manufactured/stepped_capstan_wing_sector_drum.stl")

# ------------------------------------------------------------------------------
# 4. FULL ASSEMBLED DRIVETRAIN STL (In Operational Relative Positions)
# ------------------------------------------------------------------------------
# Center distances:
# Stage 1: Motor Axis at (0, 0, 0)
# Intermediate Axis at (35.0mm, 0, 0) -> gives 5mm clearance between drums
# Wing Spar Axis at (75.0mm, 0, 0) -> gives 11mm clearance between drums
# ------------------------------------------------------------------------------
inter_positioned = inter_pulley.translate((35.0, 0, 0))
wing_positioned = wing_sector.translate((75.0, 0, 0))

complete_assembly = motor_pinion.union(inter_positioned).union(wing_positioned)

cq.exporters.export(complete_assembly, "cad/manufactured/stepped_capstan_complete_assembly.stl")
print("Generated cad/manufactured/stepped_capstan_complete_assembly.stl")
