import os
import sys
import numpy as np
import matplotlib.textpath
import matplotlib.font_manager as fm
import matplotlib.pyplot as plt
import trimesh
import manifold3d
from manifold3d import CrossSection, Manifold, JoinType, OpType, FillRule

FONT_PATH = "/System/Library/Fonts/Supplemental/Times New Roman Bold.ttf"
if not os.path.exists(FONT_PATH):
    FONT_PATH = "/System/Library/Fonts/Supplemental/Times New Roman.ttf"

fp = fm.FontProperties(fname=FONT_PATH)

def build_stamp_manifold(text, plate_w, plate_h, is_multiline=False, add_handle=True):
    corner_r = 2.0
    rim_w = 1.5
    h_relief = 3.5
    h_plate = 4.0
    
    # 1. Base plate cross-section (rounded corners)
    cs_inner_box = CrossSection.square([plate_w - 2 * corner_r, plate_h - 2 * corner_r], center=True)
    cs_plate = cs_inner_box.offset(corner_r, JoinType.Round)
    
    # 2. Border rim cross-section
    cs_border_inner = cs_plate.offset(-rim_w, JoinType.Round)
    cs_rim = cs_plate - cs_border_inner
    
    # 3. Text cross-section
    inner_w = plate_w - 2 * rim_w - 5.0
    inner_h = plate_h - 2 * rim_w - 3.5
    
    if not is_multiline:
        tp_raw = matplotlib.textpath.TextPath((0, 0), text, size=10, prop=fp)
        bb = tp_raw.get_extents()
        scale = min(inner_w / bb.width, inner_h / bb.height)
        size = 10 * scale
        tp = matplotlib.textpath.TextPath((0, 0), text, size=size, prop=fp)
        bb = tp.get_extents()
        cx = (bb.xmin + bb.xmax) / 2
        cy = (bb.ymin + bb.ymax) / 2
        tp_centered = matplotlib.textpath.TextPath((-cx, -cy), text, size=size, prop=fp)
        polys = [p.tolist() for p in tp_centered.to_polygons()]
        cs_text = CrossSection(polys, fillrule=FillRule.EvenOdd)
    else:
        l1, l2 = text
        tp1_raw = matplotlib.textpath.TextPath((0, 0), l1, size=10, prop=fp)
        tp2_raw = matplotlib.textpath.TextPath((0, 0), l2, size=10, prop=fp)
        bb1 = tp1_raw.get_extents()
        bb2 = tp2_raw.get_extents()
        max_w = max(bb1.width, bb2.width)
        total_h = bb1.height + bb2.height + 2.5
        scale = min(inner_w / max_w, inner_h / total_h)
        size = 10 * scale
        tp1 = matplotlib.textpath.TextPath((0, 0), l1, size=size, prop=fp)
        tp2 = matplotlib.textpath.TextPath((0, 0), l2, size=size, prop=fp)
        bb1 = tp1.get_extents()
        bb2 = tp2.get_extents()
        cx1 = (bb1.xmin + bb1.xmax) / 2
        cy1 = (bb1.ymin + bb1.ymax) / 2
        cx2 = (bb2.xmin + bb2.xmax) / 2
        cy2 = (bb2.ymin + bb2.ymax) / 2
        y_offset = (bb1.height + 2.0 * scale) / 2
        tp1_c = matplotlib.textpath.TextPath((-cx1, -cy1 + y_offset), l1, size=size, prop=fp)
        tp2_c = matplotlib.textpath.TextPath((-cx2, -cy2 - y_offset), l2, size=size, prop=fp)
        polys = [p.tolist() for p in tp1_c.to_polygons()] + [p.tolist() for p in tp2_c.to_polygons()]
        cs_text = CrossSection(polys, fillrule=FillRule.EvenOdd)

    # CRUCIAL: Horizontally mirror the text so that pressing it into bread dough
    # produces normal left-to-right readable lettering!
    cs_text = cs_text.scale([-1.0, 1.0])

    # 4. Extrude base plate (from z=-4 to z=0)
    m_plate = Manifold.extrude(cs_plate, h_plate).translate([0, 0, -h_plate])

    # 5. Extrude border and text with draft angles for clean dough release (from z=0 to z=3.5)
    text_slices = []
    rim_slices = []
    n_steps = 4
    for i in range(n_steps):
        t0 = i / n_steps
        t1 = (i + 1) / n_steps
        z0 = t0 * h_relief
        z1 = t1 * h_relief
        dh = z1 - z0
        # Draft taper: +0.3mm offset at base, 0.0mm at top
        d = 0.3 * (1.0 - t0)
        
        cs_t_step = cs_text.offset(d, JoinType.Round)
        text_slices.append(Manifold.extrude(cs_t_step, dh).translate([0, 0, z0]))
        
        cs_rim_step = cs_plate - cs_border_inner.offset(-d, JoinType.Round)
        rim_slices.append(Manifold.extrude(cs_rim_step, dh).translate([0, 0, z0]))

    m_text = Manifold.batch_boolean(text_slices, OpType.Add)
    m_rim = Manifold.batch_boolean(rim_slices, OpType.Add)

    components = [m_plate, m_rim, m_text]

    # 6. Optional Roman Ring Handle
    if add_handle:
        # Ring center at x=0, z=-15 mm. Outer radius 13mm, inner teardrop arch radius 8mm.
        outer_cyl = Manifold.cylinder(height=10.0, radius_low=13.0, radius_high=13.0, circular_segments=48, center=True)
        # Teardrop inner profile (self-supporting 45 degree overhang)
        cs_inner_circle = CrossSection.circle(radius=8.0, circular_segments=48)
        tri_pts = [[-8.0, 0.0], [8.0, 0.0], [0.0, 10.5]]
        cs_inner_teardrop = CrossSection.batch_hull([cs_inner_circle, CrossSection([tri_pts])])
        inner_cyl = Manifold.extrude(cs_inner_teardrop, 14.0).translate([0, 0, -7.0])
        # Ring oriented along Y axis:
        ring = (outer_cyl - inner_cyl).rotate([90, 0, 0]).translate([0, 0, -15.0])
        
        # Reinforced base gussets (flaring outward to baseplate)
        flare = Manifold.cylinder(height=10.0, radius_low=17.0, radius_high=17.0, circular_segments=48, center=True)
        flare = flare.rotate([90, 0, 0]).translate([0, 0, -8.0])
        flare_clip = Manifold.cube([50, 12, 4], center=True).translate([0, 0, -6.0])
        flare = (flare ^ flare_clip) - inner_cyl.rotate([90, 0, 0]).translate([0, 0, -15.0])
        
        m_handle = ring + flare
        
        # Trim top of handle so it doesn't protrude past z=-4 mm
        cutter = Manifold.cube([plate_w + 10, plate_h + 10, 20], center=True).translate([0, 0, 6.0])
        m_handle = m_handle - cutter

        # Flatten the bottom of the ring handle (at z=-27 mm) to create a flat resting face
        flat_cutter = Manifold.cube([plate_w + 10, plate_h + 10, 10], center=True).translate([0, 0, -32.0])
        m_handle = m_handle - flat_cutter

        components.append(m_handle)

    stamp = Manifold.batch_boolean(components, OpType.Add)
    return stamp

def export_stamp(stamp, out_path):
    mesh = stamp.to_mesh()
    tm = trimesh.Trimesh(vertices=mesh.vert_properties[:, :3], faces=mesh.tri_verts, process=True)
    tm.export(out_path)
    print(f"Exported: {out_path} | Watertight: {tm.is_watertight} | Volume: {tm.volume:.1f} mm³ | Faces: {len(tm.faces)}")

def main():
    out_dir = "/Users/drbell/Code/flight/cad/bread_stamps"
    os.makedirs(out_dir, exist_ok=True)

    stamps = [
        {
            "name": "Option_C_PISTR_MICHAELAE",
            "text": "PISTR · MICHAELAE",
            "w": 82.0, "h": 20.0, "multiline": False,
            "desc": "Roman Guild Style: 'Bakery of Michelle' (1-line)"
        },
        {
            "name": "Option_D_MICHAELA_FECIT",
            "text": "MICHAELA · FECIT",
            "w": 78.0, "h": 20.0, "multiline": False,
            "desc": "Roman Artisan Signature: 'Michelle made [this]' (1-line)"
        },
        {
            "name": "Option_D_2Line_MICHAELA_FECIT",
            "text": ["MICHAELA", "FECIT"],
            "w": 64.0, "h": 28.0, "multiline": True,
            "desc": "Roman Artisan Signature: 'Michelle / Fecit' (2-line bold)"
        },
        {
            "name": "English_MICHELLES_KITCHEN",
            "text": ["MICHELLE'S", "KITCHEN"],
            "w": 68.0, "h": 28.0, "multiline": True,
            "desc": "English Monumental Roman: 'Michelle's / Kitchen' (2-line bold)"
        }
    ]

    for item in stamps:
        print(f"\nProcessing {item['desc']}...")
        
        # 1. Monolithic with authentic Roman ring handle
        stamp_handle = build_stamp_manifold(item["text"], item["w"], item["h"], is_multiline=item["multiline"], add_handle=True)
        path_handle = os.path.join(out_dir, f"{item['name']}_with_handle.stl")
        export_stamp(stamp_handle, path_handle)
        
        # 2. Flat back version (for support-free printing or custom mounting)
        stamp_flat = build_stamp_manifold(item["text"], item["w"], item["h"], is_multiline=item["multiline"], add_handle=False)
        path_flat = os.path.join(out_dir, f"{item['name']}_flat_back.stl")
        export_stamp(stamp_flat, path_flat)

    print("\nAll 8 STL files generated successfully!")

if __name__ == "__main__":
    main()
