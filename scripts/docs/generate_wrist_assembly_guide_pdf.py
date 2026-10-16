#!/usr/bin/env python3
"""
scripts/generate_wrist_assembly_guide_pdf.py
============================================
Generates the publication-grade 2-page printable assembly manual PDF:
  - docs/MCO-1D_V9_Wrist_Assembly_Guide.pdf
  - <artifactDir>/MCO-1D_V9_Wrist_Assembly_Guide.pdf

Features:
  1. Professional aerospace technical publication styling (Navy, Slate, Crisp White).
  2. Large 3D Exploded Assembly CAD Diagram with numbered part callouts.
  3. Complete 3D Print Manufacturing BOM & Slicing Specifications Table.
  4. Off-The-Shelf (OTS) Hardware Checklist.
  5. 6-Stage Pictorial Assembly Sequence with workbench photos, torque specs, and quality gates.
  6. Maintenance & 90-second Quick-Swap Gear Replacement Protocol.
"""

import os
import sys
from reportlab.lib.pagesizes import letter
from reportlab.lib import colors
from reportlab.lib.units import inch
from reportlab.lib.styles import getSampleStyleSheet, ParagraphStyle
from reportlab.platypus import (
    SimpleDocTemplate, Paragraph, Spacer, Table, TableStyle, PageBreak, Image, KeepTogether, HRFlowable
)
from reportlab.pdfgen import canvas

# Palette
NAVY_PRIMARY = colors.HexColor("#0f172a")   # Slate 900
BLUE_ACCENT  = colors.HexColor("#0284c7")   # Sky 600
ICE_BG       = colors.HexColor("#f8fafc")   # Slate 50
BORDER_GRAY  = colors.HexColor("#cbd5e1")   # Slate 300
DARK_TEXT    = colors.HexColor("#1e293b")   # Slate 800
MUTED_TEXT   = colors.HexColor("#64748b")   # Slate 500
CARD_BG      = colors.HexColor("#ffffff")
WARN_AMBER   = colors.HexColor("#d97706")   # Amber 600
SUCCESS_GRN  = colors.HexColor("#16a34a")   # Green 600

DOC_OUT = "docs/MCO-1D_V9_Wrist_Assembly_Guide.pdf"
ARTIFACT_DIR = "/Users/drbell/.gemini/antigravity/brain/a5a482f2-2cea-48ab-945b-be0d2fd94a84"
ART_OUT = os.path.join(ARTIFACT_DIR, "MCO-1D_V9_Wrist_Assembly_Guide.pdf")

os.makedirs("docs", exist_ok=True)
os.makedirs(ARTIFACT_DIR, exist_ok=True)

class NumberedCanvas(canvas.Canvas):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._saved_page_states = []

    def showPage(self):
        self._saved_page_states.append(dict(self.__dict__))
        self._startPage()

    def save(self):
        num_pages = len(self._saved_page_states)
        for state in self._saved_page_states:
            self.__dict__.update(state)
            self.draw_page_decorations(num_pages)
            super().showPage()
        super().save()

    def draw_page_decorations(self, total_pages):
        self.saveState()
        self.setFont("Helvetica", 7.5)
        self.setFillColor(MUTED_TEXT)
        # Running Top Header
        self.drawString(36, 762, "MCO-1D FLIGHT SYSTEMS | TECHNICAL ASSEMBLY MANUAL")
        self.drawRightString(576, 762, "DOC: MCO-MAN-WRIST-V9 | REV 2.4 (OCT 2026)")
        self.setStrokeColor(BORDER_GRAY)
        self.setLineWidth(0.6)
        self.line(36, 756, 576, 756)

        # Running Footer
        self.line(36, 38, 576, 38)
        self.drawString(36, 26, "CONFIDENTIAL & PROPRIETARY — FOR BENCH ASSEMBLY & FLIGHT INTEGRATION ONLY")
        self.drawRightString(576, 26, f"PAGE {self._pageNumber} OF {total_pages}")
        self.restoreState()

def build_pdf():
    doc = SimpleDocTemplate(
        DOC_OUT,
        pagesize=letter,
        leftMargin=36,
        rightMargin=36,
        topMargin=46,
        bottomMargin=46
    )

    styles = getSampleStyleSheet()
    
    title_style = ParagraphStyle(
        'DocTitle',
        parent=styles['Normal'],
        fontName='Helvetica-Bold',
        fontSize=15,
        leading=18,
        textColor=NAVY_PRIMARY
    )
    subtitle_style = ParagraphStyle(
        'DocSubtitle',
        parent=styles['Normal'],
        fontName='Helvetica',
        fontSize=8.5,
        leading=11,
        textColor=BLUE_ACCENT
    )
    section_h1 = ParagraphStyle(
        'SectionH1',
        parent=styles['Normal'],
        fontName='Helvetica-Bold',
        fontSize=10,
        leading=12,
        textColor=NAVY_PRIMARY,
        spaceBefore=5,
        spaceAfter=3
    )
    body_style = ParagraphStyle(
        'BodyDark',
        parent=styles['Normal'],
        fontName='Helvetica',
        fontSize=7.2,
        leading=9.2,
        textColor=DARK_TEXT
    )
    body_bold = ParagraphStyle(
        'BodyBold',
        parent=styles['Normal'],
        fontName='Helvetica-Bold',
        fontSize=7.2,
        leading=9.2,
        textColor=DARK_TEXT
    )
    callout_box = ParagraphStyle(
        'CalloutText',
        parent=styles['Normal'],
        fontName='Helvetica',
        fontSize=7.0,
        leading=9.0,
        textColor=colors.HexColor("#78350f")
    )
    table_cell = ParagraphStyle(
        'TableCell',
        parent=styles['Normal'],
        fontName='Helvetica',
        fontSize=6.8,
        leading=8.5,
        textColor=DARK_TEXT
    )
    table_cell_bold = ParagraphStyle(
        'TableCellBold',
        parent=styles['Normal'],
        fontName='Helvetica-Bold',
        fontSize=6.8,
        leading=8.5,
        textColor=NAVY_PRIMARY
    )

    story = []

    # =========================================================================
    # PAGE 1: TITLE, EXPLODED VIEW & 3D PRINT BILL OF MATERIALS
    # =========================================================================
    story.append(Paragraph("<a href='file:///Users/drbell/Code/flight/docs/MCO-1D_V9_Wrist_Assembly_Guide.pdf'><font color='#0f172a'>MCO-1D ROBOTIC WING ROOT WRIST ASSEMBLY GUIDE</font></a>", title_style))
    story.append(Paragraph("V10 Stage-Coach Architecture: 10:1 Drivetrain, 25.7g Hollow Spoke Drum, Open Servos &amp; Quick-Swap Bevels | <a href='file:///Users/drbell/.gemini/antigravity/brain/a5a482f2-2cea-48ab-945b-be0d2fd94a84/v10_stagecoach_topological_optimization_audit.md'><font color='#0284c7'><u>Read Stage-Coach TO Audit</u></font></a>", subtitle_style))
    story.append(Spacer(1, 4))

    # Exploded Diagram + Visual Callout Table (Side-by-Side)
    exploded_img_path = "cad/manufactured/renders_v9/v9_10_exploded_assembly_view.png"
    photo_gear_path = os.path.join(ARTIFACT_DIR, "prototype_3dprint_bevel_1791990036863.jpg")
    photo_fairing_path = os.path.join(ARTIFACT_DIR, "faired_wrist_naca0018_1791995071448.jpg")
    photo_stagecoach_path = os.path.join(ARTIFACT_DIR, "stagecoach_skeleton_wheel_1792063295623.jpg")

    exploded_img = Image(exploded_img_path, width=4.6*inch, height=2.58*inch)
    
    # Legend Table next to exploded view with live hyperlinks
    legend_data = [
        [Paragraph("<b>CALLOUT LEGEND</b>", table_cell_bold), Paragraph("<b>SUBSYSTEM &amp; HYPERLINK</b>", table_cell_bold)],
        [Paragraph("<b>[1]</b>", table_cell_bold), Paragraph("<a href='file:///Users/drbell/Code/flight/cad/manufactured/v9_topological_fairing.stl'><font color='#0284c7'><u>NACA 0018 Fairing (0.4mm vase)</u></font></a>", table_cell)],
        [Paragraph("<b>[2]</b>", table_cell_bold), Paragraph("<a href='https://agfrc.com'><font color='#0284c7'><u>Dual AGFRC B24CLM (7 kg·cm)</u></font></a>", table_cell)],
        [Paragraph("<b>[3]</b>", table_cell_bold), Paragraph("<a href='file:///Users/drbell/Code/flight/cad/manufactured/v10_stagecoach_sector_drum.stl'><font color='#0284c7'><u>Dual 6800-2RS Bearings (10x19x5)</u></font></a>", table_cell)],
        [Paragraph("<b>[4]</b>", table_cell_bold), Paragraph("<a href='file:///Users/drbell/Code/flight/cad/manufactured/v10_stagecoach_sector_drum.stl'><font color='#0284c7'><u>Stage-Coach Drum (25.7g)</u></font></a>", table_cell)],
        [Paragraph("<b>[5]</b>", table_cell_bold), Paragraph("<a href='file:///Users/drbell/Code/flight/cad/manufactured/v9_differential_gimbal_yoke.stl'><font color='#0284c7'><u>Diff Gimbal Yoke &amp; Collar</u></font></a>", table_cell)],
        [Paragraph("<b>[6]</b>", table_cell_bold), Paragraph("<a href='file:///Users/drbell/Code/flight/cad/manufactured/v9_quickswap_bevel_gear.stl'><font color='#0284c7'><u>3x Quick-Swap Gears (m=1.0)</u></font></a>", table_cell)],
        [Paragraph("<b>[7]</b>", table_cell_bold), Paragraph("<a href='file:///Users/drbell/Code/flight/cad/manufactured/v9_differential_gimbal_yoke.stl'><font color='#0284c7'><u>Ø4x30mm Steel Dowel Axle</u></font></a>", table_cell)],
        [Paragraph("<b>[8]</b>", table_cell_bold), Paragraph("<a href='file:///Users/drbell/Code/flight/cad/manufactured/v9_topological_spar.stl'><font color='#0284c7'><u>Ø10mm Carbon Wing Spar</u></font></a>", table_cell)],
        [Paragraph("<b>[9]</b>", table_cell_bold), Paragraph("<a href='file:///Users/drbell/Code/flight/cad/manufactured/v9_stepped_pinion_assembly.stl'><font color='#0284c7'><u>Stepped Pinion Drum / 5010</u></font></a>", table_cell)],
        [Paragraph("<b>[10]</b>", table_cell_bold), Paragraph("<a href='file:///Users/drbell/Code/flight/cad/manufactured/v10_stagecoach_sector_drum.stl'><font color='#0284c7'><u>0.75mm Dyneema SK75 Line</u></font></a>", table_cell)]
    ]
    t_legend = Table(legend_data, colWidths=[0.55*inch, 2.05*inch])
    t_legend.setStyle(TableStyle([
        ('BACKGROUND', (0,0), (-1,0), ICE_BG),
        ('GRID', (0,0), (-1,-1), 0.5, BORDER_GRAY),
        ('VALIGN', (0,0), (-1,-1), 'MIDDLE'),
        ('BOTTOMPADDING', (0,0), (-1,-1), 2.5),
        ('TOPPADDING', (0,0), (-1,-1), 2.5),
        ('LEFTPADDING', (0,0), (-1,-1), 3),
        ('RIGHTPADDING', (0,0), (-1,-1), 3),
    ]))

    t_top = Table([[exploded_img, t_legend]], colWidths=[4.7*inch, 2.7*inch])
    t_top.setStyle(TableStyle([
        ('VALIGN', (0,0), (-1,-1), 'TOP'),
        ('LEFTPADDING', (0,0), (-1,-1), 0),
        ('RIGHTPADDING', (0,0), (-1,-1), 0),
        ('TOPPADDING', (0,0), (-1,-1), 0),
        ('BOTTOMPADDING', (0,0), (-1,-1), 0),
    ]))
    story.append(t_top)
    story.append(Spacer(1, 6))

    # SECTION 1: 3D PRINTING MANUFACTURING BOM WITH LIVE LINKS
    story.append(Paragraph("1. 3D PRINT MANUFACTURING SPECIFICATIONS &amp; BOM (PER WING ROOT)", section_h1))
    
    bom_data = [
        [
            Paragraph("<b>Part File (.stl / .step)</b>", table_cell_bold),
            Paragraph("<b>Material</b>", table_cell_bold),
            Paragraph("<b>Qty</b>", table_cell_bold),
            Paragraph("<b>Nozzle &amp; Layer</b>", table_cell_bold),
            Paragraph("<b>Walls &amp; Infill Mandate</b>", table_cell_bold),
            Paragraph("<b>Mass</b>", table_cell_bold),
            Paragraph("<b>Unit Cost</b>", table_cell_bold)
        ],
        [
            Paragraph("<a href='file:///Users/drbell/Code/flight/cad/manufactured/v10_stagecoach_sector_drum.stl'><font color='#0284c7'><b><u>v10_stagecoach_drum</u></b></font></a><br/><a href='file:///Users/drbell/Code/flight/cad/manufactured/v10_stagecoach_sector_drum.stl'><font color='#0284c7'>[STL]</font></a> <a href='file:///Users/drbell/Code/flight/cad/manufactured/v10_stagecoach_sector_drum.step'><font color='#64748b'>[STEP]</font></a>", table_cell),
            Paragraph("PA12-CF / PA6-CF", table_cell),
            Paragraph("1x", table_cell),
            Paragraph("0.4mm / 0.16mm", table_cell),
            Paragraph("7 radiating spokes, 5 rim perimeters, open gates", table_cell),
            Paragraph("<b>25.7 g</b>", table_cell),
            Paragraph("~$1.15", table_cell)
        ],
        [
            Paragraph("<a href='file:///Users/drbell/Code/flight/cad/manufactured/v9_quickswap_bevel_gear.stl'><font color='#0284c7'><b><u>v9_quickswap_bevel_gear</u></b></font></a><br/><a href='file:///Users/drbell/Code/flight/cad/manufactured/v9_quickswap_bevel_gear.stl'><font color='#0284c7'>[STL]</font></a> <a href='file:///Users/drbell/Code/flight/cad/manufactured/v9_quickswap_bevel_gear.step'><font color='#64748b'>[STEP]</font></a>", table_cell),
            Paragraph("PA-CF or Tough Resin", table_cell),
            Paragraph("3x", table_cell),
            Paragraph("0.4mm / 0.12mm (35µm)", table_cell),
            Paragraph("<b>≥ 5 walls (100% infill)</b>, flat on back", table_cell),
            Paragraph("4.5 g (set)", table_cell),
            Paragraph("<b>$0.20 (set)</b>", table_cell)
        ],
        [
            Paragraph("<a href='file:///Users/drbell/Code/flight/cad/manufactured/v9_differential_gimbal_yoke.stl'><font color='#0284c7'><b><u>v9_diff_gimbal_yoke</u></b></font></a><br/><a href='file:///Users/drbell/Code/flight/cad/manufactured/v9_differential_gimbal_yoke.stl'><font color='#0284c7'>[STL]</font></a> <a href='file:///Users/drbell/Code/flight/cad/manufactured/v9_differential_gimbal_yoke.step'><font color='#64748b'>[STEP]</font></a>", table_cell),
            Paragraph("PA12-CF or PET-CF", table_cell),
            Paragraph("1x", table_cell),
            Paragraph("0.4mm / 0.12mm", table_cell),
            Paragraph("6 walls, 100% solid perimeters", table_cell),
            Paragraph("8.2 g", table_cell),
            Paragraph("~$0.37", table_cell)
        ],
        [
            Paragraph("<a href='file:///Users/drbell/Code/flight/cad/manufactured/v9_topological_fairing.stl'><font color='#0284c7'><b><u>v9_topological_fairing</u></b></font></a><br/><a href='file:///Users/drbell/Code/flight/cad/manufactured/v9_topological_fairing.stl'><font color='#0284c7'>[STL]</font></a> <a href='file:///Users/drbell/Code/flight/cad/manufactured/v9_topological_fairing.step'><font color='#64748b'>[STEP]</font></a>", table_cell),
            Paragraph("Foaming LW-PLA", table_cell),
            Paragraph("1x", table_cell),
            Paragraph("0.4mm / 0.20mm", table_cell),
            Paragraph("1 perimeter (0.45mm), 0% infill (vase)", table_cell),
            Paragraph("5.8 g", table_cell),
            Paragraph("~$0.25", table_cell)
        ],
        [
            Paragraph("<a href='file:///Users/drbell/Code/flight/cad/manufactured/v9_stepped_pinion_assembly.stl'><font color='#0284c7'><b><u>v9_stepped_pinion</u></b></font></a><br/><a href='file:///Users/drbell/Code/flight/cad/manufactured/v9_stepped_pinion_assembly.stl'><font color='#0284c7'>[STL]</font></a> <a href='file:///Users/drbell/Code/flight/cad/manufactured/v9_stepped_pinion_assembly.step'><font color='#64748b'>[STEP]</font></a>", table_cell),
            Paragraph("High-Temp PA-CF / 6061", table_cell),
            Paragraph("1x", table_cell),
            Paragraph("0.4mm / 0.12mm", table_cell),
            Paragraph("100% solid, horizontal orientation", table_cell),
            Paragraph("2.1 g", table_cell),
            Paragraph("~$0.10", table_cell)
        ]
    ]
    t_bom = Table(bom_data, colWidths=[1.85*inch, 1.05*inch, 0.35*inch, 0.95*inch, 2.05*inch, 0.55*inch, 0.65*inch])
    t_bom.setStyle(TableStyle([
        ('BACKGROUND', (0,0), (-1,0), NAVY_PRIMARY),
        ('TEXTCOLOR', (0,0), (-1,0), colors.white),
        ('GRID', (0,0), (-1,-1), 0.5, BORDER_GRAY),
        ('VALIGN', (0,0), (-1,-1), 'MIDDLE'),
        ('BOTTOMPADDING', (0,0), (-1,-1), 2.2),
        ('TOPPADDING', (0,0), (-1,-1), 2.2),
        ('ROWBACKGROUNDS', (0,1), (-1,-1), [colors.white, ICE_BG]),
    ]))
    for r in range(1):
        for c in range(len(bom_data[0])):
            t_bom.setStyle(TableStyle([('TEXTCOLOR', (c, 0), (c, 0), colors.white)]))
    story.append(t_bom)
    story.append(Spacer(1, 6))

    # SECTION 2: OTS HARDWARE & CRITICAL MANDATE CALLOUT
    story.append(Paragraph("2. HARDWARE CHECKLIST &amp; SLICER QUALITY GATES", section_h1))
    
    hw_data = [
        [
            Paragraph("<b>OTS Hardware Component</b>", table_cell_bold),
            Paragraph("<b>Specification &amp; Grade</b>", table_cell_bold),
            Paragraph("<b>Qty</b>", table_cell_bold),
            Paragraph("<b>Assembly Function &amp; Torque Spec</b>", table_cell_bold)
        ],
        [
            Paragraph("<a href='file:///Users/drbell/Code/flight/cad/manufactured/v9_topological_sector_drum.stl'><font color='#0284c7'><u>Flapping Hub Bearings</u></font></a>", table_cell),
            Paragraph("6800-2RS (10mm ID x 19mm OD x 5mm W)", table_cell),
            Paragraph("2x", table_cell),
            Paragraph("Dual radial-thrust couple; light press-fit into drum", table_cell)
        ],
        [
            Paragraph("<a href='file:///Users/drbell/Code/flight/cad/manufactured/v9_differential_gimbal_yoke.stl'><font color='#0284c7'><u>Gimbal Cross-Axle</u></font></a>", table_cell),
            Paragraph("Ø4.0mm x 30.0mm Ground Steel Dowel Pin (h6)", table_cell),
            Paragraph("1x", table_cell),
            Paragraph("Carries 4g lift bending &amp; bevel gears; zero flex", table_cell)
        ],
        [
            Paragraph("<a href='https://agfrc.com'><font color='#0284c7'><u>Actuator Micro-Servos</u></font></a>", table_cell),
            Paragraph("AGFRC B24CLM (7.0 kg·cm @ 8.4V, Coreless)", table_cell),
            Paragraph("2x", table_cell),
            Paragraph("Feathering &amp; sweep differential; hex output spline", table_cell)
        ],
        [
            Paragraph("<a href='file:///Users/drbell/Code/flight/cad/manufactured/v9_topological_sector_drum.stl'><font color='#0284c7'><u>Capstan Drive Cable</u></font></a>", table_cell),
            Paragraph("0.75mm Dyneema SK75/SK99 (12-strand braided)", table_cell),
            Paragraph("~1m", table_cell),
            Paragraph("D:d = 10.0:1 on Ø7.5mm pinion; 180N pre-tension", table_cell)
        ],
        [
            Paragraph("<a href='file:///Users/drbell/Code/flight/cad/manufactured/v9_differential_bevel_wrist_cartridge.stl'><font color='#0284c7'><u>Fastener Assortment</u></font></a>", table_cell),
            Paragraph("Grade 12.9 Socket Head Cap Screws", table_cell),
            Paragraph("Kit", table_cell),
            Paragraph("3x M2x8 (gears), 2x M2.5x12 (clamp), 2x M2.5x6 (cable), 1x M3x5 (pinion)", table_cell)
        ]
    ]
    t_hw = Table(hw_data, colWidths=[1.5*inch, 2.3*inch, 0.4*inch, 3.25*inch])
    t_hw.setStyle(TableStyle([
        ('BACKGROUND', (0,0), (-1,0), ICE_BG),
        ('GRID', (0,0), (-1,-1), 0.5, BORDER_GRAY),
        ('VALIGN', (0,0), (-1,-1), 'MIDDLE'),
        ('BOTTOMPADDING', (0,0), (-1,-1), 2.2),
        ('TOPPADDING', (0,0), (-1,-1), 2.2),
    ]))
    story.append(t_hw)
    story.append(Spacer(1, 4))

    # Red-Team Slicing Callout Box with Hyperlink
    alert_data = [[
        Paragraph(
            "<b>CRITICAL RED-TEAM SLICING MANDATE:</b> Quick-swap bevel gears (<a href='file:///Users/drbell/Code/flight/cad/manufactured/v9_quickswap_bevel_gear.stl'><font color='#b45309'><u>v9_quickswap_bevel_gear.stl</u></font></a>) <b>must be sliced with ≥ 5 concentric wall perimeters</b> (or 100% concentric infill). Slicing with grid/rectilinear infill leaves the 5.05mm hex hub corners unsupported and vulnerable to radial wedging burst under 7.0 kg·cm servo stall torque. Pre-treat gear teeth with dry PTFE aerosol or MoS₂ powder before final mesh. <a href='file:///Users/drbell/.gemini/antigravity/brain/a5a482f2-2cea-48ab-945b-be0d2fd94a84/v9_wrist_adversarial_engineering_audit.md'><font color='#b45309'>[See Audit Section 3]</font></a>",
            callout_box
        )
    ]]
    t_alert = Table(alert_data, colWidths=[7.45*inch])
    t_alert.setStyle(TableStyle([
        ('BACKGROUND', (0,0), (-1,-1), colors.HexColor("#fef3c7")),
        ('BOX', (0,0), (-1,-1), 1, WARN_AMBER),
        ('TOPPADDING', (0,0), (-1,-1), 3),
        ('BOTTOMPADDING', (0,0), (-1,-1), 3),
        ('LEFTPADDING', (0,0), (-1,-1), 6),
        ('RIGHTPADDING', (0,0), (-1,-1), 6),
    ]))
    story.append(t_alert)

    story.append(PageBreak())

    # =========================================================================
    # PAGE 2: STEP-BY-STEP PICTORIAL ASSEMBLY SEQUENCE WITH HYPERLINKS
    # =========================================================================
    story.append(Paragraph("3. STEP-BY-STEP PICTORIAL BENCH ASSEMBLY SEQUENCE | <a href='file:///Users/drbell/Code/flight/cad/manufactured/v9_differential_bevel_wrist_cartridge.step'><font color='#0284c7'><u>[Open Master Cartridge 3D STEP]</u></font></a>", section_h1))
    story.append(Paragraph("Follow stages 1 through 6 in exact sequence. Click any stage heading or part link to inspect full 3D geometry.", body_style))
    story.append(Spacer(1, 4))

    step_img_w = 1.35 * inch
    step_img_h = 0.85 * inch

    img_bench_gear = Image(photo_gear_path, width=step_img_w, height=step_img_h)
    img_diff_cad = Image("cad/manufactured/renders_v9/v9_09_differential_cartridge_assembly.png", width=step_img_w, height=step_img_h)
    img_trumpet = Image("cad/manufactured/renders_v10/v10_04_front_elevation_ram_air_vents.png", width=step_img_w, height=step_img_h)
    img_planform = Image("cad/manufactured/renders_v10/v10_03_top_planform_stagecoach_spokes.png", width=step_img_w, height=step_img_h)
    img_diff_photo = Image(photo_stagecoach_path, width=step_img_w, height=step_img_h)
    img_fairing = Image(photo_fairing_path, width=step_img_w, height=step_img_h)

    def make_step_card(step_num, title, target_link, text_lines, img_obj, gate_text):
        content = [
            Paragraph(f"<b><a href='{target_link}'><font color='#0f172a'>STAGE {step_num}: {title.upper()}</font></a></b>", table_cell_bold),
            Spacer(1, 2)
        ]
        for line in text_lines:
            content.append(Paragraph(f"• {line}", table_cell))
        content.append(Spacer(1, 2))
        content.append(Paragraph(f"<b>VERIFICATION GATE:</b> <font color='#16a34a'>{gate_text}</font>", table_cell))
        
        t_card = Table([[img_obj, content]], colWidths=[1.45*inch, 2.15*inch])
        t_card.setStyle(TableStyle([
            ('VALIGN', (0,0), (-1,-1), 'TOP'),
            ('LEFTPADDING', (0,0), (-1,-1), 2),
            ('RIGHTPADDING', (0,0), (-1,-1), 2),
            ('TOPPADDING', (0,0), (-1,-1), 2),
            ('BOTTOMPADDING', (0,0), (-1,-1), 2),
        ]))
        return t_card

    card_s1 = make_step_card(
        "1", "Bearing Hub Press-Fit",
        "file:///Users/drbell/Code/flight/cad/manufactured/v9_topological_sector_drum.stl",
        [
            "Inspect PA12-CF sector drum bearing seats (19.0mm OD).",
            "Press-fit lower 6800-2RS bearing flush to Z=0.0mm datum.",
            "Press-fit upper 6800-2RS bearing flush to Z=28.0mm top deck.",
            "Ensure central Ø10mm wiring pass-through bore is clear."
        ],
        img_planform,
        "Bearings spin freely with zero axial tilt or binding."
    )

    card_s2 = make_step_card(
        "2", "Differential Cartridge Assembly",
        "file:///Users/drbell/Code/flight/cad/manufactured/v9_differential_bevel_wrist_cartridge.stl",
        [
            "Slide central spider bevel gear onto yoke aft hex boss.",
            "Thread M2x8mm axial cap screw into spider gear counterbore (0.35 N·m).",
            "Insert Ø4.0mm x 30mm ground steel cross-axle through yoke bore.",
            "Slide left and right input bevel gears onto axle ends."
        ],
        img_diff_cad,
        "Bevel gearset meshes smoothly; backlash between 0.2° and 0.4°."
    )

    card_s3 = make_step_card(
        "3", "Servo Installation & Wiring",
        "https://agfrc.com",
        [
            "Drop dual AGFRC B24CLM micro-servos into isolated bays.",
            "Verify 1.8mm center web isolates servo cases at X=0.",
            "Route servo wiring leads down through bottom chimney vents.",
            "Engage servo output hex splines into left/right input gears."
        ],
        img_diff_photo,
        "Servo case heatsinks align with bottom ram-air scoops."
    )

    card_s4 = make_step_card(
        "4", "Wing Spar Clamp & Trumpet Fit",
        "file:///Users/drbell/Code/flight/cad/manufactured/v9_differential_gimbal_yoke.stl",
        [
            "Insert gimbal yoke assembly through front drum cavity.",
            "Slide Ø10.0mm carbon spar tube 26mm deep into clamp collar.",
            "Torque 2x M2.5x12mm pinch bolts to 0.70 N·m (clamp locked).",
            "Articulate spar: check -15° feathering to +36° downstroke."
        ],
        img_trumpet,
        "No spar contact on trumpet perimeter; ≥ 2.0mm air gap."
    )

    card_s5 = make_step_card(
        "5", "0.75mm Dyneema Capstan Rigging",
        "file:///Users/drbell/Code/flight/cad/manufactured/v9_stepped_pinion_assembly.stl",
        [
            "Anchor Dyneema line at terminal -190° with M2.5 clamp washer.",
            "Wrap line around 210° continuous rim (Z=2.8-5.4mm groove).",
            "Wrap 3 full figure-8 turns around Ø7.5mm stepped pinion drum.",
            "Tension line to 180 N; lock M2.5 terminal anchor screw (0.40 N·m).",
            "Tighten M3x5mm grub screw onto motor shaft D-flat (1.10 N·m)."
        ],
        img_bench_gear,
        "Zero slip at 220 N peak reversal; drum tracks 10:1 ratio."
    )

    card_s6 = make_step_card(
        "6", "NACA 0018 Fairing & Final Q/A",
        "file:///Users/drbell/Code/flight/cad/manufactured/v9_topological_fairing.stl",
        [
            "Slide LW-PLA aerodynamic fairing over assembled wing root.",
            "Confirm nose aperture centers over carbon spar tube.",
            "Verify belly NACA scoop connects to servo bottom vents.",
            "Connect servo harness to flight controller; sweep pitch/roll."
        ],
        img_fairing,
        "Full ±36° pitch and 210° flapping stroke with zero friction."
    )

    grid_stages = [
        [card_s1, card_s2],
        [card_s3, card_s4],
        [card_s5, card_s6]
    ]
    t_stages = Table(grid_stages, colWidths=[3.72*inch, 3.72*inch])
    t_stages.setStyle(TableStyle([
        ('BOX', (0,0), (0,0), 0.5, BORDER_GRAY),
        ('BOX', (1,0), (1,0), 0.5, BORDER_GRAY),
        ('BOX', (0,1), (0,1), 0.5, BORDER_GRAY),
        ('BOX', (1,1), (1,1), 0.5, BORDER_GRAY),
        ('BOX', (0,2), (0,2), 0.5, BORDER_GRAY),
        ('BOX', (1,2), (1,2), 0.5, BORDER_GRAY),
        ('BACKGROUND', (0,0), (-1,-1), colors.white),
        ('VALIGN', (0,0), (-1,-1), 'TOP'),
        ('LEFTPADDING', (0,0), (-1,-1), 4),
        ('RIGHTPADDING', (0,0), (-1,-1), 4),
        ('TOPPADDING', (0,0), (-1,-1), 3),
        ('BOTTOMPADDING', (0,0), (-1,-1), 3),
    ]))
    story.append(t_stages)
    story.append(Spacer(1, 4))

    # SECTION 4: 90-SECOND QUICK-SWAP PROTOCOL WITH HYPERLINK
    quickswap_box = [
        [
            Paragraph("<b>FIELD PROTOCOL: 90-SECOND QUICK-SWAP BEVEL GEAR REPLACEMENT</b> | <a href='file:///Users/drbell/Code/flight/cad/manufactured/v9_quickswap_bevel_gear.stl'><font color='#0284c7'><u>[Direct STL Download: v9_quickswap_bevel_gear.stl]</u></font></a>", table_cell_bold)
        ],
        [
            Paragraph(
                "<b>Inspection Trigger:</b> If pre-flight bench checks detect gear backlash exceeding <b>1.0°</b> or tooth flank wear, initiate cartridge swap: <br/>"
                "<b>1.</b> Remove fairing shell (2x M2 nose retaining clips). <br/>"
                "<b>2.</b> Unthread axial M2x8mm socket cap screw from worn bevel gear. <br/>"
                "<b>3.</b> Slide worn gear off 5.05mm hex drive shaft; slide on fresh 3D-printed replacement gear (<a href='file:///Users/drbell/Code/flight/cad/manufactured/v9_quickswap_bevel_gear.stl'><font color='#0284c7'><u>v9_quickswap_bevel_gear.stl</u></font></a>, pre-treated with dry PTFE). <br/>"
                "<b>4.</b> Re-torque M2 screw to 0.35 N·m; verify backlash &lt; 0.3°. Re-install fairing. <i>Total downtime: &lt; 90 seconds. Cost: $0.07.</i>",
                body_style
            )
        ]
    ]
    t_qs = Table(quickswap_box, colWidths=[7.45*inch])
    t_qs.setStyle(TableStyle([
        ('BACKGROUND', (0,0), (-1,-1), ICE_BG),
        ('BOX', (0,0), (-1,-1), 0.8, BLUE_ACCENT),
        ('TOPPADDING', (0,0), (-1,-1), 3),
        ('BOTTOMPADDING', (0,0), (-1,-1), 3),
        ('LEFTPADDING', (0,0), (-1,-1), 6),
        ('RIGHTPADDING', (0,0), (-1,-1), 6),
    ]))
    story.append(t_qs)

    doc.build(story, canvasmaker=NumberedCanvas)
    
    # Copy to artifact path
    with open(DOC_OUT, 'rb') as f_in, open(ART_OUT, 'wb') as f_out:
        f_out.write(f_in.read())

    print(f"Publication-grade PDF successfully created:")
    print(f"  Local  : {DOC_OUT}")
    print(f"  Artifact: {ART_OUT}")

if __name__ == '__main__':
    build_pdf()
