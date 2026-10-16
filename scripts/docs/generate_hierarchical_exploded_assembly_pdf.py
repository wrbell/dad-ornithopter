#!/usr/bin/env python3
"""
scripts/generate_hierarchical_exploded_assembly_pdf.py
======================================================
Generates the publication-grade 2-page printable assembly manual PDF:
  - docs/MCO-1D_Hierarchical_Exploded_Installation_Guide.pdf
  - <artifactDir>/MCO-1D_Hierarchical_Exploded_Installation_Guide.pdf

Shows how:
  Tier 1: Wrist (Beveled Gear) System installs into...
  Tier 2: Flapping (Shoulder) Mechanism, which in turn attaches to...
  Tier 3: Fuselage via the Scaffolding Brace (Topological Collar clamped to spine).
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

# Professional Aerospace Palette
NAVY_PRIMARY = colors.HexColor("#0f172a")   # Slate 900
BLUE_ACCENT  = colors.HexColor("#0284c7")   # Sky 600
ICE_BG       = colors.HexColor("#f8fafc")   # Slate 50
BORDER_GRAY  = colors.HexColor("#cbd5e1")   # Slate 300
DARK_TEXT    = colors.HexColor("#1e293b")   # Slate 800
MUTED_TEXT   = colors.HexColor("#64748b")   # Slate 500
CARD_BG      = colors.HexColor("#ffffff")
WARN_AMBER   = colors.HexColor("#d97706")   # Amber 600
SUCCESS_GRN  = colors.HexColor("#16a34a")   # Green 600
BADGE_BG     = colors.HexColor("#e0f2fe")   # Sky 100

DOC_OUT = "docs/MCO-1D_Hierarchical_Exploded_Installation_Guide.pdf"
ARTIFACT_DIR = "/Users/drbell/.gemini/antigravity/brain/a5a482f2-2cea-48ab-945b-be0d2fd94a84"
ART_OUT = os.path.join(ARTIFACT_DIR, "MCO-1D_Hierarchical_Exploded_Installation_Guide.pdf")

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

    def draw_page_decorations(self, num_pages):
        self.saveState()
        self.setStrokeColor(BORDER_GRAY)
        self.setLineWidth(0.75)
        self.line(36, 11 * 72 - 32, 8.5 * 72 - 36, 11 * 72 - 32)
        
        self.setFont("Helvetica-Bold", 8)
        self.setFillColor(NAVY_PRIMARY)
        self.drawString(36, 11 * 72 - 26, "MCO-1D HIERARCHICAL EXPLODED INSTALLATION GUIDE")
        self.setFont("Helvetica", 8)
        self.setFillColor(MUTED_TEXT)
        self.drawRightString(8.5 * 72 - 36, 11 * 72 - 26, "OCTOBER 2026 | CDR PRODUCTION BASELINE")

        self.setStrokeColor(BORDER_GRAY)
        self.setLineWidth(0.75)
        self.line(36, 32, 8.5 * 72 - 36, 32)
        
        self.setFont("Helvetica", 8)
        self.setFillColor(MUTED_TEXT)
        self.drawString(36, 22, "CONFIDENTIAL & PROPRIETARY — AUTONOMOUS FLAPPING WING PROGRAM")
        page_text = f"Page {self._pageNumber} of {num_pages}"
        self.drawRightString(8.5 * 72 - 36, 22, page_text)
        self.restoreState()

def build_pdf():
    doc = SimpleDocTemplate(
        DOC_OUT,
        pagesize=letter,
        leftMargin=36,
        rightMargin=36,
        topMargin=40,
        bottomMargin=40
    )

    styles = getSampleStyleSheet()
    
    title_style = ParagraphStyle(
        'DocTitle',
        parent=styles['Normal'],
        fontName='Helvetica-Bold',
        fontSize=17,
        leading=21,
        textColor=NAVY_PRIMARY,
        spaceAfter=2
    )

    subtitle_style = ParagraphStyle(
        'DocSubtitle',
        parent=styles['Normal'],
        fontName='Helvetica-Bold',
        fontSize=9.0,
        leading=12,
        textColor=BLUE_ACCENT,
        spaceAfter=6
    )

    h2_style = ParagraphStyle(
        'H2',
        parent=styles['Normal'],
        fontName='Helvetica-Bold',
        fontSize=10.5,
        leading=13.5,
        textColor=NAVY_PRIMARY,
        spaceBefore=5,
        spaceAfter=3
    )

    body_style = ParagraphStyle(
        'Body',
        parent=styles['Normal'],
        fontName='Helvetica',
        fontSize=7.8,
        leading=10.5,
        textColor=DARK_TEXT
    )

    table_cell = ParagraphStyle(
        'TableCell',
        parent=styles['Normal'],
        fontName='Helvetica',
        fontSize=7.2,
        leading=9.2,
        textColor=DARK_TEXT
    )

    table_header = ParagraphStyle(
        'TableHeader',
        parent=styles['Normal'],
        fontName='Helvetica-Bold',
        fontSize=7.4,
        leading=9.5,
        textColor=NAVY_PRIMARY
    )

    story = []

    # =========================================================================
    # PAGE 1: 3-TIER HIERARCHY, MASTER EXPLODED DIAGRAM & COMPONENT BOM
    # =========================================================================
    story.append(Paragraph("MCO-1D Hierarchical Exploded Installation Guide", title_style))
    story.append(Paragraph("2-DOF Differential Bevel Wrist → V10 Flapping Shoulder → Scaffolding Brace → Fuselage Carbon Spine", subtitle_style))

    # Metric Banner
    banner_data = [
        [
            Paragraph("<b>TIER 1: WRIST</b><br/><font size='9.5' color='#0284c7'><b>Bevel Gearset</b></font><br/><i>2-DOF Spar Vectoring</i>", table_cell),
            Paragraph("<b>TIER 2: SHOULDER</b><br/><font size='9.5' color='#16a34a'><b>Stagecoach Drum</b></font><br/><i>10:1 Capstan (30.5g)</i>", table_cell),
            Paragraph("<b>TIER 3: SCAFFOLDING</b><br/><font size='9.5' color='#d97706'><b>Topological Collar</b></font><br/><i>Dual Clevis (Y=±35mm)</i>", table_cell),
            Paragraph("<b>FUSELAGE SPINE</b><br/><font size='9.5' color='#0f172a'><b>Ø20.0mm Carbon</b></font><br/><i>Modular Stations</i>", table_cell),
            Paragraph("<b>TOTAL ASSEMBLY</b><br/><font size='9.5' color='#0284c7'><b>73.9g Plastic</b></font><br/><i>-82% vs V2 Baseline</i>", table_cell),
        ]
    ]
    t_banner = Table(banner_data, colWidths=[108, 114, 114, 104, 100])
    t_banner.setStyle(TableStyle([
        ('BACKGROUND', (0,0), (-1,-1), ICE_BG),
        ('BOX', (0,0), (-1,-1), 0.75, BORDER_GRAY),
        ('INNERGRID', (0,0), (-1,-1), 0.5, BORDER_GRAY),
        ('TOPPADDING', (0,0), (-1,-1), 3),
        ('BOTTOMPADDING', (0,0), (-1,-1), 3),
        ('ALIGN', (0,0), (-1,-1), 'CENTER'),
    ]))
    story.append(t_banner)
    story.append(Spacer(1, 5))

    # Master Exploded View Diagram
    img_render1 = "cad/manufactured/renders_exploded/mco_01_hierarchical_exploded_isometric.png"
    if os.path.exists(img_render1):
        story.append(Image(img_render1, width=7.4 * inch, height=3.05 * inch))
    story.append(Spacer(1, 5))

    # Hierarchical System Decomposition & 12-Component Bill of Materials Table
    story.append(Paragraph("1. Hierarchical Assembly Decomposition & Component Callout Checklist", h2_style))
    bom_data = [
        [
            Paragraph("Tier", table_header),
            Paragraph("ID", table_header),
            Paragraph("Component Name & Mechanical Function", table_header),
            Paragraph("Material & Manufacturing Process", table_header),
            Paragraph("Mass", table_header),
            Paragraph("Assembly Trajectory & Interface", table_header),
        ],
        [
            Paragraph("<b>Tier 3</b><br/>Fuselage & Scaffolding", table_cell),
            Paragraph("1<br/>2<br/>3", table_cell),
            Paragraph("<b>Fuselage Carbon Spine Longeron</b> (Ø20x1.5mm)<br/><b>Topological Scaffolding Collar</b> (Bilateral Mount)<br/><b>Split-Collar M3 Pinch Clamp Fasteners</b> (Grade 12.9)", table_cell),
            Paragraph("Pultruded 3K Carbon Fiber Tube<br/>PA12-CF Selective Laser Sintering<br/>Alloy Steel Fasteners (Torque 0.90 N·m)", table_cell),
            Paragraph("32.0 g<br/>34.5 g<br/>3.2 g", table_cell),
            Paragraph("Primary structural datum along X-axis<br/>Clamps to spine tube at X=95,275,455,635mm<br/>Compresses sleeve with 1,500 N preload", table_cell),
        ],
        [
            Paragraph("<b>Tier 2</b><br/>Flapping Shoulder", table_cell),
            Paragraph("4<br/>5<br/>6<br/>7<br/>8", table_cell),
            Paragraph("<b>V10 Stagecoach Sector Wheel</b> (7 Radiating Spokes)<br/><b>GoolRC 5010 BLDC Motor & Pinion Drum</b> (10:1)<br/><b>ODrive Micro Controller & Diametral Magnet</b><br/><b>Dual-Shear Top Brace</b> (Bearing Tie-Bar)<br/><b>Ø10mm Titanium Pivot Axle & 2x 6800-2RS Bearings</b>", table_cell),
            Paragraph("PA12-CF (Helical groove, bumper stops)<br/>OTS BLDC Motor + Stepped Aluminum Pinion<br/>PCB with MPS MA702 IC (1.5mm air gap)<br/>PA12-CF Bridge (688-2RS top bearing)<br/>Grade 5 Titanium Pin + ABEC-5 Bearings", table_cell),
            Paragraph("30.5 g<br/>92.0 g<br/>8.5 g<br/>5.8 g<br/>16.2 g", table_cell),
            Paragraph("Seats inside scaffolding clevis (Y=35mm)<br/>Bolts to slotted motor stanchion (C=41.25mm)<br/>Mounts under motor with 1.50mm air gap<br/>Locks top of pinion to Ø10mm main axle<br/>Press-fits into clevis lugs & drum hub", table_cell),
        ],
        [
            Paragraph("<b>Tier 1</b><br/>Wrist Bevel Cartridge", table_cell),
            Paragraph("9<br/>10<br/>11<br/>12<br/>13", table_cell),
            Paragraph("<b>Dual AGFRC B24CLM Micro-Servos</b> (Coreless Titanium)<br/><b>Differential Gimbal Yoke & Ø4mm Cross-Axle</b><br/><b>3x Quick-Swap 3D-Printed Bevel Gears</b> (Mod 0.75)<br/><b>Wing Spar Tube Socket</b> (Ø10mm Woven Carbon)<br/><b>Aerodynamic NACA 0018 Airfoil Fairing Shell</b>", table_cell),
            Paragraph("CNC Finned Aluminum Heat-Sink Bodies<br/>PA12-CF Yoke + Ground Stainless Axle<br/>PA12-CF Quick-Swap Bevel Gearset<br/>3K Twill High-Modulus Carbon Tube<br/>Lightweight Foaming PLA (LW-PLA)", table_cell),
            Paragraph("42.0 g<br/>7.8 g<br/>4.2 g<br/>18.5 g<br/>9.8 g", table_cell),
            Paragraph("Drops into open-air sector wheel bays<br/>Inserts into central gimbal cavity (+Y)<br/>Meshes inside yoke with zero backlash<br/>Bonds into gimbal socket, emerges forward<br/>Encloses mechanism with snap-fit latches", table_cell),
        ],
    ]
    t_bom = Table(bom_data, colWidths=[52, 18, 195, 125, 40, 110])
    t_bom.setStyle(TableStyle([
        ('BACKGROUND', (0,0), (-1,0), ICE_BG),
        ('BOX', (0,0), (-1,-1), 0.75, BORDER_GRAY),
        ('INNERGRID', (0,0), (-1,-1), 0.5, BORDER_GRAY),
        ('TOPPADDING', (0,0), (-1,-1), 2.5),
        ('BOTTOMPADDING', (0,0), (-1,-1), 2.5),
        ('VALIGN', (0,0), (-1,-1), 'MIDDLE'),
    ]))
    story.append(t_bom)

    story.append(PageBreak())

    # =========================================================================
    # PAGE 2: SUBSYSTEM DETAILS & 6-STAGE INSTALLATION PROTOCOL
    # =========================================================================
    story.append(Paragraph("2. Subsystem Installation Trajectories & 3-Tier Assembly Sequence", h2_style))

    # 3-Panel Visual Subsystem Layout
    img2 = "cad/manufactured/renders_exploded/mco_02_wrist_to_shoulder_closeup.png"
    img3 = "cad/manufactured/renders_exploded/mco_03_shoulder_to_scaffolding_brace.png"
    img4 = "cad/manufactured/renders_exploded/mco_04_side_elevation_exploded.png"

    sub_panels = [
        [
            Image(img3, width=2.4 * inch, height=1.45 * inch),
            Image(img2, width=2.4 * inch, height=1.45 * inch),
            Image(img4, width=2.4 * inch, height=1.45 * inch),
        ],
        [
            Paragraph("<b>A. Tier 2 → Tier 3 (Shoulder into Brace)</b><br/>Stagecoach drum docks into the double-shear clevis (29mm span) at Y=±35mm; Ø10mm pin secures bearings.", table_cell),
            Paragraph("<b>B. Tier 1 → Tier 2 (Wrist into Shoulder)</b><br/>Differential bevel yoke & gears insert through the front trumpet socket into the wheel core; servos drop into side bays.", table_cell),
            Paragraph("<b>C. Inline Load Path (Spine → Wing Tip)</b><br/>Side elevation reveals uninterrupted load trajectories: 220N capstan tension, 4.0g lift shear, and Dyneema wrap.", table_cell),
        ]
    ]
    t_sub = Table(sub_panels, colWidths=[180, 180, 180])
    t_sub.setStyle(TableStyle([
        ('BOX', (0,0), (-1,-1), 0.75, BORDER_GRAY),
        ('INNERGRID', (0,0), (-1,-1), 0.5, BORDER_GRAY),
        ('TOPPADDING', (0,0), (-1,-1), 2),
        ('BOTTOMPADDING', (0,0), (-1,-1), 2),
        ('BACKGROUND', (0,1), (-1,1), ICE_BG),
        ('ALIGN', (0,0), (-1,0), 'CENTER'),
    ]))
    story.append(t_sub)
    story.append(Spacer(1, 5))

    # Step-by-Step 6-Stage Hierarchical Installation Protocol
    story.append(Paragraph("3. 6-Stage Hierarchical Assembly & Commissioning Protocol", h2_style))

    steps_data = [
        [
            Paragraph("Stage", table_header),
            Paragraph("Subsystem & Mechanical Action", table_header),
            Paragraph("Fastener & Torque Spec", table_header),
            Paragraph("Quality Gate & Critical Verification Metric", table_header),
        ],
        [
            Paragraph("<b>ST-01</b>", table_cell),
            Paragraph("<b>Tier 3: Fuselage Spine Collar Clamping</b><br/>Slide topological scaffolding collar over Ø20.0mm carbon spine tube to designated station (X=95, 275, 455, or 635mm). Tighten split-collar pinch bolts symmetrically.", table_cell),
            Paragraph("Fasteners: <b>2x M3x14mm</b><br/>Torque: <b>0.90 N·m</b><br/>Loctite 243", table_cell),
            Paragraph("<b>Quality Gate</b>: Verify axial slip resistance (> 1,200 N thrust without movement). Confirm clevis parallelism across Y=±35mm.", table_cell),
        ],
        [
            Paragraph("<b>ST-02</b>", table_cell),
            Paragraph("<b>Tier 2: Motor, ODrive & Pinion Stanchion Mount</b><br/>Mount GoolRC 5010 BLDC motor to collar stanchion plate via tensioning slots. Fasten ODrive Micro PCB underneath motor. Press-fit stepped pinion onto shaft.", table_cell),
            Paragraph("Motor: <b>4x M3x10mm (0.85 N·m)</b><br/>ODrive: <b>4x M3x6mm (0.45 N·m)</b><br/>Pinion: <b>M3 set screw (1.1 N·m)</b>", table_cell),
            Paragraph("<b>Quality Gate</b>: Measure magnet-to-MA702 IC air gap using feeler gauge (<b>1.50 ± 0.15 mm</b>). Verify pinion axial runout < 0.02mm.", table_cell),
        ],
        [
            Paragraph("<b>ST-03</b>", table_cell),
            Paragraph("<b>Tier 2: Stagecoach Wheel Clevis Docking</b><br/>Press dual 6800-2RS bearings into sector drum. Position drum inside scaffolding clevis lugs (Z=-14 to +14mm). Insert Ø10.0mm titanium pivot pin through clevis and drum.", table_cell),
            Paragraph("H7/h6 press-fit<br/>Bearing lubricant:<br/>Klüber Isoflex Topas NB 52", table_cell),
            Paragraph("<b>Quality Gate</b>: Verify free, zero-play rotation over full ±60° flapping range. Ensure axial float between clevis lugs is 0.2 to 0.4 mm.", table_cell),
        ],
        [
            Paragraph("<b>ST-04</b>", table_cell),
            Paragraph("<b>Tier 2: Capstan Cable Rigging & Top Brace Tie-Down</b><br/>Anchor 0.75mm Dyneema SK99 cable into sector drum tensioning bolts. Wrap cable tangentially around stepped pinion. Slide Top Brace over axle and pinion; tighten tie-bolts.", table_cell),
            Paragraph("Cable Pre-Tension: <b>180 N</b><br/>Brace Bolts: <b>2x M3x16mm (0.90 N·m)</b><br/>Axle Clamp: <b>M3x12mm (0.80 N·m)</b>", table_cell),
            Paragraph("<b>Quality Gate</b>: Measure axis-to-axis deflection under 220N peak tension (must be <b>< 20 µm</b>). Verify cable stays centered in helical groove.", table_cell),
        ],
        [
            Paragraph("<b>ST-05</b>", table_cell),
            Paragraph("<b>Tier 1: Wrist Bevel Cartridge Insertion into Drum</b><br/>Slide differential gimbal yoke into drum front cavity. Insert Ø4mm cross-axle through side gears and spider gear. Drop dual AGFRC B24CLM servos into open cooling bays.", table_cell),
            Paragraph("Servo Mount: <b>4x M2.5x8mm</b><br/>Torque: <b>0.40 N·m</b><br/>Grease: Molykote EM-30L", table_cell),
            Paragraph("<b>Quality Gate</b>: Check bevel gear backlash (<b>0.05 to 0.08 mm</b>). Confirm servos seat flush with aluminum cooling fins 80% exposed to air.", table_cell),
        ],
        [
            Paragraph("<b>ST-06</b>", table_cell),
            Paragraph("<b>Tier 1: Wing Spar Socket Bonding & Fairing Snap</b><br/>Insert Ø10.0mm woven carbon wing spar tube into gimbal yoke socket; secure with transverse M2.5 titanium lock-pin. Snap lightweight NACA 0018 fairing over assembly.", table_cell),
            Paragraph("Lock Pin: <b>M2.5 Ti pin</b><br/>Fairing: <b>4x snap-latches</b><br/>Bond: 3M Scotch-Weld DP420", table_cell),
            Paragraph("<b>Quality Gate</b>: Verify full 2-DOF wrist range of motion (-15° to +36° feathering, ±18° sweep) without fairing contact or cable bind.", table_cell),
        ],
    ]

    t_steps = Table(steps_data, colWidths=[36, 210, 114, 180])
    t_steps.setStyle(TableStyle([
        ('BACKGROUND', (0,0), (-1,0), ICE_BG),
        ('BOX', (0,0), (-1,-1), 0.75, BORDER_GRAY),
        ('INNERGRID', (0,0), (-1,-1), 0.5, BORDER_GRAY),
        ('TOPPADDING', (0,0), (-1,-1), 2.2),
        ('BOTTOMPADDING', (0,0), (-1,-1), 2.2),
        ('VALIGN', (0,0), (-1,-1), 'MIDDLE'),
    ]))
    story.append(t_steps)
    story.append(Spacer(1, 5))

    # Structural Integrity & Load Path Footer
    alert_box = [
        [
            Paragraph(
                "<b>CLOSED-LOOP STRUCTURAL INTEGRITY CERTIFICATION:</b><br/>"
                "• <b>Transverse Tension Arch</b>: 220N flapping cable pull and 4.0g wing lift couples are reacted across the central scaffolding collar in pure tension, completely shielding the Ø20mm fuselage carbon spine from asymmetric bending fatigue.<br/>"
                "• <b>Rapid Line-Replaceable Unit (LRU)</b>: The entire Tier 1 wrist differential cartridge can be extracted and swapped in under 90 seconds by removing the Ø4mm cross-axle, without disturbing the Tier 2 capstan rigging or Tier 3 fuselage clamp.",
                table_cell
            )
        ]
    ]
    t_alert = Table(alert_box, colWidths=[540])
    t_alert.setStyle(TableStyle([
        ('BACKGROUND', (0,0), (-1,-1), colors.HexColor("#f0fdf4")),
        ('BOX', (0,0), (-1,-1), 1.0, SUCCESS_GRN),
        ('TOPPADDING', (0,0), (-1,-1), 3.5),
        ('BOTTOMPADDING', (0,0), (-1,-1), 3.5),
        ('LEFTPADDING', (0,0), (-1,-1), 8),
        ('RIGHTPADDING', (0,0), (-1,-1), 8),
    ]))
    story.append(t_alert)

    doc.build(story, canvasmaker=NumberedCanvas)
    
    if os.path.exists(DOC_OUT):
        import shutil
        shutil.copyfile(DOC_OUT, ART_OUT)
        print(f"Hierarchical Exploded PDF built successfully:")
        print(f"  • {DOC_OUT}")
        print(f"  • {ART_OUT}")

if __name__ == "__main__":
    build_pdf()
