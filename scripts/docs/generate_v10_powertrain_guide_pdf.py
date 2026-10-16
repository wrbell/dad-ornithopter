#!/usr/bin/env python3
"""
scripts/generate_v10_powertrain_guide_pdf.py
============================================
Generates the publication-grade 2-page printable assembly and installation manual PDF:
  - docs/MCO-1D_V10_Powertrain_Mount_Installation_Guide.pdf
  - <artifactDir>/MCO-1D_V10_Powertrain_Mount_Installation_Guide.pdf
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

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
DOC_OUT = os.path.join(REPO_ROOT, "docs", "mark1", "MCO-1D_V10_Powertrain_Mount_Installation_Guide.pdf")

os.makedirs(os.path.dirname(DOC_OUT), exist_ok=True)

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
        self.drawString(36, 11 * 72 - 26, "MCO-1D V10 POWERTRAIN MOUNT & CHASSIS ASSEMBLY MANUAL")
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
        fontSize=18,
        leading=22,
        textColor=NAVY_PRIMARY,
        spaceAfter=3
    )

    subtitle_style = ParagraphStyle(
        'DocSubtitle',
        parent=styles['Normal'],
        fontName='Helvetica-Bold',
        fontSize=9.5,
        leading=13,
        textColor=BLUE_ACCENT,
        spaceAfter=8
    )

    h2_style = ParagraphStyle(
        'H2',
        parent=styles['Normal'],
        fontName='Helvetica-Bold',
        fontSize=11,
        leading=14,
        textColor=NAVY_PRIMARY,
        spaceBefore=6,
        spaceAfter=4
    )

    body_style = ParagraphStyle(
        'Body',
        parent=styles['Normal'],
        fontName='Helvetica',
        fontSize=8,
        leading=11,
        textColor=DARK_TEXT
    )

    body_bold = ParagraphStyle(
        'BodyBold',
        parent=body_style,
        fontName='Helvetica-Bold'
    )

    table_cell = ParagraphStyle(
        'TableCell',
        parent=styles['Normal'],
        fontName='Helvetica',
        fontSize=7.2,
        leading=9.5,
        textColor=DARK_TEXT
    )

    table_header = ParagraphStyle(
        'TableHeader',
        parent=styles['Normal'],
        fontName='Helvetica-Bold',
        fontSize=7.5,
        leading=10,
        textColor=NAVY_PRIMARY
    )

    story = []

    # =========================================================================
    # PAGE 1: CHASSIS ARCHITECTURE, EXPLODED VIEW & MANUFACTURING SPECS
    # =========================================================================
    story.append(Paragraph("MCO-1D V10 Powertrain Mount, Top Brace & ODrive Guide", title_style))
    story.append(Paragraph("Topologically Optimized Dual-Shear Stand (37.7g), Top Brace (5.8g), ODrive Micro Enclosure & Fuselage Interface", subtitle_style))

    # Metric Banner
    banner_data = [
        [
            Paragraph("<b>TOTAL PLASTIC MASS</b><br/><font size='10' color='#0284c7'><b>73.9 g</b> / wing</font><br/><i>-82.0% vs V2 baseline</i>", table_cell),
            Paragraph("<b>AIRCRAFT WEIGHT SAVED</b><br/><font size='10' color='#16a34a'><b>-2.70 kg (-5.95 lbs)</b></font><br/><i>Across 8 wing roots</i>", table_cell),
            Paragraph("<b>AXIS DEFLECTION</b><br/><font size='10' color='#0f172a'><b>16.7 µm</b> @ 220N</font><br/><i>Allowable: < 25 µm</i>", table_cell),
            Paragraph("<b>ODRIVE AIR GAP</b><br/><font size='10' color='#d97706'><b>1.50 ± 0.005 mm</b></font><br/><i>Calibrated to MPS MA702</i>", table_cell),
            Paragraph("<b>FUSELAGE INTERFACE</b><br/><font size='10' color='#0f172a'><b>4x M3 + Ø6mm Ti</b></font><br/><i>At Y = ±35.0 mm flange</i>", table_cell),
        ]
    ]
    t_banner = Table(banner_data, colWidths=[108, 114, 104, 108, 106])
    t_banner.setStyle(TableStyle([
        ('BACKGROUND', (0,0), (-1,-1), ICE_BG),
        ('BOX', (0,0), (-1,-1), 0.75, BORDER_GRAY),
        ('INNERGRID', (0,0), (-1,-1), 0.5, BORDER_GRAY),
        ('TOPPADDING', (0,0), (-1,-1), 4),
        ('BOTTOMPADDING', (0,0), (-1,-1), 4),
        ('ALIGN', (0,0), (-1,-1), 'CENTER'),
    ]))
    story.append(t_banner)
    story.append(Spacer(1, 6))

    # Large CAD Diagram
    img_render1 = "cad/manufactured/renders_v10/v10_powertrain_01_full_assembly_isometric.png"
    if os.path.exists(img_render1):
        story.append(Image(img_render1, width=7.4 * inch, height=3.0 * inch))
    story.append(Spacer(1, 6))

    # 3D Print Manufacturing Specifications Table
    story.append(Paragraph("1. 3D Print Manufacturing & Hardware Bill of Materials (Per Wing Root)", h2_style))
    bom_data = [
        [
            Paragraph("Item", table_header),
            Paragraph("Part Name & Description", table_header),
            Paragraph("Material & Slicing Profile", table_header),
            Paragraph("Mass", table_header),
            Paragraph("Key Quality Gate / Inspection Metric", table_header),
        ],
        [
            Paragraph("1", table_cell),
            Paragraph("<b>V10 Powertrain Mount (Stand)</b><br/>GoolRC 5010 slotted motor deck, ODrive pocket, tower pedestal, fuselage bracket", table_cell),
            Paragraph("PA12-CF (Carbon Nylon)<br/>0.16mm layer, 6 walls, 60% infill", table_cell),
            Paragraph("37.7 g", table_cell),
            Paragraph("Check Ø19mm bearing bore & 26x26mm standoff flat", table_cell),
        ],
        [
            Paragraph("2", table_cell),
            Paragraph("<b>V10 Dual-Shear Top Brace</b><br/>Connecting tie-bar with 688 bearing seat and Ø10mm axle clamp", table_cell),
            Paragraph("PA12-CF (Carbon Nylon)<br/>0.16mm layer, 6 walls, 100% infill", table_cell),
            Paragraph("5.8 g", table_cell),
            Paragraph("Check Ø16mm bearing press-fit & M3 pinch slot", table_cell),
        ],
        [
            Paragraph("3", table_cell),
            Paragraph("<b>V10 Stagecoach Sector Drum</b><br/>7-spoke wheel, helical groove (Z:3.2-7.2mm), bump flanges, M3 bolts", table_cell),
            Paragraph("PA12-CF (Carbon Nylon)<br/>0.16mm layer, 4 walls, 35% gyroid", table_cell),
            Paragraph("30.5 g", table_cell),
            Paragraph("Verify unbroken helical rim & side cooling gates", table_cell),
        ],
        [
            Paragraph("4", table_cell),
            Paragraph("<b>GoolRC 5010 BLDC Motor</b><br/>360KV (or MN5008), 12N14P, Ø50mm OD stator", table_cell),
            Paragraph("OTS Motor Assembly<br/>With Ø6.35x2.5mm shaft magnet", table_cell),
            Paragraph("85.0 g", table_cell),
            Paragraph("Verify magnet runout < 0.03mm, phase insulation", table_cell),
        ],
        [
            Paragraph("5", table_cell),
            Paragraph("<b>ODrive Micro Controller</b><br/>32x32mm, 26x26mm M3 pattern, MPS MA702 IC", table_cell),
            Paragraph("ODrive Robotics PCB<br/>Firmware vel_limit=40 rps", table_cell),
            Paragraph("7.8 g", table_cell),
            Paragraph("Verify CAN-FD heartbeat & 1.50mm air gap to magnet", table_cell),
        ],
        [
            Paragraph("6", table_cell),
            Paragraph("<b>Precision Bearing Set</b><br/>2x 6800-2RS (19x10x5mm) + 1x 688-2RS (16x8x5mm)", table_cell),
            Paragraph("Chrome Steel / Ceramic Hybrid<br/>ABEC-5, dual rubber contact seals", table_cell),
            Paragraph("12.5 g", table_cell),
            Paragraph("Zero radial play, pre-packed low-drag Klüber grease", table_cell),
        ],
    ]
    t_bom = Table(bom_data, colWidths=[24, 185, 135, 46, 150])
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
    # PAGE 2: SUBSYSTEM INTEGRATION & BENCH ASSEMBLY PROTOCOL
    # =========================================================================
    story.append(Paragraph("2. Subsystem Integration: ODrive, Sector Wheel & Fuselage Mating", h2_style))

    # 3-Panel Visual Layout
    img2 = "cad/manufactured/renders_v10/v10_powertrain_02_odrive_encoder_undercarriage.png"
    img3 = "cad/manufactured/renders_v10/v10_powertrain_03_sector_wheel_groove_and_bumpers.png"
    img4 = "cad/manufactured/renders_v10/v10_powertrain_04_fuselage_spine_interface.png"

    sub_panels = [
        [
            Image(img2, width=2.4 * inch, height=1.5 * inch),
            Image(img3, width=2.4 * inch, height=1.5 * inch),
            Image(img4, width=2.4 * inch, height=1.5 * inch),
        ],
        [
            Paragraph("<b>A. ODrive Micro Integration</b><br/>Calibrated 1.50mm air gap between MPS MA702 IC and Ø6.35mm shaft magnet. 26x26mm M3 standoffs.", table_cell),
            Paragraph("<b>B. V10 Sector Wheel Features</b><br/>Helical tracking groove (Z: 3.2-7.2mm), R43.5mm hard-stop bumper flanges, and M3 Dyneema tensioning bolts.", table_cell),
            Paragraph("<b>C. Fuselage Spine Interface</b><br/>Vertical flange mating at Y=±35mm. 4x M3 (20x40mm pitch) + central Ø6.00mm Ti shear pin (SF=70.4x).", table_cell),
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
    story.append(Spacer(1, 6))

    # Step-by-Step Bench Assembly Sequence
    story.append(Paragraph("3. 6-Stage Bench Assembly & Calibration Protocol", h2_style))

    steps_data = [
        [
            Paragraph("Stage", table_header),
            Paragraph("Assembly Action & Tooling", table_header),
            Paragraph("Torque & Tolerance Spec", table_header),
            Paragraph("Critical Verification Quality Gate", table_header),
        ],
        [
            Paragraph("<b>ST-01</b>", table_cell),
            Paragraph("<b>GoolRC Motor & Magnet Installation</b><br/>Press Ø6.35x2.5mm diametral magnet into rear shaft flush with rotor face. Secure motor to Stand using 4x M3x12mm screws through tensioning slots.", table_cell),
            Paragraph("Torque: <b>0.85 N·m</b><br/>Blue Loctite 243", table_cell),
            Paragraph("Confirm magnet axial runout < 0.03mm using dial indicator.", table_cell),
        ],
        [
            Paragraph("<b>ST-02</b>", table_cell),
            Paragraph("<b>ODrive Micro Enclosure & Gap Check</b><br/>Seat ODrive Micro PCB into undercarriage pocket on 4x M3 standoffs. Connect 3-phase silicone leads and CAN-FD harness through escape ports.", table_cell),
            Paragraph("Torque: <b>0.45 N·m</b><br/>Nylon washers", table_cell),
            Paragraph("<b>Quality Gate</b>: Measure magnet-to-IC gap with feeler gauge (<b>1.50 ± 0.15 mm</b>).", table_cell),
        ],
        [
            Paragraph("<b>ST-03</b>", table_cell),
            Paragraph("<b>Sector Wheel & Axle Press-Fit</b><br/>Press dual 6800-2RS bearings into sector drum hub. Insert Ø10.0mm axle through Stand tower pedestal and sector wheel core.", table_cell),
            Paragraph("Light press-fit<br/>H7/h6 clearance", table_cell),
            Paragraph("Verify smooth, zero-backlash rotation over full ±60° flapping range.", table_cell),
        ],
        [
            Paragraph("<b>ST-04</b>", table_cell),
            Paragraph("<b>Dyneema Cable Rigging & Pre-Tensioning</b><br/>Thread 0.75mm Dyneema SK99 through small drum pinion and sector wheel helical groove. Clamp cable ends under M3 tensioning bolts.", table_cell),
            Paragraph("Pre-Tension: <b>180 N</b><br/>Bolt torque: <b>1.10 N·m</b>", table_cell),
            Paragraph("Slide motor along ±1.5mm Y-slots until 180N line tension is achieved.", table_cell),
        ],
        [
            Paragraph("<b>ST-05</b>", table_cell),
            Paragraph("<b>Dual-Shear Top Brace Installation</b><br/>Slide 688-2RS bearing over top pinion journal. Seat Top Brace onto Ø10mm axle and pinion. Tighten 2x M3 stand tie-bolts and axle pinch bolt.", table_cell),
            Paragraph("Torque: <b>0.90 N·m</b><br/>Pinch bolt: <b>0.80 N·m</b>", table_cell),
            Paragraph("Check axis-to-axis deflection (must be < 20 µm under full tension).", table_cell),
        ],
        [
            Paragraph("<b>ST-06</b>", table_cell),
            Paragraph("<b>Fuselage Flange Docking & Pin Verification</b><br/>Align mount to fuselage spine bracket at Y=±35.0mm. Insert Ø6.00mm Grade 5 titanium shear pin. Secure with 4x M3x16mm Grade 12.9 screws.", table_cell),
            Paragraph("Torque: <b>1.20 N·m</b><br/>Loctite 243", table_cell),
            Paragraph("Verify zero angular play (< 0.05°) between wing mount and fuselage spine.", table_cell),
        ],
    ]

    t_steps = Table(steps_data, colWidths=[36, 210, 114, 180])
    t_steps.setStyle(TableStyle([
        ('BACKGROUND', (0,0), (-1,0), ICE_BG),
        ('BOX', (0,0), (-1,-1), 0.75, BORDER_GRAY),
        ('INNERGRID', (0,0), (-1,-1), 0.5, BORDER_GRAY),
        ('TOPPADDING', (0,0), (-1,-1), 2.5),
        ('BOTTOMPADDING', (0,0), (-1,-1), 2.5),
        ('VALIGN', (0,0), (-1,-1), 'MIDDLE'),
    ]))
    story.append(t_steps)
    story.append(Spacer(1, 6))

    # Structural Safety Footer Box
    alert_box = [
        [
            Paragraph(
                "<b>STRUCTURAL INTEGRITY & FLIGHT ENVELOPE CERTIFICATION:</b><br/>"
                "• <b>Cable Tension Margin</b>: 220N peak tension produces only 16.7 µm axis deflection (Yield SF = 30.9x in Brace, 161.5x in Stand).<br/>"
                "• <b>Maneuver Lift Margin</b>: 4.0g pullup (3.20 N·m moment) yields SF = 17.7x against interlaminar tensile allowable (38.0 MPa).<br/>"
                "• <b>Fuselage Interface Margin</b>: Ø6.00mm titanium shear pin operates at 7.81 MPa with a <b>70.4x safety factor</b> against shear yield.",
                table_cell
            )
        ]
    ]
    t_alert = Table(alert_box, colWidths=[540])
    t_alert.setStyle(TableStyle([
        ('BACKGROUND', (0,0), (-1,-1), colors.HexColor("#f0fdf4")),
        ('BOX', (0,0), (-1,-1), 1.0, SUCCESS_GRN),
        ('TOPPADDING', (0,0), (-1,-1), 4),
        ('BOTTOMPADDING', (0,0), (-1,-1), 4),
        ('LEFTPADDING', (0,0), (-1,-1), 8),
        ('RIGHTPADDING', (0,0), (-1,-1), 8),
    ]))
    story.append(t_alert)

    doc.build(story, canvasmaker=NumberedCanvas)
    print(f"PDF built successfully: {DOC_OUT}")

if __name__ == "__main__":
    build_pdf()
