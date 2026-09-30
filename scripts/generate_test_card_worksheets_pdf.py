#!/usr/bin/env python3
"""
generate_test_card_worksheets_pdf.py
Generates clean, printable, professional PDF flight test worksheets with fill-in-the-blank
boxes, check-boxes, and signature blocks for longhand field recording on the MCO-1D Mark 1.

Generates:
  1. docs/mark1/worksheets/MCO-1D_WS00_As_Built_Physical_Inspection.pdf
  2. docs/mark1/worksheets/MCO-1D_WS01_TC01_Ground_Bench_Test.pdf
  3. docs/mark1/worksheets/MCO-1D_WS02_TC02_Tethered_Lift_Hover.pdf
  4. docs/mark1/worksheets/MCO-1D_WS03_TC03_Advance_Ratio_Truck_Sweep.pdf
  5. docs/mark1/worksheets/MCO-1D_WS04_TC04_Free_Flight_STOL_Expansion.pdf
  6. docs/mark1/worksheets/MCO-1D_Mark1_All_Worksheets_Binder.pdf
"""

import os
import sys
from reportlab.lib.pagesizes import letter
from reportlab.lib import colors
from reportlab.lib.units import inch
from reportlab.lib.styles import getSampleStyleSheet, ParagraphStyle
from reportlab.platypus import (
    SimpleDocTemplate, Paragraph, Spacer, Table, TableStyle, PageBreak, KeepTogether, HRFlowable
)
from reportlab.pdfgen import canvas

# Output directory
OUTPUT_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "docs", "mark1", "worksheets"))
os.makedirs(OUTPUT_DIR, exist_ok=True)

# Aerospace Color Palette
NAVY_PRIMARY = colors.HexColor("#1A365D")   # Deep navy header
BLUE_ACCENT  = colors.HexColor("#2B6CB0")   # Section accent
ICE_BG       = colors.HexColor("#F7FAFC")   # Light table row
BORDER_GRAY  = colors.HexColor("#CBD5E0")   # Clean subtle border
DARK_TEXT    = colors.HexColor("#2D3748")   # Charcoal body text
TARGET_GRAY  = colors.HexColor("#4A5568")   # Subdued target values
WRITE_BG     = colors.HexColor("#FFFFFF")   # Bright white write-in box

class NumberedCanvas(canvas.Canvas):
    """Adds professional running header and footer with dynamic total page count."""
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
            canvas.Canvas.showPage(self)
        canvas.Canvas.save(self)

    def draw_page_decorations(self, page_count):
        self.saveState()
        self.setFont("Helvetica-Bold", 8)
        self.setFillColor(NAVY_PRIMARY)
        # Running Header
        self.drawString(36, letter[1] - 28, "MCO-1D MARK 1 — FLIGHT TEST & PHYSICAL MEASUREMENT FIELD CARD")
        self.setFont("Helvetica", 8)
        self.setFillColor(TARGET_GRAY)
        self.drawRightString(letter[0] - 36, letter[1] - 28, "CONFIDENTIAL & PROPRIETARY — MCO-1A DEMONSTRATOR")
        self.setStrokeColor(BORDER_GRAY)
        self.setLineWidth(0.6)
        self.line(36, letter[1] - 32, letter[0] - 36, letter[1] - 32)

        # Running Footer
        self.line(36, 32, letter[0] - 36, 32)
        self.setFont("Helvetica", 8)
        self.drawString(36, 22, "Physical Data Collection Form — Fill longhand in ink; photograph/scan upon test completion")
        self.drawRightString(letter[0] - 36, 22, f"Page {self._pageNumber} of {page_count}")
        self.restoreState()


def get_custom_styles():
    styles = getSampleStyleSheet()
    
    styles.add(ParagraphStyle(
        name='CardTitle',
        fontName='Helvetica-Bold',
        fontSize=15,
        leading=18,
        textColor=NAVY_PRIMARY,
        spaceAfter=2,
    ))
    styles.add(ParagraphStyle(
        name='CardSubTitle',
        fontName='Helvetica-Bold',
        fontSize=9,
        leading=12,
        textColor=BLUE_ACCENT,
        spaceAfter=6,
    ))
    styles.add(ParagraphStyle(
        name='SectionHeader',
        fontName='Helvetica-Bold',
        fontSize=10,
        leading=13,
        textColor=NAVY_PRIMARY,
        spaceBefore=5,
        spaceAfter=3,
    ))
    styles.add(ParagraphStyle(
        name='TableCell',
        fontName='Helvetica',
        fontSize=8,
        leading=10,
        textColor=DARK_TEXT,
    ))
    styles.add(ParagraphStyle(
        name='TableCellBold',
        fontName='Helvetica-Bold',
        fontSize=8,
        leading=10,
        textColor=DARK_TEXT,
    ))
    styles.add(ParagraphStyle(
        name='TableTarget',
        fontName='Helvetica-Oblique',
        fontSize=7.5,
        leading=9.5,
        textColor=TARGET_GRAY,
    ))
    styles.add(ParagraphStyle(
        name='TableWriteBlank',
        fontName='Helvetica',
        fontSize=8.5,
        leading=11,
        textColor=colors.black,
        alignment=1, # Center
    ))
    styles.add(ParagraphStyle(
        name='InstructionsText',
        fontName='Helvetica',
        fontSize=7.5,
        leading=9.5,
        textColor=TARGET_GRAY,
        spaceAfter=4,
    ))
    return styles

def build_metadata_block(card_id, card_title, styles):
    data = [
        [
            Paragraph(f"<b>AIRCRAFT:</b> MCO-1D Mark 1", styles['TableCell']),
            Paragraph(f"<b>CARD ID:</b> {card_id}", styles['TableCell']),
            Paragraph("<b>DATE:</b> ____ / ____ / 2026", styles['TableCell']),
            Paragraph("<b>LOCATION:</b> ____________________", styles['TableCell']),
        ],
        [
            Paragraph("<b>OPERATOR / TECH:</b> ________________", styles['TableCell']),
            Paragraph("<b>AIRFRAME S/N:</b> MCO-1D-001", styles['TableCell']),
            Paragraph("<b>AMBIENT TEMP:</b> _____ °C", styles['TableCell']),
            Paragraph("<b>BARO / QNH:</b> _____ hPa", styles['TableCell']),
        ]
    ]
    t = Table(data, colWidths=[1.8*inch, 1.8*inch, 1.8*inch, 1.8*inch])
    t.setStyle(TableStyle([
        ('BACKGROUND', (0,0), (-1,-1), ICE_BG),
        ('BOX', (0,0), (-1,-1), 1, BORDER_GRAY),
        ('INNERGRID', (0,0), (-1,-1), 0.5, BORDER_GRAY),
        ('TOPPADDING', (0,0), (-1,-1), 3),
        ('BOTTOMPADDING', (0,0), (-1,-1), 3),
        ('LEFTPADDING', (0,0), (-1,-1), 5),
        ('RIGHTPADDING', (0,0), (-1,-1), 5),
    ]))
    return t

def build_signoff_block(gate_name, styles):
    data = [
        [
            Paragraph(f"<b>GATE CERTIFICATION:</b> {gate_name}", styles['TableCellBold']),
            Paragraph("<b>STATUS:</b> [  ] PASS   [  ] CONDITIONAL   [  ] FAIL / RE-TEST", styles['TableCellBold']),
        ],
        [
            Paragraph("<b>INSPECTOR / LEAD:</b> ___________________________", styles['TableCell']),
            Paragraph("<b>SIGNATURE & DATE:</b> ___________________________", styles['TableCell']),
        ],
        [
            Paragraph("<b>REMARKS / DISCREPANCIES:</b> ____________________________________________________________________________________", styles['TableCell']),
            Paragraph("", styles['TableCell']),
        ]
    ]
    t = Table(data, colWidths=[3.8*inch, 3.4*inch])
    t.setStyle(TableStyle([
        ('SPAN', (0,2), (1,2)),
        ('BACKGROUND', (0,0), (-1,-1), colors.HexColor("#EDF2F7")),
        ('BOX', (0,0), (-1,-1), 1, NAVY_PRIMARY),
        ('INNERGRID', (0,0), (-1,-1), 0.5, BORDER_GRAY),
        ('TOPPADDING', (0,0), (-1,-1), 3),
        ('BOTTOMPADDING', (0,0), (-1,-1), 3),
        ('LEFTPADDING', (0,0), (-1,-1), 5),
        ('RIGHTPADDING', (0,0), (-1,-1), 5),
    ]))
    return t

# ==============================================================================
# CARD BUILDERS
# ==============================================================================

def generate_ws00(styles):
    """WS-00: As-Built Physical Airframe & Structural Inspection (2 Pages)"""
    story = []
    story.append(Paragraph("WS-00: As-Built Physical Airframe & Structural Worksheet", styles['CardTitle']))
    story.append(Paragraph("STATIC MECHANICAL BASELINE & AS-BUILT DIMENSIONS (COMPLETE BEFORE INITIAL POWER-ON)", styles['CardSubTitle']))
    story.append(build_metadata_block("WS-00", "Physical Inspection", styles))
    story.append(Spacer(1, 4))

    # Section 1: Mass Properties
    story.append(Paragraph("1. Total Airframe Mass Properties & Battery Inventory", styles['SectionHeader']))
    story.append(Paragraph("Weigh all items on a calibrated digital scale (0.1 g resolution). Record actual values.", styles['InstructionsText']))
    
    mass_data = [
        [Paragraph("<b>Item Description</b>", styles['TableCellBold']),
         Paragraph("<b>Location / Component</b>", styles['TableCellBold']),
         Paragraph("<b>Nominal Target</b>", styles['TableCellBold']),
         Paragraph("<b>As-Built Measurement</b>", styles['TableCellBold']),
         Paragraph("<b>Tolerance / Limit</b>", styles['TableCellBold'])],
        
        [Paragraph("<b>Total Assembled Flying Mass</b>", styles['TableCellBold']),
         Paragraph("All 8 wings + 6S & 2S LiPos installed", styles['TableCell']),
         Paragraph("5,687.0 g (55.77 N)", styles['TableTarget']),
         Paragraph("[                      ] g", styles['TableWriteBlank']),
         Paragraph("Max Limit: 5,950 g", styles['TableCell'])],

        [Paragraph("Dry Airframe Mass", styles['TableCell']),
         Paragraph("Both LiPo packs removed", styles['TableCell']),
         Paragraph("4,972.0 g", styles['TableTarget']),
         Paragraph("[                      ] g", styles['TableWriteBlank']),
         Paragraph("Max: 5,100 g", styles['TableCell'])],

        [Paragraph("6S Main Motor LiPo Pack", styles['TableCell']),
         Paragraph("Tattu 6S 4500mAh 60C XT90-S", styles['TableCell']),
         Paragraph("570.0 g", styles['TableTarget']),
         Paragraph("[                      ] g", styles['TableWriteBlank']),
         Paragraph("± 15 g", styles['TableCell'])],

        [Paragraph("2S Receiver/Servo LiPo Pack", styles['TableCell']),
         Paragraph("Gens Ace 2S 3000mAh XT30", styles['TableCell']),
         Paragraph("145.0 g", styles['TableTarget']),
         Paragraph("[                      ] g", styles['TableWriteBlank']),
         Paragraph("± 8 g", styles['TableCell'])],

        [Paragraph("Carbon Fuselage Spine Assembly", styles['TableCell']),
         Paragraph("20mm tube + 4 station brackets", styles['TableCell']),
         Paragraph("710.0 g", styles['TableTarget']),
         Paragraph("[                      ] g", styles['TableWriteBlank']),
         Paragraph("± 25 g", styles['TableCell'])],

        [Paragraph("Flight Control & Avionics Core", styles['TableCell']),
         Paragraph("Pi 4B + Hat + Pixhawk 4 Mini + Harness", styles['TableCell']),
         Paragraph("320.0 g", styles['TableTarget']),
         Paragraph("[                      ] g", styles['TableWriteBlank']),
         Paragraph("± 15 g", styles['TableCell'])],

        [Paragraph("Undercarriage & Wheels", styles['TableCell']),
         Paragraph("Carbon spring struts + Du-Bro wheels", styles['TableCell']),
         Paragraph("185.0 g", styles['TableTarget']),
         Paragraph("[                      ] g", styles['TableWriteBlank']),
         Paragraph("± 10 g", styles['TableCell'])],
    ]
    t_mass = Table(mass_data, colWidths=[2.2*inch, 1.8*inch, 1.1*inch, 1.2*inch, 0.9*inch])
    t_mass.setStyle(TableStyle([
        ('BACKGROUND', (0,0), (-1,0), NAVY_PRIMARY),
        ('TEXTCOLOR', (0,0), (-1,0), colors.white),
        ('ALIGN', (0,0), (-1,-1), 'LEFT'),
        ('VALIGN', (0,0), (-1,-1), 'MIDDLE'),
        ('INNERGRID', (0,0), (-1,-1), 0.5, BORDER_GRAY),
        ('BOX', (0,0), (-1,-1), 1, NAVY_PRIMARY),
        ('TOPPADDING', (0,0), (-1,-1), 2),
        ('BOTTOMPADDING', (0,0), (-1,-1), 2),
        ('BACKGROUND', (3,1), (3,-1), WRITE_BG),
    ]))
    for row in range(1, len(mass_data)):
        if row % 2 == 0:
            t_mass.setStyle(TableStyle([('BACKGROUND', (0, row), (2, row), ICE_BG), ('BACKGROUND', (4, row), (4, row), ICE_BG)]))
    story.append(t_mass)
    story.append(Spacer(1, 4))

    # Section 2: Individual Wing Weights
    story.append(Paragraph("2. Individual Wing Panel Weights (S1223 Carbon / LW-PLA Finished Panels)", styles['SectionHeader']))
    story.append(Paragraph("Bare panels with spar, ribs, covering, tip ballast slug, and root crush ferrule. Must balance port/starboard within 1.5 g.", styles['InstructionsText']))
    
    wing_weight_data = [
        [Paragraph("<b>Wing Station</b>", styles['TableCellBold']),
         Paragraph("<b>Port Panel (g)</b>", styles['TableCellBold']),
         Paragraph("<b>Starboard Panel (g)</b>", styles['TableCellBold']),
         Paragraph("<b>Target Nominal</b>", styles['TableCellBold']),
         Paragraph("<b>Port / Stbd Delta</b>", styles['TableCellBold']),
         Paragraph("<b>Check (≤ 1.5 g)</b>", styles['TableCellBold'])],
        
        [Paragraph("<b>Station 1 ($W_1$ Fore)</b>", styles['TableCell']),
         Paragraph("[            ] g", styles['TableWriteBlank']),
         Paragraph("[            ] g", styles['TableWriteBlank']),
         Paragraph("68.5 g", styles['TableTarget']),
         Paragraph("[        ] g", styles['TableWriteBlank']),
         Paragraph("[  ] PASS  [  ] FAIL", styles['TableCell'])],

        [Paragraph("<b>Station 2 ($W_2$ Mid-Fore)</b>", styles['TableCell']),
         Paragraph("[            ] g", styles['TableWriteBlank']),
         Paragraph("[            ] g", styles['TableWriteBlank']),
         Paragraph("66.8 g", styles['TableTarget']),
         Paragraph("[        ] g", styles['TableWriteBlank']),
         Paragraph("[  ] PASS  [  ] FAIL", styles['TableCell'])],

        [Paragraph("<b>Station 3 ($W_3$ Mid-Aft)</b>", styles['TableCell']),
         Paragraph("[            ] g", styles['TableWriteBlank']),
         Paragraph("[            ] g", styles['TableWriteBlank']),
         Paragraph("65.5 g", styles['TableTarget']),
         Paragraph("[        ] g", styles['TableWriteBlank']),
         Paragraph("[  ] PASS  [  ] FAIL", styles['TableCell'])],

        [Paragraph("<b>Station 4 ($W_4$ Aft)</b>", styles['TableCell']),
         Paragraph("[            ] g", styles['TableWriteBlank']),
         Paragraph("[            ] g", styles['TableWriteBlank']),
         Paragraph("63.2 g", styles['TableTarget']),
         Paragraph("[        ] g", styles['TableWriteBlank']),
         Paragraph("[  ] PASS  [  ] FAIL", styles['TableCell'])],

        [Paragraph("<b>TOTALS (All 8 Wings)</b>", styles['TableCellBold']),
         Paragraph("<b>Sum P:</b> [      ] g", styles['TableCell']),
         Paragraph("<b>Sum S:</b> [      ] g", styles['TableCell']),
         Paragraph("<b>528.0 g Total</b>", styles['TableTarget']),
         Paragraph("<b>Net:</b> [   ] g", styles['TableCellBold']),
         Paragraph("<b>Status:</b> [  ] OK", styles['TableCellBold'])],
    ]
    t_ww = Table(wing_weight_data, colWidths=[1.7*inch, 1.2*inch, 1.2*inch, 1.0*inch, 0.9*inch, 1.2*inch])
    t_ww.setStyle(TableStyle([
        ('BACKGROUND', (0,0), (-1,0), NAVY_PRIMARY),
        ('TEXTCOLOR', (0,0), (-1,0), colors.white),
        ('ALIGN', (0,0), (-1,-1), 'CENTER'),
        ('VALIGN', (0,0), (-1,-1), 'MIDDLE'),
        ('INNERGRID', (0,0), (-1,-1), 0.5, BORDER_GRAY),
        ('BOX', (0,0), (-1,-1), 1, NAVY_PRIMARY),
        ('TOPPADDING', (0,0), (-1,-1), 2.5),
        ('BOTTOMPADDING', (0,0), (-1,-1), 2.5),
        ('BACKGROUND', (0,-1), (-1,-1), colors.HexColor("#E2E8F0")),
    ]))
    story.append(t_ww)

    # Page Break for clean 2-page WS-00
    story.append(PageBreak())

    # Page 2: Geometry, Drivetrain & Rigging
    story.append(Paragraph("WS-00: As-Built Physical Airframe (Continued — Page 2)", styles['CardTitle']))
    story.append(Paragraph("GEOMETRY, LONGITUDINAL STAGGER, DRIVETRAIN & BALANCE", styles['CardSubTitle']))
    story.append(Spacer(1, 4))

    # Section 3: Wing Dimensions
    story.append(Paragraph("3. Wing Semispans & Chord Measurements (Pivot Centerline to Tip)", styles['SectionHeader']))
    geom_data = [
        [Paragraph("<b>Wing Station</b>", styles['TableCellBold']),
         Paragraph("<b>Port Semi-Span (mm)</b>", styles['TableCellBold']),
         Paragraph("<b>Stbd Semi-Span (mm)</b>", styles['TableCellBold']),
         Paragraph("<b>Nominal Span</b>", styles['TableCellBold']),
         Paragraph("<b>Root Chord (mm)</b>", styles['TableCellBold']),
         Paragraph("<b>Tip Chord (mm)</b>", styles['TableCellBold'])],
        
        [Paragraph("Station 1 ($W_1$)", styles['TableCell']),
         Paragraph("[              ] mm", styles['TableWriteBlank']),
         Paragraph("[              ] mm", styles['TableWriteBlank']),
         Paragraph("772.0 mm", styles['TableTarget']),
         Paragraph("[        ] mm (120)", styles['TableWriteBlank']),
         Paragraph("[        ] mm (120)", styles['TableWriteBlank'])],

        [Paragraph("Station 2 ($W_2$)", styles['TableCell']),
         Paragraph("[              ] mm", styles['TableWriteBlank']),
         Paragraph("[              ] mm", styles['TableWriteBlank']),
         Paragraph("749.0 mm", styles['TableTarget']),
         Paragraph("[        ] mm (120)", styles['TableWriteBlank']),
         Paragraph("[        ] mm (120)", styles['TableWriteBlank'])],

        [Paragraph("Station 3 ($W_3$)", styles['TableCell']),
         Paragraph("[              ] mm", styles['TableWriteBlank']),
         Paragraph("[              ] mm", styles['TableWriteBlank']),
         Paragraph("735.0 mm", styles['TableTarget']),
         Paragraph("[        ] mm (120)", styles['TableWriteBlank']),
         Paragraph("[        ] mm (120)", styles['TableWriteBlank'])],

        [Paragraph("Station 4 ($W_4$)", styles['TableCell']),
         Paragraph("[              ] mm", styles['TableWriteBlank']),
         Paragraph("[              ] mm", styles['TableWriteBlank']),
         Paragraph("712.0 mm", styles['TableTarget']),
         Paragraph("[        ] mm (120)", styles['TableWriteBlank']),
         Paragraph("[        ] mm (120)", styles['TableWriteBlank'])],
        
        [Paragraph("<b>Station 1 Total Wingspan</b>", styles['TableCellBold']),
         Paragraph("<b>Tip-to-Tip Span:</b> [                  ] mm", styles['TableCell']),
         Paragraph("", styles['TableCell']),
         Paragraph("1,584.0 mm Target", styles['TableTarget']),
         Paragraph("Fuselage W: [    ] mm", styles['TableCell']),
         Paragraph("40.0 mm Target", styles['TableTarget'])],
    ]
    t_geom = Table(geom_data, colWidths=[1.5*inch, 1.4*inch, 1.4*inch, 0.9*inch, 1.0*inch, 1.0*inch])
    t_geom.setStyle(TableStyle([
        ('SPAN', (1,4), (2,4)),
        ('BACKGROUND', (0,0), (-1,0), NAVY_PRIMARY),
        ('TEXTCOLOR', (0,0), (-1,0), colors.white),
        ('ALIGN', (0,0), (-1,-1), 'CENTER'),
        ('VALIGN', (0,0), (-1,-1), 'MIDDLE'),
        ('INNERGRID', (0,0), (-1,-1), 0.5, BORDER_GRAY),
        ('BOX', (0,0), (-1,-1), 1, NAVY_PRIMARY),
        ('TOPPADDING', (0,0), (-1,-1), 2),
        ('BOTTOMPADDING', (0,0), (-1,-1), 2),
        ('BACKGROUND', (0,-1), (-1,-1), colors.HexColor("#E2E8F0")),
    ]))
    story.append(t_geom)
    story.append(Spacer(1, 4))

    # Section 4: Longitudinal Stagger & Drivetrain
    story.append(Paragraph("4. Cascade Stagger, Drivetrain & Dyneema Rigging Verification", styles['SectionHeader']))
    drive_data = [
        [Paragraph("<b>Inspection Parameter</b>", styles['TableCellBold']),
         Paragraph("<b>Measurement Method / Tool</b>", styles['TableCellBold']),
         Paragraph("<b>Nominal Design Target</b>", styles['TableCellBold']),
         Paragraph("<b>As-Built Measurement</b>", styles['TableCellBold']),
         Paragraph("<b>Safety Criteria</b>", styles['TableCellBold'])],
        
        [Paragraph("Station 1 Pivot Axis ($x_1$)", styles['TableCell']),
         Paragraph("Steel rule from nose tip", styles['TableCell']),
         Paragraph("138.0 mm aft", styles['TableTarget']),
         Paragraph("[                      ] mm", styles['TableWriteBlank']),
         Paragraph("± 1.0 mm", styles['TableCell'])],

        [Paragraph("Inter-Station Stagger ($s$)", styles['TableCell']),
         Paragraph("Center-to-center pivot distance", styles['TableCell']),
         Paragraph("138.0 mm (or 250mm path)", styles['TableTarget']),
         Paragraph("[                      ] mm", styles['TableWriteBlank']),
         Paragraph("± 0.5 mm", styles['TableCell'])],

        [Paragraph("Vertical Staircase Step ($z$)", styles['TableCell']),
         Paragraph("Vertical drop per station down spine", styles['TableCell']),
         Paragraph("41.25 mm drop", styles['TableTarget']),
         Paragraph("[                      ] mm", styles['TableWriteBlank']),
         Paragraph("Matches CAD step", styles['TableCell'])],

        [Paragraph("Inter-Station Convergent Slot Gap", styles['TableCell']),
         Paragraph("Feeler gauge between TE flat & LE tube", styles['TableCell']),
         Paragraph("25.0 mm at neutral sweep", styles['TableTarget']),
         Paragraph("[                      ] mm", styles['TableWriteBlank']),
         Paragraph("23.0 to 27.0 mm", styles['TableCell'])],

        [Paragraph("<b>10:1 Capstan Gear Ratio</b>", styles['TableCellBold']),
         Paragraph("Motor turns per full sector arc", styles['TableCell']),
         Paragraph("10.00 : 1 Ratio", styles['TableTarget']),
         Paragraph("[                      ] : 1", styles['TableWriteBlank']),
         Paragraph("Must equal 10:1", styles['TableCellBold'])],

        [Paragraph("<b>Dyneema SK99 48h Tension</b>", styles['TableCellBold']),
         Paragraph("Inline hanging scale (all 8 capstans)", styles['TableCell']),
         Paragraph("180.0 N ± 5 N settled", styles['TableTarget']),
         Paragraph("[                      ] N", styles['TableWriteBlank']),
         Paragraph("Min 160N / Max 200N", styles['TableCellBold'])],

        [Paragraph("Spar Aluminum Crush Ferrules", styles['TableCell']),
         Paragraph("Visual check: Ø8x40mm ferrule glued", styles['TableCell']),
         Paragraph("Installed all 8 spar roots", styles['TableTarget']),
         Paragraph("[  ] VERIFIED ALL 8", styles['TableWriteBlank']),
         Paragraph("Mandatory (PHY-02)", styles['TableCellBold'])],

        [Paragraph("Motor Mount Thermal Gaskets", styles['TableCell']),
         Paragraph("Visual: 0.5mm FR4/silicone barrier", styles['TableCell']),
         Paragraph("Fitted on all 8 motor mounts", styles['TableTarget']),
         Paragraph("[  ] VERIFIED ALL 8", styles['TableWriteBlank']),
         Paragraph("Mandatory (PHY-04)", styles['TableCellBold'])],
    ]
    t_drive = Table(drive_data, colWidths=[1.8*inch, 1.8*inch, 1.3*inch, 1.3*inch, 1.0*inch])
    t_drive.setStyle(TableStyle([
        ('BACKGROUND', (0,0), (-1,0), NAVY_PRIMARY),
        ('TEXTCOLOR', (0,0), (-1,0), colors.white),
        ('ALIGN', (0,0), (-1,-1), 'LEFT'),
        ('VALIGN', (0,0), (-1,-1), 'MIDDLE'),
        ('INNERGRID', (0,0), (-1,-1), 0.5, BORDER_GRAY),
        ('BOX', (0,0), (-1,-1), 1, NAVY_PRIMARY),
        ('TOPPADDING', (0,0), (-1,-1), 2),
        ('BOTTOMPADDING', (0,0), (-1,-1), 2),
        ('BACKGROUND', (3,1), (3,-1), WRITE_BG),
    ]))
    story.append(t_drive)
    story.append(Spacer(1, 4))

    # Section 5: CG & Balance
    story.append(Paragraph("5. Center of Gravity (CG) Envelope & Battery Keel Sled Position", styles['SectionHeader']))
    cg_data = [
        [Paragraph("<b>Keel Sled Hole Engaged</b>", styles['TableCellBold']),
         Paragraph("<b>Measured CG from Nose Tip</b>", styles['TableCellBold']),
         Paragraph("<b>Calculated % MAC</b>", styles['TableCellBold']),
         Paragraph("<b>Nominal Design Target</b>", styles['TableCellBold']),
         Paragraph("<b>Aft Hole 6 Lockout Roll-Pin</b>", styles['TableCellBold'])],
        
        [Paragraph("Hole # [        ] (Target: 3)", styles['TableWriteBlank']),
         Paragraph("[                  ] mm", styles['TableWriteBlank']),
         Paragraph("[                  ] % MAC", styles['TableWriteBlank']),
         Paragraph("26.5% MAC (311.8 mm)", styles['TableTarget']),
         Paragraph("[  ] INSTALLED (Locked)", styles['TableCellBold'])],
    ]
    t_cg = Table(cg_data, colWidths=[1.6*inch, 1.6*inch, 1.4*inch, 1.3*inch, 1.3*inch])
    t_cg.setStyle(TableStyle([
        ('BACKGROUND', (0,0), (-1,0), NAVY_PRIMARY),
        ('TEXTCOLOR', (0,0), (-1,0), colors.white),
        ('ALIGN', (0,0), (-1,-1), 'CENTER'),
        ('VALIGN', (0,0), (-1,-1), 'MIDDLE'),
        ('INNERGRID', (0,0), (-1,-1), 0.5, BORDER_GRAY),
        ('BOX', (0,0), (-1,-1), 1, NAVY_PRIMARY),
        ('TOPPADDING', (0,0), (-1,-1), 3),
        ('BOTTOMPADDING', (0,0), (-1,-1), 3),
    ]))
    story.append(t_cg)
    story.append(Spacer(1, 6))

    story.append(build_signoff_block("WS-00: AS-BUILT AIRFRAME MECHANICAL CERTIFICATION", styles))
    return story

def generate_ws01(styles):
    """WS-01: TC-01 Ground Bench Test Worksheet (1 Page)"""
    story = []
    story.append(Paragraph("WS-01: Test Card TC-01 — Ground Bench Test Worksheet", styles['CardTitle']))
    story.append(Paragraph("STATIC THRUST, ELECTRICAL POWER & THERMAL EQUILIBRIUM (RIGID LOAD STAND)", styles['CardSubTitle']))
    story.append(build_metadata_block("TC-01", "Ground Bench Test", styles))
    story.append(Spacer(1, 4))

    story.append(Paragraph("1. Flapping Frequency Sweep & Static Performance Matrix", styles['SectionHeader']))
    story.append(Paragraph("Mount airframe securely to rigid load stand. Execute frequency steps sequentially. Record peak readings during final 10s of dwell.", styles['InstructionsText']))

    tc01_data = [
        [Paragraph("<b>Pt</b>", styles['TableCellBold']),
         Paragraph("<b>Cmd Freq</b>", styles['TableCellBold']),
         Paragraph("<b>Stroke Amp</b>", styles['TableCellBold']),
         Paragraph("<b>Dwell</b>", styles['TableCellBold']),
         Paragraph("<b>Target Thrust</b>", styles['TableCellBold']),
         Paragraph("<b>Measured Thrust</b>", styles['TableCellBold']),
         Paragraph("<b>6S Bus Current</b>", styles['TableCellBold']),
         Paragraph("<b>Peak Motor Temp</b>", styles['TableCellBold']),
         Paragraph("<b>ODrive FET Temp</b>", styles['TableCellBold']),
         Paragraph("<b>Cable Slip OK?</b>", styles['TableCellBold'])],
        
        [Paragraph("P1", styles['TableCellBold']),
         Paragraph("1.0 Hz", styles['TableCell']),
         Paragraph("± 60.0°", styles['TableCell']),
         Paragraph("30 s", styles['TableCell']),
         Paragraph("≥ 8.0 N", styles['TableTarget']),
         Paragraph("[         ] N", styles['TableWriteBlank']),
         Paragraph("[         ] A", styles['TableWriteBlank']),
         Paragraph("[         ] °C", styles['TableWriteBlank']),
         Paragraph("[         ] °C", styles['TableWriteBlank']),
         Paragraph("[ ] OK", styles['TableCell'])],

        [Paragraph("P2", styles['TableCellBold']),
         Paragraph("1.5 Hz", styles['TableCell']),
         Paragraph("± 70.0°", styles['TableCell']),
         Paragraph("30 s", styles['TableCell']),
         Paragraph("≥ 16.0 N", styles['TableTarget']),
         Paragraph("[         ] N", styles['TableWriteBlank']),
         Paragraph("[         ] A", styles['TableWriteBlank']),
         Paragraph("[         ] °C", styles['TableWriteBlank']),
         Paragraph("[         ] °C", styles['TableWriteBlank']),
         Paragraph("[ ] OK", styles['TableCell'])],

        [Paragraph("P3", styles['TableCellBold']),
         Paragraph("2.0 Hz", styles['TableCell']),
         Paragraph("± 80.0°", styles['TableCell']),
         Paragraph("30 s", styles['TableCell']),
         Paragraph("≥ 24.0 N", styles['TableTarget']),
         Paragraph("[         ] N", styles['TableWriteBlank']),
         Paragraph("[         ] A", styles['TableWriteBlank']),
         Paragraph("[         ] °C", styles['TableWriteBlank']),
         Paragraph("[         ] °C", styles['TableWriteBlank']),
         Paragraph("[ ] OK", styles['TableCell'])],

        [Paragraph("P4", styles['TableCellBold']),
         Paragraph("2.5 Hz", styles['TableCell']),
         Paragraph("± 80.0°", styles['TableCell']),
         Paragraph("45 s", styles['TableCell']),
         Paragraph("≥ 31.0 N", styles['TableTarget']),
         Paragraph("[         ] N", styles['TableWriteBlank']),
         Paragraph("[         ] A", styles['TableWriteBlank']),
         Paragraph("[         ] °C", styles['TableWriteBlank']),
         Paragraph("[         ] °C", styles['TableWriteBlank']),
         Paragraph("[ ] OK", styles['TableCell'])],

        [Paragraph("P5", styles['TableCellBold']),
         Paragraph("3.0 Hz", styles['TableCell']),
         Paragraph("± 80.0°", styles['TableCell']),
         Paragraph("60 s", styles['TableCell']),
         Paragraph("≥ 37.0 N", styles['TableTarget']),
         Paragraph("[         ] N", styles['TableWriteBlank']),
         Paragraph("[         ] A", styles['TableWriteBlank']),
         Paragraph("[         ] °C", styles['TableWriteBlank']),
         Paragraph("[         ] °C", styles['TableWriteBlank']),
         Paragraph("[ ] OK", styles['TableCell'])],

        [Paragraph("P6", styles['TableCellBold']),
         Paragraph("3.5 Hz", styles['TableCell']),
         Paragraph("± 70.0°", styles['TableCell']),
         Paragraph("20 s", styles['TableCell']),
         Paragraph("≥ 42.0 N", styles['TableTarget']),
         Paragraph("[         ] N", styles['TableWriteBlank']),
         Paragraph("[         ] A", styles['TableWriteBlank']),
         Paragraph("[         ] °C", styles['TableWriteBlank']),
         Paragraph("[         ] °C", styles['TableWriteBlank']),
         Paragraph("[ ] OK", styles['TableCell'])],
    ]
    t_tc01 = Table(tc01_data, colWidths=[0.35*inch, 0.75*inch, 0.75*inch, 0.5*inch, 0.85*inch, 0.9*inch, 0.85*inch, 0.95*inch, 0.95*inch, 0.65*inch])
    t_tc01.setStyle(TableStyle([
        ('BACKGROUND', (0,0), (-1,0), NAVY_PRIMARY),
        ('TEXTCOLOR', (0,0), (-1,0), colors.white),
        ('ALIGN', (0,0), (-1,-1), 'CENTER'),
        ('VALIGN', (0,0), (-1,-1), 'MIDDLE'),
        ('INNERGRID', (0,0), (-1,-1), 0.5, BORDER_GRAY),
        ('BOX', (0,0), (-1,-1), 1, NAVY_PRIMARY),
        ('TOPPADDING', (0,0), (-1,-1), 3),
        ('BOTTOMPADDING', (0,0), (-1,-1), 3),
        ('BACKGROUND', (5,1), (8,-1), WRITE_BG),
    ]))
    story.append(t_tc01)
    story.append(Spacer(1, 6))

    story.append(Paragraph("2. Electrical Subsystem, Servo Rail & Structural Dynamics Checks", styles['SectionHeader']))
    elec_data = [
        [Paragraph("<b>Check / Parameter Description</b>", styles['TableCellBold']),
         Paragraph("<b>Sensor / Measuring Point</b>", styles['TableCellBold']),
         Paragraph("<b>Target Pass Criteria</b>", styles['TableCellBold']),
         Paragraph("<b>Measured Value</b>", styles['TableCellBold']),
         Paragraph("<b>Evaluation</b>", styles['TableCellBold'])],
        
        [Paragraph("7.4V Servo Bus Rail Loaded Voltage", styles['TableCell']),
         Paragraph("Oscilloscope across 7.4V rail at P5", styles['TableCell']),
         Paragraph("≥ 7.10 V (droop ≤ 0.3 V)", styles['TableTarget']),
         Paragraph("[                    ] V", styles['TableWriteBlank']),
         Paragraph("[  ] PASS   [  ] FAIL", styles['TableCell'])],

        [Paragraph("16-Servo Simultaneous Reversal Surge", styles['TableCell']),
         Paragraph("Current clamp on 2S battery lead", styles['TableCell']),
         Paragraph("≤ 45.0 A peak transient", styles['TableTarget']),
         Paragraph("[                    ] A", styles['TableWriteBlank']),
         Paragraph("[  ] PASS   [  ] FAIL", styles['TableCell'])],

        [Paragraph("6S Main Bus Voltage at Full 3.5 Hz Load", styles['TableCell']),
         Paragraph("ODrive vbus telemetry", styles['TableCell']),
         Paragraph("≥ 21.60 V (no severe sag)", styles['TableTarget']),
         Paragraph("[                    ] V", styles['TableWriteBlank']),
         Paragraph("[  ] PASS   [  ] FAIL", styles['TableCell'])],

        [Paragraph("Dyneema Max Angular Slippage", styles['TableCell']),
         Paragraph("Witness mark alignment on sector rim", styles['TableCell']),
         Paragraph("≤ 0.5° (zero visible shift)", styles['TableTarget']),
         Paragraph("[                    ] °", styles['TableWriteBlank']),
         Paragraph("[  ] PASS   [  ] FAIL", styles['TableCell'])],

        [Paragraph("Airframe RMS Accelerometer Vibration", styles['TableCell']),
         Paragraph("Pixhawk 4 Mini IMU log (10–200 Hz)", styles['TableCell']),
         Paragraph("≤ 2.0 g RMS", styles['TableTarget']),
         Paragraph("[                    ] g", styles['TableWriteBlank']),
         Paragraph("[  ] PASS   [  ] FAIL", styles['TableCell'])],
        
        [Paragraph("Wing Spar Deflection at Root under P6", styles['TableCell']),
         Paragraph("Optical deflection scale at station yoke", styles['TableCell']),
         Paragraph("No delamination, crack, or slip", styles['TableTarget']),
         Paragraph("[                    ]", styles['TableWriteBlank']),
         Paragraph("[  ] PASS   [  ] FAIL", styles['TableCell'])],
    ]
    t_elec = Table(elec_data, colWidths=[2.2*inch, 1.8*inch, 1.3*inch, 1.1*inch, 0.8*inch])
    t_elec.setStyle(TableStyle([
        ('BACKGROUND', (0,0), (-1,0), NAVY_PRIMARY),
        ('TEXTCOLOR', (0,0), (-1,0), colors.white),
        ('ALIGN', (0,0), (-1,-1), 'LEFT'),
        ('VALIGN', (0,0), (-1,-1), 'MIDDLE'),
        ('INNERGRID', (0,0), (-1,-1), 0.5, BORDER_GRAY),
        ('BOX', (0,0), (-1,-1), 1, NAVY_PRIMARY),
        ('TOPPADDING', (0,0), (-1,-1), 2.5),
        ('BOTTOMPADDING', (0,0), (-1,-1), 2.5),
        ('BACKGROUND', (3,1), (3,-1), WRITE_BG),
    ]))
    story.append(t_elec)
    story.append(Spacer(1, 8))

    story.append(build_signoff_block("TC-01 GROUND BENCH TEST QUALIFICATION", styles))
    return story

def generate_ws02(styles):
    """WS-02: TC-02 Tethered Lift & Hover Worksheet (1 Page)"""
    story = []
    story.append(Paragraph("WS-02: Test Card TC-02 — Tethered Lift & Hover Worksheet", styles['CardTitle']))
    story.append(Paragraph("VERTICAL GUIDE-WIRE EQUILIBRIUM & ATTITUDE STABILIZATION AUTHORITY", styles['CardSubTitle']))
    story.append(build_metadata_block("TC-02", "Tethered Lift & Hover", styles))
    story.append(Spacer(1, 4))

    story.append(Paragraph("1. Vertical Force Equilibrium Discovery (Weightless Unstick Point)", styles['SectionHeader']))
    story.append(Paragraph("Aircraft suspended on low-friction 4mm vertical steel guide cable. Advance frequency slowly in 0.05 Hz increments until aircraft lifts off ground stand and floats weightlessly (T/W = 1.00).", styles['InstructionsText']))

    tether_eq_data = [
        [Paragraph("<b>Equilibrium Parameter</b>", styles['TableCellBold']),
         Paragraph("<b>Expected Range</b>", styles['TableCellBold']),
         Paragraph("<b>Actual Measured Value</b>", styles['TableCellBold']),
         Paragraph("<b>Engineering Notes / Telemetry Source</b>", styles['TableCellBold'])],
        
        [Paragraph("<b>Weightless Unstick Frequency ($f_{\text{hover}}$)</b>", styles['TableCellBold']),
         Paragraph("1.85 to 2.10 Hz", styles['TableTarget']),
         Paragraph("[                        ] Hz", styles['TableWriteBlank']),
         Paragraph("Frequency where load cell reaches 0.0 N tare", styles['TableCell'])],

        [Paragraph("Commanded Flapping Amplitude", styles['TableCell']),
         Paragraph("± 75.0° (150° total stroke)", styles['TableTarget']),
         Paragraph("[                        ] °", styles['TableWriteBlank']),
         Paragraph("Stroke angle setting on ODrive controllers", styles['TableCell'])],

        [Paragraph("Total 6S Bus Current at Steady Hover", styles['TableCell']),
         Paragraph("18.0 to 24.0 A", styles['TableTarget']),
         Paragraph("[                        ] A", styles['TableWriteBlank']),
         Paragraph("Summed current across all 8 ODrive inverters", styles['TableCell'])],

        [Paragraph("Total Electrical Power at Steady Hover", styles['TableCell']),
         Paragraph("400 to 530 W", styles['TableTarget']),
         Paragraph("[                        ] W", styles['TableWriteBlank']),
         Paragraph("$P_{\text{elec}} = V_{\text{bus}} \times I_{\text{bus}}$ (Target: ~475 W)", styles['TableCell'])],

        [Paragraph("Guide Wire Tare Collar Friction", styles['TableCell']),
         Paragraph("< 2.0 N friction", styles['TableTarget']),
         Paragraph("[                        ] N", styles['TableWriteBlank']),
         Paragraph("Measured by lifting aircraft manually with fish scale", styles['TableCell'])],

        [Paragraph("LiDAR Height at Steady Hover Hold", styles['TableCell']),
         Paragraph("1.0 to 1.5 m AGL", styles['TableTarget']),
         Paragraph("[                        ] m", styles['TableWriteBlank']),
         Paragraph("TF-Luna ground altitude reading", styles['TableCell'])],
    ]
    t_eq = Table(tether_eq_data, colWidths=[2.3*inch, 1.3*inch, 1.4*inch, 2.2*inch])
    t_eq.setStyle(TableStyle([
        ('BACKGROUND', (0,0), (-1,0), NAVY_PRIMARY),
        ('TEXTCOLOR', (0,0), (-1,0), colors.white),
        ('ALIGN', (0,0), (-1,-1), 'LEFT'),
        ('VALIGN', (0,0), (-1,-1), 'MIDDLE'),
        ('INNERGRID', (0,0), (-1,-1), 0.5, BORDER_GRAY),
        ('BOX', (0,0), (-1,-1), 1, NAVY_PRIMARY),
        ('TOPPADDING', (0,0), (-1,-1), 3),
        ('BOTTOMPADDING', (0,0), (-1,-1), 3),
        ('BACKGROUND', (2,1), (2,-1), WRITE_BG),
    ]))
    story.append(t_eq)
    story.append(Spacer(1, 6))

    story.append(Paragraph("2. Attitude Stabilization & Control Authority (Step Stick Injections)", styles['SectionHeader']))
    story.append(Paragraph("While suspended in steady hover trim, inject ±5° stick pulses from RC transmitter / Ground Station. Verify angular response rates from Pixhawk gyro telemetry.", styles['InstructionsText']))

    step_data = [
        [Paragraph("<b>Control Channel</b>", styles['TableCellBold']),
         Paragraph("<b>Commanded Injection</b>", styles['TableCellBold']),
         Paragraph("<b>Min Required Rate</b>", styles['TableCellBold']),
         Paragraph("<b>Measured Rate</b>", styles['TableCellBold']),
         Paragraph("<b>Response Character / Damping</b>", styles['TableCellBold'])],
        
        [Paragraph("<b>Pitch Up</b> (AoA Fore/Aft Diff)", styles['TableCell']),
         Paragraph("+ 5.0° Elevator Stick", styles['TableCell']),
         Paragraph("≥ 15.0 °/s", styles['TableTarget']),
         Paragraph("[                  ] °/s", styles['TableWriteBlank']),
         Paragraph("[  ] Deadbeat   [  ] Light Osc   [  ] Sluggish", styles['TableCell'])],

        [Paragraph("<b>Pitch Down</b> (AoA Fore/Aft Diff)", styles['TableCell']),
         Paragraph("- 5.0° Elevator Stick", styles['TableCell']),
         Paragraph("≥ 15.0 °/s", styles['TableTarget']),
         Paragraph("[                  ] °/s", styles['TableWriteBlank']),
         Paragraph("[  ] Deadbeat   [  ] Light Osc   [  ] Sluggish", styles['TableCell'])],

        [Paragraph("<b>Roll Right</b> (Port/Stbd Amp Diff)", styles['TableCell']),
         Paragraph("+ 5.0° Aileron Stick", styles['TableCell']),
         Paragraph("≥ 20.0 °/s", styles['TableTarget']),
         Paragraph("[                  ] °/s", styles['TableWriteBlank']),
         Paragraph("[  ] Deadbeat   [  ] Light Osc   [  ] Sluggish", styles['TableCell'])],

        [Paragraph("<b>Roll Left</b> (Port/Stbd Amp Diff)", styles['TableCell']),
         Paragraph("- 5.0° Aileron Stick", styles['TableCell']),
         Paragraph("≥ 20.0 °/s", styles['TableTarget']),
         Paragraph("[                  ] °/s", styles['TableWriteBlank']),
         Paragraph("[  ] Deadbeat   [  ] Light Osc   [  ] Sluggish", styles['TableCell'])],

        [Paragraph("<b>Yaw Right</b> (Fore/Aft Sweep Diff)", styles['TableCell']),
         Paragraph("+ 10.0° Rudder Stick", styles['TableCell']),
         Paragraph("≥ 12.0 °/s", styles['TableTarget']),
         Paragraph("[                  ] °/s", styles['TableWriteBlank']),
         Paragraph("[  ] Deadbeat   [  ] Light Osc   [  ] Sluggish", styles['TableCell'])],

        [Paragraph("<b>Yaw Left</b> (Fore/Aft Sweep Diff)", styles['TableCell']),
         Paragraph("- 10.0° Rudder Stick", styles['TableCell']),
         Paragraph("≥ 12.0 °/s", styles['TableTarget']),
         Paragraph("[                  ] °/s", styles['TableWriteBlank']),
         Paragraph("[  ] Deadbeat   [  ] Light Osc   [  ] Sluggish", styles['TableCell'])],
    ]
    t_step = Table(step_data, colWidths=[1.8*inch, 1.4*inch, 1.1*inch, 1.2*inch, 1.7*inch])
    t_step.setStyle(TableStyle([
        ('BACKGROUND', (0,0), (-1,0), NAVY_PRIMARY),
        ('TEXTCOLOR', (0,0), (-1,0), colors.white),
        ('ALIGN', (0,0), (-1,-1), 'LEFT'),
        ('VALIGN', (0,0), (-1,-1), 'MIDDLE'),
        ('INNERGRID', (0,0), (-1,-1), 0.5, BORDER_GRAY),
        ('BOX', (0,0), (-1,-1), 1, NAVY_PRIMARY),
        ('TOPPADDING', (0,0), (-1,-1), 3),
        ('BOTTOMPADDING', (0,0), (-1,-1), 3),
        ('BACKGROUND', (3,1), (3,-1), WRITE_BG),
    ]))
    story.append(t_step)
    story.append(Spacer(1, 8))

    story.append(build_signoff_block("TC-02 TETHERED LIFT & HOVER QUALIFICATION", styles))
    return story

def generate_ws03(styles):
    """WS-03: TC-03 Moving Vehicle Advance-Ratio Sweep (1 Page)"""
    story = []
    story.append(Paragraph("WS-03: Test Card TC-03 — Moving Vehicle Advance-Ratio Sweep", styles['CardTitle']))
    story.append(Paragraph("AERODYNAMIC POLAR MAPPING, FORWARD THRUST & LIFT (TRUCK TEST PYLON)", styles['CardSubTitle']))
    story.append(build_metadata_block("TC-03", "Truck Pylon Sweep", styles))
    story.append(Spacer(1, 4))

    story.append(Paragraph("1. Aerodynamic Force & Polar Sweep vs Forward Airspeed", styles['SectionHeader']))
    story.append(Paragraph("Mount aircraft to 6-axis load cell pylon above open-bed truck. Drive runway in steady speed steps in calm air (< 1.0 m/s wind).", styles['InstructionsText']))

    truck_data = [
        [Paragraph("<b>Run</b>", styles['TableCellBold']),
         Paragraph("<b>Truck Speed (V_inf)</b>", styles['TableCellBold']),
         Paragraph("<b>Flap Freq</b>", styles['TableCellBold']),
         Paragraph("<b>Thrust ($F_x$) Target</b>", styles['TableCellBold']),
         Paragraph("<b>Measured $F_x$</b>", styles['TableCellBold']),
         Paragraph("<b>Target Lift ($L$)</b>", styles['TableCellBold']),
         Paragraph("<b>Measured Lift</b>", styles['TableCellBold']),
         Paragraph("<b>Pitch Moment ($M_y$)</b>", styles['TableCellBold']),
         Paragraph("<b>Pitot Speed</b>", styles['TableCellBold'])],
        
        [Paragraph("R1", styles['TableCellBold']),
         Paragraph("4.0 m/s (9 mph)", styles['TableCell']),
         Paragraph("2.20 Hz", styles['TableCell']),
         Paragraph("≥ 12.0 N", styles['TableTarget']),
         Paragraph("[        ] N", styles['TableWriteBlank']),
         Paragraph("≥ 20.0 N", styles['TableTarget']),
         Paragraph("[        ] N", styles['TableWriteBlank']),
         Paragraph("[        ] N·m", styles['TableWriteBlank']),
         Paragraph("[      ] m/s", styles['TableWriteBlank'])],

        [Paragraph("R2", styles['TableCellBold']),
         Paragraph("6.2 m/s (14 mph)", styles['TableCell']),
         Paragraph("2.30 Hz", styles['TableCell']),
         Paragraph("≥ 15.0 N", styles['TableTarget']),
         Paragraph("[        ] N", styles['TableWriteBlank']),
         Paragraph("≥ 56.0 N (Unstick)", styles['TableTarget']),
         Paragraph("[        ] N", styles['TableWriteBlank']),
         Paragraph("[        ] N·m", styles['TableWriteBlank']),
         Paragraph("[      ] m/s", styles['TableWriteBlank'])],

        [Paragraph("R3", styles['TableCellBold']),
         Paragraph("9.0 m/s (20 mph)", styles['TableCell']),
         Paragraph("2.40 Hz", styles['TableCell']),
         Paragraph("≥ 8.0 N", styles['TableTarget']),
         Paragraph("[        ] N", styles['TableWriteBlank']),
         Paragraph("≥ 56.0 N (Cruise)", styles['TableTarget']),
         Paragraph("[        ] N", styles['TableWriteBlank']),
         Paragraph("[        ] N·m", styles['TableWriteBlank']),
         Paragraph("[      ] m/s", styles['TableWriteBlank'])],

        [Paragraph("R4", styles['TableCellBold']),
         Paragraph("12.0 m/s (27 mph)", styles['TableCell']),
         Paragraph("2.60 Hz", styles['TableCell']),
         Paragraph("≥ 5.0 N", styles['TableTarget']),
         Paragraph("[        ] N", styles['TableWriteBlank']),
         Paragraph("≥ 58.0 N", styles['TableTarget']),
         Paragraph("[        ] N", styles['TableWriteBlank']),
         Paragraph("[        ] N·m", styles['TableWriteBlank']),
         Paragraph("[      ] m/s", styles['TableWriteBlank'])],

        [Paragraph("R5", styles['TableCellBold']),
         Paragraph("14.0 m/s (31 mph)", styles['TableCell']),
         Paragraph("2.80 Hz", styles['TableCell']),
         Paragraph("≥ 2.0 N (Sprint)", styles['TableTarget']),
         Paragraph("[        ] N", styles['TableWriteBlank']),
         Paragraph("≥ 62.0 N", styles['TableTarget']),
         Paragraph("[        ] N", styles['TableWriteBlank']),
         Paragraph("[        ] N·m", styles['TableWriteBlank']),
         Paragraph("[      ] m/s", styles['TableWriteBlank'])],
    ]
    t_truck = Table(truck_data, colWidths=[0.35*inch, 1.15*inch, 0.7*inch, 0.85*inch, 0.75*inch, 0.95*inch, 0.8*inch, 0.85*inch, 0.8*inch])
    t_truck.setStyle(TableStyle([
        ('BACKGROUND', (0,0), (-1,0), NAVY_PRIMARY),
        ('TEXTCOLOR', (0,0), (-1,0), colors.white),
        ('ALIGN', (0,0), (-1,-1), 'CENTER'),
        ('VALIGN', (0,0), (-1,-1), 'MIDDLE'),
        ('INNERGRID', (0,0), (-1,-1), 0.5, BORDER_GRAY),
        ('BOX', (0,0), (-1,-1), 1, NAVY_PRIMARY),
        ('TOPPADDING', (0,0), (-1,-1), 3),
        ('BOTTOMPADDING', (0,0), (-1,-1), 3),
        ('BACKGROUND', (4,1), (4,-1), WRITE_BG),
        ('BACKGROUND', (6,1), (8,-1), WRITE_BG),
    ]))
    story.append(t_truck)
    story.append(Spacer(1, 6))

    story.append(Paragraph("2. Pure Passive Glide Aerodynamics Verification (Wings Stationary: f = 0.0 Hz)", styles['SectionHeader']))
    story.append(Paragraph("Hold wings completely stationary at clean cruise pitch (α = +4.0°). Measures baseline fixed-wing lift deficit vs airspeed.", styles['InstructionsText']))

    glide_data = [
        [Paragraph("<b>Truck Speed (V_inf)</b>", styles['TableCellBold']),
         Paragraph("<b>Flapping State</b>", styles['TableCellBold']),
         Paragraph("<b>Expected Passive Lift</b>", styles['TableCellBold']),
         Paragraph("<b>Measured Passive Lift</b>", styles['TableCellBold']),
         Paragraph("<b>Measured Drag ($D$)</b>", styles['TableCellBold']),
         Paragraph("<b>Measured $L/D$ Ratio</b>", styles['TableCellBold']),
         Paragraph("<b>Flight Safety Note</b>", styles['TableCellBold'])],
        
        [Paragraph("9.0 m/s (20.1 mph)", styles['TableCell']),
         Paragraph("<b>0.0 Hz (Fixed)</b>", styles['TableCell']),
         Paragraph("37.13 N (18.6 N deficit)", styles['TableTarget']),
         Paragraph("[                ] N", styles['TableWriteBlank']),
         Paragraph("[                ] N", styles['TableWriteBlank']),
         Paragraph("[              ]", styles['TableWriteBlank']),
         Paragraph("Cannot sustain 1g level flight", styles['TableCell'])],

        [Paragraph("11.0 m/s (24.6 mph)", styles['TableCell']),
         Paragraph("<b>0.0 Hz (Fixed)</b>", styles['TableCell']),
         Paragraph("55.77 N (1.00g balance)", styles['TableTarget']),
         Paragraph("[                ] N", styles['TableWriteBlank']),
         Paragraph("[                ] N", styles['TableWriteBlank']),
         Paragraph("[              ]", styles['TableWriteBlank']),
         Paragraph("Safe unpowered dead-stick glide", styles['TableCellBold'])],

        [Paragraph("13.0 m/s (29.1 mph)", styles['TableCell']),
         Paragraph("<b>0.0 Hz (Fixed)</b>", styles['TableCell']),
         Paragraph("77.80 N (excess lift)", styles['TableTarget']),
         Paragraph("[                ] N", styles['TableWriteBlank']),
         Paragraph("[                ] N", styles['TableWriteBlank']),
         Paragraph("[              ]", styles['TableWriteBlank']),
         Paragraph("Positive climb glide flare", styles['TableCell'])],
    ]
    t_glide = Table(glide_data, colWidths=[1.3*inch, 1.0*inch, 1.2*inch, 1.0*inch, 0.9*inch, 0.8*inch, 1.0*inch])
    t_glide.setStyle(TableStyle([
        ('BACKGROUND', (0,0), (-1,0), NAVY_PRIMARY),
        ('TEXTCOLOR', (0,0), (-1,0), colors.white),
        ('ALIGN', (0,0), (-1,-1), 'CENTER'),
        ('VALIGN', (0,0), (-1,-1), 'MIDDLE'),
        ('INNERGRID', (0,0), (-1,-1), 0.5, BORDER_GRAY),
        ('BOX', (0,0), (-1,-1), 1, NAVY_PRIMARY),
        ('TOPPADDING', (0,0), (-1,-1), 3),
        ('BOTTOMPADDING', (0,0), (-1,-1), 3),
        ('BACKGROUND', (3,1), (5,-1), WRITE_BG),
    ]))
    story.append(t_glide)
    story.append(Spacer(1, 6))

    story.append(Paragraph("3. Pitot Differential Pressure Sensor vs True Truck Ground Speed", styles['SectionHeader']))
    pitot_data = [
        [Paragraph("<b>Truck GPS Speed:</b> 6.2 m/s", styles['TableCell']),
         Paragraph("<b>Target Counts:</b> ~56 counts (23.5 Pa)", styles['TableCell']),
         Paragraph("<b>MS4525DO Counts:</b> [          ]", styles['TableWriteBlank']),
         Paragraph("<b>Indicated Airspeed:</b> [        ] m/s", styles['TableWriteBlank'])],
        [Paragraph("<b>Truck GPS Speed:</b> 9.0 m/s", styles['TableCell']),
         Paragraph("<b>Target Counts:</b> ~118 counts (49.6 Pa)", styles['TableCell']),
         Paragraph("<b>MS4525DO Counts:</b> [          ]", styles['TableWriteBlank']),
         Paragraph("<b>Indicated Airspeed:</b> [        ] m/s", styles['TableWriteBlank'])],
    ]
    t_pit = Table(pitot_data, colWidths=[1.8*inch, 2.0*inch, 1.7*inch, 1.7*inch])
    t_pit.setStyle(TableStyle([
        ('BACKGROUND', (0,0), (-1,-1), ICE_BG),
        ('BOX', (0,0), (-1,-1), 1, BORDER_GRAY),
        ('INNERGRID', (0,0), (-1,-1), 0.5, BORDER_GRAY),
        ('TOPPADDING', (0,0), (-1,-1), 2.5),
        ('BOTTOMPADDING', (0,0), (-1,-1), 2.5),
    ]))
    story.append(t_pit)
    story.append(Spacer(1, 8))

    story.append(build_signoff_block("TC-03 ADVANCE-RATIO TRUCK SWEEP QUALIFICATION", styles))
    return story

def generate_ws04(styles):
    """WS-04: TC-04 Free Flight STOL Runway Expansion (2 Pages)"""
    story = []
    story.append(Paragraph("WS-04: Test Card TC-04 — Free Flight STOL Sortie Worksheet", styles['CardTitle']))
    story.append(Paragraph("FIRST FREE-FLIGHT SORTIE: TAXI, UNSTICK, CLIMB, CRUISE, FLARE & ROLLOUT", styles['CardSubTitle']))
    story.append(build_metadata_block("TC-04", "Free Flight STOL", styles))
    story.append(Spacer(1, 4))

    # Pre-Flight Status
    story.append(Paragraph("1. Pre-Flight Electrical, Sensor & Environmental Go/No-Go Gate", styles['SectionHeader']))
    pre_data = [
        [Paragraph("<b>System / Parameter</b>", styles['TableCellBold']),
         Paragraph("<b>Inspection Requirement</b>", styles['TableCellBold']),
         Paragraph("<b>Go/No-Go Safety Limit</b>", styles['TableCellBold']),
         Paragraph("<b>Pre-Flight Measurement</b>", styles['TableCellBold']),
         Paragraph("<b>Status</b>", styles['TableCellBold'])],
        
        [Paragraph("6S Main Motor LiPo Voltage", styles['TableCellBold']),
         Paragraph("DMM resting check at battery lead", styles['TableCell']),
         Paragraph("≥ 25.0 V (≥ 4.17 V/cell)", styles['TableTarget']),
         Paragraph("[                    ] V", styles['TableWriteBlank']),
         Paragraph("[  ] GO   [  ] NO-GO", styles['TableCell'])],

        [Paragraph("2S Receiver/Servo LiPo Voltage", styles['TableCellBold']),
         Paragraph("DMM check at servo bus tap", styles['TableCell']),
         Paragraph("≥ 8.30 V (≥ 4.15 V/cell)", styles['TableTarget']),
         Paragraph("[                    ] V", styles['TableWriteBlank']),
         Paragraph("[  ] GO   [  ] NO-GO", styles['TableCell'])],

        [Paragraph("Surface Wind Speed", styles['TableCell']),
         Paragraph("Handheld anemometer on runway", styles['TableCell']),
         Paragraph("≤ 3.0 m/s aligned with runway", styles['TableTarget']),
         Paragraph("[                    ] m/s", styles['TableWriteBlank']),
         Paragraph("[  ] GO   [  ] NO-GO", styles['TableCell'])],

        [Paragraph("Crosswind Component", styles['TableCell']),
         Paragraph("Crosswind perpendicular to centerline", styles['TableCell']),
         Paragraph("≤ 1.0 m/s max crosswind", styles['TableTarget']),
         Paragraph("[                    ] m/s", styles['TableWriteBlank']),
         Paragraph("[  ] GO   [  ] NO-GO", styles['TableCell'])],

        [Paragraph("Battery Sled Position Lock", styles['TableCell']),
         Paragraph("Verify quick-pin in Hole 3 (250 mm)", styles['TableCell']),
         Paragraph("Hole 3 locked, Hole 6 pin in place", styles['TableTarget']),
         Paragraph("[  ] VERIFIED HOLE 3", styles['TableWriteBlank']),
         Paragraph("[  ] GO   [  ] NO-GO", styles['TableCell'])],

        [Paragraph("Pitot Cover Removed & Zeroed", styles['TableCell']),
         Paragraph("Pixhawk telemetry indicated airspeed", styles['TableCell']),
         Paragraph("0.0 m/s ± 0.2 m/s calm", styles['TableTarget']),
         Paragraph("[                    ] m/s", styles['TableWriteBlank']),
         Paragraph("[  ] GO   [  ] NO-GO", styles['TableCell'])],
    ]
    t_pre = Table(pre_data, colWidths=[2.1*inch, 1.8*inch, 1.4*inch, 1.1*inch, 0.8*inch])
    t_pre.setStyle(TableStyle([
        ('BACKGROUND', (0,0), (-1,0), NAVY_PRIMARY),
        ('TEXTCOLOR', (0,0), (-1,0), colors.white),
        ('ALIGN', (0,0), (-1,-1), 'LEFT'),
        ('VALIGN', (0,0), (-1,-1), 'MIDDLE'),
        ('INNERGRID', (0,0), (-1,-1), 0.5, BORDER_GRAY),
        ('BOX', (0,0), (-1,-1), 1, NAVY_PRIMARY),
        ('TOPPADDING', (0,0), (-1,-1), 2.5),
        ('BOTTOMPADDING', (0,0), (-1,-1), 2.5),
        ('BACKGROUND', (3,1), (3,-1), WRITE_BG),
    ]))
    story.append(t_pre)
    story.append(Spacer(1, 4))

    # Phase 1: Takeoff & Climb
    story.append(Paragraph("2. Takeoff Acceleration & Climb Performance (Phase 1)", styles['SectionHeader']))
    to_data = [
        [Paragraph("<b>Flight Parameter</b>", styles['TableCellBold']),
         Paragraph("<b>Telemetry Channel / Source</b>", styles['TableCellBold']),
         Paragraph("<b>Design Predicted Target</b>", styles['TableCellBold']),
         Paragraph("<b>Actual Flight Recorded</b>", styles['TableCellBold']),
         Paragraph("<b>Tolerance / Target</b>", styles['TableCellBold'])],
        
        [Paragraph("Takeoff Ground Roll Distance", styles['TableCellBold']),
         Paragraph("Runway visual markers / GPS", styles['TableCell']),
         Paragraph("≤ 12.0 m ground roll", styles['TableTarget']),
         Paragraph("[                        ] m", styles['TableWriteBlank']),
         Paragraph("STOL target: 9–11 m", styles['TableCell'])],

        [Paragraph("Unstick Indicated Airspeed ($V_{\text{rot}}$)", styles['TableCellBold']),
         Paragraph("Pitot airspeed at wheels-off", styles['TableCell']),
         Paragraph("6.20 m/s (13.9 mph)", styles['TableTarget']),
         Paragraph("[                        ] m/s", styles['TableWriteBlank']),
         Paragraph("± 0.4 m/s", styles['TableCell'])],

        [Paragraph("Elapsed Time to Unstick", styles['TableCell']),
         Paragraph("Time from advance lever to liftoff", styles['TableCell']),
         Paragraph("3.5 to 4.5 seconds", styles['TableTarget']),
         Paragraph("[                        ] s", styles['TableWriteBlank']),
         Paragraph("< 5.0 s limit", styles['TableCell'])],

        [Paragraph("Steady Rate of Climb ($V_z$)", styles['TableCellBold']),
         Paragraph("Pixhawk barometer / EKF3 climb rate", styles['TableCell']),
         Paragraph("1.80 to 2.20 m/s at 9 m/s", styles['TableTarget']),
         Paragraph("[                        ] m/s", styles['TableWriteBlank']),
         Paragraph("Boyd $P_s$ envelope", styles['TableCell'])],

        [Paragraph("Flight Path Climb Angle (gamma)", styles['TableCell']),
         Paragraph("gamma = arcsin(V_z / V_inf)", styles['TableCell']),
         Paragraph("11.5° to 15.0° climb", styles['TableTarget']),
         Paragraph("[                        ] °", styles['TableWriteBlank']),
         Paragraph("Clearance obstacle", styles['TableCell'])],

        [Paragraph("Peak Electrical Takeoff Power", styles['TableCell']),
         Paragraph("6S bus current × voltage log", styles['TableCell']),
         Paragraph("≤ 680 W total peak", styles['TableTarget']),
         Paragraph("[                        ] W", styles['TableWriteBlank']),
         Paragraph("< 32 A total", styles['TableCell'])],
    ]
    t_to = Table(to_data, colWidths=[2.2*inch, 1.8*inch, 1.3*inch, 1.1*inch, 0.8*inch])
    t_to.setStyle(TableStyle([
        ('BACKGROUND', (0,0), (-1,0), NAVY_PRIMARY),
        ('TEXTCOLOR', (0,0), (-1,0), colors.white),
        ('ALIGN', (0,0), (-1,-1), 'LEFT'),
        ('VALIGN', (0,0), (-1,-1), 'MIDDLE'),
        ('INNERGRID', (0,0), (-1,-1), 0.5, BORDER_GRAY),
        ('BOX', (0,0), (-1,-1), 1, NAVY_PRIMARY),
        ('TOPPADDING', (0,0), (-1,-1), 2.5),
        ('BOTTOMPADDING', (0,0), (-1,-1), 2.5),
        ('BACKGROUND', (3,1), (3,-1), WRITE_BG),
    ]))
    story.append(t_to)

    story.append(PageBreak())

    # Page 2 of WS-04: Cruise, Landing, Debrief
    story.append(Paragraph("WS-04: Free Flight STOL Sortie (Continued — Page 2)", styles['CardTitle']))
    story.append(Paragraph("CRUISE HANDLING, APPROACH, FLARE, TOUCHDOWN & POST-FLIGHT DEBRIEF", styles['CardSubTitle']))
    story.append(Spacer(1, 4))

    # Phase 2: Cruise
    story.append(Paragraph("3. Level Cruise Flight & Handling Qualities (Phase 2)", styles['SectionHeader']))
    cr_data = [
        [Paragraph("<b>Cruise Metric</b>", styles['TableCellBold']),
         Paragraph("<b>Telemetry Source / Parameter</b>", styles['TableCellBold']),
         Paragraph("<b>Nominal Design Target</b>", styles['TableCellBold']),
         Paragraph("<b>Actual Flight Recorded</b>", styles['TableCellBold']),
         Paragraph("<b>Performance Window</b>", styles['TableCellBold'])],
        
        [Paragraph("Level Cruise Airspeed (V_inf)", styles['TableCellBold']),
         Paragraph("Pitot calibrated airspeed (CAS)", styles['TableCell']),
         Paragraph("9.00 m/s (20.1 mph)", styles['TableTarget']),
         Paragraph("[                        ] m/s", styles['TableWriteBlank']),
         Paragraph("8.5 to 9.5 m/s", styles['TableCell'])],

        [Paragraph("Active Cruise Flapping Frequency", styles['TableCellBold']),
         Paragraph("ODrive commanded frequency", styles['TableCell']),
         Paragraph("2.20 to 2.50 Hz", styles['TableTarget']),
         Paragraph("[                        ] Hz", styles['TableWriteBlank']),
         Paragraph("Floor: ≥ 1.87 Hz", styles['TableCellBold'])],

        [Paragraph("Cruise Throttle Lever Position", styles['TableCell']),
         Paragraph("RC transmitter throttle channel", styles['TableCell']),
         Paragraph("48% to 55% detent", styles['TableTarget']),
         Paragraph("[                        ] %", styles['TableWriteBlank']),
         Paragraph("Cruise detent", styles['TableCell'])],

        [Paragraph("Mean Electrical Power Consumption", styles['TableCellBold']),
         Paragraph("Average bus power in steady level", styles['TableCell']),
         Paragraph("240 to 280 W", styles['TableTarget']),
         Paragraph("[                        ] W", styles['TableWriteBlank']),
         Paragraph("< 300 W limit", styles['TableCell'])],

        [Paragraph("Specific Energy (Cost of Transport)", styles['TableCellBold']),
         Paragraph("Wh energy consumed per km", styles['TableCell']),
         Paragraph("7.4 to 8.6 Wh/km", styles['TableTarget']),
         Paragraph("[                        ] Wh/km", styles['TableWriteBlank']),
         Paragraph("< 9.5 Wh/km", styles['TableCell'])],

        [Paragraph("Pitch Trim Stick Bias", styles['TableCell']),
         Paragraph("Elevator trim offset for level flight", styles['TableCell']),
         Paragraph("0.0% ± 5.0%", styles['TableTarget']),
         Paragraph("[                        ] %", styles['TableWriteBlank']),
         Paragraph("Validates 26.5% CG", styles['TableCell'])],

        [Paragraph("Roll Trim Stick Bias", styles['TableCell']),
         Paragraph("Aileron trim offset for straight track", styles['TableCell']),
         Paragraph("0.0% ± 3.0%", styles['TableTarget']),
         Paragraph("[                        ] %", styles['TableWriteBlank']),
         Paragraph("Validates wing balance", styles['TableCell'])],
    ]
    t_cr = Table(cr_data, colWidths=[2.2*inch, 1.8*inch, 1.3*inch, 1.1*inch, 0.8*inch])
    t_cr.setStyle(TableStyle([
        ('BACKGROUND', (0,0), (-1,0), NAVY_PRIMARY),
        ('TEXTCOLOR', (0,0), (-1,0), colors.white),
        ('ALIGN', (0,0), (-1,-1), 'LEFT'),
        ('VALIGN', (0,0), (-1,-1), 'MIDDLE'),
        ('INNERGRID', (0,0), (-1,-1), 0.5, BORDER_GRAY),
        ('BOX', (0,0), (-1,-1), 1, NAVY_PRIMARY),
        ('TOPPADDING', (0,0), (-1,-1), 2),
        ('BOTTOMPADDING', (0,0), (-1,-1), 2),
        ('BACKGROUND', (3,1), (3,-1), WRITE_BG),
    ]))
    story.append(t_cr)
    story.append(Spacer(1, 4))

    # Phase 3: Approach, Flare & Landing
    story.append(Paragraph("4. Approach, Flare, Touchdown & Rollout (Phase 3)", styles['SectionHeader']))
    app_data = [
        [Paragraph("<b>Landing Metric</b>", styles['TableCellBold']),
         Paragraph("<b>Target Expected</b>", styles['TableCellBold']),
         Paragraph("<b>Actual Flight Recorded</b>", styles['TableCellBold']),
         Paragraph("<b>Safety Evaluation & Notes</b>", styles['TableCellBold'])],
        
        [Paragraph("Final Approach Indicated Airspeed", styles['TableCell']),
         Paragraph("7.50 m/s ± 0.3 m/s", styles['TableTarget']),
         Paragraph("[                        ] m/s", styles['TableWriteBlank']),
         Paragraph("Stabilized glideslope approach", styles['TableCell'])],

        [Paragraph("Flare Airbrake Command Response", styles['TableCell']),
         Paragraph("Clamshell high-drag flare", styles['TableTarget']),
         Paragraph("[  ] CLEAN FLARE  [  ] SLOW", styles['TableWriteBlank']),
         Paragraph("H-54 / CONV-BRK deceleration", styles['TableCell'])],

        [Paragraph("Touchdown Airspeed (Wheels-On)", styles['TableCellBold']),
         Paragraph("4.50 to 5.20 m/s", styles['TableTarget']),
         Paragraph("[                        ] m/s", styles['TableWriteBlank']),
         Paragraph("Gentle wheel contact", styles['TableCell'])],

        [Paragraph("Landing Ground Rollout Distance", styles['TableCellBold']),
         Paragraph("≤ 8.5 m from touchdown to stop", styles['TableTarget']),
         Paragraph("[                        ] m", styles['TableWriteBlank']),
         Paragraph("Du-Bro wheel rollout braking", styles['TableCell'])],

        [Paragraph("Peak Vertical Touchdown Load", styles['TableCell']),
         Paragraph("≤ 1.8 g vertical spike", styles['TableTarget']),
         Paragraph("[                        ] g", styles['TableWriteBlank']),
         Paragraph("Gear damping performance", styles['TableCell'])],
    ]
    t_app = Table(app_data, colWidths=[2.3*inch, 1.4*inch, 1.5*inch, 2.0*inch])
    t_app.setStyle(TableStyle([
        ('BACKGROUND', (0,0), (-1,0), NAVY_PRIMARY),
        ('TEXTCOLOR', (0,0), (-1,0), colors.white),
        ('ALIGN', (0,0), (-1,-1), 'LEFT'),
        ('VALIGN', (0,0), (-1,-1), 'MIDDLE'),
        ('INNERGRID', (0,0), (-1,-1), 0.5, BORDER_GRAY),
        ('BOX', (0,0), (-1,-1), 1, NAVY_PRIMARY),
        ('TOPPADDING', (0,0), (-1,-1), 2),
        ('BOTTOMPADDING', (0,0), (-1,-1), 2),
        ('BACKGROUND', (2,1), (2,-1), WRITE_BG),
    ]))
    story.append(t_app)
    story.append(Spacer(1, 4))

    # Phase 4: Post-Flight Debrief
    story.append(Paragraph("5. Post-Flight Battery Recovery & Thermal Health Inspection", styles['SectionHeader']))
    deb_data = [
        [Paragraph("<b>Flight Duration:</b> [      ] min : [      ] sec", styles['TableCellBold']),
         Paragraph("<b>6S Rested Voltage:</b> [          ] V (≥ 21.8V)", styles['TableCellBold']),
         Paragraph("<b>Capacity Consumed:</b> [          ] mAh", styles['TableCellBold'])],
        [Paragraph("<b>2S Rested Voltage:</b> [          ] V (≥ 7.4V)", styles['TableCellBold']),
         Paragraph("<b>Max Motor Temp:</b> [          ] °C (≤ 65°C)", styles['TableCellBold']),
         Paragraph("<b>Max ODrive FET Temp:</b> [          ] °C (≤ 75°C)", styles['TableCellBold'])],
        [Paragraph("<b>Dyneema Lines Post-Flight:</b> [  ] ZERO FRAYING   [  ] WITNESS MARKS INTACT   [  ] TENSION VERIFIED (180N)", styles['TableCellBold']),
         Paragraph("", styles['TableCell']),
         Paragraph("", styles['TableCell'])],
    ]
    t_deb = Table(deb_data, colWidths=[2.4*inch, 2.4*inch, 2.4*inch])
    t_deb.setStyle(TableStyle([
        ('SPAN', (0,2), (2,2)),
        ('BACKGROUND', (0,0), (-1,-1), ICE_BG),
        ('BOX', (0,0), (-1,-1), 1, NAVY_PRIMARY),
        ('INNERGRID', (0,0), (-1,-1), 0.5, BORDER_GRAY),
        ('TOPPADDING', (0,0), (-1,-1), 2.5),
        ('BOTTOMPADDING', (0,0), (-1,-1), 2.5),
    ]))
    story.append(t_deb)
    story.append(Spacer(1, 6))

    story.append(build_signoff_block("TC-04 FREE FLIGHT ENVELOPE EXPANSION CERTIFICATION", styles))
    return story

def generate_cover_page(styles):
    story = []
    story.append(Spacer(1, 1.2*inch))
    story.append(Paragraph("MCO-1D MARK 1 METACHRONAL CASCADE ORNITHOPTER", ParagraphStyle(
        name='CoverPre', fontName='Helvetica-Bold', fontSize=12, leading=14, textColor=BLUE_ACCENT, alignment=1
    )))
    story.append(Spacer(1, 0.15*inch))
    story.append(Paragraph("FLIGHT TEST CARD & PHYSICAL MEASUREMENT FIELD BINDER", ParagraphStyle(
        name='CoverMain', fontName='Helvetica-Bold', fontSize=22, leading=26, textColor=NAVY_PRIMARY, alignment=1
    )))
    story.append(Spacer(1, 0.1*inch))
    story.append(Paragraph("Authoritative Engineering Data Collection Worksheets for Airframe Assembly, Bench Testing & Flight Trials", ParagraphStyle(
        name='CoverSub', fontName='Helvetica', fontSize=10, leading=14, textColor=TARGET_GRAY, alignment=1
    )))
    story.append(Spacer(1, 0.4*inch))
    story.append(HRFlowable(width="80%", thickness=2, color=NAVY_PRIMARY, spaceAfter=20, spaceBefore=0))
    story.append(Spacer(1, 0.2*inch))

    binder_index = [
        [Paragraph("<b>Worksheet Designation</b>", styles['TableCellBold']),
         Paragraph("<b>Title & Scope</b>", styles['TableCellBold']),
         Paragraph("<b>Hardware Milestone</b>", styles['TableCellBold']),
         Paragraph("<b>Pages</b>", styles['TableCellBold'])],
        
        [Paragraph("<b>WS-00</b>", styles['TableCellBold']),
         Paragraph("As-Built Physical Airframe & Structural Inspection", styles['TableCell']),
         Paragraph("Mechanical Assembly Baseline", styles['TableCell']),
         Paragraph("2 Pages", styles['TableCell'])],

        [Paragraph("<b>WS-01</b>", styles['TableCellBold']),
         Paragraph("Test Card TC-01: Ground Bench Test", styles['TableCell']),
         Paragraph("Rigid Load Stand Qualification", styles['TableCell']),
         Paragraph("1 Page", styles['TableCell'])],

        [Paragraph("<b>WS-02</b>", styles['TableCellBold']),
         Paragraph("Test Card TC-02: Tethered Lift & Hover", styles['TableCell']),
         Paragraph("Vertical Guide-Wire Gantry", styles['TableCell']),
         Paragraph("1 Page", styles['TableCell'])],

        [Paragraph("<b>WS-03</b>", styles['TableCellBold']),
         Paragraph("Test Card TC-03: Moving Vehicle Advance-Ratio Sweep", styles['TableCell']),
         Paragraph("Truck Pylon 6-Axis Load Cell", styles['TableCell']),
         Paragraph("1 Page", styles['TableCell'])],

        [Paragraph("<b>WS-04</b>", styles['TableCellBold']),
         Paragraph("Test Card TC-04: Free Flight STOL Runway Expansion", styles['TableCell']),
         Paragraph("Airfield Taxi, STOL & Circuit Sorties", styles['TableCell']),
         Paragraph("2 Pages", styles['TableCell'])],
    ]
    t_idx = Table(binder_index, colWidths=[1.1*inch, 3.2*inch, 2.1*inch, 0.8*inch])
    t_idx.setStyle(TableStyle([
        ('BACKGROUND', (0,0), (-1,0), NAVY_PRIMARY),
        ('TEXTCOLOR', (0,0), (-1,0), colors.white),
        ('ALIGN', (0,0), (-1,-1), 'LEFT'),
        ('VALIGN', (0,0), (-1,-1), 'MIDDLE'),
        ('INNERGRID', (0,0), (-1,-1), 0.5, BORDER_GRAY),
        ('BOX', (0,0), (-1,-1), 1, NAVY_PRIMARY),
        ('TOPPADDING', (0,0), (-1,-1), 6),
        ('BOTTOMPADDING', (0,0), (-1,-1), 6),
    ]))
    story.append(t_idx)
    story.append(Spacer(1, 0.8*inch))

    meta_box = [
        [Paragraph("<b>Demonstrator Program:</b> MCO-1A Subscale SAR Flight Demonstrator", styles['TableCell'])],
        [Paragraph("<b>Hardware Baseline:</b> 8-Wing Tandem Cascade | 10:1 Capstan Reduction | 5.687 kg MTOW", styles['TableCell'])],
        [Paragraph("<b>Field Protocol:</b> Print worksheets on standard Letter paper. Complete all fields in ballpoint ink. Photograph or scan completed sheets and upload directly into the engineering console.", styles['TableCell'])],
    ]
    t_mb = Table(meta_box, colWidths=[7.2*inch])
    t_mb.setStyle(TableStyle([
        ('BACKGROUND', (0,0), (-1,-1), ICE_BG),
        ('BOX', (0,0), (-1,-1), 1, BORDER_GRAY),
        ('TOPPADDING', (0,0), (-1,-1), 5),
        ('BOTTOMPADDING', (0,0), (-1,-1), 5),
        ('LEFTPADDING', (0,0), (-1,-1), 8),
    ]))
    story.append(t_mb)
    story.append(PageBreak())
    return story

def main():
    styles = get_custom_styles()
    
    # 1. WS-00 Individual PDF
    f_ws00 = os.path.join(OUTPUT_DIR, "MCO-1D_WS00_As_Built_Physical_Inspection.pdf")
    doc_ws00 = SimpleDocTemplate(f_ws00, pagesize=letter, leftMargin=36, rightMargin=36, topMargin=40, bottomMargin=40)
    doc_ws00.build(generate_ws00(styles), canvasmaker=NumberedCanvas)
    print(f"[OK] Generated: {f_ws00}")

    # 2. WS-01 Individual PDF
    f_ws01 = os.path.join(OUTPUT_DIR, "MCO-1D_WS01_TC01_Ground_Bench_Test.pdf")
    doc_ws01 = SimpleDocTemplate(f_ws01, pagesize=letter, leftMargin=36, rightMargin=36, topMargin=40, bottomMargin=40)
    doc_ws01.build(generate_ws01(styles), canvasmaker=NumberedCanvas)
    print(f"[OK] Generated: {f_ws01}")

    # 3. WS-02 Individual PDF
    f_ws02 = os.path.join(OUTPUT_DIR, "MCO-1D_WS02_TC02_Tethered_Lift_Hover.pdf")
    doc_ws02 = SimpleDocTemplate(f_ws02, pagesize=letter, leftMargin=36, rightMargin=36, topMargin=40, bottomMargin=40)
    doc_ws02.build(generate_ws02(styles), canvasmaker=NumberedCanvas)
    print(f"[OK] Generated: {f_ws02}")

    # 4. WS-03 Individual PDF
    f_ws03 = os.path.join(OUTPUT_DIR, "MCO-1D_WS03_TC03_Advance_Ratio_Truck_Sweep.pdf")
    doc_ws03 = SimpleDocTemplate(f_ws03, pagesize=letter, leftMargin=36, rightMargin=36, topMargin=40, bottomMargin=40)
    doc_ws03.build(generate_ws03(styles), canvasmaker=NumberedCanvas)
    print(f"[OK] Generated: {f_ws03}")

    # 5. WS-04 Individual PDF
    f_ws04 = os.path.join(OUTPUT_DIR, "MCO-1D_WS04_TC04_Free_Flight_STOL_Expansion.pdf")
    doc_ws04 = SimpleDocTemplate(f_ws04, pagesize=letter, leftMargin=36, rightMargin=36, topMargin=40, bottomMargin=40)
    doc_ws04.build(generate_ws04(styles), canvasmaker=NumberedCanvas)
    print(f"[OK] Generated: {f_ws04}")

    # 6. Complete Unified Binder PDF
    f_binder = os.path.join(OUTPUT_DIR, "MCO-1D_Mark1_All_Worksheets_Binder.pdf")
    doc_binder = SimpleDocTemplate(f_binder, pagesize=letter, leftMargin=36, rightMargin=36, topMargin=40, bottomMargin=40)
    binder_story = []
    binder_story.extend(generate_cover_page(styles))
    binder_story.extend(generate_ws00(styles))
    binder_story.append(PageBreak())
    binder_story.extend(generate_ws01(styles))
    binder_story.append(PageBreak())
    binder_story.extend(generate_ws02(styles))
    binder_story.append(PageBreak())
    binder_story.extend(generate_ws03(styles))
    binder_story.append(PageBreak())
    binder_story.extend(generate_ws04(styles))
    doc_binder.build(binder_story, canvasmaker=NumberedCanvas)
    print(f"[OK] Generated: {f_binder}")

if __name__ == '__main__':
    main()
