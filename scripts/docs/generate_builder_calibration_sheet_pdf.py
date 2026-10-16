#!/usr/bin/env python3
"""
generate_builder_calibration_sheet_pdf.py
=========================================
Generates the official, publication-grade 2-page PDF calibration build sheet:
  - docs/mark1/MCO-1D_Mark1_Builder_Calibration_Sheet.pdf
  - docs/latest/MCO-1D_Mark1_Builder_Calibration_Sheet.pdf

Features:
  - Clean aerospace typography & styling (Deep Navy, Slate Blue, Crisp Ice)
  - Comprehensive wing root geometry & cascade stagger measurement fields
  - Individual 8-wing panel measurement matrix (semi-span, mass, pitch, washout)
  - Sub-4 kg certified flight weight baseline (3.642 kg nominal, < 4.000 kg ceiling)
  - Running headers/footers with dynamic page numbering (NumberedCanvas)
  - High-contrast write-in boxes designed for hand-entry and scan ingestion
"""

import os
import sys
from reportlab.lib.pagesizes import letter
from reportlab.lib import colors
from reportlab.lib.units import inch
from reportlab.lib.styles import getSampleStyleSheet, ParagraphStyle
from reportlab.platypus import (
    SimpleDocTemplate, Paragraph, Spacer, Table, TableStyle, PageBreak, KeepTogether
)
from reportlab.pdfgen import canvas

# Palette
NAVY_PRIMARY = colors.HexColor("#1A365D")   # Deep navy header
BLUE_ACCENT  = colors.HexColor("#2B6CB0")   # Section accent
ICE_BG       = colors.HexColor("#F7FAFC")   # Light table row
BORDER_GRAY  = colors.HexColor("#CBD5E0")   # Clean subtle border
DARK_TEXT    = colors.HexColor("#2D3748")   # Charcoal body text
TARGET_GRAY  = colors.HexColor("#4A5568")   # Subdued target values
WRITE_BG     = colors.HexColor("#FFFDF0")   # High-contrast light yellow write-in box
WARN_AMBER   = colors.HexColor("#C05621")   # Alert callout

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
            canvas.Canvas.showPage(self)
        canvas.Canvas.save(self)

    def draw_page_decorations(self, page_count):
        self.saveState()
        self.setFont("Helvetica-Bold", 8)
        self.setFillColor(NAVY_PRIMARY)
        self.drawString(36, letter[1] - 26, "MCO-1D MARK 1 — AS-BUILT PHYSICAL MEASUREMENT & CALIBRATION BUILD SHEET")
        self.setFont("Helvetica", 8)
        self.setFillColor(TARGET_GRAY)
        self.drawRightString(letter[0] - 36, letter[1] - 26, "DOC ID: MCO-1D-CALIB-MK1-R2 — OFFICIAL BASELINE")
        self.setStrokeColor(BORDER_GRAY)
        self.setLineWidth(0.6)
        self.line(36, letter[1] - 30, letter[0] - 36, letter[1] - 30)

        self.line(36, 26, letter[0] - 36, 26)
        self.setFont("Helvetica", 7.5)
        self.drawString(36, 16, "Physical Airframe Calibration Manifest — Fill out with calibrated tools; hand off to avionics lead for firmware injection.")
        self.drawRightString(letter[0] - 36, 16, f"Page {self._pageNumber} of {page_count}")
        self.restoreState()

def get_styles():
    styles = getSampleStyleSheet()
    styles.add(ParagraphStyle('DocTitle', fontName='Helvetica-Bold', fontSize=13, leading=16, textColor=NAVY_PRIMARY, spaceAfter=2))
    styles.add(ParagraphStyle('DocSubTitle', fontName='Helvetica-Bold', fontSize=8, leading=10, textColor=BLUE_ACCENT, spaceAfter=4))
    styles.add(ParagraphStyle('SecHeader', fontName='Helvetica-Bold', fontSize=9, leading=11, textColor=NAVY_PRIMARY, spaceBefore=3, spaceAfter=2))
    styles.add(ParagraphStyle('TCell', fontName='Helvetica', fontSize=7.0, leading=8.5, textColor=DARK_TEXT))
    styles.add(ParagraphStyle('TCellCenter', fontName='Helvetica', fontSize=7.0, leading=8.5, textColor=DARK_TEXT, alignment=1))
    styles.add(ParagraphStyle('TCellBold', fontName='Helvetica-Bold', fontSize=7.0, leading=8.5, textColor=DARK_TEXT))
    styles.add(ParagraphStyle('TCellBoldCenter', fontName='Helvetica-Bold', fontSize=7.0, leading=8.5, textColor=DARK_TEXT, alignment=1))
    styles.add(ParagraphStyle('TTarget', fontName='Helvetica-Oblique', fontSize=6.8, leading=8.2, textColor=TARGET_GRAY))
    styles.add(ParagraphStyle('TBlank', fontName='Helvetica-Bold', fontSize=7.5, leading=9.0, textColor=colors.black, alignment=1))
    styles.add(ParagraphStyle('Instr', fontName='Helvetica', fontSize=6.8, leading=8.2, textColor=TARGET_GRAY, spaceAfter=2))
    return styles

def build_pdf(filename):
    doc = SimpleDocTemplate(
        filename,
        pagesize=letter,
        leftMargin=36,
        rightMargin=36,
        topMargin=36,
        bottomMargin=32
    )
    styles = get_styles()
    story = []

    # =========================================================================
    # PAGE 1
    # =========================================================================
    story.append(Paragraph("MCO-1D Mark 1 — As-Built Physical Measurement & Calibration Sheet", styles['DocTitle']))
    story.append(Paragraph("AUTHORITATIVE SHOP-FLOOR HANDOFF MANIFEST FOR FIRMWARE LAWS & KINEMATIC GOVERNORS (REV 2)", styles['DocSubTitle']))

    # Metadata Block
    meta_data = [
        [Paragraph("<b>AIRFRAME S/N:</b> MCO-1D-001", styles['TCell']),
         Paragraph("<b>BUILD DATE:</b> ____ / ____ / 2026", styles['TCell']),
         Paragraph("<b>SHOP LOCATION:</b> ____________________", styles['TCell'])],
        [Paragraph("<b>AIRFOIL:</b> [  ] S1223 &nbsp; [  ] SD7062", styles['TCellBold']),
         Paragraph("<b>AMBIENT TEMP:</b> ______ °C", styles['TCell']),
         Paragraph("<b>DIGITAL SCALE S/N:</b> ________________", styles['TCell'])],
        [Paragraph("<b>LEAD AVIONICS ENG:</b> ________________", styles['TCell']),
         Paragraph("<b>ATMOS PRESSURE:</b> ______ hPa", styles['TCell']),
         Paragraph("<b>CALIPER / RULE S/N:</b> _______________", styles['TCell'])]
    ]
    meta_table = Table(meta_data, colWidths=[2.4*inch, 2.3*inch, 2.5*inch])
    meta_table.setStyle(TableStyle([
        ('BACKGROUND', (0,0), (-1,-1), colors.HexColor("#EDF2F7")),
        ('BOX', (0,0), (-1,-1), 1, NAVY_PRIMARY),
        ('INNERGRID', (0,0), (-1,-1), 0.5, BORDER_GRAY),
        ('TOPPADDING', (0,0), (-1,-1), 2.5),
        ('BOTTOMPADDING', (0,0), (-1,-1), 2.5),
        ('LEFTPADDING', (0,0), (-1,-1), 4),
        ('RIGHTPADDING', (0,0), (-1,-1), 4),
    ]))
    story.append(meta_table)
    story.append(Spacer(1, 4))

    # Section 1: Airframe Weight, Balance & Center of Gravity
    story.append(Paragraph("1. Airframe Mass, Wheel Reactions & Longitudinal Center of Gravity", styles['SecHeader']))
    story.append(Paragraph("Accurately weigh flight-ready airframe with 6S 2200mAh LiPo and all 8 wings installed. Firmware dynamically updates stall speed Vs, climb Ps, and hover T/W from these values.", styles['Instr']))

    sec1_data = [
        [Paragraph("<b>Param ID</b>", styles['TCellBoldCenter']),
         Paragraph("<b>Calibration Parameter</b>", styles['TCellBold']),
         Paragraph("<b>Baseline / Nominal Target</b>", styles['TCellBold']),
         Paragraph("<b>As-Built Measured Value</b>", styles['TCellBoldCenter']),
         Paragraph("<b>Tolerance / Limits</b>", styles['TCellBold']),
         Paragraph("<b>Firmware Coupling & Action</b>", styles['TCellBold'])],

        [Paragraph("<b>M-01</b>", styles['TCellBoldCenter']),
         Paragraph("<b>All-Up Flying AUW</b><br/>All 8 wings + 6S pack installed", styles['TCellBold']),
         Paragraph("<b>3,642 g (S1223)</b><br/><b>3,636 g (SD7062)</b><br/>(Sub-4 kg envelope)", styles['TTarget']),
         Paragraph("[                      ] g", styles['TBlank']),
         Paragraph("Max MTOW: <b>< 4,000 g</b><br/>Target: 3,550–3,750 g", styles['TCell']),
         Paragraph("<b>PRIMARY GATE</b>: Recalculates stall speed floor Vs ∝ √(m), required lift L=mg, Boyd Ps, and hover T/W ratio (1.44:1 nominal).", styles['TCell'])],

        [Paragraph("<b>M-02</b>", styles['TCellBoldCenter']),
         Paragraph("<b>3-Point Landing Gear Tare</b><br/>Calibrated scales under wheels", styles['TCellBold']),
         Paragraph("Tailskid: ~10% (~360 g)<br/>Mains: ~45% ea (~1,640 g ea)", styles['TTarget']),
         Paragraph("Tail: [          ] g<br/>Port: [          ] g<br/>Stbd: [          ] g", styles['TBlank']),
         Paragraph("Port vs Stbd balance<br/>within <b>± 20 g</b>", styles['TCell']),
         Paragraph("Confirms longitudinal ground stability against tail tipping and verifies transverse lateral symmetry across gear saddle.", styles['TCell'])],

        [Paragraph("<b>M-03</b>", styles['TCellBoldCenter']),
         Paragraph("<b>Longitudinal CG Station</b><br/>Spine knife-edge balance", styles['TCellBold']),
         Paragraph("<b>352.6 mm aft datum</b><br/>(Sled Hole 3: 320mm arm)", styles['TTarget']),
         Paragraph("[                      ] mm", styles['TBlank']),
         Paragraph("352.6 ± 5.0 mm<br/>(Safe: 338–385 mm)", styles['TCell']),
         Paragraph("Verifies static margin (+48.3 mm / +33.6% MAC forward of neutral point X_NP = 400.9 mm) for positive pitch stability.", styles['TCell'])],
    ]

    sec1_table = Table(sec1_data, colWidths=[0.55*inch, 1.65*inch, 1.35*inch, 1.25*inch, 1.05*inch, 1.35*inch])
    sec1_table.setStyle(TableStyle([
        ('BACKGROUND', (0,0), (-1,0), NAVY_PRIMARY),
        ('TEXTCOLOR', (0,0), (-1,0), colors.white),
        ('ALIGN', (0,0), (-1,0), 'CENTER'),
        ('BOTTOMPADDING', (0,0), (-1,0), 2.5),
        ('TOPPADDING', (0,0), (-1,0), 2.5),
        ('BACKGROUND', (3,1), (3,-1), WRITE_BG),
        ('BOX', (0,0), (-1,-1), 1, NAVY_PRIMARY),
        ('INNERGRID', (0,0), (-1,-1), 0.5, BORDER_GRAY),
        ('VALIGN', (0,0), (-1,-1), 'MIDDLE'),
        ('TOPPADDING', (0,1), (-1,-1), 2.5),
        ('BOTTOMPADDING', (0,1), (-1,-1), 2.5),
        ('LEFTPADDING', (0,0), (-1,-1), 3),
        ('RIGHTPADDING', (0,0), (-1,-1), 3),
    ]))
    story.append(sec1_table)
    story.append(Spacer(1, 4))

    # Section 2: Wing Root Geometry & Longitudinal Cascade Stagger
    story.append(Paragraph("2. Wing Root Collar Geometry & Longitudinal Cascade Stagger (Central Spine)", styles['SecHeader']))
    story.append(Paragraph("Measure collar bearing centers along the 700mm carbon spine from fuselage nose tip (Datum 0.0mm). These dimensions govern dynamic tip clashing, inter-wing sweep clearances, and wake slipstream capture.", styles['Instr']))

    sec2_data = [
        [Paragraph("<b>Param ID</b>", styles['TCellBoldCenter']),
         Paragraph("<b>Geometric Feature</b>", styles['TCellBold']),
         Paragraph("<b>Nominal Design Target</b>", styles['TCellBold']),
         Paragraph("<b>As-Built Measured Value</b>", styles['TCellBoldCenter']),
         Paragraph("<b>Tolerance</b>", styles['TCellBold']),
         Paragraph("<b>Governing Physics & Impact</b>", styles['TCellBold'])],

        [Paragraph("<b>M-04</b>", styles['TCellBoldCenter']),
         Paragraph("<b>Root Lateral Spacing</b><br/>(2 · Y_root across spine)", styles['TCellBold']),
         Paragraph("<b>70.0 mm</b> bearing centers<br/>(Y = ±35.0 mm from spine)", styles['TTarget']),
         Paragraph("[                      ] mm", styles['TBlank']),
         Paragraph("70.0 ± 1.5 mm", styles['TCell']),
         Paragraph("<b>CRITICAL TIP CONTACT GEOMETRY</b>: Governs tip separation ΔY(θ) = 2(Y_root + (b/2)cosθ). Sets software stroke clamp θ_max ≤ 85°.", styles['TCell'])],

        [Paragraph("<b>M-05</b>", styles['TCellBoldCenter']),
         Paragraph("<b>Station 1 Arm (W1 Fore)</b><br/>Pivot center from nose datum", styles['TCellBold']),
         Paragraph("<b>X1 = 95.0 mm</b> aft datum<br/>Waterline: <b>Z1 = +48.0 mm</b>", styles['TTarget']),
         Paragraph("X1: [              ] mm<br/>Z1: [              ] mm", styles['TBlank']),
         Paragraph("X: ± 1.5 mm<br/>Z: ± 1.0 mm", styles['TCell']),
         Paragraph("Leading lifting pair arm; governs nose pitching moment and canard dynamic upwash shed onto downstream stations.", styles['TCell'])],

        [Paragraph("<b>M-06</b>", styles['TCellBoldCenter']),
         Paragraph("<b>Station 2 Arm (W2 Mid-Fore)</b><br/>Pivot center from nose datum", styles['TCellBold']),
         Paragraph("<b>X2 = 275.0 mm</b> (ΔX = 180mm)<br/>Waterline: <b>Z2 = +16.0 mm</b>", styles['TTarget']),
         Paragraph("X2: [              ] mm<br/>Z2: [              ] mm", styles['TBlank']),
         Paragraph("X: ± 1.5 mm<br/>Z: ± 1.0 mm", styles['TCell']),
         Paragraph("Second cascade station; establishes 90° metachronal wake capture timing τ = ΔX / V_inf for induced drag reduction.", styles['TCell'])],

        [Paragraph("<b>M-07</b>", styles['TCellBoldCenter']),
         Paragraph("<b>Station 3 Arm (W3 Mid-Aft)</b><br/>Pivot center from nose datum", styles['TCellBold']),
         Paragraph("<b>X3 = 455.0 mm</b> (ΔX = 180mm)<br/>Waterline: <b>Z3 = -16.0 mm</b>", styles['TTarget']),
         Paragraph("X3: [              ] mm<br/>Z3: [              ] mm", styles['TBlank']),
         Paragraph("X: ± 1.5 mm<br/>Z: ± 1.0 mm", styles['TCell']),
         Paragraph("Third cascade station; vertical drop (-32mm step) prevents chord-plane clash during differential sweep maneuvers.", styles['TCell'])],

        [Paragraph("<b>M-08</b>", styles['TCellBoldCenter']),
         Paragraph("<b>Station 4 Arm (W4 Aft)</b><br/>Pivot center from nose datum", styles['TCellBold']),
         Paragraph("<b>X4 = 635.0 mm</b> (ΔX = 180mm)<br/>Waterline: <b>Z4 = -48.0 mm</b>", styles['TTarget']),
         Paragraph("X4: [              ] mm<br/>Z4: [              ] mm", styles['TBlank']),
         Paragraph("X: ± 1.5 mm<br/>Z: ± 1.0 mm", styles['TCell']),
         Paragraph("Trailing station; downstream slipstream ejector. Governs pitch damping and overall tandem neutral point X_NP = 400.9 mm.", styles['TCell'])],

        [Paragraph("<b>M-09</b>", styles['TCellBoldCenter']),
         Paragraph("<b>Spar Quarter-Chord Offset</b><br/>Root rib LE to spar center", styles['TCellBold']),
         Paragraph("<b>36.0 mm</b> aft of root LE<br/>(Strict 25.0% MAC target)", styles['TTarget']),
         Paragraph("[                      ] mm", styles['TBlank']),
         Paragraph("36.0 ± 0.5 mm<br/>(24.5–25.5% MAC)", styles['TCell']),
         Paragraph("<b>AERO HINGE MOMENT GATE</b>: Eliminates aerodynamic moment arms on spar pivot; prevents pitching blowback on B24CLM pitch servos.", styles['TCell'])],
    ]

    sec2_table = Table(sec2_data, colWidths=[0.55*inch, 1.65*inch, 1.35*inch, 1.25*inch, 1.05*inch, 1.35*inch])
    sec2_table.setStyle(TableStyle([
        ('BACKGROUND', (0,0), (-1,0), NAVY_PRIMARY),
        ('TEXTCOLOR', (0,0), (-1,0), colors.white),
        ('ALIGN', (0,0), (-1,0), 'CENTER'),
        ('BOTTOMPADDING', (0,0), (-1,0), 2.5),
        ('TOPPADDING', (0,0), (-1,0), 2.5),
        ('BACKGROUND', (3,1), (3,-1), WRITE_BG),
        ('BOX', (0,0), (-1,-1), 1, NAVY_PRIMARY),
        ('INNERGRID', (0,0), (-1,-1), 0.5, BORDER_GRAY),
        ('VALIGN', (0,0), (-1,-1), 'MIDDLE'),
        ('TOPPADDING', (0,1), (-1,-1), 2.0),
        ('BOTTOMPADDING', (0,1), (-1,-1), 2.0),
        ('LEFTPADDING', (0,0), (-1,-1), 3),
        ('RIGHTPADDING', (0,0), (-1,-1), 3),
    ]))
    story.append(sec2_table)

    # Force Clean Page Break to Page 2
    story.append(PageBreak())

    # =========================================================================
    # PAGE 2
    # =========================================================================

    # Section 3: Wing Panel Semi-Spans, Weights & Rigging Angles
    story.append(Paragraph("3. Individual Wing Panel Matrix (Semi-Span, Mass, Neutral Pitch & Twist)", styles['SecHeader']))
    story.append(Paragraph("Measure each of the 8 assembled wing panels independently prior to airframe mounting. Ingested into ODrive FOC inertia matrices, roll trim schedules, and aerodynamic drag polar estimators.", styles['Instr']))

    sec3_data = [
        [Paragraph("<b>Wing Station</b>", styles['TCellBoldCenter']),
         Paragraph("<b>Side</b>", styles['TCellBoldCenter']),
         Paragraph("<b>Semi-Span (b/2)<br/>(Pivot to Tip)</b>", styles['TCellBoldCenter']),
         Paragraph("<b>Panel Mass (m)<br/>(Covered Panel)</b>", styles['TCellBoldCenter']),
         Paragraph("<b>Neutral Pitch (α0)<br/>(@ 1500µs PWM)</b>", styles['TCellBoldCenter']),
         Paragraph("<b>Tip Washout (Δα)<br/>(Root vs Tip Twist)</b>", styles['TCellBoldCenter']),
         Paragraph("<b>Tap Frequency<br/>(Dyneema Line)</b>", styles['TCellBoldCenter'])],

        # W1
        [Paragraph("<b>W1 Fore</b>", styles['TCellBoldCenter']),
         Paragraph("Port (L)", styles['TCellCenter']),
         Paragraph("[              ] mm", styles['TBlank']),
         Paragraph("[              ] g", styles['TBlank']),
         Paragraph("[              ] °", styles['TBlank']),
         Paragraph("[              ] °", styles['TBlank']),
         Paragraph("[              ] Hz", styles['TBlank'])],
        [Paragraph("<b>W1 Fore</b>", styles['TCellBoldCenter']),
         Paragraph("Stbd (R)", styles['TCellCenter']),
         Paragraph("[              ] mm", styles['TBlank']),
         Paragraph("[              ] g", styles['TBlank']),
         Paragraph("[              ] °", styles['TBlank']),
         Paragraph("[              ] °", styles['TBlank']),
         Paragraph("[              ] Hz", styles['TBlank'])],

        # W2
        [Paragraph("<b>W2 Mid-Fore</b>", styles['TCellBoldCenter']),
         Paragraph("Port (L)", styles['TCellCenter']),
         Paragraph("[              ] mm", styles['TBlank']),
         Paragraph("[              ] g", styles['TBlank']),
         Paragraph("[              ] °", styles['TBlank']),
         Paragraph("[              ] °", styles['TBlank']),
         Paragraph("[              ] Hz", styles['TBlank'])],
        [Paragraph("<b>W2 Mid-Fore</b>", styles['TCellBoldCenter']),
         Paragraph("Stbd (R)", styles['TCellCenter']),
         Paragraph("[              ] mm", styles['TBlank']),
         Paragraph("[              ] g", styles['TBlank']),
         Paragraph("[              ] °", styles['TBlank']),
         Paragraph("[              ] °", styles['TBlank']),
         Paragraph("[              ] Hz", styles['TBlank'])],

        # W3
        [Paragraph("<b>W3 Mid-Aft</b>", styles['TCellBoldCenter']),
         Paragraph("Port (L)", styles['TCellCenter']),
         Paragraph("[              ] mm", styles['TBlank']),
         Paragraph("[              ] g", styles['TBlank']),
         Paragraph("[              ] °", styles['TBlank']),
         Paragraph("[              ] °", styles['TBlank']),
         Paragraph("[              ] Hz", styles['TBlank'])],
        [Paragraph("<b>W3 Mid-Aft</b>", styles['TCellBoldCenter']),
         Paragraph("Stbd (R)", styles['TCellCenter']),
         Paragraph("[              ] mm", styles['TBlank']),
         Paragraph("[              ] g", styles['TBlank']),
         Paragraph("[              ] °", styles['TBlank']),
         Paragraph("[              ] °", styles['TBlank']),
         Paragraph("[              ] Hz", styles['TBlank'])],

        # W4
        [Paragraph("<b>W4 Aft</b>", styles['TCellBoldCenter']),
         Paragraph("Port (L)", styles['TCellCenter']),
         Paragraph("[              ] mm", styles['TBlank']),
         Paragraph("[              ] g", styles['TBlank']),
         Paragraph("[              ] °", styles['TBlank']),
         Paragraph("[              ] °", styles['TBlank']),
         Paragraph("[              ] Hz", styles['TBlank'])],
        [Paragraph("<b>W4 Aft</b>", styles['TCellBoldCenter']),
         Paragraph("Stbd (R)", styles['TCellCenter']),
         Paragraph("[              ] mm", styles['TBlank']),
         Paragraph("[              ] g", styles['TBlank']),
         Paragraph("[              ] °", styles['TBlank']),
         Paragraph("[              ] °", styles['TBlank']),
         Paragraph("[              ] Hz", styles['TBlank'])],

        # Target Reference Row
        [Paragraph("<b>NOMINAL TARGETS</b>", styles['TCellBoldCenter']),
         Paragraph("Both", styles['TCellCenter']),
         Paragraph("<b>1,150.0 mm</b> (±2.0)", styles['TTarget']),
         Paragraph("<b>69.0 g</b> (±2.5 g)", styles['TTarget']),
         Paragraph("<b>0.0°</b> (±0.5°)", styles['TTarget']),
         Paragraph("<b>0.0° to -1.5°</b>", styles['TTarget']),
         Paragraph("<b>210 Hz</b> (160 N)", styles['TTarget'])],
    ]

    sec3_table = Table(sec3_data, colWidths=[1.0*inch, 0.7*inch, 1.1*inch, 1.1*inch, 1.1*inch, 1.1*inch, 1.1*inch])
    sec3_table.setStyle(TableStyle([
        ('BACKGROUND', (0,0), (-1,0), NAVY_PRIMARY),
        ('TEXTCOLOR', (0,0), (-1,0), colors.white),
        ('ALIGN', (0,0), (-1,0), 'CENTER'),
        ('BOTTOMPADDING', (0,0), (-1,0), 2.5),
        ('TOPPADDING', (0,0), (-1,0), 2.5),
        ('BACKGROUND', (2,1), (-1,-2), WRITE_BG),
        ('BACKGROUND', (0,-1), (-1,-1), colors.HexColor("#EDF2F7")),
        ('BOX', (0,0), (-1,-1), 1, NAVY_PRIMARY),
        ('INNERGRID', (0,0), (-1,-1), 0.5, BORDER_GRAY),
        ('VALIGN', (0,0), (-1,-1), 'MIDDLE'),
        ('TOPPADDING', (0,1), (-1,-1), 2.0),
        ('BOTTOMPADDING', (0,1), (-1,-1), 2.0),
        ('LEFTPADDING', (0,0), (-1,-1), 3),
        ('RIGHTPADDING', (0,0), (-1,-1), 3),
    ]))
    story.append(sec3_table)
    story.append(Spacer(1, 4))

    # Section 4: Drivetrain, Pre-Tension & Avionics Calibrations
    story.append(Paragraph("4. Drivetrain, Tension, Kinematic Stops & Sensor Zero Calibrations", styles['SecHeader']))

    sec4_data = [
        [Paragraph("<b>Param ID</b>", styles['TCellBoldCenter']),
         Paragraph("<b>Subsystem / Mechanism</b>", styles['TCellBold']),
         Paragraph("<b>Baseline Target</b>", styles['TCellBold']),
         Paragraph("<b>As-Built Measured Value</b>", styles['TCellBoldCenter']),
         Paragraph("<b>Tolerance</b>", styles['TCellBold']),
         Paragraph("<b>Firmware Safety Action</b>", styles['TCellBold'])],

        [Paragraph("<b>M-10</b>", styles['TCellBoldCenter']),
         Paragraph("<b>Dyneema Rest Pre-Tension</b><br/>Acoustic tap pitch (150mm span)", styles['TCellBold']),
         Paragraph("<b>160.0 N (210 Hz)</b><br/>post-48h creep settlement", styles['TTarget']),
         Paragraph("[                      ] N<br/>Pitch: [          ] Hz", styles['TBlank']),
         Paragraph("160 ± 10 N<br/>(210 ± 8 Hz)", styles['TCell']),
         Paragraph("Eliminates slack-side cable derailment across reversals while protecting motor bearing radial life.", styles['TCell'])],

        [Paragraph("<b>M-11</b>", styles['TCellBoldCenter']),
         Paragraph("<b>Wingtip Dynamic Backlash</b><br/>Mechanical slop at 1.15m tip", styles['TCellBold']),
         Paragraph("<b>< 1.0 mm play</b> at tip<br/>(< 0.05° angular slop)", styles['TTarget']),
         Paragraph("[                      ] mm", styles['TBlank']),
         Paragraph("Max: 1.5 mm<br/>Target: < 0.8 mm", styles['TCell']),
         Paragraph("Ensures precision aerodynamic angle tracking; eliminates aeroelastic flutter trigger at high speed.", styles['TCell'])],

        [Paragraph("<b>M-12</b>", styles['TCellBoldCenter']),
         Paragraph("<b>Sector Wheel Travel Arc</b><br/>Mechanical stop-to-stop arc", styles['TCellBold']),
         Paragraph("<b>200.0° Total Arc</b><br/>(±100.0° from neutral)", styles['TTarget']),
         Paragraph("[                      ] deg", styles['TBlank']),
         Paragraph("198.0° to 202.0°<br/>Stop sym: ± 1.0°", styles['TCell']),
         Paragraph("Sets physical stops at ±100°; flight governor clamps dynamic stroke to f·θ0 ≤ 91.7°·Hz (≤1800 RPM).", styles['TCell'])],

        [Paragraph("<b>M-13</b>", styles['TCellBoldCenter']),
         Paragraph("<b>Pitot-Static Zero Offset</b><br/>MS4525DO indoor zero reading", styles['TCellBold']),
         Paragraph("<b>0.00 Pa</b> (covered tube)<br/>Calm room ambient", styles['TTarget']),
         Paragraph("[                      ] Pa", styles['TBlank']),
         Paragraph("± 2.0 Pa<br/>(± 0.02 hPa)", styles['TCell']),
         Paragraph("Calibrates low-speed rolling median filter before synthetic EKF3 airspeed blending takes over.", styles['TCell'])],
    ]

    sec4_table = Table(sec4_data, colWidths=[0.55*inch, 1.65*inch, 1.35*inch, 1.25*inch, 1.05*inch, 1.35*inch])
    sec4_table.setStyle(TableStyle([
        ('BACKGROUND', (0,0), (-1,0), NAVY_PRIMARY),
        ('TEXTCOLOR', (0,0), (-1,0), colors.white),
        ('ALIGN', (0,0), (-1,0), 'CENTER'),
        ('BOTTOMPADDING', (0,0), (-1,0), 2.5),
        ('TOPPADDING', (0,0), (-1,0), 2.5),
        ('BACKGROUND', (3,1), (3,-1), WRITE_BG),
        ('BOX', (0,0), (-1,-1), 1, NAVY_PRIMARY),
        ('INNERGRID', (0,0), (-1,-1), 0.5, BORDER_GRAY),
        ('VALIGN', (0,0), (-1,-1), 'MIDDLE'),
        ('TOPPADDING', (0,1), (-1,-1), 2.0),
        ('BOTTOMPADDING', (0,1), (-1,-1), 2.0),
        ('LEFTPADDING', (0,0), (-1,-1), 3),
        ('RIGHTPADDING', (0,0), (-1,-1), 3),
    ]))
    story.append(sec4_table)
    story.append(Spacer(1, 4))

    # Section 5: Pre-Flight Assembly Signoff Checklist
    story.append(Paragraph("5. Mandatory Pre-Flight Hardware Assembly Signoff Checklist", styles['SecHeader']))

    chk_data = [
        [Paragraph("[  ] <b>0. Sub-4 kg Flight AUW Verification</b>", styles['TCellBold']),
         Paragraph("Weigh complete flight-ready airframe with 6S 2200mAh pack. Confirm < 4,000 g (nominal 3,642 g). Record M-01.", styles['TCell'])],
        [Paragraph("[  ] <b>1. Dyneema 48-Hour Creep Settlement</b>", styles['TCellBold']),
         Paragraph("Cables rested at 200 N for 48h; re-tensioned to 160 N ± 10 N (210 Hz tap pitch); witness marks aligned; shrouds installed.", styles['TCell'])],
        [Paragraph("[  ] <b>2. Carbon Spar Internal Ferrules & Clamps</b>", styles['TCellBold']),
         Paragraph("6061-T6 ferrules (Ø8.0x40mm) epoxied into spar roots; spar clamp bolts torqued to 2.2 N·m with Loctite 243.", styles['TCell'])],
        [Paragraph("[  ] <b>3. Motor Thermal Mounts & NACA Duct Cooling</b>", styles['TCellBold']),
         Paragraph("0.5mm FR4/silicone isolator gaskets under all 8 GoolRC motors torqued to 1.8 N·m; canopy NACA ram-air scoop unobstructed.", styles['TCell'])],
        [Paragraph("[  ] <b>4. Center of Gravity Knife-Edge Balance</b>", styles['TCellBold']),
         Paragraph("Battery tray sled pin locked into Hole 3 (320mm arm); airframe balances on knife-edge at 352.6 ± 5.0 mm (Record M-03).", styles['TCell'])],
        [Paragraph("[  ] <b>5. Dual-Zone BEC Bus Electrical Droop Test</b>", styles['TCellBold']),
         Paragraph("Full 16-servo step deflection: Zone A & B rails hold ≥ 7.1V (droop ≤ 0.3V); 4,700µF buffer capacitors verified intact.", styles['TCell'])],
        [Paragraph("[  ] <b>6. Biomimetic Pterostigma Flutter Balance</b>", styles['TCellBold']),
         Paragraph("1.5g brass slugs verified at y=0.85b, x/c≈0.08 in all 8 wings; chordwise CG forward of elastic axis (≤ 24% chord).", styles['TCell'])],
        [Paragraph("[  ] <b>7. CAN-FD Termination Bus Resistance</b>", styles['TCellBold']),
         Paragraph("DMM measurement across CAN_H and CAN_L reads exactly 60.0 Ω ± 1.5 Ω with two 120-Ohm split termination resistors.", styles['TCell'])],
        [Paragraph("[  ] <b>8. Dynamic Kinematic Governor Floor Clamping</b>", styles['TCellBold']),
         Paragraph("FCC confirms f·θ0 ≤ 91.7°·Hz universal governor clamp active; ODrive velocity trips prevented below 1,800 RPM ceiling.", styles['TCell'])],
    ]
    chk_table = Table(chk_data, colWidths=[2.5*inch, 4.7*inch])
    chk_table.setStyle(TableStyle([
        ('BOX', (0,0), (-1,-1), 1, NAVY_PRIMARY),
        ('INNERGRID', (0,0), (-1,-1), 0.5, BORDER_GRAY),
        ('BACKGROUND', (0,0), (0,-1), colors.HexColor("#F7FAFC")),
        ('TOPPADDING', (0,0), (-1,-1), 1.8),
        ('BOTTOMPADDING', (0,0), (-1,-1), 1.8),
        ('LEFTPADDING', (0,0), (-1,-1), 4),
        ('RIGHTPADDING', (0,0), (-1,-1), 4),
    ]))
    story.append(chk_table)
    story.append(Spacer(1, 4))

    # Section 6: Official Engineering Signoff Block
    story.append(Paragraph("6. Production Handoff & TC-01 Bench Authorization", styles['SecHeader']))
    sign_data = [
        [Paragraph("<b>HANDOFF GATE:</b> Mark 1 Physical Calibration", styles['TCellBold']),
         Paragraph("<b>STATUS:</b> [  ] READY FOR FIRMWARE INJECTION   [  ] RE-WEIGH / ADJUST", styles['TCellBold'])],
        [Paragraph("<b>LEAD AIRFRAME BUILDER:</b> ___________________________", styles['TCell']),
         Paragraph("<b>SIGNATURE & DATE:</b> ___________________________", styles['TCell'])],
        [Paragraph("<b>AVIONICS & FIRMWARE LEAD:</b> _________________________", styles['TCell']),
         Paragraph("<b>FIRMWARE COMMIT HASH:</b> _______________________", styles['TCell'])],
        [Paragraph("<b>FLIGHT TEST DIRECTOR:</b> ___________________________", styles['TCell']),
         Paragraph("<b>TC-01 GROUND RUN APPROVAL:</b> [  ] APPROVED   [  ] HOLD", styles['TCellBold'])],
    ]
    sign_table = Table(sign_data, colWidths=[3.6*inch, 3.6*inch])
    sign_table.setStyle(TableStyle([
        ('BACKGROUND', (0,0), (-1,-1), colors.HexColor("#EDF2F7")),
        ('BOX', (0,0), (-1,-1), 1, NAVY_PRIMARY),
        ('INNERGRID', (0,0), (-1,-1), 0.5, BORDER_GRAY),
        ('TOPPADDING', (0,0), (-1,-1), 2.2),
        ('BOTTOMPADDING', (0,0), (-1,-1), 2.2),
        ('LEFTPADDING', (0,0), (-1,-1), 4),
        ('RIGHTPADDING', (0,0), (-1,-1), 4),
    ]))
    story.append(sign_table)

    # Build Document
    doc.build(story, canvasmaker=NumberedCanvas)
    print(f"[OK] Successfully built: {filename}")

if __name__ == "__main__":
    out_mark1 = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "docs", "mark1", "MCO-1D_Mark1_Builder_Calibration_Sheet.pdf"))
    out_latest = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "docs", "latest", "MCO-1D_Mark1_Builder_Calibration_Sheet.pdf"))
    os.makedirs(os.path.dirname(out_mark1), exist_ok=True)
    os.makedirs(os.path.dirname(out_latest), exist_ok=True)
    build_pdf(out_mark1)
    build_pdf(out_latest)
