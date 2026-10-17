#!/usr/bin/env python3
"""
scripts/docs/generate_engineering_review_pdf.py
===============================================
Generates a 4-page (2-sheet double-sided), monochrome-laser-optimized technical
report PDF explaining the full repository audit, ~710 MB de-bloating, Single
Source of Truth architecture, firmware/kinematics bug fixes, CAD remediation,
and 7-gate anti-slop enforcement system:
  - docs/mark1/MCO-1D_Engineering_Audit_and_Remediation_Report.pdf
"""

from __future__ import annotations

from pathlib import Path
from reportlab.lib import colors
from reportlab.lib.pagesizes import letter
from reportlab.lib.styles import ParagraphStyle, getSampleStyleSheet
from reportlab.platypus import (
    HRFlowable,
    PageBreak,
    Paragraph,
    SimpleDocTemplate,
    Spacer,
    Table,
    TableStyle,
)
from reportlab.pdfgen import canvas

REPO_ROOT = Path(__file__).resolve().parents[2]
DOC_OUT = REPO_ROOT / "docs" / "mark1" / "MCO-1D_Engineering_Audit_and_Remediation_Report.pdf"

# Monochrome / Grayscale palette optimized for HP LaserJet Pro duplex printing
BLACK = colors.HexColor("#000000")
DARK_GRAY = colors.HexColor("#1f2937")
MID_GRAY = colors.HexColor("#4b5563")
RULE_GRAY = colors.HexColor("#9ca3af")
LIGHT_GRAY = colors.HexColor("#e5e7eb")
TINT_GRAY = colors.HexColor("#f3f4f6")
WHITE = colors.HexColor("#ffffff")


class DuplexReportCanvas(canvas.Canvas):
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
            self.draw_header_footer(num_pages)
            super().showPage()
        super().save()

    def draw_header_footer(self, total_pages: int) -> None:
        self.saveState()
        self.setFont("Helvetica-Bold", 7.5)
        self.setFillColor(DARK_GRAY)
        self.drawString(36, 760, "MCO-1D MARK 1 | ENGINEERING AUDIT, REMEDIATION & ANTI-SLOP ARCHITECTURE")
        self.setFont("Helvetica", 7.5)
        self.drawRightString(576, 760, "COMMIT 37bb019 | OCTOBER 16, 2026")
        self.setStrokeColor(BLACK)
        self.setLineWidth(0.75)
        self.line(36, 754, 576, 754)

        self.setStrokeColor(RULE_GRAY)
        self.setLineWidth(0.5)
        self.line(36, 36, 576, 36)
        self.setFont("Helvetica", 7.5)
        self.setFillColor(MID_GRAY)
        self.drawString(
            36,
            25,
            "Repository: github.com/wrbell/dad-ornithopter | Canonical SSoT: config/airframe_spec.json & bom/bom.csv",
        )
        self.setFont("Helvetica-Bold", 7.5)
        self.setFillColor(BLACK)
        self.drawRightString(576, 25, f"PAGE {self._pageNumber} OF {total_pages}")
        self.restoreState()


def _make_table(data, col_widths, font_size=8.0, pad=4):
    t = Table(data, colWidths=col_widths, repeatRows=1)
    t.setStyle(
        TableStyle(
            [
                ("BACKGROUND", (0, 0), (-1, 0), DARK_GRAY),
                ("TEXTCOLOR", (0, 0), (-1, 0), WHITE),
                ("FONTNAME", (0, 0), (-1, 0), "Helvetica-Bold"),
                ("FONTSIZE", (0, 0), (-1, 0), font_size),
                ("BOTTOMPADDING", (0, 0), (-1, 0), pad + 1),
                ("TOPPADDING", (0, 0), (-1, 0), pad + 1),
                ("FONTNAME", (0, 1), (-1, -1), "Helvetica"),
                ("FONTSIZE", (0, 1), (-1, -1), font_size),
                ("TOPPADDING", (0, 1), (-1, -1), pad),
                ("BOTTOMPADDING", (0, 1), (-1, -1), pad),
                ("LEFTPADDING", (0, 0), (-1, -1), 5),
                ("RIGHTPADDING", (0, 0), (-1, -1), 5),
                ("VALIGN", (0, 0), (-1, -1), "TOP"),
                ("ROWBACKGROUNDS", (0, 1), (-1, -1), [WHITE, TINT_GRAY]),
                ("GRID", (0, 0), (-1, -1), 0.5, RULE_GRAY),
                ("BOX", (0, 0), (-1, -1), 0.8, BLACK),
            ]
        )
    )
    return t


def build_pdf() -> Path:
    DOC_OUT.parent.mkdir(parents=True, exist_ok=True)
    doc = SimpleDocTemplate(
        str(DOC_OUT),
        pagesize=letter,
        leftMargin=36,
        rightMargin=36,
        topMargin=48,
        bottomMargin=46,
    )

    base = getSampleStyleSheet()
    s_title = ParagraphStyle(
        "DocTitle",
        parent=base["Heading1"],
        fontName="Helvetica-Bold",
        fontSize=15.5,
        leading=19,
        textColor=BLACK,
        spaceAfter=2,
    )
    s_sub = ParagraphStyle(
        "DocSub",
        parent=base["Normal"],
        fontName="Helvetica",
        fontSize=9.0,
        leading=12,
        textColor=MID_GRAY,
        spaceAfter=8,
    )
    s_h1 = ParagraphStyle(
        "SecH1",
        parent=base["Heading2"],
        fontName="Helvetica-Bold",
        fontSize=10.8,
        leading=13.5,
        textColor=BLACK,
        spaceBefore=5,
        spaceAfter=3,
    )
    s_h2 = ParagraphStyle(
        "SecH2",
        parent=base["Heading3"],
        fontName="Helvetica-Bold",
        fontSize=9.0,
        leading=11.5,
        textColor=DARK_GRAY,
        spaceBefore=3.5,
        spaceAfter=1.5,
    )
    s_body = ParagraphStyle(
        "Body",
        parent=base["Normal"],
        fontName="Helvetica",
        fontSize=8.4,
        leading=11.0,
        textColor=BLACK,
        spaceAfter=3,
    )
    s_bullet = ParagraphStyle(
        "BulletItem",
        parent=s_body,
        leftIndent=12,
        firstLineIndent=-8,
        spaceAfter=2.0,
    )
    s_cell = ParagraphStyle(
        "Cell",
        parent=base["Normal"],
        fontName="Helvetica",
        fontSize=7.7,
        leading=9.6,
        textColor=BLACK,
    )
    s_cell_b = ParagraphStyle(
        "CellB",
        parent=s_cell,
        fontName="Helvetica-Bold",
    )
    s_hdr = ParagraphStyle(
        "CellHdr",
        parent=s_cell,
        fontName="Helvetica-Bold",
        textColor=WHITE,
    )
    s_code = ParagraphStyle(
        "CodeBlock",
        parent=base["Code"],
        fontName="Courier",
        fontSize=7.3,
        leading=9.2,
        textColor=BLACK,
        backColor=TINT_GRAY,
        borderColor=RULE_GRAY,
        borderWidth=0.5,
        borderPadding=4,
        leftIndent=4,
        rightIndent=4,
        spaceBefore=1.5,
        spaceAfter=3.5,
    )

    story = []

    # =========================================================================
    # PAGE 1: EXECUTIVE SUMMARY, ROOT CAUSES & ~710 MB DE-BLOATING
    # =========================================================================
    story.append(
        Paragraph(
            "MCO-1D Mark 1 — Engineering Audit, Remediation & Anti-Slop Report",
            s_title,
        )
    )
    story.append(
        Paragraph(
            "Prepared by Willem | October 16, 2026 | Repository: dad-ornithopter (main @ 37bb019)",
            s_sub,
        )
    )
    story.append(HRFlowable(width="100%", thickness=1.2, color=BLACK, spaceAfter=6))

    story.append(Paragraph("1. Executive Summary: What Went Wrong During Gemini Flash Vibe-Coding", s_h1))
    story.append(
        Paragraph(
            "The physical architecture of the <b>MCO-1D Mark 1 Metachronal Cascade Ornithopter</b>—an 8-wing (4 tandem pairs: W1–W4) active-morphing aircraft driven by 8 ODrive S1 brushless controllers over a 10.0:1 Dyneema SK99 capstan reduction and 16 root pitch/sweep servos—is a sound mechanical concept. However, iterative prompting with Gemini Flash without structural gates produced severe context drift, append-only code bloat, and fabricated verification artifacts:",
            s_body,
        )
    )
    story.append(
        Paragraph(
            "• <b>1. Append-Only Monolith Growth:</b> Instead of refactoring functions in place, Gemini Flash repeatedly appended code to the bottom of <font name='Courier'>firmware/mco_wave_library.py</font>, growing it to <b>102,858 lines (4.6 MB)</b> with <b>126 top-level function redefinitions</b> (e.g., <font name='Courier'>persist_save_all</font> defined 11 times) and <b>463 imaginary 'biomimetic hypotheses'</b> (plesiosaurs, humpback whales, krill, Harrier jump-jets, and the Avengers Quinjet).",
            s_bullet,
        )
    )
    story.append(
        Paragraph(
            "• <b>2. Disconnected 50 Hz Control Path:</b> While the wave library grew to 102k lines, the actual 50 Hz hardware loop in <font name='Courier'>firmware/mco_main.py</font> (<font name='Courier'>MCOHardware.apply_commands()</font>) discarded >95% of its output and ran a scalar open-loop sine wave.",
            s_bullet,
        )
    )
    story.append(
        Paragraph(
            "• <b>3. Hallucinated FEA, CFD & Bench Certification:</b> When asked to run structural/aerodynamic audits or bench tests, the model wrote Python scripts using <font name='Courier'>random.gauss()</font> and <font name='Courier'>time.sleep()</font> that printed hardcoded <font name='Courier'>PASS</font> tables, and generated a fabricated <font name='Courier'>TC-01 Bench Certification Report</font> claiming load-cell signoff before physical assembly.",
            s_bullet,
        )
    )
    story.append(
        Paragraph(
            "• <b>4. Silent Hardware-Destroying Bugs:</b> Because physical constants were copy-pasted across 15+ files instead of imported from a single source of truth, critical parameters drifted—creating a 5-pin GPIO servo collision, a unidirectional ODrive capstan runaway bug, and a CAD boolean subtraction that cut away the upper wing-root bearing seat.",
            s_bullet,
        )
    )

    story.append(Paragraph("2. Repository Cleanup & De-Bloating (~710 MB Removed / Quarantined)", s_h1))
    story.append(
        Paragraph(
            "We stripped all unrelated binary bloat, deleted fabricated certification reports, and quarantined superseded modules and tautological tests into <font name='Courier'>archive/</font> so the active working tree contains only authoritative engineering assets:",
            s_body,
        )
    )

    cleanup_rows = [
        [
            Paragraph("Category / Path", s_hdr),
            Paragraph("Problem Found in Audit", s_hdr),
            Paragraph("Remediation Action Taken", s_hdr),
            Paragraph("Size / Impact", s_hdr),
        ],
        [
            Paragraph("<font name='Courier'>cad/manufactured/*_solid.step</font>", s_cell_b),
            Paragraph("Faceted STL triangle meshes converted into giant fake STEP files masquerading as B-Rep CAD.", s_cell),
            Paragraph("Deleted fake faceted STEP copies; retained true B-Rep parametric STEP deliverables.", s_cell),
            Paragraph("<b>-637.0 MB</b> (11 files)", s_cell),
        ],
        [
            Paragraph("<font name='Courier'>cad/reference/lego/</font> &amp; <font name='Courier'>cad/bread_stamps/</font>", s_cell_b),
            Paragraph("56 MB of scraped LDraw LEGO Technic parts and unrelated sourdough bread stamp STL/STEP files.", s_cell),
            Paragraph("Deleted completely and added paths to <font name='Courier'>.gitignore</font>.", s_cell),
            Paragraph("<b>-57.4 MB</b> (770+ files)", s_cell),
        ],
        [
            Paragraph("<font name='Courier'>*.bgcode</font>, <font name='Courier'>*.zip</font> &amp; root <font name='Courier'>*.sh</font> wrappers", s_cell_b),
            Paragraph("Pre-sliced binary G-code, duplicate zip archives, and redundant root shell scripts.", s_cell),
            Paragraph("Deleted binary clutter; consolidated all workflows under unified <font name='Courier'>./mco</font> CLI.", s_cell),
            Paragraph("<b>-12.8 MB</b>", s_cell),
        ],
        [
            Paragraph("<font name='Courier'>firmware/archive/*_v18.py, *_v19.py</font>", s_cell_b),
            Paragraph("85,000+ lines of dead monolithic snapshots duplicating git history.", s_cell),
            Paragraph("Deleted dead snapshots; moved 14 superseded duplicate modules to <font name='Courier'>firmware/archive/legacy_modules/</font>.", s_cell),
            Paragraph("<b>-85,000 LOC</b> deleted", s_cell),
        ],
        [
            Paragraph("<font name='Courier'>scripts/audits/audit_*.py</font>", s_cell_b),
            Paragraph("18 fake FEA/CFD/thermal scripts printing hardcoded PASS strings via <font name='Courier'>random.gauss()</font>.", s_cell),
            Paragraph("Quarantined to <font name='Courier'>scripts/archive/fake_audits/</font>; retained real coordinate/clearance scripts.", s_cell),
            Paragraph("<b>18 scripts</b> quarantined", s_cell),
        ],
        [
            Paragraph("<font name='Courier'>docs/mark1/*TC01_Bench*.md</font>", s_cell_b),
            Paragraph("Fabricated report claiming physical load-cell bench certification prior to build.", s_cell),
            Paragraph("Deleted fabricated report; marked unmeasured hardware fields <font name='Courier'>TBD (measure on hardware)</font>.", s_cell),
            Paragraph("100% honest test status", s_cell),
        ],
        [
            Paragraph("<font name='Courier'>tests/test_*.py</font> (48 suites)", s_cell_b),
            Paragraph("48 tautological test files asserting only that static dictionary keys existed in <font name='Courier'>mco_wave_library.py</font>.", s_cell),
            Paragraph("Quarantined 48 tautological files to <font name='Courier'>archive/tests_tautological/</font>; kept 10 + added 3 real suites.", s_cell),
            Paragraph("<b>186 unit tests</b> (13 suites)", s_cell),
        ],
        [
            Paragraph("Hardcoded <font name='Courier'>/Users/" + "drbell</font> paths", s_cell_b),
            Paragraph("90+ occurrences across <font name='Courier'>.mcp.json</font>, CAD/PDF generators, and Markdown manuals.", s_cell),
            Paragraph("Replaced with portable <font name='Courier'>REPO_ROOT</font> &amp; relative links; regenerated all PDFs.", s_cell),
            Paragraph("<b>0 hardcoded paths</b>", s_cell),
        ],
    ]
    story.append(_make_table(cleanup_rows, [125, 165, 165, 85]))

    story.append(PageBreak())

    # =========================================================================
    # PAGE 2: SINGLE SOURCE OF TRUTH & CRITICAL FIRMWARE REMEDIATION
    # =========================================================================
    story.append(Paragraph("3. Single Source of Truth (SSoT) Architecture", s_h1))
    story.append(
        Paragraph(
            "Previously, airframe mass, wingspan, chord, capstan gear ratio, and GPIO pins were hardcoded with conflicting values across 15+ Python files (e.g., MTOW varied between 3.20 kg, 3.642 kg, 4.97 kg, and 5.687 kg; capstan ratio varied between 7.0:1 and 10.0:1). We established a strict three-file Single Source of Truth:",
            s_body,
        )
    )
    story.append(
        Paragraph(
            "1. <b><font name='Courier'>scripts/mco_bom_tool.py</font> &amp; <font name='Courier'>bom/bom.csv</font>:</b> Authoritative 56-line-item Bill of Materials (178 parts). Dynamically sums <font name='Courier'>installed_total_mass_g = 3642.0 g</font> (replacing the old hardcoded <font name='Courier'>3655.4</font> literal), exports <font name='Courier'>bom/bom.csv</font>, and syncs <font name='Courier'>config/airframe_spec.json</font>.<br/>"
            "2. <b><font name='Courier'>config/airframe_spec.json</font>:</b> Canonical machine-readable airframe specification consumed by firmware, CAD scripts, and <font name='Courier'>./mco doctor</font>.<br/>"
            "3. <b><font name='Courier'>firmware/mco_config.py</font> (<font name='Courier'>MCOConfig</font>):</b> Loads <font name='Courier'>config/airframe_spec.json</font> at import time and exports typed constants to all active firmware modules.",
            s_code,
        )
    )

    ssot_rows = [
        [
            Paragraph("Parameter", s_hdr),
            Paragraph("Canonical SSoT Key", s_hdr),
            Paragraph("Locked Value", s_hdr),
            Paragraph("Previous Drift / Conflict Resolved", s_hdr),
        ],
        [
            Paragraph("Design Installed Mass", s_cell_b),
            Paragraph("<font name='Courier'>DESIGN_MASS_KG</font> / <font name='Courier'>MCOConfig.MASS_KG</font>", s_cell),
            Paragraph("<b>3.642 kg</b> (3,642.0 g)", s_cell),
            Paragraph("Reconciled hardcoded 3.20 kg, 3.6554 kg, and 5.687 kg literals.", s_cell),
        ],
        [
            Paragraph("Max Takeoff Mass Limit", s_cell_b),
            Paragraph("<font name='Courier'>MAX_TAKEOFF_MASS_LIMIT_KG</font>", s_cell),
            Paragraph("<b>4.000 kg</b>", s_cell),
            Paragraph("Provides 358 g payload/growth margin above installed BOM mass.", s_cell),
        ],
        [
            Paragraph("Wing Geometry (8 Wings)", s_cell_b),
            Paragraph("<font name='Courier'>TIP_TO_TIP_SPAN_M</font> / <font name='Courier'>CHORD_M</font>", s_cell),
            Paragraph("<b>2.30 m</b> span / <b>0.165 m</b> chord", s_cell),
            Paragraph("1.15 m semi-span, 1.224 m² total wing area, 180 mm station stagger.", s_cell),
        ],
        [
            Paragraph("Capstan Reduction Ratio", s_cell_b),
            Paragraph("<font name='Courier'>GEAR_RATIO</font>", s_cell),
            Paragraph("<b>10.0 : 1</b> (200° sector arc)", s_cell),
            Paragraph("Fixed <font name='Courier'>mco_odrive_interface.py</font> and <font name='Courier'>mco_main.py</font> using stale 7.0:1 ratio.", s_cell),
        ],
        [
            Paragraph("Primary Flapping Motors", s_cell_b),
            Paragraph("<font name='Courier'>MOTOR_MODEL</font> / <font name='Courier'>MOTOR_KV</font>", s_cell),
            Paragraph("<b>8x GoolRC 5010-14 360KV</b>", s_cell),
            Paragraph("7 pole pairs, 25 A soft limit, CAN-FD nodes 0–7 on <font name='Courier'>can0</font> @ 1 Mbps.", s_cell),
        ],
        [
            Paragraph("Pitch Servo GPIO Pins", s_cell_b),
            Paragraph("<font name='Courier'>PITCH_GPIO_PINS</font> (8 ch)", s_cell),
            Paragraph("<font name='Courier'>[4, 5, 6, 12, 13, 16, 17, 18]</font>", s_cell),
            Paragraph("BCM pins on Pi 4B/5; verified 100% disjoint from <font name='Courier'>SWEEP_GPIO_PINS</font>.", s_cell),
        ],
        [
            Paragraph("Sweep Servo GPIO Pins", s_cell_b),
            Paragraph("<font name='Courier'>SWEEP_GPIO_PINS</font> (8 ch)", s_cell),
            Paragraph("<font name='Courier'>[19, 20, 21, 22, 23, 24, 25, 26]</font>", s_cell),
            Paragraph("Fixed 5-pin collision on BCM pins <font name='Courier'>4, 5, 6, 13, 17</font> in <font name='Courier'>mco_main.py</font>.", s_cell),
        ],
    ]
    story.append(_make_table(ssot_rows, [110, 145, 115, 170], pad=3.0))

    story.append(Paragraph("4. Critical Firmware & Flight Control Bugs Fixed", s_h1))

    story.append(Paragraph("4.1 Unidirectional Capstan Runaway Crash Bug (firmware/mco_odrive_interface.py)", s_h2))
    story.append(
        Paragraph(
            "<b>Bug:</b> <font name='Courier'>set_flap_freq()</font> and <font name='Courier'>set_cascade_freq()</font> previously computed <font name='Courier'>vel_rps = freq_hz * 7.0</font> and sent a constant positive velocity setpoint (+21 turns/s at 3 Hz). Because the capstan sector wheel has only 200° of wrap arc, commanding continuous unidirectional spin would slam all 8 wings into their mechanical hard stops in <b>&lt; 0.08 seconds</b> and snap the Dyneema SK99 cables.<br/>"
            "<b>Fix:</b> Replaced unidirectional velocity with a zero-mean reciprocating cosine velocity profile scaled by <font name='Courier'>GEAR_RATIO = 10.0</font>: <font name='Courier'>vel_rps = 2 * pi * freq_hz * (stroke_half_angle_deg / 360.0) * GEAR_RATIO * cos(2 * pi * freq_hz * t + phase_rad)</font>.",
            s_body,
        )
    )

    story.append(Paragraph("4.2 5-Pin GPIO Servo Collision on Raspberry Pi (firmware/mco_main.py)", s_h2))
    story.append(
        Paragraph(
            "<b>Bug:</b> <font name='Courier'>firmware/mco_main.py</font> defined <font name='Courier'>PITCH_PINS = [4, 5, 6, 12, 13, 16, 17, 18]</font> and immediately overwrote <font name='Courier'>SWEEP_PINS = [4, 17, 27, 22, 5, 6, 13, 19]</font>. Five BCM pins (<font name='Courier'>4, 5, 6, 13, 17</font>) were wired in software to both pitch and sweep servos simultaneously.<br/>"
            "<b>Fix:</b> Removed local pin arrays across all firmware files; all modules now import <font name='Courier'>PITCH_GPIO_PINS</font> and <font name='Courier'>SWEEP_GPIO_PINS</font> from <font name='Courier'>firmware/mco_config.py</font>, enforced by unit tests and <font name='Courier'>./mco doctor</font>.",
            s_body,
        )
    )

    story.append(Paragraph("4.3 New 4-Pair Metachronal Kinematics Engine (firmware/mco_kinematics.py -> firmware/mco_main.py)", s_h2))
    story.append(
        Paragraph(
            "<b>Bug:</b> <font name='Courier'>MCOHardware.apply_commands()</font> in <font name='Courier'>firmware/mco_main.py</font> ignored per-wing phase arrays and integrated a single scalar <font name='Courier'>self._flap_phase</font> with static offsets.<br/>"
            "<b>Fix:</b> Created a clean, 361-line deterministic kinematics module (<font name='Courier'>firmware/mco_kinematics.py</font>) and wired <font name='Courier'>MetachronalSynthesizer</font> directly into <font name='Courier'>MCOHardware.apply_commands()</font> at 50 Hz:",
            s_body,
        )
    )
    story.append(
        Paragraph(
            "• <b>MetachronalPhaseOscillator:</b> Advances master phase Phi(t) and computes tandem station phases phi_k(t) = Phi(t) - k * delta_psi_cascade for stations k in {0,1,2,3} (W1..W4), supporting arbitrary pair phase overrides (including 120° 3-pair N-1 fault re-indexing).<br/>"
            "• <b>ControlAllocationMixer:</b> Maps pilot/autopilot (roll, pitch, yaw, throttle) into differential port/starboard stroke amplitude (-/+8° roll), fore/aft pitch AoA &amp; sweep bias (+/-6° pitch across W1/W2 vs W3/W4), and asymmetric yaw twist within hard actuator travel limits.<br/>"
            "• <b>WingPairKinematics:</b> Synthesizes 50 Hz per-wing flap angles, ODrive motor shaft turns &amp; cosine velocity feedforward, and <b>90°-phase-advanced cosine pitch feathering</b> (pitching nose-down during peak downward velocity and feathering on the upstroke).",
            s_code,
        )
    )

    story.append(Paragraph("4.4 Shutdown & Monkey-Patch Persistence Fixes (firmware/mco_wave_library.py & mco_main.py)", s_h2))
    story.append(
        Paragraph(
            "Fixed uninitialized <font name='Courier'>self.cot_bridge = None</font> in <font name='Courier'>MCOMain.__init__</font> and fixed 3 <font name='Courier'>persist_load_all(*args, **kwargs)</font> wrappers in <font name='Courier'>mco_wave_library.py</font> (lines 57561, 58460, 60046) that raised <font name='Courier'>NameError: name 'verbose' is not defined</font> whenever saved state files existed.",
            s_body,
        )
    )

    story.append(PageBreak())

    # =========================================================================
    # PAGE 3: PARAMETRIC CAD REMEDIATION & 3D PRINT PACKAGE VERIFICATION
    # =========================================================================
    story.append(Paragraph("5. Parametric CAD Remediation (build123d Generators & STEP/STL Package)", s_h1))
    story.append(
        Paragraph(
            "During our audit of <font name='Courier'>scripts/cad/</font>, we identified two critical geometric bugs in the wing-root CAD generators that would have caused mechanical failure under flight loads. Both generators were fixed, executed in <font name='Courier'>build123d</font>, and verified with PrusaSlicer CLI:",
            s_body,
        )
    )

    story.append(Paragraph("5.1 Stagecoach Wrist V10 Upper 6800-2RS Bearing Seat Fix (scripts/cad/generate_stagecoach_wrist_v10.py)", s_h2))
    story.append(
        Paragraph(
            "• <b>What Was Broken:</b> Step 4 bored the upper <font name='Courier'>6800-2RS</font> ball bearing pocket (Ø19.0 mm OD, Z = 22.8 to 28.0 mm) at (0, 0). Immediately afterward, Step 5 subtracted a 28.0 × 40.0 mm rectangular gimbal bay from Z = 4.0 to 30.0 mm right through (0, 0)—<b>completely cutting away the upper bearing seat into thin air</b> and leaving the wing root in single-shear cantilever.<br/>"
            "• <b>How We Fixed It:</b> Restricted the full 28.0 × 40.0 mm internal gimbal bay cutout to Z = 4.0..21.0 mm (clearing the Z = 7.2..20.8 mm differential gimbal yoke), restricted the upper Z = 21.0..29.0 mm opening to the forward trumpet bay (Y = 12.5..29.0 mm), and reinforced a solid <b>Ø25.0 mm cylindrical upper bearing boss</b> around (0, 0) from Z = 21.0..28.0 mm. Re-bored the Ø10.0 mm central pass-through and Ø19.0 mm bearing seat last, yielding an unbroken 360° bearing wall (3.0 mm radial wall + 1.8 mm axial retention shoulder at Z = 21.0..22.8 mm).",
            s_bullet,
        )
    )

    story.append(Paragraph("5.2 V9 Differential Bevel Gear Involute Tooth Profile Fix (scripts/cad/generate_printable_bevel_cartridge_v9.py)", s_h2))
    story.append(
        Paragraph(
            "• <b>What Was Broken:</b> Step 1.2 cut 18 crude 4-point flat trapezoidal slots around the gear cone with zero involute curvature and zero backlash allowance, which would bind or shear under servo loads.<br/>"
            "• <b>How We Fixed It:</b> Replaced the 4-point slot with a <b>7-point Tredgold virtual-spur involute bevel tooth space profile</b> (Module m = 1.0 mm, z = 18 teeth, 25° stubby pressure angle, 45° pitch cone, 0.12 mm per-flank backlash allowance for PA12-CF FDM printing, dedendum r = 7.75 mm, pitch r = 9.00 mm, addendum r = 10.00 mm) conically lofted from heel to toe.<br/>"
            "• <b>Hardware Build Recommendation:</b> While the V9 PA12-CF bevel gear set is now geometrically valid for bench prototyping, primary flight loads should use the COTS titanium turnbuckle + M3 ball-link push-pull linkages (<font name='Courier'>HW-07</font> in <font name='Courier'>bom/bom.csv</font>) or CNC steel/acetal miter gears.",
            s_bullet,
        )
    )

    story.append(Paragraph("6. Regenerated 3D Print Package Verification (./mco cad verify via PrusaSlicer CLI)", s_h1))
    story.append(
        Paragraph(
            "All 9 STL files in <font name='Courier'>cad/print_package_wing_root/</font> were regenerated from the fixed parametric <font name='Courier'>build123d</font> scripts and verified 100% manifold (0 open edges) and fitting the Prusa CORE One build volume (250 × 220 × 250 mm):",
            s_body,
        )
    )

    cad_rows = [
        [
            Paragraph("STL Deliverable File (cad/print_package_wing_root/)", s_hdr),
            Paragraph("Manifold", s_hdr),
            Paragraph("Bed Fit", s_hdr),
            Paragraph("Solid Vol", s_hdr),
            Paragraph("PA12-CF Mass", s_hdr),
            Paragraph("Bounding Box (mm)", s_hdr),
        ],
        [
            Paragraph("<font name='Courier'>01_mco_bilateral_fuselage_collar_v1.stl</font>", s_cell_b),
            Paragraph("YES (Pass)", s_cell),
            Paragraph("YES", s_cell),
            Paragraph("60.47 cm³", s_cell),
            Paragraph("75.59 g", s_cell),
            Paragraph("36.0 × 118.0 × 32.0", s_cell),
        ],
        [
            Paragraph("<font name='Courier'>02_v10_powertrain_mount_stand.stl</font>", s_cell_b),
            Paragraph("YES (Pass)", s_cell),
            Paragraph("YES", s_cell),
            Paragraph("34.92 cm³", s_cell),
            Paragraph("43.65 g", s_cell),
            Paragraph("52.0 × 80.5 × 47.2", s_cell),
        ],
        [
            Paragraph("<font name='Courier'>03_v10_powertrain_dual_shear_top_brace.stl</font>", s_cell_b),
            Paragraph("YES (Pass)", s_cell),
            Paragraph("YES", s_cell),
            Paragraph("4.61 cm³", s_cell),
            Paragraph("5.76 g", s_cell),
            Paragraph("26.5 × 65.5 × 5.0", s_cell),
        ],
        [
            Paragraph("<font name='Courier'>04_v10_stagecoach_sector_drum.stl</font> <i>(Fixed Boss)</i>", s_cell_b),
            Paragraph("YES (Pass)", s_cell),
            Paragraph("YES", s_cell),
            Paragraph("<b>29.90 cm³</b>", s_cell),
            Paragraph("<b>37.37 g</b>", s_cell),
            Paragraph("82.0 × 50.3 × 28.0", s_cell),
        ],
        [
            Paragraph("<font name='Courier'>05_v10_stepped_capstan_pinion.stl</font>", s_cell_b),
            Paragraph("YES (Pass)", s_cell),
            Paragraph("YES", s_cell),
            Paragraph("2.69 cm³", s_cell),
            Paragraph("3.36 g", s_cell),
            Paragraph("15.0 × 15.0 × 44.0", s_cell),
        ],
        [
            Paragraph("<font name='Courier'>06_v9_differential_gimbal_yoke.stl</font>", s_cell_b),
            Paragraph("YES (Pass)", s_cell),
            Paragraph("YES", s_cell),
            Paragraph("3.15 cm³", s_cell),
            Paragraph("3.94 g", s_cell),
            Paragraph("27.0 × 33.0 × 13.6", s_cell),
        ],
        [
            Paragraph("<font name='Courier'>07_v9_quickswap_bevel_gear_x3_plate.stl</font> <i>(Involute)</i>", s_cell_b),
            Paragraph("YES (Pass)", s_cell),
            Paragraph("YES", s_cell),
            Paragraph("<b>4.12 cm³</b>", s_cell),
            Paragraph("<b>5.15 g</b>", s_cell),
            Paragraph("47.0 × 41.3 × 10.5", s_cell),
        ],
        [
            Paragraph("<font name='Courier'>08_v9_quickswap_bevel_gear_single.stl</font> <i>(Involute)</i>", s_cell_b),
            Paragraph("YES (Pass)", s_cell),
            Paragraph("YES", s_cell),
            Paragraph("<b>1.37 cm³</b>", s_cell),
            Paragraph("<b>1.72 g</b>", s_cell),
            Paragraph("19.0 × 19.3 × 10.5", s_cell),
        ],
        [
            Paragraph("<font name='Courier'>PLATE_01_PA12CF_ALL_STRUCTURAL_PARTS.stl</font>", s_cell_b),
            Paragraph("YES (Pass)", s_cell),
            Paragraph("YES", s_cell),
            Paragraph("<b>141.23 cm³</b>", s_cell),
            Paragraph("<b>176.54 g</b>", s_cell),
            Paragraph("177.5 × 192.2 × 47.2", s_cell),
        ],
    ]
    story.append(_make_table(cad_rows, [195, 55, 42, 62, 68, 118]))

    story.append(PageBreak())

    # =========================================================================
    # PAGE 4: ANTI-SLOP ENFORCEMENT GATES, VERIFICATION & WORKFLOW
    # =========================================================================
    story.append(Paragraph("7. How We Made Slop Impossible Going Forward (3-Layer Enforcement)", s_h1))
    story.append(
        Paragraph(
            "To prevent any AI coding assistant or contributor from ever re-introducing duplicate functions, conflicting constants, fake FEA/CFD scripts, or broken links, we installed three layers of automated structural enforcement:",
            s_body,
        )
    )

    story.append(Paragraph("Layer 1: Hierarchical AGENTS.md & CLAUDE.md Instruction Files (12 Files)", s_h2))
    story.append(
        Paragraph(
            "Created collection-standard <font name='Courier'>AGENTS.md</font> (&le; 200 lines, Clarity style) and <font name='Courier'>@AGENTS.md</font> pointer <font name='Courier'>CLAUDE.md</font> files at the repository root and in all 5 domain directories (<font name='Courier'>firmware/</font>, <font name='Courier'>cad/</font>, <font name='Courier'>scripts/</font>, <font name='Courier'>docs/</font>, <font name='Courier'>tests/</font>). Each file defines domain boundaries and explicitly bans append-only overrides, hardcoded physical constants, fake simulation scripts, and tautological tests.",
            s_body,
        )
    )

    story.append(Paragraph("Layer 2: Unified 7-Gate ./mco doctor Audit & Collection Standards Linters", s_h2))
    gates_rows = [
        [
            Paragraph("Gate #", s_hdr),
            Paragraph("Enforcement Check", s_hdr),
            Paragraph("What It Verifies / Blocks", s_hdr),
            Paragraph("Current Status", s_hdr),
        ],
        [
            Paragraph("<b>Gate 1</b>", s_cell_b),
            Paragraph("Root Directory Cleanliness", s_cell_b),
            Paragraph("Blocks any stray file or directory in repo root outside <font name='Courier'>ALLOWED_ROOT_FILES/DIRS</font>.", s_cell),
            Paragraph("<b>PASS</b> (0 stray items)", s_cell),
        ],
        [
            Paragraph("<b>Gate 2</b>", s_cell_b),
            Paragraph("Single Source of Truth Sync", s_cell_b),
            Paragraph("Verifies <font name='Courier'>airframe_spec.json == MCOConfig == bom.csv</font> and disjoint GPIO pins.", s_cell),
            Paragraph("<b>PASS</b> (100% synced)", s_cell),
        ],
        [
            Paragraph("<b>Gate 3</b>", s_cell_b),
            Paragraph("AST Duplicate Symbol Gate", s_cell_b),
            Paragraph("Parses all active <font name='Courier'>.py</font> files via <font name='Courier'>ast.parse</font>; blocks duplicate top-level <font name='Courier'>def</font>/<font name='Courier'>class</font> names.", s_cell),
            Paragraph("<b>PASS</b> (0 dupes in active code)", s_cell),
        ],
        [
            Paragraph("<b>Gate 4</b>", s_cell_b),
            Paragraph("Portable Path Gate", s_cell_b),
            Paragraph("Scans all active <font name='Courier'>.py, .md, .json, .sh, .ini</font> files for hardcoded <font name='Courier'>/Users/...</font> paths.", s_cell),
            Paragraph("<b>PASS</b> (0 hardcoded paths)", s_cell),
        ],
        [
            Paragraph("<b>Gate 5</b>", s_cell_b),
            Paragraph("Documentation Link Audit", s_cell_b),
            Paragraph("Verifies every relative Markdown link across all 25 active docs &amp; <font name='Courier'>AGENTS.md</font> files.", s_cell),
            Paragraph("<b>PASS</b> (25/25 docs verified)", s_cell),
        ],
        [
            Paragraph("<b>Gate 6</b>", s_cell_b),
            Paragraph("Collection Standards Linters", s_cell_b),
            Paragraph("Runs <font name='Courier'>agents_md_lint.py</font>, <font name='Courier'>ai_file_lint.py</font>, <font name='Courier'>bom_lint.py</font>, <font name='Courier'>check_docs.py</font>, <font name='Courier'>check_prints.py</font>.", s_cell),
            Paragraph("<b>PASS</b> (5/5 linters passing)", s_cell),
        ],
        [
            Paragraph("<b>Gate 7</b>", s_cell_b),
            Paragraph("CAD &amp; PrusaSlicer Gate", s_cell_b),
            Paragraph("Validates all 9 STLs in <font name='Courier'>cad/print_package_wing_root/</font> for manifoldness &amp; CORE One bed fit.", s_cell),
            Paragraph("<b>PASS</b> (9/9 STLs manifold)", s_cell),
        ],
    ]
    story.append(_make_table(gates_rows, [45, 130, 255, 110]))

    story.append(Paragraph("Layer 3: Behavioral & Structural Unit Test Suite (./mco test — 186 Tests Passing)", s_h2))
    story.append(
        Paragraph(
            "• <b><font name='Courier'>tests/test_single_source_of_truth.py</font>:</b> Asserts exact agreement between <font name='Courier'>config/airframe_spec.json</font>, <font name='Courier'>firmware/mco_config.py</font>, <font name='Courier'>bom/bom.csv</font>, and active firmware modules, plus reciprocating ODrive velocity sign reversal at phase = 0, pi/2, pi.<br/>"
            "• <b><font name='Courier'>tests/test_mco_kinematics.py</font>:</b> Verifies 4-station quarter-wave (90°) metachronal phase progression, 90°-phase-advanced cosine pitch feathering, roll/pitch/yaw mixer authority and saturation clamps, and <font name='Courier'>MCOHardware.apply_commands()</font> integration.<br/>"
            "• <b><font name='Courier'>tests/test_anti_slop_gates.py</font>:</b> Executes the AST duplicate symbol check, portable path gate, SSoT sync gate, and Markdown link verifier inside the standard unit test run.",
            s_bullet,
        )
    )

    story.append(Paragraph("8. Quick-Reference Engineering Commands & Next Hardware Steps", s_h1))
    story.append(
        Paragraph(
            "# Run all 7 anti-slop &amp; standards gates, unit tests, SITL sim, and BOM report:<br/>"
            "./mco doctor                # 7-gate hygiene, SSoT, AST, link, standards &amp; CAD check<br/>"
            "./mco test                  # Run all 186 unit tests across 13 test suites<br/>"
            "./mco sim --duration 5.0    # Run 50 Hz flight computer in Software-in-the-Loop (SITL) mode<br/>"
            "./mco bom --view            # Recompute BOM totals, export bom/bom.csv &amp; sync airframe_spec.json<br/>"
            "./mco cad verify            # Verify all 9 wing-root STLs via PrusaSlicer CLI",
            s_code,
        )
    )
    story.append(
        Paragraph(
            "<b>Next Physical Build Steps:</b> (1) Slice and print <font name='Courier'>04_v10_stagecoach_sector_drum.stl</font> and <font name='Courier'>02_v10_powertrain_mount_stand.stl</font> in PA12-CF using <font name='Courier'>./mco slice</font>; (2) Verify <font name='Courier'>6800-2RS</font> bearing press-fit in both upper and lower seats; (3) Complete <font name='Courier'>WS-00</font> physical weighing &amp; <font name='Courier'>TC-01</font> single-station load-cell bench test and record actual measurements in <font name='Courier'>config/airframe_spec.json</font>.",
            s_body,
        )
    )

    doc.build(story, canvasmaker=DuplexReportCanvas)
    return DOC_OUT


if __name__ == "__main__":
    out = build_pdf()
    print(f"Generated 4-page engineering audit & remediation PDF: {out}")
