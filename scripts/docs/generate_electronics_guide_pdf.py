#!/usr/bin/env python3
"""
Generate a professional, polished PDF technical briefing:
"MCO-1D Mark 1 Electronics Architecture, Assembly & Software Commissioning Guide"
Written for a computer technician / systems engineer style, yet explained with clear analogies for non-specialists.
"""

import sys
import os
from reportlab.lib.pagesizes import letter
from reportlab.lib import colors
from reportlab.lib.units import inch
from reportlab.lib.styles import getSampleStyleSheet, ParagraphStyle
from reportlab.platypus import (
    SimpleDocTemplate, Paragraph, Spacer, Table, TableStyle, KeepTogether, PageBreak, HRFlowable, Preformatted
)
from reportlab.pdfgen import canvas

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
            self.draw_page_number(num_pages)
            canvas.Canvas.showPage(self)
        canvas.Canvas.save(self)

    def draw_page_number(self, page_count):
        self.saveState()
        self.setFont("Helvetica", 8)
        self.setFillColor(colors.HexColor("#4A5568"))
        
        # Header (pages > 1)
        if self._pageNumber > 1:
            self.drawString(54, 11 * inch - 36, "MCO-1D Mark 1 — Systems Architecture, Electronics & Commissioning Manual")
            self.setStrokeColor(colors.HexColor("#CBD5E1"))
            self.setLineWidth(0.5)
            self.line(54, 11 * inch - 42, 8.5 * inch - 54, 11 * inch - 42)
        
        # Footer
        text = f"Page {self._pageNumber} of {page_count}"
        self.drawRightString(8.5 * inch - 54, 36, text)
        self.drawString(54, 36, "CONFIDENTIAL & PROPRIETARY — ADVANCED BIOMIMETIC FLIGHT SYSTEMS")
        self.setStrokeColor(colors.HexColor("#CBD5E1"))
        self.setLineWidth(0.5)
        self.line(54, 46, 8.5 * inch - 54, 46)
        
        self.restoreState()

def build_pdf(filename="MCO-1D_Electronics_and_Systems_Guide.pdf"):
    doc = SimpleDocTemplate(
        filename,
        pagesize=letter,
        leftMargin=54,
        rightMargin=54,
        topMargin=54,
        bottomMargin=54
    )

    styles = getSampleStyleSheet()

    # Custom styles
    primary_color = colors.HexColor("#0F172A") # slate 900
    accent_color = colors.HexColor("#1E3A8A")  # deep blue
    text_color = colors.HexColor("#1E293B")    # slate 800
    code_bg = colors.HexColor("#F8FAFC")

    title_style = ParagraphStyle(
        'DocTitle',
        parent=styles['Normal'],
        fontName='Helvetica-Bold',
        fontSize=20,
        leading=24,
        textColor=primary_color,
        spaceAfter=6
    )

    subtitle_style = ParagraphStyle(
        'DocSubTitle',
        parent=styles['Normal'],
        fontName='Helvetica-Bold',
        fontSize=11,
        leading=15,
        textColor=colors.HexColor("#2563EB"),
        spaceAfter=14
    )

    h1_style = ParagraphStyle(
        'Heading1_Custom',
        parent=styles['Heading1'],
        fontName='Helvetica-Bold',
        fontSize=13,
        leading=17,
        textColor=primary_color,
        spaceBefore=14,
        spaceAfter=6,
        keepWithNext=True
    )

    h2_style = ParagraphStyle(
        'Heading2_Custom',
        parent=styles['Heading2'],
        fontName='Helvetica-Bold',
        fontSize=10.5,
        leading=14,
        textColor=accent_color,
        spaceBefore=10,
        spaceAfter=4,
        keepWithNext=True
    )

    body_style = ParagraphStyle(
        'Body_Custom',
        parent=styles['Normal'],
        fontName='Helvetica',
        fontSize=8.5,
        leading=12.5,
        textColor=text_color,
        spaceAfter=6
    )

    body_bold = ParagraphStyle(
        'Body_Bold_Custom',
        parent=body_style,
        fontName='Helvetica-Bold'
    )

    callout_style = ParagraphStyle(
        'Callout_Custom',
        parent=styles['Normal'],
        fontName='Helvetica-Oblique',
        fontSize=8,
        leading=11.5,
        textColor=colors.HexColor("#0F766E")
    )

    table_cell = ParagraphStyle(
        'TableCell',
        parent=styles['Normal'],
        fontName='Helvetica',
        fontSize=7.5,
        leading=10,
        textColor=text_color
    )

    table_header = ParagraphStyle(
        'TableHeader',
        parent=styles['Normal'],
        fontName='Helvetica-Bold',
        fontSize=7.5,
        leading=10,
        textColor=colors.white
    )

    code_style = ParagraphStyle(
        'CodeSnippet',
        parent=styles['Normal'],
        fontName='Courier',
        fontSize=6.8,
        leading=8.8,
        textColor=colors.HexColor("#0F172A")
    )

    elements = []

    # Title Banner
    elements.append(Paragraph("MCO-1D MARK 1 METACHRONAL CASCADE ORNITHOPTER", title_style))
    elements.append(Paragraph("Electronics Suite Architecture, Assembly Blueprint, and Adaptive Learning Guide", subtitle_style))
    elements.append(Paragraph("<b>System Designation:</b> MCO-1D-SYS-MANUAL-MK1 &nbsp;|&nbsp; <b>Baseline:</b> October 2026 Authoritative Revision &nbsp;|&nbsp; <b>Flying Mass:</b> 3.67 kg", body_style))
    elements.append(HRFlowable(width="100%", thickness=1.5, color=accent_color, spaceBefore=4, spaceAfter=12))

    # SECTION 1: SYSTEM OVERVIEW (ANALOGY)
    elements.append(Paragraph("1. Executive Architecture Overview — The Aircraft as a Networked Server Rack", h1_style))
    elements.append(Paragraph(
        "To understand the electronics of the MCO-1D ornithopter, think of the entire airplane not as an unpredictable RC toy, "
        "but as a <b>compact, high-performance edge computing cluster and server rack that flies</b>. "
        "Instead of running cooling fans and spinning hard drives, this server powers eight robotic wings.",
        body_style
    ))
    elements.append(Paragraph(
        "The computer system is cleanly partitioned into three cooperating layers: "
        "<br/>• <b>The CPU / OS Layer (Raspberry Pi 4B):</b> Runs the primary real-time operating loop at 50 Hz (20 milliseconds per cycle), evaluates aerodynamic flight algorithms, and manages fleet telemetry."
        "<br/>• <b>The Dedicated Real-Time Coprocessor (Holybro Pixhawk 4 Mini):</b> Acts like an ultra-reliable hardware BIOS/BMC. It reads gyroscopes, accelerometers, and pilot radio signals 250 times a second, ensuring the plane stays level even if the OS is processing background data."
        "<br/>• <b>The Peripheral Actuator Bus (ODrive S1 Controllers & AGFRC Servos):</b> Connected via high-speed Controller Area Network (CAN-FD) and hardware PWM, functioning like precision motor drive cards plugged into an industrial PCI backplane.",
        body_style
    ))

    # Single battery explanation
    elements.append(Paragraph("The Single Battery Decision (Eliminating Secondary Packs)", h2_style))
    elements.append(Paragraph(
        "<b>The Computer Tech Analogy:</b> In early server installations, engineers used separate power supplies for CPU logic and disk drives due to fear of electrical line noise. "
        "Similarly, early designs of this aircraft planned for two separate battery packs (a 22.2V motor battery and a 7.4V servo battery). "
        "However, our recent adversarial audit verified that carrying two batteries added over <b>140 grams of dead weight</b>, introduced multiple battery failure points, and made pre-flight charging tedious.<br/>"
        "We streamlined the aircraft into a <b>Single-Battery Architecture</b> running a single 6S 22.2V LiPo (5000 mAh). "
        "High-efficiency switching voltage regulators (Synchronous Switching BECs) cleanly step this 22.2V down into dedicated 7.4V (servo) and 5.1V (avionics) rails. "
        "To prevent motor and servo electrical noise from resetting the computer, <b>two heavy-duty 4,700 µF solid electrolytic low-ESR capacitors</b> are soldered directly across the 7.4V servo rail, acting as local energy reservoirs and ripple filters.",
        body_style
    ))

    # Delivered vs Mandated AoA
    elements.append(Paragraph("Measuring Delivered Angle of Attack (Fix 1: AS5600L Magnetic Encoders)", h2_style))
    elements.append(Paragraph(
        "<b>The 'Commanded vs. Delivered' Problem:</b> Traditional hobby servos are open-loop to the flight computer: the computer commands '+6° pitch', but has no hardware line reading whether the wing actually made it there. "
        "Under aerodynamic pressure, air resistance pushes back against the carbon spar, causing the wing to bend or stall the servo.<br/>"
        "<b>The Hardware Fix:</b> We installed <b>eight AS5600L 12-bit contactless magnetic rotary encoders</b>—one on the pitch root shaft of every wing. "
        "A tiny diametric neodymium magnet spins with the wing spar, while a stationary microchip reads the magnetic field angle across a 1.0 mm air gap. "
        "This delivers <b>4,096 steps per revolution (0.088° resolution) with zero mechanical friction</b>. The flight computer now continuously compares <i>Commanded AoA</i> versus <i>Delivered AoA</i>, directly measuring aeroelastic wing flex in real time.",
        body_style
    ))

    elements.append(Spacer(1, 8))

    # SECTION 2: BOM TABLE
    elements.append(Paragraph("2. Master Electronics Bill of Materials (BOM)", h1_style))
    elements.append(Paragraph(
        "The complete hardware bill of materials for the updated 3.67 kg flying airframe is detailed below:",
        body_style
    ))

    bom_data = [
        [
            Paragraph("Item ID", table_header),
            Paragraph("Component & Spec", table_header),
            Paragraph("Qty", table_header),
            Paragraph("Role & Layperson Explanation", table_header),
            Paragraph("Bus / Connection", table_header)
        ],
        [
            Paragraph("<b>PROP-01</b>", table_cell),
            Paragraph("GoolRC 5010-14 360KV BLDC", table_cell),
            Paragraph("8", table_cell),
            Paragraph("Low-speed, high-torque pancake motor. Flaps each wing via a 10:1 rope capstan.", table_cell),
            Paragraph("3-Phase AC to ODrive S1", table_cell)
        ],
        [
            Paragraph("<b>PROP-02</b>", table_cell),
            Paragraph("ODrive S1 Motor Controller", table_cell),
            Paragraph("8", table_cell),
            Paragraph("High-speed FOC current/velocity servo driver for each flap motor.", table_cell),
            Paragraph("CAN-FD Bus (Nodes 0–7)", table_cell)
        ],
        [
            Paragraph("<b>PROP-03</b>", table_cell),
            Paragraph("50W 2.0 Ω Brake Resistors", table_cell),
            Paragraph("2", table_cell),
            Paragraph("Dissipates regenerative reverse current when wings decelerate at stroke limits.", table_cell),
            Paragraph("ODrive AUX_BRAKE", table_cell)
        ],
        [
            Paragraph("<b>ACT-01A</b>", table_cell),
            Paragraph("AGFRC B44BLS Metal Servos", table_cell),
            Paragraph("8", table_cell),
            Paragraph("Root sweep servos. Moves wings forward/back (30° to 90° sweep angle).", table_cell),
            Paragraph("7.4V Rail, 333 Hz PWM", table_cell)
        ],
        [
            Paragraph("<b>ACT-01B</b>", table_cell),
            Paragraph("AGFRC B24CLM Micro Servos", table_cell),
            Paragraph("8", table_cell),
            Paragraph("Root pitch servos. Twists the carbon spar to control wing Angle of Attack.", table_cell),
            Paragraph("7.4V Rail, 333 Hz PWM", table_cell)
        ],
        [
            Paragraph("<b>AV-01</b>", table_cell),
            Paragraph("Raspberry Pi 4B (4GB)", table_cell),
            Paragraph("1", table_cell),
            Paragraph("Primary Flight Computer (PFC). Executes 50 Hz kinematic executive loop and ALC.", table_cell),
            Paragraph("5.1V BEC, USB, GPIO", table_cell)
        ],
        [
            Paragraph("<b>AV-02</b>", table_cell),
            Paragraph("Waveshare CAN-FD HAT", table_cell),
            Paragraph("1", table_cell),
            Paragraph("High-speed SPI-to-CAN transceiver interface (1 Mbps nominal / 8 Mbps data).", table_cell),
            Paragraph("40-Pin Header, SPI", table_cell)
        ],
        [
            Paragraph("<b>AV-03</b>", table_cell),
            Paragraph("Holybro Pixhawk 4 Mini", table_cell),
            Paragraph("1", table_cell),
            Paragraph("Attitude Estimation & Safety Autopilot (Dual IMU, Baro, RC decoding).", table_cell),
            Paragraph("UART @ 921600 Baud", table_cell)
        ],
        [
            Paragraph("<b>AV-04</b>", table_cell),
            Paragraph("MS4525DO Pitot-Static Sensor", table_cell),
            Paragraph("1", table_cell),
            Paragraph("Differential air pressure sensor. Measures true forward flying airspeed.", table_cell),
            Paragraph("I2C Bus (0x28)", table_cell)
        ],
        [
            Paragraph("<b>AV-05</b>", table_cell),
            Paragraph("Benewake TF-Luna LiDAR", table_cell),
            Paragraph("1", table_cell),
            Paragraph("Micro downward laser rangefinder. Measures precision altitude above ground (0.1–8m).", table_cell),
            Paragraph("UART / Serial @ 115200", table_cell)
        ],
        [
            Paragraph("<b>AV-12</b>", table_cell),
            Paragraph("AS5600L Magnetic Encoders", table_cell),
            Paragraph("8", table_cell),
            Paragraph("<b>Fix 1:</b> Measures delivered wing pitch AoA directly across contactless air gap.", table_cell),
            Paragraph("I2C / Multiplexed (0x36)", table_cell)
        ],
        [
            Paragraph("<b>PWR-01</b>", table_cell),
            Paragraph("6S LiPo Battery (22.2V 5000mAh)", table_cell),
            Paragraph("1", table_cell),
            Paragraph("Single main propulsion & system battery pack. Sits in nosepod cradle.", table_cell),
            Paragraph("XT90 Anti-Spark Lead", table_cell)
        ],
        [
            Paragraph("<b>PWR-02</b>", table_cell),
            Paragraph("Matek 20A HV Synchronous BEC", table_cell),
            Paragraph("1", table_cell),
            Paragraph("Steps 22.2V down to clean 7.4V (servo bus) and 5.1V (avionics logic).", table_cell),
            Paragraph("22.2V In / Dual Out", table_cell)
        ],
        [
            Paragraph("<b>PWR-04</b>", table_cell),
            Paragraph("Panasonic 4,700 µF 16V Caps", table_cell),
            Paragraph("2", table_cell),
            Paragraph("Low-ESR reservoir capacitors. Absorbs 16-servo synchronous stall spikes.", table_cell),
            Paragraph("Bridged across 7.4V bus", table_cell)
        ],
        [
            Paragraph("<b>PWR-07</b>", table_cell),
            Paragraph("TI INA226 Current Shunt", table_cell),
            Paragraph("1", table_cell),
            Paragraph("<b>Fix 2:</b> Continuously measures current and watts burned on the 7.4V servo bus.", table_cell),
            Paragraph("I2C Bus (0x40) to Pi", table_cell)
        ]
    ]

    t_bom = Table(bom_data, colWidths=[55, 110, 25, 205, 105])
    t_bom.setStyle(TableStyle([
        ('BACKGROUND', (0, 0), (-1, 0), primary_color),
        ('ALIGN', (0, 0), (-1, -1), 'LEFT'),
        ('VALIGN', (0, 0), (-1, -1), 'TOP'),
        ('BOTTOMPADDING', (0, 0), (-1, -1), 3),
        ('TOPPADDING', (0, 0), (-1, -1), 3),
        ('ROWBACKGROUNDS', (0, 1), (-1, -1), [colors.white, colors.HexColor("#F1F5F9")]),
        ('GRID', (0, 0), (-1, -1), 0.5, colors.HexColor("#CBD5E1")),
    ]))
    elements.append(t_bom)

    elements.append(Spacer(1, 10))

    # SECTION 3: SCHEMATIC
    elements.append(Paragraph("3. Complete Electrical Interconnect Schematic", h1_style))
    elements.append(Paragraph(
        "The following diagram illustrates the complete power distribution, communication lines, and sensor interfaces:",
        body_style
    ))

    schematic_text = """
                     +─────────────────────────────────────────────────────────────+
                     │          SINGLE 6S LiPo MAIN BATTERY (22.2V, 5000 mAh)      │
                     +──────────────────────────────┬──────────────────────────────+
                                                    │ (XT90 Anti-Spark Lead / 10 AWG)
                                                    ▼
                     +─────────────────────────────────────────────────────────────+
                     │        Matek 8-Way Motor PDB (120A Main Propulsion Bus)     │
                     +──────────────┬───────────────────────────────┬──────────────+
                                    │                               │
        ┌───────────────────────────┴──────────┐                    │ (22.2V Secondary Tap)
        │ 8x 30A Blade Fused Motor Outputs     │                    ▼
        ▼                                      ▼          +───────────────────+
  +─────────────────────────+      +───────────────────+  │ Matek 20A HV BEC  │
  │ 8x ODrive S1 Controller │      │ 2x 50W 2.0Ω Brake │  │ (Converts 22.2V   │
  │ Nodes (CAN IDs: 0 to 7) │      │ Resistors         │  │  to 7.4V & 5.1V)  │
  +────────────┬────────────+      +───────────────────+  +─────────┬─────────+
               │ 3-Phase AC                                         │
               ▼                                                    ├──────────────────────────┐
  +─────────────────────────+                                       │ Clean 7.4V Servo Bus     │ Clean 5.1V Avionics Bus
  │ 8x GoolRC 5010 Motors   │                                       ▼                          ▼
  │ (10:1 Capstan Reducers) │                        +───────────────────────────+    +───────────────────────────+
  +─────────────────────────+                        │ TI INA226 Current Monitor │    │   Raspberry Pi 4B (PFC)   │
                                                     │ (Fix 2: I2C Shunt to Pi)  │    │  (Runs 50 Hz Flight Loop) │
                                                     +──────────────┬────────────+    +─────────┬──────────┬──────+
                                                                    │                           │ SPI      │ UART
                                                                    ▼                           ▼          ▼ 921600
                                                     +───────────────────────────+    +─────────────────+  │
                                                     │ 16-Port High-Current Rail │    │ Waveshare CAN-FD│  │
                                                     │ + 2x 4,700µF Low-ESR Caps │    │ HAT (120Ω Term) │  │
                                                     +──────────────┬────────────+    +─────────┬───────+  │
                                                                    │                           │          │
                                                                    ├───────────────────────────┼──────────┘
                                                                    │                           │
                               ┌────────────────────────────────────┴───────────┐               │ CAN_H / CAN_L
                               ▼                                                ▼               ▼ (1 Mbps Bus)
                  +───────────────────────────+                    +──────────────────────────+ [ODrive 0]═...═[ODrive 7]
                  │ 8x Root Sweep Servos      │                    │ 8x Wing Pitch Servos     │
                  │ (AGFRC B44BLS - 333Hz PWM)│                    │ (AGFRC B24CLM - 333Hz PWM│
                  +───────────────────────────+                    +────────────┬─────────────+
                                                                                │
                                                                                ▼ (Wing Spar Pitch Axis)
                                                                   +──────────────────────────+
                                                                   │ 8x AS5600L Angle Sensors │
                                                                   │ (Fix 1: Direct AoA / I2C)│
                                                                   +──────────────────────────+
    ════════════════════════════════════════════════════════════════════════════════════════════════════════════════════
    AVIONICS EXPANSION (Holybro Pixhawk 4 Mini Autopilot Unit):
    • Connected to Raspberry Pi via High-Speed Serial (/dev/ttyAMA0 @ 921600 baud MAVLink stream)
    • MS4525DO Pitot Airspeed Sensor: I2C Port 0x28 (Reads Indicated Airspeed in m/s)
    • Benewake TF-Luna LiDAR: UART Serial @ 115200 baud (Measures Above-Ground-Level clearance 0.1 to 8.0 m)
    • FrSky Archer Plus RS Receiver: SBUS Port (Reads pilot RadioMaster TX16S sticks)
    • Holybro SiK Telemetry Radio: TELEM1 @ 57600 baud (Wireless bidirectional link to laptop ground station)
    ════════════════════════════════════════════════════════════════════════════════════════════════════════════════════
"""
    elements.append(Preformatted(schematic_text, code_style))

    elements.append(PageBreak())

    # SECTION 4: STEP-BY-STEP ASSEMBLY GUIDE
    elements.append(Paragraph("4. Step-by-Step Installation & Electronics Assembly Guide", h1_style))
    elements.append(Paragraph(
        "Follow this physical checklist in sequence during airframe outfitting. Every connection uses keyed, locking connectors.",
        body_style
    ))

    steps = [
        ("Step 1: Fuselage Pod & Battery Tray Preparation",
         "Mount the 370 mm hybrid aerodynamic pod into the lower bay of the carbon delta truss. "
         "Position the single 6S LiPo battery in the anti-slip keel cradle at Hole 3 (X = 250 mm). "
         "This mechanically locks the aircraft Center of Gravity (CG) at 467.6 mm aft of the nose (26.5% MAC), perfectly balancing the airframe."),
        
        ("Step 2: Motor Bus & Controller Heat Sinking",
         "Bolt the 8 ODrive S1 controllers onto the mid-deck mounting tray, installing 0.5 mm silicone thermal barrier gaskets between the aluminum baseplates and the frame. "
         "Connect each ODrive to the Matek 8-Way PDB using 16 AWG silicone wire with 30A inline automotive blade fuses. "
         "Ensure cooling air from the nose NACA ducts flows unimpeded over the ODrive heatsinks and exits through the tail scavenge louvers."),

        ("Step 3: CAN-FD Daisy Chain Wiring (Crucial Bus Rules)",
         "Run a twisted-pair shielded cable (CAT6 grade) starting at the Waveshare CAN-FD HAT on the Raspberry Pi. "
         "Wire CAN_H and CAN_L sequentially from ODrive 0 to ODrive 7. "
         "<b>Bus Termination:</b> Flip the 120 Ω termination switch ON at the Raspberry Pi HAT (start of bus), and solder a 120 Ω resistor across CAN_H and CAN_L at ODrive 7 (end of bus). "
         "All intermediate controllers (ODrives 1 through 6) must have their termination resistors disabled."),

        ("Step 4: Servo Power Bus, Current Shunt & Noise Filtration",
         "Route the 7.4V BEC output through the TI INA226 current shunt board before entering the 16-port servo distribution rail. "
         "Solder the two 4,700 µF low-ESR capacitors directly across the +7.4V and GND terminals of the servo rail. "
         "Verify capacitor polarity: striped negative lead to GND, positive lead to +7.4V. "
         "Connect the 8 sweep servos (AGFRC B44BLS) and 8 pitch servos (AGFRC B24CLM) to the rail. Connect signal wires to the Raspberry Pi GPIO headers."),

        ("Step 5: Installing the AS5600L Magnetic Pitch Encoders (Fix 1)",
         "Epoxy a 6.0 mm diametric neodymium disc magnet flat against the outer circular face of each wing's pitch spar shaft. "
         "Fasten the AS5600L sensor breakout board to the fixed bracket on the fuselage frame directly opposite the magnet. "
         "Set the air gap between the chip face and magnet surface to exactly 1.0 mm ± 0.3 mm (the thickness of a credit card). "
         "Route the 4-wire I2C ribbon cable (3.3V, GND, SDA, SCL) to the Raspberry Pi multiplexer, keeping wires twisted with ground to reject motor noise."),

        ("Step 6: Autopilot, Sensors & Radio Integration",
         "Mount the Holybro Pixhawk 4 Mini on vibration-damping silicone foam on the aircraft centerline. "
         "Connect the MS4525DO Pitot tube to the nose fairing and plug its I2C lead into the Pixhawk. "
         "Fasten the TF-Luna downward LiDAR inside the belly cutout, ensuring a clean vertical view of the ground. "
         "Mount the FrSky Archer Plus RS receiver antennas at 90° separation on the composite canopy and plug the SBUS lead into the Pixhawk RC port.")
    ]

    for title, desc in steps:
        elements.append(Paragraph(f"<b>{title}</b>", h2_style))
        elements.append(Paragraph(desc, body_style))

    elements.append(Spacer(1, 10))

    # SECTION 5: FIRMWARE LOADING & QUARANTINE GOVERNOR
    elements.append(Paragraph("5. Firmware Deployment & The Maiden Flight Quarantine Governor", h1_style))
    elements.append(Paragraph(
        "<b>Loading the Software Stack:</b> The flight software is loaded directly onto the Raspberry Pi 4B over local Wi-Fi or Ethernet: "
        "<br/><code>ssh pi@mco-1.local</code><br/>"
        "<code>cd ~/Code/flight && bash scripts/mco_install.sh</code><br/>"
        "This script configures high-speed GPIO timers (<code>pigpiod</code>), sets the CAN interface (<code>can0</code> at 1 Mbps), "
        "and activates two automated system services: <code>mco-flight.service</code> (runs the 50 Hz control loop) and <code>mco-panel.service</code> (telemetry server).",
        body_style
    ))
    elements.append(Paragraph(
        "<b>Calibration File Loading:</b> Before flight, the software reads <code>firmware/calibrated_airframe_profile.json</code>. "
        "This contains builder-verified bench measurements: as-built flying mass (3.67 kg), spar pivot axis (25% MAC), rope preload (180 N), and servo zero trims.",
        body_style
    ))
    elements.append(Paragraph(
        "<b>The Maiden Flight Quarantine Governor:</b> Earlier simulation passes contained over 285 experimental waveforms with large flap angles (±96°) and high-frequency reversals. "
        "To guarantee airframe safety on maiden flight, the software activates the <b>WaveLibraryQuarantineGovernor</b> in <code>mco_flight_governor.py</code>. "
        "This places experimental hypotheses into temporary quarantine and enforces three proven, gentle flight modes:<br/>"
        "• <b>Mode 1 (Taxi / Ground Rollout):</b> Flap frequency clamped to <i>f</i> ≤ 1.5 Hz, amplitude ≤ ±15°, wing incidence held at +3.0° forward thrust bias.<br/>"
        "• <b>Mode 2 (Climb & Cruise):</b> Frequency clamped to 1.2–2.0 Hz, amplitude ≤ ±22°, incidence fixed at +3.5° (maximum L/D glide trim for the S1223 airfoil).<br/>"
        "• <b>Mode 3 (Deadstick Glide):</b> Motors stop (<i>f</i> = 0 Hz), wings lock level, servos operate as standard glider elevons.<br/>"
        "This turns the ornithopter into a predictable, high-aspect-ratio tandem motor-glider that cannot stall or over-stress its linkages on first flight.",
        body_style
    ))

    elements.append(Spacer(1, 10))

    # SECTION 6: ALC EVOLUTION & THREE CHEAP FIXES
    elements.append(Paragraph("6. How Hypotheses Are Unlocked & How the ALC Evolves", h1_style))
    elements.append(Paragraph(
        "The <b>Adaptive Learning Coordinator (ALC)</b> is the autonomous flight intelligence engine. "
        "Rather than relying on unverified formulas, it uses the <b>Three Cheap Fixes</b> to gate and unlock advanced capabilities:",
        body_style
    ))
    elements.append(Paragraph(
        "1. <b>Fix 1 (AS5600L Wing Encoders):</b> Delivers true physical wing deflection angle. Solves the 'two explanations' dilemma—if a maneuver fails, the system immediately knows whether the aerodynamics failed or the servo failed to deflect.<br/>"
        "2. <b>Fix 2 (TI INA226 Servo Shunt):</b> Measures actual electrical watts burned by the 16 servos. Allows the ALC to discover the most energy-efficient wing twist shape.<br/>"
        "3. <b>Fix 3 (TC-02 Static Thrust Stand):</b> The HX711 ground load cell test stand is directly linked into software. Experimental flap waveforms must prove net positive thrust and low current ripple on the ground before being unlocked in the air.",
        body_style
    ))

    alc_curriculum_text = """
    ╔═══════════════════════════════════════════════════════════════════════════════════════════════════╗
    ║                                ALSS CURRICULUM UNLOCK PROGRESSION                                 ║
    ╠═══════════════════════════════════════════════════════════════════════════════════════════════════╣
    ║  CLASS A CURRICULUM: Maiden Stability, Straight-Line Gliding & Steady Cruise                     ║
    ║  • Gated to quarantined modes: steady hover checks, low-amplitude flapping, glide polar audits   ║
    ║  • Requirement to advance: 5 consecutive sorties with composite performance score > 0.60         ║
    ║                                               ▼                                                   ║
    ║  CLASS B CURRICULUM: Tandem Scissor-Jet & Wake Capture (H-SPLIT-1, H-DSH-1, Climb Reflexes)      ║
    ║  • Activates alternating wing flapping (stations 2 & 4 flap while 1 & 3 glide to form a jet)     ║
    ║  • Requirement to advance: 15 consecutive sorties with composite performance score > 0.70        ║
    ║                                               ▼                                                   ║
    ║  CLASS C CURRICULUM: Full Biomimetic Mastery (Falcon Stoop & Zoom, 3-DOF Odonata Yaw Couples)     ║
    ║  • High-speed dive pullouts (FALCON-MORPH), zero-roll pure yaw pulses (H-88A/B), and perch flare. ║
    ╚═══════════════════════════════════════════════════════════════════════════════════════════════════╝
    """
    elements.append(Preformatted(alc_curriculum_text, code_style))

    elements.append(Paragraph("The 5 Evolving ALC Modalities", h2_style))
    elements.append(Paragraph(
        "After every landing, the ALC updates 5 background neural-statistical models stored in <code>/home/pi/knowledge/</code>: "
        "<br/>1. <b>Mechanical Wear Tracking:</b> Analyzes 50 Hz startup current waveforms. Detects hinge friction or spar slack before flight."
        "<br/>2. <b>Anomaly Archaeology:</b> Harvests unexpected turbulence spikes and builds compensatory wing reflexes."
        "<br/>3. <b>Regenerative Recovery:</b> Fine-tunes flap stroke deceleration timing to harvest back-EMF power into the battery."
        "<br/>4. <b>Energy Stewardship:</b> Optimizes Joules consumed per kilometer flown based on ambient air density and altitude."
        "<br/>5. <b>Electrical Bus Architect:</b> Balances electrical rail loads across the aircraft to prevent voltage sag.",
        body_style
    ))

    elements.append(Spacer(1, 10))

    # SECTION 7: GROUND STATION & RC INTEGRATION
    elements.append(Paragraph("7. Ground Station Laptop Setup & Pilot Radio Control", h1_style))
    elements.append(Paragraph(
        "<b>Ground Station Laptop Stack:</b> The ground control station runs on any macOS or Linux laptop without special hardware. "
        "<br/>1. Install dependencies: <code>pip3 install pymavlink websockets aiohttp pyserial</code>"
        "<br/>2. Plug the SiK Telemetry Radio USB dongle into the laptop."
        "<br/>3. Launch ground station services: <code>./start_gs.sh 192.168.7.1 --gemini</code>"
        "<br/>This script boots two services: <code>mco_ws_bridge.py</code> (bridges telemetry to port 5758) and <code>mco_gemini_proxy.py</code> (AI copilot). "
        "The pilot opens the interactive web flight deck (<code>ui/mco_ai_ground_station.jsx</code>) to monitor real-time wing phase diagrams, airspeed, Boyd Specific Excess Power (Ps), and sensor health.",
        body_style
    ))

    elements.append(Paragraph("RC Transmitter Mapping & The Unified Control Lever", h2_style))
    elements.append(Paragraph(
        "Piloting uses a standard RadioMaster TX16S transmitter communicating over 2.4 GHz to the FrSky Archer Plus RS receiver. "
        "The receiver outputs an SBUS serial stream to the Pixhawk, which transmits stick values to the Raspberry Pi at 921600 baud. "
        "Rather than flying individual wings, the pilot uses conventional controls mapped through the <b>Intent Arbitration Layer</b>:<br/>"
        "• <b>Channel 3 (Left Stick Vertical) — The Unified Control Lever:</b><br/>"
        "&nbsp;&nbsp;- <b>100% (Full Forward):</b> Sprint & Climb. Flap frequency rises to quarantined ceiling (2.0 Hz), forward sweep engaged.<br/>"
        "&nbsp;&nbsp;- <b>50% (Center Detent):</b> Low-Drag Cruise. Flap frequency 1.4 Hz, wing pitch held at +3.5° glide trim (9–11 m/s).<br/>"
        "&nbsp;&nbsp;- <b>0% (Full Aft):</b> Maximum Aerodynamic Airbrake. Flap motors stop and center; wings sweep back into a high-drag cup, bringing the aircraft to a walking speed on rollout without mechanical wheel brakes.<br/>"
        "• <b>Channels 1, 2, 4 (Roll, Pitch, Yaw):</b> Conventional stick feel. The flight computer automatically converts roll commands into differential flap amplitude, and yaw commands into differential sweep.<br/>"
        "• <b>Instant Pilot Override:</b> If the aircraft is performing an autonomous ALSS test card and the pilot deflects any stick by >5%, the flight computer instantly disengages the autopilot and returns full authority to the pilot.",
        body_style
    ))

    elements.append(Spacer(1, 14))
    elements.append(HRFlowable(width="100%", thickness=1.0, color=colors.HexColor("#CBD5E1"), spaceBefore=4, spaceAfter=8))
    elements.append(Paragraph("<b>Document Verification:</b> Certified ready for bench commissioning and maiden flight execution. All data cross-referenced with locked October 2026 airframe geometry, 3.67 kg mass budget, and Three Cheap Fixes hardware specifications.", callout_style))

    doc.build(elements, canvasmaker=NumberedCanvas)
    print(f"[SUCCESS] Authored publication-quality PDF: {filename}")

if __name__ == '__main__':
    target = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "docs", "mark1", "MCO-1D_Electronics_and_Systems_Guide.pdf"))
    build_pdf(target)
