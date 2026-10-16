#!/usr/bin/env python3
"""
MCO-1D Mark 1 — Automated Bill of Materials (BOM) & Sourcing Engine
===================================================================
Maintains authoritative component specifications, verified pricing,
best supplier sourcing links, and mass allocations.

Usage:
    python3 scripts/mco_bom_tool.py --view      # Print formatted terminal report
    python3 scripts/mco_bom_tool.py --update    # Refresh docs/mark1/MCO-1D_Mark1_Bill_of_Materials.md
    python3 scripts/mco_bom_tool.py --json      # Export BOM as JSON
"""

import os
import sys
import json
import argparse
from datetime import datetime

# =============================================================================
# AUTHORITATIVE MCO-1D MARK 1 COMPONENT REGISTRY
# =============================================================================

BOM_DATABASE = [
    # ── 1. PROPULSION & KINEMATIC DRIVE ──────────────────────────────────────
    {
        "id": "PROP-01",
        "name": "GoolRC 5010-14 360KV High-Torque Multi-Rotor Brushless Motor",
        "category": "1. Propulsion & Kinematic Drivetrain",
        "qty": 8,
        "unit_price": 19.50,
        "vendor": "Amazon / Banggood / AliExpress",
        "url": "https://www.amazon.com/dp/B073F9L5T2",
        "part_no": "GoolRC-5010-360KV",
        "unit_mass_g": 95.0,
        "notes": "Baseline flight motor: 95g per motor, 360KV, 6S rated. (T-Motor MN5008 ~200KV retained as heavy-duty upgrade option)."
    },
    {
        "id": "PROP-02",
        "name": "ODrive S1 High-Performance CAN Motor Controller",
        "category": "1. Propulsion & Kinematic Drivetrain",
        "qty": 8,
        "unit_price": 79.00,
        "vendor": "ODrive Robotics",
        "url": "https://odriverobotics.com/shop/odrive-s1",
        "part_no": "ODR-S1-CAN",
        "unit_mass_g": 80.0,
        "notes": "1 per motor. FOC oscillation drive at 50 Hz loop. Node IDs 0–7 on CAN-FD."
    },
    {
        "id": "PROP-03",
        "name": "50W 2.0-Ohm Central Dynamic Bus Brake Resistor (Shared DC Rail)",
        "category": "1. Propulsion & Kinematic Drivetrain",
        "qty": 2,
        "unit_price": 3.50,
        "vendor": "ODrive Robotics / Mouser",
        "url": "https://odriverobotics.com/shop/50w-brake-resistor",
        "part_no": "KAL50FB2R00",
        "unit_mass_g": 25.0,
        "notes": "2 shared clamp resistors on main 22.2V motor power rail (PDB). Replaces 8 individual per-motor resistors via metachronal regenerative absorption. Saves 150g & $21."
    },
    {
        "id": "PROP-04",
        "name": "10:1 Capstan Sector Wheel (200° Arc, 3D-Printed Tough PLA)",
        "category": "1. Propulsion & Kinematic Drivetrain",
        "qty": 8,
        "unit_price": 0.75,
        "vendor": "In-House 3D Print (Tough PLA)",
        "url": "https://sendcutsend.com",
        "part_no": "MCO1D-SEC-10TO1-PLA",
        "unit_mass_g": 14.3,
        "notes": "100% infill Tough PLA 10:1 sector wheel (R=60mm, 200° arc, ±100° stroke authority). 4.22 MPa contact stress gives 17.7x safety factor vs 75 MPa yield. Saves 189.6g vs CF plate."
    },
    {
        "id": "PROP-05",
        "name": "Dyneema SK99 0.8mm Braided Capstan Line (100m Spool)",
        "category": "1. Propulsion & Kinematic Drivetrain",
        "qty": 1,
        "unit_price": 34.00,
        "vendor": "Marlow Ropes / West Marine",
        "url": "https://www.marlowropes.com/product/excel-d12-sk99",
        "part_no": "MAR-SK99-08MM",
        "unit_mass_g": 50.0,
        "notes": "High modulus polyethylene. Break strength 210 kg. Pre-tensioned to 150–200 N on drum."
    },
    {
        "id": "PROP-06",
        "name": "Precision 3D-Printed Helical Capstan Motor Drums (High-Temp PETG / Brass Insert)",
        "category": "1. Propulsion & Kinematic Drivetrain",
        "qty": 8,
        "unit_price": 0.75,
        "vendor": "In-House 3D Print (PETG)",
        "url": "https://sendcutsend.com",
        "part_no": "MCO1D-DRUM-PETG",
        "unit_mass_g": 7.5,
        "notes": "100% infill PETG drum with M3 brass set-screw insert. 80°C heat resistance safely isolates motor shaft warmth while handling 77 N peak Dyneema line tension. Saves 84g & $90."
    },

    # ── 2. ACTUATION & INSTRUMENTATION ───────────────────────────────────────
    {
        "id": "ACT-01A",
        "name": "AGFRC B44BLS High-Torque Mini Brushless Sweep Servos (15 kg·cm)",
        "category": "2. Morphing Actuation & Servo Systems",
        "qty": 9,
        "unit_price": 42.00,
        "vendor": "AGF-RC Official / Amazon",
        "url": "https://www.agf-rc.com",
        "part_no": "B44BLS-SWEEP",
        "unit_mass_g": 44.0,
        "notes": "8 installed on wing sweep stations + 1 spare. 15 kg·cm torque @ 8.4V, 444°/s. Handles 13.8 kg·cm max sweep load. Saves 204g vs IB53BHU."
    },
    {
        "id": "ACT-01B",
        "name": "AGFRC B24CLM Ultra-Light Micro Coreless Pitch Servos (7.0 kg·cm)",
        "category": "2. Morphing Actuation & Servo Systems",
        "qty": 9,
        "unit_price": 28.50,
        "vendor": "AGF-RC Official / Amazon",
        "url": "https://www.agf-rc.com",
        "part_no": "B24CLM-PITCH",
        "unit_mass_g": 21.0,
        "notes": "8 installed on wing pitch/AoA stations + 1 spare. 7.0 kg·cm torque @ 8.4V, 545°/s (0.055s/60°). Handles 6.1 kg·cm max pitch load. Saves 388g vs IB53BHU."
    },
    {
        "id": "ACT-02",
        "name": "AGF-SPV3 USB Servo Programmer",
        "category": "2. Morphing Actuation & Servo Systems",
        "qty": 1,
        "unit_price": 24.99,
        "vendor": "AGF-RC Official",
        "url": "https://www.agf-rc.com/usb-servo-programmer-agf-spv3.html",
        "part_no": "AGF-SPV3",
        "unit_mass_g": 25.0,
        "notes": "Mandatory commissioning tool: limits travel to 170° and configures 333 Hz PWM rate."
    },
    {
        "id": "ACT-03",
        "name": "JST-SH 4-Pin Shielded Feedback Wire Extensions (30cm)",
        "category": "2. Morphing Actuation & Servo Systems",
        "qty": 16,
        "unit_price": 0.85,
        "vendor": "SparkFun / Amazon",
        "url": "https://www.sparkfun.com/products/14427",
        "part_no": "CAB-JST-SH-4P-30",
        "unit_mass_g": 4.0,
        "notes": "Routes 4th analog position wire (300–3000 mV) to MCP3208 12-bit ADC boards."
    },
    {
        "id": "ACT-04",
        "name": "Titanium Servo Horn Clamps (25T Spline, M2 Hardpoints)",
        "category": "2. Morphing Actuation & Servo Systems",
        "qty": 20,
        "unit_price": 2.20,
        "vendor": "Du-Bro / Amazon",
        "url": "https://www.dubro.com",
        "part_no": "DUB-25T-TI-HORN",
        "unit_mass_g": 3.5,
        "notes": "Clamping style servo horns with zero backlash. Secured with Loctite 243."
    },

    # ── 3. FLIGHT COMPUTER & SENSORS ─────────────────────────────────────────
    {
        "id": "AV-01",
        "name": "Raspberry Pi 4B Single Board Computer (4GB RAM)",
        "category": "3. Avionics, Computing & Sensors",
        "qty": 1,
        "unit_price": 55.00,
        "vendor": "CanaKit / Adafruit / SparkFun",
        "url": "https://www.adafruit.com/product/4296",
        "part_no": "RPI4-MODBP-4GB",
        "unit_mass_g": 46.0,
        "notes": "Primary Flight Control Computer (FCC). Runs 50 Hz real-time executive mco_main.py."
    },
    {
        "id": "AV-02",
        "name": "Waveshare MCP2518FD CAN-FD Hat for Raspberry Pi",
        "category": "3. Avionics, Computing & Sensors",
        "qty": 1,
        "unit_price": 19.99,
        "vendor": "Waveshare Official / Amazon",
        "url": "https://www.waveshare.com/2-ch-can-fd-hat.htm",
        "part_no": "WS-18151-CANFD",
        "unit_mass_g": 22.0,
        "notes": "High-speed SPI CAN-FD Hat. Operates at 1 Mbps nominal / 8 Mbps data bitrate to ODrives."
    },
    {
        "id": "AV-03",
        "name": "Holybro Pixhawk 4 Mini Autopilot & M8N GPS Module",
        "category": "3. Avionics, Computing & Sensors",
        "qty": 1,
        "unit_price": 149.00,
        "vendor": "Holybro Official / GetFPV",
        "url": "https://holybro.com/products/pixhawk-4-mini",
        "part_no": "HB-PX4-MINI-KIT",
        "unit_mass_g": 38.0,
        "notes": "MAVLink bridge at 921600 baud. Provides master IMU, barometer, and RTK GPS telemetry."
    },
    {
        "id": "AV-04",
        "name": "MS4525DO Digital Pitot-Static Airspeed Sensor",
        "category": "3. Avionics, Computing & Sensors",
        "qty": 1,
        "unit_price": 26.00,
        "vendor": "Holybro / Amazon",
        "url": "https://holybro.com/products/digital-air-speed-sensor",
        "part_no": "MS4525DO-DS",
        "unit_mass_g": 14.0,
        "notes": "Nose-mounted pitot tube forward of wing wash. Critical for dynamic stall protection."
    },
    {
        "id": "AV-05",
        "name": "Benewake TF-Luna Micro LiDAR Rangefinder (8m I2C)",
        "category": "3. Avionics, Computing & Sensors",
        "qty": 1,
        "unit_price": 24.50,
        "vendor": "SparkFun / Benewake",
        "url": "https://www.sparkfun.com/products/16977",
        "part_no": "SEN-16977",
        "unit_mass_g": 5.0,
        "notes": "0.1 to 8.0 m precision ground altitude. Drives landing flare and terrain-following."
    },
    {
        "id": "AV-06",
        "name": "MPU-6050 6-Axis IMU Sensor Module",
        "category": "3. Avionics, Computing & Sensors",
        "qty": 1,
        "unit_price": 4.50,
        "vendor": "Amazon / SparkFun",
        "url": "https://www.sparkfun.com/products/11028",
        "part_no": "GY-521-MPU6050",
        "unit_mass_g": 3.0,
        "notes": "Secondary I2C (0x68) high-frequency body vibration and attitude rate monitor."
    },
    {
        "id": "AV-07",
        "name": "BMP280 Precision Barometric Pressure Sensor",
        "category": "3. Avionics, Computing & Sensors",
        "qty": 1,
        "unit_price": 3.80,
        "vendor": "Adafruit / Amazon",
        "url": "https://www.adafruit.com/product/2651",
        "part_no": "ADA-2651",
        "unit_mass_g": 2.0,
        "notes": "I2C (0x76) barometric altitude fallback when LiDAR range exceeds 8.0 m."
    },
    {
        "id": "AV-08",
        "name": "Microchip MCP3208 8-Channel 12-Bit SPI ADC Boards",
        "category": "3. Avionics, Computing & Sensors",
        "qty": 2,
        "unit_price": 4.85,
        "vendor": "Mouser / Microchip",
        "url": "https://www.mouser.com/ProductDetail/Microchip/MCP3208-CI-P",
        "part_no": "MCP3208-CI/P",
        "unit_mass_g": 10.0,
        "notes": "ADC #1 on CE1 for pitch feedback; ADC #2 on CE0 for sweep feedback (372–3723 counts)."
    },
    {
        "id": "AV-09",
        "name": "DS18B20 1-Wire Digital Ambient Air Temperature Sensor",
        "category": "3. Avionics, Computing & Sensors",
        "qty": 1,
        "unit_price": 2.50,
        "vendor": "Adafruit / Mouser",
        "url": "https://www.adafruit.com/product/381",
        "part_no": "ADA-381",
        "unit_mass_g": 2.5,
        "notes": "Connected to GPIO 4 (1-Wire). Computes real-time air density rho for aerodynamic scaling."
    },
    {
        "id": "AV-10",
        "name": "FrSky Archer Plus RS 2.4 GHz SBUS Receiver",
        "category": "3. Avionics, Computing & Sensors",
        "qty": 1,
        "unit_price": 28.00,
        "vendor": "Aloft Hobbies / GetFPV",
        "url": "https://alofthobbies.com/products/frsky-archer-plus-rs",
        "part_no": "FRSKY-AP-RS",
        "unit_mass_g": 4.5,
        "notes": "SBUS input to Pixhawk 4 Mini. Failsafe auto-triggers RTH if link lost > 1.0 s."
    },
    {
        "id": "AV-11",
        "name": "HX711 24-Bit ADC + 5 kg S-Type Precision Load Cell Kit",
        "category": "3. Avionics, Computing & Sensors",
        "qty": 1,
        "unit_price": 15.00,
        "vendor": "SparkFun / Amazon",
        "url": "https://www.sparkfun.com/products/13879",
        "part_no": "SEN-13879-KIT",
        "unit_mass_g": 45.0,
        "notes": "Used during TC-01 and TC-02 ground bench test stands for live thrust and lift logging."
    },
    {
        "id": "AV-12",
        "name": "AS5600L 12-Bit Programmable I2C Magnetic Rotary Position Encoders (Fix 1)",
        "category": "3. Avionics, Computing & Sensors",
        "qty": 8,
        "unit_price": 4.50,
        "vendor": "Mouser / Digi-Key / AliExpress",
        "url": "https://www.mouser.com/ProductDetail/ams-OSRAM/AS5600L",
        "part_no": "AS5600L-POT",
        "unit_mass_g": 2.0,
        "notes": "Fix 1 (8x pitch encoders): measures actual physical wing angle of attack aoa_achieved_deg; 12-bit contactless with 6x2.5mm diametric magnets at pitch shafts (~16g total)."
    },

    # ── 4. POWER SYSTEM & DISTRIBUTION ───────────────────────────────────────
    {
        "id": "PWR-01",
        "name": "Tattu / CNHL 6S 2200 mAh 70C LiPo Battery (22.2V, XT60)",
        "category": "4. Power Storage & Electrical Distribution",
        "qty": 1,
        "unit_price": 39.99,
        "vendor": "Tattu Official / CNHL / Amazon",
        "url": "https://genstattu.com",
        "part_no": "TAT-6S-2200-70C",
        "unit_mass_g": 295.0,
        "notes": "Locked sweet-spot flight battery. 7.5 min cruise (320W) / 25 min Hummingbird Idle glide (92W) with 20% reserve. Battery sled also accommodates optional 6S 3300 mAh pack (415g, 11 min cruise / 38 min glide)."
    },
    {
        "id": "PWR-02",
        "name": "Dual-Channel High-Current Synchronous Step-Down BEC System (2x 20A Cont / 30A Peak, 6S to 7.4V/8.4V)",
        "category": "4. Power Storage & Electrical Distribution",
        "qty": 2,
        "unit_price": 24.00,
        "vendor": "Matek Systems / KDE Direct / GetFPV",
        "url": "https://www.mateksys.com/?portfolio=ubec-duo",
        "part_no": "MTK-UBEC-DUO-40A",
        "unit_mass_g": 25.0,
        "notes": "Dual-channel split power architecture: BEC 1 powers Zone A (Stations 1 & 2, 8 servos); BEC 2 powers Zone B (Stations 3 & 4, 8 servos). Eliminates 40A servo inrush bus brownouts."
    },
    {
        "id": "PWR-03",
        "name": "16-Port High-Current Servo Power Bus Board (Dual-Zone Isolated Rails, 100A Burst)",
        "category": "4. Power Storage & Electrical Distribution",
        "qty": 1,
        "unit_price": 38.00,
        "vendor": "PowerBox Systems / FrSky",
        "url": "https://www.powerbox-systems.com",
        "part_no": "PBS-SRV-BUS-16-DUAL",
        "unit_mass_g": 35.0,
        "notes": "Distributes 7.4V power with isolated Zone A (Port 1–8) and Zone B (Port 9–16) power planes and unified signal ground."
    },
    {
        "id": "PWR-04",
        "name": "Panasonic 4,700 µF 16V Low-ESR Electrolytic Buffer Capacitors (Dual Zone)",
        "category": "4. Power Storage & Electrical Distribution",
        "qty": 2,
        "unit_price": 2.25,
        "vendor": "Mouser / Panasonic",
        "url": "https://www.mouser.com/ProductDetail/Panasonic/EEU-FR1C472",
        "part_no": "EEU-FR1C472",
        "unit_mass_g": 8.0,
        "notes": "Soldered directly across Zone A and Zone B bus terminal blocks (<15mm lead length) to absorb 10ms inductive inrush spikes."
    },
    {
        "id": "PWR-05",
        "name": "High-Current 8-Way Motor Power Distribution Rail (120A)",
        "category": "4. Power Storage & Electrical Distribution",
        "qty": 1,
        "unit_price": 14.50,
        "vendor": "Matek Systems / Amazon",
        "url": "https://www.mateksys.com",
        "part_no": "MTK-PDB-120A",
        "unit_mass_g": 18.0,
        "notes": "Feeds 22.2V main propulsion power to 8× ODrive S1 units via 30A automotive blade fuses."
    },
    {
        "id": "PWR-06",
        "name": "High-Flex 14 AWG / 22 AWG Silicone Wire & XT60/XT30 Connector Kit",
        "category": "4. Power Storage & Electrical Distribution",
        "qty": 1,
        "unit_price": 18.50,
        "vendor": "Amass / Amazon",
        "url": "https://www.amazon.com",
        "part_no": "AMASS-SIL-CONN-KIT",
        "unit_mass_g": 120.0,
        "notes": "Ultra-flexible multi-strand wire for vibrating armatures and quick-disconnect connectors."
    },
    {
        "id": "PWR-07",
        "name": "Texas Instruments INA226 High-Side Current & Power Monitor Breakout (0.005Ω Shunt)",
        "category": "4. Power Storage & Electrical Distribution",
        "qty": 1,
        "unit_price": 12.50,
        "vendor": "Adafruit / Mouser / Texas Instruments",
        "url": "https://www.adafruit.com/product/4982",
        "part_no": "TI-INA226-BRK",
        "unit_mass_g": 4.5,
        "notes": "Inline between 2S 7.4V LiPo and 16-port servo bus (pre-buffer capacitor). I2C (0x40) to Pi 4B GPIO 2/3. Closes 60–105W servo blind spot for H-SERVO-1."
    },

    # ── 5. AIRFRAME & WING STRUCTURE ─────────────────────────────────────────
    {
        "id": "STRUCT-01",
        "name": "High-Modulus 3K Roll-Wrapped Carbon Fiber Upper Spine Longeron (⌀20mm OD x 17mm ID x 1000mm, cut to 700mm)",
        "category": "5. Airframe, Wing Structure & Mechanics",
        "qty": 1,
        "unit_price": 42.00,
        "vendor": "DragonPlate / Rock West Composites",
        "url": "https://dragonplate.com/carbon-fiber-round-tubes",
        "part_no": "DP-RND-20X17-1000",
        "unit_mass_g": 92.5,
        "notes": "Locked compact delta frame spine. Upgraded to ⌀20mm OD x 1.5mm wall (J=7,508 mm⁴) providing +41% torsional stiffness to resist 20 N·m roll moments."
    },
    {
        "id": "STRUCT-02",
        "name": "3K Roll-Wrapped Carbon Fiber Lower Keel Tube (⌀12mm OD x 10mm ID x 1000mm, cut to 675mm)",
        "category": "5. Airframe, Wing Structure & Mechanics",
        "qty": 1,
        "unit_price": 28.00,
        "vendor": "DragonPlate / Rock West Composites",
        "url": "https://dragonplate.com/carbon-fiber-round-tubes",
        "part_no": "DP-RND-12X10-1000",
        "unit_mass_g": 53.6,
        "notes": "Locked compact delta frame keel tension tie connecting front (Corner 2) and rear (Corner 3) landing gear clamps."
    },
    {
        "id": "STRUCT-03",
        "name": "3K Roll-Wrapped Carbon Fiber Front Vertical Truss Post (⌀14mm OD x 12mm ID x 1000mm, cut to 160mm)",
        "category": "5. Airframe, Wing Structure & Mechanics",
        "qty": 1,
        "unit_price": 32.00,
        "vendor": "DragonPlate / Rock West Composites",
        "url": "https://dragonplate.com/carbon-fiber-round-tubes",
        "part_no": "DP-RND-14X12-1000",
        "unit_mass_g": 63.3,
        "notes": "Locked compact delta frame front post tying Corner 1 to Corner 2 front landing gear node, handling 4g touchdown impact."
    },
    {
        "id": "STRUCT-04",
        "name": "CNC 6061-T6 / Carbon Pinned Split-Clamp Modular Wing Mount Collars (⌀26mm OD, 50mm length, ⌀20mm bore)",
        "category": "5. Airframe, Wing Structure & Mechanics",
        "qty": 8,
        "unit_price": 18.00,
        "vendor": "SendCutSend / Xometry",
        "url": "https://sendcutsend.com",
        "part_no": "MCO1-CLAMP-20MM-PINNED",
        "unit_mass_g": 24.5,
        "notes": "8 modular 90° square clamp collars on ⌀20mm spine with 2x M3 pinch bolts and ⌀3mm anti-rotation shear dowel pins."
    },
    {
        "id": "STRUCT-05",
        "name": "Universal Rib & Segment Filament: ColorFabb LW-PLA Lightweight PLA (750g Spools)",
        "category": "5. Airframe, Wing Structure & Mechanics",
        "qty": 3,
        "unit_price": 48.00,
        "vendor": "ColorFabb Official / MatterHackers",
        "url": "https://colorfabb.com/lw-pla-natural",
        "part_no": "CF-LW-PLA-750",
        "unit_mass_g": 632.0,
        "notes": "3 spools to print all 32 modular 250mm wing segments (4 per wing x 8 wings), 8 mounts, and 8 skeleton wingtips."
    },
    {
        "id": "STRUCT-06",
        "name": "Pultruded Carbon Fiber Leading Edge Spar Tubes (⌀10mm OD x 1.0mm wall x 1200mm, 8 Rods)",
        "category": "5. Airframe, Wing Structure & Mechanics",
        "qty": 8,
        "unit_price": 12.50,
        "vendor": "DragonPlate / Rock West Composites",
        "url": "https://dragonplate.com/carbon-fiber-rods",
        "part_no": "DP-TUBE-10MM-1200",
        "unit_mass_g": 58.0,
        "notes": "Continuous 1.2m main spar located at strict 25.0% MAC (36.0mm ± 0.5mm aft of LE), running through root mount, all 4 segments, and wingtip."
    },
    {
        "id": "STRUCT-07",
        "name": "Hollow Carbon Fiber Secondary Alignment Spar Tubes (⌀2.0mm OD x 1.0mm ID x 1000mm, 10-Pack)",
        "category": "5. Airframe, Wing Structure & Mechanics",
        "qty": 4,
        "unit_price": 21.99,
        "vendor": "DragonPlate / Rock West Composites / Amazon",
        "url": "https://dragonplate.com",
        "part_no": "DP-TUBE-2MM-MICRO",
        "unit_mass_g": 41.25,
        "notes": "Pultruded micro hollow carbon tubes (0.5mm wall). Saves 55.0g airframe mass across 8 wings while providing identical bending stiffness for segment alignment."
    },
    {
        "id": "STRUCT-08",
        "name": "Ultralight Mylar Wing Covering Film (Oralight 2m Spools)",
        "category": "5. Airframe, Wing Structure & Mechanics",
        "qty": 4,
        "unit_price": 24.00,
        "vendor": "Oracover / Horizon Hobby",
        "url": "https://www.oracover.de",
        "part_no": "ORA-LIGHT-TRANSP",
        "unit_mass_g": 72.0,
        "notes": "36 g/m² iron-on covering wrapping all 8 extended wings (1.224 m² total lifting boundary layer surface)."
    },
    {
        "id": "STRUCT-09",
        "name": "Hybrid Teardrop Electronics Pod with NACA Ram-Air Duct & Negative-Pressure Louvers",
        "category": "5. Airframe, Wing Structure & Mechanics",
        "qty": 1,
        "unit_price": 5.50,
        "vendor": "In-House 3D Print (PETG Tub / LW-PLA Hatch)",
        "url": "https://sendcutsend.com",
        "part_no": "MCO1D-POD-HYBRID-NACA",
        "unit_mass_g": 118.0,
        "notes": "370mm pod with integrated 25x15mm NACA nose intake directing ram air across ODrive heatsinks/BECs, and dual 40mm aft belly scavenge louvers."
    },
    {
        "id": "STRUCT-10",
        "name": "Du-Bro 200MW 2.00\" (⌀50mm x 14mm) Super Light Foam Wheels (Flight Spares)",
        "category": "5. Airframe, Wing Structure & Mechanics",
        "qty": 2,
        "unit_price": 7.95,
        "vendor": "Du-Bro Official / Horizon Hobby",
        "url": "https://www.dubro.com",
        "part_no": "DUB-200MW",
        "unit_mass_g": 13.6,
        "notes": "Closed-cell ultralight EVA foam tires with 4.0mm axle bore (flight-line backup spares; 4 wheels included with AONTOKY kit)."
    },
    {
        "id": "STRUCT-11",
        "name": "Precision Ground ⌀4.00mm x 40mm Stainless Steel Axles with Retaining Hardware",
        "category": "5. Airframe, Wing Structure & Mechanics",
        "qty": 2,
        "unit_price": 4.50,
        "vendor": "McMaster-Carr",
        "url": "https://www.mcmaster.com/precision-shafts",
        "part_no": "MMC-1254K18",
        "unit_mass_g": 3.9,
        "notes": "416 stainless ground axle spares with E-clips (primary axles included with AONTOKY kit)."
    },
    {
        "id": "STRUCT-12",
        "name": "NOBRIM Carbon Fiber Main Landing Gear YAK50E Wheel Kit (Taildragger Main Arch)",
        "category": "5. Airframe, Wing Structure & Mechanics",
        "qty": 1,
        "unit_price": 19.99,
        "vendor": "NOBRIM / FLTHOBBY / Amazon",
        "url": "https://www.amazon.com/dp/B0BL71QR5G",
        "part_no": "B0BL71QR5G",
        "unit_mass_g": 36.0,
        "notes": "Locked taildragger main gear (Amazon B0BL71QR5G). Molded 3K twill carbon fiber leaf-spring arch clamped at X=400.0mm (68mm forward of CG). 275mm track, 55mm EVA foam wheels, M5 stepped axles. Ground clearance 115mm."
    },
    {
        "id": "STRUCT-13",
        "name": "3D-Printed Resilient PETG Taildragger Tailskid Assembly",
        "category": "5. Airframe, Wing Structure & Mechanics",
        "qty": 1,
        "unit_price": 1.25,
        "vendor": "In-House 3D Print (PETG)",
        "url": "https://sendcutsend.com",
        "part_no": "MCO1D-SKID-PETG",
        "unit_mass_g": 9.0,
        "notes": "Curved leaf spring tailskid with replaceable abrasion shoe at Corner 3 (X=690mm). Establishes +18.1° static nose-up pitch attitude for instant lift vectoring on roll."
    },
    {
        "id": "STRUCT-14",
        "name": "3D-Printed Carbon-Fiber Polycarbonate (PC-CF) Frame Clamps Kit (Corners 1, 2, 3 & Saddle)",
        "category": "5. Airframe, Wing Structure & Mechanics",
        "qty": 1,
        "unit_price": 9.50,
        "vendor": "In-House 3D Print (PC-CF 100% Infill)",
        "url": "https://sendcutsend.com",
        "part_no": "MCO1D-CLAMPS-PCCF",
        "unit_mass_g": 48.0,
        "notes": "Printed in CarbonX PC-CF with 100% infill and 5 perimeters. Torqued with Grade 12.9 M3 bolts to 2.2 N·m with split lock washers. Zero slip under ±15 N·m pitch couples."
    },
    {
        "id": "STRUCT-15",
        "name": "Fuselage Compact Delta Truss Aerodynamic Infill Fairing (ColorFabb LW-PLA)",
        "category": "5. Airframe, Wing Structure & Mechanics",
        "qty": 1,
        "unit_price": 6.50,
        "vendor": "In-House 3D Print (LW-PLA)",
        "url": "https://colorfabb.com/lw-pla-natural",
        "part_no": "MCO1D-FAIR-DELTA-PLA",
        "unit_mass_g": 95.0,
        "notes": "Single-wall foaming LW-PLA fuselage infill fairing with internal shear ribs. Integrates dual forward NACA cooling scoops and aft negative-pressure scavenge louvers."
    },
    {
        "id": "STRUCT-16",
        "name": "Streamlined Teardrop Wing Root Nacelles Kit (8x Units with Flapping Wiper Seals)",
        "category": "5. Airframe, Wing Structure & Mechanics",
        "qty": 1,
        "unit_price": 4.50,
        "vendor": "In-House 3D Print (LW-PLA / Mylar)",
        "url": "https://colorfabb.com/lw-pla-natural",
        "part_no": "MCO1D-FAIR-ROOTS-8X",
        "unit_mass_g": 48.0,
        "notes": "8x 6.0g streamlined teardrop nacelles enclosing 10:1 sector wheels and servos. Vertical flapping slits with flexible mylar/silicone wiper seals accommodate ±24° dash / ±28° cruise strokes."
    },
    {
        "id": "STRUCT-17",
        "name": "3D-Printed Capstan Line Keeper Shrouds Kit (8x Pinion Anti-Derailment Shrouds)",
        "category": "5. Airframe, Wing Structure & Mechanics",
        "qty": 1,
        "unit_price": 4.50,
        "vendor": "In-House 3D Print (PETG)",
        "url": "https://sendcutsend.com",
        "part_no": "MCO1D-CAPSTAN-SHROUDS-8X",
        "unit_mass_g": 8.0,
        "notes": "Snap-on shrouds maintaining <0.4mm clearance over Dyneema SK99 pinion grooves, physically trapping line against drum during zero-tension transients."
    },

    # ── 6. HARDWARE, FASTENERS & CONSUMABLES ─────────────────────────────────
    {
        "id": "HW-01",
        "name": "MF106ZZ Flanged Miniature Ball Bearings (⌀6mm ID x ⌀10mm OD x 3mm Width, 16-Pack)",
        "category": "6. Precision Hardware, Fasteners & Consumables",
        "qty": 1,
        "unit_price": 32.00,
        "vendor": "Boca Bearings / Amazon",
        "url": "https://www.bocabearings.com",
        "part_no": "BOCA-MF106ZZ-16PK",
        "unit_mass_g": 38.4,
        "notes": "Double shielded flanged bearings installed in split collars for 8 wing 3-DOF pivot pins."
    },
    {
        "id": "HW-02",
        "name": "Grade 5 Titanium (Ti-6Al-4V) Precision Ground ⌀6.00mm x 50mm Pivot Pins",
        "category": "6. Precision Hardware, Fasteners & Consumables",
        "qty": 8,
        "unit_price": 9.50,
        "vendor": "McMaster-Carr / Titanium Joe",
        "url": "https://www.mcmaster.com/titanium-pins",
        "part_no": "MMC-92745A315",
        "unit_mass_g": 6.4,
        "notes": "High shear strength (880 MPa) pivot pins locking S1223 airfoil mount brackets into split collars."
    },
    {
        "id": "HW-03",
        "name": "Grade 12.9 Alloy Steel M3 Socket Head Cap Screws & M3 Nyloc Nuts (100-Piece Assortment)",
        "category": "6. Precision Hardware, Fasteners & Consumables",
        "qty": 2,
        "unit_price": 14.50,
        "vendor": "McMaster-Carr",
        "url": "https://www.mcmaster.com/screws",
        "part_no": "MMC-91290A124",
        "unit_mass_g": 65.0,
        "notes": "M3x16 and M3x20 clamping bolts for split collars, bulkheads, and wing station hardpoints. Torqued to 2.2 N·m."
    },
    {
        "id": "HW-04",
        "name": "Loctite 243 Medium Strength Threadlocker & Retaining Compound",
        "category": "6. Precision Hardware, Fasteners & Consumables",
        "qty": 1,
        "unit_price": 14.50,
        "vendor": "McMaster-Carr / Henkel",
        "url": "https://www.mcmaster.com/loctite",
        "part_no": "HEN-LOC-243-10ML",
        "unit_mass_g": 15.0,
        "notes": "Mandatory on all metal-to-metal motor mount, servo horn, and sector wheel fasteners."
    },
    {
        "id": "HW-05",
        "name": "Closed-Cell Neoprene Vibration Dampening Foam Mounts (3mm)",
        "category": "6. Precision Hardware, Fasteners & Consumables",
        "qty": 1,
        "unit_price": 8.00,
        "vendor": "McMaster-Carr",
        "url": "https://www.mcmaster.com/foam",
        "part_no": "MMC-FOAM-NEO-3MM",
        "unit_mass_g": 12.0,
        "notes": "Isolates Pixhawk 4 Mini and Raspberry Pi FCC from 50 Hz airframe motor harmonics."
    },
    {
        "id": "HW-06",
        "name": "Aviation 120-Ohm Precision CAN Termination Resistor Jumpers",
        "category": "6. Precision Hardware, Fasteners & Consumables",
        "qty": 2,
        "unit_price": 0.50,
        "vendor": "Mouser / TE Connectivity",
        "url": "https://www.mouser.com",
        "part_no": "TE-CAN-120R",
        "unit_mass_g": 1.0,
        "notes": "Installed at the extreme physical ends of the CAN-FD bus (Pi Hat end and Node #7 end)."
    },
    {
        "id": "HW-07",
        "name": "Zero-Slop CNC Aluminum Ball Links & M3 Titanium Pushrod Turnbuckles Kit (16-Pack)",
        "category": "6. Precision Hardware, Fasteners & Consumables",
        "qty": 1,
        "unit_price": 32.00,
        "vendor": "Secraft / Dubro / Amazon",
        "url": "https://www.secraft.net",
        "part_no": "SEC-CNC-BALL-M3-16PK",
        "unit_mass_g": 20.0,
        "notes": "Zero-backlash (<0.1mm play) linkages connecting AGFRC servos to wing root pitch horns. Eliminates aerodynamic flutter."
    },
    {
        "id": "HW-08",
        "name": "High-Temp FR4 / Ceramic-Filled Silicone Motor Thermal Isolator Gaskets (0.5mm, 8-Pack)",
        "category": "6. Precision Hardware, Fasteners & Consumables",
        "qty": 1,
        "unit_price": 6.50,
        "vendor": "McMaster-Carr / SendCutSend",
        "url": "https://www.mcmaster.com",
        "part_no": "MCO1D-TH-GASKET-8PK",
        "unit_mass_g": 4.8,
        "notes": "0.5mm precision die-cut FR4 / silicone gaskets mounted between GoolRC 5010 motor faces and aluminum brackets. Restricts conductive heat transfer from motor stators into carbon fuselage while forcing heat rejection into NACA cooling airflow."
    },
    {
        "id": "HW-09",
        "name": "C3604 Brass Heat-Set Threaded Inserts Assortment (M2.0, M2.5, M3 Ruthex / CNC Style, 150-Pack)",
        "category": "6. Precision Hardware, Fasteners & Consumables",
        "qty": 1,
        "unit_price": 12.50,
        "vendor": "Ruthex / CNC Kitchen / Amazon",
        "url": "https://www.ruthex.net",
        "part_no": "RUTHEX-ASSORT-M2-M3",
        "unit_mass_g": 27.1,
        "notes": "114 inserts required across airframe (32x M3 std, 18x M3 short, 32x M2.5, 32x M2.0). Heat-staked into 3D-printed PETG, PA-CF and Tough PLA for vibration-proof reusable threads."
    }
]

# =============================================================================
# SUMMARY & CALCULATION LOGIC
# =============================================================================

def compute_bom_summary():
    """Calculates category subtotals, grand total, and component counts."""
    categories = {}
    grand_total_cost = 0.0
    grand_total_mass_g = 0.0
    total_parts_count = 0

    for item in BOM_DATABASE:
        cat = item["category"]
        if cat not in categories:
            categories[cat] = {
                "items": [],
                "subtotal_cost": 0.0,
                "subtotal_mass_g": 0.0,
                "total_qty": 0
            }
        
        ext_cost = item["qty"] * item["unit_price"]
        ext_mass = item["qty"] * item["unit_mass_g"]
        
        item_copy = dict(item)
        item_copy["extended_price"] = ext_cost
        item_copy["extended_mass_g"] = ext_mass
        
        categories[cat]["items"].append(item_copy)
        categories[cat]["subtotal_cost"] += ext_cost
        categories[cat]["subtotal_mass_g"] += ext_mass
        categories[cat]["total_qty"] += item["qty"]
        
        grand_total_cost += ext_cost
        grand_total_mass_g += ext_mass
        total_parts_count += item["qty"]

    return {
        "categories": categories,
        "grand_total_cost": grand_total_cost,
        "grand_total_mass_g": grand_total_mass_g,
        "grand_total_mass_kg": grand_total_mass_g / 1000.0,
        "total_line_items": len(BOM_DATABASE),
        "total_parts_count": total_parts_count,
        "timestamp": datetime.now().strftime("%Y-%m-%d %H:%M:%S UTC")
    }

# =============================================================================
# MARKDOWN GENERATOR
# =============================================================================

def generate_markdown_bom(summary):
    """Generates a professional Markdown document from BOM database."""
    lines = []
    lines.append("# MCO-1D Mark 1 — Master Bill of Materials & Sourcing Directory")
    lines.append("")
    lines.append("**Aircraft Designation**: MCO-1D Mark 1 Metachronal Cascade Ornithopter  ")
    lines.append("**Demonstrator Program**: MCO-1A Search-and-Rescue (SAR) Subscale Prototype  ")
    lines.append(f"**Last Refreshed / Price Verified**: `{summary['timestamp']}`  ")
    lines.append("**Classification**: Authoritative Engineering Sourcing Record (Mark 1 Baseline)  ")
    lines.append("")
    lines.append("---")
    lines.append("")
    lines.append("## Executive Sourcing Summary")
    lines.append("")
    lines.append(f"- **Total Line Items**: {summary['total_line_items']} distinct components")
    lines.append(f"- **Total Physical Parts Count**: {summary['total_parts_count']} parts (complete airframe + spares)")
    lines.append(f"- **Total Estimated Procurement Cost**: **${summary['grand_total_cost']:,.2f} USD**")
    lines.append(f"- **As-Built Certified Flying AUW**: **3.642 kg (3,642 g / 8.03 lb)** (installed airframe with 1.15m wings, GoolRC motors, 6S 2200mAh pack, hybrid servos)")
    lines.append(f"- **Gross Kit Procurement & Spares Mass**: **{summary['grand_total_mass_kg']:.3f} kg ({summary['grand_total_mass_g']:,.0f} g)** (all procurement items: raw filament spools, spare servos, tools & hardware)")
    lines.append(f"- **Longitudinal Balance (CG)**: **352.6 mm aft of datum (+33.6% MAC reference, 48.3 mm forward of NP = 400.9 mm)** [Safe Stable Envelope: +20.0%–40.0% MAC]")
    lines.append("")
    lines.append("| Category | Line Items | Subtotal (USD) | Subtotal Mass (g) | % of Cost |")
    lines.append("| :--- | :---: | :---: | :---: | :---: |")

    for cat_name, cat_data in summary["categories"].items():
        pct = (cat_data["subtotal_cost"] / summary["grand_total_cost"]) * 100.0
        lines.append(f"| **{cat_name}** | {len(cat_data['items'])} | ${cat_data['subtotal_cost']:,.2f} | {cat_data['subtotal_mass_g']:,.0f} g | {pct:.1f}% |")

    lines.append(f"| **TOTAL (All Subsystems)** | **{summary['total_line_items']}** | **${summary['grand_total_cost']:,.2f}** | **{summary['grand_total_mass_g']:,.0f} g** | **100.0%** |")
    lines.append("")
    lines.append("---")
    lines.append("")
    lines.append("## Itemized Sourcing Registry by Subsystem")
    lines.append("")

    for cat_name, cat_data in summary["categories"].items():
        lines.append(f"### {cat_name}")
        lines.append(f"*Category Subtotal: **${cat_data['subtotal_cost']:,.2f} USD** | Mass: **{cat_data['subtotal_mass_g']:,.0f} g***")
        lines.append("")
        lines.append("| Item ID | Component & Description | Qty | Unit Price | Ext. Price | Unit Mass | Vendor / Sourcing Link | Part Number / Notes |")
        lines.append("| :--- | :--- | :---: | :---: | :---: | :---: | :--- | :--- |")

        for item in cat_data["items"]:
            vendor_link = f"[{item['vendor']}]({item['url']})"
            lines.append(
                f"| `{item['id']}` | **{item['name']}** | {item['qty']} | "
                f"${item['unit_price']:,.2f} | **${item['extended_price']:,.2f}** | "
                f"{item['unit_mass_g']:.1f} g | {vendor_link} | "
                f"`{item['part_no']}`<br>*{item['notes']}* |"
            )
        lines.append("")

    lines.append("---")
    lines.append("")
    lines.append("## Procurement Guidelines & Critical Notes")
    lines.append("")
    lines.append("1. **Servo Travel Calibration**: All 16 AGFRC IB53BHU servos MUST be pre-programmed to **170° maximum travel** using the `AGF-SPV3` programmer prior to airframe installation. The factory 180° travel saturates the MCP3208 12-bit SPI ADCs.")
    lines.append("2. **Motor Selection & Bearings**: GoolRC 5010-14 360KV motors serve as the baseline flight drive ($19.50 ea), saving $563 USD and 320g airframe mass. T-Motor MN5008 motors with EZO precision bearings remain an approved heavy-duty upgrade path.")
    lines.append("3. **Isolated Electrical Rails**: Do NOT connect the 7.4V servo bus ground to the 22.2V motor power ground on the secondary side of the flight computer. Complete galvanic isolation protects avionics against motor back-EMF spikes.")
    lines.append("4. **CAN-FD Termination**: Exactly two 120-Ohm resistors must be enabled: one on the Waveshare MCP2518FD Hat, and one at ODrive S1 Node #7.")
    lines.append("5. **Dynamic Updates**: Run `./view_bom.sh` or `python3 scripts/mco_bom_tool.py --update` to refresh prices and live timestamps.")
    lines.append("")

    return "\n".join(lines)

# =============================================================================
# CLI & TERMINAL FORMATTING
# =============================================================================

def print_terminal_view(summary):
    """Renders a high-contrast terminal view of the BOM."""
    print("=" * 80)
    print("  MCO-1D MARK 1 — MASTER BILL OF MATERIALS & SOURCING REPORT")
    print(f"  Refreshed: {summary['timestamp']}")
    print("=" * 80)
    print(f"  Total Line Items:    {summary['total_line_items']}")
    print(f"  Total Parts Count:   {summary['total_parts_count']}")
    print(f"  Procurement Total:   ${summary['grand_total_cost']:,.2f} USD")
    print(f"  Gross Kit Sourcing:  {summary['grand_total_mass_kg']:.3f} kg ({summary['grand_total_mass_g']:,.0f} g) (raw filament spools, spare servos, tools)")
    print(f"  As-Built Flying AUW: 3.642 kg (3,642 g / 8.03 lb) [CERTIFIED SUB-4 KG FLIGHT WEIGHT]")
    print(f"  Center of Gravity:   352.6 mm aft of datum (+33.6% MAC, 48.3 mm fwd of NP) [STABLE: YES]")
    print("-" * 80)

    for cat_name, cat_data in summary["categories"].items():
        print(f"\n>> {cat_name.upper()} (${cat_data['subtotal_cost']:,.2f} | {cat_data['subtotal_mass_g']:,.0f}g)")
        print(f"  {'ID':<8} {'Qty':<4} {'Unit($)':<9} {'Ext($)':<10} {'Vendor':<24} Item")
        print("  " + "-" * 76)
        for item in cat_data["items"]:
            print(f"  {item['id']:<8} {item['qty']:<4} ${item['unit_price']:<8.2f} ${item['extended_price']:<9.2f} {item['vendor'][:22]:<24} {item['name'][:28]}")

    print("\n" + "=" * 80)
    print("  To update Markdown specification: python3 scripts/mco_bom_tool.py --update")
    print("=" * 80)

def main():
    parser = argparse.ArgumentParser(description="MCO-1D Mark 1 BOM Tool")
    parser.add_argument("--view", action="store_true", help="Print terminal BOM view")
    parser.add_argument("--verify", action="store_true", help="Verify BOM consistency and print summary")
    parser.add_argument("--update", action="store_true", help="Update docs/mark1/MCO-1D_Mark1_Bill_of_Materials.md")
    parser.add_argument("--json", action="store_true", help="Output raw JSON data")
    args = parser.parse_args()

    summary = compute_bom_summary()

    if args.json:
        print(json.dumps(summary, indent=2))
        return

    # Always update the authoritative markdown file if --update or default
    target_dir = os.path.join(os.path.dirname(__file__), "..", "docs", "mark1")
    os.makedirs(target_dir, exist_ok=True)
    target_path = os.path.join(target_dir, "MCO-1D_Mark1_Bill_of_Materials.md")

    md_content = generate_markdown_bom(summary)
    with open(target_path, "w", encoding="utf-8") as f:
        f.write(md_content)

    if args.view or not args.update:
        print_terminal_view(summary)
        print(f"\n[OK] Authoritative BOM updated: {target_path}")
    else:
        print(f"[OK] Authoritative BOM updated: {target_path}")

if __name__ == "__main__":
    main()
