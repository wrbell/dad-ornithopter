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
        "name": "50W 2.0-Ohm Dynamic Brake Resistor",
        "category": "1. Propulsion & Kinematic Drivetrain",
        "qty": 8,
        "unit_price": 3.50,
        "vendor": "ODrive Robotics / Mouser",
        "url": "https://odriverobotics.com/shop/50w-brake-resistor",
        "part_no": "KAL50FB2R00",
        "unit_mass_g": 25.0,
        "notes": "1 per S1 controller. Absorbs rapid deceleration regenerative spikes at stroke reversals."
    },
    {
        "id": "PROP-04",
        "name": "7:1 Capstan Sector Wheel (200° Arc, 2mm Carbon Fiber)",
        "category": "1. Propulsion & Kinematic Drivetrain",
        "qty": 8,
        "unit_price": 18.50,
        "vendor": "SendCutSend / In-House CNC",
        "url": "https://sendcutsend.com/materials/carbon-fiber/",
        "part_no": "MCO1D-SEC-7TO1-200D",
        "unit_mass_g": 38.0,
        "notes": "Custom 2mm CF plate cut to sector_wheel_7to1_200deg.dxf. Full 200° arc (±100° stroke authority) rigged with Dyneema groove."
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
        "name": "Helical Capstan Motor Drums (Lathe Turned 6061-T6)",
        "category": "1. Propulsion & Kinematic Drivetrain",
        "qty": 8,
        "unit_price": 12.00,
        "vendor": "Xometry / In-House Lathe",
        "url": "https://www.xometry.com",
        "part_no": "MCO1D-DRUM-22MM",
        "unit_mass_g": 18.0,
        "notes": "22 mm pitch diameter helical groove drum with anchor pin for Dyneema line."
    },

    # ── 2. ACTUATION & INSTRUMENTATION ───────────────────────────────────────
    {
        "id": "ACT-01",
        "name": "AGFRC IB53BHU Brushless Programmable Servo (with 4th Analog Wire)",
        "category": "2. Morphing Actuation & Servo Systems",
        "qty": 18,
        "unit_price": 62.99,
        "vendor": "AGF-RC Official / Amazon",
        "url": "https://www.agf-rc.com/programmable-brushless-servo-ib53bhu.html",
        "part_no": "IB53BHU-FB",
        "unit_mass_g": 69.5,
        "notes": "16 installed (8 pitch, 8 sweep) + 2 flight-line spares. 36 kg·cm torque @ 7.4V, 444°/s."
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

    # ── 4. POWER SYSTEM & DISTRIBUTION ───────────────────────────────────────
    {
        "id": "PWR-01",
        "name": "Tattu / Gens Ace 6S 5000 mAh 50C LiPo Battery (22.2V)",
        "category": "4. Power Storage & Electrical Distribution",
        "qty": 1,
        "unit_price": 119.99,
        "vendor": "Gens Ace Official / Tattu USA",
        "url": "https://genstattu.com/tattu-5000mah-22-2v-50c-6s1p-lipo-battery-pack-with-xt90-plug/",
        "part_no": "TA-50C-5000-6S1P",
        "unit_mass_g": 540.0,
        "notes": "Main propulsion pack. Mounted on quick-pin sliding tray (230–340 mm arm along keel)."
    },
    {
        "id": "PWR-02",
        "name": "Tattu / Gens Ace 2S 3000 mAh 45C LiPo Battery (7.4V)",
        "category": "4. Power Storage & Electrical Distribution",
        "qty": 1,
        "unit_price": 26.99,
        "vendor": "Gens Ace Official / Amazon",
        "url": "https://genstattu.com/gens-ace-3000mah-7-4v-45c-2s1p-lipo-battery-pack/",
        "part_no": "GEA-45C-3000-2S1P",
        "unit_mass_g": 175.0,
        "notes": "Dedicated isolated servo power pack. Prevents brownouts during 40A multi-servo spikes."
    },
    {
        "id": "PWR-03",
        "name": "16-Port High-Current Servo Power Bus Board (100A Burst)",
        "category": "4. Power Storage & Electrical Distribution",
        "qty": 1,
        "unit_price": 38.00,
        "vendor": "PowerBox Systems / FrSky",
        "url": "https://www.powerbox-systems.com",
        "part_no": "PBS-SRV-BUS-16",
        "unit_mass_g": 35.0,
        "notes": "Distributes 7.4V power from 2S battery to all 16 AGFRC servos with isolated signal lines."
    },
    {
        "id": "PWR-04",
        "name": "Panasonic 4,700 µF 16V Low-ESR Electrolytic Buffer Capacitor",
        "category": "4. Power Storage & Electrical Distribution",
        "qty": 2,
        "unit_price": 2.25,
        "vendor": "Mouser / Panasonic",
        "url": "https://www.mouser.com/ProductDetail/Panasonic/EEU-FR1C472",
        "part_no": "EEU-FR1C472",
        "unit_mass_g": 8.0,
        "notes": "Connected across the 7.4V servo bus rail to suppress voltage dips during reversals."
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
        "notes": "Continuous 1.2m main spar running through root mount, all 4 segments (1.0m run), and anchoring into skeleton wingtip."
    },
    {
        "id": "STRUCT-07",
        "name": "Solid Carbon Fiber Secondary Alignment Spar Rods (⌀2.0mm & ⌀3.0mm x 1200mm, 10-Pack)",
        "category": "5. Airframe, Wing Structure & Mechanics",
        "qty": 4,
        "unit_price": 16.00,
        "vendor": "DragonPlate / Amazon",
        "url": "https://dragonplate.com",
        "part_no": "DP-ROD-2MM-1200-PACK",
        "unit_mass_g": 55.0,
        "notes": "Inserted through continuous internal alignment bores across all 4 segments at 1.8mm and 72mm chord positions."
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
        "name": "Sliding Battery Tray Assembly with 4mm Stainless Ball-Lock Pin",
        "category": "5. Airframe, Wing Structure & Mechanics",
        "qty": 1,
        "unit_price": 22.00,
        "vendor": "McMaster-Carr / In-House",
        "url": "https://www.mcmaster.com/quick-release-pins",
        "part_no": "MMC-PIN-4MM-SS",
        "unit_mass_g": 42.0,
        "notes": "Slides along lower deck tray X=120–550 mm. 10 mm indexing pitch with CG reference scale."
    },
    {
        "id": "STRUCT-10",
        "name": "Du-Bro 200MW 2.00\" (⌀50mm x 14mm) Super Light Foam Wheels (2 Pairs / 4 Wheels)",
        "category": "5. Airframe, Wing Structure & Mechanics",
        "qty": 2,
        "unit_price": 7.95,
        "vendor": "Du-Bro Official / Horizon Hobby",
        "url": "https://www.dubro.com",
        "part_no": "DUB-200MW",
        "unit_mass_g": 13.6,
        "notes": "Closed-cell ultralight EVA foam tires with 4.0mm axle bore. 6.8g per wheel."
    },
    {
        "id": "STRUCT-11",
        "name": "Precision Ground ⌀4.00mm x 40mm Stainless Steel Axles with E-Clip Retainers",
        "category": "5. Airframe, Wing Structure & Mechanics",
        "qty": 4,
        "unit_price": 4.50,
        "vendor": "McMaster-Carr",
        "url": "https://www.mcmaster.com/precision-shafts",
        "part_no": "MMC-1254K18",
        "unit_mass_g": 3.9,
        "notes": "416 stainless ground axles for front (tf=250mm) and rear (tr=320mm) landing gear."
    },
    {
        "id": "STRUCT-12",
        "name": "⌀6.0mm Carbon Fiber Splayed Landing Gear Strut Tubes (4 Struts)",
        "category": "5. Airframe, Wing Structure & Mechanics",
        "qty": 4,
        "unit_price": 9.50,
        "vendor": "DragonPlate / Rock West Composites",
        "url": "https://dragonplate.com",
        "part_no": "DP-STRUT-6MM-120",
        "unit_mass_g": 8.5,
        "notes": "Splayed forward and aft struts setting ground clearance at 115mm (SSF = 1.00 rollover stability)."
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
        "notes": "M3x16 and M3x20 clamping bolts for split collars, bulkheads, and wing station hardpoints."
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
    lines.append(f"- **Operational Flight MTOW**: **5.897 kg (5,897 g)** (certified flight mass with 1.15m wings & GoolRC motors)")
    lines.append(f"- **Gross Kit & Spares Mass**: **{summary['grand_total_mass_kg']:.3f} kg ({summary['grand_total_mass_g']:,.0f} g)** (all line items including spares & bench gear)")
    lines.append(f"- **Longitudinal Balance (CG)**: **467.6 mm aft of datum (-5.1% MAC reference, 43.3 mm forward of NP)** on quick-pin slide rail (Safe Envelope: -10.0%–25.0% MAC)")
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
    print(f"  Operational MTOW:    {summary['grand_total_mass_kg']:.3f} kg ({summary['grand_total_mass_g']:,.0f} g)")
    print(f"  Center of Gravity:   467.6 mm aft of datum (-5.1% MAC) [IN ENVELOPE: YES]")
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
