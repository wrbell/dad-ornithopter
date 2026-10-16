# MCO-1D Mark 1 — Master Bill of Materials & Sourcing Directory

**Aircraft Designation**: MCO-1D Mark 1 Metachronal Cascade Ornithopter  
**Last Refreshed**: `2026-10-16 23:34:37 UTC`  
**Source of Truth**: `bom/bom.csv` + `config/airframe_spec.json`  

---

## Executive Sourcing Summary

- **Total Line Items**: 56 distinct components
- **Total Physical Parts Count**: 178 parts (airframe + spares)
- **Total Estimated Procurement Cost**: **$3,070.17 USD**
- **Calculated Design Installed Mass**: **3.642 kg (3,642.0 g / 8.03 lb)** (computed sum of `installed_mass_g` across all rows)
- **Measured Hardware Mass**: **TBD (measure on hardware)**
- **Gross Kit Procurement & Spares Mass**: **7.091 kg (7,090.8 g)** (includes full filament spools, spare servos, and bench tools)
- **Calculated Design CG**: **352.6 mm aft of datum (-7.5% MAC relative to LEMAC = 365.0 mm)**; Measured Hardware CG: **TBD (measure on hardware)**

| Category | Line Items | Subtotal (USD) | Gross Mass (g) | Installed Mass (g) | % of Cost |
| :--- | :---: | :---: | :---: | :---: | :---: |
| **1. Propulsion & Kinematic Drivetrain** | 6 | $841.00 | 1,674.4 g | 1,624.0 g | 27.4% |
| **2. Morphing Actuation & Servo Systems** | 5 | $717.09 | 744.0 g | 520.0 g | 23.4% |
| **3. Avionics, Computing & Sensors** | 12 | $373.99 | 218.0 g | 126.0 g | 12.2% |
| **4. Power Storage & Electrical Distribution** | 7 | $175.99 | 538.5 g | 400.0 g | 5.7% |
| **5. Airframe, Wing Structure & Mechanics** | 17 | $750.60 | 3,615.4 g | 972.0 g | 24.4% |
| **6. Precision Hardware, Fasteners & Consumables** | 9 | $211.50 | 300.5 g | 0.0 g | 6.9% |
| **TOTAL (All Subsystems)** | **56** | **$3,070.17** | **7,090.8 g** | **3,642.0 g** | **100.0%** |

---

## Itemized Sourcing Registry by Subsystem

### 1. Propulsion & Kinematic Drivetrain
*Category Subtotal: **$841.00 USD** | Gross Mass: **1,674.4 g** | Installed Mass: **1,624.0 g***

| Item ID | Component & Description | Qty | Unit Price | Ext. Price | Unit Mass | Installed Mass | Vendor / Link | Part Number / Notes |
| :--- | :--- | :---: | :---: | :---: | :---: | :---: | :--- | :--- |
| `PROP-01` | **GoolRC 5010-14 360KV High-Torque Multi-Rotor Brushless Motor** | 8 | $19.50 | **$156.00** | 95.0 g | 760.0 g | [Amazon / Banggood / AliExpress](https://www.amazon.com/dp/B073F9L5T2) | `GoolRC-5010-360KV`<br>*Baseline flight motor: 95g per motor, 360KV, 6S rated. (T-Motor MN5008 ~200KV retained as heavy-duty upgrade option).* |
| `PROP-02` | **ODrive S1 High-Performance CAN Motor Controller** | 8 | $79.00 | **$632.00** | 80.0 g | 640.0 g | [ODrive Robotics](https://odriverobotics.com/shop/odrive-s1) | `ODR-S1-CAN`<br>*1 per motor. FOC oscillation drive at 50 Hz loop. Node IDs 0-7 on CAN-FD.* |
| `PROP-03` | **50W 2.0-Ohm Central Dynamic Bus Brake Resistor (Shared DC Rail)** | 2 | $3.50 | **$7.00** | 25.0 g | 50.0 g | [ODrive Robotics / Mouser](https://odriverobotics.com/shop/50w-brake-resistor) | `KAL50FB2R00`<br>*2 shared clamp resistors on main 22.2V motor power rail (PDB). Replaces 8 individual per-motor resistors.* |
| `PROP-04` | **10:1 Capstan Sector Wheel (200 deg Arc, 3D-Printed Tough PLA)** | 8 | $0.75 | **$6.00** | 14.3 g | 114.0 g | [In-House 3D Print (Tough PLA)](https://sendcutsend.com) | `SD-M-0001`<br>*Legacy ID MCO1D-SEC-10TO1-PLA. 100% infill Tough PLA 10:1 sector wheel (R=60mm, 200 deg arc).* |
| `PROP-05` | **Dyneema SK99 0.8mm Braided Capstan Line (100m Spool)** | 1 | $34.00 | **$34.00** | 50.0 g | 4.0 g | [Marlow Ropes / West Marine](https://www.marlowropes.com/product/excel-d12-sk99) | `MAR-SK99-08MM`<br>*High modulus polyethylene. Break strength 210 kg. Pre-tensioned to 150-200 N on drum.* |
| `PROP-06` | **Precision 3D-Printed Helical Capstan Motor Drums (High-Temp PETG / Brass Insert)** | 8 | $0.75 | **$6.00** | 7.5 g | 56.0 g | [In-House 3D Print (PETG)](https://sendcutsend.com) | `SD-M-0002`<br>*Legacy ID MCO1D-DRUM-PETG. 100% infill PETG drum with M3 brass set-screw insert.* |

### 2. Morphing Actuation & Servo Systems
*Category Subtotal: **$717.09 USD** | Gross Mass: **744.0 g** | Installed Mass: **520.0 g***

| Item ID | Component & Description | Qty | Unit Price | Ext. Price | Unit Mass | Installed Mass | Vendor / Link | Part Number / Notes |
| :--- | :--- | :---: | :---: | :---: | :---: | :---: | :--- | :--- |
| `ACT-01A` | **AGFRC B44BLS High-Torque Mini Brushless Sweep Servos (15 kg-cm)** | 9 | $42.00 | **$378.00** | 44.0 g | 352.0 g | [AGF-RC Official / Amazon](https://www.agf-rc.com) | `B44BLS-SWEEP`<br>*8 installed on wing sweep stations + 1 spare. 15 kg-cm torque @ 8.4V, 444 deg/s.* |
| `ACT-01B` | **AGFRC B24CLM Ultra-Light Micro Coreless Pitch Servos (7.0 kg-cm)** | 9 | $28.50 | **$256.50** | 21.0 g | 168.0 g | [AGF-RC Official / Amazon](https://www.agf-rc.com) | `B24CLM-PITCH`<br>*8 installed on wing pitch/AoA stations + 1 spare. 7.0 kg-cm torque @ 8.4V, 545 deg/s.* |
| `ACT-02` | **AGF-SPV3 USB Servo Programmer** | 1 | $24.99 | **$24.99** | 25.0 g | 0.0 g | [AGF-RC Official](https://www.agf-rc.com/usb-servo-programmer-agf-spv3.html) | `AGF-SPV3`<br>*Mandatory commissioning bench tool: limits travel to 170 deg and configures 333 Hz PWM rate.* |
| `ACT-03` | **JST-SH 4-Pin Shielded Feedback Wire Extensions (30cm)** | 16 | $0.85 | **$13.60** | 4.0 g | 0.0 g | [SparkFun / Amazon](https://www.sparkfun.com/products/14427) | `CAB-JST-SH-4P-30`<br>*Routes 4th analog position wire (300-3000 mV) to MCP3208 12-bit ADC boards (mass included in PWR-06 harness).* |
| `ACT-04` | **Titanium Servo Horn Clamps (25T Spline, M2 Hardpoints)** | 20 | $2.20 | **$44.00** | 3.5 g | 0.0 g | [Du-Bro / Amazon](https://www.dubro.com) | `DUB-25T-TI-HORN`<br>*Clamping style servo horns with zero backlash (mass accounted in ACT-01A/B installed assemblies).* |

### 3. Avionics, Computing & Sensors
*Category Subtotal: **$373.99 USD** | Gross Mass: **218.0 g** | Installed Mass: **126.0 g***

| Item ID | Component & Description | Qty | Unit Price | Ext. Price | Unit Mass | Installed Mass | Vendor / Link | Part Number / Notes |
| :--- | :--- | :---: | :---: | :---: | :---: | :---: | :--- | :--- |
| `AV-01` | **Raspberry Pi 4B Single Board Computer (4GB RAM)** | 1 | $55.00 | **$55.00** | 46.0 g | 46.0 g | [CanaKit / Adafruit / SparkFun](https://www.adafruit.com/product/4296) | `RPI4-MODBP-4GB`<br>*Primary Flight Control Computer (FCC). Runs 50 Hz real-time executive mco_main.py.* |
| `AV-02` | **Waveshare MCP2518FD CAN-FD Hat for Raspberry Pi** | 1 | $19.99 | **$19.99** | 22.0 g | 0.0 g | [Waveshare Official / Amazon](https://www.waveshare.com/2-ch-can-fd-hat.htm) | `WS-18151-CANFD`<br>*High-speed SPI CAN-FD Hat. Operates at 1 Mbps nominal bitrate to ODrives (mass accounted in bus harness).* |
| `AV-03` | **Holybro Pixhawk 4 Mini Autopilot & M8N GPS Module** | 1 | $149.00 | **$149.00** | 38.0 g | 38.0 g | [Holybro Official / GetFPV](https://holybro.com/products/pixhawk-4-mini) | `HB-PX4-MINI-KIT`<br>*MAVLink bridge at 921600 baud. Provides master IMU, barometer, and GPS telemetry.* |
| `AV-04` | **MS4525DO Digital Pitot-Static Airspeed Sensor** | 1 | $26.00 | **$26.00** | 14.0 g | 14.0 g | [Holybro / Amazon](https://holybro.com/products/digital-air-speed-sensor) | `MS4525DO-DS`<br>*Nose-mounted pitot tube forward of wing wash. Critical for dynamic stall protection.* |
| `AV-05` | **Benewake TF-Luna Micro LiDAR Rangefinder (8m I2C)** | 1 | $24.50 | **$24.50** | 5.0 g | 5.0 g | [SparkFun / Benewake](https://www.sparkfun.com/products/16977) | `SEN-16977`<br>*0.1 to 8.0 m precision ground altitude. Drives landing flare and terrain-following.* |
| `AV-06` | **MPU-6050 6-Axis IMU Sensor Module** | 1 | $4.50 | **$4.50** | 3.0 g | 2.0 g | [Amazon / SparkFun](https://www.sparkfun.com/products/11028) | `GY-521-MPU6050`<br>*Secondary I2C (0x68) high-frequency body vibration and attitude rate monitor.* |
| `AV-07` | **BMP280 Precision Barometric Pressure Sensor** | 1 | $3.80 | **$3.80** | 2.0 g | 1.0 g | [Adafruit / Amazon](https://www.adafruit.com/product/2651) | `ADA-2651`<br>*I2C (0x76) barometric altitude fallback when LiDAR range exceeds 8.0 m.* |
| `AV-08` | **Microchip MCP3208 8-Channel 12-Bit SPI ADC Boards** | 2 | $4.85 | **$9.70** | 10.0 g | 20.0 g | [Mouser / Microchip](https://www.mouser.com/ProductDetail/Microchip/MCP3208-CI-P) | `MCP3208-CI-P`<br>*ADC #1 on CE1 for pitch feedback; ADC #2 on CE0 for sweep feedback (372-3723 counts).* |
| `AV-09` | **DS18B20 1-Wire Digital Ambient Air Temperature Sensor** | 1 | $2.50 | **$2.50** | 2.5 g | 0.0 g | [Adafruit / Mouser](https://www.adafruit.com/product/381) | `ADA-381`<br>*Connected to GPIO 4 (1-Wire). Computes real-time air density rho for aerodynamic scaling.* |
| `AV-10` | **FrSky Archer Plus RS 2.4 GHz SBUS Receiver** | 1 | $28.00 | **$28.00** | 4.5 g | 0.0 g | [Aloft Hobbies / GetFPV](https://alofthobbies.com/products/frsky-archer-plus-rs) | `FRSKY-AP-RS`<br>*SBUS input to Pixhawk 4 Mini. Failsafe auto-triggers RTH if link lost > 1.0 s.* |
| `AV-11` | **HX711 24-Bit ADC + 5 kg S-Type Precision Load Cell Kit** | 1 | $15.00 | **$15.00** | 45.0 g | 0.0 g | [SparkFun / Amazon](https://www.sparkfun.com/products/13879) | `SEN-13879-KIT`<br>*Used during TC-01 and TC-02 ground bench test stands for live thrust and lift logging (not flown).* |
| `AV-12` | **AS5600L 12-Bit Programmable I2C Magnetic Rotary Position Encoders** | 8 | $4.50 | **$36.00** | 2.0 g | 0.0 g | [Mouser / Digi-Key / AliExpress](https://www.mouser.com/ProductDetail/ams-OSRAM/AS5600L) | `AS5600L-POT`<br>*Optional contactless magnetic encoders at pitch shafts (requires I2C multiplexer PCA9548A if installed).* |

### 4. Power Storage & Electrical Distribution
*Category Subtotal: **$175.99 USD** | Gross Mass: **538.5 g** | Installed Mass: **400.0 g***

| Item ID | Component & Description | Qty | Unit Price | Ext. Price | Unit Mass | Installed Mass | Vendor / Link | Part Number / Notes |
| :--- | :--- | :---: | :---: | :---: | :---: | :---: | :--- | :--- |
| `PWR-01` | **Tattu / CNHL 6S 2200 mAh 70C LiPo Battery (22.2V, XT60)** | 1 | $39.99 | **$39.99** | 295.0 g | 295.0 g | [Tattu Official / CNHL / Amazon](https://genstattu.com) | `TAT-6S-2200-70C`<br>*Locked sweet-spot flight battery (22.2V nominal, 48.84 Wh, 295g).* |
| `PWR-02` | **Dual-Channel High-Current Synchronous Step-Down BEC System (2x 20A Cont / 30A Peak, 6S to 7.4V/8.4V)** | 2 | $24.00 | **$48.00** | 25.0 g | 25.0 g | [Matek Systems / KDE Direct / GetFPV](https://www.mateksys.com/?portfolio=ubec-duo) | `MTK-UBEC-DUO-40A`<br>*Dual-channel split power architecture (1 unit installed + 1 spare): powers Zone A and Zone B servo rails.* |
| `PWR-03` | **16-Port High-Current Servo Power Bus Board (Dual-Zone Isolated Rails, 100A Burst)** | 1 | $38.00 | **$38.00** | 35.0 g | 0.0 g | [PowerBox Systems / FrSky](https://www.powerbox-systems.com) | `PBS-SRV-BUS-16-DUAL`<br>*Distributes 7.4V power with isolated Zone A (Port 1-8) and Zone B (Port 9-16) planes (mass in PWR-06 harness).* |
| `PWR-04` | **Panasonic 4,700 uF 16V Low-ESR Electrolytic Buffer Capacitors (Dual Zone)** | 2 | $2.25 | **$4.50** | 8.0 g | 0.0 g | [Mouser / Panasonic](https://www.mouser.com/ProductDetail/Panasonic/EEU-FR1C472) | `EEU-FR1C472`<br>*Soldered directly across Zone A and Zone B bus terminal blocks (<15mm lead length).* |
| `PWR-05` | **High-Current 8-Way Motor Power Distribution Rail (120A)** | 1 | $14.50 | **$14.50** | 18.0 g | 0.0 g | [Matek Systems / Amazon](https://www.mateksys.com) | `MTK-PDB-120A`<br>*Feeds 22.2V main propulsion power to 8x ODrive S1 units (mass in PWR-06 harness).* |
| `PWR-06` | **High-Flex 14 AWG / 22 AWG Silicone Wire & XT60/XT30 Connector Kit** | 1 | $18.50 | **$18.50** | 120.0 g | 80.0 g | [Amass / Amazon](https://www.amazon.com) | `AMASS-SIL-CONN-KIT`<br>*Installed airframe silicone wiring harness, PDB busbars, and connectors (80g installed).* |
| `PWR-07` | **Texas Instruments INA226 High-Side Current & Power Monitor Breakout (0.005 Ohm Shunt)** | 1 | $12.50 | **$12.50** | 4.5 g | 0.0 g | [Adafruit / Mouser / Texas Instruments](https://www.adafruit.com/product/4982) | `TI-INA226-BRK`<br>*Inline on 7.4V servo bus, I2C (0x40) to Pi 4B GPIO 2/3.* |

### 5. Airframe, Wing Structure & Mechanics
*Category Subtotal: **$750.60 USD** | Gross Mass: **3,615.4 g** | Installed Mass: **972.0 g***

| Item ID | Component & Description | Qty | Unit Price | Ext. Price | Unit Mass | Installed Mass | Vendor / Link | Part Number / Notes |
| :--- | :--- | :---: | :---: | :---: | :---: | :---: | :--- | :--- |
| `STRUCT-01` | **High-Modulus 3K Roll-Wrapped Carbon Fiber Upper Spine Longeron (20mm OD x 17mm ID x 1000mm, cut to 700mm)** | 1 | $42.00 | **$42.00** | 92.5 g | 65.0 g | [DragonPlate / Rock West Composites](https://dragonplate.com/carbon-fiber-round-tubes) | `DP-RND-20X17-1000`<br>*Locked compact delta frame spine (cut to 700mm, 65g installed). 20mm OD x 1.5mm wall (J=7,508 mm4).* |
| `STRUCT-02` | **3K Roll-Wrapped Carbon Fiber Lower Keel Tube (12mm OD x 10mm ID x 1000mm, cut to 675mm)** | 1 | $28.00 | **$28.00** | 53.6 g | 36.0 g | [DragonPlate / Rock West Composites](https://dragonplate.com/carbon-fiber-round-tubes) | `DP-RND-12X10-1000`<br>*Locked compact delta frame keel tension tie (cut to 675mm, 36g installed).* |
| `STRUCT-03` | **3K Roll-Wrapped Carbon Fiber Front Vertical Truss Post (14mm OD x 12mm ID x 1000mm, cut to 160mm)** | 1 | $32.00 | **$32.00** | 63.3 g | 10.0 g | [DragonPlate / Rock West Composites](https://dragonplate.com/carbon-fiber-round-tubes) | `DP-RND-14X12-1000`<br>*Locked compact delta frame front post tying Corner 1 to Corner 2 (cut to 160mm, 10g installed).* |
| `STRUCT-04` | **Pinned Split-Clamp Modular Wing Mount Collars (26mm OD, 50mm length, 20mm bore)** | 8 | $18.00 | **$144.00** | 24.5 g | 109.0 g | [SendCutSend / In-House Print](https://sendcutsend.com) | `SD-M-0003`<br>*Legacy ID MCO1-CLAMP-20MM-PINNED. 8 modular clamp collars on 20mm spine (combined with STRUCT-01..03 = 220g delta frame).* |
| `STRUCT-05` | **Universal Rib & Segment Filament: ColorFabb LW-PLA Lightweight PLA (750g Spools)** | 3 | $48.00 | **$144.00** | 632.0 g | 160.0 g | [ColorFabb Official / MatterHackers](https://colorfabb.com/lw-pla-natural) | `SD-M-0004`<br>*Legacy ID CF-LW-PLA-750. 3 spools procured; 160g foaming LW-PLA printed across all 8 wing skeletons.* |
| `STRUCT-06` | **Pultruded Carbon Fiber Leading Edge Spar Tubes (10mm OD x 1.0mm wall x 1200mm, 8 Rods)** | 8 | $12.50 | **$100.00** | 58.0 g | 280.0 g | [DragonPlate / Rock West Composites](https://dragonplate.com/carbon-fiber-rods) | `DP-TUBE-10MM-1200`<br>*Continuous main spar at 25.0% MAC (cut to 1150mm, 35ginstalled per wing = 280g across 8 wings).* |
| `STRUCT-07` | **Hollow Carbon Fiber Secondary Alignment Spar Tubes (2.0mm OD x 1.0mm ID x 1000mm, 10-Pack)** | 4 | $21.99 | **$87.96** | 41.2 g | 40.0 g | [DragonPlate / Rock West Composites / Amazon](https://dragonplate.com) | `DP-TUBE-2MM-MICRO`<br>*Pultruded micro hollow carbon tubes (40g installed across 8 wings).* |
| `STRUCT-08` | **Ultralight Mylar Wing Covering Film (Oralight 2m Spools)** | 4 | $24.00 | **$96.00** | 72.0 g | 72.0 g | [Oracover / Horizon Hobby](https://www.oracover.de) | `ORA-LIGHT-TRANSP`<br>*36 g/m2 iron-on covering wrapping all 8 wings (1.224 m2 planform; STRUCT-05..08 = 552g = 4x 138g wing pairs).* |
| `STRUCT-09` | **Hybrid Teardrop Electronics Pod with NACA Ram-Air Duct & Negative-Pressure Louvers** | 1 | $5.50 | **$5.50** | 118.0 g | 116.0 g | [In-House 3D Print (PETG Tub / LW-PLA Hatch)](https://sendcutsend.com) | `SD-M-0005`<br>*Legacy ID MCO1D-POD-HYBRID-NACA. 370mm pod with integrated NACA nose intake (116g installed).* |
| `STRUCT-10` | **Du-Bro 200MW 2.00 in (50mm x 14mm) Super Light Foam Wheels (Flight Spares)** | 2 | $7.95 | **$15.90** | 13.6 g | 0.0 g | [Du-Bro Official / Horizon Hobby](https://www.dubro.com) | `DUB-200MW`<br>*Closed-cell ultralight EVA foam tires with 4.0mm axle bore (flight-line backup spares).* |
| `STRUCT-11` | **Precision Ground 4.00mm x 40mm Stainless Steel Axles with Retaining Hardware** | 2 | $4.50 | **$9.00** | 3.9 g | 0.0 g | [McMaster-Carr](https://www.mcmaster.com/precision-shafts) | `MMC-1254K18`<br>*416 stainless ground axle spares with E-clips.* |
| `STRUCT-12` | **NOBRIM Carbon Fiber Main Landing Gear YAK50E Wheel Kit (Taildragger Main Arch)** | 1 | $19.99 | **$19.99** | 36.0 g | 36.0 g | [NOBRIM / FLTHOBBY / Amazon](https://www.amazon.com/dp/B0BL71QR5G) | `B0BL71QR5G`<br>*Locked taildragger main gear clamped at X=400.0mm. 275mm track, 55mm EVA foam wheels.* |
| `STRUCT-13` | **3D-Printed Resilient PETG Taildragger Tailskid Assembly** | 1 | $1.25 | **$1.25** | 9.0 g | 9.0 g | [In-House 3D Print (PETG)](https://sendcutsend.com) | `SD-M-0006`<br>*Legacy ID MCO1D-SKID-PETG. Curved leaf spring tailskid at Corner 3 (X=690mm, 9g installed).* |
| `STRUCT-14` | **3D-Printed Carbon-Fiber Polycarbonate (PC-CF) Frame Clamps Kit (Corners 1, 2, 3 & Saddle)** | 1 | $9.50 | **$9.50** | 48.0 g | 39.0 g | [In-House 3D Print (PC-CF 100% Infill)](https://sendcutsend.com) | `SD-M-0007`<br>*Legacy ID MCO1D-CLAMPS-PCCF. Printed in CarbonX PC-CF with 100% infill (39g installed after trim).* |
| `STRUCT-15` | **Fuselage Compact Delta Truss Aerodynamic Infill Fairing (ColorFabb LW-PLA)** | 1 | $6.50 | **$6.50** | 95.0 g | 0.0 g | [In-House 3D Print (LW-PLA)](https://colorfabb.com/lw-pla-natural) | `SD-M-0008`<br>*Legacy ID MCO1D-FAIR-DELTA-PLA. Optional aerodynamic fairing skin (omitted in naked bench/initial flight configuration).* |
| `STRUCT-16` | **Streamlined Teardrop Wing Root Nacelles Kit (8x Units with Flapping Wiper Seals)** | 1 | $4.50 | **$4.50** | 48.0 g | 0.0 g | [In-House 3D Print (LW-PLA / Mylar)](https://colorfabb.com/lw-pla-natural) | `SD-M-0009`<br>*Legacy ID MCO1D-FAIR-ROOTS-8X. Optional 8x 6.0g streamlined teardrop wrist fairings.* |
| `STRUCT-17` | **3D-Printed Capstan Line Keeper Shrouds Kit (8x Pinion Anti-Derailment Shrouds)** | 1 | $4.50 | **$4.50** | 8.0 g | 0.0 g | [In-House 3D Print (PETG)](https://sendcutsend.com) | `SD-M-0010`<br>*Legacy ID MCO1D-CAPSTAN-SHROUDS-8X. Snap-on shrouds maintaining <0.4mm clearance over Dyneema SK99 grooves.* |

### 6. Precision Hardware, Fasteners & Consumables
*Category Subtotal: **$211.50 USD** | Gross Mass: **300.5 g** | Installed Mass: **0.0 g***

| Item ID | Component & Description | Qty | Unit Price | Ext. Price | Unit Mass | Installed Mass | Vendor / Link | Part Number / Notes |
| :--- | :--- | :---: | :---: | :---: | :---: | :---: | :--- | :--- |
| `HW-01` | **MF106ZZ Flanged Miniature Ball Bearings (6mm ID x 10mm OD x 3mm Width, 16-Pack)** | 1 | $32.00 | **$32.00** | 38.4 g | 0.0 g | [Boca Bearings / Amazon](https://www.bocabearings.com) | `BOCA-MF106ZZ-16PK`<br>*Double shielded flanged bearings installed in split collars (mass included in STRUCT-04 / PROP-04 subassemblies).* |
| `HW-02` | **Grade 5 Titanium (Ti-6Al-4V) Precision Ground 6.00mm x 50mm Pivot Pins** | 8 | $9.50 | **$76.00** | 6.4 g | 0.0 g | [McMaster-Carr / Titanium Joe](https://www.mcmaster.com/titanium-pins) | `MMC-92745A315`<br>*High shear strength (880 MPa) pivot pins (mass included in STRUCT-04 / PROP-04 subassemblies).* |
| `HW-03` | **Grade 12.9 Alloy Steel M3 Socket Head Cap Screws & M3 Nyloc Nuts (100-Piece Assortment)** | 2 | $14.50 | **$29.00** | 65.0 g | 0.0 g | [McMaster-Carr](https://www.mcmaster.com/screws) | `MMC-91290A124`<br>*M3x16 and M3x20 clamping bolts (installed fastener mass included in subassembly allocations).* |
| `HW-04` | **Loctite 243 Medium Strength Threadlocker & Retaining Compound** | 1 | $14.50 | **$14.50** | 15.0 g | 0.0 g | [McMaster-Carr / Henkel](https://www.mcmaster.com/loctite) | `HEN-LOC-243-10ML`<br>*Mandatory on all metal-to-metal motor mount, servo horn, and sector wheel fasteners.* |
| `HW-05` | **Closed-Cell Neoprene Vibration Dampening Foam Mounts (3mm)** | 1 | $8.00 | **$8.00** | 12.0 g | 0.0 g | [McMaster-Carr](https://www.mcmaster.com/foam) | `MMC-FOAM-NEO-3MM`<br>*Isolates Pixhawk 4 Mini and Raspberry Pi FCC from 50 Hz airframe motor harmonics.* |
| `HW-06` | **Aviation 120-Ohm Precision CAN Termination Resistor Jumpers** | 2 | $0.50 | **$1.00** | 1.0 g | 0.0 g | [Mouser / TE Connectivity](https://www.mouser.com) | `TE-CAN-120R`<br>*Installed at the physical ends of the CAN-FD bus (Pi Hat end and Node #7 end).* |
| `HW-07` | **Zero-Slop CNC Aluminum Ball Links & M3 Titanium Pushrod Turnbuckles Kit (16-Pack)** | 1 | $32.00 | **$32.00** | 20.0 g | 0.0 g | [Secraft / Dubro / Amazon](https://www.secraft.net) | `SEC-CNC-BALL-M3-16PK`<br>*Zero-backlash (<0.1mm play) linkages connecting AGFRC servos to wing root pitch horns.* |
| `HW-08` | **High-Temp FR4 / Ceramic-Filled Silicone Motor Thermal Isolator Gaskets (0.5mm, 8-Pack)** | 1 | $6.50 | **$6.50** | 4.8 g | 0.0 g | [McMaster-Carr / SendCutSend](https://www.mcmaster.com) | `MCO1D-TH-GASKET-8PK`<br>*0.5mm thermal isolator gaskets mounted between GoolRC 5010 motor faces and brackets.* |
| `HW-09` | **C3604 Brass Heat-Set Threaded Inserts Assortment (M2.0, M2.5, M3 Ruthex / CNC Style, 150-Pack)** | 1 | $12.50 | **$12.50** | 27.1 g | 0.0 g | [Ruthex / CNC Kitchen / Amazon](https://www.ruthex.net) | `RUTHEX-ASSORT-M2-M3`<br>*114 inserts required across airframe (installed insert mass included in 3D-printed subassemblies).* |

---

## Procurement Guidelines & Critical Notes

1. **Servo Travel Calibration**: Pre-program all 16 AGFRC servos (8x B44BLS sweep, 8x B24CLM pitch) to **170 deg maximum travel** using the `AGF-SPV3` programmer prior to installation.
2. **Motor Selection**: GoolRC 5010-14 360KV motors serve as the baseline flight drive ($19.50 ea).
3. **Power Bus Architecture**: Two Matek UBEC Duo step-down regulators power Zone A (Stations 1-2) and Zone B (Stations 3-4) servo rails at 7.4V.
4. **CAN-FD Termination**: Enable two 120-Ohm termination resistors: one on the Waveshare MCP2518FD Hat, and one at ODrive S1 Node #7.
5. **Dynamic Updates**: Run `./mco bom --update` to refresh `bom/bom.csv`, `config/airframe_spec.json`, and this Markdown report.
