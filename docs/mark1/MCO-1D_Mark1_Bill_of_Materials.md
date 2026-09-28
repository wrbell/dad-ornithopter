# MCO-1D Mark 1 — Master Bill of Materials & Sourcing Directory

**Aircraft Designation**: MCO-1D Mark 1 Metachronal Cascade Ornithopter  
**Demonstrator Program**: MCO-1A Search-and-Rescue (SAR) Subscale Prototype  
**Last Refreshed / Price Verified**: `2026-09-22 19:56:47 UTC`  
**Classification**: Authoritative Engineering Sourcing Record (Mark 1 Baseline)  

---

## Executive Sourcing Summary

- **Total Line Items**: 45 distinct components
- **Total Physical Parts Count**: 176 parts (complete airframe + spares)
- **Total Estimated Procurement Cost**: **$3,863.28 USD**
- **Operational Flight MTOW**: **5.897 kg (5,897 g)** (certified flight mass with 1.15m wings & GoolRC motors)
- **Gross Kit & Spares Mass**: **8.226 kg (8,226 g)** (all line items including spares & bench gear)
- **Longitudinal Balance (CG)**: **467.6 mm aft of datum (-5.1% MAC reference, 43.3 mm forward of NP)** on quick-pin slide rail (Safe Envelope: -10.0%–25.0% MAC)

| Category | Line Items | Subtotal (USD) | Subtotal Mass (g) | % of Cost |
| :--- | :---: | :---: | :---: | :---: |
| **1. Propulsion & Kinematic Drivetrain** | 6 | $1,094.00 | 2,098 g | 28.3% |
| **2. Morphing Actuation & Servo Systems** | 4 | $1,216.41 | 1,410 g | 31.5% |
| **3. Avionics, Computing & Sensors** | 11 | $337.99 | 202 g | 8.7% |
| **4. Power Storage & Electrical Distribution** | 6 | $222.48 | 904 g | 5.8% |
| **5. Airframe, Wing Structure & Mechanics** | 12 | $831.90 | 3,363 g | 21.5% |
| **6. Precision Hardware, Fasteners & Consumables** | 6 | $160.50 | 249 g | 4.2% |
| **TOTAL (All Subsystems)** | **45** | **$3,863.28** | **8,226 g** | **100.0%** |

---

## Itemized Sourcing Registry by Subsystem

### 1. Propulsion & Kinematic Drivetrain
*Category Subtotal: **$1,094.00 USD** | Mass: **2,098 g***

| Item ID | Component & Description | Qty | Unit Price | Ext. Price | Unit Mass | Vendor / Sourcing Link | Part Number / Notes |
| :--- | :--- | :---: | :---: | :---: | :---: | :--- | :--- |
| `PROP-01` | **GoolRC 5010-14 360KV High-Torque Multi-Rotor Brushless Motor** | 8 | $19.50 | **$156.00** | 95.0 g | [Amazon / Banggood / AliExpress](https://www.amazon.com/dp/B073F9L5T2) | `GoolRC-5010-360KV`<br>*Baseline flight motor: 95g per motor, 360KV, 6S rated. (T-Motor MN5008 ~200KV retained as heavy-duty upgrade option).* |
| `PROP-02` | **ODrive S1 High-Performance CAN Motor Controller** | 8 | $79.00 | **$632.00** | 80.0 g | [ODrive Robotics](https://odriverobotics.com/shop/odrive-s1) | `ODR-S1-CAN`<br>*1 per motor. FOC oscillation drive at 50 Hz loop. Node IDs 0–7 on CAN-FD.* |
| `PROP-03` | **50W 2.0-Ohm Dynamic Brake Resistor** | 8 | $3.50 | **$28.00** | 25.0 g | [ODrive Robotics / Mouser](https://odriverobotics.com/shop/50w-brake-resistor) | `KAL50FB2R00`<br>*1 per S1 controller. Absorbs rapid deceleration regenerative spikes at stroke reversals.* |
| `PROP-04` | **7:1 Capstan Sector Wheel (200° Arc, 2mm Carbon Fiber)** | 8 | $18.50 | **$148.00** | 38.0 g | [SendCutSend / In-House CNC](https://sendcutsend.com/materials/carbon-fiber/) | `MCO1D-SEC-7TO1-200D`<br>*Custom 2mm CF plate cut to sector_wheel_7to1_200deg.dxf. Full 200° arc (±100° stroke authority) rigged with Dyneema groove.* |
| `PROP-05` | **Dyneema SK99 0.8mm Braided Capstan Line (100m Spool)** | 1 | $34.00 | **$34.00** | 50.0 g | [Marlow Ropes / West Marine](https://www.marlowropes.com/product/excel-d12-sk99) | `MAR-SK99-08MM`<br>*High modulus polyethylene. Break strength 210 kg. Pre-tensioned to 150–200 N on drum.* |
| `PROP-06` | **Helical Capstan Motor Drums (Lathe Turned 6061-T6)** | 8 | $12.00 | **$96.00** | 18.0 g | [Xometry / In-House Lathe](https://www.xometry.com) | `MCO1D-DRUM-22MM`<br>*22 mm pitch diameter helical groove drum with anchor pin for Dyneema line.* |

### 2. Morphing Actuation & Servo Systems
*Category Subtotal: **$1,216.41 USD** | Mass: **1,410 g***

| Item ID | Component & Description | Qty | Unit Price | Ext. Price | Unit Mass | Vendor / Sourcing Link | Part Number / Notes |
| :--- | :--- | :---: | :---: | :---: | :---: | :--- | :--- |
| `ACT-01` | **AGFRC IB53BHU Brushless Programmable Servo (with 4th Analog Wire)** | 18 | $62.99 | **$1,133.82** | 69.5 g | [AGF-RC Official / Amazon](https://www.agf-rc.com/programmable-brushless-servo-ib53bhu.html) | `IB53BHU-FB`<br>*16 installed (8 pitch, 8 sweep) + 2 flight-line spares. 36 kg·cm torque @ 7.4V, 444°/s.* |
| `ACT-02` | **AGF-SPV3 USB Servo Programmer** | 1 | $24.99 | **$24.99** | 25.0 g | [AGF-RC Official](https://www.agf-rc.com/usb-servo-programmer-agf-spv3.html) | `AGF-SPV3`<br>*Mandatory commissioning tool: limits travel to 170° and configures 333 Hz PWM rate.* |
| `ACT-03` | **JST-SH 4-Pin Shielded Feedback Wire Extensions (30cm)** | 16 | $0.85 | **$13.60** | 4.0 g | [SparkFun / Amazon](https://www.sparkfun.com/products/14427) | `CAB-JST-SH-4P-30`<br>*Routes 4th analog position wire (300–3000 mV) to MCP3208 12-bit ADC boards.* |
| `ACT-04` | **Titanium Servo Horn Clamps (25T Spline, M2 Hardpoints)** | 20 | $2.20 | **$44.00** | 3.5 g | [Du-Bro / Amazon](https://www.dubro.com) | `DUB-25T-TI-HORN`<br>*Clamping style servo horns with zero backlash. Secured with Loctite 243.* |

### 3. Avionics, Computing & Sensors
*Category Subtotal: **$337.99 USD** | Mass: **202 g***

| Item ID | Component & Description | Qty | Unit Price | Ext. Price | Unit Mass | Vendor / Sourcing Link | Part Number / Notes |
| :--- | :--- | :---: | :---: | :---: | :---: | :--- | :--- |
| `AV-01` | **Raspberry Pi 4B Single Board Computer (4GB RAM)** | 1 | $55.00 | **$55.00** | 46.0 g | [CanaKit / Adafruit / SparkFun](https://www.adafruit.com/product/4296) | `RPI4-MODBP-4GB`<br>*Primary Flight Control Computer (FCC). Runs 50 Hz real-time executive mco_main.py.* |
| `AV-02` | **Waveshare MCP2518FD CAN-FD Hat for Raspberry Pi** | 1 | $19.99 | **$19.99** | 22.0 g | [Waveshare Official / Amazon](https://www.waveshare.com/2-ch-can-fd-hat.htm) | `WS-18151-CANFD`<br>*High-speed SPI CAN-FD Hat. Operates at 1 Mbps nominal / 8 Mbps data bitrate to ODrives.* |
| `AV-03` | **Holybro Pixhawk 4 Mini Autopilot & M8N GPS Module** | 1 | $149.00 | **$149.00** | 38.0 g | [Holybro Official / GetFPV](https://holybro.com/products/pixhawk-4-mini) | `HB-PX4-MINI-KIT`<br>*MAVLink bridge at 921600 baud. Provides master IMU, barometer, and RTK GPS telemetry.* |
| `AV-04` | **MS4525DO Digital Pitot-Static Airspeed Sensor** | 1 | $26.00 | **$26.00** | 14.0 g | [Holybro / Amazon](https://holybro.com/products/digital-air-speed-sensor) | `MS4525DO-DS`<br>*Nose-mounted pitot tube forward of wing wash. Critical for dynamic stall protection.* |
| `AV-05` | **Benewake TF-Luna Micro LiDAR Rangefinder (8m I2C)** | 1 | $24.50 | **$24.50** | 5.0 g | [SparkFun / Benewake](https://www.sparkfun.com/products/16977) | `SEN-16977`<br>*0.1 to 8.0 m precision ground altitude. Drives landing flare and terrain-following.* |
| `AV-06` | **MPU-6050 6-Axis IMU Sensor Module** | 1 | $4.50 | **$4.50** | 3.0 g | [Amazon / SparkFun](https://www.sparkfun.com/products/11028) | `GY-521-MPU6050`<br>*Secondary I2C (0x68) high-frequency body vibration and attitude rate monitor.* |
| `AV-07` | **BMP280 Precision Barometric Pressure Sensor** | 1 | $3.80 | **$3.80** | 2.0 g | [Adafruit / Amazon](https://www.adafruit.com/product/2651) | `ADA-2651`<br>*I2C (0x76) barometric altitude fallback when LiDAR range exceeds 8.0 m.* |
| `AV-08` | **Microchip MCP3208 8-Channel 12-Bit SPI ADC Boards** | 2 | $4.85 | **$9.70** | 10.0 g | [Mouser / Microchip](https://www.mouser.com/ProductDetail/Microchip/MCP3208-CI-P) | `MCP3208-CI/P`<br>*ADC #1 on CE1 for pitch feedback; ADC #2 on CE0 for sweep feedback (372–3723 counts).* |
| `AV-09` | **DS18B20 1-Wire Digital Ambient Air Temperature Sensor** | 1 | $2.50 | **$2.50** | 2.5 g | [Adafruit / Mouser](https://www.adafruit.com/product/381) | `ADA-381`<br>*Connected to GPIO 4 (1-Wire). Computes real-time air density rho for aerodynamic scaling.* |
| `AV-10` | **FrSky Archer Plus RS 2.4 GHz SBUS Receiver** | 1 | $28.00 | **$28.00** | 4.5 g | [Aloft Hobbies / GetFPV](https://alofthobbies.com/products/frsky-archer-plus-rs) | `FRSKY-AP-RS`<br>*SBUS input to Pixhawk 4 Mini. Failsafe auto-triggers RTH if link lost > 1.0 s.* |
| `AV-11` | **HX711 24-Bit ADC + 5 kg S-Type Precision Load Cell Kit** | 1 | $15.00 | **$15.00** | 45.0 g | [SparkFun / Amazon](https://www.sparkfun.com/products/13879) | `SEN-13879-KIT`<br>*Used during TC-01 and TC-02 ground bench test stands for live thrust and lift logging.* |

### 4. Power Storage & Electrical Distribution
*Category Subtotal: **$222.48 USD** | Mass: **904 g***

| Item ID | Component & Description | Qty | Unit Price | Ext. Price | Unit Mass | Vendor / Sourcing Link | Part Number / Notes |
| :--- | :--- | :---: | :---: | :---: | :---: | :--- | :--- |
| `PWR-01` | **Tattu / Gens Ace 6S 5000 mAh 50C LiPo Battery (22.2V)** | 1 | $119.99 | **$119.99** | 540.0 g | [Gens Ace Official / Tattu USA](https://genstattu.com/tattu-5000mah-22-2v-50c-6s1p-lipo-battery-pack-with-xt90-plug/) | `TA-50C-5000-6S1P`<br>*Main propulsion pack. Mounted on quick-pin sliding tray (230–340 mm arm along keel).* |
| `PWR-02` | **Tattu / Gens Ace 2S 3000 mAh 45C LiPo Battery (7.4V)** | 1 | $26.99 | **$26.99** | 175.0 g | [Gens Ace Official / Amazon](https://genstattu.com/gens-ace-3000mah-7-4v-45c-2s1p-lipo-battery-pack/) | `GEA-45C-3000-2S1P`<br>*Dedicated isolated servo power pack. Prevents brownouts during 40A multi-servo spikes.* |
| `PWR-03` | **16-Port High-Current Servo Power Bus Board (100A Burst)** | 1 | $38.00 | **$38.00** | 35.0 g | [PowerBox Systems / FrSky](https://www.powerbox-systems.com) | `PBS-SRV-BUS-16`<br>*Distributes 7.4V power from 2S battery to all 16 AGFRC servos with isolated signal lines.* |
| `PWR-04` | **Panasonic 4,700 µF 16V Low-ESR Electrolytic Buffer Capacitor** | 2 | $2.25 | **$4.50** | 8.0 g | [Mouser / Panasonic](https://www.mouser.com/ProductDetail/Panasonic/EEU-FR1C472) | `EEU-FR1C472`<br>*Connected across the 7.4V servo bus rail to suppress voltage dips during reversals.* |
| `PWR-05` | **High-Current 8-Way Motor Power Distribution Rail (120A)** | 1 | $14.50 | **$14.50** | 18.0 g | [Matek Systems / Amazon](https://www.mateksys.com) | `MTK-PDB-120A`<br>*Feeds 22.2V main propulsion power to 8× ODrive S1 units via 30A automotive blade fuses.* |
| `PWR-06` | **High-Flex 14 AWG / 22 AWG Silicone Wire & XT60/XT30 Connector Kit** | 1 | $18.50 | **$18.50** | 120.0 g | [Amass / Amazon](https://www.amazon.com) | `AMASS-SIL-CONN-KIT`<br>*Ultra-flexible multi-strand wire for vibrating armatures and quick-disconnect connectors.* |

### 5. Airframe, Wing Structure & Mechanics
*Category Subtotal: **$831.90 USD** | Mass: **3,363 g***

| Item ID | Component & Description | Qty | Unit Price | Ext. Price | Unit Mass | Vendor / Sourcing Link | Part Number / Notes |
| :--- | :--- | :---: | :---: | :---: | :---: | :--- | :--- |
| `STRUCT-01` | **High-Modulus 3K Roll-Wrapped Carbon Fiber Upper Spine Longerons (⌀12mm OD x 10mm ID x 1000mm)** | 2 | $34.50 | **$69.00** | 26.2 g | [DragonPlate / Rock West Composites](https://dragonplate.com/carbon-fiber-round-tubes) | `DP-RND-12X10-1000`<br>*Upgraded Rev B spine longerons. Triples polar moment of inertia (J=678 mm⁴) against metachronal flapping twist.* |
| `STRUCT-02` | **3K Roll-Wrapped Carbon Fiber Lower Keel & Bulkhead Tubes (⌀8mm OD x 6mm ID x 1000mm)** | 4 | $22.00 | **$88.00** | 17.5 g | [DragonPlate / Rock West Composites](https://dragonplate.com/carbon-fiber-round-tubes) | `DP-RND-08X06-1000`<br>*Lower keel longerons, vertical bulkhead posts, and transverse cross-brace torque ties.* |
| `STRUCT-03` | **3K Carbon Fiber Diagonal Shear Truss Bracing Tubes (⌀6mm OD x 4mm ID x 1000mm)** | 2 | $16.50 | **$33.00** | 13.1 g | [DragonPlate / Amazon](https://dragonplate.com/carbon-fiber-round-tubes) | `DP-RND-06X04-1000`<br>*Diagonal shear truss web members providing high torsional and bending triangulated stiffness.* |
| `STRUCT-04` | **CNC 6061-T6 / Carbon Split-Clamp Structural Wing Mount Collars (⌀18mm OD, 60mm clamp length, ⌀12mm bore)** | 8 | $18.00 | **$144.00** | 28.5 g | [SendCutSend / Xometry](https://sendcutsend.com) | `MCO1-CLAMP-12MM-3DOF`<br>*8 circumferential clamp collars wrapping ⌀12mm longerons with 2x M3 pinch bolts and 4x M3 mount patterns.* |
| `STRUCT-05` | **Universal Rib & Segment Filament: ColorFabb LW-PLA Lightweight PLA (750g Spools)** | 3 | $48.00 | **$144.00** | 632.0 g | [ColorFabb Official / MatterHackers](https://colorfabb.com/lw-pla-natural) | `CF-LW-PLA-750`<br>*3 spools to print all 32 modular 250mm wing segments (4 per wing x 8 wings), 8 mounts, and 8 skeleton wingtips.* |
| `STRUCT-06` | **Pultruded Carbon Fiber Leading Edge Spar Tubes (⌀10mm OD x 1.0mm wall x 1200mm, 8 Rods)** | 8 | $12.50 | **$100.00** | 58.0 g | [DragonPlate / Rock West Composites](https://dragonplate.com/carbon-fiber-rods) | `DP-TUBE-10MM-1200`<br>*Continuous 1.2m main spar running through root mount, all 4 segments (1.0m run), and anchoring into skeleton wingtip.* |
| `STRUCT-07` | **Solid Carbon Fiber Secondary Alignment Spar Rods (⌀2.0mm & ⌀3.0mm x 1200mm, 10-Pack)** | 4 | $16.00 | **$64.00** | 55.0 g | [DragonPlate / Amazon](https://dragonplate.com) | `DP-ROD-2MM-1200-PACK`<br>*Inserted through continuous internal alignment bores across all 4 segments at 1.8mm and 72mm chord positions.* |
| `STRUCT-08` | **Ultralight Mylar Wing Covering Film (Oralight 2m Spools)** | 4 | $24.00 | **$96.00** | 72.0 g | [Oracover / Horizon Hobby](https://www.oracover.de) | `ORA-LIGHT-TRANSP`<br>*36 g/m² iron-on covering wrapping all 8 extended wings (1.224 m² total lifting boundary layer surface).* |
| `STRUCT-09` | **Sliding Battery Tray Assembly with 4mm Stainless Ball-Lock Pin** | 1 | $22.00 | **$22.00** | 42.0 g | [McMaster-Carr / In-House](https://www.mcmaster.com/quick-release-pins) | `MMC-PIN-4MM-SS`<br>*Slides along lower deck tray X=120–550 mm. 10 mm indexing pitch with CG reference scale.* |
| `STRUCT-10` | **Du-Bro 200MW 2.00" (⌀50mm x 14mm) Super Light Foam Wheels (2 Pairs / 4 Wheels)** | 2 | $7.95 | **$15.90** | 13.6 g | [Du-Bro Official / Horizon Hobby](https://www.dubro.com) | `DUB-200MW`<br>*Closed-cell ultralight EVA foam tires with 4.0mm axle bore. 6.8g per wheel.* |
| `STRUCT-11` | **Precision Ground ⌀4.00mm x 40mm Stainless Steel Axles with E-Clip Retainers** | 4 | $4.50 | **$18.00** | 3.9 g | [McMaster-Carr](https://www.mcmaster.com/precision-shafts) | `MMC-1254K18`<br>*416 stainless ground axles for front (tf=250mm) and rear (tr=320mm) landing gear.* |
| `STRUCT-12` | **⌀6.0mm Carbon Fiber Splayed Landing Gear Strut Tubes (4 Struts)** | 4 | $9.50 | **$38.00** | 8.5 g | [DragonPlate / Rock West Composites](https://dragonplate.com) | `DP-STRUT-6MM-120`<br>*Splayed forward and aft struts setting ground clearance at 115mm (SSF = 1.00 rollover stability).* |

### 6. Precision Hardware, Fasteners & Consumables
*Category Subtotal: **$160.50 USD** | Mass: **249 g***

| Item ID | Component & Description | Qty | Unit Price | Ext. Price | Unit Mass | Vendor / Sourcing Link | Part Number / Notes |
| :--- | :--- | :---: | :---: | :---: | :---: | :--- | :--- |
| `HW-01` | **MF106ZZ Flanged Miniature Ball Bearings (⌀6mm ID x ⌀10mm OD x 3mm Width, 16-Pack)** | 1 | $32.00 | **$32.00** | 38.4 g | [Boca Bearings / Amazon](https://www.bocabearings.com) | `BOCA-MF106ZZ-16PK`<br>*Double shielded flanged bearings installed in split collars for 8 wing 3-DOF pivot pins.* |
| `HW-02` | **Grade 5 Titanium (Ti-6Al-4V) Precision Ground ⌀6.00mm x 50mm Pivot Pins** | 8 | $9.50 | **$76.00** | 6.4 g | [McMaster-Carr / Titanium Joe](https://www.mcmaster.com/titanium-pins) | `MMC-92745A315`<br>*High shear strength (880 MPa) pivot pins locking S1223 airfoil mount brackets into split collars.* |
| `HW-03` | **Grade 12.9 Alloy Steel M3 Socket Head Cap Screws & M3 Nyloc Nuts (100-Piece Assortment)** | 2 | $14.50 | **$29.00** | 65.0 g | [McMaster-Carr](https://www.mcmaster.com/screws) | `MMC-91290A124`<br>*M3x16 and M3x20 clamping bolts for split collars, bulkheads, and wing station hardpoints.* |
| `HW-04` | **Loctite 243 Medium Strength Threadlocker & Retaining Compound** | 1 | $14.50 | **$14.50** | 15.0 g | [McMaster-Carr / Henkel](https://www.mcmaster.com/loctite) | `HEN-LOC-243-10ML`<br>*Mandatory on all metal-to-metal motor mount, servo horn, and sector wheel fasteners.* |
| `HW-05` | **Closed-Cell Neoprene Vibration Dampening Foam Mounts (3mm)** | 1 | $8.00 | **$8.00** | 12.0 g | [McMaster-Carr](https://www.mcmaster.com/foam) | `MMC-FOAM-NEO-3MM`<br>*Isolates Pixhawk 4 Mini and Raspberry Pi FCC from 50 Hz airframe motor harmonics.* |
| `HW-06` | **Aviation 120-Ohm Precision CAN Termination Resistor Jumpers** | 2 | $0.50 | **$1.00** | 1.0 g | [Mouser / TE Connectivity](https://www.mouser.com) | `TE-CAN-120R`<br>*Installed at the extreme physical ends of the CAN-FD bus (Pi Hat end and Node #7 end).* |

---

## Procurement Guidelines & Critical Notes

1. **Servo Travel Calibration**: All 16 AGFRC IB53BHU servos MUST be pre-programmed to **170° maximum travel** using the `AGF-SPV3` programmer prior to airframe installation. The factory 180° travel saturates the MCP3208 12-bit SPI ADCs.
2. **Motor Selection & Bearings**: GoolRC 5010-14 360KV motors serve as the baseline flight drive ($19.50 ea), saving $563 USD and 320g airframe mass. T-Motor MN5008 motors with EZO precision bearings remain an approved heavy-duty upgrade path.
3. **Isolated Electrical Rails**: Do NOT connect the 7.4V servo bus ground to the 22.2V motor power ground on the secondary side of the flight computer. Complete galvanic isolation protects avionics against motor back-EMF spikes.
4. **CAN-FD Termination**: Exactly two 120-Ohm resistors must be enabled: one on the Waveshare MCP2518FD Hat, and one at ODrive S1 Node #7.
5. **Dynamic Updates**: Run `./view_bom.sh` or `python3 scripts/mco_bom_tool.py --update` to refresh prices and live timestamps.
