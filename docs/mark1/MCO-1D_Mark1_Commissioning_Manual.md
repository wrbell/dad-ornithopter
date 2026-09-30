# MCO-1D Mark 1 — Commissioning, Calibration & Bringup Manual

**Aircraft Title**: MCO-1D Mark 1 Metachronal Cascade Ornithopter  
**Document Designation**: MCO-1D-COM-MK1  
**Classification**: Authoritative Commissioning Baseline — Revision Mark 1 (September 2026)  

---

## 1. Safety Protocols & Initial Power-Up Sequence

Commissioning the **MCO-1D Mark 1** involves high-torque oscillating drivetrains (8× T-Motor MN5008 capable of $2.2\text{ N}\cdot\text{m}$ peak torque each) and 16 high-speed brushless servos on an isolated $7.4\text{V}$ rail.

### 1.1 Mandatory Safety Rules
> [!CAUTION]
> **Mechanical Pinch Points**: The 10:1 capstan cable drive and sector wheels generate over $250\text{ N}$ of linear cable tension. Keep hands and tools clear of the sector wheel travel arc during power-up and calibration.

> [!WARNING]
> **Electrical Isolation**: Never bridge the 7.4V servo bus ground to the 22.2V motor ground at any point other than the opto-isolated FCC interface. Never apply 6S main battery power without 50W dynamic brake resistors connected to each ODrive S1.

### 1.2 Initial Power-Up Checklist
1. **Bench Supply Step-In**: Power the 22.2V motor bus initially using a current-limited DC bench supply set to **$22.0\text{V}$ and $2.0\text{A}$ current limit**.
2. **Servo Bus Check**: Apply 7.4V from the 2S 3000mAh pack to the servo bus board. Verify with a DMM:
   * Voltage: $+7.4\text{V} \pm 0.4\text{V}$.
   * Quiescent Current: $< 0.8\text{A}$ across all 16 servos at idle.
3. **Flight Computer Boot**: Apply 5V power to the Raspberry Pi 4B and Pixhawk 4 Mini. Verify solid green 3.3V/5V power LEDs and active heartbeat on the Waveshare CAN-FD Hat.

---

## 2. ODrive S1 CAN-FD Configuration

The Mark 1 employs **eight ODrive S1 controllers** communicating over CAN-FD at **1 Mbps nominal / 8 Mbps data bitrates** via `mco_odrive_interface.py`.

```
CAN Bus Topology:
  [ Pi 4B + Waveshare Hat ] <--- 1 Mbps / 8 Mbps ---> [ S1 #0 ] <-> [ S1 #1 ] <-> ... <-> [ S1 #7 ]
  [ 120-Ohm Terminator ON ]                                                    [ 120-Ohm Terminator ON ]
```

### 2.1 Parameter Configuration via `odrivetool`
Connect to each ODrive S1 individually via USB and execute the following configuration script:

```python
# Configure Motor Parameters for T-Motor MN5008 ~200KV
odrv0.config.dc_bus_overvoltage_trip_level = 26.5  # 6S LiPo max
odrv0.config.dc_bus_undervoltage_trip_level = 19.0 # 6S LiPo min cut-off
odrv0.config.dc_max_positive_current = 35.0        # Amps peak
odrv0.config.dc_max_negative_current = -15.0       # Regen braking limit

# Dynamic Brake Resistor
odrv0.config.brake_resistor0.enable = True
odrv0.config.brake_resistor0.resistance = 2.0      # Ohms (50W resistor)

# Motor Physics (MN5008)
odrv0.axis0.config.motor.motor_type = MotorType.HIGH_CURRENT
odrv0.axis0.config.motor.pole_pairs = 12           # 24N28P or 12 pole pairs
odrv0.axis0.config.motor.torque_constant = 0.048   # N*m/A for 200KV
odrv0.axis0.config.motor.current_soft_max = 25.0   # Continuous Amps
odrv0.axis0.config.motor.current_hard_max = 35.0   # Peak Amps

# Kinematic Limits (10:1 Capstan Reduction)
# ODrive S1 native velocity is in turns/s (revolutions/s).
# At 10:1 reduction, 3.5 Hz flapping requires 35.0 turns/s on the motor shaft.
# vel_limit = 40.0 turns/s (251.3 rad/s or 2400 RPM) provides 14.3% dynamic margin.
odrv0.axis0.controller.config.vel_limit = 40.0     # turns/s (allows 3.5 Hz flapping at 10:1)
odrv0.axis0.controller.config.control_mode = ControlMode.POSITION_CONTROL
odrv0.axis0.controller.config.input_mode = InputMode.POS_FILTER

# CAN-FD Bus Settings (Node ID = wing_index: 0 to 7)
odrv0.can.config.node_id = 0                       # Set 0..7 per wing
odrv0.can.config.baud_rate = 1000000               # 1 Mbps Nominal
odrv0.can.config.enable = True

odrv0.save_configuration()
```

### 2.2 Calibration & Phase Alignment
Run calibration with the Dyneema drive cable disconnected from the motor drum:
```python
odrv0.axis0.requested_state = AxisState.FULL_CALIBRATION_SEQUENCE
```
Verify motor emits a brief calibration tone, turns 1 revolution forward and backward, and reports `error = 0x0000`. Set `pre_calibrated = True` and reconnect the capstan cable.

---

## 3. Servo Travel & Frequency Programming (AGF-SPV3)

The 16× AGFRC IB53BHU servos must be programmed before final mechanical linkage attachment.

> [!IMPORTANT]
> **Prevent ADC Saturation**: The factory default servo travel is $180^\circ$, which outputs $0\text{ mV}$ to $3300\text{ mV}$ on the analog feedback wire. This saturates the MCP3208 SPI ADCs at the extremes. All servos **must be programmed to 170° max travel**.

### 3.1 Programming Procedure
1. Connect the `AGF-SPV3` USB programmer to a PC running the AGF-RC Servo Software.
2. Plug the servo lead into the programmer.
3. Apply the following register values:
   * **Angle Range**: Set to **$170^\circ$**.
   * **PWM Frequency**: Set to **$333\text{ Hz}$** (matches `mco_servo_interface.py` pigpio output).
   * **Deadband**: Set to **$1.0\ \mu\text{s}$**.
   * **Overload Protection**: Enabled (trips at $> 4.0\text{A}$ for $> 2.0\text{ s}$).
4. Save parameters to servo EEPROM. Verify that sweeping PWM from $500\ \mu\text{s}$ to $2500\ \mu\text{s}$ produces exactly $170^\circ$ physical shaft rotation.

---

## 4. MCP3208 12-Bit SPI ADC Feedback Calibration

Position feedback from each wing's pitch and sweep servos is monitored in real time by two Microchip MCP3208 8-channel 12-bit SPI ADCs connected to the Raspberry Pi SPI1 interface:
* **ADC #1 (SPI1 CE1)**: 8× Pitch Servo Position Feedback (Channels 0–7).
* **ADC #2 (SPI1 CE0)**: 8× Sweep Servo Position Feedback (Channels 0–7).

### 4.1 Calibration Transfer Function
With servos programmed to $170^\circ$, the analog feedback wire yields:
* Minimum Travel ($0^\circ$): $300\text{ mV} \to 372\text{ ADC counts}$.
* Maximum Travel ($170^\circ$): $3000\text{ mV} \to 3723\text{ ADC counts}$.

$$\theta_{\text{actual}} = \left( \frac{\text{ADC\_Counts} - 372}{3723 - 372} \right) \times 170.0^\circ$$

### 4.2 Calibration Verification
Execute the automated feedback verification script:
```bash
python3 firmware/mco_preflight_checks.py --adc-verify
```
Verify that all 16 channels respond proportionally when wings are manually deflected, with zero deadband or channel cross-talk.

---

## 5. Autopilot & Sensor Bridge Calibration

### 5.1 MS4525DO Digital Pitot Zeroing
1. Cover the nose pitot tube with a protective foam sheath to eliminate ambient drafts.
2. Power the Pixhawk 4 Mini and launch the MAVLink bridge:
   ```bash
   python3 firmware/mco_mavlink_bridge.py --zero-pitot
   ```
3. Verify airspeed reads $0.00\text{ m/s} \pm 0.05\text{ m/s}$ in zero wind. Remove the sheath and blow gently into the dynamic port to confirm instantaneous velocity rise.

### 5.2 Benewake TF-Luna LiDAR Zeroing
1. Place the aircraft on level ground resting on its carbon landing skids.
2. Read the ground distance register over I2C (address `0x10`):
   ```bash
   python3 -c "import smbus; b=smbus.SMBus(1); print('Lidar:', b.read_i2c_block_data(0x10, 0, 9))"
   ```
3. Set the ground elevation offset so height reads **$0.02\text{ m}$ (ground taxi height)** at rest.

---

## 6. Full Pre-Flight Built-In-Test (BIT) & Automated Bench Qualification

Before any flight operation or test stand trial, execute the complete pre-flight self-test:

```bash
python3 firmware/mco_preflight.py
```

### Expected Pass Criteria
```
============================================================
  MCO-1D MARK 1 PRE-FLIGHT BUILT-IN TEST
============================================================
  [PASS] Raspberry Pi 4B FCC Executive Environment
  [PASS] Waveshare MCP2518FD CAN-FD Interface (can0: UP 1M/8M)
  [PASS] 8/8 ODrive S1 Nodes Active (0-7, errors=0x0000)
  [PASS] 16/16 AGFRC Servos Responding on pigpio 333 Hz
  [PASS] MCP3208 12-Bit SPI ADCs Calibrated (16 Channels Valid)
  [PASS] MS4525DO Pitot-Static Sensor Valid (Airspeed: 0.0 m/s)
  [PASS] TF-Luna LiDAR Ground Distance Valid (AGL: 0.02 m)
  [PASS] Pixhawk 4 Mini MAVLink Bridge Connected (921600 baud)
  [PASS] CG Static Model in Envelope: 323.8 mm (26.5% MAC)
------------------------------------------------------------
  AIRCRAFT SYSTEM STATUS: GO FOR FLIGHT
============================================================
```

### 6.1 Automated Laboratory Bench Qualification (TC-01)
Prior to mounting the airframe onto the physical load stand, run the automated TC-01 qualification harness:
```bash
python3 scripts/run_tc01_bench_checkout.py
```
This verifies all 10 physical calibration gates (static thrust $\ge 35.0\,\text{N}$, motor power factor $0.127$, loaded servo slew floor, barometric density coupling, $P_s$ climb ceiling, CF-1 dispatch safety, and mountain trigger detection), confirming full system readiness.

