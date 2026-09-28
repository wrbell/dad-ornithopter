# Firmware Archive Directory

This directory stores archived, superseded baseline snapshots and legacy prototype files preserved for historical reference and engineering audit trace:

1. **`mco_wave_library_clean.py`** (3.4 MB):
   - Historical baseline snapshot of the wave kinematics library prior to multi-layer aerodynamic and safety patch integration.
   - Superseded by `firmware/mco_wave_library.py`.

2. **`mco_wave_library_pre_audit.py`** (3.7 MB):
   - Pre-audit development snapshot of the wave kinematics library.
   - Superseded by `firmware/mco_wave_library.py`.

3. **`wing_drone_firmware.py`** (29 KB):
   - Early prototype standalone firmware script designed for 24-servo PCA9685 I2C pwm driver boards.
   - Superseded by `firmware/mco_main.py` (ODrive S1 CAN-FD + AGFRC IB53BHU programmable servos).
