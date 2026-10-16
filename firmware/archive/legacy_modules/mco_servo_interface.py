"""
Servo Interface (PWM via RPi GPIO)
====================================
Controls 16 servos: 8 sweep (ZOSKAY DS3225) + 8 pitch (Emax ES08MD II)
Uses RPi hardware PWM via pigpio daemon.

Install: sudo apt install pigpio python3-pigpio
Start daemon: sudo systemctl enable pigpiod && sudo systemctl start pigpiod

Servo specs:
  ZOSKAY DS3225: 500-2500 μs, 270° range, centre=1500μs
  Emax ES08MD II: 500-2500 μs, 180° range, centre=1500μs
"""

try:
    import pigpio
    PIGPIO_AVAILABLE = True
except ImportError:
    PIGPIO_AVAILABLE = False
    print("[WARN] pigpio not available — servo in simulation mode")

# ─── Pin assignments (BCM numbering) ────────────────────────────
# Adjust these to match your wiring
SWEEP_PINS = [4, 17, 27, 22, 5, 6, 13, 19]    # W1p,W1s,W2p,W2s,W3p,W3s,W4p,W4s
PITCH_PINS = [18, 23, 24, 25, 12, 16, 20, 21]

SERVO_MIN_US  = 500
SERVO_MAX_US  = 2500
SERVO_CTR_US  = 1500

# Angle→μs conversion
SWEEP_RANGE_DEG = 270.0    # ZOSKAY full range
PITCH_RANGE_DEG = 180.0    # Emax full range

def sweep_deg_to_us(angle_deg):
    """Convert sweep angle (0=forward, 210=stow) to μs."""
    # Map 0-270° to 500-2500μs
    us = SERVO_MIN_US + (angle_deg / SWEEP_RANGE_DEG) * (SERVO_MAX_US - SERVO_MIN_US)
    return int(max(SERVO_MIN_US, min(SERVO_MAX_US, us)))

def pitch_deg_to_us(angle_deg):
    """Convert pitch angle (-5° to +50°) to μs. Centre=0° = 1500μs."""
    us = SERVO_CTR_US + (angle_deg / (PITCH_RANGE_DEG/2)) * (SERVO_MAX_US - SERVO_CTR_US)
    return int(max(SERVO_MIN_US, min(SERVO_MAX_US, us)))


class ServoInterface:
    def __init__(self):
        self._pi = None
        if PIGPIO_AVAILABLE:
            try:
                self._pi = pigpio.pi()
                if not self._pi.connected:
                    print("[Servo] pigpiod not running — simulation mode")
                    self._pi = None
                else:
                    for pin in SWEEP_PINS + PITCH_PINS:
                        self._pi.set_mode(pin, pigpio.OUTPUT)
                    print("[Servo] pigpio connected, 16 channels ready")
            except Exception as e:
                print(f"[Servo] pigpio error: {e}")

    def set_sweep(self, wing_idx, angle_deg):
        """Set sweep angle for one wing (0=forward, 210=stow)."""
        us = sweep_deg_to_us(angle_deg)
        if self._pi:
            self._pi.set_servo_pulsewidth(SWEEP_PINS[wing_idx], us)

    def set_pitch(self, wing_idx, angle_deg):
        """Set pitch/AoA angle for one wing."""
        us = pitch_deg_to_us(angle_deg)
        if self._pi:
            self._pi.set_servo_pulsewidth(PITCH_PINS[wing_idx], us)

    def set_all_sweep(self, angles_deg):
        for i, a in enumerate(angles_deg):
            self.set_sweep(i, a)

    def set_all_pitch(self, angles_deg):
        for i, a in enumerate(angles_deg):
            self.set_pitch(i, a)

    def centre_all(self):
        """Move all servos to neutral position."""
        self.set_all_sweep([60.0]*8)   # 60° = forward deployed
        self.set_all_pitch([0.0]*8)

    def stow_all(self, rate_degps=30.0, step_hz=20):
        """
        Progressively stow all wings to 210°.
        Stow order: W4 → W3 → W2 → W1 (firmware-locked).
        """
        import time
        stow_order = [6, 7, 4, 5, 2, 3, 0, 1]  # W4p,W4s,W3p,W3s,W2p,W2s,W1p,W1s
        current = [60.0] * 8
        target  = 210.0
        step_deg = rate_degps / step_hz

        for idx in stow_order:
            while current[idx] < target:
                current[idx] = min(current[idx] + step_deg, target)
                self.set_sweep(idx, current[idx])
                time.sleep(1.0/step_hz)
            time.sleep(0.2)   # 0.2s offset between pairs (firmware spec)

    def close(self):
        if self._pi:
            for pin in SWEEP_PINS + PITCH_PINS:
                self._pi.set_servo_pulsewidth(pin, 0)  # turn off PWM
            self._pi.stop()
