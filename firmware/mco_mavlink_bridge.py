"""
MAVLink Bridge: RPi Zero 2W ↔ Pixhawk Mini
============================================
The RPi runs the wave library and sends velocity commands to ODrives.
The Pixhawk Mini runs ArduPlane/ArduCopter as safety FCC.
They communicate via MAVLink over UART (/dev/serial0 on RPi).

RPi role: wave library, ODrive CAN commands, servo PWM
Pixhawk role: IMU fusion, GPS, RC input, stall protection, failsafe

Data from Pixhawk to RPi (read by bridge):
  - Airspeed (pitot), attitude, GPS, RC channels, armed state

Data from RPi to Pixhawk (sent by bridge):
  - Heartbeat (keep alive)
  - Status text (mode name, fault messages)

Required: pymavlink  (pip install pymavlink)
UART: Pixhawk TELEM2 → RPi /dev/serial0, 57600 baud
"""

import time, threading

try:
    from pymavlink import mavutil
    MAV_AVAILABLE = True
except ImportError:
    MAV_AVAILABLE = False
    print("[WARN] pymavlink not installed — MAVLink in simulation mode")


class PixhawkBridge:
    def __init__(self, port="/dev/serial0", baud=57600):
        self._conn   = None
        self._lock   = threading.Lock()
        self.airspeed_ms  = 0.0
        self.armed        = False
        self.roll_deg     = 0.0
        self.pitch_deg    = 0.0
        self.rc_channels  = [1500]*8
        self._running     = False

        if MAV_AVAILABLE:
            try:
                self._conn = mavutil.mavlink_connection(
                    port, baud=baud,
                    source_system=1, source_component=200
                )
                print(f"[MAVLink] Connected to Pixhawk on {port}")
                self._running = True
                t = threading.Thread(target=self._recv_loop, daemon=True)
                t.start()
            except Exception as e:
                print(f"[MAVLink] Connection failed: {e} — simulation mode")

    def _recv_loop(self):
        while self._running and self._conn:
            msg = self._conn.recv_match(blocking=True, timeout=0.1)
            if msg is None: continue
            mt = msg.get_type()
            with self._lock:
                if mt == "VFR_HUD":
                    self.airspeed_ms = msg.airspeed
                elif mt == "ATTITUDE":
                    self.roll_deg  = math.degrees(msg.roll)
                    self.pitch_deg = math.degrees(msg.pitch)
                elif mt == "HEARTBEAT":
                    # armed if bit 7 of base_mode set
                    self.armed = bool(msg.base_mode & 128)
                elif mt == "RC_CHANNELS":
                    self.rc_channels = [
                        msg.chan1_raw, msg.chan2_raw, msg.chan3_raw,
                        msg.chan4_raw, msg.chan5_raw, msg.chan6_raw,
                        msg.chan7_raw, msg.chan8_raw,
                    ]

    def send_heartbeat(self):
        if self._conn:
            self._conn.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
                mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                0, 0, 0
            )

    def send_statustext(self, text, severity=6):
        if self._conn:
            text_bytes = text[:50].encode().ljust(50, b"\0")
            self._conn.mav.statustext_send(severity, text_bytes)

    def get_state(self):
        with self._lock:
            return {
                "airspeed_ms":  self.airspeed_ms,
                "armed":        self.armed,
                "roll_deg":     self.roll_deg,
                "pitch_deg":    self.pitch_deg,
                "rc_channels":  list(self.rc_channels),
            }

    def close(self):
        self._running = False
        if self._conn:
            self._conn.close()
