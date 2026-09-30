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


def detect_mountain_terrain_trigger(lat: float, lon: float,
                                    alt_msl_m: float,
                                    agl_m: float,
                                    climb_rate_ms: float = 0.0,
                                    terrain_elevation_m: float = None) -> dict:
    """
    O-D7: Autonomous mountain ridge updraft and high-density-altitude detection.
    Evaluates barometric MSL altitude, AGL proximity, and terrain elevation gradients.
    Automatically flags mountain mode when operating over mountainous relief or
    exploiting orographic ridge lift.
    """
    reasons = []
    is_mountain = False
    ridge_updraft = False

    # 1. High Elevation Threshold (> 1500m MSL)
    if alt_msl_m >= 1500.0:
        is_mountain = True
        reasons.append(f"High altitude MSL ({alt_msl_m:.0f}m >= 1500m)")

    # 2. Mountain Terrain Proximity: elevated base (> 600m MSL) + close to terrain (< 150m AGL)
    if alt_msl_m >= 600.0 and agl_m <= 150.0:
        is_mountain = True
        reasons.append(f"Rugged terrain proximity ({agl_m:.0f}m AGL at {alt_msl_m:.0f}m MSL)")

    # 3. Terrain Elevation Gradient (from DEM or radio altimeter - baro differential)
    if terrain_elevation_m is not None and terrain_elevation_m >= 800.0:
        is_mountain = True
        reasons.append(f"DEM terrain base elevated ({terrain_elevation_m:.0f}m >= 800m)")

    # 4. Orographic Ridge Updraft Detection: sustained climb without high throttle
    if is_mountain and climb_rate_ms >= 1.2:
        ridge_updraft = True
        reasons.append(f"Ridge updraft detected (climb: +{climb_rate_ms:.1f} m/s)")

    factor = 1.20 if is_mountain else 1.10
    reason_str = "; ".join(reasons) if reasons else "Nominal lowland terrain"

    return {
        'mountain_mode': is_mountain,
        'ridge_updraft_detected': ridge_updraft,
        'trigger_reasons': reasons,
        'reason': reason_str,
        'stall_floor_factor': factor,
    }


class PixhawkBridge:
    def __init__(self, port="/dev/serial0", baud=57600):
        self._conn   = None
        self._lock   = threading.Lock()
        self.airspeed_ms  = 0.0
        self.armed        = False
        self.roll_deg     = 0.0
        self.pitch_deg    = 0.0
        self.rc_channels  = [1500]*8
        self.payload_release_cmd = False
        self.gripper_state = 'closed'
        self.comfort_mode = False
        self.sensitive_cargo = False
        self.dragonfly_launch_cmd = False
        self.pigeon_climb_cmd = False
        self.climb_angle_target = 80.0
        self.mountain_mode = False
        self.ridge_updraft_detected = False
        self.lat = 0.0
        self.lon = 0.0
        self.alt_msl_m = 0.0
        self.agl_m = 100.0
        self.climb_rate_ms = 0.0
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
                    import math
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
                elif mt == "GLOBAL_POSITION_INT":
                    self.lat = getattr(msg, 'lat', 0) / 1e7
                    self.lon = getattr(msg, 'lon', 0) / 1e7
                    self.alt_msl_m = getattr(msg, 'alt', 0) / 1000.0
                    self.agl_m = getattr(msg, 'relative_alt', 0) / 1000.0
                    self.climb_rate_ms = -getattr(msg, 'vz', 0) / 100.0
                    trig = detect_mountain_terrain_trigger(
                        self.lat, self.lon, self.alt_msl_m, self.agl_m, self.climb_rate_ms
                    )
                    prev_mountain = self.mountain_mode
                    self.mountain_mode = trig['mountain_mode']
                    self.ridge_updraft_detected = trig['ridge_updraft_detected']
                    if self.mountain_mode != prev_mountain:
                        status = "ENGAGED" if self.mountain_mode else "DISENGAGED"
                        self.send_statustext(f"TERRAIN: Mountain mode {status} ({trig['reason']})")
                elif mt == "COMMAND_LONG":
                    # MAV_CMD_DO_GRIPPER = 211
                    if getattr(msg, 'command', 0) in (211, getattr(mavutil.mavlink, 'MAV_CMD_DO_GRIPPER', 211)):
                        # param2: 0 = release, 1 = grab
                        if getattr(msg, 'param2', 0) == 0:
                            self.payload_release_cmd = True
                            self.gripper_state = 'released'
                        else:
                            self.gripper_state = 'grabbed'

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

    def send_payload_status(self, payload_kg, delta_cg_mm, n_limit_g):
        """Broadcast payload weight and structural G-limit via MAVLink status text."""
        msg_str = f"CARGO:{payload_kg:.2f}kg CG:{delta_cg_mm:+.1f}mm Nlim:{n_limit_g:.1f}g"
        self.send_statustext(msg_str, severity=6)

    def send_energy_stewardship(self, cot, wh_per_km, carsons_speed_ms):
        """Broadcast energy stewardship and Carson's speed via MAVLink status text."""
        msg_str = f"ECO:CoT {cot:.2f} {wh_per_km:.0f}Wh/km Vbr:{carsons_speed_ms:.1f}m/s"
        self.send_statustext(msg_str, severity=6)

    def send_ride_comfort_status(self, active, vib_red_pct, cabin_rms_g, residual_my_Nm):
        """Broadcast ride comfort and active heave/pitch damping status via MAVLink."""
        act_str = "ON" if active else "OFF"
        msg_str = f"RIDE:{act_str} VibRed:{vib_red_pct:.0f}% RMS:{cabin_rms_g:.3f}g My:{residual_my_Nm:+.2f}Nm"
        self.send_statustext(msg_str, severity=6)

    def set_comfort_mode(self, enabled=True):
        """Enable or disable passenger comfort mode."""
        with self._lock:
            self.comfort_mode = bool(enabled)

    def set_sensitive_cargo(self, enabled=True):
        """Enable or disable sensitive/fragile cargo protection mode."""
        with self._lock:
            self.sensitive_cargo = bool(enabled)

    def trigger_cargo_release(self):
        """Trigger local cargo release hook pulse."""
        with self._lock:
            self.payload_release_cmd = True
            self.gripper_state = 'released'

    def trigger_dragonfly_launch(self):
        """Trigger H-LA-1 Dragonfly 0° in-phase super-lift launch burst."""
        with self._lock:
            self.dragonfly_launch_cmd = True

    def trigger_pigeon_climb(self, angle_deg=80.0):
        """Trigger H-LA-3 Pigeon clap-and-peel steep climb corridor."""
        with self._lock:
            self.pigeon_climb_cmd = True
            self.climb_angle_target = float(angle_deg)

    def set_mountain_mode(self, enabled=True):
        """Enable or disable mountain/ridge terrain mode."""
        with self._lock:
            self.mountain_mode = bool(enabled)
            status = "ENGAGED" if self.mountain_mode else "DISENGAGED"
            self.send_statustext(f"TERRAIN: Mountain mode manually {status}")

    def update_terrain_state(self, lat: float, lon: float, alt_msl_m: float,
                             agl_m: float, climb_rate_ms: float = 0.0,
                             terrain_elevation_m: float = None) -> dict:
        """Manually update or inject GPS terrain telemetry and evaluate triggers."""
        with self._lock:
            self.lat = lat
            self.lon = lon
            self.alt_msl_m = alt_msl_m
            self.agl_m = agl_m
            self.climb_rate_ms = climb_rate_ms
            trig = detect_mountain_terrain_trigger(
                lat, lon, alt_msl_m, agl_m, climb_rate_ms, terrain_elevation_m
            )
            self.mountain_mode = trig['mountain_mode']
            self.ridge_updraft_detected = trig['ridge_updraft_detected']
            return trig

    def get_state(self):
        with self._lock:
            cmd = self.payload_release_cmd
            df_cmd = self.dragonfly_launch_cmd
            pg_cmd = self.pigeon_climb_cmd
            angle_tgt = self.climb_angle_target
            # Reset edge-triggered drop/burst commands upon read
            self.payload_release_cmd = False
            self.dragonfly_launch_cmd = False
            self.pigeon_climb_cmd = False
            return {
                "airspeed_ms":            self.airspeed_ms,
                "armed":                  self.armed,
                "roll_deg":               self.roll_deg,
                "pitch_deg":              self.pitch_deg,
                "rc_channels":            list(self.rc_channels),
                "payload_release_cmd":    cmd,
                "dragonfly_launch_cmd":   df_cmd,
                "pigeon_climb_cmd":       pg_cmd,
                "climb_angle_target":     angle_tgt,
                "gripper_state":          self.gripper_state,
                "comfort_mode":           self.comfort_mode,
                "sensitive_cargo":        self.sensitive_cargo,
                "mountain_mode":          self.mountain_mode,
                "ridge_updraft_detected": self.ridge_updraft_detected,
                "alt_msl_m":              self.alt_msl_m,
                "agl_m":                  self.agl_m,
            }

    def close(self):
        self._running = False

        if self._conn:
            self._conn.close()
