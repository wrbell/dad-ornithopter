#!/usr/bin/env python3
"""
MCO-1D MAVLink Bridge
======================
Reads sensor data from Pixhawk via MAVLink (TELEM2 @ 921600 baud)
and publishes it as a Python dict at 50 Hz for the wave library.

Also forwards wave library control outputs back to Pixhawk
for logging and ground station telemetry.

Usage (standalone test):
    python3 mco_mavlink.py --port /dev/ttyAMA0 --baud 921600

Integration:
    from mco_mavlink import MCOMAVLink
    bridge = MCOMAVLink('/dev/ttyAMA0', 921600)
    bridge.start()
    sensors = bridge.get_sensors()   # call every tick
    bridge.send_heartbeat()

MAVLink messages consumed:
    ATTITUDE         → pitch_deg_imu, roll_deg_imu, heading_deg,
                       pitch_rate_dps, roll_rate_dps, yaw_rate_radps
    GLOBAL_POSITION_INT → lat, lon, h_agl_m (alt above home)
    VFR_HUD          → V_ms (airspeed), sink_rate_ms
    SYS_STATUS       → bus_voltage_v, motor_I_A
    RANGEFINDER      → rangefinder_m (h_agl_m supplement)
    RC_CHANNELS      → lever_pct, pitch_stick, roll_stick, yaw_stick
    SCALED_IMU2      → accel_z_g, imu_rms_g
    BATTERY_STATUS   → bus_voltage_v (supplement)
"""

import time
import threading
import math
import sys
import argparse

try:
    from pymavlink import mavutil
    HAS_MAVLINK = True
except ImportError:
    HAS_MAVLINK = False
    print("pymavlink not installed. Run: pip install pymavlink")


# RC channel mapping (adjust to match Pixhawk RC setup)
RC_LEVER_CH   = 3    # throttle channel → wave function lever
RC_PITCH_CH   = 2    # pitch → pitch stick
RC_ROLL_CH    = 1    # roll → roll stick
RC_YAW_CH     = 4    # yaw → yaw stick

# RC calibration (adjust after radio calibration)
RC_MIN        = 1000   # microseconds (full left / down / aft)
RC_MID        = 1500   # microseconds (centre)
RC_MAX        = 2000   # microseconds (full right / up / fwd)

def rc_normalise(pwm, ch_type='bipolar'):
    """Convert RC PWM to normalised value."""
    pwm = max(RC_MIN, min(RC_MAX, pwm))
    if ch_type == 'unipolar':   # 0.0 to 1.0 (lever)
        return (pwm - RC_MIN) / (RC_MAX - RC_MIN) * 100.0
    else:                        # -1.0 to +1.0 (sticks)
        return (pwm - RC_MID) / (RC_MAX - RC_MID)


class MCOMAVLink:
    """
    MAVLink bridge for MCO-1D.
    Runs a background thread reading MAVLink at 100+ Hz.
    Provides get_sensors() returning the wave library sensor dict at 50Hz.
    """

    def __init__(self, port='/dev/ttyAMA0', baud=921600):
        self.port   = port
        self.baud   = baud
        self._conn  = None
        self._lock  = threading.Lock()
        self._alive = True
        self._t     = None

        # Sensor state — updated by MAVLink thread
        self._s = {
            'V_ms':             5.0,
            'h_agl_m':          0.0,
            'pitch_deg_imu':    0.0,
            'roll_deg_imu':     0.0,
            'heading_deg':      0.0,
            'pitch_rate_dps':   0.0,
            'roll_rate_dps':    0.0,
            'yaw_rate_radps':   0.0,
            'sink_rate_ms':     0.0,
            'accel_z_g':        1.0,
            'imu_rms_g':        0.02,
            'bus_voltage_v':    14.8,
            'servo_voltage_v':  7.4,
            'motor_I_A':        0.0,
            'motor_currents_a': [0.0] * 8,
            'rangefinder_m':    2.0,
            'lat':              None,
            'lon':              None,
            'V_wind_ms':        0.0,
            'wind_bearing_deg': 0.0,
            'wind_speed_z_ms':  0.0,
            'wind_valid':       False,
            'lever_pct':        50.0,
            'pitch_stick':      0.0,
            'roll_stick':       0.0,
            'yaw_stick':        0.0,
            'pitch_stick_rate': 0.0,
            'roll_stick_rate':  0.0,
            'rc_signal_valid':  False,
            'imu_ok':           False,
            'baro_ok':          False,
            'time_to_impact_s': 99.0,
            'Ps_measured':      0.0,
            'dt_s':             0.02,
        }
        self._prev_pitch = 0.0
        self._prev_roll  = 0.0
        self._prev_t     = time.monotonic()

    def start(self):
        """Start the MAVLink reader thread."""
        if not HAS_MAVLINK:
            print("MAVLink bridge: pymavlink not available — using defaults")
            return
        self._t = threading.Thread(target=self._run, daemon=True,
                                    name='mco-mavlink')
        self._t.start()
        print(f"MAVLink bridge started on {self.port} @ {self.baud}")

    def stop(self):
        self._alive = False

    def _connect(self):
        """Connect to Pixhawk. Retries every 5 seconds."""
        while self._alive:
            try:
                self._conn = mavutil.mavlink_connection(
                    self.port, baud=self.baud,
                    source_system=255, source_component=0)
                self._conn.wait_heartbeat(timeout=10)
                print(f"MAVLink: connected (system {self._conn.target_system})")
                return True
            except Exception as e:
                print(f"MAVLink: connection failed ({e}) — retrying in 5s")
                time.sleep(5)
        return False

    def _run(self):
        if not self._connect():
            return

        while self._alive:
            try:
                msg = self._conn.recv_match(blocking=True, timeout=0.1)
                if msg is None:
                    continue
                self._process(msg)
            except Exception as e:
                print(f"MAVLink read error: {e} — reconnecting")
                time.sleep(2)
                self._connect()

    def _process(self, msg):
        """Process a single MAVLink message."""
        t   = msg.get_type()
        now = time.monotonic()

        with self._lock:
            s = self._s

            if t == 'ATTITUDE':
                s['pitch_deg_imu']  = math.degrees(msg.pitch)
                s['roll_deg_imu']   = math.degrees(msg.roll)
                s['heading_deg']    = math.degrees(msg.yaw) % 360
                s['pitch_rate_dps'] = math.degrees(msg.pitchspeed)
                s['roll_rate_dps']  = math.degrees(msg.rollspeed)
                s['yaw_rate_radps'] = msg.yawspeed
                s['imu_ok']         = True

                # Stick rate estimation
                dt = now - self._prev_t
                if dt > 0:
                    s['pitch_stick_rate'] = (s['pitch_stick'] - self._prev_pitch) / dt
                    s['roll_stick_rate']  = (s['roll_stick']  - self._prev_roll)  / dt
                self._prev_pitch = s['pitch_stick']
                self._prev_roll  = s['roll_stick']
                self._prev_t     = now

            elif t == 'VFR_HUD':
                s['V_ms']          = float(msg.airspeed)
                s['sink_rate_ms']  = -float(msg.climb)   # positive = descending
                s['heading_deg']   = float(msg.heading)
                # Confirm pitot is live — prevents hover mode ground runaway
                try:
                    from mco_wave_library import pitot_update
                    pitot_update(float(msg.airspeed))
                except Exception:
                    pass

            elif t == 'GLOBAL_POSITION_INT':
                s['h_agl_m']      = msg.relative_alt / 1000.0  # mm → m
                s['lat']          = msg.lat / 1e7
                s['lon']          = msg.lon / 1e7
                s['baro_ok']      = True
                # Velocity in body/earth frame (cm/s → m/s)
                s['vel_north_ms'] = getattr(msg, 'vx', 0) / 100.0
                s['vel_east_ms']  = getattr(msg, 'vy', 0) / 100.0
                s['vel_down_ms']  = getattr(msg, 'vz', 0) / 100.0
                s['gps_ok']       = True

            elif t == 'RANGEFINDER':
                s['rangefinder_m'] = float(msg.distance)
                # Use rangefinder for h_agl below 5m (more accurate near ground)
                if msg.distance < 5.0:
                    s['h_agl_m'] = float(msg.distance)

            elif t == 'SCALED_IMU2':
                # Compute approximate IMU RMS from acceleration vector
                ax = msg.xacc / 1000.0   # mG → G
                ay = msg.yacc / 1000.0
                az = msg.zacc / 1000.0
                s['accel_z_g'] = az
                s['imu_rms_g'] = math.sqrt(ax**2 + ay**2 + (az-1.0)**2)

            elif t in ('BATTERY_STATUS', 'SYS_STATUS'):
                if hasattr(msg, 'voltage_battery'):
                    v = msg.voltage_battery / 1000.0  # mV → V
                    if 8.0 < v < 30.0:
                        s['bus_voltage_v'] = v
                elif hasattr(msg, 'battery_remaining'):
                    pass  # percentage only, skip

            elif t == 'RC_CHANNELS':
                # Map RC channels to control inputs
                try:
                    # Lever: throttle channel, 0-100%
                    s['lever_pct']   = rc_normalise(msg.chan3_raw, 'unipolar')
                    s['pitch_stick'] = rc_normalise(msg.chan2_raw, 'bipolar')
                    s['roll_stick']  = rc_normalise(msg.chan1_raw, 'bipolar')
                    s['yaw_stick']   = rc_normalise(msg.chan4_raw, 'bipolar')
                    s['rc_signal_valid'] = (msg.rssi > 0)
                except Exception:
                    pass

            elif t == 'WIND':
                # MAVLink WIND message (ID 168): direction (deg from), speed (m/s), speed_z (m/s)
                s['V_wind_ms']        = float(getattr(msg, 'speed', 0.0))
                s['wind_bearing_deg'] = float(getattr(msg, 'direction', 0.0))
                s['wind_speed_z_ms']  = float(getattr(msg, 'speed_z', 0.0))
                s['wind_valid']       = True

            # Compute time to impact from sink rate and height
            h   = s['h_agl_m']
            sk  = s['sink_rate_ms']
            s['time_to_impact_s'] = (h / sk) if sk > 0.1 else 99.0

    def get_sensors(self):
        """Return current sensor dict (copy). Thread-safe."""
        with self._lock:
            d = dict(self._s)
        d['dt_s'] = 0.02   # 50Hz tick rate
        return d

    def send_status(self, result_dict):
        """
        Send wave library result dict back to Pixhawk as named
        value messages for GCS display. Non-blocking.
        """
        if self._conn is None:
            return
        try:
            freq = float(result_dict.get('freq_hz', 0))
            self._conn.mav.named_value_float_send(
                int(time.monotonic() * 1000) & 0xFFFFFFFF,
                b'MCO_FREQ\x00\x00',
                freq)
        except Exception:
            pass


def main():
    """Standalone test — prints sensor dict at 2 Hz."""
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', default='/dev/ttyAMA0')
    parser.add_argument('--baud', type=int, default=921600)
    args = parser.parse_args()

    bridge = MCOMAVLink(args.port, args.baud)
    bridge.start()

    print("MAVLink bridge running. Ctrl+C to stop.")
    print("Showing sensor dict every 0.5s:\n")

    try:
        while True:
            s = bridge.get_sensors()
            print(f"  V={s['V_ms']:.1f}m/s  h={s['h_agl_m']:.2f}m  "
                  f"pitch={s['pitch_deg_imu']:.1f}°  "
                  f"roll={s['roll_deg_imu']:.1f}°  "
                  f"batt={s['bus_voltage_v']:.1f}V  "
                  f"rc={'OK' if s['rc_signal_valid'] else 'NO'}")
            time.sleep(0.5)
    except KeyboardInterrupt:
        bridge.stop()
        print("\nStopped.")


if __name__ == '__main__':
    main()
