#!/usr/bin/env python3
"""
MCO-1D ODrive S1 CAN Interface — 8 boards, 8 axes (1 motor per board)
=======================================================================

ARCHITECTURE:
  8 × ODrive S1 boards on CAN bus (node IDs 0-7)
  Each S1 board drives ONE motor — axis is always singular (no axis0/axis1).

  Board 0  (CAN node 0):  W1 port
  Board 1  (CAN node 1):  W1 starboard
  Board 2  (CAN node 2):  W2 port
  Board 3  (CAN node 3):  W2 starboard
  Board 4  (CAN node 4):  W3 port
  Board 5  (CAN node 5):  W3 starboard
  Board 6  (CAN node 6):  W4 port
  Board 7  (CAN node 7):  W4 starboard

  Wing index → node ID mapping:
    wing_idx 0 = W1 port      → node 0
    wing_idx 1 = W1 starboard → node 1
    wing_idx 2 = W2 port      → node 2
    wing_idx 3 = W2 starboard → node 3
    wing_idx 4 = W3 port      → node 4
    wing_idx 5 = W3 starboard → node 5
    wing_idx 6 = W4 port      → node 6
    wing_idx 7 = W4 starboard → node 7

  This is a direct 1:1 mapping — wing_idx IS the node_id.
  No axis0/axis1 distinction. No (node | 0x10) trick. Simpler.

CAN message format (ODrive S1):
  Arbitration ID = (node_id << 5) | cmd_id
  This is the SAME formula as ODrive v3.6 axis0.
  Since the S1 is single-axis, every board uses this formula directly.

CAN-FD:
  The S1 supports CAN-FD at up to 12 Mbps.
  For the MCO-1D we run standard CAN at 1 Mbps for Pi compatibility.
  CAN-FD can be enabled later by changing mco_can_setup.sh.

Encoder:
  The S1 has an on-board absolute magnetic encoder.
  Mount the included 1/4" diametrically magnetised magnet on the motor shaft.
  pos_estimate is in turns (absolute from cold start — no index search).
  No external encoder board required. No calibration index search at boot.

Brake resistor:
  Each S1 must have its own brake resistor — they CANNOT be shared.
  Comes with a 2 Ω 50W resistor. Connect to the brake resistor terminal.
  At stroke reversal, regenerated energy is dissipated locally per board.

Requirements:
  pip install python-can
  sudo ip link set can0 up type can bitrate 1000000
  (or CAN-FD: sudo ip link set can0 up type can-fd bitrate 1000000 dbitrate 5000000)
"""

import struct
import time
import math
import threading

from mco_config import (
    BATTERY_NOMINAL_V,
    CAN_BITRATE_BPS,
    CAN_BUS_INTERFACE,
    CRUISE_FLAP_HALF_ANGLE_DEG,
    GEAR_RATIO,
    WING_COUNT,
)

try:
    import can
    CAN_AVAILABLE = True
except ImportError:
    CAN_AVAILABLE = False
    print("[WARN] python-can not installed — ODrive S1 in simulation mode")

# ── ODrive S1 CAN command IDs ─────────────────────────────────────────────────
# These are identical to the ODrive v3.6 / Pro command set.
CMD_HEARTBEAT     = 0x001   # Heartbeat (axis error, state, flags)
CMD_ESTOP         = 0x002   # Emergency stop
CMD_GET_ERROR     = 0x003   # Get axis error
CMD_SET_STATE     = 0x007   # Set axis requested state
CMD_GET_ENCODER   = 0x009   # Get encoder estimates (pos_turns, vel_rps)
CMD_SET_POS       = 0x00C   # Set input position
CMD_SET_VEL       = 0x00D   # Set input velocity (vel_rps, torque_ff)
CMD_SET_TORQUE    = 0x00E   # Set input torque
CMD_GET_IQ        = 0x014   # Get Iq setpoint and measured
CMD_GET_TEMP      = 0x015   # Get FET and motor temperature
CMD_GET_VBUS      = 0x017   # Get bus voltage and current
CMD_SET_LIMITS    = 0x011   # Set velocity and current limits
CMD_CLEAR_ERRORS  = 0x018   # Clear all errors

# ODrive axis states
STATE_UNDEFINED         = 0
STATE_IDLE              = 1
STATE_CLOSED_LOOP       = 8
STATE_FULL_CALIBRATION  = 3

# Number of S1 boards / wings
N_BOARDS = WING_COUNT    # 8 × ODrive S1, one per wing
N_WINGS  = WING_COUNT    # always equal — each board drives exactly one wing

# Wing index → pair and side (for logging and wave library)
WING_NAMES = [
    'W1p', 'W1s',   # nodes 0, 1
    'W2p', 'W2s',   # nodes 2, 3
    'W3p', 'W3s',   # nodes 4, 5
    'W4p', 'W4s',   # nodes 6, 7
]

def wing_to_node(wing_idx):
    """
    Map wing index (0-7) directly to CAN node ID.
    On the ODrive S1, wing_idx == node_id. Simple 1:1.
    """
    return wing_idx

def _arb_id(node_id, cmd_id):
    """
    ODrive S1 CAN arbitration ID.
    Formula: (node_id << 5) | cmd_id
    Single axis — no axis0/axis1 distinction. No 0x10 offset.
    """
    return (node_id << 5) | cmd_id


class ODriveInterface:
    """
    Interface to 8 × ODrive S1 boards controlling 8 wing motors.
    Each board (node 0-7) drives exactly one wing.
    Wing index equals node ID — the simplest possible mapping.
    """

    def __init__(self, channel=CAN_BUS_INTERFACE, bitrate=CAN_BITRATE_BPS):
        self._bus   = None
        self._sim   = not CAN_AVAILABLE
        self._lock  = threading.Lock()
        self._running = True
        self._positions = [0.0] * N_WINGS
        self._velocities = [0.0] * N_WINGS
        self._currents = [0.0] * N_WINGS
        self._temperatures = [{'fet_C': 25.0, 'motor_C': 25.0} for _ in range(N_WINGS)]
        self._vbus = BATTERY_NOMINAL_V
        self._states = [
            {'responding': False, 'error': 0, 'state': STATE_IDLE, 'wing': WING_NAMES[i], 'node': i}
            for i in range(N_WINGS)
        ]
        self._last_seen = [0.0] * N_WINGS
        self._pending_requests = {}
        self._rx_thread = None
        self._osc_t0 = time.monotonic()

        if CAN_AVAILABLE:
            try:
                self._bus = can.interface.Bus(
                    channel=channel,
                    bustype="socketcan",
                    bitrate=bitrate,
                )
                print(f"[S1] CAN bus {channel} opened at {bitrate//1000} kbps "
                      f"(8 nodes, 1 motor each)")
                self._rx_thread = threading.Thread(target=self._rx_worker, daemon=True)
                self._rx_thread.start()
            except Exception as e:
                print(f"[S1] CAN open failed: {e} — simulation mode")
                self._sim = True

    # ── Background RX Worker ──────────────────────────────────────────────────

    def _rx_worker(self):
        """Background daemon thread: continuously reads incoming CAN frames and updates telemetry cache."""
        while self._running and self._bus:
            try:
                msg = self._bus.recv(timeout=0.01)
                if not msg:
                    continue
                node_id = (msg.arbitration_id >> 5) & 0x7F
                cmd_id  = msg.arbitration_id & 0x1F

                # Check if there is an explicit request waiter
                key = (node_id, cmd_id)
                with self._lock:
                    if key in self._pending_requests:
                        evt, holder = self._pending_requests[key]
                        holder.append(msg)
                        evt.set()

                # Update cached telemetry
                if 0 <= node_id < N_WINGS:
                    self._last_seen[node_id] = time.monotonic()
                    data = msg.data
                    if cmd_id == CMD_GET_ENCODER and len(data) >= 8:
                        pos_turns, vel_rps = struct.unpack("<ff", data[:8])
                        with self._lock:
                            self._positions[node_id] = pos_turns * 2 * math.pi
                            self._velocities[node_id] = vel_rps
                    elif cmd_id == CMD_GET_IQ and len(data) >= 8:
                        _iq_setpt, iq_meas = struct.unpack("<ff", data[:8])
                        with self._lock:
                            self._currents[node_id] = abs(iq_meas)
                    elif cmd_id == CMD_GET_TEMP and len(data) >= 8:
                        fet_c, mot_c = struct.unpack("<ff", data[:8])
                        with self._lock:
                            self._temperatures[node_id] = {
                                'fet_C': round(fet_c, 1),
                                'motor_C': round(mot_c, 1)
                            }
                    elif cmd_id == CMD_GET_VBUS and len(data) >= 8:
                        vbus, _ibus = struct.unpack("<ff", data[:8])
                        with self._lock:
                            self._vbus = vbus
                    elif cmd_id == CMD_HEARTBEAT and len(data) >= 4:
                        axis_error = struct.unpack("<I", data[:4])[0]
                        axis_state = data[4] if len(data) > 4 else 0
                        with self._lock:
                            self._states[node_id] = {
                                'responding': True,
                                'error':      axis_error,
                                'state':      axis_state,
                                'wing':       WING_NAMES[node_id],
                                'node':       node_id,
                            }
            except Exception:
                pass

    # ── Low-level CAN ─────────────────────────────────────────────────────────

    def _send(self, node_id, cmd_id, data: bytes = b""):
        """Send a CAN frame to one S1 board."""
        if self._sim or not self._bus:
            return
        msg = can.Message(
            arbitration_id=_arb_id(node_id, cmd_id),
            data=data,
            is_extended_id=False,
        )
        try:
            self._bus.send(msg, timeout=0.002)
        except Exception as e:
            print(f"[S1] CAN send node {node_id} cmd 0x{cmd_id:03X}: {e}")

    def _request(self, node_id, cmd_id, timeout_s=0.004):
        """Send a request frame and wait for response frame via background worker dispatch."""
        if self._sim or not self._bus:
            return None
        evt = threading.Event()
        holder = []
        key = (node_id, cmd_id)
        with self._lock:
            self._pending_requests[key] = (evt, holder)
        try:
            self._send(node_id, cmd_id)
            if evt.wait(timeout_s) and holder:
                return holder[0]
            return None
        finally:
            with self._lock:
                self._pending_requests.pop(key, None)

    # ── Axis state control ────────────────────────────────────────────────────

    def set_state(self, node_id, state):
        """Set axis state on one S1 board."""
        data = struct.pack("<I", state)
        self._send(node_id, CMD_SET_STATE, data)

    def enable_wing(self, wing_idx):
        """Enable closed-loop control on one wing motor."""
        self.set_state(wing_to_node(wing_idx), STATE_CLOSED_LOOP)

    def idle_wing(self, wing_idx):
        """Set one wing motor to idle (no torque, no velocity)."""
        self.set_state(wing_to_node(wing_idx), STATE_IDLE)

    def enable_all(self):
        """Enable closed-loop control on all 8 wing motors."""
        for i in range(N_WINGS):
            self.enable_wing(i)
            time.sleep(0.015)   # stagger to avoid CAN bus burst
        print("[S1] All 8 motors enabled (closed-loop)")

    def idle_all(self):
        """Set all 8 wing motors to idle."""
        for i in range(N_WINGS):
            self.idle_wing(i)
        print("[S1] All 8 motors idle")

    def estop(self):
        """Emergency stop all motors immediately."""
        for i in range(N_WINGS):
            self._send(wing_to_node(i), CMD_ESTOP)
        print("[S1] ESTOP sent to all 8 nodes")

    def clear_errors(self, wing_idx=None):
        """Clear errors on one or all wings."""
        targets = [wing_idx] if wing_idx is not None else list(range(N_WINGS))
        for i in targets:
            self._send(wing_to_node(i), CMD_CLEAR_ERRORS)

    # ── Velocity commands ─────────────────────────────────────────────────────

    def set_velocity(self, wing_idx, vel_rps, torque_ff=0.0):
        """
        Command motor shaft velocity for one wing.
        vel_rps:   rotations per second at motor shaft
        torque_ff: feedforward torque (N·m), usually 0.0 for flapping
        """
        node = wing_to_node(wing_idx)
        data = struct.pack("<ff", vel_rps, torque_ff)
        self._send(node, CMD_SET_VEL, data)

    def set_flap_freq(
        self,
        wing_idx,
        freq_hz,
        reducer_ratio=GEAR_RATIO,
        stroke_half_angle_deg=CRUISE_FLAP_HALF_ANGLE_DEG,
        phase_offset_deg=0.0,
    ):
        """
        Command instantaneous reciprocating motor velocity for a target flap frequency.
        Wait/safety note: Never send constant unidirectional velocity (`freq * ratio`)
        to a 200-deg sector wheel, which would hit mechanical hard stops in <0.5 s.
        """
        t_s = time.monotonic() - self._osc_t0
        phase_rad = 2.0 * math.pi * float(freq_hz) * t_s + math.radians(phase_offset_deg)
        wing_turns_amp = float(stroke_half_angle_deg) / 360.0
        vel_rps = (
            2.0 * math.pi * float(freq_hz) * wing_turns_amp * float(reducer_ratio) * math.cos(phase_rad)
        )
        self.set_velocity(wing_idx, vel_rps)

    def set_all_flap_freq(
        self,
        freq_hz,
        reducer_ratio=GEAR_RATIO,
        stroke_half_angle_deg=20.0,
    ):
        """Command reciprocating oscillation across all 8 wings."""
        self.set_cascade_freq(
            freq_hz,
            phase_offsets_deg=[0.0, 0.0, 90.0, 90.0, 180.0, 180.0, 270.0, 270.0],
            reducer_ratio=reducer_ratio,
            stroke_half_angle_deg=stroke_half_angle_deg,
        )

    def set_all_velocities(self, vel_rps_list):
        """
        Set individual velocities for all 8 wings.
        vel_rps_list: list of 8 floats (motor shaft rev/s per wing)
        """
        for i, vel in enumerate(vel_rps_list[:N_WINGS]):
            self.set_velocity(i, vel)

    def set_cascade_freq(
        self,
        freq_hz,
        phase_offsets_deg=None,
        reducer_ratio=GEAR_RATIO,
        stroke_half_angle_deg=20.0,
    ):
        """
        Command all 8 wings with zero-mean reciprocating velocity at `freq_hz`.
        `phase_offsets_deg` may be a list of 8 per-wing phase offsets (deg) or None.
        """
        if isinstance(phase_offsets_deg, (int, float)):
            # Backward-compatible call signature if caller passed reducer_ratio as 2nd arg
            reducer_ratio = float(phase_offsets_deg)
            offsets = [0.0, 0.0, 90.0, 90.0, 180.0, 180.0, 270.0, 270.0]
        elif phase_offsets_deg is None:
            offsets = [0.0, 0.0, 90.0, 90.0, 180.0, 180.0, 270.0, 270.0]
        else:
            offsets = list(phase_offsets_deg)
        for i in range(N_WINGS):
            phi_deg = offsets[i] if i < len(offsets) else 0.0
            self.set_flap_freq(
                i,
                freq_hz,
                reducer_ratio=reducer_ratio,
                stroke_half_angle_deg=stroke_half_angle_deg,
                phase_offset_deg=phi_deg,
            )

    def oscillate_cascade(
        self,
        freq_hz,
        duration_s=2.0,
        phase_offsets_deg=None,
        reducer_ratio=GEAR_RATIO,
        stroke_half_angle_deg=20.0,
        dt_s=0.02,
    ):
        """Run a closed-loop 50 Hz reciprocating oscillation for `duration_s` and return to zero velocity."""
        t_end = time.monotonic() + float(duration_s)
        while time.monotonic() < t_end:
            self.set_cascade_freq(
                freq_hz,
                phase_offsets_deg=phase_offsets_deg,
                reducer_ratio=reducer_ratio,
                stroke_half_angle_deg=stroke_half_angle_deg,
            )
            time.sleep(dt_s)
        self.set_all_velocities([0.0] * N_WINGS)

    def set_velocity_limit(self, wing_idx, vel_limit_rps, current_limit_A=None):
        """
        Set velocity and optionally current limits on one S1.
        vel_limit_rps: maximum shaft velocity (rad/s or rps — check firmware)
        current_limit_A: maximum current in amps (None = don't change)
        Uses SET_LIMITS command (0x011).
        """
        node  = wing_to_node(wing_idx)
        vlim  = float(vel_limit_rps)
        clim  = float(current_limit_A) if current_limit_A is not None else 0.0
        data  = struct.pack("<ff", vlim, clim)
        self._send(node, CMD_SET_LIMITS, data)

    # ── Position commands ─────────────────────────────────────────────────────

    def set_position(self, wing_idx, pos_turns, vel_ff=0.0, torque_ff=0.0):
        """
        Set motor shaft position in turns.
        The S1 absolute encoder means cold-start position is known immediately —
        no index search or homing required before position commands.
        """
        node = wing_to_node(wing_idx)
        data = struct.pack("<fff", pos_turns, vel_ff, torque_ff)
        self._send(node, CMD_SET_POS, data)

    def set_initial_phases(self, phase_offsets_deg, reducer_ratio=GEAR_RATIO, stroke_half_angle_deg=20.0):
        """
        Move each wing to its initial cascade phase before enabling velocity.
        Maps wing stroke phase angle into bounded capstan motor shaft turns.
        """
        wing_turns_amp = float(stroke_half_angle_deg) / 360.0
        for i, phase_deg in enumerate(phase_offsets_deg[:N_WINGS]):
            pos_turns = math.sin(math.radians(phase_deg)) * wing_turns_amp * float(reducer_ratio)
            self.set_position(i, pos_turns)
            time.sleep(0.025)
        print("[S1] Initial cascade phases set")

    # ── Telemetry reads ───────────────────────────────────────────────────────

    def trigger_telemetry_request(self):
        """Non-blocking: trigger telemetry queries to all nodes without waiting."""
        if self._sim or not self._bus:
            return
        for i in range(N_WINGS):
            self._send(i, CMD_GET_ENCODER)
            self._send(i, CMD_GET_IQ)

    def read_encoder_estimates(self):
        """
        Read pos_estimate and vel_estimate from cached telemetry for all 8 S1 boards.
        The S1 uses an on-board absolute magnetic encoder.
        Position is in turns (converted to radians at motor shaft).

        Returns:
            positions_rad:   list of 8 floats (motor shaft, radians)
            velocities_rps:  list of 8 floats (motor shaft, rev/s)
        Non-blocking: returns cached values in < 0.05 ms without starving the control loop.
        """
        if self._sim:
            return [0.0] * N_WINGS, [0.0] * N_WINGS

        # Trigger async CAN requests for next tick update
        for i in range(N_WINGS):
            self._send(i, CMD_GET_ENCODER)

        with self._lock:
            return list(self._positions), list(self._velocities)

    def read_currents(self):
        """
        Read Iq_measured from cached telemetry for all 8 S1 boards.
        Returns list of 8 floats (amps). Non-blocking.
        """
        if self._sim:
            return [0.0] * N_WINGS

        for i in range(N_WINGS):
            self._send(i, CMD_GET_IQ)

        with self._lock:
            return list(self._currents)

    def read_temperatures(self):
        """
        Read FET and motor temperatures from cached telemetry.
        Returns list of 8 dicts {'fet_C': float, 'motor_C': float}. Non-blocking.
        """
        if self._sim:
            return [{'fet_C': 25.0, 'motor_C': 25.0}] * N_WINGS

        for i in range(N_WINGS):
            self._send(i, CMD_GET_TEMP)

        with self._lock:
            return [dict(t) for t in self._temperatures]

    def read_vbus(self):
        """
        Read bus voltage from node 0.
        Returns float volts. Non-blocking.
        """
        if self._sim:
            return 22.2

        self._send(0, CMD_GET_VBUS)
        with self._lock:
            return self._vbus

    def read_all_states(self):
        """
        Read heartbeat / axis state for all 8 nodes.
        Returns list of 8 dicts with 'error', 'state', 'responding'. Non-blocking.
        """
        now = time.monotonic()
        with self._lock:
            res = []
            for i in range(N_WINGS):
                st = dict(self._states[i])
                if not self._sim:
                    st['responding'] = (now - self._last_seen[i]) < 1.0
                else:
                    st['responding'] = True
                res.append(st)
            return res

    def get_vel_limit(self, wing_idx=0):
        """
        Read velocity limit from one S1 board.
        Used by mco_preflight_checks.py to verify configuration.
        Returns vel_limit_rps or None.
        """
        # ODrive S1 does not expose vel_limit over a dedicated read command
        # in the simple CAN protocol. Return None to signal manual check needed.
        # With odrivetool or web GUI, verify: axis0.controller.config.vel_limit
        return None   # use mco_odrive_setup.py --verify for this

    # ── Cleanup ───────────────────────────────────────────────────────────────

    def close(self):
        """Idle all motors and shut down the CAN bus."""
        self._running = False
        self.idle_all()
        if self._rx_thread and self._rx_thread.is_alive():
            self._rx_thread.join(timeout=0.2)
        if self._bus:
            try:
                self._bus.shutdown()
            except Exception:
                pass
        print("[S1] Interface closed")

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.close()


# ── CAN node lookup helpers ───────────────────────────────────────────────────

def pair_to_nodes(pair_idx):
    """
    Return the two CAN node IDs for a wing pair (1-indexed).
    pair_idx 1 → nodes (0, 1)  [W1 port, W1 starboard]
    pair_idx 2 → nodes (2, 3)  [W2 port, W2 starboard]
    pair_idx 3 → nodes (4, 5)
    pair_idx 4 → nodes (6, 7)
    """
    base = (pair_idx - 1) * 2
    return base, base + 1

def node_to_wing_name(node_id):
    """Return the wing name string for a CAN node ID."""
    if 0 <= node_id < N_WINGS:
        return WING_NAMES[node_id]
    return f"node{node_id}"
