#!/usr/bin/env python3
"""
mco_kinematics.py
=================
Deterministic 50 Hz metachronal phase oscillator, control allocation mixer,
and 8-wing reciprocating kinematics synthesizer for the MCO-1D Mark 1.

Architecture:
  1. MetachronalPhaseOscillator:
     Maintains master flapping phase `phi_0` (rad) and computes the 4 tandem
     station phases `station_phases_rad[k] = (phi_0 - k * cascade_phase_rad) % 2pi`
     for stations `k in 0..3` (W1..W4), mapped to 8 wings (`2*k` port, `2*k+1` stbd).
  2. ControlAllocationMixer:
     Maps normalized pilot/autopilot stick inputs `(roll, pitch, yaw, throttle)`
     and optional wave-library or flight-governor command overrides into bounded
     per-wing stroke half-angles, pitch trim/feathering amplitudes, and sweep angles.
  3. WingPairKinematics & MetachronalSynthesizer:
     Synthesizes closed-form 50 Hz actuator setpoints for all 8 wings:
       - Flapping angle `flap_angle_deg[i]` and stroke direction `stroke_dir[i]`
       - ODrive capstan position `motor_pos_turns[i]` and velocity `motor_vel_rps[i]`
       - 90°-phase-advanced pitch feathering angle `pitch_servo_deg[i]`
       - Planform sweep servo angle `sweep_servo_deg[i]`
"""

from __future__ import annotations

import math
from typing import Any

from mco_config import (
    CRUISE_FLAP_FREQ_HZ,
    CRUISE_FLAP_HALF_ANGLE_DEG,
    GEAR_RATIO,
    LOOP_DT_S,
    MAX_FLAP_FREQ_HZ,
    MAX_WING_STROKE_DEG,
    MCOConfig,
    SERVO_TRAVEL_DEG,
    TANDEM_PAIR_COUNT,
    WING_COUNT,
)

TWO_PI: float = 2.0 * math.pi
DEFAULT_CASCADE_PHASE_DEG: float = 90.0
DEFAULT_SWEEP_DEG: float = 60.0
DEFAULT_PITCH_TRIM_DEG: float = 4.0
DEFAULT_FEATHER_AMP_DEG: float = 10.0
MIN_SWEEP_DEG: float = 15.0
MAX_SWEEP_DEG: float = 165.0
MAX_PITCH_SERVO_DEG: float = SERVO_TRAVEL_DEG / 2.0


def _clamp(value: float, low: float, high: float) -> float:
    """Clamp scalar `value` into `[low, high]`."""
    return max(low, min(high, float(value)))


def _expand_per_wing(
    value: float | list[float] | tuple[float, ...] | None,
    default: float,
    count: int = WING_COUNT,
) -> list[float]:
    """Normalize a scalar, 4-pair list, or 8-wing list into an 8-element float list."""
    if value is None:
        return [float(default)] * count
    if isinstance(value, (int, float)):
        return [float(value)] * count
    seq = list(value)
    if len(seq) == TANDEM_PAIR_COUNT and count == WING_COUNT:
        expanded: list[float] = []
        for item in seq:
            expanded.extend([float(item), float(item)])
        return expanded
    return [float(seq[i]) if i < len(seq) else float(default) for i in range(count)]


class MetachronalPhaseOscillator:
    """
    4-station metachronal cascade phase oscillator.

    Tracks master phase `phi_0` in `[0, 2*pi)` and computes station phases:
        station_phases_rad[k] = (phi_0 - k * cascade_phase_rad) % (2 * pi)
    for tandem stations `k in 0..3` (`W1..W4`), mapping to 8 wings (`2*k` port,
    `2*k + 1` starboard).
    """

    def __init__(
        self,
        cascade_phase_deg: float = DEFAULT_CASCADE_PHASE_DEG,
        initial_phase_rad: float = 0.0,
    ) -> None:
        self.phi_0: float = float(initial_phase_rad) % TWO_PI
        self.cascade_phase_deg: float = float(cascade_phase_deg)
        self._custom_pair_offsets_rad: list[float] | None = None

    @property
    def master_phase_rad(self) -> float:
        """Return current master phase `phi_0` in radians."""
        return self.phi_0

    @master_phase_rad.setter
    def master_phase_rad(self, value: float) -> None:
        self.phi_0 = float(value) % TWO_PI

    @property
    def pair_offsets_rad(self) -> list[float]:
        """Return the 4 tandem station phase lag offsets in radians."""
        if self._custom_pair_offsets_rad is not None:
            return list(self._custom_pair_offsets_rad)
        cascade_rad = math.radians(self.cascade_phase_deg)
        return [(k * cascade_rad) % TWO_PI for k in range(TANDEM_PAIR_COUNT)]

    def set_cascade_phase(self, cascade_phase_deg: float) -> list[float]:
        """Set uniform inter-station phase lag in degrees and clear custom pair offsets."""
        self.cascade_phase_deg = float(cascade_phase_deg)
        self._custom_pair_offsets_rad = None
        return self.pair_offsets_rad

    def set_pair_offsets(self, pair_offsets_rad: list[float] | None) -> None:
        """Override station phase offsets (e.g. for 3-pair 120° RTH or ground-effect phase)."""
        if pair_offsets_rad is None:
            self._custom_pair_offsets_rad = None
        else:
            self._custom_pair_offsets_rad = [float(x) % TWO_PI for x in pair_offsets_rad]

    def reset(self, phi_0: float = 0.0) -> None:
        """Reset master oscillator phase and clear custom pair offsets."""
        self.phi_0 = float(phi_0) % TWO_PI
        self._custom_pair_offsets_rad = None

    def advance(self, dt_s: float, freq_hz: float) -> float:
        """Advance master phase `phi_0` by `2 * pi * freq_hz * dt_s`."""
        dt_clamped = max(0.0, float(dt_s))
        freq_clamped = _clamp(freq_hz, 0.0, MAX_FLAP_FREQ_HZ)
        self.phi_0 = (self.phi_0 + TWO_PI * freq_clamped * dt_clamped) % TWO_PI
        return self.phi_0

    def get_station_phases_rad(
        self,
        cascade_phase_deg: float | None = None,
        pair_offsets_rad: list[float] | None = None,
    ) -> list[float]:
        """Compute instantaneous phase `station_phases_rad[k]` for stations `k in 0..3`."""
        if pair_offsets_rad is not None:
            offsets = [float(x) for x in pair_offsets_rad]
        elif cascade_phase_deg is not None:
            c_rad = math.radians(float(cascade_phase_deg))
            offsets = [k * c_rad for k in range(TANDEM_PAIR_COUNT)]
        else:
            offsets = self.pair_offsets_rad

        station_phases: list[float] = []
        fallback_rad = math.radians(self.cascade_phase_deg)
        for k in range(TANDEM_PAIR_COUNT):
            offset_k = offsets[k] if k < len(offsets) else (k * fallback_rad)
            station_phases.append((self.phi_0 - offset_k) % TWO_PI)
        return station_phases

    def get_wing_phases_rad(
        self,
        cascade_phase_deg: float | None = None,
        pair_offsets_rad: list[float] | None = None,
        wing_phase_offsets_deg: list[float | None] | None = None,
    ) -> list[float]:
        """Map the 4 tandem station phases to all 8 wings (`2*k` port, `2*k+1` starboard)."""
        station_phases = self.get_station_phases_rad(
            cascade_phase_deg=cascade_phase_deg,
            pair_offsets_rad=pair_offsets_rad,
        )
        wing_phases: list[float] = []
        for i in range(WING_COUNT):
            if (
                wing_phase_offsets_deg is not None
                and i < len(wing_phase_offsets_deg)
                and wing_phase_offsets_deg[i] is not None
            ):
                offset_rad = math.radians(float(wing_phase_offsets_deg[i]))
                wing_phases.append((self.phi_0 - offset_rad) % TWO_PI)
            else:
                station_idx = i // 2
                wing_phases.append(station_phases[station_idx])
        return wing_phases


class ControlAllocationMixer:
    """
    Maps normalized pilot/autopilot control inputs `(roll, pitch, yaw, throttle)`
    and optional command overrides into bounded 8-wing kinematic parameters.
    """

    ROLL_STROKE_GAIN_DEG: float = 8.0
    PITCH_TRIM_GAIN_DEG: float = 6.0
    YAW_PITCH_GAIN_DEG: float = 4.0
    PITCH_SWEEP_BIAS_GAIN_DEG: float = 5.0
    YAW_SWEEP_GAIN_DEG: float = 6.0

    def allocate(
        self,
        roll: float = 0.0,
        pitch: float = 0.0,
        yaw: float = 0.0,
        throttle: float = 0.5,
        *,
        freq_hz: float | None = None,
        stroke_amp_deg: float | list[float] | None = None,
        aoa_cmd_deg: float | list[float] | None = None,
        sweep_cmd_deg: float | list[float] | None = None,
        dihedral_bias_deg: float | list[float] | None = None,
        cascade_phase_deg: float = DEFAULT_CASCADE_PHASE_DEG,
        pitch_feather_amp_deg: float | list[float] | None = None,
        wing_commands: list[dict[str, Any]] | None = None,
    ) -> dict[str, Any]:
        """Compute per-wing stroke half-angles, pitch trims, feathering amplitudes, and sweeps."""
        roll_u = _clamp(roll, -1.0, 1.0)
        pitch_u = _clamp(pitch, -1.0, 1.0)
        yaw_u = _clamp(yaw, -1.0, 1.0)
        throttle_u = _clamp(throttle, 0.0, 1.0)

        if freq_hz is None:
            if throttle_u <= 0.0:
                target_freq_hz = 0.0
            else:
                target_freq_hz = _clamp(
                    CRUISE_FLAP_FREQ_HZ * (0.5 + 0.5 * throttle_u) / 0.75,
                    0.0,
                    MAX_FLAP_FREQ_HZ,
                )
        else:
            target_freq_hz = _clamp(freq_hz, 0.0, MAX_FLAP_FREQ_HZ)

        default_stroke = (
            0.0 if throttle_u <= 0.0 and stroke_amp_deg is None and not wing_commands
            else CRUISE_FLAP_HALF_ANGLE_DEG
        )
        base_strokes = _expand_per_wing(stroke_amp_deg, default_stroke)
        base_pitches = _expand_per_wing(aoa_cmd_deg, DEFAULT_PITCH_TRIM_DEG)
        base_sweeps = _expand_per_wing(sweep_cmd_deg, DEFAULT_SWEEP_DEG)
        base_dihedrals = _expand_per_wing(dihedral_bias_deg, 0.0)
        base_feathers = (
            _expand_per_wing(pitch_feather_amp_deg, DEFAULT_FEATHER_AMP_DEG)
            if pitch_feather_amp_deg is not None
            else None
        )

        stroke_half_angle_out: list[float] = [0.0] * WING_COUNT
        pitch_trim_out: list[float] = [0.0] * WING_COUNT
        pitch_feather_amp_out: list[float] = [0.0] * WING_COUNT
        sweep_out: list[float] = [DEFAULT_SWEEP_DEG] * WING_COUNT
        dihedral_bias_out: list[float] = [0.0] * WING_COUNT
        quarantined_out: list[bool] = [False] * WING_COUNT
        wing_phase_offsets_deg: list[float | None] = [None] * WING_COUNT

        for i in range(WING_COUNT):
            is_port = (i % 2 == 0)
            station_idx = i // 2
            is_fore_pair = station_idx in (0, 1)

            cmd = wing_commands[i] if (wing_commands and i < len(wing_commands) and isinstance(wing_commands[i], dict)) else {}
            is_quarantined = bool(cmd.get("quarantined") or cmd.get("gear_tier3_stripped"))
            quarantined_out[i] = is_quarantined

            if "phase_offset_deg" in cmd and cmd["phase_offset_deg"] is not None:
                wing_phase_offsets_deg[i] = float(cmd["phase_offset_deg"])

            raw_stroke = float(cmd.get("flap_amp_deg", base_strokes[i]))
            raw_pitch = float(cmd.get("pitch_deg", base_pitches[i]))
            raw_sweep = float(cmd.get("sweep_deg", base_sweeps[i]))
            raw_dihedral = float(cmd.get("dihedral_bias_deg", base_dihedrals[i]))

            # 1. Stroke half-angle with differential roll allocation (-roll*8 on port, +roll*8 on stbd)
            if is_quarantined or (raw_stroke <= 0.0 and stroke_amp_deg is None and wing_commands is not None):
                stroke_i = 0.0
            else:
                roll_delta_deg = (-roll_u * self.ROLL_STROKE_GAIN_DEG) if is_port else (+roll_u * self.ROLL_STROKE_GAIN_DEG)
                stroke_i = _clamp(raw_stroke + roll_delta_deg, 0.0, MAX_WING_STROKE_DEG)
            stroke_half_angle_out[i] = stroke_i

            # 2. Pitch trim angle with longitudinal pitch (+pitch*6 fore, -pitch*6 aft) & yaw differential
            pitch_long_deg = (+pitch_u * self.PITCH_TRIM_GAIN_DEG) if is_fore_pair else (-pitch_u * self.PITCH_TRIM_GAIN_DEG)
            pitch_yaw_deg = (+yaw_u * self.YAW_PITCH_GAIN_DEG) if is_port else (-yaw_u * self.YAW_PITCH_GAIN_DEG)
            pitch_trim_i = _clamp(
                raw_pitch + pitch_long_deg + pitch_yaw_deg,
                -MAX_PITCH_SERVO_DEG,
                MAX_PITCH_SERVO_DEG,
            )
            pitch_trim_out[i] = pitch_trim_i

            # 3. Dynamic pitch feathering amplitude (90° phase-advanced propulsive twist)
            if is_quarantined or stroke_i <= 0.0 or target_freq_hz <= 0.0 or cmd.get("gear_tier2_locked"):
                feather_i = 0.0
            elif "pitch_feather_amp_deg" in cmd:
                feather_i = _clamp(float(cmd["pitch_feather_amp_deg"]), 0.0, 25.0)
            elif base_feathers is not None:
                feather_i = _clamp(base_feathers[i], 0.0, 25.0)
            else:
                stroke_ratio = stroke_i / max(1.0, CRUISE_FLAP_HALF_ANGLE_DEG)
                feather_i = _clamp(DEFAULT_FEATHER_AMP_DEG * stroke_ratio, 0.0, 20.0)
            pitch_feather_amp_out[i] = feather_i

            # 4. Planform sweep angle with fore/aft longitudinal pitch bias & yaw differential
            sweep_pitch_bias = (-pitch_u * self.PITCH_SWEEP_BIAS_GAIN_DEG) if is_fore_pair else (+pitch_u * self.PITCH_SWEEP_BIAS_GAIN_DEG)
            sweep_yaw_bias = (-yaw_u * self.YAW_SWEEP_GAIN_DEG) if is_port else (+yaw_u * self.YAW_SWEEP_GAIN_DEG)
            sweep_i = _clamp(raw_sweep + sweep_pitch_bias + sweep_yaw_bias, MIN_SWEEP_DEG, MAX_SWEEP_DEG)
            sweep_out[i] = sweep_i

            # 5. Dihedral bias angle
            dihedral_bias_out[i] = _clamp(raw_dihedral, -60.0, 60.0)

        return {
            "freq_hz": target_freq_hz,
            "cascade_phase_deg": float(cascade_phase_deg),
            "stroke_half_angle_deg": stroke_half_angle_out,
            "pitch_trim_deg": pitch_trim_out,
            "pitch_feather_amp_deg": pitch_feather_amp_out,
            "sweep_deg": sweep_out,
            "dihedral_bias_deg": dihedral_bias_out,
            "quarantined": quarantined_out,
            "wing_phase_offsets_deg": wing_phase_offsets_deg,
        }


class WingPairKinematics:
    """
    Evaluates closed-form 50 Hz position, velocity, and 90°-phase-advanced
    feathering kinematics across all 4 tandem wing pairs (8 wings).
    """

    def evaluate(
        self,
        wing_phases_rad: list[float],
        freq_hz: float,
        allocation: dict[str, Any],
        phase_lead_rad: float = 0.0,
    ) -> dict[str, Any]:
        """Compute instantaneous flap, ODrive motor, pitch servo, and sweep servo setpoints."""
        stroke_half_angle_deg = allocation["stroke_half_angle_deg"]
        pitch_trim_deg = allocation["pitch_trim_deg"]
        pitch_feather_amp_deg = allocation["pitch_feather_amp_deg"]
        sweep_deg = allocation["sweep_deg"]
        dihedral_bias_deg = allocation["dihedral_bias_deg"]

        flap_angle_deg: list[float] = [0.0] * WING_COUNT
        stroke_dir: list[int] = [1] * WING_COUNT
        motor_pos_turns: list[float] = [0.0] * WING_COUNT
        motor_vel_rps: list[float] = [0.0] * WING_COUNT
        pitch_servo_deg: list[float] = [0.0] * WING_COUNT
        sweep_servo_deg: list[float] = [DEFAULT_SWEEP_DEG] * WING_COUNT

        for i in range(WING_COUNT):
            phase_i = float(wing_phases_rad[i])
            sin_p = math.sin(phase_i)
            cos_p = math.cos(phase_i)

            half_angle_i = float(stroke_half_angle_deg[i])
            dihedral_i = float(dihedral_bias_deg[i])

            # Instantaneous flapping stroke angle (deg) and stroke direction (+1 down/fwd, -1 up/aft)
            flap_i = dihedral_i + half_angle_i * sin_p
            dir_i = 1 if cos_p >= 0.0 else -1

            # ODrive capstan motor position (turns) and velocity (turns/s = rps)
            pos_turns_i = (flap_i / 360.0) * GEAR_RATIO
            vel_rps_i = TWO_PI * float(freq_hz) * (half_angle_i / 360.0) * GEAR_RATIO * cos_p

            # 90° phase-advanced pitch feathering:
            # sin(phase_i + pi/2) == cos(phase_i). Subtracting pitch_feather_amp * cos(phase_i + phase_lead_rad)
            # pitches the wing nose-down at peak positive stroke velocity (cos == +1) for forward thrust,
            # and feathers nose-up at peak negative velocity (cos == -1).
            pitch_i = _clamp(
                float(pitch_trim_deg[i]) - float(pitch_feather_amp_deg[i]) * math.cos(phase_i + float(phase_lead_rad)),
                -MAX_PITCH_SERVO_DEG,
                MAX_PITCH_SERVO_DEG,
            )
            sweep_i = _clamp(float(sweep_deg[i]), MIN_SWEEP_DEG, MAX_SWEEP_DEG)

            flap_angle_deg[i] = flap_i
            stroke_dir[i] = dir_i
            motor_pos_turns[i] = pos_turns_i
            motor_vel_rps[i] = vel_rps_i
            pitch_servo_deg[i] = pitch_i
            sweep_servo_deg[i] = sweep_i

        return {
            "freq_hz": float(freq_hz),
            "wing_phases_rad": list(wing_phases_rad),
            "flap_phases_norm": [round(p / TWO_PI, 4) for p in wing_phases_rad],
            "flap_angle_deg": flap_angle_deg,
            "stroke_dir": stroke_dir,
            "motor_pos_turns": motor_pos_turns,
            "motor_vel_rps": motor_vel_rps,
            "pitch_servo_deg": pitch_servo_deg,
            "sweep_servo_deg": sweep_servo_deg,
            "stroke_half_angle_deg": list(stroke_half_angle_deg),
            "pitch_trim_deg": list(pitch_trim_deg),
            "pitch_feather_amp_deg": list(pitch_feather_amp_deg),
            "dihedral_bias_deg": list(dihedral_bias_deg),
        }


class MetachronalSynthesizer:
    """
    Top-level 50 Hz kinematics synthesizer combining `MetachronalPhaseOscillator`,
    `ControlAllocationMixer`, and `WingPairKinematics`.
    """

    CONFIG = MCOConfig

    def __init__(
        self,
        cascade_phase_deg: float = DEFAULT_CASCADE_PHASE_DEG,
        initial_phase_rad: float = 0.0,
    ) -> None:
        self.oscillator = MetachronalPhaseOscillator(
            cascade_phase_deg=cascade_phase_deg,
            initial_phase_rad=initial_phase_rad,
        )
        self.mixer = ControlAllocationMixer()
        self.kinematics = WingPairKinematics()
        self._last_step: dict[str, Any] = self._build_initial_state()

    def _build_initial_state(self) -> dict[str, Any]:
        wing_phases = self.oscillator.get_wing_phases_rad()
        alloc = self.mixer.allocate(throttle=0.0, freq_hz=0.0)
        return self.kinematics.evaluate(wing_phases, 0.0, alloc)

    def reset(self, phi_0: float = 0.0) -> None:
        """Reset oscillator phase and cached state."""
        self.oscillator.reset(phi_0=phi_0)
        self._last_step = self._build_initial_state()

    def step(
        self,
        dt_s: float = LOOP_DT_S,
        commands: dict[str, Any] | list[dict[str, Any]] | None = None,
        sticks: dict[str, float] | None = None,
        *,
        freq_hz: float | None = None,
        pair_offsets_rad: list[float] | None = None,
        phase_lead_rad: float = 0.0,
    ) -> dict[str, Any]:
        """Advance oscillator by `dt_s` and compute all 8-wing kinematic setpoints."""
        sticks_dict = sticks or {}
        roll = float(sticks_dict.get("roll", 0.0))
        pitch = float(sticks_dict.get("pitch", 0.0))
        yaw = float(sticks_dict.get("yaw", 0.0))
        throttle = float(sticks_dict.get("throttle", 0.5))

        wing_cmds: list[dict[str, Any]] | None = None
        cmd_freq = freq_hz
        stroke_ovr = None
        aoa_ovr = None
        sweep_ovr = None
        dihedral_ovr = None
        feather_ovr = None
        cascade_deg = self.oscillator.cascade_phase_deg

        if isinstance(commands, list):
            wing_cmds = commands
        elif isinstance(commands, dict):
            if "wing_commands" in commands and isinstance(commands["wing_commands"], list):
                wing_cmds = commands["wing_commands"]
            if cmd_freq is None and "freq_hz" in commands:
                cmd_freq = float(commands["freq_hz"])
            stroke_ovr = commands.get("stroke_amp_deg")
            aoa_ovr = commands.get("aoa_cmd_deg")
            sweep_ovr = commands.get("sweep_cmd_deg")
            dihedral_ovr = commands.get("dihedral_bias_deg")
            feather_ovr = commands.get("pitch_feather_amp_deg")
            if "cascade_phase_deg" in commands and commands["cascade_phase_deg"] is not None:
                cascade_deg = float(commands["cascade_phase_deg"])
            if pair_offsets_rad is None and "pair_phase_offsets" in commands:
                pair_offsets_rad = commands["pair_phase_offsets"]

        alloc = self.mixer.allocate(
            roll=roll,
            pitch=pitch,
            yaw=yaw,
            throttle=throttle,
            freq_hz=cmd_freq,
            stroke_amp_deg=stroke_ovr,
            aoa_cmd_deg=aoa_ovr,
            sweep_cmd_deg=sweep_ovr,
            dihedral_bias_deg=dihedral_ovr,
            cascade_phase_deg=cascade_deg,
            pitch_feather_amp_deg=feather_ovr,
            wing_commands=wing_cmds,
        )

        if pair_offsets_rad is not None:
            self.oscillator.set_pair_offsets(pair_offsets_rad)
        elif "cascade_phase_deg" in (commands if isinstance(commands, dict) else {}):
            self.oscillator.set_cascade_phase(cascade_deg)

        self.oscillator.advance(dt_s, alloc["freq_hz"])
        station_phases_rad = self.oscillator.get_station_phases_rad(pair_offsets_rad=pair_offsets_rad)
        wing_phases_rad = self.oscillator.get_wing_phases_rad(
            pair_offsets_rad=pair_offsets_rad,
            wing_phase_offsets_deg=alloc["wing_phase_offsets_deg"],
        )

        frame = self.kinematics.evaluate(
            wing_phases_rad=wing_phases_rad,
            freq_hz=alloc["freq_hz"],
            allocation=alloc,
            phase_lead_rad=phase_lead_rad,
        )
        frame["master_phase_rad"] = self.oscillator.phi_0
        frame["station_phases_rad"] = station_phases_rad
        frame["pair_offsets_rad"] = self.oscillator.pair_offsets_rad
        self._last_step = frame
        return frame

    def get_wing_kinematics(self) -> dict[str, Any]:
        """Return latest flap angles, normalized phases, stroke directions, and servo targets."""
        return {
            "flap_degs": [round(x, 2) for x in self._last_step["flap_angle_deg"]],
            "flap_phases": list(self._last_step["flap_phases_norm"]),
            "stroke_dirs": list(self._last_step["stroke_dir"]),
            "pitch_servo_degs": [round(x, 2) for x in self._last_step["pitch_servo_deg"]],
            "sweep_servo_degs": [round(x, 2) for x in self._last_step["sweep_servo_deg"]],
            "motor_pos_turns": [round(x, 4) for x in self._last_step["motor_pos_turns"]],
            "motor_vel_rps": [round(x, 4) for x in self._last_step["motor_vel_rps"]],
        }
