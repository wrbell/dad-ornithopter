"""
8-Wing Metachronal FPV Drone Firmware
======================================
Hardware: Raspberry Pi / any Linux SBC
Wings:    4 per side, 8 total
Servos:   3 per wing (stroke, pitch, deviation) = 24 total
IMU:      MPU-6050 or BNO055 via I2C
RC Input: SBUS or PWM (5 channels)
Servo driver: Two PCA9685 boards (16ch each, I2C)

Channels:
  CH1 = Roll     (aileron equivalent)
  CH2 = Pitch    (elevator equivalent)
  CH3 = Throttle (collective lift)
  CH4 = Yaw      (rudder equivalent)
  CH5 = WaveMode (metachronal style selector)

Wing numbering (each side, front to back):
  W1=frontmost, W2, W3, W4=rearmost
  Left side:  wings 0-3
  Right side: wings 4-7

Servo mapping per wing:
  servo[wing*3 + 0] = stroke   (phi)
  servo[wing*3 + 1] = pitch    (psi / feather)
  servo[wing*3 + 2] = deviation (theta)
"""

import math
import time
import threading
from dataclasses import dataclass, field
from typing import List, Tuple
from enum import Enum

# ─────────────────────────────────────────────
# CONSTANTS & TUNING PARAMETERS
# ─────────────────────────────────────────────

FLAP_FREQ_HZ       = 1.0       # Base flapping frequency
CONTROL_HZ         = 50        # Servo update rate
IMU_HZ             = 200       # IMU sampling rate
NUM_WINGS          = 8         # Total wings
WINGS_PER_SIDE     = 4         # Wings per side

# Stroke geometry (degrees)
STROKE_AMPLITUDE   = 70.0      # ± degrees, power stroke sweep
DEVIATION_AMPLITUDE= 12.0      # ± degrees, up/down deviation

# Angle of attack profiles (degrees)
AOA_POWER          = 35.0      # Angle during power stroke
AOA_RECOVERY       = 5.0       # Angle during recovery (feathered)
AOA_PITCH_FLIP_MS  = 60        # Time to complete pitch reversal (ms)

# Metachronal wave defaults
DEFAULT_WAVE_SPACING_DEG = 90.0   # Phase between adjacent wings (degrees)

# RC input normalization (microseconds)
RC_MIN_US  = 1000
RC_MAX_US  = 2000
RC_MID_US  = 1500
RC_DEADBAND= 0.03   # ±3% deadband on sticks

# PID gains (tune for your specific airframe)
PID_ROLL  = (0.8, 0.05, 0.12)   # Kp, Ki, Kd
PID_PITCH = (0.7, 0.04, 0.10)
PID_YAW   = (0.6, 0.02, 0.08)

# Servo centre and range (microseconds PWM)
SERVO_CENTRE_US = 1500
SERVO_RANGE_US  = 500           # ±500us = ±90 degrees typically

# ─────────────────────────────────────────────
# ENUMERATIONS
# ─────────────────────────────────────────────

class WaveMode(Enum):
    METACHRONAL   = 0   # Traveling wave front→back (default cruise)
    REVERSE_WAVE  = 1   # Traveling wave back→front (braking/reverse)
    STANDING      = 2   # Alternating pairs (W1W3 vs W2W4)
    IN_PHASE      = 3   # All wings together (max force burst)
    COUNTER_PHASE = 4   # Front pair vs rear pair opposed (smooth hover)

class FlightMode(Enum):
    DISARMED  = 0
    HOVER     = 1
    CRUISE    = 2
    CLIMB     = 3
    DESCEND   = 4
    FLARE     = 5   # Emergency stop / landing

# ─────────────────────────────────────────────
# DATA STRUCTURES
# ─────────────────────────────────────────────

@dataclass
class RCInput:
    roll:      float = 0.0   # -1.0 to +1.0
    pitch:     float = 0.0
    throttle:  float = 0.0   # 0.0 to 1.0
    yaw:       float = 0.0   # -1.0 to +1.0
    wave_mode: WaveMode = WaveMode.METACHRONAL
    armed:     bool = False

@dataclass
class IMUData:
    roll_deg:   float = 0.0
    pitch_deg:  float = 0.0
    yaw_deg:    float = 0.0
    roll_rate:  float = 0.0
    pitch_rate: float = 0.0
    yaw_rate:   float = 0.0

@dataclass
class WingKinematics:
    """Instantaneous angles for one wing (degrees)"""
    phi:   float = 0.0   # stroke position
    psi:   float = 0.0   # pitch / feather angle
    theta: float = 0.0   # deviation

@dataclass
class PIDState:
    integral:  float = 0.0
    prev_error:float = 0.0
    output:    float = 0.0

@dataclass
class WingFaultState:
    failed: List[bool] = field(default_factory=lambda: [False]*8)

# ─────────────────────────────────────────────
# RC INPUT PARSER
# ─────────────────────────────────────────────

class RCParser:
    """
    Parses 5-channel RC input.
    In real hardware: read SBUS uart or GPIO PWM.
    Here we provide the normalisation logic.
    """

    def normalise(self, raw_us: int, is_throttle: bool = False) -> float:
        """Convert PWM microseconds to -1..+1 (or 0..1 for throttle)"""
        clamped = max(RC_MIN_US, min(RC_MAX_US, raw_us))
        if is_throttle:
            return (clamped - RC_MIN_US) / (RC_MAX_US - RC_MIN_US)
        else:
            val = (clamped - RC_MID_US) / ((RC_MAX_US - RC_MID_US))
            # Apply deadband
            if abs(val) < RC_DEADBAND:
                val = 0.0
            return max(-1.0, min(1.0, val))

    def parse(self, channels_us: List[int]) -> RCInput:
        """
        channels_us: list of 5 PWM values in microseconds
        Returns normalised RCInput
        """
        if len(channels_us) < 5:
            return RCInput()  # safe default

        rc = RCInput()
        rc.roll      = self.normalise(channels_us[0])
        rc.pitch     = self.normalise(channels_us[1])
        rc.throttle  = self.normalise(channels_us[2], is_throttle=True)
        rc.yaw       = self.normalise(channels_us[3])

        # CH5: wave mode selector (switch or dial)
        ch5_norm = self.normalise(channels_us[4], is_throttle=True)
        mode_index = int(ch5_norm * (len(WaveMode) - 1) + 0.5)
        rc.wave_mode = WaveMode(max(0, min(len(WaveMode)-1, mode_index)))

        # Armed if throttle near zero AND yaw full right (standard arming)
        rc.armed = True  # Simplified: implement proper arm/disarm sequence

        return rc

# ─────────────────────────────────────────────
# WAVE GENERATOR
# ─────────────────────────────────────────────

class WaveGenerator:
    """
    Generates per-wing phase offsets based on:
      - Wave mode (metachronal, reverse, standing, etc.)
      - RC inputs (throttle, roll, pitch, yaw)
      - Time
    """

    def __init__(self):
        self.time_origin = time.time()

    def elapsed(self) -> float:
        return time.time() - self.time_origin

    def get_phase_offsets(self,
                          rc: RCInput,
                          mode: WaveMode,
                          freq: float) -> List[float]:
        """
        Returns list of 8 phase offsets (degrees) for each wing.
        Wings 0-3 = left side (W1L front to W4L rear)
        Wings 4-7 = right side (W1R front to W4R rear)

        Phase offset shifts when in the stroke cycle each wing starts.
        0° = in phase with master clock
        180° = opposite to master clock
        """
        offsets = [0.0] * NUM_WINGS

        # ── Base metachronal spacing ──────────────────────────
        spacing = DEFAULT_WAVE_SPACING_DEG  # 90° default

        if mode == WaveMode.METACHRONAL:
            # Left side: wave travels W1→W4 (front to back)
            for i in range(WINGS_PER_SIDE):
                offsets[i] = i * spacing           # 0,90,180,270

            # Right side: mirrors left (same direction)
            for i in range(WINGS_PER_SIDE):
                offsets[WINGS_PER_SIDE + i] = i * spacing

        elif mode == WaveMode.REVERSE_WAVE:
            # Wave travels W4→W1 (back to front) = reverse thrust
            for i in range(WINGS_PER_SIDE):
                offsets[i] = (WINGS_PER_SIDE - 1 - i) * spacing
            for i in range(WINGS_PER_SIDE):
                offsets[WINGS_PER_SIDE + i] = (WINGS_PER_SIDE - 1 - i) * spacing

        elif mode == WaveMode.STANDING:
            # W1,W3 together; W2,W4 opposite → alternating pairs
            for i in range(WINGS_PER_SIDE):
                offsets[i] = 0.0 if i % 2 == 0 else 180.0
            for i in range(WINGS_PER_SIDE):
                offsets[WINGS_PER_SIDE + i] = 0.0 if i % 2 == 0 else 180.0

        elif mode == WaveMode.IN_PHASE:
            # All 8 wings together — maximum force burst
            offsets = [0.0] * NUM_WINGS

        elif mode == WaveMode.COUNTER_PHASE:
            # Front pair vs rear pair opposed
            for i in range(WINGS_PER_SIDE):
                offsets[i] = 0.0 if i < 2 else 180.0
            for i in range(WINGS_PER_SIDE):
                offsets[WINGS_PER_SIDE + i] = 0.0 if i < 2 else 180.0

        # ── Apply RC stick modifications ──────────────────────

        # YAW: left side wave reversal vs right side
        # Positive yaw = left side reverse, right side forward
        if abs(rc.yaw) > RC_DEADBAND:
            yaw_phase_shift = rc.yaw * 180.0  # up to full reversal
            for i in range(WINGS_PER_SIDE):
                offsets[i] += yaw_phase_shift                  # left side
                offsets[WINGS_PER_SIDE + i] -= yaw_phase_shift # right side opposite

        # Normalise all offsets to 0–360°
        offsets = [o % 360.0 for o in offsets]

        return offsets

# ─────────────────────────────────────────────
# PER-WING KINEMATIC RESOLVER
# ─────────────────────────────────────────────

class KinematicResolver:
    """
    Given a wing's phase position in the stroke cycle (0–360°),
    plus control inputs, returns the 3 servo angles for that wing.
    """

    def resolve(self,
                cycle_phase_deg: float,
                aoa_power: float,
                aoa_recovery: float,
                stroke_amp: float,
                deviation_amp: float,
                side: str = 'left') -> WingKinematics:
        """
        cycle_phase_deg: current position in flap cycle (0-360°)
        side: 'left' or 'right' (mirrors stroke direction)
        """
        rad = math.radians(cycle_phase_deg)

        # ── Stroke (phi): sinusoidal ──────────────────────────
        phi = stroke_amp * math.sin(rad)
        if side == 'right':
            phi = -phi   # Mirror for right side

        # ── Pitch (psi): smoothed square wave ────────────────
        # Power stroke (0°–180°): hold at aoa_power
        # Recovery (180°–360°): hold at aoa_recovery (feathered)
        # Rapid flip at transitions (near 0° and 180°)
        flip_zone = 20.0   # degrees either side of transition
        if cycle_phase_deg < flip_zone:
            # Transition: recovery → power
            t = cycle_phase_deg / flip_zone
            psi = aoa_recovery + (aoa_power - aoa_recovery) * self._smooth_step(t)
        elif cycle_phase_deg < 180.0 - flip_zone:
            # Mid power stroke
            psi = aoa_power
        elif cycle_phase_deg < 180.0 + flip_zone:
            # Transition: power → recovery
            t = (cycle_phase_deg - (180.0 - flip_zone)) / (2 * flip_zone)
            psi = aoa_power + (aoa_recovery - aoa_power) * self._smooth_step(t)
        elif cycle_phase_deg < 360.0 - flip_zone:
            # Mid recovery stroke
            psi = aoa_recovery
        else:
            # Final transition back
            t = (cycle_phase_deg - (360.0 - flip_zone)) / flip_zone
            psi = aoa_recovery + (aoa_power - aoa_recovery) * self._smooth_step(t)

        # ── Deviation (theta): figure-8 modifier ─────────────
        # Runs at 2× flap frequency, adds slight vertical arc
        theta = deviation_amp * math.sin(2.0 * rad)

        return WingKinematics(phi=phi, psi=psi, theta=theta)

    @staticmethod
    def _smooth_step(t: float) -> float:
        """Cubic smoothstep: smooth 0→1 transition"""
        t = max(0.0, min(1.0, t))
        return t * t * (3.0 - 2.0 * t)

# ─────────────────────────────────────────────
# PID CONTROLLER
# ─────────────────────────────────────────────

class PIDController:
    def __init__(self, kp: float, ki: float, kd: float,
                 output_limit: float = 1.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.limit = output_limit
        self.state = PIDState()
        self._last_time = time.time()

    def update(self, setpoint: float, measured: float) -> float:
        now = time.time()
        dt = now - self._last_time
        if dt <= 0:
            return self.state.output
        self._last_time = now

        error = setpoint - measured

        # Proportional
        p = self.kp * error

        # Integral (with anti-windup clamp)
        self.state.integral += error * dt
        self.state.integral = max(-self.limit/self.ki if self.ki else 0,
                                  min(self.limit/self.ki if self.ki else 0,
                                      self.state.integral))
        i = self.ki * self.state.integral

        # Derivative
        d = self.kd * (error - self.state.prev_error) / dt
        self.state.prev_error = error

        output = p + i + d
        self.state.output = max(-self.limit, min(self.limit, output))
        return self.state.output

    def reset(self):
        self.state = PIDState()

# ─────────────────────────────────────────────
# FLIGHT CONTROLLER — MAIN CLASS
# ─────────────────────────────────────────────

class EightWingFlightController:
    """
    Main flight controller for 8-wing metachronal FPV drone.

    Usage:
        fc = EightWingFlightController()
        fc.start()

        # In main loop:
        rc_channels = read_rc_input()   # your hardware interface
        imu_data    = read_imu()        # your hardware interface
        servo_us    = fc.update(rc_channels, imu_data)
        write_servos(servo_us)          # your hardware interface
    """

    def __init__(self, flap_freq: float = FLAP_FREQ_HZ):
        self.freq       = flap_freq
        self.rc_parser  = RCParser()
        self.wave_gen   = WaveGenerator()
        self.resolver   = KinematicResolver()
        self.faults     = WingFaultState()

        # PID controllers for attitude stabilisation
        self.pid_roll  = PIDController(*PID_ROLL,  output_limit=0.5)
        self.pid_pitch = PIDController(*PID_PITCH, output_limit=0.5)
        self.pid_yaw   = PIDController(*PID_YAW,   output_limit=0.4)

        # State
        self.armed       = False
        self.flight_mode = FlightMode.DISARMED

    # ── Main update — call at CONTROL_HZ ─────────────────────

    def update(self,
               rc_channels_us: List[int],
               imu: IMUData) -> List[int]:
        """
        Returns list of 24 servo PWM values (microseconds).
        Index: wing*3+0=stroke, wing*3+1=pitch, wing*3+2=deviation
        """
        # 1. Parse RC input
        rc = self.rc_parser.parse(rc_channels_us)

        if not rc.armed:
            return [SERVO_CENTRE_US] * (NUM_WINGS * 3)

        # 2. Determine flight mode from throttle
        self.flight_mode = self._classify_flight_mode(rc)

        # 3. PID stabilisation — correct attitude errors
        roll_correction  = self.pid_roll.update(rc.roll * 30.0,
                                                 imu.roll_deg)
        pitch_correction = self.pid_pitch.update(rc.pitch * 20.0,
                                                  imu.pitch_deg)
        yaw_correction   = self.pid_yaw.update(rc.yaw * 45.0,
                                                imu.yaw_rate)

        # 4. Base parameters from flight mode
        base_aoa_power, base_aoa_rec, base_stroke_amp = \
            self._mode_parameters(self.flight_mode, rc.throttle)

        # 5. Get current cycle phase for each wing
        t = self.wave_gen.elapsed()
        cycle_master_deg = (t * self.freq * 360.0) % 360.0

        # 6. Get phase offsets from wave generator
        phase_offsets = self.wave_gen.get_phase_offsets(
            rc, rc.wave_mode, self.freq
        )

        # 7. Resolve kinematics for all 8 wings
        servo_outputs = []

        for wing_idx in range(NUM_WINGS):
            # Skip failed wings — output neutral
            if self.faults.failed[wing_idx]:
                servo_outputs.extend([SERVO_CENTRE_US] * 3)
                continue

            side = 'left' if wing_idx < WINGS_PER_SIDE else 'right'
            side_idx = wing_idx % WINGS_PER_SIDE  # 0=front, 3=rear

            # Wing's current cycle position
            wing_phase = (cycle_master_deg + phase_offsets[wing_idx]) % 360.0

            # Per-wing AoA modifications for roll/pitch/yaw control
            aoa_power, stroke_amp = self._apply_control_mixing(
                side, side_idx,
                base_aoa_power, base_stroke_amp,
                rc, roll_correction, pitch_correction, yaw_correction
            )

            # Resolve to servo angles
            kin = self.resolver.resolve(
                wing_phase,
                aoa_power,
                base_aoa_rec,
                stroke_amp,
                DEVIATION_AMPLITUDE,
                side
            )

            # Convert angles to servo microseconds
            phi_us   = self._angle_to_us(kin.phi)
            psi_us   = self._angle_to_us(kin.psi)
            theta_us = self._angle_to_us(kin.theta)

            servo_outputs.extend([phi_us, psi_us, theta_us])

        return servo_outputs

    # ── Control mixing ────────────────────────────────────────

    def _apply_control_mixing(self,
                               side: str,
                               side_idx: int,
                               base_aoa: float,
                               base_amp: float,
                               rc: RCInput,
                               roll_corr: float,
                               pitch_corr: float,
                               yaw_corr: float
                               ) -> Tuple[float, float]:
        """
        Applies roll, pitch, yaw corrections to individual wing parameters.
        Returns (modified_aoa_power, modified_stroke_amplitude)
        """
        aoa = base_aoa
        amp = base_amp
        is_left = (side == 'left')
        is_front = (side_idx < 2)  # W1, W2 are front; W3, W4 are rear

        # ── ROLL (differential amplitude L vs R) ─────────────
        roll_input = rc.roll + roll_corr
        roll_amp_delta = roll_input * base_amp * 0.35
        if is_left:
            amp -= roll_amp_delta   # reduce left on positive roll
        else:
            amp += roll_amp_delta   # increase right on positive roll

        # ── PITCH (differential AoA front vs rear) ───────────
        pitch_input = rc.pitch + pitch_corr
        pitch_aoa_delta = pitch_input * 12.0  # max ±12° AoA shift
        if is_front:
            aoa += pitch_aoa_delta   # front wings increase AoA = nose down
        else:
            aoa -= pitch_aoa_delta   # rear wings reduce AoA

        # ── YAW handled in wave generator (phase offsets) ────
        # But also apply small AoA asymmetry for torque augmentation
        yaw_input = rc.yaw + yaw_corr
        yaw_aoa_delta = abs(yaw_input) * 5.0
        if (is_left and yaw_input > 0) or (not is_left and yaw_input < 0):
            aoa += yaw_aoa_delta
        else:
            aoa -= yaw_aoa_delta

        # Clamp to safe ranges
        aoa = max(5.0, min(50.0, aoa))
        amp = max(20.0, min(85.0, amp))

        return aoa, amp

    # ── Flight mode classifier ────────────────────────────────

    def _classify_flight_mode(self, rc: RCInput) -> FlightMode:
        if rc.throttle < 0.05:
            return FlightMode.DESCEND
        elif rc.throttle > 0.85:
            return FlightMode.CLIMB
        elif rc.pitch < -0.5:
            return FlightMode.FLARE
        else:
            return FlightMode.HOVER  # Hover as default stable mode

    # ── Mode parameters ───────────────────────────────────────

    def _mode_parameters(self,
                          mode: FlightMode,
                          throttle: float
                          ) -> Tuple[float, float, float]:
        """Returns (aoa_power, aoa_recovery, stroke_amplitude)"""
        # Scale stroke amplitude with throttle
        amp = STROKE_AMPLITUDE * (0.5 + 0.5 * throttle)

        table = {
            FlightMode.HOVER:   (35.0, 5.0,  amp),
            FlightMode.CLIMB:   (42.0, 8.0,  amp * 1.15),
            FlightMode.CRUISE:  (30.0, 3.0,  amp),
            FlightMode.DESCEND: (20.0, 20.0, amp * 0.7),  # symmetric drag
            FlightMode.FLARE:   (45.0, 40.0, amp),         # braking
            FlightMode.DISARMED:(0.0,  0.0,  0.0),
        }
        return table.get(mode, (35.0, 5.0, amp))

    # ── Fault handling ────────────────────────────────────────

    def declare_fault(self, wing_idx: int):
        """
        Called when a servo or wing failure is detected.
        Compensates by adjusting opposite-side wing.
        """
        self.faults.failed[wing_idx] = True
        print(f"[FAULT] Wing {wing_idx} failed — compensating")

        # Mirror wing on opposite side
        if wing_idx < WINGS_PER_SIDE:
            mirror = wing_idx + WINGS_PER_SIDE
        else:
            mirror = wing_idx - WINGS_PER_SIDE

        # Increase mirror wing amplitude to compensate
        print(f"[FAULT] Increasing amplitude on wing {mirror}")
        # In real implementation: adjust per-wing amplitude table

    # ── Utility ──────────────────────────────────────────────

    @staticmethod
    def _angle_to_us(angle_deg: float) -> int:
        """Convert servo angle (degrees) to PWM microseconds"""
        us = SERVO_CENTRE_US + (angle_deg / 90.0) * SERVO_RANGE_US
        return int(max(RC_MIN_US, min(RC_MAX_US, us)))


# ─────────────────────────────────────────────
# SIMULATION / VISUALISER (no hardware needed)
# ─────────────────────────────────────────────

class Simulator:
    """
    Runs the flight controller with mock RC and IMU data.
    Prints servo outputs for inspection and tuning.
    Use this to verify waveforms before connecting hardware.
    """

    def __init__(self):
        self.fc = EightWingFlightController()

    def mock_rc(self, throttle: float = 0.5,
                roll: float = 0.0,
                pitch: float = 0.0,
                yaw: float = 0.0,
                wave_mode: int = 0) -> List[int]:
        """Generate mock RC PWM values"""
        def to_us(norm, is_throttle=False):
            if is_throttle:
                return int(RC_MIN_US + norm * (RC_MAX_US - RC_MIN_US))
            return int(RC_MID_US + norm * (RC_MAX_US - RC_MID_US))

        return [
            to_us(roll),
            to_us(pitch),
            to_us(throttle, is_throttle=True),
            to_us(yaw),
            to_us(wave_mode / 4.0, is_throttle=True)
        ]

    def mock_imu(self, roll=0.0, pitch=0.0, yaw=0.0) -> IMUData:
        return IMUData(roll_deg=roll, pitch_deg=pitch, yaw_deg=yaw)

    def run(self, duration_sec: float = 3.0, scenario: str = 'hover'):
        """
        Run a simulation scenario and print servo outputs.
        Scenarios: 'hover', 'roll_left', 'yaw_right', 'climb', 'reverse'
        """
        print(f"\n{'='*60}")
        print(f"  8-Wing Metachronal Drone Simulator")
        print(f"  Scenario: {scenario.upper()}  Duration: {duration_sec}s")
        print(f"{'='*60}\n")

        scenarios = {
            'hover':      dict(throttle=0.5, roll=0.0,  pitch=0.0,  yaw=0.0,  wave_mode=0),
            'roll_left':  dict(throttle=0.5, roll=-0.6, pitch=0.0,  yaw=0.0,  wave_mode=0),
            'yaw_right':  dict(throttle=0.5, roll=0.0,  pitch=0.0,  yaw=0.6,  wave_mode=0),
            'climb':      dict(throttle=0.8, roll=0.0,  pitch=0.0,  yaw=0.0,  wave_mode=0),
            'reverse':    dict(throttle=0.5, roll=0.0,  pitch=0.0,  yaw=0.0,  wave_mode=1),
            'max_force':  dict(throttle=1.0, roll=0.0,  pitch=0.0,  yaw=0.0,  wave_mode=3),
        }

        params = scenarios.get(scenario, scenarios['hover'])
        rc_us  = self.mock_rc(**params)
        imu    = self.mock_imu()

        dt = 1.0 / CONTROL_HZ
        steps = int(duration_sec / dt)
        print(f"{'Time':>8} | {'W0φ':>5} {'W0ψ':>5} | {'W1φ':>5} {'W1ψ':>5} | "
              f"{'W4φ':>5} {'W4ψ':>5} | {'W7φ':>5} {'W7ψ':>5}")
        print('-' * 70)

        for step in range(steps):
            t = step * dt
            servos = self.fc.update(rc_us, imu)

            # Print every 10th step (5 Hz display)
            if step % 10 == 0:
                # Extract stroke and pitch for wings 0,1,4,7
                def s(w, ch): return servos[w*3+ch]
                def deg(us): return round((us - SERVO_CENTRE_US) / SERVO_RANGE_US * 90, 1)

                print(f"{t:>8.2f} | "
                      f"{deg(s(0,0)):>5} {deg(s(0,1)):>5} | "
                      f"{deg(s(1,0)):>5} {deg(s(1,1)):>5} | "
                      f"{deg(s(4,0)):>5} {deg(s(4,1)):>5} | "
                      f"{deg(s(7,0)):>5} {deg(s(7,1)):>5}")

            time.sleep(dt)

        print(f"\nSimulation complete. {steps} control cycles executed.")
        print(f"Columns: φ=stroke angle, ψ=pitch/feather (degrees)")
        print(f"Wings shown: W0L(front-left), W1L, W4R(front-right), W7R(rear-right)")


# ─────────────────────────────────────────────
# HARDWARE INTERFACE STUBS
# (Replace with real drivers for your platform)
# ─────────────────────────────────────────────

class HardwareInterface:
    """
    Stub interface — replace method bodies with real hardware calls.

    For Raspberry Pi:
      - RC input: pigpio library for PWM reading, or pyserial for SBUS
      - IMU: smbus2 for MPU-6050, or adafruit-circuitpython-bno055
      - Servos: adafruit-pca9685 library (two boards for 24 channels)
    """

    def read_rc_channels(self) -> List[int]:
        """Read 5 RC channels, return list of microsecond values"""
        # TODO: Replace with real RC input driver
        # Example using pigpio:
        #   return [pigpio_instance.get_servo_pulsewidth(pin) for pin in RC_PINS]
        raise NotImplementedError("Implement RC input for your hardware")

    def read_imu(self) -> IMUData:
        """Read IMU and return fused attitude estimate"""
        # TODO: Replace with real IMU driver
        # Example using MPU-6050 via smbus2:
        #   raw = mpu.get_all_data()
        #   return IMUData(roll_deg=raw.roll, ...)
        raise NotImplementedError("Implement IMU driver for your hardware")

    def write_servos(self, servo_us: List[int]):
        """Write 24 servo PWM values"""
        # TODO: Replace with real servo driver
        # Example using two PCA9685 boards:
        #   for i, us in enumerate(servo_us):
        #       board = pca_board_0 if i < 16 else pca_board_1
        #       channel = i if i < 16 else i - 16
        #       board.channels[channel].duty_cycle = us_to_duty(us)
        raise NotImplementedError("Implement servo output for your hardware")


# ─────────────────────────────────────────────
# MAIN — SIMULATION OR REAL HARDWARE
# ─────────────────────────────────────────────

def main():
    """
    Run in simulation mode (no hardware needed).
    To run on real hardware: replace Simulator with HardwareInterface
    and call fc.update() in a timed loop.
    """
    sim = Simulator()

    # Run through several scenarios
    for scenario in ['hover', 'roll_left', 'yaw_right', 'climb', 'reverse']:
        sim.run(duration_sec=2.0, scenario=scenario)
        time.sleep(0.2)


if __name__ == '__main__':
    main()
