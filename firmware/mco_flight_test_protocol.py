"""
MCO Flight Test Protocol
=========================
Rev A  2026-06-16

Grounded in:
  FAA AC 90-89C (2023) — Amateur-Built Aircraft and Ultralight Flight Testing
  EAA Flight Test Manual v2 (2024) — Task-Based Phase I Program
  EAA Stage Three: Expanding the Flight Envelope (Bingelis, EAA Sport Aviation)
  FAA PHAK (FAA-H-8083-25B) Ch. 4-5, 10-11

The MCO is unprecedented in one specific way: it is a multi-wing
metachronal cascading ornithopter. No envelope expansion programme
for this configuration has ever been written. This protocol therefore
adapts FAA/EAA doctrine to the MCO's unique parameters — honouring
every principle while addressing the specific tests only a flapping-wing
aircraft with variable sweep, dihedral, amplitude, and cascade phase
requires.

FOR BOTH AIRCRAFT:
  MCO-1D: drone testbed, no pilot, all phases can be executed autonomously
  MCO-1A: piloted SAR aircraft, all safety margins more conservative

The MCO-1D is the PRIMARY TEST VEHICLE. Every envelope limit is first
found on the drone before the MCO-1A is ever considered.
"""

import math, json, time, os

# ─────────────────────────────────────────────────────────────────
# AIRCRAFT CONSTANTS (from wave library)
# ─────────────────────────────────────────────────────────────────
MCO_1D = {
    'MTOW_KG':       5.687,
    'VS_KT':         12.77,
    'VS_MS':         6.57,
    'VNE_KT':        54.4,
    'VNE_MS':        28.0,
    'CL_MAX':        1.60,
    'S_M2':          1.320,
    'CHORD_MM':      143.7,
    'FREQ_MAX_HZ':   3.0,
    'AMP_DEG':       96.0,
    'PHASE_DEG':     60.0,
    'PILOTS':        0,
    'WINGSPAN_M':    2.348,
}

MCO_1A = {
    'MTOW_KG':       255.0,
    'VS_MS':         13.4,    # 30 mph
    'VS_KT':         26.0,
    'VNE_KT':        None,    # to be determined by test programme
    'CL_MAX':        2.60,    # GA(W)-1 + VGs + coverts
    'S_M2':          8.70,
    'CHORD_MM':      602,
    'FREQ_MAX_HZ':   2.0,
    'AMP_DEG':       80.0,
    'PHASE_DEG':     60.0,
    'PILOTS':        1,
    'WINGSPAN_M':    3.614,
}

# ─────────────────────────────────────────────────────────────────
# TEST POINT STRUCTURE
# ─────────────────────────────────────────────────────────────────
class TestCard:
    """
    Mirrors the EAA Flight Test Card format.
    Each card: objective, preconditions, procedure, success criteria,
    abort criteria, data to record, and next step.
    """
    def __init__(self, number, title, phase, aircraft='both'):
        self.number    = number
        self.title     = title
        self.phase     = phase          # PHASE_1 through PHASE_5
        self.aircraft  = aircraft       # 'MCO-1D', 'MCO-1A', or 'both'
        self.objective    = ''
        self.preconditions = []
        self.procedure    = []
        self.success      = []
        self.abort        = []
        self.data         = []
        self.after        = []          # what to do after this card
        self.expand_with  = []          # what expanding this limit enables
        self.sw_expand    = []          # software changes that push limit further
        self.hw_expand    = []          # hardware changes that push limit further

    def render(self) -> str:
        lines = []
        lines.append(f"{'='*60}")
        lines.append(f"TEST CARD {self.number:02d} — {self.title.upper()}")
        lines.append(f"Phase: {self.phase}  |  Aircraft: {self.aircraft}")
        lines.append(f"{'='*60}")
        lines.append(f"\nOBJECTIVE:")
        lines.append(f"  {self.objective}")
        if self.preconditions:
            lines.append(f"\nPRECONDITIONS:")
            for p in self.preconditions:
                lines.append(f"  ✓ {p}")
        if self.procedure:
            lines.append(f"\nPROCEDURE:")
            for i,p in enumerate(self.procedure, 1):
                lines.append(f"  {i}. {p}")
        if self.success:
            lines.append(f"\nSUCCESS CRITERIA:")
            for s in self.success:
                lines.append(f"  ✓ {s}")
        if self.abort:
            lines.append(f"\nABORT CRITERIA (revert / RTB immediately):")
            for a in self.abort:
                lines.append(f"  ✗ {a}")
        if self.data:
            lines.append(f"\nDATA TO RECORD:")
            for d in self.data:
                lines.append(f"  → {d}")
        if self.after:
            lines.append(f"\nAFTER THIS CARD:")
            for a in self.after:
                lines.append(f"  • {a}")
        if self.expand_with:
            lines.append(f"\nEXPANDING THIS LIMIT ENABLES:")
            for e in self.expand_with:
                lines.append(f"  ↑ {e}")
        if self.sw_expand:
            lines.append(f"\nSOFTWARE CHANGES TO PUSH THIS LIMIT FURTHER:")
            for s in self.sw_expand:
                lines.append(f"  [SW] {s}")
        if self.hw_expand:
            lines.append(f"\nHARDWARE CHANGES TO PUSH THIS LIMIT FURTHER:")
            for h in self.hw_expand:
                lines.append(f"  [HW] {h}")
        return '\n'.join(lines)


def build_protocol():
    cards = []

    # ─────────────────────────────────────────────────────────────
    # PHASE 1 — GROUND AND TAXI (Hours 0-2, per AC 90-89C Ch.3)
    # FAA: "Verify aircraft is controllable on the ground before
    #        attempting any airborne operation"
    # ─────────────────────────────────────────────────────────────

    tc = TestCard(1, "Static Ground Run — Drive System Verification",
                  "PHASE 1 — GROUND", "MCO-1D")
    tc.objective = ("Verify drive system, ODrive, and wing mechanisms "
                    "operate correctly through full stroke range at all "
                    "frequencies before any flight attempt.")
    tc.preconditions = [
        "Aircraft secured to ground anchors rated for 4× MTOW",
        "ODrive S1 firmware configured (vel_limit=150, current_lim=20A, 8 boards)",
        "All 8 wings installed and free to move",
        "Telemetry link confirmed active",
        "All personnel behind safety line (≥3m from wing tips)",
    ]
    tc.procedure = [
        "Spin up at f=0.5 Hz, AMP=60°. Verify all 8 wings respond symmetrically.",
        "Increase to f=1.0 Hz. Record ODrive current per axis.",
        "Increase to f=2.0 Hz. Check for vibration, rope slip, binding.",
        "Increase to f=3.0 Hz (MAX). Hold for 30 seconds. Record motor temperature.",
        "Reduce to 0 Hz. Inspect all rope runs, pulleys, and sector wheels.",
        "Repeat at AMP=96° (proposed sector wheel upgrade). Verify clearances.",
        "Test all sweep positions (0°, 60°, 90°, 120°, 210°) while flapping at 1 Hz.",
        "Test all dihedral positions (-20° to +60°) while flapping at 1 Hz.",
        "Execute CF-1 burst mode 3× at f=2.5 Hz. Record peak current.",
        "Full stop. Inspect all drive components for wear.",
    ]
    tc.success = [
        "All 8 wings respond symmetrically within ±5% of commanded amplitude",
        "Motor current at 3 Hz ≤ 30A (within limit with margin)",
        "No rope slip, binding, or sector wheel end-stop collision",
        "Estimated motor temperature rise ≤ 50°C after 30s at 3 Hz",
        "Telemetry data logged for all axes",
    ]
    tc.abort = [
        "Any wing fails to move symmetrically",
        "Motor current exceeds 36A on any axis",
        "Rope slippage detected (velocity error > 10 rad/s on ODrive)",
        "Visible structural deformation in any component",
        "Smoke or burning smell from any motor or controller",
    ]
    tc.data = [
        "Current (A) per ODrive axis at each frequency",
        "Peak current during CF-1 burst",
        "Motor temperature estimate (MotorThermalMonitor output)",
        "Position error (rad) per axis at each frequency",
        "Any asymmetry between port and starboard wings",
    ]
    tc.after = [
        "Record all data in flight log",
        "Address any abort conditions before proceeding to TC-02",
        "Update FLAP_AMP_LIMIT_MAX_DEG if amplitude testing is successful",
    ]
    tc.sw_expand = [
        "Increase vel_limit to 200 rad/s via odrivetool if 150 insufficient",
        "Increase FLAP_AMP_LIMIT_MAX_DEG from 85° to 96° if sector wheel extended",
        "Enable MotorThermalMonitor advisory at 80°C rise",
    ]
    tc.hw_expand = [
        "Extend sector wheel arc from 160° to 192° for ±96° amplitude",
        "Switch to 0.75mm SK75 Dyneema for better compliance",
        "Install urethane bump stops at ±96° end stops",
    ]
    cards.append(tc)

    tc = TestCard(2, "Tethered Hover / Lift Test",
                  "PHASE 1 — GROUND", "MCO-1D")
    tc.objective = ("Verify the aircraft generates sufficient lift to exceed "
                    "MTOW under controlled conditions. Calibrate CT from "
                    "direct thrust measurement.")
    tc.preconditions = [
        "TC-01 completed successfully",
        "Load cell rated ≥10× MTOW (35N minimum) installed beneath aircraft",
        "Aircraft suspended 0.5m above load cell on tension line",
        "Safety tether prevents vehicle from exceeding 1m altitude",
        "Wind < 5 kt",
    ]
    tc.procedure = [
        "Start at f=1.0 Hz, AMP=80°. Record lift from load cell.",
        "Increase frequency in 0.25 Hz steps to 3.0 Hz. Record lift at each step.",
        "At each frequency, hold for 10s, average reading.",
        "Repeat at AMP=90°, AMP=96° (if sector wheel permits).",
        "Record the frequency at which lift = MTOW (hover point).",
        "Record the frequency at which lift = 1.3 × MTOW (safety margin).",
        "Calculate CT from measured thrust: CT = T / (0.5 × ρ × V_stroke² × S)",
    ]
    tc.success = [
        "Aircraft achieves lift ≥ MTOW (5.687 × 9.81 = 55.8N) at some f ≤ 3 Hz",
        "CT value confirms or updates wave library placeholder (1.13)",
        "Lift scales as f² with R² > 0.95 (confirms T ∝ f² scaling)",
        "Lift scales as AMP² with R² > 0.90 (confirms T ∝ AMP² scaling)",
    ]
    tc.abort = [
        "Tether tension > 50N (structural overload risk)",
        "Any structural deformation visible",
        "Lift does not increase monotonically with frequency",
    ]
    tc.data = [
        "Lift force (N) vs frequency at each amplitude — PRIMARY CT CALIBRATION",
        "CT value computed from data",
        "f_hover (Hz) — frequency at which lift = MTOW",
        "AMP² scaling exponent (should be 2.0)",
    ]
    tc.after = [
        "Update CT constant in wave library (# CAL: → measured value)",
        "Update all performance predictions based on measured CT",
        "Proceed to TC-03 only if hover point f_hover ≤ 2.8 Hz (margin to 3.0 Hz)",
    ]
    tc.expand_with = [
        "CT calibration unlocks all absolute performance predictions",
        "Measured f_hover tells us exactly how much safety margin we have",
        "AMP² calibration validates the ±96° thrust benefit calculation",
    ]
    tc.sw_expand = [
        "Update CT_EST in mco_wave_library.py with measured value",
        "Update FREQ_LIMIT_MAX_HZ if hover is achievable below 2.5 Hz",
        "Enable the Ps target table modulation by density altitude",
    ]
    tc.hw_expand = [
        "If f_hover > 2.8 Hz: extend sector wheel to ±96° IMMEDIATELY",
        "  (±96° reduces f_hover by factor √(1/1.35) = 0.86 → saves ~0.4 Hz margin)",
        "If f_hover > 3.0 Hz: aircraft cannot hover — redesign required",
    ]
    cards.append(tc)

    # ─────────────────────────────────────────────────────────────
    # PHASE 2 — FIRST FLIGHT (Hours 2-10, per AC 90-89C Ch.4-5)
    # FAA: "Fly conservatively. Remain within gliding distance of the
    #        runway at all times during the first 10 hours."
    # ─────────────────────────────────────────────────────────────

    tc = TestCard(3, "First Flight — Controllability Demonstration",
                  "PHASE 2 — FIRST FLIGHT", "MCO-1D")
    tc.objective = ("Demonstrate basic controllability in all three axes. "
                    "Confirm stable flight at cruise parameters. "
                    "Per AC 90-89C §4.9: minimum 10-minute flight, "
                    "remain within gliding distance of runway.")
    tc.preconditions = [
        "TC-01 and TC-02 completed successfully",
        "CT measured and wave library updated",
        "f_hover confirmed ≤ 2.8 Hz",
        "All preflight checks complete (see mco_preflight.py)",
        "Chase observer with binoculars and radio",
        "Emergency recovery waypoint programmed in Pixhawk",
        "Wind < 8 kt, gusts < 4 kt",
        "Visibility > 3 miles",
        "Ceiling > 1000 ft AGL",
    ]
    tc.procedure = [
        "Arm. Initiate takeoff intent. Verify takeoff phase activates.",
        "Climb to 10m AGL at standard climb rate. Verify climb intent.",
        "Level at 10m. Verify cruise intent activates at f ≈ 1 Hz.",
        "Fly a 200m straight line at cruise speed. Observe tracking.",
        "Execute a 90° left turn. Observe bank angle and coordination.",
        "Execute a 90° right turn. Observe symmetry with left turn.",
        "Extend to 300m from launch point. Observe telemetry quality.",
        "Return to launch. Execute standard landing pattern.",
        "Land. Post-flight inspection.",
    ]
    tc.success = [
        "Aircraft climbs to 10m AGL without oscillation",
        "Level flight maintained ±2m altitude for 60 seconds",
        "Turns executed with bank ≤ 20° and no altitude loss > 3m",
        "Telemetry link unbroken throughout",
        "All motor currents within expected range",
        "Landing within 10m of target",
    ]
    tc.abort = [
        "Any uncontrolled oscillation in pitch, roll, or yaw",
        "Altitude loss > 3m during level flight attempt",
        "Motor current spike > 30A on any axis",
        "Telemetry link loss > 2 seconds",
        "Any structural noise (wing flutter, rope snap)",
        "GPS lock lost",
    ]
    tc.data = [
        "Actual vs commanded trajectory (GPS)",
        "Motor currents at cruise (CT calibration confirmation)",
        "Actual cruise frequency achieved",
        "Pitch, roll, yaw angles throughout flight",
        "Battery voltage sag at various loads",
    ]
    tc.after = [
        "Full structural inspection after first flight",
        "Compare actual cruise frequency to TC-02 prediction",
        "Proceed to TC-04 only after ≥5 successful flights at this level",
    ]
    cards.append(tc)

    # ─────────────────────────────────────────────────────────────
    # PHASE 3 — ENVELOPE EXPANSION (Hours 10-25, per AC 90-89C Ch.6)
    # FAA §6.1: "Gradually expand. Never more than one variable
    #             at a time. Always have a safe escape route."
    # EAA: "Perform potentially dangerous tests later in the program."
    # ─────────────────────────────────────────────────────────────

    tc = TestCard(4, "Stall Characteristics — Clean Configuration",
                  "PHASE 3 — ENVELOPE EXPANSION", "MCO-1D")
    tc.objective = ("Establish the actual stall speed and stall behaviour "
                    "in clean configuration. Per AC 90-89C §6.3: "
                    "'Approach the stall gradually. Do not attempt "
                    "to stall the aircraft abruptly.'")
    tc.preconditions = [
        "≥5 successful Phase 2 flights completed",
        "Minimum altitude 200m AGL for all stall tests",
        "Emergency recovery mode tested and confirmed active",
        "Wind < 6 kt, smooth air (no thermals)",
        "Test area clear of obstacles",
    ]
    tc.procedure = [
        "Climb to 200m AGL.",
        "Set cruise intent. Note airspeed (V_cruise).",
        "Reduce frequency gradually in 0.1 Hz steps.",
        "At each step: hold for 5 seconds, record airspeed and altitude.",
        "Approach stall: note buffet onset (if any) and Vs.",
        "At first sign of stall (altitude loss > 2m in 2s): "
            "immediately activate CF-1 stall recovery.",
        "Record recovery altitude lost.",
        "Repeat 3 times. Average the stall speed.",
        "Compare to predicted Vs = 4.85 m/s (9.42 kt).",
    ]
    tc.success = [
        "Stall speed within ±15% of predicted 4.85 m/s",
        "CF-1 recovery complete within 10m altitude loss",
        "No structural sounds during stall approach or recovery",
        "Consistent stall speed across 3 repetitions (±5%)",
    ]
    tc.abort = [
        "Altitude loss > 20m before recovery completes",
        "Any structural noise or visible wing deformation",
        "Uncommanded roll at stall > 30°",
        "Recovery altitude < 50m AGL",
    ]
    tc.data = [
        "Actual stall speed V_S1 (m/s and kt) — UPDATE WAVE LIBRARY",
        "Altitude lost in stall recovery (m)",
        "CF-1 recovery time (seconds)",
        "Any stall warning characteristics (buffet, pitch drop)",
        "Comparison: actual vs predicted stall speed",
    ]
    tc.after = [
        "Update VS_KT in wave library with measured value",
        "Update stall_speed_check() safety margins if Vs differs > 10% from prediction",
        "Establish 1.3×Vs as minimum manoeuvre speed",
        "Proceed to TC-05 (stall in turns) only after 3 clean TC-04 results",
    ]
    tc.expand_with = [
        "Measured Vs calibrates all subsequent stall margin calculations",
        "Recovery altitude tells us how much AGL margin we need for stall tests",
        "Clean stall must be established before any loaded or banked stall attempts",
    ]
    tc.sw_expand = [
        "If Vs < 4.5 m/s: lower VS_KT — more CL than predicted. "
            "Increase CF-1 frequency ceiling by 0.2 Hz",
        "If Vs > 5.5 m/s: raise VS_KT — less CL than predicted. "
            "Reduce stall_margin_min_ms safety gate accordingly",
        "Tune stall_speed_check() STALL_APPROACH_FREQ_HZ to measured f_hover",
    ]
    tc.hw_expand = [
        "If recovery takes > 15m altitude: extend sector wheel to ±96° "
            "for faster burst recovery thrust",
        "If stall is abrupt (no buffet): add covert flap rows to W1+W2 "
            "to soften stall onset character",
    ]
    cards.append(tc)

    tc = TestCard(5, "Maximum Speed (Vne Approach) — Speed Envelope",
                  "PHASE 3 — ENVELOPE EXPANSION", "MCO-1D")
    tc.objective = ("Establish the actual Vne by approaching maximum speed "
                    "incrementally and checking for flutter, control "
                    "reversal, or structural instability. "
                    "Per EAA Stage Three: 'Flutter is the most dangerous "
                    "characteristic to explore. Approach with extreme caution.'")
    tc.preconditions = [
        "≥10 successful flights. Stall speed measured (TC-04 complete).",
        "All structural fasteners inspected and re-torqued",
        "Wing skin inspected for delamination or loose sections",
        "Minimum altitude 300m AGL for all high-speed tests",
        "All wings stowed (210°) and locked before sprint",
    ]
    tc.procedure = [
        "Climb to 300m. Set stow intent.",
        "Verify W1+W2 stowed, W3+W4 in sprint configuration.",
        "Accelerate to V_cruise + 5 mph. Hold 5 seconds. Check for vibration.",
        "If smooth: accelerate to V_cruise + 10 mph. Hold 5 seconds.",
        "Continue in 5 mph steps.",
        "At each speed: check IMU vibration, wing visual inspection via camera.",
        "If ANY vibration increase or unusual behaviour: STOP, decelerate gently.",
        "Establish Vne as the highest speed with no vibration, flutter, or "
            "control anomaly. Set as Vne - 10% for initial operations.",
        "Do not chase a specific target Vne — find the actual limit.",
    ]
    tc.success = [
        "Speed increased in controlled steps with no flutter observed",
        "IMU vibration RMS stays below 2× cruise baseline at all speeds",
        "No control reversal or uncommanded motion at any tested speed",
        "Wing skin remains attached at all tested speeds",
    ]
    tc.abort = [
        "IMU vibration suddenly increases (flutter onset signal)",
        "Uncommanded roll or pitch excursion > 15°",
        "Any structural sound from wing or drive",
        "Control response becomes sluggish or reversed",
        "Battery voltage drops below safe minimum",
    ]
    tc.data = [
        "Airspeed at each test point (GPS-derived TAS)",
        "IMU vibration RMS at each speed step",
        "Motor current at each speed step",
        "Actual Vne established (mph and kt)",
        "Photos/video of wings at each speed step",
    ]
    tc.after = [
        "Set Vne in wave library operating limits",
        "Establish Vno (normal operating speed) = 0.9 × Vne",
        "Establish caution speed range (yellow arc) from Vno to Vne",
    ]
    tc.sw_expand = [
        "Sweep configuration at sprint — test W1+W2 at different stow angles "
            "to find lowest-drag stow position",
        "Adjust sprint_phase_offset for best thrust at high speed",
        "Implement flutter monitoring in flight_recorder (track IMU FFT peaks)",
    ]
    tc.hw_expand = [
        "If flutter onset below 80 mph: add mass balance weights to W3+W4 tips",
        "If flutter onset below 60 mph: wing spar stiffness review required",
        "Oratex skin tension adjustment — higher tension raises flutter speed",
        "TE stow slot receivers reduce stowed-wing buffet → raise Vne",
    ]
    cards.append(tc)

    tc = TestCard(6, "Climb Performance — Vy and Vx",
                  "PHASE 3 — ENVELOPE EXPANSION", "MCO-1D")
    tc.objective = ("Establish best rate of climb speed (Vy) and best "
                    "angle of climb speed (Vx). Per AC 90-89C §5.7: "
                    "'Climbs and descents should be gradually explored.'")
    tc.preconditions = [
        "Stall speed confirmed (TC-04 complete)",
        "Air smooth, wind < 5 kt",
        "Start altitude 50m AGL, climb ceiling 300m AGL",
    ]
    tc.procedure = [
        "At V = 1.3×Vs: climb at max frequency (3.0 Hz). "
            "Record altitude gain per minute.",
        "Increase speed in 5 mph steps up to 80% Vne.",
        "At each speed: hold heading, record altitude gain rate over 30s.",
        "Plot climb rate vs airspeed → Vy is peak of the curve.",
        "At Vy: hold altitude, reduce frequency until rate = 100 fpm. "
            "Record frequency and speed → confirms Ps model.",
        "At 1.3×Vs: repeat with maximum available climb angle. "
            "Record ground covered per foot of altitude → Vx.",
    ]
    tc.success = [
        "Vy identified with R/C variation > 50 fpm between adjacent test points",
        "Vx identified (steepest angle speed, lower than Vy)",
        "Measured R/C within 30% of predicted (CT now calibrated)",
    ]
    tc.abort = [
        "Altitude below 50m AGL at any point",
        "Motor current > 30A during max climb",
    ]
    tc.data = [
        "R/C (fpm) vs airspeed table — VY and VX values",
        "Maximum climb rate achieved (fpm) — UPDATE PERF MODEL",
        "Frequency at Vy — confirms f-efficiency relationship",
        "Time to climb 200m from 50m — absolute performance datum",
    ]
    tc.after = [
        "Update mco_performance_model.py with measured Vy, Vx, R/C",
        "Enable PhaseOptimiser (it can now run safely)",
    ]
    tc.sw_expand = [
        "Once Vy confirmed: set CLIMB_TARGET_V_MS in wave library",
        "Boyd controller Ps targets calibrated to actual R/C vs Ps relationship",
        "Enable CascadeOptimiser after ≥10 flights with confirmed Vy",
    ]
    tc.hw_expand = [
        "If R/C < 200 fpm: sector wheel extension to ±96° is the priority fix",
        "If R/C > 500 fpm: explore higher AGL ceiling testing",
    ]
    cards.append(tc)

    tc = TestCard(7, "Service Ceiling Approach",
                  "PHASE 3 — ENVELOPE EXPANSION", "MCO-1D")
    tc.objective = ("Establish the service ceiling (altitude where R/C "
                    "drops to 100 fpm) per AC 90-89C §7.2. "
                    "Per FAA: 'Service ceiling tests should be performed "
                    "gradually, climbing in increments.'")
    tc.preconditions = [
        "Vy and R/C established (TC-06 complete)",
        "At least 20 successful flights",
        "Full battery, checked voltage",
        "No wind, ISA conditions preferred",
        "Pixhawk barometer altitude calibrated against GPS pressure altitude",
    ]
    tc.procedure = [
        "Climb at Vy to 100m AGL. Record R/C at this altitude.",
        "Continue climb to 200m, 400m, 600m, 800m, 1000m.",
        "At each altitude: stabilise for 30s, record R/C over 60s.",
        "Plot R/C vs altitude. Extrapolate to find 100 fpm altitude.",
        "Service ceiling = altitude where R/C = 100 fpm.",
        "Do NOT attempt to actually reach service ceiling in first tests — "
            "extrapolate from the trend.",
        "Verify the ρ_ratio × thrust relationship matches ISA predictions.",
    ]
    tc.success = [
        "R/C decreases monotonically with altitude (confirms model)",
        "Service ceiling extrapolated from 5+ data points with R² > 0.90",
        "Measured R/C at each altitude within 25% of ρ-scaled prediction",
    ]
    tc.abort = [
        "Battery voltage drops below safe minimum",
        "R/C drops below 200 fpm (approaching ceiling — RTB)",
        "GPS altitude > 1200m AGL without prior approval",
    ]
    tc.data = [
        "R/C (fpm) at each altitude",
        "Extrapolated service ceiling (m and ft)",
        "ρ_ratio at each altitude vs R/C reduction — validate model",
    ]
    tc.after = [
        "Update mco_performance_model.py with measured ceiling",
        "Confirm ±96° amplitude would extend ceiling by predicted ~7,500 ft",
    ]
    tc.sw_expand = [
        "density_altitude() function feed from Pixhawk barometer — confirm active",
        "Ps_target table modulation by density altitude — enable and calibrate",
    ]
    tc.hw_expand = [
        "Sector wheel to ±96°: predicted +7,500 ft ceiling gain",
        "Reducing MTOW by removing non-essential payload also raises ceiling",
    ]
    cards.append(tc)

    tc = TestCard(8, "Flutter Investigation — Structural Limit",
                  "PHASE 3 — ENVELOPE EXPANSION", "MCO-1D")
    tc.objective = ("Per AC 90-89C §6.7-6.8: Flutter is the most critical "
                    "structural test. It occurs suddenly and can cause "
                    "catastrophic structural failure within seconds. "
                    "The goal is to find flutter ONSET — not to experience "
                    "full flutter. Approach with extreme caution.")
    tc.preconditions = [
        "Vne approach testing (TC-05) complete",
        "At least 25 flights completed",
        "Wings fully inspected: all ribs, spars, skin secure",
        "All wing joints torqued to spec (AN4 hardware)",
        "IMU high-frequency logging enabled (200 Hz minimum)",
        "Video camera on wings during test",
        "Start at 60% of TC-05 maximum speed. Increment 5 mph per pass.",
    ]
    tc.procedure = [
        "Confirm aircraft is at established Vno (90% of TC-05 Vne).",
        "FLUTTER MONITORING: IMU FFT output should show no peak "
            "> 2× background in 10-50 Hz range.",
        "Accelerate to Vno + 5 mph. Hold 5 seconds. Examine FFT.",
        "If clean: Vno + 10 mph. If any FFT peak emerges: STOP.",
        "Continue to established Vne in 5 mph steps.",
        "If flutter onset detected (FFT peak rise): "
            "smoothly reduce speed. Do NOT abruptly pitch up.",
        "Report: flutter speed (if found), or confirm clean to Vne.",
        "Do not exceed Vne during this test.",
    ]
    tc.success = [
        "No flutter detected below Vne",
        "IMU FFT shows no emerging peaks in 10-50 Hz range at any speed",
        "Aircraft handled normally at all tested speeds",
    ]
    tc.abort = [
        "ANY IMU FFT peak suddenly rises > 2× background: decelerate NOW",
        "Visible wing oscillation on camera",
        "Any structural sound from wings",
        "Control feel changes at any speed",
    ]
    tc.data = [
        "IMU FFT spectra at each speed (peak frequency, amplitude)",
        "Video of wings at each speed step",
        "Flutter onset speed if observed (mph)",
        "Type of flutter if observed (bending, torsion, coupled)",
    ]
    tc.after = [
        "If flutter onset observed: lower Vne to 85% of flutter speed",
        "Report flutter details to design team for structural analysis",
    ]
    tc.sw_expand = [
        "Implement real-time FFT flutter monitor in flight_recorder.py",
        "Auto-decelerate if 10-50 Hz IMU peak exceeds threshold",
        "Log FFT data every 0.5 seconds during high-speed flight",
    ]
    tc.hw_expand = [
        "If flutter below 80 mph: add mass balance to wing tips (small weights "
            "at 20-25% chord from LE at tip)",
        "If torsional flutter: increase trailing edge stiffness (spar to TE rod)",
        "Reduce chord if flutter consistently below 60 mph",
    ]
    cards.append(tc)

    tc = TestCard(9, "Load Factor / G-Force Envelope",
                  "PHASE 3 — ENVELOPE EXPANSION", "MCO-1D")
    tc.objective = ("Per EAA Stage Three: 'Testing to a modest 3G limit "
                    "should be sufficient for most non-aerobatic lightplanes.' "
                    "Establish the MCO's actual structural limit by approaching "
                    "it in 0.25G increments through banked turns.")
    tc.preconditions = [
        "TC-04 (stall) and TC-05 (Vne) complete",
        "Minimum 300m AGL",
        "IMU g-load logging active",
        "Approach load factor in banked turns at 1.3×Vs",
        "Bank angle: 30°=1.15g, 45°=1.41g, 60°=2.00g, 75°=3.86g",
    ]
    tc.procedure = [
        "Level flight at 1.3×Vs. Initiate 30° bank turn. Note g-load.",
        "Recovery. Wait 30 seconds. Inspect for any anomalies.",
        "Increase to 45° bank. Hold for 5 seconds. Note g-load and stall speed.",
        "Recovery. Structural inspection via camera.",
        "Increase to 60° bank (2.0G). This is the initial design limit.",
        "If clean and no anomalies: attempt 65° bank (2.37G).",
        "Stop at 2.5G initially. This is conservative — MCO's SF is 15× on spar.",
        "On subsequent flights, expand to 3.0G if all checks pass.",
    ]
    tc.success = [
        "Aircraft controllable at all tested bank angles",
        "No structural noise or anomalies during turns",
        "Stall speed in turns matches n = 1/cos(bank) prediction",
        "Recovery clean at all tested load factors",
    ]
    tc.abort = [
        "Any structural sound during banked turn",
        "Uncontrolled roll or pitch excursion",
        "Altitude loss > 30m during recovery",
        "IMU g-load exceeds 3.5G (structural margin)",
    ]
    tc.data = [
        "Peak g-load at each bank angle",
        "Stall speed in turns vs prediction (n = 1/cos(bank)) — PHAK validation",
        "Maximum tested g-load without anomaly",
        "Recovery altitude lost at each test point",
    ]
    tc.after = [
        "Establish maximum g-load for normal operations (2.5G initial)",
        "Update manoeuvre envelope in wave library",
        "Stall in turns data validates PHAK formulas in wave library",
    ]
    tc.sw_expand = [
        "stall_speed_check() turn stall formula already implemented — validate",
        "Adjust stall_margin_min_ms safety gate based on measured turn stall data",
        "Enable banked stall protection (reduce freq before entering high-bank)",
    ]
    tc.hw_expand = [
        "Higher g-limit requires spar review (rattan to CF tube upgrade for 1A)",
        "For MCO-1A: AN4 hardware at all joints is mandatory before 2G testing",
    ]
    cards.append(tc)

    # ─────────────────────────────────────────────────────────────
    # PHASE 4 — PERFORMANCE AND ENDURANCE (Hours 25-40)
    # AC 90-89C Ch.7: "Maximum gross weight, service ceiling,
    #                   navigation, fuel consumption"
    # ─────────────────────────────────────────────────────────────

    tc = TestCard(10, "Cascade Phase Optimisation",
                  "PHASE 4 — PERFORMANCE", "MCO-1D")
    tc.objective = ("Enable the PhaseOptimiser and CascadeOptimiser. "
                    "Systematically map the Ps response surface over "
                    "the (phase_offset, diff_sweep) parameter space.")
    tc.preconditions = [
        "≥15 successful flights. All Phase 3 cards complete.",
        "CT and Vs measured and wave library updated.",
        "PhaseOptimiser enabled (PHASE_OPT_ENABLED = True)",
        "Smooth air, consistent conditions for reliable Ps measurement",
    ]
    tc.procedure = [
        "Enable PhaseOptimiser (CH6 toggle).",
        "Fly standard cruise for 20 minutes. Allow optimiser to run.",
        "Check optimiser log: which phase offset improved Ps?",
        "On next flight: enable CascadeOptimiser with best phase from above.",
        "Fly 20 minutes. Allow 2D (phase, diff_sweep) search to run.",
        "Repeat over ≥5 flights. Allow convergence.",
        "Report: optimal phase, optimal diff_sweep, Ps improvement vs nominal.",
    ]
    tc.success = [
        "PhaseOptimiser converges within ±5° of a stable optimum",
        "CascadeOptimiser converges to a stable (phase, diff_sweep) pair",
        "Ps improvement at optimum > 5% vs nominal parameters",
        "All experiments kept safety-clean (0 aborts)",
    ]
    tc.abort = [
        "Any experiment causing motor current spike",
        "Ps decreasing monotonically (no improvement found after 20 experiments)",
    ]
    tc.data = [
        "Optimal cascade phase offset (degrees) — UPDATE PHASE_OPT_DEG",
        "Optimal leading-pair differential sweep (degrees)",
        "Ps at optimal vs nominal (% improvement)",
        "Whether metachronal or tandem wake-capture mode dominates",
    ]
    tc.after = [
        "Update CASCADE_PHASE_DEFAULT_DEG in wave library",
        "Update differential_sweep_for_wake_capture() with measured optimal",
        "Enable AdaptiveOptimiser (mco_adaptive_optimiser.py) full operation",
    ]
    tc.expand_with = [
        "Optimal phase expands effective range without any hardware change",
        "Tandem mode may open access to higher efficiency at cruise speeds",
    ]
    tc.sw_expand = [
        "Expand search range if optimum is at edge of [30°, 90°] window",
        "Enable speed-varying phase schedule (different phase at slow vs sprint)",
        "Enable eagle GL-9 soaring mode after GL-8 basic soar is confirmed",
    ]
    cards.append(tc)

    tc = TestCard(11, "Eagle Soar Mode — Glide Optimisation",
                  "PHASE 4 — PERFORMANCE", "MCO-1D")
    tc.objective = ("Validate GL-8 and GL-9 soar configurations. "
                    "Measure actual glide ratio and compare to predicted 10.5:1.")
    tc.preconditions = [
        "≥20 successful flights",
        "Smooth air preferred (thermals will confound measurements)",
        "Minimum start altitude 300m AGL",
    ]
    tc.procedure = [
        "Climb to 300m AGL. Engine off (frequency = 0 Hz).",
        "Enter GL-0 (standard glide). Record altitude lost per 100m ground.",
        "L/D = ground distance / altitude lost.",
        "Repeat for GL-1, GL-8, GL-9.",
        "For GL-8 (eagle splay): W1+W4 at 20° dihedral, W2+W3 at 5°.",
        "For GL-9 (scimitar): as GL-8 but W1=60°, W4=120°.",
        "Compare measured L/D values across configurations.",
        "Identify which config gives best L/D.",
    ]
    tc.success = [
        "GL-0 L/D within ±20% of predicted 10.5:1",
        "GL-8 L/D > GL-1 L/D (differential dihedral benefit confirmed)",
        "GL-9 L/D ≥ GL-8 L/D (scimitar adds to GL-8)",
        "Consistent L/D across 3 repetitions (±5%)",
    ]
    tc.data = [
        "L/D ratio for GL-0, GL-1, GL-8, GL-9",
        "Sink rate (fpm) at best glide speed for each configuration",
        "Best glide speed for each configuration",
        "EAGLE_OUTER_DIHEDRAL_DEG optimal value (if different from 20°)",
    ]
    tc.after = [
        "Update gl_polar[] in wave library with measured L/D values",
        "Update engine-out footprint calculations for AOH",
        "If GL-9 > GL-8: confirm scimitar sweep benefit is real",
    ]
    tc.sw_expand = [
        "Enable AdaptiveOptimiser for GL-8/GL-9 eagle_outer_dihedral optimisation",
        "Enable speed-varying optimal glide speed (V_LD varies with ρ)",
    ]
    tc.hw_expand = [
        "If GL-8 shows > 20% CDi reduction: all future aircraft should use "
            "differential dihedral rail geometry",
        "If GL-9 L/D consistently > GL-8: scimitar sweep angle optimisation next",
    ]
    cards.append(tc)

    # ─────────────────────────────────────────────────────────────
    # PHASE 5 — ADVANCED / MCO-1A TRANSITION
    # ─────────────────────────────────────────────────────────────

    tc = TestCard(12, "MCO-1A Transition — First Piloted Flight Preconditions",
                  "PHASE 5 — MCO-1A TRANSITION", "MCO-1A")
    tc.objective = ("Before any piloted flight of the MCO-1A, the following "
                    "conditions must be met, derived from AC 90-89C and "
                    "EAA Flight Test Manual v2 task-based requirements.")
    tc.preconditions = [
        "MCO-1D has completed all Phase 3 and 4 cards",
        "MCO-1D has ≥40 total flight hours",
        "All MCO-1D envelope limits transferred to MCO-1A wave library",
        "MCO-1A structural inspection by EAA Technical Counsellor",
        "MCO-1A CG calculation verified (all 3 axes per AC 90-89C §2.14)",
        "MCO-1A preflight_wb_check() integrated and tested",
        "Emergency procedures documented and trained",
        "Test pilot qualifications: minimum 500 hours, tailwheel/unusual aircraft",
        "Test pilot has reviewed all MCO-1D flight data and test reports",
        "Chase aircraft available with radio communication",
        "Emergency vehicle on standby at test site",
        "MCO-1A experimental certificate issued",
        "Weight and balance confirmed for first flight configuration",
        "All AN hardware torqued and safety-wired at structural joints",
        "Lord mount vibration isolators inspected",
        "KTM 790 engine 10-hour run-in complete",
    ]
    tc.procedure = [
        "MCO-1A first flight programme mirrors MCO-1D Phase 2 (TC-03)",
        "First flight: conservative parameters only. No optimiser. No stow.",
        "All limits set 20% inside MCO-1D confirmed limits initially.",
        "Expand systematically following MCO-1D protocol precedent.",
    ]
    tc.abort = [
        "Any precondition not met — no exceptions",
    ]
    tc.data = [
        "All MCO-1D final performance numbers for comparison baseline",
        "MCO-1A-specific CT calibration (will differ from MCO-1D)",
        "MCO-1A stall speed (predicted 30 mph — measure carefully)",
    ]
    cards.append(tc)

    return cards


def generate_protocol_document(cards):
    lines = []
    lines.append("MCO FLIGHT TEST PROTOCOL")
    lines.append("Metachronal Cascade Ornithopter — Rev A  2026-06-16")
    lines.append("Based on: FAA AC 90-89C (2023)  |  EAA Flight Test Manual v2 (2024)")
    lines.append("="*60)
    lines.append("")
    lines.append("PROGRAMME OVERVIEW")
    lines.append("-"*60)
    lines.append("""
The FAA requires all experimental aircraft to demonstrate:
  "The aircraft is controllable throughout its normal range of
   speeds and throughout all the maneuvers to be executed, and
   that the aircraft has no hazardous operating characteristics
   or design features."  (14 CFR §91.319(b))

For the MCO, this means demonstrating controllability across:
  • Full speed range (Vs to Vne)
  • Full altitude range (ground to service ceiling)
  • All flapping configurations (phase, amplitude, sweep, dihedral)
  • All loading conditions (minimum to MTOW)
  • All flight regimes (slow, cruise, sprint, climb, glide)

The MCO-1D drone is the PRIMARY test vehicle. Every limit is
first found on the drone before the MCO-1A piloted aircraft
is tested. This is not just good practice — it is the only
responsible approach for a novel flight configuration.

PHASE STRUCTURE (mirrors AC 90-89C chapter structure):
  Phase 1: Ground and taxi verification (TC-01 to TC-02)
  Phase 2: First flight — basic controllability (TC-03)
  Phase 3: Envelope expansion — all critical limits (TC-04 to TC-09)
  Phase 4: Performance and optimisation (TC-10 to TC-11)
  Phase 5: MCO-1A transition (TC-12 and beyond)

MCO-1D minimum Phase I: 40 hours (non-TC engine, per AC 90-89C)
MCO-1A minimum Phase I: 40 hours (non-TC engine, per AC 90-89C)
""")
    lines.append("="*60)
    lines.append("")

    for card in cards:
        lines.append(card.render())
        lines.append("")
        lines.append("")

    lines.append("="*60)
    lines.append("LIMIT EXPANSION STRATEGY SUMMARY")
    lines.append("="*60)
    lines.append("""
Every limit has three expansion pathways. After each test card
confirms a limit, the following options are available:

1. SOFTWARE EXPANSION (fastest, lowest risk):
   • Tune constants in wave library (VS_KT, FREQ_LIMIT_MAX_HZ, etc.)
   • Enable optimiser tiers (Phase→Cascade→Adaptive)
   • Refine Boyd Ps targets based on measured performance
   • Implement flutter monitor FFT in flight_recorder

2. FIRMWARE / ALGORITHM EXPANSION (medium risk):
   • Adjust safety gate thresholds based on measured margins
   • Enable additional flight modes (GL-8, GL-9, stoop, zoom)
   • Implement adaptive cascade phase scheduling
   • Enable density-altitude-corrected thrust targets

3. HARDWARE EXPANSION (highest impact, requires rebuild):
   • Sector wheel to 192° arc (±96°) — +35% thrust
   • Fuselage narrowed to 150mm — enables clap-and-fling
   • Wing mass balance — raises flutter speed
   • Covert flaps on W1+W2 (groove VGs) — improves CL
   • Stow slot receivers — raises Vne by reducing stowed-wing drag
   • KTM 790 bolt-ons — +34% continuous power for MCO-R

THE GOLDEN RULE OF ENVELOPE EXPANSION:
  "Never more than one change at a time. Test. Confirm.
   Then make the next change."
  — AC 90-89C §6.1, adapted for ornithopter configuration
""")
    return '\n'.join(lines)


if __name__ == '__main__':
    cards = build_protocol()
    doc   = generate_protocol_document(cards)
    path  = '/mnt/user-data/outputs/MCO_Flight_Test_Protocol.txt'
    with open(path, 'w') as f:
        f.write(doc)
    print(f"Written: {path}")
    print(f"Cards:   {len(cards)}")
    print(f"Lines:   {len(doc.splitlines())}")
    print()
    # Print first card as sample
    print(cards[0].render()[:1200])
