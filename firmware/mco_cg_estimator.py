# NOTE (Rev 18): CGFlightEstimator and CGCompensator in this file
# are conceptually superseded by cg_management_tick() in mco_wave_library.py.
# The wave library implementation runs every tick via the tick wrapper
# and includes airborne detection, stability margin scheduling, and
# lateral fast trim — all of which are absent here.
# This file is retained for reference and for future CGStaticModel
# preflight weight-and-balance calculations.
# Do not call CGCompensator.apply() directly — use cg_management_tick().
#
"""
mco_cg_estimator.py
====================
MCO-1D Centre-of-Gravity Estimation and Compensation System

Based on FAA-H-8083-1B (Aircraft Weight and Balance Handbook, 2016)
applied to a novel multi-wing ornithopter without traditional control surfaces.

THREE-LAYER ARCHITECTURE:

  Layer 1 — Static W&B (preflight, ground)
    CGStaticModel: component arm table + weight inputs → CG position.
    All weights are PLACEHOLDER until first flight. The structure is
    complete so the pilot can enter measured values immediately.

  Layer 2 — In-Flight CG Estimator (runs every 5 seconds)
    CGFlightEstimator: infers CG offset from sensor behaviour.
    Three methods: pitch trim deviation, pitch rate response,
    lateral roll trim asymmetry. Kalman-style weighted fusion.

  Layer 3 — CG Compensation Controller (runs every 50 Hz loop)
    CGCompensator: modifies wave library pitch/sweep targets to
    keep the aircraft controllable despite CG uncertainty.
    Authority: ±3° AoA differential W1/W4 ≈ ±10mm CG compensation.
    Works transparently — pilot feels nothing.

CALIBRATION CONSTANTS:
  K_pitch, K_lateral, K_rate are initialised from physical models.
  The in-flight optimiser (mco_optimiser.py) updates them over
  the first few flights once real data is available.
  All placeholder values are marked with # CAL: to make them
  easy to find and update at first flight.

FAA-H-8083-1B CONNECTIONS:
  Datum:          Forward tip of fuselage (nose). All arms positive aft.
  LEMAC:          Leading edge of mean aerodynamic chord.
  CG_% MAC:       (CG_mm - LEMAC_mm) / MAC_mm × 100
  CG envelope:    Forward limit = 15% MAC (longitudinal stability)
                  Aft limit     = 35% MAC (pitch control authority)
  These limits will be confirmed by flight test and updated.
"""

import math


# =================================================================
# PHYSICAL CONSTANTS AND PLACEHOLDER COMPONENT TABLE
# =================================================================

# Datum: nose tip of fuselage. All arms measured aft = positive.
# REPLACE all values marked # CAL: with measured data at first flight.
DATUM_DESCRIPTION = "Nose tip of fuselage"

# Mean Aerodynamic Chord position
LEMAC_MM    = 475.0   # mm aft of datum (mean leading edge of 4 tandem stations: [100, 350, 600, 850]mm)
MAC_MM      = 143.7   # mm (= CHORD_MM, confirmed)

# CG envelope limits (% MAC — referenced to LEMAC = 475mm, Neutral Point = 510.9mm)
# Static Margin +5% to +20% corresponds to -10.0% to +15.0% MAC relative to LEMAC
CG_FWD_LIMIT_PCT_MAC  = -10.0 # % MAC forward limit (460.6 mm aft of datum)
CG_AFT_LIMIT_PCT_MAC  = 25.0  # % MAC aft limit (510.9 mm aft of datum, neutral point)

# Compensation authority limits
CG_COMP_AUTHORITY_MM  = 30.0  # mm — max CG offset we can compensate
CG_WARN_THRESHOLD_MM  = 15.0  # mm — issue warning above this offset
CG_CRITICAL_MM        = 25.0  # mm — reduce performance above this

# =================================================================
# COMPONENT ARM TABLE
# (all distances in mm AFT of datum = positive)
# Replace zeros / estimates with measured values at build time.
# =================================================================
COMPONENT_ARMS = [
    # Airframe structure & central spine (1000mm tube datum 0 at nose)
    ("Aluminium spine & mounts",  500,   450,  "20mm tube (1000mm) + 8 station brackets"),
    ("W1 wing pair (1150mm)",     100,   320,  "Universal ribs, 4x 250mm segments/side, ⌀10mm CF spars, wingtip"),
    ("W2 wing pair (1150mm)",     350,   320,  "Universal ribs, 4x 250mm segments/side, ⌀10mm CF spars, wingtip"),
    ("W3 wing pair (1150mm)",     600,   320,  "Universal ribs, 4x 250mm segments/side, ⌀10mm CF spars, wingtip"),
    ("W4 wing pair (1150mm)",     850,   320,  "Universal ribs, 4x 250mm segments/side, ⌀10mm CF spars, wingtip"),
    ("Wing pivot & sector units", 475,   240,  "8 pivots, bearings, 8 sector wheels"),

    # Flap drive system (8 motors + rope drives + pulleys)
    ("GoolRC 5010-14 motors x8",  475,   760,  "8x 95g baseline brushless motors (360KV), 8x pinion pulleys"),
    ("ODrive S1 controllers x8",  475,   640,  "8x 80g along motor rail"),
    ("50W Brake resistors x8",    475,   200,  "Mounted on heat sink rail"),
    ("Dyneema capstan system",    475,    50,  "Ropes and capstans"),

    # Servos (16x AGFRC IB53BHU - 69.5g each, BOM v18)
    ("AGFRC IB53BHU sweep x8",    475,   556,  "8x 69.5g sweep servos"),
    ("AGFRC IB53BHU pitch x8",    475,   556,  "8x 69.5g pitch servos"),

    # Computers and avionics
    ("Raspberry Pi 4B (4GB)",     180,    55,  "Flight computer"),
    ("Pixhawk 4 Mini + GPS",      160,    80,  "Autopilot and mast GPS"),
    ("MCP3208 ADC boards x2",     250,    20,  "Pitch and sweep 12-bit SPI feedback"),
    ("Sensors (IMU, Baro, Pitot)",120,    25,  "Nose bay sensor suite"),
    ("Wiring harness (silicone)", 475,   120,  "High-current silicone bus wiring"),

    # Undercarriage
    ("Landing gear & skids",      475,   150,  "Carbon landing bow / skids"),

    # Power system (Adjustable slide rail with indexing quick-pins: 120–550mm)
    ("6S LiPo 5000mAh (Main)",    475,   540,  "Main propulsion — quick-pin sliding tray (120–550mm rail, nominal at 475mm)"),
    ("2S LiPo 3000mAh (Servos)",  475,   175,  "Isolated 45C servo power bus — quick-pin sliding tray (nominal at 475mm)"),

    # Reserve payload
    ("Payload bay (empty)",       475,     0,  "Reserve SAR payload allowance"),
]


class CGStaticModel:
    """
    Pre-flight weight and balance calculator.

    Usage:
        model = CGStaticModel()
        # Override any placeholder weight:
        model.set_component_weight("6S LiPo 5000mAh", 542)  # measured
        # Get results:
        result = model.compute()
        print(result.summary())

    All weights in grams. All arms and CG in mm from datum.
    FAA-H-8083-1B method: CG = total_moment / total_weight
    """

    def __init__(self):
        # Deep copy so we can modify without affecting the module table
        self._components = [
            {'name': c[0], 'arm_mm': c[1],
             'weight_g': c[2], 'notes': c[3]}
            for c in COMPONENT_ARMS
        ]
        self._extra = []   # for pilot-added items

    def set_component_weight(self, name, weight_g):
        """Update a component weight from measured value."""
        for c in self._components:
            if c['name'] == name:
                c['weight_g'] = weight_g
                return True
        return False

    def set_component_arm(self, name, arm_mm):
        """Update a component arm from measured value."""
        for c in self._components:
            if c['name'] == name:
                c['arm_mm'] = arm_mm
                return True
        return False

    def add_item(self, name, arm_mm, weight_g):
        """Add a payload or non-standard item."""
        self._extra.append(
            {'name': name, 'arm_mm': arm_mm, 'weight_g': weight_g})

    def compute(self):
        """
        Compute CG and return CGResult.
        FAA-H-8083-1B formula:
          CG (mm from datum) = sum(weight × arm) / sum(weight)
        """
        all_items = self._components + self._extra
        total_weight_g  = sum(c['weight_g'] for c in all_items)
        total_moment_gm = sum(c['weight_g'] * c['arm_mm']
                              for c in all_items)

        if total_weight_g <= 0:
            return CGResult(0, 0, 0, False, "No weight entered")

        cg_mm       = total_moment_gm / total_weight_g
        cg_pct_mac  = (cg_mm - LEMAC_MM) / MAC_MM * 100.0
        in_envelope = (CG_FWD_LIMIT_PCT_MAC
                       <= cg_pct_mac
                       <= CG_AFT_LIMIT_PCT_MAC)

        return CGResult(
            cg_mm        = round(cg_mm, 1),
            cg_pct_mac   = round(cg_pct_mac, 1),
            mtow_g       = total_weight_g,
            in_envelope  = in_envelope,
            components   = all_items,
        )


class CGResult:
    """Result of a static CG calculation."""
    def __init__(self, cg_mm, cg_pct_mac, mtow_g,
                 in_envelope, components=None, note=''):
        self.cg_mm       = cg_mm
        self.cg_pct_mac  = cg_pct_mac
        self.mtow_g      = mtow_g
        self.mtow_kg     = mtow_g / 1000.0
        self.in_envelope = in_envelope
        self.components  = components or []
        self.note        = note

    def summary(self):
        lines = [
            "MCO-1D WEIGHT AND BALANCE SUMMARY",
            f"  Datum:          {DATUM_DESCRIPTION}",
            f"  MTOW:           {self.mtow_kg:.3f} kg  "
            f"({self.mtow_g:.0f} g)",
            f"  CG position:    {self.cg_mm:.1f} mm aft of datum",
            f"  LEMAC:          {LEMAC_MM:.0f} mm aft of datum",
            f"  CG % MAC:       {self.cg_pct_mac:.1f}%",
            f"  CG envelope:    {CG_FWD_LIMIT_PCT_MAC}% – "
            f"{CG_AFT_LIMIT_PCT_MAC}% MAC",
            f"  IN ENVELOPE:    {'YES ✓' if self.in_envelope else 'NO ✗ — DO NOT FLY'}",
        ]
        if not self.in_envelope:
            if self.cg_pct_mac < CG_FWD_LIMIT_PCT_MAC:
                lines.append("  ACTION: Move battery AFT or add tail ballast")
            else:
                lines.append("  ACTION: Move battery FORWARD or add nose ballast")
        if self.note:
            lines.append(f"  NOTE: {self.note}")
        lines.append("")
        lines.append("  Top 5 heaviest components:")
        for c in sorted(self.components, key=lambda x: x['weight_g'],
                        reverse=True)[:5]:
            lines.append(f"    {c['name']:<35} {c['weight_g']:>5}g"
                         f"  arm={c['arm_mm']:>4}mm")
        return '\n'.join(lines)


# =================================================================
# LAYER 2: IN-FLIGHT CG ESTIMATOR
# =================================================================

class CGFlightEstimator:
    """
    Infers longitudinal and lateral CG offset from flight behaviour.

    Called every 5 seconds when flight conditions are steady enough
    for a valid observation (level flight, low bank, low vspeed).

    The estimate converges over many flights via the Kalman-style
    update: new_estimate = old × (1-alpha) + observation × alpha
    where alpha = confidence × 0.1 (slow, safe convergence).

    All calibration constants (K_*) are PLACEHOLDERS.
    Replace with measured values after first-flight calibration flight.
    """

    # Steady-flight detection thresholds for valid observation
    STEADY_BANK_MAX_DEG    = 3.0    # deg — wings level
    STEADY_VSPEED_MAX_MS   = 0.15   # m/s — near-level
    STEADY_PITCHRATE_MAX   = 1.0    # deg/s — stable pitch
    STEADY_DURATION_S      = 5.0    # seconds of steady flight needed

    # Calibration gains  # CAL: update after first flight
    K_PITCH_MM_PER_DEG     = 8.0    # mm CG offset per degree AoA trim error
    K_LATERAL_MM_PER_DEG   = 5.0    # mm lateral CG per degree sweep asymmetry
    K_RATE_SCALE           = 1.0    # pitch rate ratio → CG ratio  # CAL:

    # Kalman-style update rate
    ALPHA_PER_OBSERVATION  = 0.08   # how much each observation moves estimate
    ALPHA_MAX_CONFIDENCE   = 0.15   # max update at confidence=1.0

    def __init__(self):
        # Current CG estimates (mm offset from nominal, + = aft)
        self.delta_cg_long_mm   = 0.0   # longitudinal
        self.delta_cg_lat_mm    = 0.0   # lateral (+ = starboard)
        self.confidence         = 0.0   # 0–1 confidence in estimate
        self._steady_t          = 0.0   # seconds of current steady phase
        self._obs_history       = []    # rolling buffer of recent observations
        self._n_observations    = 0

    def update(self, t, V_ms, bank_deg, pitch_deg, vspeed_ms,
               pitch_rate_dps, roll_rate_dps,
               pitch_cmd_deg,        # what pitch servo is commanding
               pitch_model_deg,      # what model predicts for this V/freq
               sweep_port_deg,       # mean port sweep
               sweep_stbd_deg,       # mean stbd sweep
               freq_hz):
        """
        Called every 5 seconds. Returns updated delta_cg estimates.

        pitch_cmd_deg:   mean pitch AoA commanded by ControlState
        pitch_model_deg: predicted AoA for nominal CG at this V/freq
        """
        # ── Steadiness check ─────────────────────────────────────
        steady = (abs(bank_deg)      < self.STEADY_BANK_MAX_DEG and
                  abs(vspeed_ms)     < self.STEADY_VSPEED_MAX_MS and
                  abs(pitch_rate_dps)< self.STEADY_PITCHRATE_MAX)

        if steady:
            self._steady_t += 5.0
        else:
            self._steady_t = 0.0
            return self._result()

        if self._steady_t < self.STEADY_DURATION_S:
            return self._result()

        # ── Method 1: Pitch trim deviation ───────────────────────
        # FAA-H-8083-1B: forward CG needs more nose-up force.
        # If we're commanding more AoA than model predicts: CG is forward.
        delta_aoa    = pitch_cmd_deg - pitch_model_deg
        obs_long_1   = delta_aoa * self.K_PITCH_MM_PER_DEG
        conf_1       = min(1.0, abs(delta_aoa) / 3.0)   # 3° = full confidence

        # ── Method 2: Sweep asymmetry (lateral CG) ────────────────
        delta_sweep  = sweep_port_deg - sweep_stbd_deg
        obs_lat      = delta_sweep * self.K_LATERAL_MM_PER_DEG
        conf_lat     = min(1.0, abs(delta_sweep) / 5.0)  # 5° = full confidence

        # ── Fuse observations ─────────────────────────────────────
        confidence   = conf_1
        alpha        = self.ALPHA_PER_OBSERVATION * confidence

        self.delta_cg_long_mm = (self.delta_cg_long_mm * (1.0 - alpha)
                                  + obs_long_1 * alpha)
        self.delta_cg_lat_mm  = (self.delta_cg_lat_mm * (1.0 - alpha)
                                  + obs_lat * alpha)

        self.confidence = min(1.0, self.confidence
                              + confidence * self.ALPHA_PER_OBSERVATION)
        self._n_observations += 1

        return self._result()

    def _result(self):
        return {
            'delta_cg_long_mm': round(self.delta_cg_long_mm, 1),
            'delta_cg_lat_mm':  round(self.delta_cg_lat_mm,  1),
            'confidence':       round(self.confidence, 3),
            'n_obs':            self._n_observations,
            'status': ('calibrating' if self._n_observations < 10
                       else 'tracking' if self.confidence > 0.5
                       else 'low_confidence'),
        }

    def reset(self):
        """Call at start of each new flight."""
        self.delta_cg_long_mm = 0.0
        self.delta_cg_lat_mm  = 0.0
        self.confidence       = 0.0
        self._steady_t        = 0.0
        self._n_observations  = 0


# =================================================================
# LAYER 3: CG COMPENSATION CONTROLLER
# =================================================================

class CGCompensator:
    """
    Adjusts wave library pitch/sweep/dihedral targets to compensate
    for estimated CG offset. Runs every 50 Hz loop.

    The compensation is transparent to the pilot — the aircraft simply
    uses slightly adjusted wing geometry while appearing to fly normally.

    COMPENSATION MECHANISMS (from FAA-H-8083-1B principles):

    Longitudinal (forward CG — nose heavy):
      → Increase W4 pitch AoA slightly (tail lifts → nose up moment)
      → Decrease W1 pitch AoA slightly (less forward lift)
      → Net effect: moves effective CL aft → compensates forward CG
      → Exactly analogous to increasing tailplane incidence in GA aircraft

    Longitudinal (aft CG — tail heavy):
      → Increase W1 pitch AoA → more forward lift → nose up
      → Decrease W4 pitch AoA
      → Net: moves effective CL forward

    Lateral (port-heavy):
      → Reduce port sweep slightly → more port lift
      → OR increase stbd sweep → less stbd lift
      → Same mechanism as Rev K auto-coordination but driven by CG,
        not yaw rate

    Authority limits (from physics analysis):
      3° AoA differential W1/W4 ≈ 10mm longitudinal compensation
      Combined sweep + AoA: up to ~30mm
      Beyond 25mm CG offset: issue warning; reduce aerobatic authority
      Beyond 30mm: restrict to cruise and approach only
    """

    # Maximum compensation per wing (deg)
    MAX_PITCH_COMP_DEG   = 4.0   # max AoA correction per wing
    MAX_SWEEP_COMP_DEG   = 8.0   # max sweep correction for lateral

    # Scale factors: mm of CG offset → degrees of correction
    # From physics: 3° AoA diff ≈ 10mm → K = 3/10 = 0.3 deg/mm
    K_LONG_DEG_PER_MM    = 0.30  # longitudinal  # CAL: refine at first flight
    K_LAT_DEG_PER_MM     = 0.20  # lateral

    def __init__(self):
        self._last_delta_long = 0.0
        self._last_delta_lat  = 0.0

    def apply(self, delta_cg_long_mm, delta_cg_lat_mm,
              confidence, base_pitches, base_sweeps,
              manoeuvre_name='cruise'):
        """
        Apply CG compensation to base pitch and sweep targets.

        delta_cg_long_mm: + = CG aft of nominal, - = forward
        delta_cg_lat_mm:  + = CG to starboard,   - = port
        confidence:       0–1 from estimator
        base_pitches:     list of 8 base pitch targets (deg)
        base_sweeps:      list of 8 base sweep targets (deg)
        manoeuvre_name:   used to scale authority (less in aerobatics)

        Returns (pitches_corrected, sweeps_corrected,
                 warning_level, note)
        """
        # Scale compensation by confidence (don't fully trust low-conf estimate)
        eff_long = delta_cg_long_mm * min(1.0, confidence * 2.0)
        eff_lat  = delta_cg_lat_mm  * min(1.0, confidence * 2.0)

        # Warning levels
        abs_long = abs(eff_long)
        if abs_long > CG_CRITICAL_MM:
            warning = 'CRITICAL'
        elif abs_long > CG_WARN_THRESHOLD_MM:
            warning = 'CAUTION'
        else:
            warning = 'NORMAL'

        # Reduce compensation authority during aerobatics
        # (don't want CG corrections fighting pilot inputs)
        if manoeuvre_name in ('dart', 'stoop', 'chandelle', 'tight_turn'):
            authority_scale = 0.3   # only 30% correction in aerobatics
        elif manoeuvre_name in ('zoom_climb', 'power_dive'):
            authority_scale = 0.5
        else:
            authority_scale = 1.0

        # ── Longitudinal compensation ─────────────────────────────
        # Positive delta (aft CG): increase W1 pitch, decrease W4
        # Negative delta (fwd CG): decrease W1 pitch, increase W4
        long_corr = eff_long * self.K_LONG_DEG_PER_MM * authority_scale
        long_corr = max(-self.MAX_PITCH_COMP_DEG,
                        min(self.MAX_PITCH_COMP_DEG, long_corr))

        pitches = list(base_pitches)
        for i in range(8):
            pair = i // 2 + 1   # 1=W1, 2=W2, 3=W3, 4=W4
            if pair == 1:
                # W1 (forward): aft CG → increase AoA (fwd lift)
                pitches[i] = base_pitches[i] + long_corr
            elif pair == 4:
                # W4 (aft): aft CG → decrease AoA (less tail lift)
                pitches[i] = base_pitches[i] - long_corr * 1.5  # W4 more effective
            else:
                # W2, W3: proportional, smaller correction
                pitches[i] = base_pitches[i] + long_corr * 0.3
            # Clamp to safe AoA range
            pitches[i] = max(-5.0, min(12.0, pitches[i]))

        # ── Lateral compensation ──────────────────────────────────
        # Positive (stbd heavy): reduce stbd sweep → more stbd lift
        lat_corr = eff_lat * self.K_LAT_DEG_PER_MM * authority_scale
        lat_corr = max(-self.MAX_SWEEP_COMP_DEG,
                       min(self.MAX_SWEEP_COMP_DEG, lat_corr))

        sweeps = list(base_sweeps)
        for i in range(8):
            is_stbd = (i % 2 == 1)  # odd = stbd
            if is_stbd:
                # Stbd heavy (lat_corr > 0): reduce stbd sweep
                sweeps[i] = max(60.0, base_sweeps[i] - lat_corr)
            else:
                # Port: increase sweep to reduce port lift
                sweeps[i] = min(135.0, base_sweeps[i] + lat_corr)

        note = (f"CG comp: long={eff_long:+.1f}mm "
                f"lat={eff_lat:+.1f}mm "
                f"conf={confidence:.2f} {warning}")

        return pitches, sweeps, warning, note

    def manoeuvre_cg_safety_check(self, delta_cg_long_mm,
                                   confidence, manoeuvre_name):
        """
        Before executing a demanding manoeuvre, check whether the
        estimated CG offset is safe for that manoeuvre.
        Returns (safe: bool, reason: str).

        FAA-H-8083-1B: adverse-load checks are required before
        manoeuvres at extreme CG positions.
        """
        if confidence < 0.3:
            return True, "low confidence — nominal CG assumed"

        abs_long = abs(delta_cg_long_mm)

        # Aft CG is most dangerous for pitch-sensitive manoeuvres
        if delta_cg_long_mm > 20.0:
            if manoeuvre_name in ('stoop', 'zoom_climb', 'chandelle'):
                return False, (f"CG aft {delta_cg_long_mm:.0f}mm "
                               f"— pitch control margin insufficient "
                               f"for {manoeuvre_name}")

        # Forward CG reduces pitch authority in zoom climb
        if delta_cg_long_mm < -20.0:
            if manoeuvre_name == 'zoom_climb':
                return False, (f"CG fwd {abs(delta_cg_long_mm):.0f}mm "
                               f"— nose-up authority limited")

        # Beyond critical: restrict to cruise only
        if abs_long > CG_CRITICAL_MM:
            if manoeuvre_name not in ('cruise', 'glide', 'approach_normal'):
                return False, (f"CG offset {delta_cg_long_mm:+.0f}mm "
                               f"exceeds critical limit — cruise/glide only")

        return True, "ok"


# =================================================================
# FLIGHT LOG INTEGRATION
# =================================================================

class CGFlightLog:
    """
    Records CG estimates across a flight for post-flight analysis.
    Written as part of the flight summary.
    """
    def __init__(self):
        self._records = []

    def record(self, t, delta_cg_long, delta_cg_lat, confidence, warning):
        self._records.append((round(t,1), round(delta_cg_long,1),
                               round(delta_cg_lat,1),
                               round(confidence,2), warning))

    def summary_lines(self):
        if not self._records:
            return ["  No CG estimates recorded."]
        lines = ["  CG ESTIMATION SUMMARY:"]
        # Mean and max
        longs = [r[1] for r in self._records if r[3] > 0.3]
        lats  = [r[2] for r in self._records if r[3] > 0.3]
        if longs:
            mean_l = sum(longs)/len(longs)
            max_l  = max(longs, key=abs)
            lines.append(f"    Longitudinal CG offset: "
                         f"mean={mean_l:+.1f}mm  max={max_l:+.1f}mm")
        if lats:
            mean_lat = sum(lats)/len(lats)
            lines.append(f"    Lateral CG offset: mean={mean_lat:+.1f}mm")

        warnings = [r[4] for r in self._records if r[4] != 'NORMAL']
        if warnings:
            cautions  = warnings.count('CAUTION')
            criticals = warnings.count('CRITICAL')
            lines.append(f"    Warnings: {cautions} CAUTION, "
                         f"{criticals} CRITICAL")
            if criticals > 0:
                lines.append("    *** ACTION REQUIRED: Check battery position ***")
        else:
            lines.append("    CG within normal range throughout flight.")

        lines.append(f"    Calibration: K_pitch={CGFlightEstimator.K_PITCH_MM_PER_DEG}"
                     f"  K_lat={CGFlightEstimator.K_LATERAL_MM_PER_DEG}"
                     f"  (# CAL: update after first flight)")
        return lines


# =================================================================
# CONVENIENCE FUNCTION — called from mco_main.py
# =================================================================

def preflight_wb_check(component_overrides=None):
    """
    Run the static W&B check. Call before every flight.
    component_overrides: dict of {name: weight_g} for measured values.

    Returns CGResult. Print result.summary() for the pilot.
    """
    model = CGStaticModel()
    if component_overrides:
        for name, weight_g in component_overrides.items():
            model.set_component_weight(name, weight_g)
    return model.compute()



# =================================================================
# DYNAMIC IN-FLIGHT PAYLOAD & CG ESTIMATOR (CARGO DRONE SUITE)
# =================================================================

class DynamicPayloadEstimator:
    """
    Real-time recursive estimator for cargo payload mass and CG shift.
    Uses aerodynamic lift-to-power balance and longitudinal pitch trim torque
    to infer cargo mass (kg) and CG displacement (mm) within <250ms of loading/drop.
    """
    EMPTY_MASS_KG       = 5.687   # As-built operational empty MTOW (kg)
    DESIGN_LOAD_FACTOR  = 3.50    # Clean airframe max positive g-load limit
    INNOVATION_THRESHOLD= 0.35    # Threshold (kg) to detect instantaneous drop/pickup

    def __init__(self, empty_mass_kg=5.687):
        self.empty_mass_kg    = empty_mass_kg
        self._payload_kg      = 0.0
        self._delta_cg_mm     = 0.0
        self._conf            = 0.0
        self._p_cov           = 1.0     # Recursive estimation covariance
        self._last_drop_time  = None
        self._sample_count    = 0

    def update(self, t, V_ms, lift_cmd_n, az_g=1.0, pitch_trim_deg=0.0,
               is_airborne=True):
        """
        Update payload mass and CG estimate.

        t:               Timestamp (seconds)
        V_ms:            Airspeed (m/s)
        lift_cmd_n:      Instantaneous aerodynamic lift computed from wing kinematics (N)
        az_g:            Vertical acceleration in g's from IMU (1.0 = 1g steady flight)
        pitch_trim_deg:  Symmetric W1-W4 differential pitch trim offset (deg)
        is_airborne:     True if airborne

        Returns dictionary of payload estimation and structural limits.
        """
        if not is_airborne or V_ms < 3.0:
            return {
                'payload_kg':       round(self._payload_kg, 3),
                'gross_mass_kg':    round(self.empty_mass_kg + self._payload_kg, 3),
                'delta_cg_mm':      round(self._delta_cg_mm, 1),
                'n_limit_g':        round(self.DESIGN_LOAD_FACTOR, 2),
                'confidence':       round(self._conf, 2),
                'is_cargo_detected':self._payload_kg >= 0.15,
                'drop_detected':    False,
            }

        effective_g = max(0.4, min(2.5, az_g))
        measured_gross = max(self.empty_mass_kg * 0.8, lift_cmd_n / (effective_g * 9.80665))
        instant_payload = max(0.0, measured_gross - self.empty_mass_kg)

        # Innovation check for sudden payload release (drop detection)
        drop_detected = False
        if self._payload_kg > 0.30 and (self._payload_kg - instant_payload) > self.INNOVATION_THRESHOLD:
            drop_detected = True
            self._last_drop_time = t
            self._payload_kg = instant_payload
            self._delta_cg_mm = 0.0
            self._p_cov = 0.5
        else:
            # Recursive Least Squares / Kalman filter single-state update
            kalman_gain = self._p_cov / (self._p_cov + 0.15)
            self._payload_kg += kalman_gain * (instant_payload - self._payload_kg)
            self._p_cov = (1.0 - kalman_gain) * self._p_cov + 0.005

        # Infer longitudinal CG shift from pitch trim delta (approx 8.0 mm / deg trim)
        instant_cg = pitch_trim_deg * 8.0
        self._delta_cg_mm = 0.85 * self._delta_cg_mm + 0.15 * instant_cg

        self._sample_count += 1
        self._conf = min(1.0, self._sample_count * 0.05)

        gross = self.empty_mass_kg + self._payload_kg
        # G-limiter derating per FAA Part 23 airworthiness standards
        n_lim = self.DESIGN_LOAD_FACTOR * (self.empty_mass_kg / gross)

        return {
            'payload_kg':       round(self._payload_kg, 3),
            'gross_mass_kg':    round(gross, 3),
            'delta_cg_mm':      round(self._delta_cg_mm, 1),
            'n_limit_g':        round(max(1.8, min(self.DESIGN_LOAD_FACTOR, n_lim)), 2),
            'confidence':       round(self._conf, 2),
            'is_cargo_detected':self._payload_kg >= 0.15,
            'drop_detected':    drop_detected,
        }


# =================================================================
# SELF-TEST
# =================================================================
if __name__ == '__main__':
    print("=== MCO-1D CG ESTIMATOR SELF-TEST ===\n")

    # Static W&B with placeholder weights
    result = preflight_wb_check()
    print(result.summary())

    print()
    print("In-flight estimator test:")
    est = CGFlightEstimator()
    for i in range(15):
        t = i * 5.0
        # Simulate: pitch servo commanding 2° more than model
        # → should detect ~16mm forward CG offset
        r = est.update(t=t, V_ms=9.0, bank_deg=0.5,
                       pitch_deg=3.0, vspeed_ms=0.05,
                       pitch_rate_dps=0.3, roll_rate_dps=0.1,
                       pitch_cmd_deg=5.0,    # commanding 2° more than model
                       pitch_model_deg=3.0,  # model predicts 3°
                       sweep_port_deg=62.0,  # slight port-heavy trim
                       sweep_stbd_deg=60.0,
                       freq_hz=0.8)
    print(f"  After {i+1} observations:")
    print(f"  delta_cg_long = {r['delta_cg_long_mm']:+.1f}mm  "
          f"(expect ~+16mm forward)")
    print(f"  delta_cg_lat  = {r['delta_cg_lat_mm']:+.1f}mm")
    print(f"  confidence    = {r['confidence']:.3f}")
    print(f"  status        = {r['status']}")

    print()
    print("Compensation controller test:")
    comp = CGCompensator()
    pitches, sweeps, warning, note = comp.apply(
        delta_cg_long_mm=+16.0,  # forward CG
        delta_cg_lat_mm=+3.0,    # slight stbd heavy
        confidence=0.7,
        base_pitches=[3.0]*8,
        base_sweeps=[60.0]*8,
        manoeuvre_name='cruise')
    print(f"  {note}")
    print(f"  W1 pitch: {pitches[0]:+.2f}°  W4 pitch: {pitches[6]:+.2f}°")
    print(f"  Port sweep: {sweeps[0]:.1f}°   Stbd sweep: {sweeps[1]:.1f}°")
    print(f"  Warning: {warning}")

    print()
    print("Safety check test:")
    safe, reason = comp.manoeuvre_cg_safety_check(+22.0, 0.8, 'stoop')
    print(f"  Stoop with +22mm aft CG: safe={safe} '{reason}'")
    safe2, reason2 = comp.manoeuvre_cg_safety_check(+22.0, 0.8, 'cruise')
    print(f"  Cruise with +22mm aft CG: safe={safe2} '{reason2}'")
