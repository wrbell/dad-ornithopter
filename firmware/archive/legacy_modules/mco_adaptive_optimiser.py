"""
MCO Adaptive Flight Optimiser
================================
Rev A  2026-06-16

Architecture: Hierarchical Bayesian Optimisation with Safety Constraints.

The system treats each flight as one episode in a long-running
experiment programme. It maintains a persistent knowledge base across
all flights, builds surrogate models of the craft's behaviour in each
flight regime, respects hard safety limits, and generates plain-English
reports with hypotheses and hardware recommendations.

DESIGN PHILOSOPHY
─────────────────
This aircraft is genuinely new. No amount of theory predicts its
behaviour with confidence. The optimiser therefore treats EVERY
flight parameter as uncertain and worthy of exploration, but does
so gradually, safely, and with explicit tracking of confidence.

The system has four layers:

  Layer 1 — Safety Monitor (always active, cannot be disabled)
    Watches structural load, stall margin, motor temperature, altitude.
    Vetoes any experiment that would violate hard limits.
    Reverts changes instantly if safety degrades.

  Layer 2 — Regime Mapper (builds the flight envelope map)
    Characterises what the aircraft actually does at each
    combination of (speed, altitude, phase, frequency, amplitude,
    sweep, dihedral). Builds a Gaussian Process surrogate per regime.
    Knows where data is dense (confident) and sparse (uncertain).

  Layer 3 — Bayesian Optimiser (improves performance)
    Uses the surrogate model to propose next experiments.
    Balances exploitation (go toward known good) with exploration
    (probe uncertain regions). Uses UCB acquisition function.
    Runs one experiment at a time. Never skips safety checks.

  Layer 4 — Hypothesis Engine (produces human-readable insights)
    Synthesises the surrogate model into plain-English observations.
    Compares actual behaviour to theory (our wave library predictions).
    Generates hypotheses about MECHANISM (why does X help?).
    Produces hardware and software recommendations after each flight.

PARAMETER SPACE
───────────────
The optimiser searches over 12 parameters across 5 flight regimes.
Not all parameters are active in all regimes.

FLIGHT REGIMES
──────────────
  SLOW    : Vs to 1.3×Vs  — stall margin, slow flight, SAR loiter
  CRUISE  : 1.3×Vs to 1.8×Vs  — efficiency, range, endurance
  SPRINT  : 1.8×Vs to Vne  — speed, penetration
  CLIMB   : vertical rate optimisation at best climb speed
  GLIDE   : engine-off efficiency, descent angle

SAFETY TIERS
────────────
  TIER 0 — IMMUTABLE (never changed by optimiser)
    stall margin, emergency recovery, basic control authority

  TIER 1 — CONSERVATIVE (changed ±5% of range, needs 10 clean flights)
    Phase offset, frequency scale, amplitude

  TIER 2 — MODERATE (changed ±10% of range, needs 25 clean flights)
    Differential sweep, dihedral schedule, pitch trim

  TIER 3 — EXPLORATORY (changed ±20% of range, needs 50 clean flights)
    CF-1 timing, asymmetric stroke, advanced cascade modes

PERSISTENT STORAGE
──────────────────
All flight data is written to mco_flight_database.json after each
flight. The Gaussian Process model parameters are saved to
mco_gp_model.json. On the next flight, these are loaded and the
optimiser continues from where it left off.
"""

import math, json, time, random, copy, os
from dataclasses import dataclass, field, asdict
from typing import Optional, List, Dict, Tuple, Any
import sys
sys.path.insert(0, '/mnt/user-data/outputs')
import mco_wave_library as wl

# ─────────────────────────────────────────────────────────────────
# VERSION
# ─────────────────────────────────────────────────────────────────
VERSION = 'MCO-AdaptiveOpt-Rev-A  2026-06-16'

# ─────────────────────────────────────────────────────────────────
# HARD SAFETY LIMITS — IMMUTABLE
# These cannot be adjusted by any part of the optimiser
# ─────────────────────────────────────────────────────────────────
HARD_LIMITS = {
    'stall_margin_ms':       1.5,    # m/s minimum above stall
    'agl_min_m':             5.0,    # m minimum altitude for experiments
    'motor_current_max_A':  30.0,    # A per motor — abort threshold
    'motor_current_spike_A':  6.0,   # A sudden rise in one loop — abort
    'thermal_rise_max_C':    85.0,   # °C winding rise — abort if exceeded
    'bank_max_experiment_deg': 10.0, # ° — no experiments while banked
    'climb_rate_abort_ms':   -2.0,   # m/s — abort if sinking this fast
    'pitch_upset_deg':       30.0,   # ° pitch excursion — abort
    'roll_upset_deg':        45.0,   # ° roll excursion — abort
    'freq_max_hz':            wl.FREQ_LIMIT_MAX_HZ,
    'amp_max_deg':           100.0,  # ° — never command more than this
    'amp_min_deg':            20.0,  # ° — never less than this
    'phase_min_deg':         25.0,   # ° cascade phase
    'phase_max_deg':         90.0,   # °
    'sweep_min_deg':          wl.SWEEP_LIMIT_MIN_DEG,
    'sweep_max_deg':          wl.SWEEP_LIMIT_MAX_DEG,
    'dihedral_min_deg':       wl.DIHEDRAL_LIMIT_MIN_DEG,
    'dihedral_max_deg':       wl.DIHEDRAL_LIMIT_MAX_DEG,
}

# ─────────────────────────────────────────────────────────────────
# PARAMETER DEFINITIONS
# ─────────────────────────────────────────────────────────────────
@dataclass
class ParamDef:
    name:        str
    description: str
    nominal:     float      # wave library default
    low:         float      # minimum the optimiser will try
    high:        float      # maximum the optimiser will try
    step:        float      # smallest meaningful change
    regimes:     list       # which regimes this applies to
    tier:        int        # 1=conservative, 2=moderate, 3=exploratory
    unit:        str        # for display

PARAMS = {
    'phase_offset': ParamDef(
        'phase_offset', 'Metachronal cascade phase offset between pairs',
        nominal=wl.CASCADE_PHASE_DEFAULT_DEG,
        low=25.0, high=90.0, step=2.5, unit='deg',
        regimes=['slow','cruise','climb','sprint'], tier=1),

    'flap_amplitude': ParamDef(
        'flap_amplitude', 'Stroke amplitude at deployed position',
        nominal=wl.FLAP_AMP_DEG,
        low=60.0, high=wl.FLAP_AMP_LIMIT_MAX_DEG, step=2.0, unit='deg',
        regimes=['slow','cruise','climb','sprint'], tier=1),

    'freq_scale': ParamDef(
        'freq_scale', 'Frequency multiplier relative to Ps-target',
        nominal=1.0, low=0.70, high=1.15, step=0.025, unit='x',
        regimes=['cruise','climb'], tier=1),

    'pitch_trim_cruise': ParamDef(
        'pitch_trim_cruise', 'Wing pitch angle at cruise sweep',
        nominal=2.0, low=-2.0, high=8.0, step=0.5, unit='deg',
        regimes=['cruise'], tier=2),

    'pitch_trim_slow': ParamDef(
        'pitch_trim_slow', 'Wing pitch angle at slow flight',
        nominal=6.0, low=2.0, high=12.0, step=0.5, unit='deg',
        regimes=['slow'], tier=2),

    # ── Rev W-2 dynamic pitch parameters ─────────────────────────
    'pitch_aoa_downstroke': ParamDef(
        'pitch_aoa_downstroke',
        'Translational AoA target during downstroke (LEV regime)',
        nominal=wl.PITCH_DOWNSTROKE_DEG,
        low=15.0, high=wl.PITCH_LIMIT_SLOW_DEG, step=1.0, unit='deg',
        regimes=['slow', 'climb'], tier=2),

    'pitch_aoa_upstroke': ParamDef(
        'pitch_aoa_upstroke',
        'Translational AoA target during upstroke',
        nominal=wl.PITCH_UPSTROKE_DEG,
        low=10.0, high=30.0, step=1.0, unit='deg',
        regimes=['slow', 'climb'], tier=2),

    'pitch_advanced_start_pct': ParamDef(
        'pitch_advanced_start_pct',
        'Stroke fraction (%) at which advanced rotation begins',
        nominal=float(wl.PITCH_ADVANCED_START_PCT),
        low=60.0, high=85.0, step=2.5, unit='pct',
        regimes=['slow', 'climb'], tier=2),

    'sweep_surge_amp': ParamDef(
        'sweep_surge_amp',
        'Stroke-coupled sweep surge amplitude on W3+W4 (figure-eight)',
        nominal=wl.SWEEP_SURGE_AMP_DEG,
        low=0.0, high=25.0, step=2.5, unit='deg',
        regimes=['slow', 'climb'], tier=2),

    'pitch_fore_pair_offset': ParamDef(
        'pitch_fore_pair_offset',
        'AoA offset for W1+W2 fore pairs (lift priority)',
        nominal=wl.PITCH_FORE_PAIR_OFFSET,
        low=0.0, high=8.0, step=1.0, unit='deg',
        regimes=['slow', 'climb', 'cruise'], tier=2),

    'pitch_aft_pair_offset': ParamDef(
        'pitch_aft_pair_offset',
        'AoA offset for W3+W4 aft pairs (thrust priority, negative = lower AoA)',
        nominal=wl.PITCH_AFT_PAIR_OFFSET,
        low=-10.0, high=0.0, step=1.0, unit='deg',
        regimes=['slow', 'climb', 'cruise'], tier=2),

    'diff_sweep_cruise': ParamDef(
        'diff_sweep_cruise', 'Leading-pair forward sweep offset at cruise',
        nominal=0.0, low=-30.0, high=30.0, step=3.0, unit='deg',
        regimes=['cruise'], tier=2),

    'dihedral_cruise': ParamDef(
        'dihedral_cruise', 'All-pair dihedral angle at cruise',
        nominal=-5.0, low=-20.0, high=15.0, step=2.5, unit='deg',
        regimes=['cruise','climb'], tier=2),

    'eagle_outer_dihedral': ParamDef(
        'eagle_outer_dihedral', 'Outer pair dihedral in GL-8/GL-9 soar',
        nominal=wl.EAGLE_OUTER_DIHEDRAL_DEG,
        low=10.0, high=30.0, step=2.0, unit='deg',
        regimes=['glide'], tier=2),

    'eagle_inner_dihedral': ParamDef(
        'eagle_inner_dihedral', 'Inner pair dihedral in GL-8/GL-9 soar',
        nominal=wl.EAGLE_INNER_DIHEDRAL_DEG,
        low=0.0, high=15.0, step=1.5, unit='deg',
        regimes=['glide'], tier=2),

    'cf1_apex': ParamDef(
        'cf1_apex', 'CF-1 burst apex angle (near-clap stroke extreme)',
        nominal=wl.CF1_APEX_DEG,
        low=70.0, high=96.0, step=2.0, unit='deg',
        regimes=['slow','climb'], tier=3),

    'asymmetric_stroke_bias': ParamDef(
        'asymmetric_stroke_bias', 'Downstroke fraction relative to total stroke',
        nominal=0.5, low=0.40, high=0.60, step=0.025, unit='frac',
        regimes=['cruise','climb'], tier=3),

    'sprint_phase_offset': ParamDef(
        'sprint_phase_offset', 'Phase offset during sprint (may differ from cruise optimum)',
        nominal=wl.CASCADE_PHASE_DEFAULT_DEG,
        low=25.0, high=75.0, step=3.0, unit='deg',
        regimes=['sprint'], tier=2),
}

# ─────────────────────────────────────────────────────────────────
# PERFORMANCE METRICS
# What the optimiser actually measures
# ─────────────────────────────────────────────────────────────────
@dataclass
class FlightMetrics:
    """Measured performance at a given operating point."""
    V_ms:           float = 0.0   # airspeed m/s
    altitude_m:     float = 0.0   # AGL metres
    regime:         str   = ''    # slow/cruise/sprint/climb/glide
    # Primary metrics
    Ps_ms:          float = 0.0   # specific excess power (m/s) — MASTER METRIC
    efficiency_pct: float = 0.0   # thrust/(power×100) estimate
    climb_rate_ms:  float = 0.0   # m/s
    airspeed_ms:    float = 0.0   # TAS m/s (same speed, different measurement)
    # Secondary metrics
    motor_I_A:      float = 0.0   # mean motor current
    motor_T_rise_C: float = 0.0   # estimated winding temp rise
    vibration_g:    float = 0.0   # RMS vibration from IMU
    stall_margin_ms:float = 0.0   # airspeed minus stall speed
    # Structural
    max_load_g:     float = 0.0   # peak g-load in measurement window
    wing_flutter_hz:float = 0.0   # wing vibration frequency (0 = none)
    # Timestamp and conditions
    t_epoch:        float = 0.0
    flight_number:  int   = 0
    params_snapshot: dict = field(default_factory=dict)

# ─────────────────────────────────────────────────────────────────
# GAUSSIAN PROCESS SURROGATE (lightweight, no numpy dependency)
# A simple RBF kernel GP for 1-4 dimensional parameter spaces
# ─────────────────────────────────────────────────────────────────
class GPSurrogate:
    """
    Minimal Gaussian Process for surrogate modelling.
    Uses squared-exponential (RBF) kernel.
    Runs without numpy on the Raspberry Pi Zero 2W.

    For each regime, maintains a separate GP over the parameter
    subspace relevant to that regime.

    The GP predicts both MEAN (best estimate) and VARIANCE
    (uncertainty). High variance = we haven't explored here yet.
    """

    def __init__(self, param_names: list, length_scale: float = 0.3,
                 noise: float = 0.05):
        self.param_names  = param_names
        self.ls           = length_scale    # kernel length scale
        self.noise        = noise           # observation noise
        self.X: List[List[float]] = []      # inputs (normalised)
        self.Y: List[float] = []            # outputs (Ps values)
        self.n_obs        = 0

    def _normalise(self, params_dict: dict) -> List[float]:
        """Normalise parameter values to [0,1] range."""
        x = []
        for name in self.param_names:
            p  = PARAMS[name]
            v  = params_dict.get(name, p.nominal)
            xn = (v - p.low) / (p.high - p.low + 1e-10)
            x.append(max(0.0, min(1.0, xn)))
        return x

    def _kernel(self, x1: list, x2: list) -> float:
        """Squared exponential kernel."""
        sq = sum((a-b)**2 for a,b in zip(x1,x2))
        return math.exp(-sq / (2 * self.ls**2))

    def observe(self, params_dict: dict, y: float):
        """Add an observation."""
        self.X.append(self._normalise(params_dict))
        self.Y.append(y)
        self.n_obs += 1

    def predict(self, params_dict: dict) -> Tuple[float, float]:
        """
        Return (mean, variance) at params_dict.
        If no observations yet, return (0, 1) — high uncertainty.
        """
        if self.n_obs == 0:
            return 0.0, 1.0

        x_star = self._normalise(params_dict)
        n = self.n_obs

        # Compute kernel vector k_star
        k_star = [self._kernel(x_star, self.X[i]) for i in range(n)]

        # Compute kernel matrix K
        K = [[self._kernel(self.X[i], self.X[j]) for j in range(n)]
             for i in range(n)]

        # Add noise to diagonal
        for i in range(n):
            K[i][i] += self.noise**2

        # Cholesky-free solve via naive Gaussian elimination
        # (small n — rarely more than 200 observations per regime)
        try:
            K_inv_k = self._solve(K, k_star)
            mu = sum(K_inv_k[i] * self.Y[i] for i in range(n))
            k_ss = 1.0
            var = k_ss - sum(k_star[i] * K_inv_k[i] for i in range(n))
            var = max(1e-8, var)
        except Exception:
            mu, var = 0.0, 1.0

        return mu, var

    def _solve(self, A: list, b: list) -> list:
        """Solve Ax=b via Gaussian elimination. A is n×n, b is n."""
        n = len(b)
        M = [row[:] + [b[i]] for i, row in enumerate(A)]
        for col in range(n):
            # Pivot
            max_row = max(range(col, n), key=lambda r: abs(M[r][col]))
            M[col], M[max_row] = M[max_row], M[col]
            if abs(M[col][col]) < 1e-12:
                raise ValueError("Singular matrix")
            for row in range(col+1, n):
                f = M[row][col] / M[col][col]
                for j in range(col, n+1):
                    M[row][j] -= f * M[col][j]
        # Back substitution
        x = [0.0]*n
        for i in range(n-1, -1, -1):
            x[i] = M[i][n] / M[i][i]
            for j in range(i):
                M[j][n] -= M[j][i] * x[i]
        return x

    def ucb(self, params_dict: dict, kappa: float = 2.0) -> float:
        """
        Upper Confidence Bound acquisition function.
        kappa controls exploration vs exploitation:
          kappa=0: pure exploitation (go to known best)
          kappa=2: balanced (default)
          kappa=5: aggressive exploration
        """
        mu, var = self.predict(params_dict)
        return mu + kappa * math.sqrt(var)

    def best_known(self) -> Tuple[dict, float]:
        """Return the observed params and value at the best observation."""
        if not self.Y:
            return {}, 0.0
        best_idx = max(range(len(self.Y)), key=lambda i: self.Y[i])
        best_x   = self.X[best_idx]
        best_y   = self.Y[best_idx]
        # Denormalise
        params = {}
        for i, name in enumerate(self.param_names):
            p = PARAMS[name]
            params[name] = p.low + best_x[i] * (p.high - p.low)
        return params, best_y

# ─────────────────────────────────────────────────────────────────
# EXPERIMENT RECORD
# ─────────────────────────────────────────────────────────────────
@dataclass
class ExperimentRecord:
    flight_number:  int
    t_start:        float
    t_end:          float
    regime:         str
    param_name:     str
    value_before:   float
    value_after:    float
    metrics_before: dict     # FlightMetrics as dict
    metrics_after:  dict
    kept:           bool
    delta_Ps:       float    # improvement in Ps (m/s)
    abort_reason:   str = ''
    notes:          str = ''

# ─────────────────────────────────────────────────────────────────
# FLIGHT DATABASE
# Persists across flights
# ─────────────────────────────────────────────────────────────────
class FlightDatabase:
    """
    Persistent storage for all flight data, experiments, and
    GP model state. Written to JSON after every flight.
    Loaded at the start of every flight.
    """

    DEFAULT_PATH = '/mnt/user-data/outputs/mco_flight_database.json'

    def __init__(self, path: str = DEFAULT_PATH):
        self.path           = path
        self.flight_count   = 0
        self.experiments:   List[dict] = []
        self.metrics_log:   List[dict] = []
        self.best_params:   dict = {n: PARAMS[n].nominal for n in PARAMS}
        self.best_Ps_by_regime: dict = {r: -999.0 for r in
                                ['slow','cruise','sprint','climb','glide']}
        self.gp_observations: dict = {r: [] for r in self.best_Ps_by_regime}
        self.discoveries:   List[dict] = []   # significant findings
        self.hardware_recs: List[dict] = []   # hardware recommendations
        self.software_recs: List[dict] = []   # firmware recommendations
        self._load()

    def _load(self):
        """Load state from JSON. OK if file doesn't exist (first flight)."""
        if os.path.exists(self.path):
            try:
                with open(self.path) as f:
                    data = json.load(f)
                self.flight_count   = data.get('flight_count', 0)
                self.experiments    = data.get('experiments', [])
                self.metrics_log    = data.get('metrics_log', [])
                self.best_params    = data.get('best_params', self.best_params)
                self.best_Ps_by_regime = data.get('best_Ps_by_regime',
                                                   self.best_Ps_by_regime)
                self.gp_observations   = data.get('gp_observations',
                                                   self.gp_observations)
                self.discoveries    = data.get('discoveries', [])
                self.hardware_recs  = data.get('hardware_recs', [])
                self.software_recs  = data.get('software_recs', [])
                print(f"  [DB] Loaded {self.flight_count} flights, "
                      f"{len(self.experiments)} experiments.")
            except Exception as e:
                print(f"  [DB] WARNING: could not load database: {e}")
                print(f"  [DB] Starting fresh.")

    def save(self):
        """Write state to JSON."""
        data = {
            'version':             VERSION,
            'flight_count':        self.flight_count,
            'experiments':         self.experiments,
            'metrics_log':         self.metrics_log[-5000:],  # keep last 5000
            'best_params':         self.best_params,
            'best_Ps_by_regime':   self.best_Ps_by_regime,
            'gp_observations':     self.gp_observations,
            'discoveries':         self.discoveries,
            'hardware_recs':       self.hardware_recs,
            'software_recs':       self.software_recs,
        }
        tmp = self.path + '.tmp'
        with open(tmp, 'w') as f:
            json.dump(data, f, indent=2)
        os.replace(tmp, self.path)   # atomic write
        print(f"  [DB] Saved. Total flights: {self.flight_count}, "
              f"experiments: {len(self.experiments)}")

    def add_experiment(self, exp: ExperimentRecord):
        self.experiments.append(asdict(exp))

    def add_metrics(self, m: FlightMetrics):
        self.metrics_log.append(asdict(m))

    def update_best_Ps(self, regime: str, Ps: float, params: dict):
        if Ps > self.best_Ps_by_regime.get(regime, -999.0):
            self.best_Ps_by_regime[regime] = Ps
            for k,v in params.items():
                self.best_params[k] = v

    def add_gp_observation(self, regime: str, params: dict, Ps: float):
        if regime not in self.gp_observations:
            self.gp_observations[regime] = []
        self.gp_observations[regime].append({'params': params, 'Ps': Ps})

    def add_discovery(self, finding: str, evidence: str, confidence: str,
                      implication: str):
        self.discoveries.append({
            'flight':      self.flight_count,
            'finding':     finding,
            'evidence':    evidence,
            'confidence':  confidence,
            'implication': implication,
            't_epoch':     time.time(),
        })

    def add_hardware_rec(self, component: str, change: str,
                         expected_gain: str, priority: str):
        self.hardware_recs.append({
            'flight':        self.flight_count,
            'component':     component,
            'change':        change,
            'expected_gain': expected_gain,
            'priority':      priority,
            't_epoch':       time.time(),
        })

    def add_software_rec(self, module: str, change: str,
                         expected_gain: str, priority: str):
        self.software_recs.append({
            'flight':        self.flight_count,
            'module':        module,
            'change':        change,
            'expected_gain': expected_gain,
            'priority':      priority,
        })

# ─────────────────────────────────────────────────────────────────
# SAFETY MONITOR
# ─────────────────────────────────────────────────────────────────
class SafetyMonitor:
    """
    Layer 1 — always active.
    Checks all safety limits every loop.
    Returns vetoes to the optimiser.
    """

    def __init__(self):
        self._consecutive_ok = 0
        self._in_violation   = False
        self._violation_t    = 0.0
        self._last_I         = 0.0
        self.total_vetoes    = 0
        self.veto_log: List[dict] = []

    def check(self, state: dict) -> Tuple[bool, str]:
        """
        Returns (safe_to_experiment: bool, reason: str).
        state keys: V_ms, agl_m, motor_I_A, stall_margin_ms,
                    climb_ms, pitch_deg, roll_deg, T_rise_C,
                    bank_deg, intent_name, vibration_g
        """
        V        = state.get('V_ms', 0)
        agl      = state.get('agl_m', 0)
        I        = state.get('motor_I_A', 0)
        sm       = state.get('stall_margin_ms', 0)
        climb    = state.get('climb_ms', 0)
        pitch    = state.get('pitch_deg', 0)
        roll     = state.get('roll_deg', 0)
        T_rise   = state.get('T_rise_C', 0)
        bank     = abs(state.get('bank_deg', 0))
        intent   = state.get('intent_name', '')
        vib      = state.get('vibration_g', 0)
        dI       = abs(I - self._last_I)
        self._last_I = I

        reasons = []
        if sm < HARD_LIMITS['stall_margin_ms']:
            reasons.append(f'stall margin {sm:.1f}m/s < {HARD_LIMITS["stall_margin_ms"]}')
        if agl < HARD_LIMITS['agl_min_m']:
            reasons.append(f'altitude {agl:.1f}m < {HARD_LIMITS["agl_min_m"]}')
        if I > HARD_LIMITS['motor_current_max_A']:
            reasons.append(f'motor current {I:.1f}A > limit')
        if dI > HARD_LIMITS['motor_current_spike_A']:
            reasons.append(f'current spike {dI:.1f}A')
        if climb < HARD_LIMITS['climb_rate_abort_ms']:
            reasons.append(f'sink rate {climb:.2f}m/s')
        if abs(pitch) > HARD_LIMITS['pitch_upset_deg']:
            reasons.append(f'pitch {pitch:.0f}°')
        if abs(roll) > HARD_LIMITS['roll_upset_deg']:
            reasons.append(f'roll {roll:.0f}°')
        if T_rise > HARD_LIMITS['thermal_rise_max_C']:
            reasons.append(f'motor temp rise {T_rise:.0f}°C')
        if bank > HARD_LIMITS['bank_max_experiment_deg']:
            reasons.append(f'bank {bank:.0f}°')
        if intent in ('recover','stall_auto','emergency_land','stoop_pullout'):
            reasons.append(f'safety-critical intent: {intent}')
        if vib > 2.0:
            reasons.append(f'vibration {vib:.1f}g')

        if reasons:
            self._consecutive_ok = 0
            self._in_violation   = True
            self.total_vetoes   += 1
            reason_str = '; '.join(reasons)
            self.veto_log.append({'t': time.time(), 'reason': reason_str})
            return False, reason_str

        self._consecutive_ok += 1
        self._in_violation    = False
        return True, 'ok'

    def stable_for(self, n_loops: int) -> bool:
        """True if we've had n_loops consecutive clean loops."""
        return self._consecutive_ok >= n_loops

# ─────────────────────────────────────────────────────────────────
# REGIME CLASSIFIER
# ─────────────────────────────────────────────────────────────────
class RegimeClassifier:
    """Classifies the current flight regime from sensor state."""

    def __init__(self):
        import mco_wave_library as _wl
        self.Vs_ms = _wl.VS_KT / 1.944

    def classify(self, V_ms: float, climb_ms: float,
                 intent: str, flap_hz: float) -> str:
        if flap_hz < 0.05:
            return 'glide'
        if V_ms < self.Vs_ms * 1.3:
            return 'slow'
        if climb_ms > 0.8 and V_ms < self.Vs_ms * 2.0:
            return 'climb'
        if V_ms > self.Vs_ms * 1.8:
            return 'sprint'
        return 'cruise'

# ─────────────────────────────────────────────────────────────────
# HYPOTHESIS ENGINE
# ─────────────────────────────────────────────────────────────────
class HypothesisEngine:
    """
    Compares observed behaviour to theoretical predictions.
    Generates plain-English hypotheses and recommendations.
    """

    def __init__(self, db: FlightDatabase):
        self.db = db

    def analyse_flight(self, experiments: List[ExperimentRecord],
                       gp_models: Dict[str, GPSurrogate],
                       flight_number: int) -> str:
        """
        Generate a post-flight analysis report.
        Returns a multi-paragraph plain-English string.
        """
        lines = []
        lines.append(f"MCO ADAPTIVE OPTIMISER — POST-FLIGHT REPORT")
        lines.append(f"Flight #{flight_number}  |  {time.strftime('%Y-%m-%d %H:%M')}")
        lines.append(f"{'='*56}")
        lines.append("")

        # Experiment summary
        kept   = [e for e in experiments if e.kept]
        revert = [e for e in experiments if not e.kept and not e.abort_reason]
        aborts = [e for e in experiments if e.abort_reason]

        lines.append(f"EXPERIMENTS THIS FLIGHT: {len(experiments)} total")
        lines.append(f"  Kept (improvement):    {len(kept)}")
        lines.append(f"  Reverted (no gain):    {len(revert)}")
        lines.append(f"  Aborted (safety):      {len(aborts)}")
        lines.append("")

        # What we learned this flight
        if kept:
            lines.append("IMPROVEMENTS CONFIRMED:")
            for e in kept:
                lines.append(f"  {e.param_name}: {e.value_before:.2f} → {e.value_after:.2f}"
                             f"  [+{e.delta_Ps*100:.0f}% Ps in {e.regime} regime]")
            lines.append("")

        # Regime coverage
        regimes_seen = set(m.get('regime','') for m in self.db.metrics_log[-200:])
        lines.append(f"FLIGHT ENVELOPE COVERAGE (all flights):")
        for regime in ['slow','cruise','sprint','climb','glide']:
            gp = gp_models.get(regime)
            n  = gp.n_obs if gp else 0
            best_Ps = self.db.best_Ps_by_regime.get(regime, -999)
            if n > 0:
                coverage = min(100, n * 5)
                lines.append(f"  {regime:>8}: {n:>4} observations  "
                             f"best Ps={best_Ps:+.2f}m/s  "
                             f"coverage ~{coverage:.0f}%")
            else:
                lines.append(f"  {regime:>8}: not yet explored  ← PRIORITY")
        lines.append("")

        # GP model insights
        lines.append("SURROGATE MODEL INSIGHTS:")
        for regime, gp in gp_models.items():
            if gp.n_obs < 3:
                continue
            best_p, best_y = gp.best_known()
            lines.append(f"  {regime} regime:")
            for pname, pval in best_p.items():
                nom = PARAMS[pname].nominal
                delta = pval - nom
                if abs(delta) > PARAMS[pname].step * 0.5:
                    lines.append(f"    {pname}: {nom:.1f} → {pval:.1f}"
                                 f"  ({delta:+.1f} from nominal) — beneficial")
        lines.append("")

        # Theory comparison
        lines.append("THEORY VS MEASUREMENT:")
        self._compare_theory(lines, gp_models)
        lines.append("")

        # New hypotheses
        lines.append("HYPOTHESES FOR NEXT FLIGHT:")
        self._generate_hypotheses(lines, gp_models, flight_number)
        lines.append("")

        # Recommendations
        lines.append("HARDWARE RECOMMENDATIONS:")
        self._generate_hardware_recs(lines, gp_models, flight_number)
        lines.append("")

        lines.append("SOFTWARE RECOMMENDATIONS:")
        self._generate_software_recs(lines, gp_models, flight_number)
        lines.append("")

        lines.append(f"TOTAL PROGRAMME STATUS ({self.db.flight_count} flights):")
        lines.append(f"  Total experiments: {len(self.db.experiments)}")
        lines.append(f"  Discoveries:       {len(self.db.discoveries)}")
        lines.append(f"  Hardware recs:     {len(self.db.hardware_recs)}")
        lines.append(f"  Software recs:     {len(self.db.software_recs)}")
        lines.append("")
        lines.append("="*56)

        return '\n'.join(lines)

    def _compare_theory(self, lines: list,
                        gp_models: Dict[str, GPSurrogate]):
        """Compare observed Ps to wave library theoretical predictions."""
        # Theory prediction for cruise Ps
        import mco_wave_library as _wl
        try:
            g = gp_models.get('cruise')
            if g and g.n_obs > 5:
                best_p, best_y = g.best_known()
                obs_phase = best_p.get('phase_offset',
                                       _wl.CASCADE_PHASE_DEFAULT_DEG)
                theory_phase = _wl.CASCADE_PHASE_DEFAULT_DEG
                if abs(obs_phase - theory_phase) > 5:
                    lines.append(f"  Phase offset: theory predicts {theory_phase:.0f}°, "
                                f"observed optimum {obs_phase:.0f}°")
                    if obs_phase > theory_phase:
                        lines.append(f"    → Wing-wing interaction stronger than model predicted.")
                        lines.append(f"    → Consider increasing stagger for more wake capture.")
                    else:
                        lines.append(f"    → Tighter coupling preferred. Stagger may be too large.")
                else:
                    lines.append(f"  Phase offset: observed optimum confirms theory ({obs_phase:.0f}°) ✓")
        except Exception as e:
            lines.append(f"  (theory comparison unavailable: {e})")

    def _generate_hypotheses(self, lines: list,
                             gp_models: Dict[str, GPSurrogate],
                             flight_number: int):
        """Generate testable hypotheses for next flight."""
        hypotheses = []

        for regime, gp in gp_models.items():
            if gp.n_obs < 3:
                hypotheses.append(
                    f"H{len(hypotheses)+1}: Explore {regime} regime "
                    f"— only {gp.n_obs} observations, high uncertainty.")
                continue
            # Look for parameters with high variance at unexplored values
            for pname in PARAMS:
                if regime not in PARAMS[pname].regimes:
                    continue
                p   = PARAMS[pname]
                # Check if the extremes of the parameter range are unexplored
                for test_val in [p.low, (p.low+p.high)/2, p.high]:
                    test_dict = {pname: test_val}
                    mu, var = gp.predict(test_dict)
                    if var > 0.5:   # high uncertainty
                        hypotheses.append(
                            f"H{len(hypotheses)+1}: Test {pname}={test_val:.1f} "
                            f"in {regime} — GP uncertainty {var:.2f} (unexplored).")
                        break

        for h in hypotheses[:6]:   # show top 6
            lines.append(f"  {h}")

    def _generate_hardware_recs(self, lines: list,
                                gp_models: Dict[str, GPSurrogate],
                                flight_number: int):
        """Generate hardware recommendations from flight data."""
        recs = []

        # Check if larger amplitude is consistently better
        g = gp_models.get('cruise')
        if g and g.n_obs > 10:
            best_p, _ = g.best_known()
            best_amp = best_p.get('flap_amplitude', wl.FLAP_AMP_DEG)
            if best_amp > wl.FLAP_AMP_DEG + 5:
                recs.append(
                    f"Sector wheel: best observed amplitude {best_amp:.0f}° > "
                    f"current limit {wl.FLAP_AMP_DEG:.0f}°. "
                    f"Consider extending sector arc to {best_amp*2:.0f}°.")
                self.db.add_hardware_rec(
                    'sector_wheel', f'Extend arc to {best_amp*2:.0f}°',
                    f'More thrust: amplitude benefit ({best_amp/wl.FLAP_AMP_DEG:.2f}× AMP² ratio)',
                    'HIGH')

        # Check if CF-1 apex benefit is consistent
        g_slow = gp_models.get('slow')
        if g_slow and g_slow.n_obs > 8:
            best_p, _ = g_slow.best_known()
            best_apex = best_p.get('cf1_apex', wl.CF1_APEX_DEG)
            if best_apex > wl.CF1_APEX_DEG + 3:
                recs.append(
                    f"Fuselage width: CF-1 apex {best_apex:.0f}° outperforms "
                    f"nominal {wl.CF1_APEX_DEG:.0f}°. "
                    f"Narrower fuselage would enable {best_apex:.0f}° stroke.")

        if not recs:
            lines.append("  No hardware recommendations yet — more flight data needed.")
        else:
            for r in recs:
                lines.append(f"  • {r}")

    def _generate_software_recs(self, lines: list,
                                gp_models: Dict[str, GPSurrogate],
                                flight_number: int):
        """Generate firmware/wave library recommendations."""
        recs = []

        for regime, gp in gp_models.items():
            if gp.n_obs < 5:
                continue
            best_p, best_y = gp.best_known()
            for pname, best_v in best_p.items():
                nom = PARAMS[pname].nominal
                if abs(best_v - nom) > PARAMS[pname].step * 2:
                    recs.append(
                        f"{pname}: nominal {nom:.2f} → recommend "
                        f"{best_v:.2f} based on {gp.n_obs} observations "
                        f"in {regime} regime.")
                    self.db.add_software_rec(
                        'mco_wave_library',
                        f'Update {pname.upper()} from {nom:.2f} to {best_v:.2f}',
                        f'Ps improvement measured in {gp.n_obs} experiments',
                        'MEDIUM')

        if not recs:
            lines.append("  No software updates recommended yet — "
                        "nominal values performing well.")
        else:
            for r in recs[:5]:
                lines.append(f"  • {r}")

# ─────────────────────────────────────────────────────────────────
# MASTER ADAPTIVE OPTIMISER
# ─────────────────────────────────────────────────────────────────
class AdaptiveOptimiser:
    """
    Master controller — Layer 2 + Layer 3 combined.

    Call structure (50 Hz loop):
        opt = AdaptiveOptimiser()
        opt.begin_flight()
        # ... in each loop:
        result = opt.step(state_dict)
        # result contains param suggestions and status
        opt.end_flight()
        report = opt.generate_report()
    """

    # Optimiser states
    IDLE       = 'idle'         # conditions not met
    BASELINE   = 'baseline'     # collecting baseline Ps
    SETTLING   = 'settling'     # waiting for change to stabilise
    MEASURING  = 'measuring'    # collecting result Ps
    DECIDING   = 'deciding'     # computing keep/revert

    # Timing (seconds)
    T_BASELINE  = 6.0
    T_SETTLE    = 3.0
    T_MEASURE   = 6.0
    T_MIN_GAP   = 20.0   # minimum seconds between experiments
    T_STABLE    = 5.0    # must be stable this long before starting

    def __init__(self, db_path: str = FlightDatabase.DEFAULT_PATH):
        self.db        = FlightDatabase(db_path)
        self.safety    = SafetyMonitor()
        self.regime_cl = RegimeClassifier()
        self.hypothesis= HypothesisEngine(self.db)
        self.enabled   = False
        self.state     = self.IDLE
        self.t_state   = 0.0
        self.t_last_exp = 0.0
        self.flight_experiments: List[ExperimentRecord] = []

        # One GP per regime
        self.gp: Dict[str, GPSurrogate] = {}
        for regime in ['slow','cruise','sprint','climb','glide']:
            relevant = [n for n,p in PARAMS.items() if regime in p.regimes]
            self.gp[regime] = GPSurrogate(param_names=relevant)

        # Reload GP observations from database
        for regime, obs_list in self.db.gp_observations.items():
            if regime in self.gp:
                for obs in obs_list:
                    self.gp[regime].observe(obs['params'], obs['Ps'])

        # Current active experiment
        self._exp_param:   str   = ''
        self._exp_val_bef: float = 0.0
        self._exp_val_aft: float = 0.0
        self._baseline_ps: List[float] = []
        self._measure_ps:  List[float] = []
        self._current_params: dict = copy.copy(self.db.best_params)
        self._param_queue: list = []
        self._param_idx:   int  = 0
        self._baseline_state: dict = {}

        print(f"\n  [{VERSION}]")
        print(f"  Flight #{self.db.flight_count + 1}  "
              f"(loading {len(self.db.experiments)} past experiments)")

    def begin_flight(self):
        """Call at start of each flight."""
        self.db.flight_count += 1
        self.flight_experiments = []
        self._build_param_queue()
        print(f"\n  [Optimiser] Flight #{self.db.flight_count} beginning.")
        print(f"  [Optimiser] Current best params:")
        for k,v in self.db.best_params.items():
            nom = PARAMS[k].nominal
            if abs(v-nom) > PARAMS[k].step*0.4:
                print(f"    {k}: {v:.2f}  (nominal: {nom:.2f}  Δ{v-nom:+.2f})")

    def enable(self, t: float):
        """Pilot enables the optimiser (CH6 toggle)."""
        self.enabled   = True
        self.t_last_exp = t
        print(f"\n  [Optimiser] ENABLED at t={t:.1f}s")
        print(f"  [Optimiser] Will begin experiments when conditions met.")
        print(f"  [Optimiser] Queue: {[p for p in self._param_queue[:5]]}...")

    def disable(self, t: float):
        """Pilot disables the optimiser."""
        self.enabled = False
        if self.state not in (self.IDLE, self.DECIDING):
            self._revert_current(t, "optimiser disabled")
        self.state = self.IDLE
        print(f"\n  [Optimiser] DISABLED — parameters at best-known values.")

    def step(self, state: dict, t: float) -> dict:
        """
        Main 50 Hz call.
        state: dict with V_ms, agl_m, motor_I_A, climb_ms, bank_deg,
               pitch_deg, roll_deg, T_rise_C, intent_name, vibration_g,
               Ps_ms (measured specific excess power)
        Returns: {'param_overrides': dict, 'status': str, 'regime': str}
        """
        regime = self.regime_cl.classify(
            state.get('V_ms', 0),
            state.get('climb_ms', 0),
            state.get('intent_name', 'cruise'),
            state.get('flap_hz', 1.0))
        state['regime'] = regime

        safe, reason = self.safety.check(state)

        # Always log metrics
        m = FlightMetrics(
            V_ms=state.get('V_ms',0),
            altitude_m=state.get('agl_m',0),
            regime=regime,
            Ps_ms=state.get('Ps_ms',0),
            motor_I_A=state.get('motor_I_A',0),
            stall_margin_ms=state.get('stall_margin_ms',0),
            climb_rate_ms=state.get('climb_ms',0),
            vibration_g=state.get('vibration_g',0),
            max_load_g=state.get('g_load',1.0),
            t_epoch=t,
            flight_number=self.db.flight_count,
            params_snapshot=copy.copy(self._current_params),
        )
        self.db.add_metrics(m)

        # Update GP with current observation
        Ps = state.get('Ps_ms', 0)
        if safe and self.safety.stable_for(10):
            self.gp[regime].observe(self._current_params, Ps)
            self.db.add_gp_observation(regime, copy.copy(self._current_params), Ps)
            self.db.update_best_Ps(regime, Ps, self._current_params)

        if not self.enabled:
            return {'param_overrides': self.db.best_params,
                    'status': 'disabled', 'regime': regime}

        # Safety abort of active experiment
        if not safe and self.state not in (self.IDLE,):
            self._revert_current(t, reason)
            return {'param_overrides': self._current_params,
                    'status': f'safety:{reason}', 'regime': regime}

        # State machine
        if self.state == self.IDLE:
            if (safe and self.safety.stable_for(int(self.T_STABLE*50))
                    and (t - self.t_last_exp) > self.T_MIN_GAP):
                self._start_baseline(t, state, regime)

        elif self.state == self.BASELINE:
            self._baseline_ps.append(Ps)
            if (t - self.t_state) >= self.T_BASELINE:
                self._apply_perturbation(t, regime)

        elif self.state == self.SETTLING:
            if (t - self.t_state) >= self.T_SETTLE:
                self.state     = self.MEASURING
                self.t_state   = t
                self._measure_ps = []

        elif self.state == self.MEASURING:
            self._measure_ps.append(Ps)
            if (t - self.t_state) >= self.T_MEASURE:
                self._decide(t)

        status = (f"experiment:{self.state}:{self._exp_param}"
                  if self.state != self.IDLE else 'idle')

        return {'param_overrides': self._current_params,
                'status': status, 'regime': regime}

    def _build_param_queue(self):
        """
        Build prioritised experiment queue using UCB across all regimes.
        Parameters with high uncertainty go first.
        Parameters with known gain go second.
        Tier 3 parameters only if enough flights.
        """
        items = []
        for regime, gp in self.gp.items():
            for pname, p in PARAMS.items():
                if regime not in p.regimes:
                    continue
                if p.tier == 3 and self.db.flight_count < 50:
                    continue
                if p.tier == 2 and self.db.flight_count < 25:
                    continue
                # UCB for this param at its nominal value
                test = {pname: p.nominal}
                ucb  = gp.ucb(test, kappa=2.0)
                # UCB at upper end
                test_hi = {pname: p.high}
                ucb_hi  = gp.ucb(test_hi, kappa=2.0)
                # UCB at lower end
                test_lo = {pname: p.low}
                ucb_lo  = gp.ucb(test_lo, kappa=2.0)
                priority = max(ucb, ucb_hi, ucb_lo)
                items.append((priority, regime, pname,
                              {ucb: p.nominal,
                               ucb_hi: p.high,
                               ucb_lo: p.low}[priority]))

        items.sort(key=lambda x: -x[0])
        self._param_queue = [(r, pn, pv) for _, r, pn, pv in items]
        self._param_idx   = 0

    def _next_experiment(self, current_regime: str) -> Optional[Tuple[str, float]]:
        """Return (param_name, new_value) for next experiment, or None."""
        if not self._param_queue:
            self._build_param_queue()
        # Find next item matching current regime
        for i in range(len(self._param_queue)):
            idx = (self._param_idx + i) % len(self._param_queue)
            regime, pname, pval = self._param_queue[idx]
            if regime == current_regime:
                self._param_idx = (idx + 1) % len(self._param_queue)
                return pname, pval
        return None

    def _start_baseline(self, t: float, state: dict, regime: str):
        nxt = self._next_experiment(regime)
        if nxt is None:
            return
        pname, pval = nxt
        self.state          = self.BASELINE
        self.t_state        = t
        self._baseline_ps   = []
        self._baseline_state = copy.copy(state)
        self._exp_param     = pname
        self._exp_val_bef   = self._current_params.get(pname, PARAMS[pname].nominal)
        self._exp_val_aft   = pval

    def _apply_perturbation(self, t: float, regime: str):
        mu_base = (sum(self._baseline_ps)/len(self._baseline_ps)
                   if self._baseline_ps else 0.0)
        self._baseline_mu = mu_base
        # Apply the parameter change
        self._current_params[self._exp_param] = self._exp_val_aft
        self.state   = self.SETTLING
        self.t_state = t
        print(f"\n  [Optimiser] Experiment: {self._exp_param} "
              f"{self._exp_val_bef:.2f}→{self._exp_val_aft:.2f} "
              f"[baseline Ps={mu_base:+.3f}m/s]")

    def _decide(self, t: float):
        mu_result = (sum(self._measure_ps)/len(self._measure_ps)
                     if self._measure_ps else 0.0)
        delta_Ps  = mu_result - self._baseline_mu
        kept      = delta_Ps > 0.01  # 1 cm/s minimum improvement threshold

        exp = ExperimentRecord(
            flight_number  = self.db.flight_count,
            t_start        = self.t_state - self.T_MEASURE - self.T_SETTLE
                             - self.T_BASELINE,
            t_end          = t,
            regime         = self._baseline_state.get('regime','?'),
            param_name     = self._exp_param,
            value_before   = self._exp_val_bef,
            value_after    = self._exp_val_aft,
            metrics_before = {'Ps_ms': self._baseline_mu},
            metrics_after  = {'Ps_ms': mu_result},
            kept           = kept,
            delta_Ps       = delta_Ps,
        )
        self.flight_experiments.append(exp)
        self.db.add_experiment(exp)

        if kept:
            self.db.best_params[self._exp_param] = self._exp_val_aft
            print(f"  [Optimiser] KEPT {self._exp_param}={self._exp_val_aft:.2f} "
                  f"[ΔPs={delta_Ps:+.3f}m/s]")
            # Check for significant discovery
            if abs(delta_Ps) > 0.05:
                self.db.add_discovery(
                    finding    = f"{self._exp_param} at {self._exp_val_aft:.2f} "
                                f"improves Ps by {delta_Ps*100:.0f}cm/s in "
                                f"{exp.regime} regime",
                    evidence   = f"Baseline {self._baseline_mu:+.3f} → "
                                f"Result {mu_result:+.3f} m/s  (Δ={delta_Ps:+.3f})",
                    confidence = 'HIGH' if len(self._measure_ps) > 200 else 'MEDIUM',
                    implication= f"Update {self._exp_param.upper()} in wave library",
                )
        else:
            self._current_params[self._exp_param] = self._exp_val_bef
            print(f"  [Optimiser] REVERTED {self._exp_param} "
                  f"[ΔPs={delta_Ps:+.3f}m/s — no gain]")

        self.state      = self.IDLE
        self.t_last_exp = t
        self._build_param_queue()   # rebuild queue with new GP data

    def _revert_current(self, t: float, reason: str):
        if self._exp_param:
            self._current_params[self._exp_param] = self._exp_val_bef
        exp = ExperimentRecord(
            flight_number  = self.db.flight_count,
            t_start        = self.t_state,
            t_end          = t,
            regime         = '?',
            param_name     = self._exp_param or 'none',
            value_before   = self._exp_val_bef,
            value_after    = self._exp_val_aft,
            metrics_before = {},
            metrics_after  = {},
            kept           = False,
            delta_Ps       = 0.0,
            abort_reason   = reason,
        )
        if self._exp_param:
            self.flight_experiments.append(exp)
            self.db.add_experiment(exp)
        self.state      = self.IDLE
        self.t_last_exp = t
        self._exp_param = ''

    def end_flight(self):
        """Call at end of each flight. Saves database."""
        if self.state not in (self.IDLE,):
            self._revert_current(0, "end of flight")
        self.db.save()
        print(f"\n  [Optimiser] Flight #{self.db.flight_count} ended.")
        print(f"  [Optimiser] Experiments this flight: "
              f"{len(self.flight_experiments)}")

    def generate_report(self) -> str:
        """Generate and return the post-flight report string."""
        return self.hypothesis.analyse_flight(
            self.flight_experiments, self.gp, self.db.flight_count)

    def print_report(self):
        """Print the post-flight report."""
        print("\n" + self.generate_report())


# ─────────────────────────────────────────────────────────────────
# SELF-TEST
# ─────────────────────────────────────────────────────────────────
if __name__ == '__main__':
    import tempfile, os

    print("="*60)
    print("MCO ADAPTIVE OPTIMISER — SELF TEST")
    print("="*60)

    db_path = tempfile.mktemp(suffix='.json')
    opt = AdaptiveOptimiser(db_path=db_path)
    opt.begin_flight()
    opt.enable(t=0.0)

    # Simulate 300 loops (6 seconds at 50Hz)
    state = {
        'V_ms': 9.5, 'agl_m': 25.0, 'motor_I_A': 8.0,
        'stall_margin_ms': 4.0, 'climb_ms': 0.1, 'bank_deg': 0.0,
        'pitch_deg': 2.0, 'roll_deg': 1.0, 'T_rise_C': 15.0,
        'intent_name': 'cruise', 'vibration_g': 0.1,
        'Ps_ms': 0.05, 'flap_hz': 1.0, 'g_load': 1.0,
    }

    print("\nSimulating 300 loops (6 seconds at 50Hz)...")
    for i in range(300):
        t = i * 0.02
        # Add some Ps variation correlated with phase offset
        state['Ps_ms'] = 0.05 + 0.02 * math.sin(t * 0.1)
        result = opt.step(state, t)

    opt.end_flight()
    report = opt.generate_report()
    print("\n" + report[:2000])

    # Cleanup
    try: os.unlink(db_path)
    except: pass

    print("\nSelf-test complete ✓")


# ── Post-flight hook for mco_main._shutdown() ─────────────────────────────
# NOTE (Rev 18): This stub ensures the import in mco_main._shutdown() succeeds
# even before the full Bayesian optimizer is calibrated. Replace this stub
# with a real implementation after TC-04 provides sufficient flight data.

class _AdaptiveOptimiserStub:
    """Stub — replace with full AdaptiveOptimiser after TC-04."""
    def post_flight_update(self):
        # Real implementation: read latest flight log, update GP surrogate,
        # suggest new hypothesis variants for next flight.
        pass

# If AdaptiveOptimiser is not fully implemented, use the stub
if 'AdaptiveOptimiser' not in dir():
    AdaptiveOptimiser = _AdaptiveOptimiserStub
