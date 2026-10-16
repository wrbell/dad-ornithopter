# NOTE (Rev 18): mco_optimiser.py is the original standalone optimizer.
# The per-tick mechanism weight optimizer is now integrated into
# mco_wave_library.py (_mechanism_learning_tick, mechanism_weight_update).
# Post-flight optimization is handled by:
#   mco_certification_tools.py (HQ analyser → optimizer feedback)
#   mco_adaptive_optimiser.py  (Bayesian optimizer, wired in _shutdown)
# This file is retained for reference and future multi-objective work.
# Do not call mco_optimiser functions from the main flight loop.
#
"""
MCO-1D In-Flight Parameter Optimiser
======================================
Implements a safe, conservative online optimisation system that
experiments with wave library parameters during flight and keeps
changes that improve measured performance.

Design principles:
  1. SAFETY FIRST — any change that threatens stall margin, causes
     unusual current draw, or produces anomalous sensor readings
     is immediately reverted. The aircraft always has an escape route.

  2. ONE PARAMETER AT A TIME — only one parameter is perturbed
     per experiment. This makes it unambiguous which change caused
     which result.

  3. SMALL STEPS — perturbations are a fraction of the parameter
     range. The aircraft nudges, measures, nudges again.

  4. INFORMED SEARCH — the optimiser is not random. It uses the
     performance model's predictions to decide which parameters
     are most likely to yield improvement, and in which direction.

  5. FULL AUDIT TRAIL — every experiment is logged with before,
     after, and measured outcome. The flight delta report includes
     a complete experiment log.

Parameters the optimiser can adjust:
  Phase offset between wing pairs      (PHASE_OFFSET_DEG)
  Flap frequency multiplier            (freq_scale)
  Amplitude at each sweep position     (FLAP_AMP_SCHEDULE entries)
  Dihedral bias at high sweep          (DIHEDRAL_BIAS_SCHEDULE entries)
  Pitch trim at each sweep position    (PITCH_TRIM_SCHEDULE entries)
  CF-1 pause duration                  (CF1_PHASE_CLAP)

Metrics the optimiser maximises:
  Primary:    thrust-to-power ratio  (efficiency — more thrust per watt)
  Secondary:  forward speed          (at same throttle)
  Tertiary:   climb rate             (at same throttle)

The pilot enables the optimiser via a toggle switch (CH6).
When enabled, a gentle beep pattern on the Pixhawk buzzer signals
that an experiment is in progress.
"""

import time, math, copy, random


# ─────────────────────────────────────────────────────────────────
# SAFETY LIMITS — these cannot be violated by the optimiser
# ─────────────────────────────────────────────────────────────────
SAFETY = {
    'stall_margin_min_ms':  1.5,   # m/s — abort if margin drops below this
    'current_max_A':       22.0,   # A per motor — abort if exceeded
    'current_spike_A':      5.0,   # A sudden rise — abort immediately
    'climb_rate_min_ms':   -1.5,   # m/s — abort if sinking faster than this
    'agl_min_m':            3.0,   # m — do not experiment below this altitude
    'speed_min_ms':         4.0,   # m/s — do not experiment below this airspeed
}

# ─────────────────────────────────────────────────────────────────
# PARAMETER DEFINITIONS
# Each entry defines what can be changed and by how much
# ─────────────────────────────────────────────────────────────────
PARAM_DEFS = {
    # name: (current_getter, min_val, max_val, step_size, description)

    # Rev S: phase range updated to [30, 90] per literature
    # (Ford & Santhanakrishnan 2021; Meng 2022; Tiwari 2023)
    'phase_offset': {
        'min': 30.0, 'max': 90.0, 'step': 3.0,
        'desc': "Metachronal phase offset between adjacent wing pairs (degrees)",
        'affects': "cascade wake timing, hindwing thrust augmentation",
        'literature_note': (
            "Tandem-wing studies: max hindwing thrust at phase near 0-45 deg. "
            "Metachronal bio: 30-80 deg for forward locomotion. "
            "Default changed from 90 to 60 deg (Rev S)."
        ),
    },
    # Rev S: differential sweep — the 2nd dimension of the wake-capture
    # optimisation space. Controls effective tip-to-tip longitudinal
    # spacing between adjacent pairs to match wake travel time with
    # phase offset. Leading pair swept forward; trailing at 90 deg.
    # Literature: Meng 2022 showed +40% thrust at optimal spacing.
    'diff_sweep': {
        'min': 45.0, 'max': 90.0, 'step': 3.0,
        'desc': "Leading-pair forward sweep offset for wake-capture geometry (degrees)",
        'affects': "effective tip-to-tip spacing, wake capture timing",
        'literature_note': (
            "At phase=30 deg: optimal spacing ~600mm (3.6c) → leading sweep ~53 deg. "
            "At phase=45 deg: optimal spacing ~900mm (5.5c) → leading sweep ~9 deg. "
            "Coupled with phase_offset in 2D CascadeOptimiser."
        ),
    },
    'freq_scale': {
        'min': 0.85, 'max': 1.15, 'step': 0.03,
        'desc': "Frequency multiplier applied to throttle-to-freq mapping",
        'affects': "overall flap rate, thrust magnitude",
    },
    'amp_60deg': {
        'min': 65.0, 'max': 88.0, 'step': 2.0,
        'desc': "Flap amplitude at 60 deg sweep (normal forward flight)",
        'affects': "thrust in standard cruise",
    },
    'amp_135deg': {
        'min': 12.0, 'max': 28.0, 'step': 2.0,
        'desc': "Flap amplitude at 135 deg sweep (sprint mode)",
        'affects': "thrust in sprint/GA cruise",
    },
    'dihedral_140deg': {
        'min': -14.0, 'max': -4.0, 'step': 1.5,
        'desc': "Dihedral bias at 140 deg sweep (sprint thrust plane)",
        'affects': "forward thrust component in sprint",
    },
    'pitch_trim_90deg': {
        'min': -2.0, 'max': +4.0, 'step': 0.5,
        'desc': "Pitch trim offset at 90 deg sweep",
        'affects': "lift/drag balance in transition",
    },
    'cf1_pause': {
        'min': 6, 'max': 16, 'step': 1,
        'desc': "CF-1 clap pause duration (% of stroke)",
        'affects': "clap-and-fling vortex quality",
    },
}


# ─────────────────────────────────────────────────────────────────
# EXPERIMENT RECORD
# ─────────────────────────────────────────────────────────────────
class Experiment:
    """One parameter perturbation and its measured outcome."""

    def __init__(self, param_name, value_before, value_after,
                 t_start, flight_mode):
        self.param_name    = param_name
        self.value_before  = value_before
        self.value_after   = value_after
        self.t_start       = t_start
        self.flight_mode   = flight_mode

        # Measured baseline (before change)
        self.baseline_thrust_per_watt = None
        self.baseline_speed_ms        = None
        self.baseline_climb_ms        = None

        # Measured result (after change settles)
        self.result_thrust_per_watt   = None
        self.result_speed_ms          = None
        self.result_climb_ms          = None

        # Outcome
        self.improvement_pct   = None   # positive = better
        self.kept              = None   # True/False
        self.abort_reason      = None
        self.t_end             = None

    def score(self):
        """
        Primary metric: thrust-to-power ratio change.
        Secondary: speed change.
        Returns signed improvement percentage.
        """
        if self.result_thrust_per_watt is None:
            return 0.0
        if self.baseline_thrust_per_watt and self.baseline_thrust_per_watt > 0:
            ratio_change = ((self.result_thrust_per_watt
                             - self.baseline_thrust_per_watt)
                            / self.baseline_thrust_per_watt * 100)
        else:
            ratio_change = 0.0
        # Weight speed improvement at 30% of efficiency improvement
        if self.result_speed_ms and self.baseline_speed_ms and self.baseline_speed_ms > 0:
            speed_change = ((self.result_speed_ms - self.baseline_speed_ms)
                            / self.baseline_speed_ms * 100) * 0.30
        else:
            speed_change = 0.0
        return ratio_change + speed_change

    def summary_line(self):
        kept_str = "KEPT" if self.kept else ("REVERTED" if self.kept is False else "ABORTED")
        if self.abort_reason:
            kept_str = f"ABORTED ({self.abort_reason})"
        score_str = f"{self.improvement_pct:+.1f}%" if self.improvement_pct is not None else "n/a"
        return (f"  t={self.t_start:.0f}s  {self.param_name}"
                f"  {self.value_before:.3g}→{self.value_after:.3g}"
                f"  Δ={score_str}  [{kept_str}]")


# ─────────────────────────────────────────────────────────────────
# PARAMETER STATE
# Live values the optimiser can modify
# ─────────────────────────────────────────────────────────────────
class ParameterState:
    """
    Holds the current working values of all optimisable parameters.
    The flight loop reads from here instead of directly from
    wave_library constants, so the optimiser can make live changes.
    """

    def __init__(self):
        try:
            import mco_wave_library as wl
        except ImportError:
            import wave_library as wl

        self.phase_offset    = 90.0    # degrees
        self.freq_scale      = 1.0     # multiplier
        self.amp_60deg       = 80.0    # degrees
        self.amp_135deg      = 20.0    # degrees
        self.dihedral_140deg = -8.0    # degrees
        self.pitch_trim_90   = 0.0     # degrees
        self.cf1_pause       = 10      # % of stroke

        # Track best-known values
        self.best_values     = self._snapshot()
        self.best_score      = -999.0

    def _snapshot(self):
        return {
            'phase_offset':    self.phase_offset,
            'freq_scale':      self.freq_scale,
            'amp_60deg':       self.amp_60deg,
            'amp_135deg':      self.amp_135deg,
            'dihedral_140deg': self.dihedral_140deg,
            'pitch_trim_90':   self.pitch_trim_90,
            'cf1_pause':       self.cf1_pause,
        }

    def get(self, param_name):
        return getattr(self, param_name.replace('dihedral_140deg', 'dihedral_140deg')
                       .replace('pitch_trim_90deg', 'pitch_trim_90'))

    def set(self, param_name, value):
        attr = param_name
        if param_name == 'pitch_trim_90deg':
            attr = 'pitch_trim_90'
        setattr(self, attr, value)

    def restore(self, param_name, value):
        self.set(param_name, value)

    def get_flap_amplitude(self, sweep_deg):
        """
        Returns the flap amplitude for a given sweep angle,
        using the optimiser's live values for the breakpoints
        it controls, and wave_library values for the rest.
        """
        try:
            import mco_wave_library as wl
        except ImportError:
            import wave_library as wl
        if sweep_deg <= 70:
            return self.amp_60deg
        elif sweep_deg <= 100:
            t = (sweep_deg - 70) / 30
            return self.amp_60deg * (1-t) + 40.0 * t
        elif sweep_deg <= 140:
            t = (sweep_deg - 100) / 40
            return 40.0 * (1-t) + self.amp_135deg * t
        else:
            return wl.get_flap_amplitude(sweep_deg)

    def get_dihedral_bias(self, sweep_deg):
        try:
            import mco_wave_library as wl
        except ImportError:
            import wave_library as wl
        if abs(sweep_deg - 140) < 15:
            return self.dihedral_140deg
        return wl.get_dihedral_bias(sweep_deg)

    def get_pitch_trim(self, sweep_deg):
        try:
            import mco_wave_library as wl
        except ImportError:
            import wave_library as wl
        if abs(sweep_deg - 90) < 20:
            return self.pitch_trim_90
        return wl.get_pitch_trim(sweep_deg)

    def update_best(self, score):
        if score > self.best_score:
            self.best_score  = score
            self.best_values = self._snapshot()
            return True
        return False

    def restore_best(self):
        for k, v in self.best_values.items():
            self.set(k, v)


# ─────────────────────────────────────────────────────────────────
# IN-FLIGHT OPTIMISER
# ─────────────────────────────────────────────────────────────────
class InFlightOptimiser:
    """
    Runs during flight when enabled by pilot switch.
    Perturbs one parameter at a time, measures outcome,
    keeps improvements, reverts regressions.
    """

    # States
    IDLE      = 'idle'       # waiting for conditions
    BASELINE  = 'baseline'   # collecting baseline measurement
    PERTURBING= 'perturbing' # change applied, waiting to settle
    MEASURING = 'measuring'  # collecting result measurement
    DECIDING  = 'deciding'   # computing outcome, keeping or reverting

    # Timing
    BASELINE_DURATION_S  = 4.0   # seconds to measure baseline
    SETTLE_DURATION_S    = 2.0   # seconds for change to settle
    MEASURE_DURATION_S   = 4.0   # seconds to measure result
    MIN_INTERVAL_S       = 15.0  # minimum seconds between experiments
    MIN_ALT_M            = SAFETY['agl_min_m']
    MIN_SPEED_MS         = SAFETY['speed_min_ms']

    def __init__(self, params: ParameterState):
        self.params        = params
        self.state         = self.IDLE
        self.enabled       = False
        self.experiments   = []
        self.current_exp   = None
        self._state_t      = 0.0
        self._last_exp_t   = 0.0
        self._baseline_buf = []
        self._result_buf   = []
        self._param_queue  = self._build_queue()
        self._param_idx    = 0

    def _build_queue(self):
        """
        Build a prioritised list of parameters to experiment with.
        Ordered by expected impact — most likely to improve first.
        """
        return [
            'phase_offset',        # most fundamental cascade parameter
            'amp_60deg',           # standard cruise amplitude
            'freq_scale',          # overall rate multiplier
            'dihedral_140deg',     # sprint thrust direction
            'amp_135deg',          # sprint amplitude
            'pitch_trim_90deg',    # transition trim
            'cf1_pause',           # only if CF-1 has been used
        ]

    def enable(self, t):
        if not self.enabled:
            self.enabled = True
            self._last_exp_t = t
            print("\n  [Optimiser] ENABLED — will begin experiments when conditions met")
            print("  [Optimiser] Fly normally. Changes are small and automatically reverted.")

    def disable(self, t):
        if self.enabled:
            self.enabled = False
            # If mid-experiment, abort and revert
            if self.current_exp and self.current_exp.kept is None:
                self._abort(t, "optimiser disabled by pilot")
            self.state = self.IDLE
            print("\n  [Optimiser] DISABLED — all parameters at best-known values")
            self.params.restore_best()

    def _safe_to_experiment(self, V_ms, agl_m, current_A, stall_margin_ms,
                            climb_ms):
        """Check all safety conditions before starting an experiment."""
        if V_ms < self.MIN_SPEED_MS:
            return False, f"speed too low ({V_ms:.1f} < {self.MIN_SPEED_MS} m/s)"
        if agl_m < self.MIN_ALT_M:
            return False, f"altitude too low ({agl_m:.1f} < {self.MIN_ALT_M} m)"
        if stall_margin_ms < SAFETY['stall_margin_min_ms']:
            return False, f"stall margin too small ({stall_margin_ms:.1f} m/s)"
        if current_A > SAFETY['current_max_A']:
            return False, f"current too high ({current_A:.1f} A)"
        if climb_ms < SAFETY['climb_rate_min_ms']:
            return False, f"sinking too fast ({climb_ms:.2f} m/s)"
        return True, "ok"

    def _abort(self, t, reason):
        """Abort current experiment and revert the change."""
        if self.current_exp:
            self.current_exp.abort_reason = reason
            self.current_exp.kept = False
            self.current_exp.t_end = t
            self.params.restore(self.current_exp.param_name,
                                self.current_exp.value_before)
            print(f"  [Optimiser] ABORT {self.current_exp.param_name}: {reason}")
            self.experiments.append(self.current_exp)
            self.current_exp = None
        self.state = self.IDLE
        self._last_exp_t = t

    def _metrics_from_buffer(self, buf):
        """Extract performance metrics from a measurement buffer."""
        if not buf:
            return None, None, None
        thrust_per_watt = (sum(b['thrust_per_watt'] for b in buf)
                           / len(buf))
        speed_ms        = sum(b['V_ms'] for b in buf) / len(buf)
        climb_ms        = sum(b['climb_ms'] for b in buf) / len(buf)
        return thrust_per_watt, speed_ms, climb_ms

    def _choose_perturbation(self, param_name, current_val, flight_mode):
        """
        Choose direction and magnitude of perturbation.
        Uses previous experiment results to guide direction.
        """
        pdef    = PARAM_DEFS[param_name]
        step    = pdef['step']

        # Look at history for this parameter — bias toward
        # the direction that previously helped
        param_history = [e for e in self.experiments
                         if e.param_name == param_name
                         and e.improvement_pct is not None]

        if param_history:
            # Find the best previous result and its direction
            best_prev = max(param_history, key=lambda e: e.improvement_pct or -99)
            if best_prev.improvement_pct > 0.5:
                # Continue in the direction that helped
                direction = 1 if best_prev.value_after > best_prev.value_before else -1
            elif best_prev.improvement_pct < -0.5:
                # Try the opposite direction
                direction = -1 if best_prev.value_after > best_prev.value_before else 1
            else:
                direction = random.choice([-1, 1])
        else:
            # No history — start by nudging toward theoretical optimum
            theoretical_optimum = {
                'phase_offset':    82.0,   # from biological data
                'freq_scale':      1.05,   # slightly higher than base
                'amp_60deg':       82.0,   # close to current
                'amp_135deg':      22.0,   # close to current
                'dihedral_140deg': -10.0,  # slightly more than current
                'pitch_trim_90deg': 1.0,
                'cf1_pause':       10,
            }
            opt = theoretical_optimum.get(param_name, current_val)
            direction = 1 if opt > current_val else -1

        new_val = current_val + direction * step
        new_val = max(pdef['min'], min(pdef['max'], new_val))

        return new_val

    def update(self, t, V_ms, agl_m, freq_hz, current_A,
               power_W, climb_ms, stall_margin_ms, thrust_N,
               flight_mode):
        """
        Main update call — call this every loop at 2Hz when optimiser enabled.
        Returns any parameter changes to apply (or None).
        Updates happen through self.params directly.
        """
        if not self.enabled:
            return

        # Current efficiency metric
        thrust_per_watt = (thrust_N / power_W
                           if power_W > 0.5 else 0.0)

        measurement = {
            'V_ms':            V_ms,
            'thrust_per_watt': thrust_per_watt,
            'climb_ms':        climb_ms,
            'current_A':       current_A,
        }

        # ── Continuous safety watch ───────────────────────────
        # Abort immediately if safety violated mid-experiment
        if self.current_exp:
            if current_A > SAFETY['current_max_A']:
                self._abort(t, f"current spike {current_A:.1f}A")
                return
            if stall_margin_ms < SAFETY['stall_margin_min_ms']:
                self._abort(t, f"stall margin {stall_margin_ms:.1f}m/s")
                return
            if agl_m < self.MIN_ALT_M * 0.8:
                self._abort(t, f"altitude {agl_m:.1f}m")
                return

        # ── State machine ─────────────────────────────────────
        dt_state = t - self._state_t

        if self.state == self.IDLE:
            # Check whether to start a new experiment
            if t - self._last_exp_t < self.MIN_INTERVAL_S:
                return
            safe, reason = self._safe_to_experiment(
                V_ms, agl_m, current_A, stall_margin_ms, climb_ms)
            if not safe:
                return

            # Choose next parameter
            param_name = self._param_queue[
                self._param_idx % len(self._param_queue)]
            self._param_idx += 1

            # Skip cf1_pause unless CF-1 has been used this flight
            if param_name == 'cf1_pause' and not any(
                    'CF-1' in e.param_name for e in self.experiments):
                return

            current_val = self.params.get(param_name)
            self.state      = self.BASELINE
            self._state_t   = t
            self._baseline_buf = []
            self._current_param_name = param_name
            self._current_val_before = current_val
            print(f"\n  [Optimiser] Baseline: {param_name} "
                  f"(current={current_val:.3g})")

        elif self.state == self.BASELINE:
            self._baseline_buf.append(measurement)

            if dt_state >= self.BASELINE_DURATION_S:
                # Enough baseline data — apply perturbation
                new_val = self._choose_perturbation(
                    self._current_param_name,
                    self._current_val_before,
                    flight_mode)

                self.current_exp = Experiment(
                    param_name=self._current_param_name,
                    value_before=self._current_val_before,
                    value_after=new_val,
                    t_start=t,
                    flight_mode=flight_mode,
                )
                # Record baseline metrics
                tpw, spd, clmb = self._metrics_from_buffer(
                    self._baseline_buf)
                self.current_exp.baseline_thrust_per_watt = tpw
                self.current_exp.baseline_speed_ms        = spd
                self.current_exp.baseline_climb_ms        = clmb

                # Apply the change
                self.params.set(self._current_param_name, new_val)
                self.state    = self.PERTURBING
                self._state_t = t
                self._result_buf = []
                print(f"  [Optimiser] Perturbing {self._current_param_name}: "
                      f"{self._current_val_before:.3g} → {new_val:.3g}")

        elif self.state == self.PERTURBING:
            # Let the change settle before measuring
            if dt_state >= self.SETTLE_DURATION_S:
                self.state    = self.MEASURING
                self._state_t = t
                print(f"  [Optimiser] Measuring result...")

        elif self.state == self.MEASURING:
            self._result_buf.append(measurement)

            if dt_state >= self.MEASURE_DURATION_S:
                self.state = self.DECIDING

        elif self.state == self.DECIDING:
            # Compute outcome
            tpw, spd, clmb = self._metrics_from_buffer(self._result_buf)
            self.current_exp.result_thrust_per_watt = tpw
            self.current_exp.result_speed_ms        = spd
            self.current_exp.result_climb_ms        = clmb

            score = self.current_exp.score()
            self.current_exp.improvement_pct = round(score, 2)

            KEEP_THRESHOLD = 0.5   # % — keep if improvement > 0.5%

            if score > KEEP_THRESHOLD:
                # Keep the change
                self.current_exp.kept = True
                self.params.update_best(score)
                print(f"  [Optimiser] KEPT {self.current_exp.param_name}: "
                      f"{self.current_exp.value_before:.3g} → "
                      f"{self.current_exp.value_after:.3g}  "
                      f"(+{score:.1f}%)")
            else:
                # Revert
                self.current_exp.kept = False
                self.params.restore(self.current_exp.param_name,
                                    self.current_exp.value_before)
                if score < -0.5:
                    print(f"  [Optimiser] REVERTED {self.current_exp.param_name} "
                          f"({score:.1f}% — worse)")
                else:
                    print(f"  [Optimiser] REVERTED {self.current_exp.param_name} "
                          f"({score:.1f}% — negligible)")

            self.current_exp.t_end = t
            self.experiments.append(self.current_exp)
            self.current_exp  = None
            self._last_exp_t  = t
            self.state        = self.IDLE
            self._state_t     = t

    def write_experiment_log(self, flight_number, output_dir):
        """Append the experiment log to the delta report."""
        import os
        import datetime
        os.makedirs(output_dir, exist_ok=True)
        now_str = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M")
        fname   = f"flight_{flight_number:03d}_{now_str}_optimiser.txt"
        fpath   = os.path.join(output_dir, fname)

        kept     = [e for e in self.experiments if e.kept is True]
        reverted = [e for e in self.experiments if e.kept is False]
        aborted  = [e for e in self.experiments
                    if e.kept is False and e.abort_reason]

        lines = [
            "=" * 60,
            "MCO-1D IN-FLIGHT OPTIMISER LOG",
            f"Flight {flight_number}",
            "=" * 60,
            "",
            f"Experiments run:    {len(self.experiments)}",
            f"Parameters kept:    {len(kept)}",
            f"Parameters reverted:{len(reverted)}",
            f"Safety aborts:      {len(aborted)}",
            "",
            "EXPERIMENT DETAIL",
        ]
        for e in self.experiments:
            lines.append(e.summary_line())

        lines += ["", "BEST PARAMETER VALUES FOUND THIS FLIGHT"]
        for k, v in self.params.best_values.items():
            lines.append(f"  {k:<25} {v}")

        if kept:
            lines += [
                "",
                "WAVE LIBRARY UPDATES NEEDED",
                "The following kept changes should be written into",
                "wave_library.py as new default values.",
                "Paste this file to Claude with wave_library.py",
                "and request: 'Update wave library with optimiser results'",
                "",
            ]
            for e in kept:
                pdef = PARAM_DEFS.get(e.param_name, {})
                lines.append(f"  {e.param_name}")
                lines.append(f"    Old: {e.value_before}")
                lines.append(f"    New: {e.value_after}  "
                              f"({e.improvement_pct:+.1f}% improvement)")
                lines.append(f"    Affects: {pdef.get('affects', '—')}")
                lines.append("")

        try:
            with open(fpath, 'w') as f:
                f.write('\n'.join(lines) + '\n')
            print(f"[Optimiser] Log saved: {fpath}")
            return fpath
        except Exception as ex:
            print(f"[Optimiser] Failed to save log: {ex}")
            return None


# =================================================================
# PHASE OFFSET OPTIMISER (Post-first-flight; Rev O3-P)
# =================================================================
# The cascade phase offset (90° between adjacent pairs) is set by
# theory. This experiment perturbs phase during steady cruise and
# measures Ps change, converging toward the empirically best offset.
#
# HOW IT WORKS:
#   1. During steady cruise (low bank, stable Ps, V near design),
#      apply a small phase perturbation (±PHASE_PERTURB_DEG).
#   2. Wait PHASE_SETTLE_S for the aircraft to reach steady state.
#   3. Measure Ps before and after.
#   4. Update phase toward the direction that improved Ps.
#   5. Repeat. Converges slowly (α = 0.05) to prevent instability.
#
# SAFETY CONSTRAINTS (non-negotiable):
#   - Phase offset always in [PHASE_MIN_DEG, PHASE_MAX_DEG]
#   - Experiment only runs in CRUISE intent, level flight
#   - Maximum single-step perturbation: ±PHASE_PERTURB_DEG
#   - Experiment disabled unless PHASE_OPT_ENABLED = True
#   - Disabled automatically if bank > 5° or V < 1.2×Vs
#
# OBSERVABILITY NOTE:
#   Ps is fully observable (T−D)V/W. Phase offset is controllable.
#   This is the only parameter in the Q-L research proposal that
#   is both observable with MCO-1D sensors and safe to optimise.
# =================================================================

PHASE_OPT_ENABLED    = False   # Enable only after first-flight validation
PHASE_OPT_DEG        = 90.0    # Current best-known phase offset (degrees)
PHASE_MIN_DEG        = 30.0    # Safety minimum
                               # (literature: 30-80° optimal for metachronal systems)
PHASE_MAX_DEG        = 90.0    # Safety maximum
                               # (90° confirmed suboptimal by tandem-wing literature)
PHASE_PERTURB_DEG    = 5.0     # Maximum perturbation per step
PHASE_SETTLE_S       = 3.0     # Seconds to wait for steady state after perturbation
PHASE_ALPHA          = 0.05    # Learning rate (slow and stable)
PHASE_MIN_V_MS       = None    # Set at init from VS_KT
PHASE_MAX_BANK_DEG   = 5.0     # Must be wings-level


class PhaseOptimiser:
    """
    Bounded phase offset optimiser.
    Safe to run during cruise after first-flight calibration is complete.
    Converges toward the cascade phase offset that maximises Ps.
    """

    def __init__(self):
        import sys as _sys
        if '/mnt/user-data/outputs' not in _sys.path:
            _sys.path.insert(0, '/mnt/user-data/outputs')
        import mco_wave_library as _wl
        self.current_phase = PHASE_OPT_DEG
        self.best_phase    = PHASE_OPT_DEG
        self.best_ps       = -999.0
        self._phase_min_v  = _wl.VS_KT / 1.944 * 1.2
        self._state        = 'idle'   # idle | perturbing | measuring
        self._t_perturb    = 0.0
        self._ps_before    = None
        self._direction    = +1.0     # +1 or -1
        self._n_updates    = 0
        self._log          = []

    def safe_to_run(self, V_ms, bank_deg, intent_name, Ps):
        """Check all safety gates before perturbing."""
        if not PHASE_OPT_ENABLED:
            return False, 'disabled'
        if intent_name != 'cruise':
            return False, f'intent={intent_name}'
        if abs(bank_deg) > PHASE_MAX_BANK_DEG:
            return False, f'bank={bank_deg:.1f}°'
        if V_ms < self._phase_min_v:
            return False, f'V={V_ms:.1f}m/s too slow'
        return True, 'ok'

    def update(self, t, V_ms, bank_deg, Ps_measured, intent_name,
               ctrl_state):
        """
        Call every loop. Returns the recommended phase offset.
        ctrl_state is passed to allow writing the new phase offset
        into the active metachronal timing when updated.
        """
        safe, reason = self.safe_to_run(V_ms, bank_deg, intent_name, Ps_measured)
        if not safe:
            self._state = 'idle'
            return self.current_phase

        if self._state == 'idle':
            # Start a perturbation experiment
            self._ps_before  = Ps_measured
            self._t_perturb  = t
            self._direction  = +1.0 if self._n_updates % 2 == 0 else -1.0
            candidate = self.current_phase + self._direction * PHASE_PERTURB_DEG
            candidate = max(PHASE_MIN_DEG, min(PHASE_MAX_DEG, candidate))
            self.current_phase = candidate
            self._state = 'measuring'

        elif self._state == 'measuring':
            # Wait for steady state
            if (t - self._t_perturb) < PHASE_SETTLE_S:
                return self.current_phase
            # Measure Ps after settling
            ps_after = Ps_measured
            delta_Ps = ps_after - self._ps_before

            # Update toward better Ps
            if delta_Ps > 0:
                # Perturbation improved Ps — keep direction, move further
                new_phase = self.current_phase + PHASE_ALPHA * PHASE_PERTURB_DEG * self._direction
            else:
                # Perturbation hurt Ps — reverse
                new_phase = self.current_phase - PHASE_ALPHA * PHASE_PERTURB_DEG * self._direction

            new_phase = max(PHASE_MIN_DEG, min(PHASE_MAX_DEG, new_phase))
            self.current_phase = new_phase

            if ps_after > self.best_ps:
                self.best_ps    = ps_after
                self.best_phase = new_phase

            self._log.append({
                't': round(t, 1),
                'phase': round(new_phase, 2),
                'delta_Ps': round(delta_Ps, 4),
                'direction': self._direction,
            })
            self._n_updates += 1
            self._state = 'idle'

        return round(self.current_phase, 2)

    def summary(self):
        lines = ['Phase Optimiser Summary:',
                 f'  Enabled: {PHASE_OPT_ENABLED}',
                 f'  Updates: {self._n_updates}',
                 f'  Current phase: {self.current_phase:.1f}°',
                 f'  Best phase:    {self.best_phase:.1f}°  (Ps={self.best_ps:.3f})',
                 f'  Range: [{PHASE_MIN_DEG}°, {PHASE_MAX_DEG}°]']
        if self._log:
            recent = self._log[-5:]
            lines.append('  Last 5 experiments:')
            for r in recent:
                lines.append(f'    t={r["t"]}s  phase={r["phase"]}°  ΔPs={r["delta_Ps"]:+.4f}')
        return '\n'.join(lines)


# =================================================================
# CASCADE OPTIMISER (Rev S) — 2D Phase × Differential-Sweep Search
# =================================================================
# The CascadeOptimiser searches the two-dimensional space of:
#   (phase_offset_deg, diff_sweep_deg)
# to maximise Ps (specific excess power = thrust efficiency proxy).
#
# WHY TWO DIMENSIONS?
#   Phase alone (PhaseOptimiser) cannot fix the wake-capture timing
#   problem: at our root stagger of 0.836c and J=7.24, the shed LEV
#   from W1 reaches W2 in only 1.9% of a stroke period regardless
#   of phase. By simultaneously adjusting the leading pair's sweep
#   angle, we change the effective tip-to-tip spacing and therefore
#   the LEV travel time, matching it to the phase offset.
#
#   The optimal co-ordinate is approximately:
#     phase=30°  + diff_sweep=53°  (leading pair)  → timing error ≈ 1.5%
#     phase=45°  + diff_sweep=45°                  → timing error ≈ 3.0%
#   Both are achievable within normal servo ranges.
#
# SEARCH STRATEGY:
#   Grid exploration of 5 candidate (phase, diff_sweep) pairs,
#   starting from the literature-predicted optimum (30°, 53°).
#   Each candidate is tested for PHASE_SETTLE_S seconds; the one
#   with the highest Ps is kept. Then a local gradient search
#   refines the winner to ±PHASE_PERTURB_DEG in each dimension.
#
#   The search order prioritises candidates by expected Ps gain
#   (literature-predicted benefit, highest first), so early
#   termination due to weather / battery still captures the most
#   valuable experiments first.
#
# SAFETY: identical gates to PhaseOptimiser. Disabled until after
#   first-flight PhaseOptimiser has run and produced a confirmed
#   best phase value to seed the 2D search.
# =================================================================

# ── 2D search enabled flag — set True only after PhaseOptimiser
#    has converged on a best phase value from real flight data.
CASCADE_OPT_ENABLED = False   # Requires PHASE_OPT_ENABLED to have run first

# Candidate (phase_deg, diff_sweep_deg) pairs ordered by
# literature-predicted benefit (highest expected gain first).
# Basis: differential_sweep_for_wake_capture() at each phase value.
# Candidates computed from differential_sweep_for_wake_capture() at each phase:
#   phase=30°: ideal leading sweep = 75.4°, spacing = 1.97c, timing error ≈ 0%
#   phase=45°: ideal leading sweep = 61.9°, spacing = 2.95c, timing error ≈ 0%
#   phase=60°: ideal leading sweep = 46.4°, spacing = 3.94c, timing error ≈ 0%
#   phase=90°: ideal infeasible (needs >1 span); best achievable = 0°, 5.33c
CASCADE_CANDIDATES = [
    (30.0, 75.0),   # phase=30°, sweep=75°: ideal spacing 1.97c ← highest expected gain
    (45.0, 62.0),   # phase=45°, sweep=62°: ideal spacing 2.95c
    (60.0, 46.0),   # phase=60°, sweep=46°: ideal spacing 3.94c
    (45.0, 60.0),   # phase=45°, sweep=60°: slightly sub-optimal for comparison
    (30.0, 60.0),   # phase=30°, sweep=60°: sub-optimal spacing for comparison
]


class CascadeOptimiser:
    """
    2D (phase_offset, diff_sweep) optimiser for metachronal wake capture.

    Run after PhaseOptimiser has confirmed a stable best phase value
    in real flight. The 2D search refines both phase and the leading-
    pair sweep angle simultaneously to maximise Ps.

    Disabled by default (CASCADE_OPT_ENABLED = False).
    Enable after PhaseOptimiser has converged.
    """

    def __init__(self):
        import sys as _sys
        if '/mnt/user-data/outputs' not in _sys.path:
            _sys.path.insert(0, '/mnt/user-data/outputs')
        import mco_wave_library as _wl
        self._phase_min_v = _wl.VS_KT / 1.944 * 1.2
        # Seed from PHASE_OPT_DEG (result of PhaseOptimiser)
        self.current_phase    = PHASE_OPT_DEG
        self.current_diff_sw  = 60.0              # deployed = no differential
        self.best_phase       = PHASE_OPT_DEG
        self.best_diff_sw     = 60.0
        self.best_ps          = -999.0
        self._state           = 'idle'
        self._t_exp           = 0.0
        self._ps_before       = None
        self._candidate_idx   = 0                 # which candidate to try next
        self._phase_lut       = None              # lazy-load from wl
        self._n_updates       = 0
        self._log             = []

    def _load_lut(self):
        """Build wake-capture geometry LUT from wave library."""
        if self._phase_lut is not None:
            return
        import mco_wave_library as _wl
        self._phase_lut = {}
        for phase, dsw in CASCADE_CANDIDATES:
            geo = _wl.differential_sweep_for_wake_capture(phase)
            self._phase_lut[(phase, dsw)] = geo

    def safe_to_run(self, V_ms, bank_deg, intent_name):
        if not CASCADE_OPT_ENABLED:
            return False, 'disabled — run PhaseOptimiser first'
        if intent_name != 'cruise':
            return False, f'intent={intent_name}'
        if abs(bank_deg) > PHASE_MAX_BANK_DEG:
            return False, f'bank={bank_deg:.1f}°'
        if V_ms < self._phase_min_v:
            return False, f'V={V_ms:.1f}m/s too slow'
        return True, 'ok'

    def update(self, t, V_ms, bank_deg, Ps_measured, intent_name):
        """
        Call every loop. Returns (phase_deg, diff_sweep_deg).
        The FCC applies phase_deg to metachronal_phase_offsets()
        and diff_sweep_deg as the leading-pair sweep offset.
        """
        safe, _ = self.safe_to_run(V_ms, bank_deg, intent_name)
        if not safe:
            self._state = 'idle'
            return self.current_phase, self.current_diff_sw

        self._load_lut()

        if self._state == 'idle':
            if self._candidate_idx >= len(CASCADE_CANDIDATES):
                # All candidates tested — do local gradient refinement
                self._state = 'refining'
                self._t_exp = t
                self._ps_before = Ps_measured
                # Perturb in phase dimension first
                self._refine_dim = 'phase'
                cand_phase = max(PHASE_MIN_DEG,
                    min(PHASE_MAX_DEG,
                        self.current_phase - PHASE_PERTURB_DEG))
                self.current_phase = cand_phase
            else:
                # Try next candidate from the ordered list
                cand_phase, cand_dsw = CASCADE_CANDIDATES[self._candidate_idx]
                self.current_phase   = cand_phase
                self.current_diff_sw = cand_dsw
                self._ps_before = Ps_measured
                self._t_exp     = t
                self._state     = 'measuring'
            return self.current_phase, self.current_diff_sw

        elif self._state == 'measuring':
            if (t - self._t_exp) < PHASE_SETTLE_S:
                return self.current_phase, self.current_diff_sw
            ps_after   = Ps_measured
            delta_Ps   = ps_after - self._ps_before
            entry = {
                't':           round(t, 1),
                'phase':       self.current_phase,
                'diff_sw':     self.current_diff_sw,
                'delta_Ps':    round(delta_Ps, 4),
                'candidate_i': self._candidate_idx,
            }
            if ps_after > self.best_ps:
                self.best_ps      = ps_after
                self.best_phase   = self.current_phase
                self.best_diff_sw = self.current_diff_sw
                entry['is_best'] = True
            else:
                entry['is_best'] = False
                # Revert to best known while testing
                self.current_phase   = self.best_phase
                self.current_diff_sw = self.best_diff_sw
            self._log.append(entry)
            self._candidate_idx += 1
            self._n_updates += 1
            self._state = 'idle'

        elif self._state == 'refining':
            if (t - self._t_exp) < PHASE_SETTLE_S:
                return self.current_phase, self.current_diff_sw
            ps_after = Ps_measured
            delta_Ps = ps_after - self._ps_before
            if delta_Ps > 0:
                pass   # keep current perturbation
            else:
                # Revert this dimension, try other direction
                if self._refine_dim == 'phase':
                    self.current_phase = min(PHASE_MAX_DEG,
                        self.best_phase + PHASE_PERTURB_DEG)
                else:
                    self.current_diff_sw = min(90.0,
                        self.best_diff_sw + PHASE_PERTURB_DEG)
            if self._refine_dim == 'phase':
                self._refine_dim = 'diff_sw'
                self.current_diff_sw = max(PARAM_DEFS['diff_sweep']['min'],
                    self.best_diff_sw - PHASE_PERTURB_DEG)
            else:
                self._state = 'idle'   # refinement complete this cycle
            self._t_exp = t
            self._ps_before = Ps_measured
            self._n_updates += 1

        return self.current_phase, self.current_diff_sw

    def summary(self):
        lines = [
            'CascadeOptimiser Summary (Rev S):',
            f'  Enabled:       {CASCADE_OPT_ENABLED}',
            f'  Updates:       {self._n_updates}',
            f'  Current:       phase={self.current_phase:.1f}°  '
            f'diff_sw={self.current_diff_sw:.1f}°',
            f'  Best:          phase={self.best_phase:.1f}°  '
            f'diff_sw={self.best_diff_sw:.1f}°  '
            f'(Ps={self.best_ps:.3f})',
            f'  Candidates tested: {self._candidate_idx}/{len(CASCADE_CANDIDATES)}',
        ]
        if self._log:
            lines.append('  Last 5 experiments:')
            for r in self._log[-5:]:
                best_tag = ' ← BEST' if r.get('is_best') else ''
                lines.append(
                    f'    t={r["t"]}s  phase={r["phase"]:.0f}°  '
                    f'dsw={r["diff_sw"]:.0f}°  '
                    f'ΔPs={r["delta_Ps"]:+.4f}{best_tag}')
        return '\n'.join(lines)
