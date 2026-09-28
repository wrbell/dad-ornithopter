"""
MCO-1D Performance Model  (Boyd-enhanced)
==========================================
Adds three Boyd-informed enhancements to the existing model:

1. SPECIFIC EXCESS POWER (Ps)
   Boyd's core metric from Energy-Maneuverability theory.
   Ps = (Thrust - Drag) / Weight × Velocity
   Positive: aircraft can accelerate, climb, or tighten manoeuvre.
   Zero:     maximum sustained performance at this state.
   Negative: aircraft is spending energy it cannot sustain.

2. ENERGY STATE TRACKING
   Accumulates Ps across the flight to build an E-M diagram:
   a map of (frequency, sweep) operating points and their Ps values.
   The optimiser can read this map to find the region of
   highest sustained Ps — the sweet spot Boyd would seek.

3. STOOP Ps MONITOR
   During the stoop dive, Ps is negative (gravity-powered).
   The zero-crossing of Ps during pullout is the aerodynamically
   correct trigger point for STOOP-4A — the point at which the
   aircraft's own power can sustain the manoeuvre without
   continued energy debt. This replaces the fixed-altitude trigger
   with a physics-based one.

Boyd reference:
   Boyd, J.R., Aerial Attack Study, Report 50-10-6C, 1964.
   Boyd & Christie, Energy-Maneuverability Theory, 1966.
   Ps = (T - D) / W × V  — the foundational E-M equation.
"""

import math
import os
import datetime


# ─────────────────────────────────────────────────────────────────
# PHYSICAL CONSTANTS  (unchanged from original)
# ─────────────────────────────────────────────────────────────────
RHO          = 1.225
G            = 9.81
MTOW_KG      = 5.687

CHORD_M      = 0.165
SEMI_SPANS_M = {1: 1.000, 2: 1.000, 3: 1.000, 4: 1.000}
AR           = 12.12
E_OSWALD     = 0.92
S_TOTAL_M2   = 1.320
CL_MAX       = 1.60
CD0          = 0.022

REDUCER_RATIO = 10
KV           = 200.0
KT           = 60 / (2 * math.pi * KV)
RI           = 0.10
VBUS         = 22.2
I_WING       = 0.0240  # kg*m^2 (1.0m wing panel + tip moment of inertia)

# ─────────────────────────────────────────────────────────────────
# BOYD CONSTANTS
# ─────────────────────────────────────────────────────────────────
# Ps thresholds for real-time alerts
PS_WARN_NEGATIVE   = -0.5   # m/s  — mild energy debt, acceptable in turns
PS_ALERT_NEGATIVE  = -2.0   # m/s  — significant energy debt, log it
PS_STOOP_PULLOUT   =  0.0   # m/s  — Ps zero-crossing: pullout trigger

# EM diagram grid resolution
EM_FREQ_STEPS  = [0.4, 0.6, 0.8, 1.0, 1.5, 2.0, 2.5, 3.0]   # Hz
EM_SWEEP_STEPS = [60, 90, 120, 135, 150, 165]                  # degrees


# ─────────────────────────────────────────────────────────────────
# PERFORMANCE MODEL  (original, unchanged)
# ─────────────────────────────────────────────────────────────────
class PerformanceModel:
    """Original predict() unchanged — Boyd additions are additive."""

    def __init__(self):
        self.W = MTOW_KG * G

    def predict(self, freq_hz, V_ms, sweep_deg_mean,
                amplitude_deg, throttle_frac,
                bank_deg=0.0, h_agl_m=0.0):
        result = {}
        sweep_from_abeam = max(0.0, sweep_deg_mean - 60.0)
        S_eff = S_TOTAL_M2 * math.cos(math.radians(sweep_from_abeam))
        S_eff = max(0.01, S_eff)

        Vs = math.sqrt(2 * self.W / (RHO * S_eff * CL_MAX))
        result['stall_speed_ms']  = round(Vs, 2)
        result['stall_margin_ms'] = round(V_ms - Vs, 2)
        result['stall_margin_pct']= round((V_ms - Vs) / Vs * 100, 1)

        # ── ITEM 2: Load-factor-adjusted stall (FAA AFH Ch.10) ──
        # In a banked turn stall speed increases as sqrt(load factor)
        import math as _math
        bank_rad = _math.radians(abs(bank_deg))
        cos_bank = max(0.10, _math.cos(bank_rad))
        lf = 1.0 / cos_bank
        Vs_turn = Vs * _math.sqrt(lf)
        result['stall_speed_banked_ms'] = round(Vs_turn, 2)
        result['stall_margin_banked_ms'] = round(V_ms - Vs_turn, 2)
        result['load_factor'] = round(lf, 3)

        # ── ITEM 4: Energy height (FAA AFH Ch.4) ─────────────────
        # ES = h + V²/(2g)  — total specific energy in metres
        ES = h_agl_m + (V_ms ** 2) / (2.0 * 9.81)
        result['energy_height_m'] = round(ES, 2)

        q = 0.5 * RHO * max(V_ms, 0.5)**2
        CL_needed = min(self.W / (q * S_eff), CL_MAX)
        CDi = CL_needed**2 / (math.pi * AR * E_OSWALD)
        result['CL_operating'] = round(CL_needed, 3)

        amp_rad = math.radians(amplitude_deg)
        b_mean  = sum(SEMI_SPANS_M.values()) / 4
        V_tip   = 2 * math.pi * freq_hz * amp_rad * b_mean
        result['V_tip_ms'] = round(V_tip, 2)

        J = V_ms / V_tip if V_tip > 0.01 else 99.0
        result['advance_ratio'] = round(J, 3)
        result['regime'] = 'unsteady' if J < 1.0 else 'quasi-steady'

        if   J < 0.5:  CT_base = 0.35
        elif J < 1.0:  CT_base = 0.35 - 0.10 * (J - 0.5) / 0.5
        else:          CT_base = 0.25 - 0.10 * min(J - 1.0, 1.0)
        sweep_penalty = max(0.40, min(1.0,
                            1.0 - 0.35 * (sweep_from_abeam / 75.0)))
        CT = CT_base * sweep_penalty

        V_eff  = math.sqrt(V_ms**2 + V_tip**2)
        T_pred = CT * 0.5 * RHO * V_eff**2 * S_eff
        result['thrust_N']     = round(T_pred, 1)
        result['CT_predicted'] = round(CT, 3)

        D_profile = CD0 * q * S_eff
        D_induced = CDi * q * S_eff
        D_fus     = 0.120 * q
        D_total   = D_profile + D_induced + D_fus
        result['drag_N']  = round(D_total, 1)
        result['lift_N']  = round(CL_needed * q * S_eff, 1)

        net_thrust = T_pred - D_total
        result['net_thrust_N']  = round(net_thrust, 1)
        excess_power = net_thrust * V_ms
        result['climb_rate_ms'] = round(excess_power / self.W, 2)

        rpm_motor = freq_hz * 60 * REDUCER_RATIO
        omega     = rpm_motor * 2 * math.pi / 60
        BEMF      = KT * omega
        # Combined root torque: static aerodynamic lift + reciprocating wing inertia
        T_aero    = (self.W / 8) * (b_mean * 0.55)
        omega_osc = 2.0 * math.pi * freq_hz
        T_inert   = I_WING * amp_rad * (omega_osc ** 2)
        T_root    = math.sqrt(T_aero**2 + (0.707 * T_inert)**2)
        T_motor   = T_root / REDUCER_RATIO
        I_motor   = max(0.0, min(T_motor / KT, 30.0))
        P_elec_all= VBUS * I_motor * 8

        result['motor_rpm']       = round(rpm_motor, 0)
        result['BEMF_V']          = round(BEMF, 2)
        result['current_A_each']  = round(I_motor, 2)
        result['current_A_total'] = round(I_motor * 8, 2)
        result['power_W']         = round(P_elec_all, 1)
        result['heat_W_each']     = round(I_motor**2 * RI, 3)
        result['thermal_ok']      = I_motor**2 * RI < 15.0

        eff = T_pred / P_elec_all * 100 if P_elec_all > 0.1 else 0.0
        result['efficiency_pct']  = round(eff, 1)

        mah_ps = (I_motor * 8 * 1000) / 3600 if I_motor * 8 > 0.01 else 0.001
        result['endurance_min']   = round(5000 / mah_ps / 60, 1)

        # ── BOYD ADDITION: Specific Excess Power ─────────────
        result['Ps'] = specific_excess_power(
            T_pred, D_total, self.W, V_ms)
        result['Ps_status'] = ps_status(result['Ps'])

        # ── ENERGY STEWARDSHIP: Cost of Transport & Carson's Speed ──
        cot = cost_of_transport(P_elec_all, self.W, V_ms)
        v_carson = carsons_speed(self.W)
        wh_km = (P_elec_all / (3.6 * V_ms)) if V_ms > 0.5 else 999.0
        j_per_m = (P_elec_all / V_ms) if V_ms > 0.5 else 999.0
        result['cost_of_transport'] = round(cot, 4)
        result['carsons_speed_ms']  = round(v_carson, 2)
        result['wh_per_km']         = round(wh_km, 2)
        result['joules_per_meter']  = round(j_per_m, 2)

        return result


# ─────────────────────────────────────────────────────────────────
# BOYD CORE FUNCTIONS
# ─────────────────────────────────────────────────────────────────

# SOFT-STOP EFFICIENCY NOTE:
# The soft-stop system reduces motor velocity to 35% in the final 10°
# before each reversal. The aerodynamic effect on thrust and efficiency
# depends on what fraction of the stroke is in the taper zone.
#
# Geometric analysis at amp=96°, zone=10°:
#   Zone fraction = 10/96 = 10.4% of half-stroke
#   Mean velocity in zone ≈ 0.675 × full (cosine average over zone)
#   Thrust reduction (proportional to v²) in zone ≈ 1 - 0.675² = 54%
#   Weighted thrust reduction = 0.104 × 0.54 = ~5.6% total thrust loss
#
# This is the cost of rope protection. At 1.87 Hz hover frequency,
# the 5.6% thrust loss raises the hover frequency requirement by ~2.8%
# (from 1.87 Hz to ~1.92 Hz) — well within the measurement uncertainty
# of TC-02 and the available thermal margin.
#
# TC-01 step 12 (static thrust curve) measures the ACTUAL thrust
# including the soft-stop effect, so the TC-01 feasibility gate
# automatically accounts for it.
SOFT_STOP_THRUST_LOSS_ESTIMATE = 0.056   # ~5.6% at zone=10°, min_vel=0.35

def specific_excess_power(thrust_N, drag_N, weight_N, V_ms):
    """
    Boyd's Ps — Specific Excess Power.
    Ps = (Thrust - Drag) / Weight × Velocity

    Units: metres per second (equivalent climb rate at current speed).

    Positive: aircraft has energy surplus — can accelerate, climb,
              or tighten manoeuvre without losing energy state.
    Zero:     aircraft is at maximum sustained performance for this
              combination of speed, altitude, and configuration.
              This is the boundary of the sustainable envelope.
    Negative: aircraft is spending energy it cannot replenish —
              it will slow down or descend unless configuration changes.

    In the stoop:  Ps is negative (engine cannot match gravity input).
    At pullout:    Ps crosses zero when the wings generate enough lift
                   and thrust to sustain the manoeuvre — this is the
                   Boyd-informed trigger for STOOP-4A completion.
    In cruise:     Ps should be positive, indicating reserve capacity
                   for gusts, turns, or acceleration.
    """
    if weight_N <= 0 or V_ms < 0.1:
        return 0.0
    return round((thrust_N - drag_N) / weight_N * V_ms, 3)


def ps_status(Ps):
    """Human-readable Ps status string."""
    if   Ps >  2.0: return "SURPLUS (strong)"
    elif Ps >  0.5: return "SURPLUS (margin)"
    elif Ps >  0.0: return "SURPLUS (marginal)"
    elif Ps > -0.5: return "NEUTRAL"
    elif Ps > -2.0: return "DEFICIT (manageable)"
    else:           return "DEFICIT (significant)"


def ps_pullout_recommended(Ps, agl_m, V_ms, stall_margin_ms):
    """
    Boyd-informed stoop pullout recommendation.
    Returns (should_pull_out: bool, reason: str).

    The correct pullout trigger is when Ps crosses zero on the way
    to positive — the aircraft's own power can now sustain the
    manoeuvre. Pulling out earlier wastes altitude; pulling out
    later risks insufficient recovery energy.
    """
    if agl_m < 25.0:
        return True, f"HARD FLOOR: AGL {agl_m:.0f}m — pull out NOW"
    if stall_margin_ms < 1.0:
        return True, f"STALL MARGIN: {stall_margin_ms:.1f}m/s — pull out NOW"
    if Ps >= PS_STOOP_PULLOUT and V_ms > 6.0:
        return True, f"Ps={Ps:.2f} m/s at V={V_ms:.1f}m/s — Ps zero crossing reached"
    return False, f"Ps={Ps:.2f} m/s — continue"


def cost_of_transport(power_W, weight_N, V_ms):
    """
    Dimensionless Cost of Transport (CoT):
      CoT = P_elec / (W * V)
    Measures Joules expended per Newton of weight per meter of flight.
    Lower values indicate greater aerodynamic and electrical efficiency.
    """
    if weight_N <= 0 or V_ms < 0.2:
        return 99.0
    return max(0.0, power_W / (weight_N * V_ms))


def carsons_speed(weight_N=None, altitude_m=0.0):
    """
    Carson's Speed (Best Range Speed, B.H. Carson 1980):
      V_br = 3^(1/4) * V_md ≈ 1.31607 * V_md
    Maximises forward velocity per unit power, giving maximum distance
    per battery Watt-hour (km/Wh) in cruise.
    """
    w = weight_N if (weight_N is not None and weight_N > 0) else (MTOW_KG * G)
    rho = RHO * math.exp(-altitude_m / 8500.0) if altitude_m > 0 else RHO
    denom = math.sqrt(math.pi * AR * E_OSWALD * CD0)
    if denom <= 0:
        return 10.0
    v_md = math.sqrt(2.0 * w / (rho * S_TOTAL_M2 * denom))
    return round((3.0 ** 0.25) * v_md, 2)


# ─────────────────────────────────────────────────────────────────
# ENERGY-MANEUVERABILITY MAP
# Builds the EM diagram from accumulated flight data
# ─────────────────────────────────────────────────────────────────

class EMMap:
    """
    Accumulates (freq, sweep, V_ms, Ps) observations during flight
    and builds a grid of mean Ps values — the MCO-1D E-M diagram.

    After enough flights the map identifies:
    - The (freq, sweep) combination that maximises Ps
      → optimal sustained cruise configuration
    - The zero-Ps boundary
      → the edge of the sustainable envelope
    - Regions of Ps deficit
      → configurations to avoid for sustained flight

    The map is written to the flight log and can be pasted
    to Claude for analysis and wave library updates.
    """

    def __init__(self):
        # Grid: key = (freq_bucket, sweep_bucket), value = [Ps samples]
        self._grid   = {}
        self._n_obs  = 0
        self.best_freq  = None
        self.best_sweep = None
        self.best_Ps    = -999.0

    def _bucket(self, freq_hz, sweep_deg):
        """Snap to nearest grid point."""
        fb = min(EM_FREQ_STEPS,
                 key=lambda f: abs(f - freq_hz))
        sb = min(EM_SWEEP_STEPS,
                 key=lambda s: abs(s - sweep_deg))
        return fb, sb

    def record(self, freq_hz, sweep_deg, V_ms, Ps):
        """Record one Ps observation at this operating point."""
        if V_ms < 1.0:
            return   # don't record ground data
        key = self._bucket(freq_hz, sweep_deg)
        if key not in self._grid:
            self._grid[key] = []
        self._grid[key].append(Ps)
        self._n_obs += 1

        # Track best known operating point
        mean_Ps = sum(self._grid[key]) / len(self._grid[key])
        if mean_Ps > self.best_Ps:
            self.best_Ps    = mean_Ps
            self.best_freq  = key[0]
            self.best_sweep = key[1]

    def mean_ps(self, freq_hz, sweep_deg):
        """Return mean Ps at this operating point, or None."""
        key = self._bucket(freq_hz, sweep_deg)
        vals = self._grid.get(key)
        if not vals:
            return None
        return round(sum(vals) / len(vals), 3)

    def summary(self):
        """Return a formatted EM map for the flight log."""
        if not self._grid:
            return ["  EM Map: no data (< 1 m/s throughout)"]

        lines = [
            "  ENERGY-MANEUVERABILITY MAP  (Boyd Ps grid)",
            "  Ps = (Thrust-Drag)/Weight × V  [m/s]",
            "  Positive = energy surplus  |  Zero = max sustained  "
            "|  Negative = energy debt",
            "",
            "  Sweep →",
            "  Freq ↓    " +
            "".join(f"  {s:>5}°" for s in EM_SWEEP_STEPS),
            "  " + "─" * (10 + 8 * len(EM_SWEEP_STEPS)),
        ]

        for fb in EM_FREQ_STEPS:
            row = f"  {fb:.1f} Hz    "
            for sb in EM_SWEEP_STEPS:
                val = self.mean_ps(fb, sb)
                if val is None:
                    row += "     — "
                elif val > 0.5:
                    row += f"  {val:+.2f}"
                elif val > 0.0:
                    row += f"  {val:+.2f}"
                elif val > -0.5:
                    row += f"  {val:+.2f}"
                else:
                    row += f"  {val:+.2f}"
            lines.append(row)

        lines += [
            "",
            f"  Total observations: {self._n_obs}",
        ]
        if self.best_freq:
            lines.append(
                f"  Best operating point: "
                f"{self.best_freq}Hz / {self.best_sweep}° sweep  "
                f"(mean Ps={self.best_Ps:+.3f} m/s)")
            lines.append(
                f"  → Recommended wave library params: "
                f"CRUISE_FREQ_HZ={self.best_freq}, "
                f"CRUISE_SWEEP_DEG={self.best_sweep}")
        return lines

    def best_config_recommendation(self):
        """
        Return a dict with the best-known wave library parameters,
        ready to paste into the optimiser log for Claude.
        """
        if self.best_freq is None:
            return None
        return {
            'source':           'EM_Map_Boyd',
            'CRUISE_FREQ_HZ':   self.best_freq,
            'CRUISE_SWEEP_DEG': self.best_sweep,
            'mean_Ps_ms':       round(self.best_Ps, 3),
            'note': (f"Best sustained operating point from EM map. "
                     f"Ps={self.best_Ps:+.3f} m/s at "
                     f"{self.best_freq}Hz/{self.best_sweep}° sweep."),
        }


# ─────────────────────────────────────────────────────────────────
# LIVE COMPARATOR  (Boyd-enhanced)
# ─────────────────────────────────────────────────────────────────

class LiveComparator:
    """
    Original comparator plus:
    - Ps computed and included in every prediction
    - Ps alerts: warn when aircraft enters energy deficit
    - Ps stoop monitor: trigger pullout at zero crossing
    - EM map accumulation
    """

    THRESHOLDS = {
        'airspeed_ms':    0.8,
        'current_A':      3.0,
        'power_W':        8.0,
        'climb_rate_ms':  0.4,
        'stall_margin_ms':1.0,
        'Ps_warn':       PS_WARN_NEGATIVE,
        'Ps_alert':      PS_ALERT_NEGATIVE,
    }

    def __init__(self):
        self.model  = PerformanceModel()
        self.em_map = EMMap()
        self.deltas = []
        self._sample_n          = 0
        self._sum_current_err   = 0.0
        self._sum_power_err     = 0.0
        self.ct_measurements    = []
        self._Ps_samples        = []
        self._Ps_negative_count = 0
        self._in_stoop          = False

    def compare(self, t, freq_hz, V_ms, sweep_deg_mean,
                amplitude_deg, throttle_frac,
                measured_current_A, measured_power_W,
                measured_climb_ms, agl_m,
                in_stoop=False, bank_deg=0.0):
        """
        Returns dict with 'predicted', 'measured', 'alerts',
        'Ps', 'pullout_recommended'.
        """
        pred = self.model.predict(
            freq_hz, V_ms, sweep_deg_mean,
            amplitude_deg, throttle_frac,
            bank_deg=bank_deg, h_agl_m=agl_m)

        measured = {
            'airspeed_ms':     round(V_ms, 2),
            'current_A_total': round(measured_current_A, 2),
            'power_W':         round(measured_power_W, 1),
            'climb_rate_ms':   round(measured_climb_ms, 2),
            'agl_m':           round(agl_m, 2),
        }

        # ── CT measurement ────────────────────────────────────
        if V_ms > 1.0 and measured_power_W > 0.5:
            thrust_est = measured_power_W * 0.85 / V_ms
            q    = 0.5 * RHO * V_ms**2
            S_eff= max(0.01, S_TOTAL_M2 * math.cos(
                math.radians(max(0, sweep_deg_mean - 60.0))))
            CT_m = thrust_est / (q * S_eff) if q * S_eff > 0.01 else 0
            if 0.01 < CT_m < 1.0:
                self.ct_measurements.append(round(CT_m, 4))

        # ── BOYD: accumulate EM map ───────────────────────────
        Ps = pred['Ps']
        self.em_map.record(freq_hz, sweep_deg_mean, V_ms, Ps)
        # Energy height tracking (FAA Ch.4 + Kershner)
        ES = pred.get('energy_height_m', 0.0)
        if not hasattr(self, '_es_prev'): self._es_prev = ES
        if not hasattr(self, '_es_t_prev'): self._es_t_prev = t
        dt_es = t - self._es_t_prev
        es_rate = (ES - self._es_prev) / dt_es if dt_es > 0.1 else 0.0
        self._es_prev = ES; self._es_t_prev = t
        pred['energy_height_rate_ms'] = round(es_rate, 3)
        self._Ps_samples.append(Ps)
        if Ps < 0:
            self._Ps_negative_count += 1

        # ── BOYD: stoop pullout monitor ───────────────────────
        self._in_stoop = in_stoop
        pull_out, pull_reason = ps_pullout_recommended(
            Ps, agl_m, V_ms, pred['stall_margin_ms'])

        # ── Deltas ────────────────────────────────────────────
        delta_I    = measured_current_A - pred['current_A_total']
        delta_P    = measured_power_W   - pred['power_W']
        delta_clmb = measured_climb_ms  - pred['climb_rate_ms']

        self._sample_n        += 1
        self._sum_current_err += abs(delta_I)
        self._sum_power_err   += abs(delta_P)

        # ── Alerts ────────────────────────────────────────────
        alerts = []

        if pred['stall_margin_ms'] < 1.0 and V_ms > 0.5:
            alerts.append(
                f"STALL PROXIMITY: {pred['stall_margin_ms']:.1f}m/s margin")

        if abs(delta_I) > self.THRESHOLDS['current_A']:
            direction = "HIGH" if delta_I > 0 else "LOW"
            alerts.append(
                f"CURRENT {direction}: measured {measured_current_A:.1f}A "
                f"vs predicted {pred['current_A_total']:.1f}A "
                f"(Δ{delta_I:+.1f}A)")

        if abs(delta_P) > self.THRESHOLDS['power_W']:
            alerts.append(
                f"POWER MISMATCH: measured {measured_power_W:.0f}W "
                f"vs predicted {pred['power_W']:.0f}W "
                f"(Δ{delta_P:+.0f}W)")

        if abs(delta_clmb) > self.THRESHOLDS['climb_rate_ms']:
            alerts.append(
                f"CLIMB RATE MISMATCH: measured {measured_climb_ms:.2f}m/s "
                f"vs predicted {pred['climb_rate_ms']:.2f}m/s "
                f"(Δ{delta_clmb:+.2f}m/s)")

        if not pred['thermal_ok']:
            alerts.append(
                f"THERMAL: {pred['heat_W_each']:.1f}W per motor")

        # ── BOYD: Ps alerts ───────────────────────────────────
        # Energy height rate alert (FAA Ch.4)
        es_rate = pred.get('energy_height_rate_ms', 0.0)
        if es_rate < -1.0 and not in_stoop and V_ms > 2.0:
            alerts.append(
                f"ENERGY HEIGHT DROPPING: {es_rate:.2f} m/s "
                f"(ES={pred.get('energy_height_m',0):.0f}m) — "
                f"aircraft spending stored energy")

        # Load-factor stall alert
        stall_banked = pred.get('stall_margin_banked_ms', pred['stall_margin_ms'])
        if stall_banked < 1.0 and abs(bank_deg) > 20:
            alerts.append(
                f"BANKED STALL PROXIMITY: margin={stall_banked:.1f}m/s "
                f"at {bank_deg:.0f}° bank (Vs_turn={pred.get('stall_speed_banked_ms', 0):.1f}m/s)")

        if not in_stoop:
            if Ps < self.THRESHOLDS['Ps_alert']:
                alerts.append(
                    f"ENERGY DEFICIT Ps={Ps:.2f} m/s — "
                    f"aircraft spending energy it cannot sustain")
            elif Ps < self.THRESHOLDS['Ps_warn']:
                alerts.append(
                    f"ENERGY LOW Ps={Ps:.2f} m/s — "
                    f"marginal surplus, watch altitude")

        # ── BOYD: stoop pullout alert ─────────────────────────
        if in_stoop and pull_out:
            alerts.append(f"PULLOUT: {pull_reason}")

        if alerts:
            self.deltas.append({
                't':          round(t, 1),
                'freq':       round(freq_hz, 2),
                'V_ms':       round(V_ms, 2),
                'sweep':      round(sweep_deg_mean, 0),
                'Ps':         round(Ps, 3),
                'delta_I':    round(delta_I, 2),
                'delta_P':    round(delta_P, 1),
                'delta_clmb': round(delta_clmb, 3),
                'alerts':     alerts,
            })

        return {
            'predicted':           pred,
            'measured':            measured,
            'alerts':              alerts,
            'Ps':                  Ps,
            'Ps_status':           pred['Ps_status'],
            'pullout_recommended': pull_out,
            'pullout_reason':      pull_reason,
        }

    # ── Statistics ────────────────────────────────────────────────
    def mean_ct(self):
        if not self.ct_measurements: return None
        return round(sum(self.ct_measurements)/len(self.ct_measurements), 4)

    def mean_current_error(self):
        if self._sample_n == 0: return 0.0
        return round(self._sum_current_err / self._sample_n, 3)

    def mean_power_error(self):
        if self._sample_n == 0: return 0.0
        return round(self._sum_power_err / self._sample_n, 3)

    def mean_Ps(self):
        if not self._Ps_samples: return None
        return round(sum(self._Ps_samples)/len(self._Ps_samples), 3)

    def Ps_negative_fraction(self):
        if self._sample_n == 0: return 0.0
        return round(self._Ps_negative_count / self._sample_n, 3)


# ─────────────────────────────────────────────────────────────────
# DELTA REPORT  (Boyd-enhanced)
# ─────────────────────────────────────────────────────────────────

class DeltaReport:

    def __init__(self, comparator: LiveComparator):
        self.comp = comparator

    def write(self, flight_number, output_dir, wave_lib_version):
        os.makedirs(output_dir, exist_ok=True)
        now_str = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M")
        fname   = f"flight_{flight_number:03d}_{now_str}_delta.txt"
        fpath   = os.path.join(output_dir, fname)

        ct_mean   = self.comp.mean_ct()
        mean_I    = self.comp.mean_current_error()
        mean_P    = self.comp.mean_power_error()
        mean_Ps   = self.comp.mean_Ps()
        neg_frac  = self.comp.Ps_negative_fraction()

        stall_flags   = [d for d in self.comp.deltas
                         if any('STALL'   in a for a in d['alerts'])]
        current_flags = [d for d in self.comp.deltas
                         if any('CURRENT' in a for a in d['alerts'])]
        power_flags   = [d for d in self.comp.deltas
                         if any('POWER'   in a for a in d['alerts'])]
        climb_flags   = [d for d in self.comp.deltas
                         if any('CLIMB'   in a for a in d['alerts'])]
        ps_flags      = [d for d in self.comp.deltas
                         if any('ENERGY'  in a for a in d['alerts'])]
        pullout_flags = [d for d in self.comp.deltas
                         if any('PULLOUT' in a for a in d['alerts'])]

        lines = [
            "=" * 60,
            "MCO-1D PERFORMANCE DELTA REPORT  (Boyd-enhanced)",
            f"Flight {flight_number}  |  wave_library {wave_lib_version}",
            "=" * 60,
            "",
            "MEAN ERRORS ACROSS FLIGHT",
            f"  Current error (mean):   {mean_I:.3f} A",
            f"  Power error (mean):     {mean_P:.1f} W",
            f"  CT measured (mean):     {ct_mean if ct_mean else 'insufficient'}",
            "",
            "─" * 60,
            "BOYD ENERGY-MANEUVERABILITY ANALYSIS",
            "─" * 60,
            f"  Mean Ps across flight:  {mean_Ps:+.3f} m/s" if mean_Ps else
            f"  Mean Ps across flight:  no data",
            f"  Ps negative fraction:   {neg_frac*100:.1f}% of flight "
            f"(ideal < 10% for sustained cruise)",
            f"  Ps flagged events:      {len(ps_flags)}",
            "",
        ]

        # Ps interpretation
        if mean_Ps is not None:
            if mean_Ps > 1.0:
                lines += [
                    "  STATUS: Healthy energy surplus.",
                    "  Aircraft has reserve for manoeuvre and gusts.",
                    "  Optimiser can explore higher frequency or tighter sweep.",
                ]
            elif mean_Ps > 0.0:
                lines += [
                    "  STATUS: Marginal surplus — near maximum sustained performance.",
                    "  Aircraft is operating close to its Ps=0 boundary.",
                    "  Current configuration is near-optimal for this flight.",
                ]
            elif mean_Ps > -0.5:
                lines += [
                    "  STATUS: Slightly negative — aircraft spending energy slowly.",
                    "  Acceptable for short periods (turns, climbs).",
                    "  RECOMMENDATION: Slightly reduce frequency or sweep for cruise.",
                ]
            else:
                lines += [
                    "  STATUS: ** SUSTAINED ENERGY DEFICIT **",
                    "  Aircraft cannot maintain this configuration indefinitely.",
                    "  RECOMMENDATION: Reduce freq or sweep — see EM map below.",
                ]
            lines.append("")

        # EM Map
        lines += ["  E-M DIAGRAM (measured Ps by operating point):"]
        lines += ["  " + "-"*50]
        lines += self.comp.em_map.summary()
        lines += [""]

        # Best config
        best = self.comp.em_map.best_config_recommendation()
        if best:
            lines += [
                "WAVE LIBRARY RECOMMENDATION FROM EM MAP",
                f"  Best sustained operating point found this flight:",
                f"  Frequency: {best['CRUISE_FREQ_HZ']} Hz",
                f"  Sweep:     {best['CRUISE_SWEEP_DEG']}°",
                f"  Mean Ps:   {best['mean_Ps_ms']:+.3f} m/s",
                f"  → Consider updating CRUISE_FREQ_HZ and adding this",
                f"    as the default cruise point in the sweep schedule.",
                "",
            ]

        # Pullout events
        if pullout_flags:
            lines += [
                "STOOP PULLOUT EVENTS (Boyd Ps trigger)",
                f"  {len(pullout_flags)} pullout recommendation(s) fired:",
            ]
            for d in pullout_flags[:5]:
                lines.append(
                    f"  t={d['t']}s  V={d['V_ms']}m/s  "
                    f"Ps={d['Ps']:+.3f}m/s  AGL from context")
            lines.append("")

        # CT
        if ct_mean:
            lines += ["─"*60, "CT ANALYSIS", "─"*60]
            lines += [
                f"  Measured CT:           {ct_mean:.4f}",
                f"  Model CT (nominal):    0.1500",
                f"  Model CT (optimistic): 0.2500",
            ]
            if   ct_mean < 0.08:
                lines += ["  STATUS: ** BELOW CONSERVATIVE — check rope tension **",
                          "  CHECK: FLAP_AMP_DEG, rope seated in pulleys"]
            elif ct_mean < 0.15:
                lines += ["  STATUS: Below conservative — sprint speed limited",
                          "  RECOMMENDATION: Increase CF1_DEFAULT_FREQ_HZ by 0.3 Hz"]
            elif ct_mean < 0.20:
                lines += ["  STATUS: Conservative range — model valid"]
            elif ct_mean < 0.25:
                lines += ["  STATUS: Approaching optimistic — sprint may be achievable"]
            else:
                lines += ["  STATUS: At or above optimistic — excellent performance"]
            lines.append("")

        # Standard divergences
        lines += ["─"*60, "DIVERGENCE LOG", "─"*60,
                  f"  Total flagged events: {len(self.comp.deltas)}", ""]

        for flag_list, label, extra in [
            (current_flags, "CURRENT", True),
            (power_flags,   "POWER",   False),
            (stall_flags,   "STALL PROXIMITY", False),
            (climb_flags,   "CLIMB RATE", True),
        ]:
            if not flag_list: continue
            avg = (sum(d.get('delta_I' if 'CURRENT' in label else
                             'delta_clmb', 0) for d in flag_list)
                   / len(flag_list))
            lines += [f"  {label} EVENTS: {len(flag_list)}  avg Δ={avg:+.2f}"]
            lines += [f"    First: t={flag_list[0]['t']}s  "
                      f"V={flag_list[0]['V_ms']}m/s  "
                      f"sweep={flag_list[0]['sweep']:.0f}°  "
                      f"Ps={flag_list[0]['Ps']:+.3f}m/s"]
            lines.append("")

        # Recommendations
        lines += ["="*60,
                  "PARAMETER TUNING RECOMMENDATIONS",
                  "="*60, ""]

        recs = []
        if ct_mean and ct_mean < 0.12:
            recs.append("1. Low CT — check rope tension.\n"
                        "   If mechanical OK: increase FLAP_AMP_DEG 80→82.")
        if mean_I > 2.5:
            recs.append("2. High current — drag above model.\n"
                        "   Check rib profile accuracy and membrane tension.")
        if mean_I < -2.0:
            recs.append("2. Low current — wings not reaching amplitude.\n"
                        "   Check rope tension and ODrive S1 vel_limit (mco_odrive_setup.py --verify).")
        if len(stall_flags) > 3:
            recs.append("3. Multiple stall events.\n"
                        "   Increase STALL_V_FLOOR_MS: change 1.10 → 1.15.")
        if mean_Ps is not None and mean_Ps < -0.5:
            recs.append(f"4. Sustained Ps deficit ({mean_Ps:+.3f}m/s).\n"
                        f"   Current config exceeds sustainable envelope.\n"
                        f"   Use EM map best point: "
                        f"{best['CRUISE_FREQ_HZ']}Hz/{best['CRUISE_SWEEP_DEG']}° "
                        f"(Ps={best['mean_Ps_ms']:+.3f}m/s)."
                        if best else
                        f"4. Sustained Ps deficit. Reduce frequency by 0.2Hz.")
        if not recs:
            recs.append("No specific recommendations — flight matched model well.\n"
                        "If sprint speed below target: increase freq mapping 2.6→2.7.")

        for r in recs:
            for line in r.split('\n'):
                lines.append(f"  {line}")
            lines.append("")

        lines += [
            "="*60,
            "PASTE TO CLAUDE FOR PARAMETER UPDATES",
            "="*60, "",
            "  Send: this file + flight summary + wave_library.py",
            "       + project_state_template.txt",
            "  Request: 'Update wave library based on this flight data'",
            "",
        ]

        try:
            with open(fpath, 'w') as f:
                f.write('\n'.join(lines) + '\n')
            print(f"[DeltaReport] Saved: {fpath}")
            return fpath
        except Exception as e:
            print(f"[DeltaReport] Failed: {e}")
            return None
