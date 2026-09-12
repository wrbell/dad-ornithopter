"""
MCO-1D Flight Recorder
========================
Tracks flight statistics and writes the plain-text summary
on disarm. Imported by main.py — do not run directly.
"""
import os, datetime

SUMMARY_DIR = "/home/pi/mco/flight_logs"


class FlightRecorder:
    """
    Tracks flight statistics every loop iteration.
    Call .arm() when the aircraft arms, .update() every loop,
    .disarm() on landing.  Call .write_summary() to save the file.
    """

    def __init__(self):
        self.reset()

    def reset(self):
        self.armed_time       = None
        self.disarm_time      = None
        self.arm_wall_time    = None   # real-world datetime

        # Airspeed
        self.v_max_ms         = 0.0
        self.v_samples        = []    # (t, V_ms) every second

        # Altitude
        self.agl_min_m        = 9999.0
        self.agl_max_m        = 0.0

        # Battery
        self.v_batt_start     = None
        self.v_batt_min       = 9999.0
        self.v_batt_samples   = []    # (t, V)
        self.mah_used         = 0.0
        self.amps_peak        = 0.0

        # Modes and sweep
        self.modes_seen       = set()
        self.sweep_transitions= 0
        self.sweep_max_seen   = 60.0
        self._last_sweep      = 60.0

        # CF-1 bursts
        self.cf1_bursts       = 0
        self.cf1_v_gains      = []    # airspeed gain per burst m/s
        self._cf1_v_before    = None

        # Anomalies
        self.anomalies        = []

        # Servo timing  (for lag detection)
        self._servo_cmd_times = {}    # wing_idx -> last command time

        # Phase offset in use
        self.phase_offsets_used = set()

        # Aerodynamic regime (Rev O3-P-Q)
        self.k_min                 = 999.0
        self.k_max                 = 0.0
        self.k_mean                = 0.0
        self._k_sum                = 0.0
        self._k_n                  = 0
        self.J_min                 = 999.0
        self.J_max                 = 0.0
        self.lev_dominated_samples = 0
        self.quasi_steady_samples  = 0
        self.ct_cal_valid_samples  = 0
        self.regime_counts         = {
            'quasi_steady':0,'unsteady':0,'lev_dominated':0}

        # Frequency range
        self.freq_min         = 9999.0
        self.freq_max         = 0.0

        # Loop statistics
        self._loop_overruns   = 0
        self._loops_total     = 0
        self._last_vsample_t  = 0.0
        self._last_bsample_t  = 0.0

    # ── ARM / DISARM ─────────────────────────────────────────────
    def arm(self, t_elapsed, v_batt):
        self.armed_time    = t_elapsed
        self.arm_wall_time = datetime.datetime.now()
        self.v_batt_start  = v_batt
        self.modes_seen.add("ground_roll")

    def disarm(self, t_elapsed):
        self.disarm_time = t_elapsed

    # ── UPDATE (called every loop) ───────────────────────────────
    def update_aero_regime(self, freq_hz, V_ms):
        """Call each loop to log k, J, aerodynamic regime."""
        try:
            import mco_wave_library as _wl
            r = _wl.aero_regime(freq_hz, V_ms)
            k = r['k']; J = r['J']
            if k < self.k_min: self.k_min = k
            if k > self.k_max: self.k_max = k
            if J < self.J_min: self.J_min = J
            if J > self.J_max: self.J_max = J
            self._k_sum += k; self._k_n += 1
            self.k_mean = self._k_sum / self._k_n
            self.regime_counts[r['regime']] = (
                self.regime_counts.get(r['regime'],0) + 1)
            if k > 0.20: self.lev_dominated_samples += 1
            if k < 0.05: self.quasi_steady_samples  += 1
            if r['ct_cal_ok']: self.ct_cal_valid_samples += 1
        except Exception:
            pass

    def update(self, t, V_ms, agl_m, freq_hz, sweep_angles,
               mode_name, v_batt, amps, cf1_active, loop_overrun):

        if self.armed_time is None:
            return   # not armed yet — don't record

        self._loops_total += 1
        if loop_overrun:
            self._loop_overruns += 1

        # Airspeed
        if V_ms > self.v_max_ms:
            self.v_max_ms = V_ms
        if t - self._last_vsample_t >= 1.0:
            self.v_samples.append((round(t, 1), round(V_ms, 2)))
            self._last_vsample_t = t

        # AGL
        if 0.0 < agl_m < self.agl_min_m:
            self.agl_min_m = agl_m
        if agl_m > self.agl_max_m:
            self.agl_max_m = agl_m

        # Battery
        if v_batt < self.v_batt_min:
            self.v_batt_min = v_batt
        if amps > self.amps_peak:
            self.amps_peak = amps
        self.mah_used += amps * DT / 3.6      # A * s → mAh
        if t - self._last_bsample_t >= 5.0:
            self.v_batt_samples.append((round(t, 1), round(v_batt, 2)))
            self._last_bsample_t = t

        # Modes
        self.modes_seen.add(mode_name)

        # Frequency range
        if freq_hz > 0.1:
            if freq_hz < self.freq_min: self.freq_min = freq_hz
            if freq_hz > self.freq_max: self.freq_max = freq_hz

        # Sweep transitions and max
        mean_sweep = sum(sweep_angles) / len(sweep_angles)
        if mean_sweep > self.sweep_max_seen:
            self.sweep_max_seen = mean_sweep
        if abs(mean_sweep - self._last_sweep) > 15.0:
            self.sweep_transitions += 1
        self._last_sweep = mean_sweep

        # CF-1 burst tracking
        if cf1_active and self._cf1_v_before is None:
            self._cf1_v_before = V_ms
            self.cf1_bursts += 1
        elif not cf1_active and self._cf1_v_before is not None:
            gain = V_ms - self._cf1_v_before
            self.cf1_v_gains.append(round(gain, 3))
            self._cf1_v_before = None

        # Low battery warning
        if v_batt < 19.8 and v_batt > 0:
            if not any("Low battery" in a for a in self.anomalies):
                self.anomalies.append(
                    f"t={t:.0f}s  Low battery: {v_batt:.1f}V")

        # High loop overrun rate warning
        if self._loops_total > 200:
            overrun_pct = self._loop_overruns / self._loops_total * 100
            if overrun_pct > 5.0:
                if not any("Loop overrun" in a for a in self.anomalies):
                    self.anomalies.append(
                        f"Loop overruns: {overrun_pct:.1f}% of loops "
                        f"({self._loop_overruns}/{self._loops_total})")

    def log_anomaly(self, t, message):
        """Call this from flight loop for any specific detected fault."""
        self.anomalies.append(f"t={t:.1f}s  {message}")

    # ── WRITE SUMMARY ────────────────────────────────────────────
    def write_summary(self, flight_number, wave_lib_version="unknown",
                      notes="", output_dir=SUMMARY_DIR):
        """
        Write the flight summary text file.
        Returns the file path on success, None on failure.
        """
        os.makedirs(output_dir, exist_ok=True)

        if self.armed_time is None:
            print("[Recorder] No armed flight data — summary not written.")
            return None

        dt_str = self.arm_wall_time.strftime("%Y-%m-%d_%H-%M") \
                 if self.arm_wall_time else "unknown-time"
        fname = f"flight_{flight_number:03d}_{dt_str}.txt"
        fpath = os.path.join(output_dir, fname)

        duration = (self.disarm_time or 0.0) - self.armed_time
        mm = int(duration // 60)
        ss = int(duration % 60)

        # CF-1 stats
        if self.cf1_v_gains:
            cf1_avg_gain = sum(self.cf1_v_gains) / len(self.cf1_v_gains)
            cf1_line = (f"{self.cf1_bursts} burst(s)  "
                        f"avg speed gain {cf1_avg_gain:+.2f} m/s  "
                        f"individual: {self.cf1_v_gains}")
        else:
            cf1_line = "0 bursts"

        # Battery summary
        batt_delta = 0.0
        if self.v_batt_start and self.v_batt_min < 9999:
            batt_delta = self.v_batt_start - self.v_batt_min

        # Voltage profile (sample every 30s for brevity)
        batt_profile = [
            f"{t:.0f}s:{v:.1f}V"
            for i, (t, v) in enumerate(self.v_batt_samples)
            if i % 6 == 0
        ]

        # Speed profile (sample every 10s for brevity)
        speed_profile = [
            f"{t:.0f}s:{v:.1f}m/s"
            for i, (t, v) in enumerate(self.v_samples)
            if i % 10 == 0
        ]

        lines = [
            "=" * 60,
            f"MCO-1D FLIGHT SUMMARY",
            f"Flight number : {flight_number}",
            f"Date/time     : {self.arm_wall_time.strftime('%Y-%m-%d  %H:%M:%S') if self.arm_wall_time else 'unknown'}",
            f"wave_library  : version {wave_lib_version}",
            "=" * 60,
            "",
            "DURATION",
            f"  {mm}m {ss:02d}s  ({duration:.1f} seconds total)",
            "",
            "AIRSPEED",
            f"  Peak:   {self.v_max_ms:.2f} m/s  ({self.v_max_ms*1.944:.1f} kt)  ({self.v_max_ms*3.6:.1f} km/h)",
            f"  Freq range flown:  {self.freq_min:.2f} Hz  to  {self.freq_max:.2f} Hz",
            f"  Speed profile (every 10s):  {', '.join(speed_profile)}",
            "",
            "ALTITUDE",
            f"  Min AGL: {self.agl_min_m:.2f} m  (ground proximity)",
            f"  Max AGL: {self.agl_max_m:.2f} m",
            "",
            "SWEEP",
            f"  Max sweep angle seen: {self.sweep_max_seen:.1f} deg",
            f"  Sweep transitions (>15 deg step): {self.sweep_transitions}",
            "",
            "BATTERY",
            f"  Start voltage:  {self.v_batt_start:.2f} V" if self.v_batt_start else "  Start voltage:  not recorded",
            f"  Min voltage:    {self.v_batt_min:.2f} V",
            f"  Voltage drop:   {batt_delta:.2f} V",
            f"  mAh consumed:   {self.mah_used:.0f} mAh",
            f"  Peak current:   {self.amps_peak:.1f} A",
            f"  Voltage profile (every 30s):  {', '.join(batt_profile)}",
            "",
            "WAVE LIBRARY MODES ACTIVATED",
            f"  {', '.join(sorted(self.modes_seen))}",
            "",
            "CF-1 CLAP-AND-FLING",
            f"  {cf1_line}",
            "",
            "ANOMALIES",
        ]

        if self.anomalies:
            for a in self.anomalies:
                lines.append(f"  ** {a}")
        else:
            lines.append("  None detected")

        lines += [
            "",
            "PILOT / OBSERVER NOTES",
            f"  {notes if notes else '(none recorded — add before pasting to Claude)'}",
            "",
            "=" * 60,
            "HOW TO USE THIS FILE",
            "  1. Add your own observations under PILOT / OBSERVER NOTES above.",
            "  2. Open a conversation with Claude.",
            "  3. Paste the project state summary (from your project_state.txt).",
            "  4. Upload or paste this file.",
            "  5. Upload the current wave_library.py.",
            "  6. Describe what you want to tune or add.",
            "  Claude will analyse the numbers and return updated files.",
            "=" * 60,
        ]

        try:
            with open(fpath, "w") as f:
                f.write("\n".join(lines) + "\n")
            print(f"[Recorder] Summary saved: {fpath}")
            return fpath
        except Exception as e:
            print(f"[Recorder] Failed to write summary: {e}")
            return None


# ─────────────────────────────────────────────────────────────────
# MAIN FLIGHT LOOP
# ─────────────────────────────────────────────────────────────────
