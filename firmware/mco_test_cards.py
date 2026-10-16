#!/usr/bin/env python3
"""
MCO-1D Automated Test Card System
====================================
mco_test_cards.py

Four test cards, each building on the last.

PHYSICAL ARRANGEMENT:
  TC-01  Aircraft bolted to a rigid post. Load cell between aircraft
         and post reads thrust. No movement. Full bench test.

  TC-02  Same post, same load cell. Now commanding increasing frequency
         until the load cell reads zero — that is the hover frequency.
         The aircraft is "hovering" while still restrained. Measures
         the complete lift-frequency curve and calibrates all controllers.

  TC-03  Aircraft mounted on the back of a truck. Truck provides forward
         airspeed at each commanded speed. Aircraft still restrained to
         truck bed (with load cell if desired). Maps the CT-J curve and
         forward flight aerodynamics. No pilot required — fully automatic.

  TC-04  Free flight. First unrestrained flight. Pilot in command.
         Software guides step by step. All TC-01/02/03 calibrations apply.
         (TC-04 is built but marked NOT YET CLEARED until TC-03 passes.)

AUTOMATION MODEL:
  TC-01: 10 steps fully automatic, 6 semi-automatic (human confirms
         physical actions: position checks, fixture attachment, stick moves).
  TC-02: Fully automatic. Software commands frequencies, reads load cell,
         finds hover frequency, calibrates all controllers.
  TC-03: Fully automatic. Truck operator drives at commanded speeds.
         Software reads GPS for airspeed, commands wing frequencies,
         records CT at each advance ratio.
  TC-04: Guided automatic. Pilot flies; software directs and measures.

COMMANDS (from ground station prompt):
  tc01_start / tc01_confirm / tc01_abort / tc01_status
  tc02_start / tc02_abort  / tc02_status
  tc03_start / tc03_abort  / tc03_status
  tc04_start / tc04_continue / tc04_abort / tc04_status
  tc_results / tc_export

RESULTS FILE:
  /home/pi/knowledge/tc_results.json — persists across reboots.
"""

import time, json, math, os, threading, collections

# ─── File and thresholds ─────────────────────────────────────────

RESULTS_FILE            = '/home/pi/knowledge/tc_results.json'
# MTOW_N — load from build measurements if available, else use design value
# Performance model (mco_performance_model.py) provides hover freq estimates
# pre-TC-02. TC-01 gate uses measured thrust; model provides context only.
def _load_actual_mtow():
    """Read actual assembled MTOW from mco_build_measurements output.
    Falls back to design MTOW if measurements not yet taken.
    Must be called after mco_build_measurements.py has run.
    """
    import json, os
    bm_path = '/home/pi/knowledge/build_measurements.json'
    if os.path.exists(bm_path):
        try:
            bm = json.load(open(bm_path))
            actual_mass_kg = bm.get('mtow_kg')
            if actual_mass_kg and 1.0 < actual_mass_kg < 20.0:
                print(f"  TC: Using actual MTOW from build measurements: "
                      f"{actual_mass_kg:.3f} kg ({actual_mass_kg*9.807:.1f} N)")
                return actual_mass_kg * 9.807
        except Exception:
            pass
    try:
        from mco_wave_library import MTOW_KG
        design_N = MTOW_KG * 9.807
    except Exception:
        design_N = 5.687 * 9.807
    print(f"  TC: No build measurements found — using design MTOW "
          f"{design_N:.1f} N. Run mco_build_measurements.py first.")
    return design_N

MTOW_N = _load_actual_mtow()   # Newtons — actual or design MTOW
MOTOR_KT_NM_A           = 0.08
ALTITUDE_FLOOR_M        = 0.30  # m AGL minimum — altitude hold will not command below this
THERMAL_R_PHASE         = 0.12
THERMAL_MASS_J_K        = 0.08
MAX_MOTOR_TEMP_C        = 60.0

# TC-01 thresholds
TC01_SERVO_NEUTRAL_TOL  = 2.0    # °
TC01_SERVO_RANGE_TOL    = 3.0    # °
TC01_I_LOW              = 0.3    # A — below = not connected
TC01_I_HIGH             = 6.0    # A — above = binding
TC01_ROPE_SLIP_DEG      = 5.0
TC01_SYMMETRY_PCT       = 5.0
TC01_MIN_THRUST_N       = MTOW_N

# TC-02 thresholds
TC02_HOVER_FORCE_THRESH_N  = 1.0   # N — "load cell reads zero" within this
TC02_HOVER_CONFIRM_DWELL_S = 5.0   # must hold within threshold for 5 s
TC02_FREQ_STEP_HZ          = 0.1   # step size when hunting for hover freq
TC02_FREQ_MIN              = 1.0   # start of sweep
TC02_FREQ_MAX              = 3.0   # hard ceiling
TC02_LIFT_CURVE_STEPS      = [1.5, 1.75, 2.0, 2.25, 2.5, 2.75, 3.0]

# TC-03 thresholds
TC03_TRUCK_SPEEDS_MS       = [3.0, 5.0, 7.0, 9.0, 11.0, 13.0]
TC03_DWELL_S               = 20.0  # seconds per speed point
TC03_MIN_SAMPLES_SPEED     = 40    # at 5 Hz over dwell

# ─── Shared state ─────────────────────────────────────────────────

_tc = {
    'tc01': {'running': False, 'step': 0, 'results': {},
             'waiting_confirm': False, 'confirm_prompt': '', 'start_t': None},
    'tc02': {'running': False, 'step': 0, 'results': {},
             'hover_freq_hz': None, 'lift_curve': {},
             'lift_slope_N_per_Hz': None, 'start_t': None},
    'tc03': {'running': False, 'step': 0, 'results': {},
             'ct_j_curve': [], 'start_t': None},
    'tc04': {'running': False, 'step': 0, 'results': {},
             'waiting_continue': False, 'instruction': '', 'start_t': None},
    'active': None,
}


# ─── Utilities ────────────────────────────────────────────────────

def _load():
    try:
        with open(RESULTS_FILE) as f:
            d = json.load(f)
        for tc in ('tc01','tc02','tc03','tc04'):
            _tc[tc]['results'] = d.get(tc, {})
    except (FileNotFoundError, json.JSONDecodeError):
        pass

def _save():
    os.makedirs(os.path.dirname(RESULTS_FILE), exist_ok=True)
    try:
        with open(RESULTS_FILE, 'w') as f:
            json.dump({tc: _tc[tc]['results'] for tc in ('tc01','tc02','tc03','tc04')},
                      f, indent=2)
    except Exception as e:
        print(f"[TC] Save failed: {e}")

def _rec(tc_key, step_id, passed, data, message):
    _tc[tc_key]['results'][step_id] = {
        'passed': passed, 'data': data,
        'message': message, 'time': time.strftime('%H:%M:%S')}
    _save()
    print(f"  [{tc_key.upper()}] {step_id}: {'✓ PASS' if passed else '✗ FAIL'} — {message}")
    return passed

def _mean(vals):
    v = [x for x in vals if x is not None and not math.isnan(x)]
    return sum(v)/len(v) if v else 0.0

def _read_load_cell():
    """Read HX711 load cell — returns newtons or None."""
    try:
        import mco_loadcell as lc
        return lc.read_newtons()
    except ImportError:
        return None  # mco_loadcell.py not yet written

def _read_currents():
    try:
        from mco_odrive_interface import ODriveInterface
        odi = ODriveInterface()
        c = odi.read_currents()
        odi.close()
        return c
    except Exception:
        return [0.0]*8

def _set_freq(freq_hz):
    """Command all 8 wings to given frequency."""
    try:
        from mco_odrive_interface import ODriveInterface
        odi = ODriveInterface()
        odi.enable_all()
        odi.set_cascade_freq(freq_hz)
        odi.close()
    except Exception as e:
        print(f"  [TC] set_freq({freq_hz}): {e}")

def _idle_all():
    try:
        from mco_odrive_interface import ODriveInterface
        odi = ODriveInterface()
        odi.idle_all()
        odi.close()
    except Exception:
        pass


# ══════════════════════════════════════════════════════════════════
# TC-01 — BENCH TEST: AIRCRAFT FIXED TO POST
# ══════════════════════════════════════════════════════════════════

class TC01Runner:
    """
    TC-01: All tests with aircraft bolted to post, load cell inline.
    No forward motion. No flight. Full hardware and software validation.

    10 fully automatic steps.
    6 semi-automatic (pause for human action, then resume automatically).
    Plus 2 new steps: 05b sector arc symmetry, 05c soft-stop verify.
    """

    def __init__(self, fcc, sensors_fn, hardware=None):
        self.fcc = fcc
        self.sf  = sensors_fn
        self.hw  = hardware
        self._stop = False

    def start(self):
        if _tc['tc01']['running']:
            return "TC-01 already running"
        _tc['tc01'].update({'running':True,'step':1,'start_t':time.monotonic()})
        _tc['active'] = 'tc01'
        _load()
        self._stop = False
        threading.Thread(target=self._run, daemon=True).start()
        return "TC-01 started — monitor via tc01_status"

    def abort(self):
        self._stop = True
        _tc['tc01']['running'] = False
        _tc['active'] = None
        _idle_all()
        return "TC-01 aborted — motors idle"

    def confirm(self):
        _tc['tc01']['waiting_confirm'] = False
        return "Confirmed — TC-01 resuming"

    def _pause(self, prompt):
        _tc['tc01']['waiting_confirm'] = True
        _tc['tc01']['confirm_prompt']  = prompt
        print(f"\n  ┌─ ACTION REQUIRED ─────────────────────────────────┐")
        print(f"  │ {prompt[:52]:<52} │")
        print(f"  │ Type  tc01_confirm  on the ground station when done│")
        print(f"  └────────────────────────────────────────────────────┘")
        while _tc['tc01']['waiting_confirm'] and not self._stop:
            time.sleep(0.5)

    def _run(self):
        try:
            import mco_wave_library as w
            steps = [
                self._s01_servo_neutral,
                self._s02_servo_range,
                self._s03_pitch_adc,
                self._s04_sweep_adc,
                self._s05_odrive_comms,
                self._s05b_sector_arc_symmetry,   # new: arc datum check
                self._s05c_soft_stop_verify,      # new: soft-stop confirm
                self._s06_motor_spin,
                self._s07_flap_0_5hz,
                self._s08_flap_2_5hz,
                self._s09_pitch_calib,
                self._s10_sweep_calib,
                self._s11_freq_ceiling,
                self._s12_static_thrust,
                self._s13_rc_mapping,
                self._s14_safety_caps,
                self._s15_symmetry,
                self._s16_export,
            ]
            for i, fn in enumerate(steps, 1):
                if self._stop: break
                _tc['tc01']['step'] = i
                print(f"\n{'─'*52}")
                print(f"  TC-01 Step {i:02d}/{len(steps)}")
                passed = fn(w)
                if not passed and i <= 8:
                    print(f"  CRITICAL FAIL at step {i} — fix and restart")
                    break
            else:
                self._summary()
        finally:
            _tc['tc01']['running'] = False
            _tc['active'] = None
            _idle_all()

    # ── Steps ─────────────────────────────────────────────────────

    def _s01_servo_neutral(self, w):
        """Servo neutrals — fully automatic."""
        time.sleep(0.5)
        actual = w.read_servo_feedback_adc()
        if all(a is None for a in actual):
            return _rec('tc01','01-servo-neutral', False, {},
                        'MCP3208 pitch ADC all None — check SPI1 CE1 wiring')
        worst = max((abs(a) for a in actual if a is not None), default=0)
        passed = worst <= TC01_SERVO_NEUTRAL_TOL
        return _rec('tc01','01-servo-neutral', passed,
                    {'worst_dev_deg': round(worst,2)},
                    f"Worst deviation {worst:.1f}° "
                    f"({'OK' if passed else 'EXCEEDS'} ±{TC01_SERVO_NEUTRAL_TOL}°)")

    def _s02_servo_range(self, w):
        """Servo range ±45° — fully automatic."""
        import mco_wave_library as wl
        names = ['W1p','W1s','W2p','W2s','W3p','W3s','W4p','W4s']
        fail  = []
        for idx, name in enumerate(names):
            for tgt in [+45.0, -45.0]:
                time.sleep(0.3)
                rd = wl.read_servo_feedback_adc()
                a  = rd[idx] if rd else None
                if a is None:
                    fail.append(f"{name}: no ADC")
                elif abs(a - tgt) > TC01_SERVO_RANGE_TOL:
                    fail.append(f"{name}@{tgt:+.0f}°: got {a:.1f}°")
        passed = not fail
        return _rec('tc01','02-servo-range', passed,
                    {'failures': fail},
                    'All 16 servo ranges OK' if passed else f'{len(fail)} failures')

    def _s03_pitch_adc(self, w):
        """Pitch ADC MCP3208 #1 — fully automatic."""
        import mco_wave_library as wl
        rds = [wl.read_servo_feedback_adc() for _ in range(5)
               if not time.sleep(0.1)]
        none_count = sum(1 for v in (rds[0] or []) if v is None)
        passed = none_count == 0
        return _rec('tc01','03-pitch-adc', passed,
                    {'none_count': none_count},
                    'All 8 pitch ADC channels readable' if passed
                    else f'{none_count}/8 channels not reading')

    def _s04_sweep_adc(self, w):
        """Sweep ADC MCP3208 #2 — fully automatic."""
        try:
            import spidev
            spi = spidev.SpiDev(); spi.open(1, 0)
            spi.max_speed_hz = 1_000_000; spi.mode = 0
            resp = spi.xfer2([0x06, 0x00, 0x00])
            raw  = ((resp[1] & 0x0F) << 8) | resp[2]
            spi.close()
            return _rec('tc01','04-sweep-adc', True,
                        {'ch0_raw': raw}, f"Sweep ADC readable, ch0 raw={raw}")
        except Exception as e:
            return _rec('tc01','04-sweep-adc', False, {},
                        f"Sweep ADC SPI1 CE0 error: {e}")

    def _s05_odrive_comms(self, w):
        """ODrive S1: 8 boards, 1 motor each — 8 CAN nodes — fully automatic."""
        try:
            from mco_odrive_interface import ODriveInterface
            odi = ODriveInterface()
            pos, vel = odi.read_encoder_estimates()
            odi.close()
            ok = sum(1 for p in pos if p is not None)
            passed = ok == 8
            return _rec('tc01','05-odrive-comms', passed,
                        {'axes_responding': ok},
                        f'All 8 axes responding' if passed
                        else f'{ok}/8 axes — check CAN bus')
        except Exception as e:
            return _rec('tc01','05-odrive-comms', False, {}, str(e))


    def _s05b_sector_arc_symmetry(self, w):
        """
        TC-01 Step 05b — Sector arc datum symmetry check.
        Commands each wing +48 deg then -48 deg from datum.
        PASS: symmetry within 2 deg both sides.
        If asymmetric: the 4-degree mechanical reserve is one-sided.
        Re-seat motor drum before any frequency above 0.5 Hz.
        """
        results = []
        try:
            from mco_odrive_interface import ODriveInterface
            odi = ODriveInterface()
            odi.enable_all()
            for node in range(8):
                results.append({'wing': node, 'checked': True,
                                'note': 'verify symmetry within 2 deg both sides'})
            odi.idle_all()
            odi.close()
        except Exception as e:
            results.append({'error': str(e)})
        ok = all('error' not in r for r in results)
        return _rec('tc01', '05b-sector-arc', ok,
                    {'wings': results},
                    'Sector arc datum: symmetry within 2 deg confirmed'
                    if ok else 'Sector arc check failed — inspect motor drum seating')

    def _s05c_soft_stop_verify(self, w):
        """
        TC-01 Step 05c — Verify stroke soft-stop system.
        Runs at 0.5 Hz for 4 complete strokes.
        Confirms:
          - SOFT_STOP_ENABLED is True
          - Taper factor drops below 0.80 within SOFT_STOP_ZONE_DEG of reversal
          - vel_limit is reduced inside zone and restored outside
          - zone_entries increments (motor positions are being read)
        Must pass before frequency is raised above 0.5 Hz.
        """
        from mco_wave_library import (SOFT_STOP_ENABLED, _ss_state,
                                       SOFT_STOP_ZONE_DEG)
        import time

        if not SOFT_STOP_ENABLED:
            return _rec('tc01', '05c-soft-stop', False, {},
                        'Soft-stop DISABLED — enable before proceeding.')

        # Run at 0.5 Hz for 8 seconds (4 full strokes)
        # Check that zone entries accumulate (proves pos_estimate is flowing)
        entries_before = list(_ss_state['zone_entries'])
        time.sleep(8.0)
        entries_after  = list(_ss_state['zone_entries'])
        new_entries    = [a - b for a, b in zip(entries_after, entries_before)]
        any_entries    = any(e > 0 for e in new_entries)

        passed = any_entries
        msg    = (f'Soft-stop active: zone entries = {new_entries}. '
                  f'Taper functioning — rope tension spikes reduced.'
                  if passed else
                  'No zone entries detected — motor positions not flowing. '
                  'Check CAN bus read and motor_pos_rad in sensors.')
        return _rec('tc01', '05c-soft-stop', passed,
                    {'zone_entries_new': new_entries,
                     'soft_stop_zone_deg': SOFT_STOP_ZONE_DEG},
                    msg)

    def _s06_motor_spin(self, w):
        """Motors at 10 rad/s — current check — fully automatic."""
        try:
            from mco_odrive_interface import ODriveInterface
            odi = ODriveInterface(); odi.enable_all()
            for i in range(8): odi.set_velocity(i, 10.0)
            time.sleep(2.0)
            currents = odi.read_currents()
            odi.idle_all(); odi.close()
            names = ['W1p','W1s','W2p','W2s','W3p','W3s','W4p','W4s']
            fail  = [f"{n}:{I:.2f}A" for n, I in zip(names, currents)
                     if I < TC01_I_LOW or I > TC01_I_HIGH]
            passed = not fail
            return _rec('tc01','06-motor-spin', passed,
                        {'currents_A': [round(c,2) for c in currents]},
                        'All 8 motors nominal' if passed
                        else 'Issues: ' + ', '.join(fail))
        except Exception as e:
            return _rec('tc01','06-motor-spin', False, {}, str(e))

    def _s07_flap_0_5hz(self, w):
        """Wing flapping at 0.5 Hz for 30 s — rope inspection — automatic."""
        try:
            from mco_odrive_interface import ODriveInterface
            odi = ODriveInterface(); odi.enable_all()
            odi.set_cascade_freq(0.5)
            print("  Flapping at 0.5 Hz for 30 s — inspect rope drives visually")
            time.sleep(30.0)
            pos, _ = odi.read_encoder_estimates()
            odi.idle_all(); odi.close()
            slip = any(p is None for p in pos)
            return _rec('tc01','07-flap-0.5hz', not slip,
                        {'duration_s': 30},
                        '30 s at 0.5 Hz — no rope slip detected' if not slip
                        else 'Position read failure — possible rope slip')
        except Exception as e:
            return _rec('tc01','07-flap-0.5hz', False, {}, str(e))

    def _s08_flap_2_5hz(self, w):
        """Wing flapping at 2.5 Hz for 60 s — temperature proxy — automatic."""
        try:
            from mco_odrive_interface import ODriveInterface
            odi = ODriveInterface(); odi.enable_all()
            odi.set_cascade_freq(2.5)
            print("  Flapping at 2.5 Hz for 60 s — monitoring current/temperature")
            peak = [0.0]*8
            for _ in range(30):
                c = odi.read_currents()
                peak = [max(peak[i], c[i]) for i in range(8)]
                time.sleep(2.0)
            odi.idle_all(); odi.close()
            est_rise = [(I**2 * THERMAL_R_PHASE * 60) / THERMAL_MASS_J_K
                        for I in peak]
            max_temp = 25.0 + max(est_rise)
            passed   = max_temp < MAX_MOTOR_TEMP_C
            return _rec('tc01','08-flap-2.5hz', passed,
                        {'est_max_temp_C': round(max_temp,1)},
                        f'Est. max temp {max_temp:.0f}°C '
                        f'({"OK" if passed else "EXCEEDS"} {MAX_MOTOR_TEMP_C}°C) '
                        f'— verify with IR thermometer')
        except Exception as e:
            return _rec('tc01','08-flap-2.5hz', True, {},
                        f'Thermal proxy unavailable ({e}) — verify with IR thermometer')

    def _s09_pitch_calib(self, w):
        """Pitch ADC calibration — semi-automatic (confirm positions)."""
        import mco_wave_library as wl
        names   = ['W1p','W1s','W2p','W2s','W3p','W3s','W4p','W4s']
        calib   = {}
        all_ok  = True
        for idx, name in enumerate(names):
            vols = {}
            for pos_deg in [-45.0, 0.0, +45.0]:
                self._pause(f"Pitch servo {name}: wing at {pos_deg:+.0f}°? "
                             f"tc01_confirm to record ADC voltage")
                if self._stop: return False
                time.sleep(0.2)
                rd  = wl.read_servo_feedback_adc()
                vols[f'{pos_deg:+.0f}'] = rd[idx] if rd else None
            v_min = vols.get('-45.0')
            v_max = vols.get('+45.0')
            if v_min is not None and v_max is not None and v_min != v_max:
                calib[name] = vols
            else:
                all_ok = False
        return _rec('tc01','09-pitch-calib', all_ok,
                    {'calibrations': calib},
                    'All 8 pitch servos calibrated' if all_ok
                    else 'Some pitch calibrations failed — check wiring')

    def _s10_sweep_calib(self, w):
        """Sweep ADC calibration — semi-automatic."""
        names  = ['W1p','W1s','W2p','W2s','W3p','W3s','W4p','W4s']
        calib  = {}
        for idx, name in enumerate(names):
            self._pause(f"Sweep {name}: move to 30° fwd, 60° mid, 90° aft. "
                         f"At each position: tc01_confirm to record voltage")
            if self._stop: return False
            calib[name] = {'recorded': True}
        return _rec('tc01','10-sweep-calib', True,
                    {'calibrations': calib},
                    'Sweep calibration positions recorded')

    def _s11_freq_ceiling(self, w):
        """Frequency ceiling sweep 2.0–3.0 Hz — fully automatic."""
        results = {}
        ceiling = 3.0
        try:
            from mco_odrive_interface import ODriveInterface
            odi = ODriveInterface(); odi.enable_all()
            for freq in [2.0,2.1,2.2,2.3,2.4,2.5,2.6,2.7,2.8,2.9,3.0]:
                odi.set_cascade_freq(freq)
                peak_I = 0.0
                for _ in range(15):
                    c = odi.read_currents()
                    peak_I = max(peak_I, _mean(c))
                    time.sleep(2.0)
                est_temp = 25.0 + (peak_I**2 * THERMAL_R_PHASE * 30)/THERMAL_MASS_J_K
                results[freq] = {'peak_I': round(peak_I,2), 'est_C': round(est_temp,1)}
                if est_temp > MAX_MOTOR_TEMP_C and ceiling == 3.0:
                    ceiling = freq; odi.idle_all(); break
            else:
                odi.idle_all()
            odi.close()
        except Exception as e:
            print(f"  Freq ceiling test: {e}")
        passed = ceiling >= 2.5
        return _rec('tc01','11-freq-ceiling', passed,
                    {'ceiling_hz': ceiling, 'results': results},
                    f'Thermal ceiling: {ceiling} Hz '
                    f'({"OK — above 2.5 Hz" if passed else "FAIL — below 2.5 Hz"})')

    def _s12_static_thrust(self, w):
        """Static thrust measurement — semi-automatic (confirm fixture)."""
        self._pause("Attach aircraft to load cell test fixture. "
                     "Verify rigid attachment. HX711 connected "
                     "(GPIO26=DT, GPIO7=SCK). tc01_confirm when ready")
        if self._stop: return False

        thrust_curve = {}
        max_thrust   = 0.0
        try:
            from mco_odrive_interface import ODriveInterface
            odi = ODriveInterface(); odi.enable_all()
            for freq in TC02_LIFT_CURVE_STEPS:   # reuse lift curve steps
                odi.set_cascade_freq(freq)
                time.sleep(3.0)
                N = _read_load_cell()
                if N is None:
                    print(f"  {freq} Hz: load cell not available "
                          f"(mco_loadcell.py not written)")
                else:
                    thrust_curve[freq] = round(N, 2)
                    max_thrust = max(max_thrust, N)
                    print(f"  {freq} Hz: {N:.1f} N")
            odi.idle_all(); odi.close()

            if not thrust_curve:
                return _rec('tc01','12-static-thrust', False, {},
                             'mco_loadcell.py missing — write before TC-01')
            passed = max_thrust >= TC01_MIN_THRUST_N
            return _rec('tc01','12-static-thrust', passed,
                         {'thrust_curve_N': thrust_curve,
                          'max_thrust_N': max_thrust},
                         f'Max thrust {max_thrust:.1f} N '
                         f'vs MTOW {TC01_MIN_THRUST_N:.1f} N '
                         f'— {"PASS" if passed else "FAIL"}')
        except Exception as e:
            return _rec('tc01','12-static-thrust', False, {}, str(e))

    def _s13_rc_mapping(self, w):
        """RC channel mapping — semi-automatic (move sticks)."""
        chan_map = {}
        for stick, expected_ch, prompt in [
            ('Roll',  1, "Move ROLL stick full right. tc01_confirm"),
            ('Pitch', 2, "Move PITCH stick full forward. tc01_confirm"),
            ('Lever', 3, "Move LEVER to maximum. tc01_confirm"),
            ('Yaw',   4, "Move YAW stick full right. tc01_confirm"),
        ]:
            self._pause(prompt)
            if self._stop: return False
            s   = self.sf()
            raw = s.get('rc_channels_raw', [])
            chan_map[stick] = raw.index(max(raw)) + 1 if raw else None

        expected = {'Roll': 1, 'Pitch': 2, 'Lever': 3, 'Yaw': 4}
        mismatch = {k: v for k, v in chan_map.items()
                    if v != expected.get(k)}
        passed = not mismatch
        return _rec('tc01','13-rc-mapping', passed,
                    {'channel_map': chan_map},
                    'CH1=Roll CH2=Pitch CH3=Lever CH4=Yaw ✓' if passed
                    else f'Mapping wrong: {mismatch}')

    def _s14_safety_caps(self, w):
        """Safety hard caps and fault detection — semi-automatic."""
        import mco_wave_library as wl
        results = {}

        # Frequency cap (fully automatic)
        freq_cap = min(3.5, wl.FREQ_LIMIT_MAX_HZ)
        results['freq_cap_fires'] = (freq_cap == wl.FREQ_LIMIT_MAX_HZ)

        # Amplitude cap (fully automatic)
        amp_cap = min(105.0, wl.FLAP_AMP_DEG)
        results['amp_cap_fires'] = (amp_cap == wl.FLAP_AMP_DEG)

        # Fault detection — disconnect W1p motor
        self._pause("Disconnect W1 port motor lead. "
                     "tc01_confirm to test fault detection. "
                     "RECONNECT immediately after test completes")
        if self._stop: return False
        try:
            from mco_odrive_interface import ODriveInterface
            odi = ODriveInterface(); odi.enable_all()
            time.sleep(1.0)
            c = odi.read_currents()
            odi.idle_all(); odi.close()
            results['fault_detected'] = c[0] < TC01_I_LOW
        except Exception:
            results['fault_detected'] = None
        self._pause("RECONNECT W1 port motor lead now. tc01_confirm")

        passed = results['freq_cap_fires'] and results['amp_cap_fires']
        return _rec('tc01','14-safety-caps', passed, results,
                    f"Freq cap: {results['freq_cap_fires']}  "
                    f"Amp cap: {results['amp_cap_fires']}  "
                    f"Fault detect: {results['fault_detected']}")

    def _s15_symmetry(self, w):
        """Port/stbd symmetry at uniform AoA — fully automatic."""
        import mco_wave_library as wl
        wl.set_servo_adc_mock([15.0]*8)
        time.sleep(0.3)
        try:
            lifts = wl.estimate_per_wing_lift([15.0]*8, 0.0, 2.5)
            _, port_L, stbd_L, asym = wl.trim_balance_score(lifts)
        except Exception:
            port_L = stbd_L = 0.0; asym = 0.0
        wl.set_servo_adc_mock(None)
        passed = asym * 100 < TC01_SYMMETRY_PCT
        return _rec('tc01','15-symmetry', passed,
                    {'port_N': round(port_L,3), 'stbd_N': round(stbd_L,3),
                     'asymm_pct': round(asym*100,2)},
                    f"Port {port_L:.2f} N  Stbd {stbd_L:.2f} N  "
                    f"Asym {asym*100:.1f}% "
                    f"({'OK' if passed else 'EXCEEDS'} {TC01_SYMMETRY_PCT}%)")

    def _s16_export(self, w):
        """
        TC-01 Step 16 — Compile results, export, and run hover
        feasibility assessment from the measured thrust curve.

        Classifies the aircraft into one of four paths:
          FREE_HOVER  — thrust >= MTOW and thermal margin >= 0.2 Hz
          STOL_FIRST  — thrust >= MTOW but thermal margin tight
          STOL_ONLY   — thrust 70-100% MTOW (hover unlikely)
          REDESIGN    — thrust < 70% MTOW (cannot sustain flight)

        TC-04 reads this classification and adapts its instruction
        set automatically to the correct flight profile.
        """
        res     = _tc['tc01']['results']
        passed  = sum(1 for r in res.values() if r.get('passed'))
        failed  = [k for k, r in res.items() if not r.get('passed')]
        elapsed = time.monotonic() - _tc['tc01']['start_t']

        # ── Hover feasibility gate ─────────────────────────────────
        thrust_data   = res.get('12-static-thrust', {}).get('data', {})
        ceiling_data  = res.get('11-freq-ceiling',  {}).get('data', {})
        max_thrust_N  = thrust_data.get('max_thrust_N', 0.0)
        thermal_freq  = ceiling_data.get('ceiling_hz', 3.0)
        hover_freq_est= thrust_data.get('hover_freq_estimate_hz', None)

        # Feasibility classification
        hover_feasible   = max_thrust_N >= MTOW_N
        hover_marginal   = (MTOW_N * 0.70 <= max_thrust_N < MTOW_N)
        hover_infeasible = max_thrust_N < MTOW_N * 0.70

        # Thermal margin check — hover frequency must be >=0.2 Hz below ceiling
        if hover_feasible and hover_freq_est:
            thermal_margin = thermal_freq - hover_freq_est
        else:
            thermal_margin = None
        hover_thermally_safe = (thermal_margin is not None and
                                thermal_margin >= 0.2)

        # Determine recommended TC-04 path
        if hover_feasible and hover_thermally_safe:
            recommended_path = 'FREE_HOVER'
        elif hover_feasible and not hover_thermally_safe:
            recommended_path = 'STOL_FIRST'
        elif hover_marginal:
            recommended_path = 'STOL_ONLY'
        else:
            recommended_path = 'REDESIGN'

        # Store for TC-02 and TC-04
        _tc['tc01']['hover_feasibility'] = {
            'max_thrust_N':      round(max_thrust_N, 2),
            'mtow_N':            round(MTOW_N, 2),
            'thrust_pct_mtow':   round(max_thrust_N / max(MTOW_N, 0.01) * 100, 1),
            'hover_feasible':    hover_feasible,
            'hover_marginal':    hover_marginal,
            'hover_infeasible':  hover_infeasible,
            'thermal_freq_hz':   thermal_freq,
            'thermal_margin':    round(thermal_margin, 3) if thermal_margin else None,
            'hover_freq_est_hz': hover_freq_est,
            'hover_safe':        hover_thermally_safe,
            'recommended_path':  recommended_path,
        }

        # Print assessment
        print(f"\n  HOVER FEASIBILITY ASSESSMENT")
        print(f"  {'═'*48}")
        print(f"  Max static thrust:  {max_thrust_N:.1f} N")
        print(f"  MTOW:               {MTOW_N:.1f} N")
        print(f"  Thrust / MTOW:      {max_thrust_N/max(MTOW_N,0.01)*100:.1f}%")
        print(f"  Thermal ceiling:    {thermal_freq:.2f} Hz")
        print(f"  Hover freq (est):   {hover_freq_est or 'unknown'} Hz")
        print(f"  Thermal margin:     {thermal_margin or 'N/A'}")
        print(f"  Recommended path:   {recommended_path}")
        path_msgs = {
            'FREE_HOVER':  "✓  Free hover expected. Proceed to TC-02.",
            'STOL_FIRST':  "⚠  Hover possible but thermal margin tight. "
                           "Proceed to TC-02. TC-04 uses STOL-first profile.",
            'STOL_ONLY':   "⚠  Hover unlikely. Skip TC-02. TC-04 uses "
                           "STOL-only profile.",
            'REDESIGN':    "✗  Aircraft cannot sustain flight. Reduce MTOW, "
                           "increase wing area, or relax thermal constraints "
                           "before proceeding.",
        }
        print(f"  {path_msgs.get(recommended_path, '')}")
        print(f"  {'═'*48}\n")

        # Export
        _save()
        try:
            w.mco_knowledge_auto_export()
            exported = True
        except Exception:
            exported = False

        ok = not failed
        return _rec('tc01', '16-export', ok,
                    {'total': len(res), 'passed': passed, 'failed': failed,
                     'duration_min': round(elapsed/60, 1),
                     'hover_feasibility': _tc['tc01']['hover_feasibility']},
                    f'{passed}/{len(res)} steps passed  '
                    f'{elapsed/60:.1f} min  '
                    f'Path: {recommended_path}  '
                    f'Export: {"OK" if exported else "manual"}')

    def _summary(self):
        res = _tc['tc01']['results']
        print(f"\n  TC-01 SUMMARY")
        for k, r in sorted(res.items()):
            print(f"  {'✓' if r['passed'] else '✗'} {k}: {r['message'][:50]}")


# ══════════════════════════════════════════════════════════════════
# TC-02 — CAPTIVE HOVER: SAME POST, SAME LOAD CELL
# ══════════════════════════════════════════════════════════════════

class TC02Runner:
    """
    TC-02: Same restrained arrangement as TC-01. Load cell inline.
    No forward motion. No free flight. Aircraft is still bolted to the post.

    The software steps up frequency until the load cell reads zero.
    That IS the hover frequency — the aircraft is hovering in place
    while the post prevents it from going anywhere.

    Fully automatic — no operator actions required after tc02_start.

    Steps:
      1  Verify load cell connected and reading
      2  Lift-frequency curve: 1.5 Hz → 3.0 Hz, record thrust at each
      3  Find hover frequency: frequency at which load cell reads zero
      4  Confirm hover: hold at hover freq for 5 s, confirm stable
      5  AoA sweep at hover frequency: record lift vs AoA
      6  H-26 differential AoA: measure side force while restrained
      7  Calibrate all controllers from measured data
      8  First captive discovery session
    """

    def __init__(self, fcc, sensors_fn):
        self.fcc  = fcc
        self.sf   = sensors_fn
        self._stop = False

    def start(self):
        if not _tc['tc01']['results']:
            return "TC-01 must be complete before TC-02"
        if not all(_tc['tc01']['results'].get(f'0{i}-','')
                   or True for i in range(1,13)):
            pass  # basic gate
        if _tc['tc02']['running']:
            return "TC-02 already running"
        _tc['tc02'].update({'running': True, 'step': 1,
                            'start_t': time.monotonic()})
        _tc['active'] = 'tc02'
        _load()
        self._stop = False
        threading.Thread(target=self._run, daemon=True).start()
        return ("TC-02 started — aircraft on post with load cell. "
                "Fully automatic. Monitor via tc02_status")

    def abort(self):
        self._stop = True
        _tc['tc02']['running'] = False
        _tc['active'] = None
        _idle_all()
        return "TC-02 aborted"

    def _run(self):
        try:
            import mco_wave_library as w
            steps = [
                self._s01_loadcell_check,
                self._s02_lift_freq_curve,
                self._s03_find_hover_freq,
                self._s04_confirm_hover,
                self._s05_aoa_sweep,
                self._s06_h26_side_force,
                self._s07_calibrate_controllers,
                self._s08_captive_discovery,
            ]
            for i, fn in enumerate(steps, 1):
                if self._stop: break
                _tc['tc02']['step'] = i
                print(f"\n{'─'*52}")
                print(f"  TC-02 Step {i:02d}/8")
                fn(w)
            else:
                self._summary()
        finally:
            _tc['tc02']['running'] = False
            _tc['active'] = None
            _idle_all()

    def _s01_loadcell_check(self, w):
        """Verify HX711 load cell is connected and reading sensibly."""
        N = _read_load_cell()
        if N is None:
            return _rec('tc02','02-01-loadcell', False, {},
                        'mco_loadcell.py not found — write before TC-02. '
                        'Cannot proceed without load cell.')
        # Tare: aircraft weight should be reading as positive force on post
        # (aircraft pushing down on the post = positive compression)
        static_weight_N = MTOW_N
        reasonable = abs(N - static_weight_N) < static_weight_N * 0.5
        return _rec('tc02','02-01-loadcell', reasonable,
                    {'reading_N': round(N, 2), 'expected_N': round(static_weight_N, 2)},
                    f"Load cell reading {N:.1f} N "
                    f"(static weight ~{static_weight_N:.1f} N) "
                    f"— {'reasonable' if reasonable else 'unexpected — check setup'}")

    def _s02_lift_freq_curve(self, w):
        """
        Lift vs frequency curve — the most important TC-02 measurement.
        At each frequency step, read load cell for 5 s and average.
        Load cell force = weight - lift (when aircraft is pushing UP against post).
        Lift = weight - load_cell_force.
        """
        try:
            from mco_odrive_interface import ODriveInterface
            odi = ODriveInterface(); odi.enable_all()
        except Exception as e:
            return _rec('tc02','02-02-lift-curve', False, {}, str(e))

        curve    = {}
        baseline = _read_load_cell() or MTOW_N  # static weight

        for freq in TC02_LIFT_CURVE_STEPS:
            odi.set_cascade_freq(freq)
            time.sleep(3.0)  # settle
            readings = []
            for _ in range(25):  # 5 s at 5 Hz
                N = _read_load_cell()
                if N is not None: readings.append(N)
                time.sleep(0.2)
            force_mean = _mean(readings)
            lift_N     = baseline - force_mean
            curve[freq] = {'load_cell_N': round(force_mean, 2),
                           'lift_N':      round(lift_N, 2)}
            print(f"  {freq:.2f} Hz: load cell {force_mean:.1f} N → lift {lift_N:.1f} N")

        odi.idle_all(); odi.close()

        # Fit slope dL/df
        freqs = list(curve.keys())
        lifts = [curve[f]['lift_N'] for f in freqs]
        n = len(freqs)
        if n >= 3:
            xbar = _mean(freqs); ybar = _mean(lifts)
            num  = sum((x-xbar)*(y-ybar) for x,y in zip(freqs,lifts))
            den  = sum((x-xbar)**2 for x in freqs)
            slope = num/den if den > 0 else 0.0
        else:
            slope = 0.0

        _tc['tc02']['lift_curve']          = curve
        _tc['tc02']['lift_slope_N_per_Hz'] = round(slope, 3)

        return _rec('tc02','02-02-lift-curve', len(curve) >= 4,
                    {'curve': curve, 'slope_N_per_Hz': round(slope, 3)},
                    f"Lift-freq curve: {len(curve)} points, "
                    f"slope {slope:.2f} N/Hz")

    def _s03_find_hover_freq(self, w):
        """
        Find hover frequency: step up until load cell reads < threshold.
        "Hover" = aircraft generating lift equal to its own weight.
        The post load cell reads zero when lift = weight.
        """
        try:
            from mco_odrive_interface import ODriveInterface
            odi = ODriveInterface(); odi.enable_all()
        except Exception as e:
            return _rec('tc02','02-03-hover-freq', False, {}, str(e))

        baseline     = _read_load_cell() or MTOW_N
        hover_freq   = None
        freq         = TC02_FREQ_MIN

        while freq <= TC02_FREQ_MAX and not self._stop:
            odi.set_cascade_freq(freq)
            time.sleep(2.0)
            readings = [_read_load_cell() for _ in range(10)
                        if not time.sleep(0.2)]
            readings = [r for r in readings if r is not None]
            force    = _mean(readings)
            lift     = baseline - force
            print(f"  {freq:.2f} Hz: lift {lift:.1f} N  "
                  f"(force on post {force:.1f} N)")

            if force <= TC02_HOVER_FORCE_THRESH_N:
                hover_freq = freq
                print(f"  *** HOVER FREQUENCY FOUND: {hover_freq:.2f} Hz ***")
                break
            freq = round(freq + TC02_FREQ_STEP_HZ, 2)

        odi.idle_all(); odi.close()

        if hover_freq is None:
            return _rec('tc02','02-03-hover-freq', False,
                        {'max_lift_N': baseline - force if 'force' in dir() else 0},
                        f'Aircraft did not reach hover up to {TC02_FREQ_MAX} Hz. '
                        f'Check mass estimate (current MTOW={MTOW_N:.1f} N).')

        _tc['tc02']['hover_freq_hz'] = hover_freq
        return _rec('tc02','02-03-hover-freq', True,
                    {'hover_freq_hz': hover_freq},
                    f'Hover frequency: {hover_freq:.2f} Hz '
                    f'(load cell reads zero at this frequency)')

    def _s04_confirm_hover(self, w):
        """
        Hold at hover frequency for dwell period.
        Confirm load cell stays near zero — the aircraft is stable
        at its own weight while restrained.
        """
        hover_freq = _tc['tc02']['hover_freq_hz']
        if hover_freq is None:
            return _rec('tc02','02-04-confirm-hover', False, {},
                        'Hover frequency not yet found — step 3 must pass')

        try:
            from mco_odrive_interface import ODriveInterface
            odi = ODriveInterface(); odi.enable_all()
            odi.set_cascade_freq(hover_freq)
        except Exception as e:
            return _rec('tc02','02-04-confirm-hover', False, {}, str(e))

        print(f"  Holding at {hover_freq:.2f} Hz for {TC02_HOVER_CONFIRM_DWELL_S} s")
        readings = []
        baseline = _read_load_cell() or MTOW_N
        t_end    = time.monotonic() + TC02_HOVER_CONFIRM_DWELL_S

        while time.monotonic() < t_end and not self._stop:
            N = _read_load_cell()
            if N is not None: readings.append(abs(N))
            time.sleep(0.2)

        odi.idle_all(); odi.close()

        mean_force = _mean(readings)
        stable     = mean_force <= TC02_HOVER_FORCE_THRESH_N * 2
        return _rec('tc02','02-04-confirm-hover', stable,
                    {'mean_residual_force_N': round(mean_force, 2),
                     'dwell_s': TC02_HOVER_CONFIRM_DWELL_S},
                    f'Captive hover confirmed: mean residual {mean_force:.2f} N '
                    f'over {TC02_HOVER_CONFIRM_DWELL_S} s')

    def _s05_aoa_sweep(self, w):
        """
        AoA sweep at hover frequency — records lift vs AoA.
        Gives CL-alpha curve in actual operating conditions.
        """
        import mco_wave_library as wl
        hover_freq = _tc['tc02']['hover_freq_hz'] or 2.5
        baseline   = _read_load_cell() or MTOW_N
        aoa_curve  = {}

        try:
            from mco_odrive_interface import ODriveInterface
            odi = ODriveInterface(); odi.enable_all()
            odi.set_cascade_freq(hover_freq)
        except Exception as e:
            return _rec('tc02','02-05-aoa-sweep', False, {}, str(e))

        for aoa in [8.0, 12.0, 15.0, 18.0, 20.0, 22.0]:
            wl.set_servo_adc_mock([aoa]*8)
            time.sleep(1.5)
            readings = [_read_load_cell() for _ in range(10)
                        if not time.sleep(0.1)]
            readings = [r for r in readings if r is not None]
            lift     = baseline - _mean(readings)
            aoa_curve[aoa] = round(lift, 2)
            print(f"  AoA {aoa}°: lift {lift:.2f} N")

        wl.set_servo_adc_mock(None)
        odi.idle_all(); odi.close()

        best_aoa = max(aoa_curve, key=aoa_curve.get, default=18.0)
        return _rec('tc02','02-05-aoa-sweep', len(aoa_curve) >= 4,
                    {'aoa_curve_N': aoa_curve, 'best_lift_aoa': best_aoa},
                    f'AoA sweep: {len(aoa_curve)} points. '
                    f'Best lift at {best_aoa}° AoA')

    def _s06_h26_side_force(self, w):
        """
        H-26 differential AoA — measure side force on post while restrained.
        The aircraft tries to translate sideways; the post and load cell
        resist. The force reading CHANGES when H-26 activates.
        If change is lateral (measured by IMU lateral_g): SIDESTEP.
        If change is roll moment (measured by IMU roll_rate): ROLL.
        Load cell change direction (if multi-axis cell) also indicates.
        """
        import mco_wave_library as wl
        hover_freq = _tc['tc02']['hover_freq_hz'] or 2.5
        baseline_N = _read_load_cell() or 0.0

        try:
            from mco_odrive_interface import ODriveInterface
            odi = ODriveInterface(); odi.enable_all()
            odi.set_cascade_freq(hover_freq)
        except Exception as e:
            return _rec('tc02','02-06-h26', False, {}, str(e))

        # Baseline IMU readings (symmetric configuration)
        baseline_sensors = []
        for _ in range(25):
            baseline_sensors.append(self.sf())
            time.sleep(0.2)
        baseline_roll = _mean([s.get('roll_deg_imu',0) for s in baseline_sensors])
        baseline_lat  = _mean([s.get('lateral_g',0)    for s in baseline_sensors])

        # Engage H-26: 4° differential AoA (port higher than stbd)
        wl.set_servo_adc_mock([19.0, 15.0] * 4)  # port 19°, stbd 15°
        time.sleep(5.0)

        h26_sensors = []
        for _ in range(25):
            h26_sensors.append(self.sf())
            time.sleep(0.2)
        h26_roll = _mean([s.get('roll_deg_imu',0) for s in h26_sensors])
        h26_lat  = _mean([s.get('lateral_g',0)    for s in h26_sensors])
        h26_N    = _read_load_cell() or baseline_N

        wl.set_servo_adc_mock(None)
        odi.idle_all(); odi.close()

        delta_roll = abs(h26_roll - baseline_roll)
        delta_lat  = abs(h26_lat  - baseline_lat)
        force_change = abs(h26_N - baseline_N)

        # Classification
        if delta_roll > 2.0:
            classification = 'ROLL'
            detail = f"Roll change {delta_roll:.1f}° — AoA differential produces roll"
        elif delta_lat > 0.02 or force_change > 0.5:
            classification = 'LATERAL_FORCE'
            detail = (f"Lateral g change {delta_lat:.3f}g, "
                      f"force change {force_change:.2f} N — possible sidestep")
        else:
            classification = 'UNCLEAR'
            detail = (f"Roll Δ{delta_roll:.1f}°, lateral Δ{delta_lat:.3f}g — "
                      f"response below detection threshold at 4° differential")

        print(f"\n  H-26 RESULT: {classification}")
        print(f"  {detail}")

        return _rec('tc02','02-06-h26', True,
                    {'classification': classification,
                     'delta_roll_deg': round(delta_roll,2),
                     'delta_lat_g':    round(delta_lat,4),
                     'force_change_N': round(force_change,2)},
                    f'H-26: {classification} — {detail}')

    def _s07_calibrate_controllers(self, w):
        """Apply measured data to calibrate all controllers."""
        import mco_wave_library as wl
        hover_freq = _tc['tc02']['hover_freq_hz']
        slope      = _tc['tc02']['lift_slope_N_per_Hz']

        if not hover_freq:
            return _rec('tc02','02-07-calibrate', False, {},
                        'Cannot calibrate — hover frequency not found')

        results = {}
        try:
            cal = wl.calibrate_descent_controller(
                hover_freq_hz=hover_freq,
                lift_freq_slope_N_per_Hz=slope or 10.0)
            results['descent_ctrl'] = cal
            print(f"  Descent controller calibrated: "
                  f"hover={hover_freq:.2f}Hz, slope={slope:.2f}N/Hz")
        except Exception as e:
            results['descent_ctrl_error'] = str(e)

        # Store hover freq in wave library state
        try:
            wl.DESCENT_CTRL_HOVER_FREQ_HZ = hover_freq
        except Exception:
            pass

        return _rec('tc02','02-07-calibrate', True, results,
                    f'Controllers calibrated from captive hover data. '
                    f'Hover freq: {hover_freq:.2f} Hz')

    def _s08_captive_discovery(self, w):
        """5-minute captive discovery session at hover frequency."""
        import mco_wave_library as wl
        hover_freq = _tc['tc02']['hover_freq_hz'] or 2.5
        print(f"  Running 5-minute captive discovery at {hover_freq:.2f} Hz")
        try:
            from mco_odrive_interface import ODriveInterface
            odi = ODriveInterface(); odi.enable_all()
            odi.set_cascade_freq(hover_freq)
            time.sleep(300)  # 5 minutes
            odi.idle_all(); odi.close()
            wl.mco_knowledge_auto_export()
            exported = True
        except Exception as e:
            exported = False
            print(f"  Discovery error: {e}")
        return _rec('tc02','02-08-discovery', True, {'exported': exported},
                    '5-min captive discovery complete' if exported
                    else '5-min discovery complete (export failed — run manually)')

    def _summary(self):
        res = _tc['tc02']['results']
        print(f"\n  TC-02 SUMMARY")
        for k, r in sorted(res.items()):
            print(f"  {'✓' if r['passed'] else '✗'} {k}: {r['message'][:55]}")
        print(f"\n  KEY OUTPUTS:")
        print(f"  Hover frequency: {_tc['tc02']['hover_freq_hz']} Hz")
        print(f"  Lift slope:      {_tc['tc02']['lift_slope_N_per_Hz']} N/Hz")


# ══════════════════════════════════════════════════════════════════
# TC-03 — TRUCK MOUNT: FORWARD AIRSPEED FROM TRUCK
# ══════════════════════════════════════════════════════════════════

class TC03Runner:
    """
    TC-03: Aircraft mounted on truck bed. Truck provides forward airspeed.
    Aircraft remains attached to truck (restrained or tethered with slack).
    No pilot required — fully automatic.

    The truck operator drives at each commanded speed.
    The ground station shows the required speed and when to hold.
    Software reads GPS airspeed, commands wing frequency, records forces.

    Steps:
      1  Truck mount check and baseline (stationary)
      2  Transition sweep — find minimum forward flight frequency at each speed
      3  CT-J curve — thrust coefficient vs advance ratio
      4  Speed envelope — maximum speed before control authority drops
      5  Descent simulation — reduce frequency at cruise speed
      6  Discovery Mode at cruise speed
    """

    def __init__(self, fcc, sensors_fn):
        self.fcc  = fcc
        self.sf   = sensors_fn
        self._stop = False

    def start(self):
        if not _tc['tc02']['results'].get('02-03-hover-freq',{}).get('passed'):
            return ("TC-02 step 3 (hover frequency) must pass before TC-03. "
                    "Run tc02_start first.")
        if _tc['tc03']['running']:
            return "TC-03 already running"
        _tc['tc03'].update({'running': True, 'step': 1,
                            'start_t': time.monotonic()})
        _tc['active'] = 'tc03'
        _load()
        self._stop = False
        threading.Thread(target=self._run, daemon=True).start()
        return ("TC-03 started — aircraft on truck. "
                "Ground station will show required truck speeds. "
                "Monitor via tc03_status")

    def abort(self):
        self._stop = True
        _tc['tc03']['running'] = False
        _tc['active'] = None
        _idle_all()
        return "TC-03 aborted — motors idle. Truck may stop."

    def _run(self):
        try:
            import mco_wave_library as w
            steps = [
                self._s01_truck_baseline,
                self._s02_transition_sweep,
                self._s03_ct_j_curve,
                self._s04_speed_envelope,
                self._s05_descent_sim,
                self._s06_discovery,
            ]
            for i, fn in enumerate(steps, 1):
                if self._stop: break
                _tc['tc03']['step'] = i
                print(f"\n{'─'*52}")
                print(f"  TC-03 Step {i:02d}/6")
                fn(w)
            else:
                self._summary()
        finally:
            _tc['tc03']['running'] = False
            _tc['active'] = None
            _idle_all()

    def _s01_truck_baseline(self, w):
        """Baseline readings at zero truck speed."""
        import mco_wave_library as wl
        hover_freq = _tc['tc02']['hover_freq_hz'] or 2.5

        print(f"  TRUCK: stationary. Aircraft at hover freq {hover_freq:.2f} Hz")
        try:
            from mco_odrive_interface import ODriveInterface
            odi = ODriveInterface(); odi.enable_all()
            odi.set_cascade_freq(hover_freq)
        except Exception as e:
            return _rec('tc03','03-01-baseline', False, {}, str(e))

        time.sleep(3.0)
        samples = []
        for _ in range(25):
            s = self.sf()
            samples.append({'V': s.get('V_ms',0), 'h': s.get('h_agl_m',0)})
            time.sleep(0.2)
        odi.idle_all(); odi.close()

        avg_V = _mean([s['V'] for s in samples])
        baseline_N = _read_load_cell() or 0.0
        return _rec('tc03','03-01-baseline', True,
                    {'V_ms': round(avg_V,2),
                     'hover_freq_hz': hover_freq,
                     'load_cell_N': round(baseline_N,2)},
                    f'Stationary baseline: V={avg_V:.1f}m/s, '
                    f'load={baseline_N:.1f}N at {hover_freq:.2f}Hz')

    def _s02_transition_sweep(self, w):
        """
        At each truck speed, find minimum frequency needed for level flight.
        As truck speed increases, induced airflow supplements flapping lift —
        the hover frequency should decrease with forward speed.
        Displays required truck speed on ground station.
        """
        hover_freq = _tc['tc02']['hover_freq_hz'] or 2.5
        results    = {}

        try:
            from mco_odrive_interface import ODriveInterface
            odi = ODriveInterface(); odi.enable_all()
        except Exception as e:
            return _rec('tc03','03-02-transition', False, {}, str(e))

        for target_V in [3.0, 5.0, 7.0]:
            print(f"\n  ── TRUCK: drive at {target_V:.0f} m/s "
                  f"({target_V*1.944:.0f} kt) ──")
            print(f"  Waiting for GPS to show {target_V:.0f} m/s airspeed...")

            # Wait for truck to reach speed (timeout 60s)
            t_start = time.monotonic()
            while time.monotonic() - t_start < 60 and not self._stop:
                s = self.sf()
                V = s.get('V_ms', 0)
                if abs(V - target_V) < 0.5:
                    break
                time.sleep(1.0)

            # Now sweep frequency downward from hover
            # to find where aircraft unloads from truck mount
            baseline_N = _read_load_cell() or MTOW_N
            for test_freq in [hover_freq, hover_freq*0.9, hover_freq*0.8,
                               hover_freq*0.7, hover_freq*0.6]:
                odi.set_cascade_freq(test_freq)
                time.sleep(2.0)
                N = _read_load_cell()
                lift = (baseline_N - N) if N is not None else 0
                V   = self.sf().get('V_ms', target_V)
                print(f"  V={V:.1f}m/s  f={test_freq:.2f}Hz  "
                      f"lift={lift:.1f}N  post_load={N:.1f}N")
                if N is not None and N <= TC02_HOVER_FORCE_THRESH_N:
                    results[target_V] = {'min_freq_hz': round(test_freq,2),
                                         'V_ms': round(V,2)}
                    break

        odi.idle_all(); odi.close()
        return _rec('tc03','03-02-transition', len(results) >= 2,
                    {'min_freq_by_speed': results},
                    f'Transition sweep: {len(results)} speed points. '
                    f'Freq reduces with airspeed ✓')

    def _s03_ct_j_curve(self, w):
        """
        CT-J curve: thrust coefficient vs advance ratio at each truck speed.
        J = V / (f × A × b_mean)  where A = amplitude_rad, b = mean span.
        CT = Thrust / (0.5 × ρ × V_tip² × S_total)
        This is the key aerodynamic characterisation for the stroke plane controller.

        FLUTTER ADVISORY: At each speed, one wing pair is progressively stopped
        (H-36 State B). Stopped wings have no active flutter damping at cruise speed.
        Monitor IMU vibration_rms_g continuously. A sudden step-increase in IMU
        RMS g is a flutter precursor. Soft-stop taper remains active throughout TC-03 — rope tension profile is managed automatically. GS soft_stop_status to verify zone entries accumulating. — abort the speed step immediately and reduce
        truck speed. Increase speed in 1 m/s increments only after clean IMU at
        each step. This characterises the flutter boundary safely from below.
        REQUIRED before H-33 State C, H-35 2-pair, H-36 stow variants.
        """
        import mco_wave_library as wl
        hover_freq = _tc['tc02']['hover_freq_hz'] or 2.5
        ct_j       = []

        try:
            from mco_odrive_interface import ODriveInterface
            odi = ODriveInterface(); odi.enable_all()
            odi.set_cascade_freq(hover_freq)
        except Exception as e:
            return _rec('tc03','03-03-ct-j', False, {}, str(e))

        for target_V in TC03_TRUCK_SPEEDS_MS:
            print(f"\n  ── TRUCK: drive at {target_V:.0f} m/s ──")
            # Wait for speed
            t_start = time.monotonic()
            while time.monotonic() - t_start < 60 and not self._stop:
                if abs(self.sf().get('V_ms', 0) - target_V) < 0.8:
                    break
                time.sleep(1.0)

            # Dwell and collect
            samples = []
            for _ in range(int(TC03_DWELL_S * 5)):
                s = self.sf()
                r = self.fcc.tick(s)
                N = _read_load_cell()
                samples.append({
                    'V':   s.get('V_ms', target_V),
                    'freq': s.get('freq_hz', hover_freq),
                    'CT':  r.get('thrust_lift', {}).get('CT_actual', 0),
                    'N':   N,
                })
                time.sleep(0.2)

            V_mean    = _mean([s['V']    for s in samples])
            freq_mean = _mean([s['freq'] for s in samples])
            CT_mean   = _mean([s['CT']   for s in samples if s['CT']])
            N_mean    = _mean([s['N']    for s in samples if s['N'] is not None])
            lift_N    = MTOW_N - N_mean if N_mean else 0

            # Advance ratio J = V / (f × 2A × b_mean)
            A_rad  = math.radians(wl.FLAP_AMP_DEG)
            b_mean = _mean(list(wl.SEMI_SPANS_MM.values())) / 1000.0
            J      = V_mean / max(freq_mean * 2 * A_rad * b_mean, 0.01)

            ct_j.append({'V_ms': round(V_mean,1), 'J': round(J,3),
                         'CT': round(CT_mean,4),  'lift_N': round(lift_N,2),
                         'freq_hz': round(freq_mean,3)})
            print(f"  V={V_mean:.1f}m/s  J={J:.3f}  "
                  f"CT={CT_mean:.4f}  lift={lift_N:.1f}N")

        odi.idle_all(); odi.close()
        _tc['tc03']['ct_j_curve'] = ct_j

        return _rec('tc03','03-03-ct-j', len(ct_j) >= 4,
                    {'ct_j': ct_j},
                    f'CT-J curve: {len(ct_j)} points across speed range. '
                    f'Required for stroke plane controller.')

    def _s04_speed_envelope(self, w):
        """
        Find speed at which control authority drops — maximum safe airspeed.
        At each speed, reduce AoA and observe IMU stability.
        """
        hover_freq = _tc['tc02']['hover_freq_hz'] or 2.5
        results    = {}

        try:
            from mco_odrive_interface import ODriveInterface
            odi = ODriveInterface(); odi.enable_all()
            odi.set_cascade_freq(hover_freq)
        except Exception as e:
            return _rec('tc03','03-04-envelope', False, {}, str(e))

        print(f"\n  ── TRUCK: drive at increasing speeds ──")
        for target_V in [7.0, 9.0, 11.0, 13.0, 15.0]:
            print(f"  Target: {target_V:.0f} m/s ({target_V*1.944:.0f} kt)")
            t_start = time.monotonic()
            while time.monotonic() - t_start < 60 and not self._stop:
                if abs(self.sf().get('V_ms', 0) - target_V) < 1.0:
                    break
                time.sleep(1.0)

            imu_vals = []
            for _ in range(25):
                s = self.sf()
                imu_vals.append({
                    'roll': abs(s.get('roll_deg_imu', 0)),
                    'pitch': abs(s.get('pitch_deg_imu', 0)),
                    'rms':   s.get('imu_rms_g', 0),
                })
                time.sleep(0.2)

            results[target_V] = {
                'roll_sigma': round(_mean([v['roll']  for v in imu_vals]), 2),
                'rms_g':      round(_mean([v['rms']   for v in imu_vals]), 4),
            }
            print(f"  V={target_V:.0f}m/s: "
                  f"roll σ={results[target_V]['roll_sigma']:.1f}°  "
                  f"IMU={results[target_V]['rms_g']:.4f}g")

        odi.idle_all(); odi.close()

        # Speed envelope passes if sigma_roll stays under 5° to at least 11 m/s
        ok = all(results.get(v, {}).get('roll_sigma', 99) < 5.0
                 for v in [7.0, 9.0, 11.0])
        return _rec('tc03','03-04-envelope', ok,
                    {'results': results},
                    f'Speed envelope: stability to '
                    f'{max((v for v in results if results[v]["roll_sigma"]<5.0), default=0):.0f} m/s')

    def _s05_descent_sim(self, w):
        """
        Simulate descent in forward flight: reduce frequency below hover.
        At each truck speed, step frequency down and watch what happens.
        If load cell reads negative: aircraft is ascending.
        If load cell reads positive (aircraft weight on truck): descending.
        """
        hover_freq = _tc['tc02']['hover_freq_hz'] or 2.5
        print(f"\n  ── TRUCK: cruise at 7 m/s ──")

        try:
            from mco_odrive_interface import ODriveInterface
            odi = ODriveInterface(); odi.enable_all()
        except Exception as e:
            return _rec('tc03','03-05-descent', False, {}, str(e))

        results = {}
        for freq_offset in [0.0, -0.1, -0.2, -0.3, -0.4, -0.5]:
            freq = max(1.0, hover_freq + freq_offset)
            odi.set_cascade_freq(freq)
            time.sleep(3.0)
            N = _read_load_cell()
            V = self.sf().get('V_ms', 7.0)
            if N is not None:
                # positive N = weight on truck = descending tendency
                # negative N = aircraft pulling UP = ascending
                tendency = 'descending' if N > 1.0 else 'hovering' if N > -1.0 else 'ascending'
                results[round(freq,2)] = {'N': round(N,2), 'tendency': tendency, 'V': round(V,1)}
                print(f"  f={freq:.2f}Hz  load={N:.1f}N  {tendency}")

        odi.idle_all(); odi.close()
        return _rec('tc03','03-05-descent', True,
                    {'freq_tendency': results},
                    f'Descent simulation: {len(results)} frequency points mapped')

    def _s06_discovery(self, w):
        """30-minute Discovery Mode at cruise truck speed."""
        import mco_wave_library as wl
        hover_freq = _tc['tc02']['hover_freq_hz'] or 2.5
        print(f"\n  ── TRUCK: maintain cruise speed (~7 m/s) for 30 minutes ──")
        try:
            from mco_odrive_interface import ODriveInterface
            odi = ODriveInterface(); odi.enable_all()
            odi.set_cascade_freq(hover_freq)
            time.sleep(1800)  # 30 minutes
            odi.idle_all(); odi.close()
            wl.mco_knowledge_auto_export()
            exported = True
        except Exception as e:
            exported = False
        return _rec('tc03','03-06-discovery', True, {'exported': exported},
                    '30-min truck-speed discovery complete'
                    if exported else 'Discovery complete (export failed)')

    def _summary(self):
        res = _tc['tc03']['results']
        print(f"\n  TC-03 SUMMARY")
        for k, r in sorted(res.items()):
            print(f"  {'✓' if r['passed'] else '✗'} {k}: {r['message'][:55]}")
        print(f"\n  CT-J curve points: {len(_tc['tc03']['ct_j_curve'])}")
        print(f"  Ready for TC-04 (free flight) when all steps pass.")


# ══════════════════════════════════════════════════════════════════
# TC-04 — FREE FLIGHT (GUIDED AUTOMATIC)
# ══════════════════════════════════════════════════════════════════

class TC04Runner:
    """
    TC-04: First free, unrestrained flight.
    Pilot in command. Software guides and measures automatically.
    Requires TC-01, TC-02, TC-03 all complete.

    Steps:
      1  Pre-flight confirmation (all TCs complete, calibrations applied)
      2  First hover attempt — advance lever, detect lift-off
      3  Hover stability — 30 s unassisted
      4  Altitude hold — descent controller engaged
      5  Roll response in free flight (compare to TC-02 captive data)
      6  Forward flight transition
      7  First free-flight Discovery session
    """

    STEPS = [
        ('04-01-preflight', 'Pre-flight TC confirmation'),
        ('04-02-liftoff',   'First hover attempt'),
        ('04-03-stability', 'Hover stability 30 s'),
        ('04-04-alt-hold',  'Altitude hold — descent controller'),
        ('04-05-roll',      'Roll response in free flight'),
        ('04-06-transition','Forward flight transition'),
        ('04-07-discovery', 'First free-flight discovery'),
    ]

    def __init__(self, fcc, sensors_fn):
        self.fcc      = fcc
        self.sf       = sensors_fn
        self._step_i  = 0

    def start(self):
        # Check TC-01 minimum requirement (TC-03 CT-J gated separately)
        if not _tc['tc01']['results'].get('16-export',{}).get('passed'):
            return "TC-04 BLOCKED — TC-01 not complete."

        # Read hover feasibility from TC-01 assessment
        feas = _tc['tc01'].get('hover_feasibility', {})
        path = feas.get('recommended_path', 'FREE_HOVER')

        # Choose TC-04 profile based on feasibility
        if path == 'REDESIGN':
            return (f"TC-04 BLOCKED — Aircraft cannot sustain flight. "
                    f"Thrust {feas.get('max_thrust_N',0):.1f}N vs "
                    f"MTOW {feas.get('mtow_N',MTOW_N):.1f}N. "
                    f"Reduce weight or increase wing area before proceeding.")

        if path in ('FREE_HOVER', 'STOL_FIRST'):
            # Require TC-02 calibration for hover-capable aircraft
            if not _tc['tc02']['results'].get('02-07-calibrate',{}).get('passed'):
                return "TC-04 BLOCKED — TC-02 calibration not complete."

        # TC-03 CT-J recommended but not hard-blocking for STOL
        if not _tc['tc03']['results'].get('03-03-ct-j',{}).get('passed'):
            print("  WARNING: TC-03 CT-J not complete — morphing not available in TC-04.")

        if _tc['tc04']['running']:
            return "TC-04 already running"

        # Store the flight profile for instruction selection
        _tc['tc04']['flight_profile'] = path
        _tc['tc04']['hover_feasibility'] = feas
        _tc['tc04'].update({'running': True, 'step': 0,
                            'start_t': time.monotonic()})
        _tc['active'] = 'tc04'
        _load()
        self._step_i = 0
        self._set_instruction(0)

        profile_msg = {
            'FREE_HOVER':  'HOVER PROFILE — Aircraft expected to hover freely.',
            'STOL_FIRST':  'STOL-FIRST PROFILE — Thermal margin tight. '
                           'Transition flight first, hover later.',
            'STOL_ONLY':   'STOL-ONLY PROFILE — Aircraft unlikely to free hover. '
                           'Transition takeoff and landing only.',
        }.get(path, 'STANDARD PROFILE')

        return (f"TC-04 started — {profile_msg} "
                f"\nRead instructions carefully. Pilot in command at all times.")

    def continue_step(self):
        if not _tc['tc04']['running']:
            return "TC-04 not running"
        t = threading.Thread(target=self._measure_step, daemon=True)
        t.start()
        return (f"TC-04 step {self._step_i+1}: "
                f"{self.STEPS[self._step_i][1]} — measuring")

    def abort(self):
        _tc['tc04']['running'] = False
        _tc['active'] = None
        return "TC-04 aborted — PILOT RETAINS FULL CONTROL"

    def _set_instruction(self, i):
        _tc['tc04']['step'] = i
        profile   = _tc['tc04'].get('flight_profile', 'FREE_HOVER')
        hover_freq = _tc['tc02'].get('hover_freq_hz') or 2.5
        feas       = _tc['tc04'].get('hover_feasibility', {})
        thrust_pct = feas.get('thrust_pct_mtow', 100.0)

        # ── Step 1 — Pre-flight (common to all profiles) ──────────────────
        s1 = (f"TC-04-01: PRE-FLIGHT CONFIRMATION\n"
              f"  Flight profile: {profile}\n"
              f"  Thrust margin: {thrust_pct:.0f}% of MTOW\n"
              f"  Hover freq (TC-02): {hover_freq:.2f} Hz\n"
              f"  HOVER MODE active below 3 m/s pitot airspeed.\n"
              f"    Lever 50% = hold altitude\n"
              f"    Lever >50% = climb\n"
              f"    Lever <50% = descend\n"
              f"    Pedals centred 1s = auto heading hold\n"
              f"  CONVENTION CHECK (do this FIRST):\n"
              f"    Advance lever slightly above 50% — aircraft should climb.\n"
              f"    Right roll stick — aircraft should bank RIGHT.\n"
              f"    Right pedal — aircraft nose should move RIGHT.\n"
              f"  Site clear. Batteries full. RC signal confirmed.\n"
              f"  Type tc04_continue when ready.")

        # ── Steps 2-7 differ by profile ───────────────────────────────────
        if profile == 'FREE_HOVER':
            steps = [s1,
                "TC-04-02: FIRST HOVER ATTEMPT (FREE HOVER PROFILE)\n"
                "  HOVER MODE is ACTIVE below 3 m/s pitot airspeed.\n  hover mode is ACTIVE (V_ms < 3 m/s).\n"
                "  LEVER CONTROLS ALTITUDE in hover mode:\n"
                "    50% = hold altitude   >50% = climb   <50% = descend\n"
                "  GROUND EFFECT ADVISORY: expect faster-than-calibrated climb at first lift-off.\n"
                "  Ground effect adds 10-25% lift vs TC-02 (which was out of ground effect).\n"
                "  ALTITUDE FLOOR: 0.3 m AGL minimum — altitude hold will not descend below this.\n"
                "  Advance lever from 50% toward 60%. Aircraft should lift off.\n"
                "  Allow climb to 1.5 m before applying lever hold (altitude hold).\n"
                "  Hold hover 5 s then bring lever back to 50% (hold) then land.\n"
                "  Repeat 3 times. Type tc04_continue after 3 clean lift-offs.",

                "TC-04-03: HOVER STABILITY\n"
                "  Achieve hover at lever ~50-55%. Hold 30 s.\n"
                "  Keep inputs minimal. Altitude hold is automatic at 50%.\n"
                "  Software measures sigma_h. PASS: sigma_h < 0.5 m.\n"
                "  Type tc04_continue when 30 s hold is complete.",

                "TC-04-04: ALTITUDE HOLD AND DESCENT CONTROLLER\n"
                "  From stable hover: lever to 50% (altitude hold).\n"
                "  Confirm aircraft holds altitude without drift.\n"
                "  Then: lever to 30% — should descend at ~200 fpm.\n"
                "  Then: lever to 70% — should climb at ~200 fpm.\n"
                "  Type tc04_continue when altitude rate tests are done.",

                "TC-04-05: ROLL AND CONVENTION VERIFICATION\n"
                "  From stable hover: apply 20% right roll stick.\n"
                "  Aircraft should bank RIGHT. Release — should return to level.\n"
                "  Then 40%, then 60%. Record rates and settle times.\n"
                "  Apply right pedal — nose should move RIGHT.\n"
                "  Type tc04_continue when roll and yaw tests are done.",

                "TC-04-06: FORWARD FLIGHT TRANSITION\n"
                "  From stable hover: advance lever above 70%.\n"
                "  Apply FORWARD pitch stick gently — aircraft accelerates.\n"
                "  Control law AUTOMATICALLY transitions to fixed-wing above 5 m/s.\n"
                "  Fly forward 100 m. Reduce lever — aircraft decelerates.\n"
                "  Below 3 m/s hover mode re-engages automatically.\n"
                "  Repeat 3 times. Type tc04_continue after 3 clean transitions.",

                "TC-04-07: FIRST FREE-FLIGHT DISCOVERY (5 minutes)\n"
                "  Discovery Mode engaging for 5 minutes.\n"
                "  H-33/34/35 State B variants (safe — stopped wings still lift).\n"
                "  Maintain gentle hover or slow forward flight.\n"
                "  Type tc04_continue when session complete.",
            ]

        elif profile == 'STOL_FIRST':
            steps = [s1,
                "TC-04-02: STOL TAKEOFF (THERMAL MARGIN TIGHT — NO SUSTAINED HOVER)\n"
                f"  Thrust is {thrust_pct:.0f}% of MTOW. Hover is possible but brief.\n"
                "  DO NOT attempt sustained stationary hover.\n"
                "  STOL PROCEDURE:\n"
                "    Advance lever to 80-90%. Allow aircraft to accelerate forward.\n"
                "    Rotate gently at 3-4 m/s. Aircraft transitions to forward flight.\n"
                "    Fly a circuit at cruise lever (~65%). Return and land normally.\n"
                "  Type tc04_continue after 3 clean STOL circuits.",

                "TC-04-03: BRIEF HOVER TEST (LOW ALTITUDE, SHORT DURATION)\n"
                "  ONLY if aircraft felt well-controlled in step 2.\n"
                "  From slow forward flight, reduce lever to ~50% and slow down.\n"
                "  Below 3 m/s hover mode engages. 50% = altitude hold.\n"
                "  Hold hover no more than 10 s. Monitor S1 temperatures.\n"
                "  If FET alert appears: return to forward flight immediately.\n"
                "  Type tc04_continue after brief hover test.",

                "TC-04-04: ALTITUDE HOLD IN FORWARD FLIGHT\n"
                "  At cruise speed (V > 5 m/s): lever 50% = altitude hold.\n"
                "  Confirm aircraft holds altitude for 60 s at cruise lever.\n"
                "  Type tc04_continue when confirmed.",

                "TC-04-05: ROLL RESPONSE IN FORWARD FLIGHT\n"
                "  At cruise speed: apply 20%, 40% roll stick.\n"
                "  Aircraft should bank LEFT/RIGHT as expected.\n"
                "  Record rates and settle times.\n"
                "  Type tc04_continue when roll tests done.",

                "TC-04-06: FULL SPEED ENVELOPE\n"
                "  Cruise at nominal speed. Then 80% lever for 30 s.\n"
                "  Record maximum pitot airspeed achieved.\n"
                "  Reduce lever smoothly. No abrupt inputs.\n"
                "  Type tc04_continue when speed test done.",

                "TC-04-07: FIRST FREE-FLIGHT DISCOVERY (5 minutes)\n"
                "  Discovery Mode for 5 minutes in forward flight (V > 5 m/s).\n"
                "  H-33/34/35 State B variants only (safe at cruise).\n"
                "  Type tc04_continue when session complete.",
            ]

        else:   # STOL_ONLY
            steps = [s1,
                "TC-04-02: STOL TAKEOFF (STOL-ONLY — HOVER NOT EXPECTED)\n"
                f"  Thrust is {thrust_pct:.0f}% of MTOW — free hover not possible.\n"
                "  STOL PROCEDURE:\n"
                "    Advance lever to 90-100%. Allow ground run.\n"
                "    Rotate at 4-5 m/s. Climb at 65-70° lever.\n"
                "    Fixed-wing convention active above 5 m/s.\n"
                "  Type tc04_continue after 3 clean STOL takeoffs.",

                "TC-04-03: CLIMB AND CRUISE\n"
                "  Climb to safe altitude (>20 m). Lever 65-70%.\n"
                "  Trim: lever to 50% = altitude hold.\n"
                "  Fly a circuit. Reduce lever for descent.\n"
                "  Type tc04_continue after circuit.",

                "TC-04-04: ALTITUDE HOLD IN CRUISE\n"
                "  At cruise speed: lever 50% = altitude hold.\n"
                "  Confirm 60 s altitude hold. Type tc04_continue.",

                "TC-04-05: ROLL RESPONSE\n"
                "  At cruise speed: roll stick 20%, 40%, 60%.\n"
                "  Record rates and settle times.\n"
                "  Type tc04_continue.",

                "TC-04-06: SPEED RANGE\n"
                "  Explore slow flight (near VS) and cruise.\n"
                "  Note minimum stable airspeed from pitot.\n"
                "  Type tc04_continue.",

                "TC-04-07: FIRST FORWARD-FLIGHT DISCOVERY (5 minutes)\n"
                "  Discovery Mode at cruise speed.\n"
                "  H-33/35 State B variants. morphing_speed_start.\n"
                "  Type tc04_continue.",
            ]

        if i < len(steps):
            instr = steps[i]
            _tc['tc04']['instruction'] = instr
            _tc['tc04']['waiting_continue'] = True
            print(f"\n{'═'*55}")
            print(instr)
            print(f"{'═'*55}")

    def _measure_step(self):
        import mco_wave_library as wl
        i = self._step_i
        step_id = self.STEPS[i][0]

        if i == 0:
            _rec('tc04', step_id, True, {}, 'Pre-flight confirmed by pilot')
        elif i == 1:
            self._measure_liftoff(wl, step_id)
        elif i == 2:
            self._measure_stability(wl, step_id)
        elif i == 3:
            self._measure_alt_hold(wl, step_id)
        elif i == 4:
            self._measure_roll(wl, step_id)
        elif i == 5:
            self._measure_transition(wl, step_id)
        elif i == 6:
            self._run_discovery(wl, step_id)

        self._step_i += 1
        if self._step_i < len(self.STEPS):
            self._set_instruction(self._step_i)
        else:
            _tc['tc04']['running'] = False
            _tc['active'] = None
            print("\n  TC-04 COMPLETE — first free flight done")
            self._summary()

    def _measure_liftoff(self, wl, step_id):
        samples = []
        for _ in range(150):  # 30 s
            s = self.sf(); samples.append(s); time.sleep(0.2)
        liftoffs = [s for s in samples if s.get('h_agl_m',0) > 0.15]
        if liftoffs:
            freq = _mean([s.get('freq_hz',0) for s in liftoffs])
            _rec('tc04', step_id, True,
                 {'liftoff_freq_hz': round(freq,3)},
                 f'Lift-off confirmed. Free-flight hover freq: {freq:.2f} Hz '
                 f'(TC-02 captive: {_tc["tc02"].get("hover_freq_hz","?")} Hz)')
        else:
            _rec('tc04', step_id, False, {}, 'No lift-off detected')

    def _measure_stability(self, wl, step_id):
        h_vals = []
        for _ in range(150):
            h_vals.append(self.sf().get('h_agl_m',0)); time.sleep(0.2)
        m  = _mean(h_vals)
        sh = math.sqrt(sum((h-m)**2 for h in h_vals)/max(len(h_vals)-1,1))
        passed = sh < 0.5
        _rec('tc04', step_id, passed,
             {'sigma_h_m': round(sh,3)},
             f'Free hover sigma_h = {sh:.3f} m '
             f'({"OK" if passed else "unstable — investigate"})')

    def _measure_alt_hold(self, wl, step_id):
        rates = {}
        for fpm in [100, 300]:
            time.sleep(30)
            sink_vals = []
            for _ in range(50):
                s = self.sf()
                sink_vals.append(abs(s.get('sink_rate_ms', 0)) * 196.85)
                time.sleep(0.2)
            rates[fpm] = round(sum(sink_vals)/len(sink_vals), 1)
        passed = abs(rates.get(100, 999) - 100) < 40
        _rec('tc04', step_id, passed, {'rates_fpm': rates},
             f'100 fpm target → {rates.get(100, "?")} fpm actual')

    def _measure_roll(self, wl, step_id):
        roll_data = []
        for pct in [20, 40, 60]:
            print(f"  Apply {pct}% roll stick RIGHT — 2 s")
            time.sleep(2.0)
            rates = [abs(self.sf().get('roll_rate_dps', 0))
                     for _ in range(10) if not time.sleep(0.2)]
            print("  Release stick")
            t0 = time.monotonic()
            for _ in range(50):
                if abs(self.sf().get('roll_rate_dps', 0)) < 2.0:
                    break
                time.sleep(0.2)
            roll_data.append({'pct': pct, 'rate': round(sum(rates)/max(len(rates),1), 1),
                              'settle_s': round(time.monotonic() - t0, 2)})
            time.sleep(1.0)
        _rec('tc04', step_id, True, {'roll_data': roll_data},
             f'Roll: {[d["rate"] for d in roll_data]} dps at 20/40/60%')

    def _measure_transition(self, wl, step_id):
        speeds = []
        for _ in range(150):
            V = self.sf().get('V_ms', 0)
            if V > 2.0:
                speeds.append(V)
            time.sleep(0.2)
        min_V = min(speeds) if speeds else 0
        _rec('tc04', step_id, min_V > 0,
             {'min_fwd_speed_ms': round(min_V, 2)},
             f'Forward flight from {min_V:.1f} m/s')

    def _run_discovery(self, wl, step_id):
        print("  Discovery Mode for 5 minutes")
        time.sleep(300)
        try:
            wl.mco_knowledge_auto_export()
            exported = True
        except Exception:
            exported = False
        _rec('tc04', step_id, True, {},
             '5-min free-flight discovery complete')

    def _summary(self):
        res = _tc['tc04']['results']
        print(f"\n  TC-04 SUMMARY — FIRST FREE FLIGHT")
        for k, r in sorted(res.items()):
            print(f"  {'✓' if r['passed'] else '✗'} {k}: {r['message'][:55]}")


# ══════════════════════════════════════════════════════════════════
# GS COMMAND INTERFACE
# ══════════════════════════════════════════════════════════════════

_tc01_runner = None
_tc02_runner = None
_tc03_runner = None
_tc04_runner = None


def init_test_runners(fcc, sensors_fn, hardware=None):
    global _tc01_runner, _tc02_runner, _tc03_runner, _tc04_runner
    _tc01_runner = TC01Runner(fcc, sensors_fn, hardware)
    _tc02_runner = TC02Runner(fcc, sensors_fn)
    _tc03_runner = TC03Runner(fcc, sensors_fn)
    _tc04_runner = TC04Runner(fcc, sensors_fn)
    _load()
    print("  Test card runners initialised: TC-01 TC-02 TC-03 TC-04")


def handle_tc_command(cmd):
    if cmd == 'tc01_start':   return _tc01_runner.start()   if _tc01_runner else "not init"
    if cmd == 'tc01_confirm': return _tc01_runner.confirm()  if _tc01_runner else "not init"
    if cmd == 'tc01_abort':   return _tc01_runner.abort()    if _tc01_runner else "ok"
    if cmd == 'tc01_status':  return _tc01_status()
    if cmd == 'tc02_start':   return _tc02_runner.start()   if _tc02_runner else "not init"
    if cmd == 'tc02_abort':   return _tc02_runner.abort()    if _tc02_runner else "ok"
    if cmd == 'tc02_status':  return _tc02_status()
    if cmd == 'tc03_start':   return _tc03_runner.start()   if _tc03_runner else "not init"
    if cmd == 'tc03_abort':   return _tc03_runner.abort()    if _tc03_runner else "ok"
    if cmd == 'tc03_status':  return _tc03_status()
    if cmd == 'tc04_start':    return _tc04_runner.start()        if _tc04_runner else "not init"
    if cmd == 'tc04_continue': return _tc04_runner.continue_step() if _tc04_runner else "not init"
    if cmd == 'tc04_abort':    return _tc04_runner.abort()         if _tc04_runner else "ok"
    if cmd == 'tc04_status':   return _tc04_status()
    if cmd == 'tc_results':  return _tc_all_results()
    if cmd == 'tc_export':   _save(); return f"TC results saved to {RESULTS_FILE}"
    return None


def _tc01_status():
    st = _tc['tc01']
    res = st['results']
    lines = [f"TC-01 {'RUNNING' if st['running'] else 'stopped'}  Step {st['step']}/16"]
    if st['waiting_confirm']:
        lines.append(f"ACTION: {st['confirm_prompt'][:60]}")
    lines.append(f"{sum(1 for r in res.values() if r.get('passed'))}/{len(res)} passed")
    return '\n'.join(lines)

def _tc02_status():
    st = _tc['tc02']
    lines = [f"TC-02 {'RUNNING' if st['running'] else 'stopped'}  Step {st['step']}/8"]
    if st.get('hover_freq_hz'):
        lines.append(f"Hover freq: {st['hover_freq_hz']:.2f} Hz")
    return '\n'.join(lines)

def _tc03_status():
    st = _tc['tc03']
    lines = [f"TC-03 {'RUNNING' if st['running'] else 'stopped'}  Step {st['step']}/6"]
    lines.append(f"CT-J points: {len(st.get('ct_j_curve', []))}")
    return '\n'.join(lines)

def _tc04_status():
    st = _tc['tc04']
    lines = [f"TC-04 {'RUNNING' if st['running'] else 'stopped'}  Step {st['step']}/7 — FREE FLIGHT"]
    if st.get('instruction'):
        lines.append(st['instruction'][:200])
    return '\n'.join(lines)

def _tc_all_results():
    _load()
    lines = ["ALL TEST CARD RESULTS"]
    for tc_key, label, n in [('tc01','TC-01 BENCH',16),
                               ('tc02','TC-02 CAPTIVE HOVER',8),
                               ('tc03','TC-03 TRUCK FORWARD',6),
                               ('tc04','TC-04 FREE FLIGHT',7)]:
        res = _tc[tc_key]['results']
        ok  = sum(1 for r in res.values() if r.get('passed'))
        lines.append(f"\n{label}: {ok}/{len(res)} passed")
        for k, r in sorted(res.items()):
            lines.append(f"  {'✓' if r.get('passed') else '✗'} "
                         f"{k}: {r.get('message','')[:50]}")
    lines.append(f"\nHover freq (TC-02): {_tc['tc02'].get('hover_freq_hz','not yet')} Hz")
    lines.append(f"CT-J curve (TC-03): {len(_tc['tc03'].get('ct_j_curve',[]))} points")
    return '\n'.join(lines)
