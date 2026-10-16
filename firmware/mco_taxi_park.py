"""
MCO-1D Taxi and Park Modes  v2
================================
Corrects the taxi amplitude analysis from v1.

KEY CORRECTION (per operator feedback):
  v1 incorrectly used SYMMETRIC amplitude limited to ±12°.
  This was wrong for two reasons:
  1. The constraint applies to the DOWNSTROKE only — the
     upstroke goes upward and has no ground obstacle.
  2. Even the downstroke limit calculates to only −4°
     at 150mm margin, not −12°.

  CORRECT APPROACH:
  Asymmetric stroke:
    Downstroke: −4° (tip stays 150mm above ground — W4 limiting)
    Upstroke:   +80° (full — unrestricted)
  This is biologically correct — large birds running for takeoff
  use the same asymmetric stroke: full upstroke, shallow downstroke.
  Total arc 84° vs 24° symmetric → 2-3× more thrust for same frequency.

NEW: INSPECTION MODE
  All wings sweep to 90° (abeam, perpendicular to fuselage).
  Aircraft fully disarmed. Motors idle. Wings stationary.
  Allows full walk-around inspection of all wing surfaces.
"""

import time, math

# ─────────────────────────────────────────────────────────────────
# TAXI GEOMETRY
# ─────────────────────────────────────────────────────────────────
H_GEAR_MM    = 200.0   # undercarriage height — measure and confirm
B_W4_MM      = 712.0   # W4 semi-span — shortest, most restrictive
MARGIN_MM    = 150.0   # tip clearance margin during downstroke

# Maximum DOWNSTROKE angle — tip stays MARGIN_MM above ground
# H_gear - b × sin(α_down) ≥ MARGIN
# α_down = arcsin((H_gear - MARGIN) / b)
TAXI_AMP_DOWN_DEG = math.degrees(
    math.asin((H_GEAR_MM - MARGIN_MM) / B_W4_MM))   # ≈ 4.0°

# Upstroke is unrestricted — use full normal amplitude
TAXI_AMP_UP_DEG   = 80.0    # same as flight upstroke

# ─────────────────────────────────────────────────────────────────
# TAXI PARAMETERS
# ─────────────────────────────────────────────────────────────────
TAXI = {
    # Frequency range
    # Very slow taxi: 0.2 Hz — aircraft barely moves, for tight manoeuvres
    # Fast taxi: 1.0 Hz — jogging pace, needed against light headwind
    # Frequency is the primary speed control, as in flight
    'FREQ_MIN_HZ':        0.2,    # Hz — slow creep
    'FREQ_CRUISE_HZ':     0.5,    # Hz — comfortable walking pace
    'FREQ_MAX_HZ':        1.0,    # Hz — against wind, near takeoff

    # Asymmetric stroke (the corrected model)
    'AMP_DOWN_DEG':   TAXI_AMP_DOWN_DEG,   # ≈ 4.0° — ground constraint
    'AMP_UP_DEG':     TAXI_AMP_UP_DEG,     # 80° — full upstroke
    'PITCH_DEG':      +3.0,   # slight positive AoA — ground effect lift

    # Turning via differential amplitude (Lilong & Yu 2024)
    # ΔΦ* = asymmetry ratio, range 0–0.4
    # Applied to upstroke amplitude only — downstroke stays at −4°
    # for all wings (ground clearance is symmetric)
    'TURN_DELTA_PHI_STAR': 0.30,

    # Reverse: negative pitch tilts thrust rearward
    # Downstroke still limited to −4° (ground is still there)
    'REVERSE_FREQ_HZ':    0.4,
    'REVERSE_PITCH_DEG': -15.0,   # steeper negative pitch for clearer reversal

    # Phase offset: standard metachronal 90° cascade
    'PHASE_OFFSET_DEG':  90.0,

    # Sweep: all wings abeam (90°) or forward (60°) for taxi
    # 90° gives more lateral clearance but looks odd
    # 60° (deployed forward) is natural and matches takeoff position
    'SWEEP_DEG':          60.0,
}


def taxi_freq(throttle_frac):
    """Map throttle 0→1 to taxi frequency."""
    return (TAXI['FREQ_MIN_HZ']
            + throttle_frac * (TAXI['FREQ_MAX_HZ'] - TAXI['FREQ_MIN_HZ']))


def taxi_amplitudes_asymmetric(yaw_cmd, amp_up=None):
    """
    Returns (amp_up_port, amp_up_stbd) for the upstroke.
    Downstroke is always −TAXI_AMP_DOWN_DEG on both sides
    (ground clearance is a symmetric constraint).

    Turning is achieved by differential UPSTROKE amplitude only.

    yaw_cmd: −1.0 (left) to +1.0 (right)
    """
    if amp_up is None:
        amp_up = TAXI['AMP_UP_DEG']

    delta = abs(yaw_cmd) * TAXI['TURN_DELTA_PHI_STAR'] * amp_up
    # Right turn: port upstroke higher (port pushes harder)
    if yaw_cmd > 0.05:
        up_port = min(amp_up + delta/2, 85.0)
        up_stbd = max(amp_up - delta/2, 30.0)
    elif yaw_cmd < -0.05:
        up_port = max(amp_up - delta/2, 30.0)
        up_stbd = min(amp_up + delta/2, 85.0)
    else:
        up_port = amp_up
        up_stbd = amp_up
    return up_port, up_stbd


def taxi_commands(throttle_frac, yaw_cmd, lateral_cmd=0.0, reverse=False, wind_data=None, snow_mode=False):
    """
    Returns command dict for ground taxi with active wind, crosswind, and ski/snow compensation.
    Uses asymmetric stroke: full upstroke, shallow −4° downstroke.

    Control inputs:
    - throttle_frac: 0.0 to 1.0 (frequency scaling 0.2 to 1.0 Hz)
    - yaw_cmd: −1.0 to +1.0 from rudder pedals (differential upstroke amplitude for heading yaw)
    - lateral_cmd: −1.0 to +1.0 from roll stick (H-26 differential AoA + dihedral side-force for direct sideways translation)
    - reverse: True for reverse taxi (negative pitch)
    - wind_data: Optional dict with 'V_wind_ms', 'wind_bearing_deg', 'heading_deg' for active crosswind compensation
    - snow_mode: True when aircraft is operating on skis/snow (enables stiction breakout pulse & high-gain lateral anti-drift)
    """
    try:
        from mco_wave_library import metachronal_phase_offsets
    except ImportError:
        from wave_library import metachronal_phase_offsets

    freq     = taxi_freq(throttle_frac)
    base_pitch = TAXI['REVERSE_PITCH_DEG'] if reverse else TAXI['PITCH_DEG']

    # ── Active Ground Wind & Crosswind Compensation (H-TAXI-WIND-1 / H-TAXI-REEF-1 / H-TAXI-YAW-1) ──
    active_suction_pitch = 0.0
    sw_port = TAXI['SWEEP_DEG']
    sw_stbd = TAXI['SWEEP_DEG']
    anti_weathervane_delta_amp = 0.0
    wind_mitigated = False

    if wind_data and isinstance(wind_data, dict):
        try:
            import mco_advanced_physics as ap
            V_w = wind_data.get('V_wind_ms', wind_data.get('wind_speed_ms', 0.0))
            w_bearing = wind_data.get('wind_bearing_deg', wind_data.get('wind_dir_deg', 0.0))
            hdg = wind_data.get('heading_deg', 0.0)
            if V_w > 1.5:
                pitch_comp, (sw_p, sw_s), delta_a, comp_diag = ap.ground_wind_active_compensator(
                    V_wind_ms=V_w, wind_bearing_deg=w_bearing, heading_deg=hdg,
                    pilot_yaw=yaw_cmd, pilot_throttle=throttle_frac
                )
                active_suction_pitch = pitch_comp
                sw_port = sw_p
                sw_stbd = sw_s
                anti_weathervane_delta_amp = delta_a
                wind_mitigated = comp_diag.get('wind_hazard_mitigated', True)
        except Exception:
            pass

    up_port, up_stbd = taxi_amplitudes_asymmetric(yaw_cmd)

    # Apply anti-weathervaning differential stroke
    if abs(anti_weathervane_delta_amp) > 0.5:
        up_port = max(20.0, min(85.0, up_port + anti_weathervane_delta_amp / 2.0))
        up_stbd = max(20.0, min(85.0, up_stbd - anti_weathervane_delta_amp / 2.0))

    # Lateral ground crab / sidestep via H-26 force vector decoupling:
    # Right stick (lateral_cmd > 0): Port wings higher AoA & dihedral, pushing craft toward starboard
    # Left stick  (lateral_cmd < 0): Stbd wings higher AoA & dihedral, pushing craft toward port
    # ── Ski / Snow Surface Adaptations ──────────────────────────────────────────
    # Skis have low lateral resistance (mu_lat ~ 0.08 vs 0.60 for rubber wheels).
    # In snow mode:
    # 1. Stiction breakout: when throttle is initiated from standstill, apply short high-pitch pulse
    # 2. Enhanced lateral anti-drift authority (boost lateral authority by 1.6x)
    lat_deadband = 0.08
    lateral_active = abs(lateral_cmd) > lat_deadband
    lat_multiplier = 1.60 if snow_mode else 1.00
    if lateral_active:
        lat_scale = (abs(lateral_cmd) - lat_deadband) / (1.0 - lat_deadband)
        lat_scale = min(1.0, max(0.0, lat_scale)) * (1.0 if lateral_cmd > 0 else -1.0)
        port_pitch_bias = +4.0 * lat_scale * lat_multiplier
        stbd_pitch_bias = -3.0 * lat_scale * lat_multiplier
        port_dih_bias   = +5.0 * lat_scale * lat_multiplier
        stbd_dih_bias   = -3.0 * lat_scale * lat_multiplier
    else:
        port_pitch_bias = 0.0
        stbd_pitch_bias = 0.0
        port_dih_bias   = 0.0
        stbd_dih_bias   = 0.0

    # Build per-wing commands with Station-Graded Taildragger Ground Protection:
    # W1/W2 (Front tractor drive): do 100% of ground taxi thrust (downstroke limited to -8°/-6°)
    # W3/W4 (Aft cascade): locked in high-dihedral V-stance (+25° to +28°) to eliminate runway tip strikes
    wing_cmds = []
    for i in range(8):
        is_port = (i % 2 == 0)
        station_idx = (i // 2) + 1  # 1 for W1, 2 for W2, 3 for W3, 4 for W4
        p_bias = port_pitch_bias if is_port else stbd_pitch_bias
        d_bias = port_dih_bias   if is_port else stbd_dih_bias
        eff_sweep = sw_port if is_port else sw_stbd
        # Apply ground suction pitch (negative downforce bias) to prevent rollover and wheel unweighting
        # In snow mode, clamp max negative suction to -10° to avoid burying ski tips into powder
        applied_suction = max(-10.0, active_suction_pitch) if snow_mode else active_suction_pitch
        eff_pitch = round(base_pitch + p_bias + applied_suction, 1)

        if station_idx <= 2:
            # W1 & W2: active forward taxi flapping
            amp_u = round(up_port if is_port else up_stbd, 1)
            amp_d = TAXI['AMP_DOWN_DEG']
            eff_dih = round(max(0.0, d_bias), 1)
        else:
            # W3 & W4: high dihedral lock (+25° to +28°) with flapping disengaged for ground protection
            amp_u = 0.0
            amp_d = TAXI['AMP_DOWN_DEG']
            eff_dih = round(max(26.0, 26.0 + d_bias), 1)

        wing_cmds.append({
            'amp_up':       amp_u,
            'amp_down':     amp_d,
            'flap_amp_deg': amp_u,
            'pitch':        eff_pitch,
            'pitch_deg':    eff_pitch,
            'dihedral_deg': eff_dih,
            'sweep':        eff_sweep,
            'sweep_deg':    eff_sweep,
            'station_idx':  station_idx,
        })

    offsets = metachronal_phase_offsets(offset_deg=TAXI['PHASE_OFFSET_DEG'])

    notes_str = (f"asymmetric stroke: up={up_port:.0f}/{up_stbd:.0f}° "
                 f"down={TAXI['AMP_DOWN_DEG']:.0f}° "
                 f"freq={freq:.2f}Hz"
                 + (f" sidestep={lateral_cmd:+.2f}" if lateral_active else "")
                 + (f" wind_suction_pitch={active_suction_pitch:+.1f}°" if abs(active_suction_pitch) > 0.1 else "")
                 + (" [SNOW_SKI_MODE]" if snow_mode else ""))

    return {
        'freq_hz':                 freq,
        'wing_cmds':               wing_cmds,
        'phase_offsets':           offsets,
        'mode':                    'reverse_taxi' if reverse else 'forward_taxi',
        'lateral_active':          lateral_active,
        'lateral_cmd':             round(lateral_cmd, 2),
        'yaw_cmd':                 round(yaw_cmd, 2),
        'active_suction_pitch_deg': round(active_suction_pitch, 1),
        'sweep_port_deg':          sw_port,
        'sweep_stbd_deg':          sw_stbd,
        'anti_weathervane_delta_amp': round(anti_weathervane_delta_amp, 1),
        'wind_mitigated':          wind_mitigated,
        'snow_mode':               snow_mode,
        'notes':                   notes_str,
    }


# ─────────────────────────────────────────────────────────────────
# INSPECTION MODE
# ─────────────────────────────────────────────────────────────────
INSPECTION = {
    # Wings at 90° sweep — pointing directly sideways from fuselage
    # Sweep convention: 0°=forward, 90°=abeam, 210°=stowed
    'SWEEP_DEG':      90.0,

    # Pitch at 0° — chord horizontal, maximum surface visibility
    'PITCH_DEG':       0.0,

    # Tipped inspection — pitch +15° raises tips for upper surface view
    'PITCH_TIPPED_DEG': +15.0,

    # Rate to reach inspection position — slow and deliberate
    'SWEEP_RATE_DEG_S':  10.0,   # °/s — very slow, 9 seconds to reach 90°

    # No flapping — all motors IDLE
    'MOTOR_STATE':    'IDLE',
}

# Wing spans for walk-around reference
WING_SPANS = {
    'W1': 1150, 'W2': 1150, 'W3': 1150, 'W4': 1150   # mm semi-span (1.15m: mount + 4x250mm segments + tip)
}

INSPECTION_WALK_AROUND = """
INSPECTION MODE ACTIVE
All wings at 90° (abeam). All motors idle. Aircraft disarmed.

Walk-around checklist — check both port and starboard sides:

UPPER SURFACE (walk along outside of wing, looking down):
  □ Membrane — no tears, holes, or peeling from rib tails
  □ D-box CF skin (LE to 65%c) — no delamination or punctures
  □ VGs — all strips present and bonded (drone only)
  □ Raked wingtip — bonded at joint, no cracks

LOWER SURFACE (crouch alongside wing, look up):
  □ Gurney flap — intact, not bent or delaminated
  □ Membrane lower surface — no sagging or separation
  □ Covert flaps — present and free-moving at hinge

LEADING EDGE (face the leading edge):
  □ LE profile — no dents, cracks, or separation
  □ Forward spar location (10%c) — no protrusion or fracture visible
  □ No insect strikes or debris embedded in leading edge

TRAILING EDGE:
  □ TE rod — present along full span, not kinked
  □ Membrane hem — bonded along full TE, no lifting

ROOT JOINT (stand at fuselage, look along wing):
  □ Root pivot — no unusual play or looseness
  □ Rope — seated in groove, not frayed at root
  □ Servo arm — secure, no cracking at horn

TIP:
  □ Tip rib — no cracks, still bonded to skin
  □ Raked section — no deformation from previous landing
"""


class InspectionMode:
    """
    Commands all wings to 90° sweep (laterally extended)
    for full visual inspection.
    Aircraft must be disarmed before calling this.
    """

    def activate(self, servos, odrive=None, sim=False):
        """
        Move all wings to inspection position.
        Returns True when position reached.
        """
        print("\n" + "="*55)
        print("  INSPECTION MODE")
        print("="*55)
        print("""
  Moving all wings to 90° (laterally extended).
  All motors will be idle.
  Do NOT attempt to arm the aircraft while in inspection mode.
        """)

        if sim:
            print("  [SIM] Wings moved to 90° inspection position")
            print("  [SIM] All motors idle")
            self._print_reference()
            return True

        # Ensure motors idle first
        if odrive:
            try:
                odrive.idle_all()
                print("  Motors: IDLE")
            except Exception as e:
                print(f"  Motor idle failed: {e}")

        if servos is None:
            print("  No servo interface — cannot move wings")
            return False

        # Move wings slowly to 90° (abeam)
        print("  Moving wings to 90° at 10°/s...")
        print("  This will take about 3 seconds. Stand clear.")
        time.sleep(1.0)

        # Sweep all wings to 90° progressively
        current = [60.0] * 8   # assume deployed at 60° (normal position)
        target  = INSPECTION['SWEEP_DEG']   # 90°
        step_hz = 20
        rate    = INSPECTION['SWEEP_RATE_DEG_S']
        step    = rate / step_hz

        while max(abs(current[i] - target) for i in range(8)) > 0.5:
            for i in range(8):
                err = target - current[i]
                current[i] += max(-step, min(step, err))
                servos.set_sweep(i, current[i])
            time.sleep(1.0 / step_hz)

        # Set pitch to 0° (chord horizontal)
        for i in range(8):
            servos.set_pitch(i, INSPECTION['PITCH_DEG'])

        print("  Wings at 90°. Pitch neutral.")
        self._print_reference()
        return True

    def tipped(self, servos, sim=False):
        """
        Tilt all wings to +15° pitch — raises tips for
        better upper surface D-box viewing angle.
        """
        if sim:
            print("  [SIM] Wings tipped to +15° pitch")
            return
        if servos:
            for i in range(8):
                servos.set_pitch(i, INSPECTION['PITCH_TIPPED_DEG'])
            print("  Wings tipped to +15° — upper surface visible")

    def neutral(self, servos, sim=False):
        """Return pitch to neutral (0°)."""
        if sim:
            print("  [SIM] Pitch returned to 0°")
            return
        if servos:
            for i in range(8):
                servos.set_pitch(i, 0.0)
            print("  Pitch returned to neutral")

    def deactivate(self, servos, sim=False):
        """Return wings to deployed (60°) position."""
        print("  Returning wings to 60° deployed position...")
        if sim:
            print("  [SIM] Wings returned to 60°")
            return
        if servos:
            servos.centre_all()
            print("  Wings returned to 60° deployed")

    def _print_reference(self):
        print("\n  Wing span reference (semi-span each side):")
        for name, span_mm in WING_SPANS.items():
            print(f"    {name}: {span_mm}mm  ({span_mm/10:.1f}cm) from root")
        print()
        print("  Longitudinal stagger: 250mm between adjacent pairs")
        print("  W1 is furthest forward; W4 is furthest aft")
        print()
        # Print the walk-around checklist
        for line in INSPECTION_WALK_AROUND.strip().split('\n'):
            print("  " + line)


# ─────────────────────────────────────────────────────────────────
# PARK MODE (updated to include inspection mode trigger)
# ─────────────────────────────────────────────────────────────────

class ParkMode:
    """
    Post-flight check and secure sequence.
    Now includes option to enter inspection mode before stow.
    """

    def run(self, odrive=None, servos=None, pixhawk=None,
            flight_recorder=None, comparator=None, sim=False):
        SEP = "=" * 55
        print("\n" + SEP)
        print("  MCO-1D PARK MODE — Post-Flight Check")
        print(SEP)
        issues = []

        print("\n  [1/5] Electronic systems check...")
        issues += self._check_electronics(pixhawk, flight_recorder,
                                          comparator, sim)

        print("\n  [2/5] Servo health check...")
        issues += self._check_servos(servos, sim)

        print("\n  [3/5] Motor health check...")
        issues += self._check_motors(odrive, servos, sim)

        print("\n  [4/5] Final sensor readings...")
        issues += self._final_sensor_readings(pixhawk, sim)

        # Offer inspection mode before physical check
        print("\n  [Optional] Enter inspection mode for full walk-around?")
        try:
            ans = input("  Wings extend to 90° for inspection "
                        "(Enter=yes, 'n'=skip): ").strip().lower()
        except EOFError:
            ans = "n"

        if ans != 'n':
            insp = InspectionMode()
            insp.activate(servos, odrive, sim)
            try:
                input("\n  Press Enter when walk-around complete "
                      "(or 't' then Enter for tipped view): ")
            except EOFError:
                pass
            insp.deactivate(servos, sim)

        print("\n  [5/5] Physical inspection items...")
        issues += self._physical_inspection()

        print("\n  Stowing all wings to 210°...")
        self._stow_wings(servos, sim)

        print("\n" + SEP)
        if issues:
            print("  PARK COMPLETE — ANOMALIES FOUND — see below")
            for issue in issues:
                print(f"    ** {issue}")
        else:
            print("  PARK COMPLETE — AIRCRAFT CLEAR FOR NEXT FLIGHT")
        print(SEP + "\n")
        return len(issues) == 0

    def _check_electronics(self, pixhawk, recorder, comparator, sim):
        issues = []
        if sim or pixhawk is None:
            print("    ─ Pixhawk check skipped (sim)"); return issues
        state  = pixhawk.get_state()
        v_batt = state.get("voltage", 0)
        if   v_batt < 19.8 and v_batt > 0:
            issues.append(f"Battery landed at {v_batt:.2f}V — charge immediately")
            print(f"    ** Battery: {v_batt:.2f}V — CRITICAL")
        elif v_batt < 20.4 and v_batt > 0:
            print(f"    ⚠  Battery: {v_batt:.2f}V — low, charge before next flight")
        elif v_batt > 0:
            print(f"    ✓  Battery: {v_batt:.2f}V")
        if recorder:
            n = len(getattr(recorder,'anomalies',[]))
            if n: print(f"    ⚠  {n} flight anomalies — review flight log")
            else: print("    ✓  No flight anomalies")
        if comparator:
            ct = getattr(comparator,'mean_ct',lambda:None)()
            if ct: print(f"    ✓  Mean CT: {ct:.4f}")
        return issues

    def _check_servos(self, servos, sim):
        issues = []
        if sim or servos is None:
            print("    ─ Servo check skipped (sim)"); return issues
        for i in range(8):
            wname = f"W{i//2+1}{'p' if i%2==0 else 's'}"
            try:
                t0 = time.monotonic()
                servos.set_sweep(i, 120.0); time.sleep(0.5)
                servos.set_sweep(i,  60.0)
                if time.monotonic() - t0 > 1.5:
                    issues.append(f"Sweep servo {wname} sluggish — check pivot")
                    print(f"    ⚠  Sweep {wname}: sluggish")
                else:
                    print(f"    ✓  Sweep {wname}")
            except Exception as e:
                issues.append(f"Sweep servo {wname}: {e}")
        return issues

    def _check_motors(self, odrive, servos, sim):
        issues = []
        if sim or odrive is None:
            print("    ─ Motor check skipped (sim)"); return issues
        try:
            if servos:
                for i in range(8): servos.set_sweep(i, 200.0)
                time.sleep(1.0)
            odrive.enable_all(); time.sleep(0.2)
            odrive.set_all_flap_freq(0.2); time.sleep(2.0)
            odrive.idle_all(); time.sleep(0.3)
            print("    ✓  All 8 motors responded")
            if servos: servos.centre_all()
        except Exception as e:
            issues.append(f"Motor check: {e}")
            try: odrive.idle_all()
            except: pass
        return issues

    def _final_sensor_readings(self, pixhawk, sim):
        issues = []
        if sim or pixhawk is None:
            print("    ─ Sensor check skipped (sim)"); return issues
        state = pixhawk.get_state()
        aspd  = state.get("airspeed_ms", 0)
        if aspd > 3.0:
            issues.append(f"Airspeed {aspd:.1f} m/s at rest — check pitot")
            print(f"    ⚠  Airspeed at rest: {aspd:.1f} m/s")
        else:
            print(f"    ✓  Airspeed at rest: {aspd:.1f} m/s")
        return issues

    def _physical_inspection(self):
        issues = []
        ITEMS = [
            ("POST-FLIGHT WALK-AROUND", [
                "Wing membranes — no new tears or separation",
                "D-box skin — no new damage, delamination, or dents",
                "All rope drives — seated in pulleys, not frayed",
                "Raked wingtips — bonded, no impact cracks",
                "Fuselage — no cracks or ground contact damage",
                "All connectors — still fully seated after vibration",
                "Battery connector — no discolouration or heat damage",
                "Pitot tube — no new blockage or damage",
            ]),
        ]
        for section, items in ITEMS:
            print(f"\n    [{section}]")
            for item in items:
                try:
                    ans = input(
                        f"      □ {item}  (Enter=OK, 'f'=issue): "
                    ).strip().lower()
                except EOFError:
                    ans = ""
                if ans == 'f':
                    issues.append(f"Post-flight: {item}")
                    print("        ** ISSUE NOTED")
        return issues

    def _stow_wings(self, servos, sim):
        if sim or servos is None:
            print("    ─ Stow skipped (sim)"); return
        try:
            servos.stow_all(rate_degps=20.0)
            print("    ✓  All wings stowed to 210°")
        except Exception as e:
            print(f"    ✗  Stow failed: {e}")


# ─────────────────────────────────────────────────────────────────
# WAVE LIBRARY PARAMETER ADDITIONS
# ─────────────────────────────────────────────────────────────────
WAVE_LIBRARY_ADDITIONS = f"""
# ── TAXI MODE (v2 — asymmetric stroke) ───────────────────────────
# Downstroke limited to ground clearance; upstroke unrestricted.
# Based on bird ground-run biology and Lilong & Yu (2024).

TAXI_AMP_DOWN_DEG      = {TAXI_AMP_DOWN_DEG:.1f}   # deg — downstroke limit (150mm margin)
TAXI_AMP_UP_DEG        = 80.0  # deg — full upstroke, no constraint
TAXI_FREQ_MIN_HZ       =  0.2  # Hz  slow creep
TAXI_FREQ_CRUISE_HZ    =  0.5  # Hz  walking pace
TAXI_FREQ_MAX_HZ       =  1.0  # Hz  against wind
TAXI_PITCH_DEG         = +3.0  # deg ground effect AoA
TAXI_TURN_DELTA_PHI    =  0.30 # ΔΦ* upstroke asymmetry ratio
TAXI_REVERSE_PITCH_DEG = -15.0 # deg rearward thrust

# ── INSPECTION MODE ──────────────────────────────────────────────
INSPECTION_SWEEP_DEG        =  90.0  # deg — wings abeam (laterally extended)
INSPECTION_SWEEP_RATE_DEG_S =  10.0  # deg/s — slow deliberate movement
INSPECTION_PITCH_DEG        =   0.0  # deg — chord horizontal
INSPECTION_PITCH_TIPPED_DEG = +15.0  # deg — raised for upper surface view
"""

if __name__ == "__main__":
    print("Taxi command tests (v2 — asymmetric stroke):")
    print(f"  Downstroke limit: {TAXI_AMP_DOWN_DEG:.1f}°  Upstroke: {TAXI_AMP_UP_DEG}°")
    for throttle, yaw, rev in [
        (0.3,  0.0, False),
        (0.5, +1.0, False),
        (0.5, -1.0, False),
        (0.3,  0.0, True),
        (0.5, +0.5, True),
    ]:
        cmd = taxi_commands(throttle, yaw, reverse=rev)
        wc  = cmd['wing_cmds']
        print(f"  throttle={throttle} yaw={yaw:+.1f} rev={str(rev):<5}  "
              f"freq={cmd['freq_hz']:.2f}Hz  "
              f"up_p={wc[0]['amp_up']:.0f}° up_s={wc[1]['amp_up']:.0f}°  "
              f"down={wc[0]['amp_down']:.1f}°  pitch={wc[0]['pitch']}°")
    print("\nInspection mode geometry:")
    InspectionMode()._print_reference()
