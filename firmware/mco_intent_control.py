# NOTE (Rev 18): Intent now flows via sensors["intent"] which is
# set from result["intent"] → self._last_intent → next tick sensors.
# The CG management stability margin and landing mode selection
# read intent from sensors["intent"] each tick.
# This file provides the IntentController class for generating
# intent labels from pilot inputs and flight state.
# Wire IntentController.update() into mco_main if richer intent
# classification is needed beyond the wave library default.
#
"""
MCO-1D Intent-Based Control System  (mco_intent_control.py)
============================================================
Rev K: Auto-coordination, decoupled yaw/roll, flat turn
Rev L: Fin yaw (W1+W4 vertical pitch), dart manoeuvre
"""

import time
import math

# ── INTENT TAXONOMY ──────────────────────────────────────────────

class Intent:
    CRUISE         = 'cruise'
    CLIMB          = 'climb'
    DESCEND        = 'descend'
    TURN_LEFT      = 'turn_left'
    TURN_RIGHT     = 'turn_right'
    GLIDE          = 'glide'
    SOAR           = 'soar'
    SPRINT         = 'sprint'
    APPROACH_NORMAL= 'approach_normal'
    APPROACH_STEEP = 'approach_steep'
    APPROACH_ABORT = 'approach_abort'
    LAND           = 'land'
    TIGHT_TURN     = 'tight_turn'
    CHANDELLE      = 'chandelle'
    SPIRAL_DOWN    = 'spiral_down'
    STOOP          = 'stoop'
    CF1_BURST      = 'cf1_burst'
    LOITER         = 'loiter'
    ORBIT_LEFT     = 'orbit_left'
    ORBIT_RIGHT    = 'orbit_right'
    APPROACH_SITE  = 'approach_site'
    RECOVER        = 'recover'
    RTH            = 'rth'
    EMERGENCY_LAND = 'emergency_land'
    WING_ALL       = 'wing_all'
    WING_PARTIAL   = 'wing_partial'
    WING_SINGLE    = 'wing_single'
    WING_SLOT      = 'wing_slot'
    WING_GAP       = 'wing_gap'
    WING_REAR      = 'wing_rear'
    TAXI_FWD       = 'taxi_fwd'
    TAXI_REV       = 'taxi_rev'
    INSPECTION     = 'inspection'
    # Rev K
    FLAT_TURN_LEFT  = 'flat_turn_left'
    FLAT_TURN_RIGHT = 'flat_turn_right'
    SLIP            = 'slip'
    SKID            = 'skid'
    # Rev L
    DART_RIGHT      = 'dart_right'
    DART_LEFT       = 'dart_left'

    # Rev N
    ZOOM_CLIMB      = 'zoom_climb'
    POWER_DIVE      = 'power_dive'
    STOOP_AUTO      = 'stoop_auto'

    # Rev O3
    CRAB_LEFT       = 'crab_left'
    CRAB_RIGHT      = 'crab_right'


# ── INTENT INTERPRETER ───────────────────────────────────────────

class IntentInterpreter:
    STICK_THRESHOLD     = 0.12
    THROTTLE_CLIMB_MIN  = 0.65
    THROTTLE_DESCEND_MAX= 0.25
    ROLL_TURN_MIN       = 0.15
    GLIDE_LEVER_MIN     = 0.10
    LEVER_DETENT_LOW    = 47.5
    LEVER_DETENT_HIGH   = 52.5

    def __init__(self):
        self._last_intent    = Intent.CRUISE
        self._coord_mode     = 'auto'
        self._macro_requested= None
        import mco_wave_library as _wl_init
        self._gesture_det    = _wl_init.GestureDetector()

    def interpret(self, sticks, switches, context):
        ch1 = sticks.get('roll',     0.0)
        ch2 = sticks.get('pitch',    0.0)
        ch3 = sticks.get('throttle', 0.5)
        ch4 = sticks.get('yaw',      0.0)
        ch6 = sticks.get('gl_lever', 0.0)

        mode_sw  = switches.get('mode_sw',   0)
        macro_sw = switches.get('macro_sw',  False)
        macro_lng= switches.get('macro_long',False)
        rth_sw   = switches.get('rth_sw',    False)
        wing_sw  = switches.get('wing_sw',   0)

        V_ms      = context.get('V_ms',      8.0)
        h_agl_m   = context.get('h_agl_m',   50.0)
        bank_deg  = context.get('bank_deg',  0.0)
        Ps        = context.get('Ps',        0.0)
        on_ground = context.get('on_ground', False)

        # Rev K — coordination mode
        # Rev N: gesture detection for zoom/dive/stoop
        import mco_wave_library as _wl
        _gesture, _gconf, _gparams = _wl.classify_pitch_intent(
            sticks.get('pitch', 0.0), ch3*100.0, V_ms,
            h_agl_m, context.get('pitch_deg', 0.0),
            self._gesture_det)
        if _gesture == 'zoom_climb' and not on_ground:
            return Intent.ZOOM_CLIMB, _gconf, _gparams
        if _gesture == 'stoop_auto' and not on_ground:
            return Intent.STOOP_AUTO, _gconf, _gparams
        if _gesture == 'power_dive' and not on_ground:
            return Intent.POWER_DIVE, _gconf, _gparams

        # Rev O3: crab via flat_turn mode + roll stick
        if (self._coord_mode == 'flat_turn'
                and abs(ch1) > 0.2 and not on_ground):
            crab_dir = 'left' if ch1 < 0 else 'right'
            crab_intent = (Intent.CRAB_LEFT
                if ch1 < 0 else Intent.CRAB_RIGHT)
            return crab_intent, abs(ch1), {
                'direction': crab_dir,
                'intensity': abs(ch1)}

        coord_mode = _wl.select_coordination_mode(
            mode_sw, macro_lng, on_ground)
        self._coord_mode = coord_mode

        # ── Safety ───────────────────────────────────────────────
        if rth_sw:
            return Intent.RTH, 1.0, {}
        if context.get('stall_imminent', False):
            return Intent.RECOVER, 1.0, {'reason': 'stall_imminent'}
        if context.get('battery_critical', False):
            return Intent.EMERGENCY_LAND, 1.0, {'reason': 'battery_critical'}

        # ── Ground ───────────────────────────────────────────────
        if on_ground or h_agl_m < 0.15:
            if wing_sw == 2:
                return Intent.INSPECTION, 1.0, {}
            if macro_lng:
                return Intent.TAXI_REV, 0.9, {'throttle': ch3, 'yaw': ch4}
            return Intent.TAXI_FWD, 0.9, {'throttle': ch3, 'yaw': ch4}

        # ── Wing config ──────────────────────────────────────────
        if wing_sw == 0:
            gs_id = 'GS-0'
        elif wing_sw == 1:
            gs_id = 'GS-2' if (V_ms > 10.0 and ch3 < 0.3) else 'GS-1'
        else:
            if macro_lng:   gs_id = 'GS-5'
            elif macro_sw:  gs_id = 'GS-3'
            else:           gs_id = 'GS-4'

        # ── Macro triggers ───────────────────────────────────────
        if macro_sw or macro_lng:
            if mode_sw == 0:
                if macro_lng:
                    return Intent.STOOP, 0.95, {'gs_id': gs_id}
                return Intent.CF1_BURST, 0.9, {}
            elif mode_sw == 1:
                direction = 'right' if ch1 > 0 else 'left'
                if macro_lng:
                    # Full roll + long press = dart (Rev L)
                    if abs(ch1) > 0.7:
                        di = 'right' if ch1 > 0 else 'left'
                        return (Intent.DART_RIGHT if ch1 > 0
                                else Intent.DART_LEFT), 0.95, {
                                    'direction': di}
                    return Intent.CHANDELLE, 0.9, {'direction': direction}
                return Intent.TIGHT_TURN, 0.95, {
                    'bank_deg': 65.0, 'direction': direction}
            elif mode_sw == 2:
                if macro_lng:
                    return Intent.APPROACH_SITE, 0.9, {}
                return Intent.SPIRAL_DOWN, 0.9, {}

        # ── SAR mode ────────────────────────────────────────────
        if mode_sw == 2:
            if abs(ch1) < self.STICK_THRESHOLD:
                return Intent.LOITER, 0.8, {'gs_id': gs_id}
            if ch1 > self.ROLL_TURN_MIN:
                return Intent.ORBIT_RIGHT, 0.8, {
                    'bank_deg': 30 + ch1*20, 'gs_id': gs_id}
            return Intent.ORBIT_LEFT, 0.8, {
                'bank_deg': 30 + abs(ch1)*20, 'gs_id': gs_id}

        # ── Manoeuvre mode ───────────────────────────────────────
        if mode_sw == 1:
            if abs(ch1) > self.STICK_THRESHOLD:
                direction = 'right' if ch1 > 0 else 'left'
                bank = min(30 + abs(ch1)*35, 65.0)
                if ch2 > 0.3 and abs(ch1) > 0.5:
                    return Intent.CHANDELLE, 0.85, {
                        'direction': direction, 'gs_id': gs_id}
                mt = 'MT-4' if bank > 60 else 'MT-3' if bank > 45 else 'MT-2'
                return Intent.TIGHT_TURN, abs(ch1), {
                    'bank_deg': bank, 'direction': direction,
                    'mt_id': mt, 'gs_id': gs_id,
                    'coord_mode': coord_mode}

        # ── Cruise mode ──────────────────────────────────────────
        lever_pct = ch3 * 100.0

        if lever_pct > 52.5:
            return Intent.GLIDE, 1.0, {'lever_pct': lever_pct, 'gs_id': gs_id}

        if lever_pct < 15.0:
            return Intent.SPRINT, 1.0, {'lever_pct': lever_pct, 'gs_id': gs_id}

        if abs(ch1) > self.ROLL_TURN_MIN:
            direction = 'right' if ch1 > 0 else 'left'
            roll_abs  = abs(ch1)
            if   roll_abs > 0.70: mt = 'MT-3'; bank = 55 + roll_abs*10
            elif roll_abs > 0.30: mt = 'MT-2'; bank = 35 + roll_abs*20
            else:                  mt = 'MT-1'; bank = 20 + roll_abs*30
            bank = min(bank, 65.0)
            return (Intent.TIGHT_TURN if bank > 50 else
                    Intent.TURN_RIGHT if ch1 > 0 else Intent.TURN_LEFT), \
                   roll_abs, {'bank_deg': bank, 'direction': direction,
                              'mt_id': mt, 'gs_id': gs_id,
                              'coord_mode': coord_mode}

        if lever_pct < 47.5 and ch2 > 0.2:
            return Intent.CLIMB, ch2, {'lever_pct': lever_pct, 'gs_id': gs_id}

        if self.LEVER_DETENT_LOW <= lever_pct <= self.LEVER_DETENT_HIGH:
            return Intent.CRUISE, 0.7, {'lever_pct': lever_pct, 'gs_id': gs_id}

        return Intent.CRUISE, 0.6, {'throttle': ch3, 'gs_id': gs_id}


# ── BEHAVIOUR NODE BASE ──────────────────────────────────────────

RUNNING = 'RUNNING'
SUCCESS = 'SUCCESS'
FAILURE = 'FAILURE'


class BehaviourNode:
    name          = 'base'
    priority      = 99
    interruptible = True

    def __init__(self):
        self._state   = None
        self._t_start = None
        self._log     = []

    def on_enter(self, context):
        self._t_start = context.get('t', 0.0)
        self._state   = RUNNING

    def tick(self, context, ctrl):
        return RUNNING

    def on_exit(self, context, completed):
        pass

    def can_be_preempted_by(self, other_priority, is_safety=False):
        if is_safety: return True
        if not self.interruptible: return False
        return other_priority <= self.priority

    def elapsed(self, context):
        if self._t_start is None: return 0.0
        return context.get('t', 0.0) - self._t_start

    def log(self, msg):
        self._log.append(msg)


# ── CONTINUOUS BEHAVIOURS ────────────────────────────────────────

class CruiseBehaviour(BehaviourNode):
    name = 'cruise'; priority = 10; interruptible = True
    def tick(self, context, ctrl):
        import mco_wave_library as wl
        Ps    = context.get('Ps', 0.2)
        V     = context.get('V_ms', 8.5)
        pitch = context.get('pitch_deg', 0.0)
        lever = context.get('lever_pct', 40.0)
        p = wl.lever_airspeed_params(lever, V, pitch, Ps, ctrl.freq_hz)
        ctrl._tgt_freq      = p['freq_hz']
        ctrl._tgt_sweeps    = [p['sweep_deg']] * 8
        ctrl._tgt_dihedrals = [p['dihedral_deg']] * 8
        ctrl._tgt_pitches   = [p['pitch_deg']] * 8
        return RUNNING


class ClimbBehaviour(BehaviourNode):
    name = 'climb'; priority = 9; interruptible = True
    def tick(self, context, ctrl):
        import mco_wave_library as wl
        Ps    = context.get('Ps', 0.5)
        V     = context.get('V_ms', 8.5)
        pitch = context.get('pitch_deg', 10.0)
        lever = context.get('lever_pct', 30.0)
        # Rev O3: near stall → X-29 forward sweep for retained authority
        x29 = wl.x29_high_aoa_params(V, Ps, ctrl.freq_hz)
        if x29 is not None:
            ctrl._tgt_freq      = x29['freq_hz']
            ctrl._tgt_sweeps    = x29['sweeps']
            ctrl._tgt_dihedrals = x29['dihedrals']
            ctrl._tgt_pitches   = x29['pitches']
        else:
            p = wl.lever_airspeed_params(lever, V, pitch, Ps, ctrl.freq_hz)
            ctrl._tgt_freq      = p['freq_hz']
            ctrl._tgt_sweeps    = [p['sweep_deg']] * 8
            ctrl._tgt_dihedrals = [0.0] * 8
            ctrl._tgt_pitches   = [p['pitch_deg']] * 8
        return RUNNING


class GlideBehaviour(BehaviourNode):
    """Unified lever — glide/approach/airbrake zones."""
    name = 'glide'; priority = 9; interruptible = True
    def tick(self, context, ctrl):
        import mco_wave_library as wl
        lever = context.get('lever_pct', 50.0)
        V     = context.get('V_ms', 8.0)
        Ps    = context.get('Ps', 0.0)
        pitch = context.get('pitch_deg', 0.0)
        safe, _ = wl.lever_stall_check(lever, V)
        if not safe:
            lever = min(lever, wl.LEVER_APPROACH_END)
        if wl.lever_in_detent(lever):
            return RUNNING
        p = wl.lever_airspeed_params(lever, V, pitch, Ps, ctrl.freq_hz)
        ctrl._tgt_freq      = p['freq_hz']
        ctrl._tgt_sweeps    = [p['sweep_deg']]    * 8
        ctrl._tgt_dihedrals = [p['dihedral_deg']] * 8
        ctrl._tgt_pitches   = [p['pitch_deg']]    * 8
        return RUNNING


class TurnBehaviour(BehaviourNode):
    """Four-phase Boyd turn with Rev K auto-coordination."""
    name = 'turn'; priority = 8; interruptible = True
    PHASE_ENTRY   = 'entry'
    PHASE_BANKED  = 'banked'
    PHASE_ROLLOUT = 'rollout'
    PHASE_RESUME  = 'resume'

    def on_enter(self, context):
        super().on_enter(context)
        self._phase      = self.PHASE_ENTRY
        self._phase_t    = context.get('t', 0.0)
        self._mt_id      = context.get('mt_id', 'MT-1')
        self._dir        = context.get('direction', 'right')

    def tick(self, context, ctrl):
        import mco_wave_library as wl
        t         = context.get('t', 0.0)
        V_ms      = context.get('V_ms', 9.0)
        bank_deg  = context.get('bank_deg', 0.0)
        Ps        = context.get('Ps', 0.5)
        coord     = context.get('coord_mode', 'auto')
        dt_phase  = t - self._phase_t
        mm        = wl.MANOEUVRE_MODES.get(self._mt_id, {})
        bank_tgt  = mm.get('bank_deg', 30.0)

        if self._phase == self.PHASE_ENTRY:
            cmd = wl.turn_entry_commands(bank_tgt, self._dir, V_ms, ctrl.freq_hz)
            ctrl._tgt_freq      = cmd['freq_hz']
            ctrl._tgt_sweeps    = cmd['sweeps']
            ctrl._tgt_dihedrals = cmd['dihedrals']
            ctrl._tgt_pitches   = [cmd['pitch_deg']] * 8
            if dt_phase >= 0.5 or abs(bank_deg) > 5.0:
                self._phase   = self.PHASE_BANKED
                self._phase_t = t

        elif self._phase == self.PHASE_BANKED:
            cmd = wl.turn_in_progress_commands(
                bank_deg=abs(bank_deg), turn_direction=self._dir,
                V_ms=V_ms, Ps=Ps, current_freq_hz=ctrl.freq_hz)
            ctrl._tgt_freq   = cmd['freq_hz']
            ctrl._tgt_sweeps = cmd['sweeps']
            base_d = cmd['dihedrals']
            if coord == 'auto':
                base_d = wl.auto_coordinated_params(
                    base_d, bank_deg, V_ms,
                    context.get('yaw_rate_radps', 0.0), self._dir)
            elif coord in ('decoupled', 'semi'):
                yd = wl.yaw_dihedral_asymmetry(context.get('yaw', 0.0))
                base_d = [base_d[i] + yd[i] for i in range(8)]
            ctrl._tgt_dihedrals = base_d
            ctrl._tgt_pitches   = [cmd['pitch_deg']] * 8

        elif self._phase == self.PHASE_ROLLOUT:
            cmd = wl.turn_rollout_commands(abs(bank_deg), self._dir)
            ctrl._tgt_freq      = cmd['freq_hz']
            ctrl._tgt_sweeps    = cmd['sweeps']
            ctrl._tgt_dihedrals = cmd['dihedrals']
            ctrl._tgt_pitches   = [cmd['pitch_deg']] * 8
            if abs(bank_deg) < 5.0 and dt_phase > 0.5:
                self._phase   = self.PHASE_RESUME
                self._phase_t = t

        elif self._phase == self.PHASE_RESUME:
            ctrl._tgt_freq      = wl.TURN_CRUISE_FREQ_HZ
            ctrl._tgt_sweeps    = [60.0] * 8
            ctrl._tgt_dihedrals = [0.0]  * 8
            ctrl._tgt_pitches   = [0.0]  * 8
            if dt_phase >= 2.0:
                self._state = SUCCESS
                return SUCCESS

        return RUNNING

    def begin_rollout(self, context):
        if self._phase == self.PHASE_BANKED:
            self._phase   = self.PHASE_ROLLOUT
            self._phase_t = context.get('t', 0.0)


class SprintBehaviour(BehaviourNode):
    name = 'sprint'; priority = 9; interruptible = True
    def tick(self, context, ctrl):
        import mco_wave_library as wl
        Ps = context.get('Ps', 1.0)
        V  = context.get('V_ms', 12.0)
        freq, sweep, dihs, pitch = wl.sprint_params(Ps, V, 0.02, ctrl.freq_hz)
        ctrl._tgt_freq      = freq
        ctrl._tgt_sweeps    = [sweep] * 8
        ctrl._tgt_dihedrals = dihs
        ctrl._tgt_pitches   = [pitch] * 8
        return RUNNING


# ── MACRO BEHAVIOURS ─────────────────────────────────────────────

class StoopMacro(BehaviourNode):
    name = 'stoop'; priority = 2; interruptible = False
    PHASES = ['entry', 'dive', 'pullout', 'recovery']

    def on_enter(self, context):
        super().on_enter(context)
        self._phase   = 'entry'
        self._phase_t = context.get('t', 0.0)

    def tick(self, context, ctrl):
        import mco_wave_library as wl
        t     = context.get('t', 0.0)
        V_ms  = context.get('V_ms', 0.0)
        h_agl = context.get('h_agl_m', 100.0)
        Ps    = context.get('Ps', 0.0)
        dt    = t - self._phase_t

        if self._phase == 'entry':
            freq, sw, dihs, pt = wl.stoop_dive_params(V_ms, h_agl, ctrl.freq_hz)
            ctrl._tgt_sweeps    = [sw]  * 8
            ctrl._tgt_dihedrals = dihs
            ctrl._tgt_freq      = freq
            ctrl._tgt_pitches   = [pt] * 8
            if all(s > 200 for s in ctrl._tgt_sweeps):
                self._phase = 'dive'; self._phase_t = t

        elif self._phase == 'dive':
            freq, sw, dihs, pt = wl.stoop_dive_params(V_ms, h_agl, ctrl.freq_hz)
            ctrl._tgt_freq      = freq
            ctrl._tgt_sweeps    = [sw]  * 8
            ctrl._tgt_dihedrals = dihs
            ctrl._tgt_pitches   = [pt] * 8
            pullout, reason = wl.ps_pullout_recommended(
                Ps, h_agl, V_ms, V_ms - wl.VS_KT/1.944)
            if pullout:
                self._phase = 'pullout'; self._phase_t = t

        elif self._phase == 'pullout':
            freq, sw, dihs, pt = wl.stoop_pullout_params(Ps, V_ms, 0.02, ctrl.freq_hz)
            ctrl._tgt_freq      = freq
            ctrl._tgt_sweeps    = [sw]  * 8
            ctrl._tgt_dihedrals = dihs
            ctrl._tgt_pitches   = [pt] * 8
            if V_ms < 12.0 and dt > 2.0:
                self._phase = 'recovery'; self._phase_t = t

        elif self._phase == 'recovery':
            ctrl._tgt_sweeps    = [60.0] * 8
            ctrl._tgt_freq      = 0.8
            ctrl._tgt_dihedrals = [0.0]  * 8
            ctrl._tgt_pitches   = [0.0]  * 8
            if dt > 3.0:
                self._state = SUCCESS; return SUCCESS

        return RUNNING

    def can_be_preempted_by(self, other_priority, is_safety=False):
        return is_safety or other_priority == 0


class CF1Macro(BehaviourNode):
    name = 'cf1_burst'; priority = 4; interruptible = True
    def on_enter(self, context):
        super().on_enter(context)
        self._strokes = 0
    def tick(self, context, ctrl):
        import mco_wave_library as wl
        V_ms = context.get('V_ms', 0.0)
        ok, reason = wl.cf1_available(V_ms, wl.CF1_DEFAULT_FREQ_HZ, ctrl._tgt_sweeps)
        if not ok:
            self._state = FAILURE; return FAILURE
        ctrl._tgt_freq      = wl.CF1_DEFAULT_FREQ_HZ
        ctrl._tgt_sweeps    = [60.0]  * 8
        ctrl._tgt_dihedrals = [0.0]   * 8
        ctrl._tgt_pitches   = [wl.CF1_PITCH_OFFSET_DEG] * 8
        self._strokes = int(self.elapsed(context) * wl.CF1_DEFAULT_FREQ_HZ)
        if self._strokes >= wl.CF1_BURST_MAX:
            self._state = SUCCESS; return SUCCESS
        return RUNNING


class RecoverBehaviour(BehaviourNode):
    name = 'recover'; priority = 0; interruptible = False
    def tick(self, context, ctrl):
        import mco_wave_library as wl
        V_ms  = context.get('V_ms', 5.0)
        pitch = context.get('pitch_deg', 0.0)
        # Rev O3: X-29 forward sweep for stall recovery
        # Root stalls first → tip pitch servos remain effective
        p = wl.x29_stall_recovery_params(
            V_ms, pitch, ctrl.freq_hz)
        ctrl._tgt_freq      = p['freq_hz']
        ctrl._tgt_sweeps    = p['sweeps']
        ctrl._tgt_dihedrals = p['dihedrals']
        ctrl._tgt_pitches   = p['pitches']
        if p['recovering'] and self.elapsed(context) > 1.5:
            self._state = SUCCESS; return SUCCESS
        if self.elapsed(context) > 5.0:
            ctrl.emergency_recover()
            self._state = SUCCESS; return SUCCESS
        return RUNNING


class ChandellesMacro(BehaviourNode):
    name = 'chandelle'; priority = 5; interruptible = True
    def on_enter(self, context):
        super().on_enter(context)
        self._entry_heading = context.get('heading_deg', 0.0)
        self._direction     = context.get('direction', 'right')
    def tick(self, context, ctrl):
        import mco_wave_library as wl
        heading_change = abs(
            context.get('heading_deg', 0.0) - self._entry_heading) % 360
        if heading_change > 180: heading_change = 360 - heading_change
        freq  = wl.chandelle_freq(heading_change)
        bank  = wl.chandelle_bank(heading_change)
        ctrl._tgt_freq   = freq
        ctrl._tgt_sweeps = [wl.CHANDELLE_SWEEP_DEG] * 8
        ctrl._tgt_pitches= [wl.CHANDELLE_PITCH_OFFSET_DEG] * 8
        sw_delta = bank * 0.5
        for i in range(8):
            is_port = (i % 2 == 0)
            inner   = ((self._direction=='right') and not is_port) or \
                      ((self._direction=='left')  and     is_port)
            ctrl._tgt_sweeps[i] = (wl.CHANDELLE_SWEEP_DEG + sw_delta
                                    if inner else wl.CHANDELLE_SWEEP_DEG)
        if heading_change >= 178:
            self._state = SUCCESS; return SUCCESS
        return RUNNING


# ── REV L — DART MACRO ───────────────────────────────────────────

class DartMacro(BehaviourNode):
    """
    Rev L: Dragonfly dart — W1+W4 pitch 75° (vertical fins).
    W2+W3 continue flapping to sustain lift throughout.
    2.4g lateral, 90° heading change in 0.15s at V=14m/s.
    Analogous to Fokker Dr.I snap-turn, Iz = 0.17 kg·m².
    """
    name = 'dart'; priority = 3; interruptible = False

    def on_enter(self, context):
        super().on_enter(context)
        import mco_wave_library as wl
        self._dir = context.get('direction', 'right')
        V = context.get('V_ms', 9.0)
        safe, reason = wl.dart_safe_to_initiate(
            V, context.get('h_agl_m', 50.0),
            context.get('bank_deg', 0.0))
        if not safe and 'sprint' not in reason:
            self._state = FAILURE
            self.log(f'Dart blocked: {reason}')
            return
        self._phase = (wl.DART_PHASE_ENGAGE
            if V >= wl.DART_PREBUILD_V_MS * 0.95
            else wl.DART_PHASE_PREBUILD)
        self._phase_t = context.get('t', 0.0)
        self.log(f'Dart {self._dir} V={V:.1f}m/s')

    def tick(self, context, ctrl):
        import mco_wave_library as wl
        t  = context.get('t', 0.0)
        V  = context.get('V_ms', 9.0)
        dt = t - self._phase_t
        if self._phase == wl.DART_PHASE_DISENGAGE:
            p = wl.dart_disengage_fling(
                dt, self._dir, V, ctrl.freq_hz)
        else:
            p  = wl.dart_sequence_params(
                self._phase, dt, self._dir, V,
                context.get('bank_deg', 0.0), ctrl.freq_hz)
        ctrl._tgt_freq      = p['freq_hz']
        ctrl._tgt_sweeps    = p['sweeps']
        ctrl._tgt_dihedrals = p['dihedrals']
        ctrl._tgt_pitches   = p['pitches']
        # Sync fin mode so enforce_physical_limits uses right AoA range
        ctrl._fin_mode_pairs = wl._FIN_MODE_PAIRS.copy() \
            if hasattr(wl,'_FIN_MODE_PAIRS') else None
        if p['phase_done']:
            _NX = {wl.DART_PHASE_PREBUILD:  wl.DART_PHASE_ENGAGE,
                   wl.DART_PHASE_ENGAGE:    wl.DART_PHASE_PIVOT,
                   wl.DART_PHASE_PIVOT:     wl.DART_PHASE_DISENGAGE,
                   wl.DART_PHASE_DISENGAGE: wl.DART_PHASE_RESUME,
                   wl.DART_PHASE_RESUME:    None}
            nxt = _NX.get(self._phase)
            if nxt is None:
                self._state = SUCCESS
                self.log(f'Dart {self._dir} complete')
                return SUCCESS
            self.log(f'Dart phase {self._phase}→{nxt}')
            self._phase = nxt
            self._phase_t = t
        return RUNNING

    def can_be_preempted_by(self, other_priority, is_safety=False):
        return is_safety


# ── REV K — COORDINATION BEHAVIOURS ─────────────────────────────

class ZoomClimbMacro(BehaviourNode):
    """Rev N: Zoom climb — V→h Boyd energy conversion."""
    name = 'zoom_climb'; priority = 4; interruptible = True

    def on_enter(self, context):
        super().on_enter(context)
        import mco_wave_library as wl
        V = context.get('V_ms', 9.0)
        safe, reason = wl.zoom_safe_to_initiate(
            V, context.get('h_agl_m', 50.0))
        if not safe:
            self._state = FAILURE; self.log(f'Zoom blocked:{reason}'); return
        self._phase   = wl.ZOOM_PHASE_ENTRY
        self._phase_t = context.get('t', 0.0)
        self.log(f'Zoom V={V:.1f}m/s')

    def tick(self, context, ctrl):
        import mco_wave_library as wl
        t=context.get('t',0.0); V=context.get('V_ms',9.0)
        Ps=context.get('Ps',0.0); dt=t-self._phase_t
        np_=wl.zoom_phase_from_state(V,dt,Ps)
        if np_!=self._phase:
            self.log(f'{self._phase}→{np_}'); self._phase=np_; self._phase_t=t
        if self._phase==wl.ZOOM_PHASE_ENTRY:
            p=wl.zoom_entry_params(V)
        elif self._phase==wl.ZOOM_PHASE_CLIMB:
            p=wl.zoom_climb_params(V,Ps,ctrl.freq_hz,dt)
        elif self._phase==wl.ZOOM_PHASE_APEX:
            p=wl.zoom_apex_params(V)
        else:
            p=wl.zoom_resume_params(V,Ps,ctrl.freq_hz)
            if dt>2.0: self._state=SUCCESS; return SUCCESS
        ctrl._tgt_freq=p['freq_hz']; ctrl._tgt_sweeps=p['sweeps']
        ctrl._tgt_dihedrals=p['dihedrals']; ctrl._tgt_pitches=p['pitches']
        return RUNNING


class PowerDiveBehaviour(BehaviourNode):
    """Rev N: Power dive — wings deployed, gravity+thrust."""
    name = 'power_dive'; priority = 8; interruptible = True

    def tick(self, context, ctrl):
        import mco_wave_library as wl
        pitch=context.get('pitch_deg',-15.0)
        V=context.get('V_ms',9.0); Ps=context.get('Ps',0.5)
        lever=context.get('lever_pct',40.0)
        safe,_=wl.power_dive_safe(V,context.get('h_agl_m',50.0),pitch)
        if not safe: self._state=FAILURE; return FAILURE
        p=wl.power_dive_params(pitch,V,Ps,ctrl.freq_hz,lever)
        ctrl._tgt_freq=p['freq_hz']; ctrl._tgt_sweeps=p['sweeps']
        ctrl._tgt_dihedrals=p['dihedrals']; ctrl._tgt_pitches=p['pitches']
        return RUNNING


class CrabBehaviour(BehaviourNode):
    """
    Rev O3: Crab manoeuvre — lateral translation without heading change.
    Port wings at 30° sweep → 17N lateral = 3.5g.
    Fin pitch at 44° cancels induced yaw → heading held.
    Stbd wings at 60° → normal forward thrust + lift.
    4 independent control axes simultaneously.
    """
    name = 'crab'; priority = 7; interruptible = True

    def tick(self, context, ctrl):
        import mco_wave_library as wl
        direction = context.get('direction', 'left')
        intensity = context.get('intensity', 1.0)
        V_ms  = context.get('V_ms', 9.0)
        bank  = context.get('bank_deg', 0.0)
        safe, reason = wl.crab_safe(V_ms,
            context.get('h_agl_m', 50.0), bank)
        if not safe:
            self._state = FAILURE; return FAILURE
        p = wl.crab_params(
            direction, intensity, V_ms,
            ctrl.freq_hz, heading_hold=True)
        ctrl._tgt_sweeps    = p['sweeps']
        ctrl._tgt_pitches   = p['pitches']
        ctrl._tgt_freq      = p['freq_hz']
        ctrl._tgt_dihedrals = [0.0] * 8
        return RUNNING


class FlatTurnBehaviour(BehaviourNode):
    """Rev K/L: Flat turn — phase asymmetry + fin pitch (W1+W4)."""
    name = 'flat_turn'; priority = 8; interruptible = True

    def tick(self, context, ctrl):
        import mco_wave_library as wl
        yaw  = context.get('yaw', 0.0)
        bank = context.get('bank_deg', 0.0)
        base = [0.0, 90.0, 180.0, 270.0, 0.0, 90.0, 180.0, 270.0]
        phases, sw_corr, freq, _ = wl.flat_turn_params(
            yaw, bank, base, ctrl.freq_hz)
        ctrl._tgt_freq = freq
        for i in range(8):
            ctrl._tgt_sweeps[i] = max(60.0, ctrl._tgt_sweeps[i] + sw_corr[i])
        # Rev L: add fin pitch for maximum yaw authority
        yaw_dir = 'right' if yaw >= 0 else 'left'
        ctrl._tgt_pitches = wl.fin_pitch_angles(abs(yaw) * 0.5, yaw_dir)
        # Rev O2: W4 vectored thrust
        if abs(yaw) > 0.3:
            vt = wl.w4_vectored_thrust_params(
                yaw_dir, abs(yaw)*0.7, ctrl.freq_hz)
            ctrl._tgt_sweeps[6] = vt['w4_port_sweep']
            ctrl._tgt_sweeps[7] = vt['w4_stbd_sweep']
        return RUNNING


# ── MISC BEHAVIOURS ──────────────────────────────────────────────

class WingConfigBehaviour(BehaviourNode):
    name = 'wing_config'; priority = 7; interruptible = True
    def tick(self, context, ctrl):
        import mco_wave_library as wl
        gs_id = context.get('gs_id', 'GS-0')
        V_ms  = context.get('V_ms', 8.0)
        ok, _ = wl.partial_stow_ok(gs_id, V_ms,
            context.get('h_agl_m', 50.0), 0.0)
        if not ok: gs_id = 'GS-0'
        try:
            cfg = wl.get_partial_stow_config(gs_id)
            for i in range(8):
                pair = i // 2 + 1
                if pair in cfg['stowed_pairs']:
                    ctrl._tgt_sweeps[i] = 210.0
        except Exception:
            pass
        return RUNNING


class TaxiFwdBehaviour(BehaviourNode):
    name = 'taxi_fwd'; priority = 10; interruptible = True
    def tick(self, context, ctrl):
        from mco_taxi_park import taxi_commands
        cmd = taxi_commands(context.get('throttle', 0.3),
                            context.get('yaw', 0.0))
        ctrl._tgt_freq = cmd['freq_hz']
        for i, wc in enumerate(cmd['wing_cmds']):
            ctrl._tgt_sweeps[i]  = wc['sweep']
            ctrl._tgt_pitches[i] = wc['pitch']
        return RUNNING


class TaxiRevBehaviour(BehaviourNode):
    name = 'taxi_rev'; priority = 10; interruptible = True
    def tick(self, context, ctrl):
        from mco_taxi_park import taxi_commands
        cmd = taxi_commands(context.get('throttle', 0.2),
                            context.get('yaw', 0.0), reverse=True)
        ctrl._tgt_freq = cmd['freq_hz']
        for i, wc in enumerate(cmd['wing_cmds']):
            ctrl._tgt_sweeps[i]  = wc['sweep']
            ctrl._tgt_pitches[i] = wc['pitch']
        return RUNNING


class InspectionBehaviour(BehaviourNode):
    name = 'inspection'; priority = 11; interruptible = True
    def tick(self, context, ctrl):
        ctrl._tgt_sweeps    = [90.0] * 8
        ctrl._tgt_freq      = 0.0
        ctrl._tgt_dihedrals = [0.0]  * 8
        ctrl._tgt_pitches   = [0.0]  * 8
        return RUNNING


class LoiterBehaviour(BehaviourNode):
    name = 'loiter'; priority = 8; interruptible = True
    def tick(self, context, ctrl):
        import mco_wave_library as wl
        Ps   = context.get('Ps', 0.1)
        V    = context.get('V_ms', 8.0)
        bank = context.get('bank_deg', 25.0)
        freq, sweep, dihs, pitch = wl.loiter_params(
            Ps, V, bank, 0.02, ctrl.freq_hz)
        ctrl._tgt_freq      = freq
        ctrl._tgt_sweeps    = [sweep] * 8
        ctrl._tgt_dihedrals = dihs
        ctrl._tgt_pitches   = [pitch] * 8
        return RUNNING


class OrbitBehaviour(BehaviourNode):
    name = 'orbit'; priority = 8; interruptible = True
    def tick(self, context, ctrl):
        import mco_wave_library as wl
        bank      = context.get('bank_deg', 30.0)
        direction = context.get('direction', 'right')
        ctrl._tgt_freq = 0.5
        sw_delta = bank * 0.4
        for i in range(8):
            is_port = (i % 2 == 0)
            inner = ((direction=='right') and not is_port) or \
                    ((direction=='left')  and     is_port)
            ctrl._tgt_sweeps[i] = 60.0 + sw_delta if inner else 60.0
        ctrl._tgt_pitches = [wl.bank_pitch_correction_deg(bank)] * 8
        return RUNNING


# ── BEHAVIOUR REGISTRY ───────────────────────────────────────────

BEHAVIOUR_REGISTRY = {
    Intent.RECOVER:         RecoverBehaviour,
    Intent.EMERGENCY_LAND:  RecoverBehaviour,
    Intent.STOOP:           StoopMacro,
    Intent.TIGHT_TURN:      TurnBehaviour,
    Intent.CHANDELLE:       ChandellesMacro,
    Intent.CF1_BURST:       CF1Macro,
    Intent.DART_LEFT:       DartMacro,
    Intent.DART_RIGHT:      DartMacro,
    Intent.TURN_LEFT:       TurnBehaviour,
    Intent.TURN_RIGHT:      TurnBehaviour,
    Intent.SPRINT:          SprintBehaviour,
    Intent.CLIMB:           ClimbBehaviour,
    Intent.DESCEND:         GlideBehaviour,
    Intent.GLIDE:           GlideBehaviour,
    Intent.APPROACH_NORMAL: GlideBehaviour,
    Intent.SOAR:            GlideBehaviour,
    Intent.CRUISE:          CruiseBehaviour,
    Intent.WING_ALL:        WingConfigBehaviour,
    Intent.WING_PARTIAL:    WingConfigBehaviour,
    Intent.WING_SINGLE:     WingConfigBehaviour,
    Intent.WING_SLOT:       WingConfigBehaviour,
    Intent.WING_GAP:        WingConfigBehaviour,
    Intent.WING_REAR:       WingConfigBehaviour,
    Intent.TAXI_FWD:        TaxiFwdBehaviour,
    Intent.TAXI_REV:        TaxiRevBehaviour,
    Intent.INSPECTION:      InspectionBehaviour,
    Intent.LOITER:          LoiterBehaviour,
    Intent.ORBIT_LEFT:      OrbitBehaviour,
    Intent.ORBIT_RIGHT:     OrbitBehaviour,
    Intent.FLAT_TURN_LEFT:  FlatTurnBehaviour,
    Intent.FLAT_TURN_RIGHT: FlatTurnBehaviour,
    # Approach / landing variants
    Intent.APPROACH_STEEP:   GlideBehaviour,
    Intent.LAND:             GlideBehaviour,
    Intent.APPROACH_SITE:    GlideBehaviour,
    Intent.APPROACH_ABORT:   ClimbBehaviour,
    Intent.RTH:              CruiseBehaviour,
    Intent.SPIRAL_DOWN:      GlideBehaviour,
    Intent.SLIP:             TurnBehaviour,
    Intent.SKID:             TurnBehaviour,
    Intent.ZOOM_CLIMB:       ZoomClimbMacro,
    Intent.POWER_DIVE:       PowerDiveBehaviour,
    Intent.STOOP_AUTO:       StoopMacro,
    Intent.CRAB_LEFT:        CrabBehaviour,
    Intent.CRAB_RIGHT:       CrabBehaviour,
}


# ── BEHAVIOUR DISPATCHER ─────────────────────────────────────────

class BehaviourDispatcher:
    def __init__(self, ctrl):
        self.ctrl         = ctrl
        self.interpreter  = IntentInterpreter()
        self.active       = CruiseBehaviour()
        self.active.on_enter({'t': 0.0})
        self._history     = []
        self._last_intent = Intent.CRUISE
        import mco_wave_library as wl
        self.em_report    = wl.BoydEMReport()

    def update(self, t, sticks, switches, context):
        context['t'] = t
        intent, confidence, params = self.interpreter.interpret(
            sticks, switches, context)
        context.update(params)
        is_safety = intent in (Intent.RECOVER, Intent.EMERGENCY_LAND, Intent.RTH)
        should_switch = False
        target_cls    = BEHAVIOUR_REGISTRY.get(intent, CruiseBehaviour)
        new_priority  = getattr(target_cls, 'priority', 10)
        if intent != self._last_intent or is_safety:
            if intent != self._last_intent:
                self.interpreter._on_intent_change()  # Rev O3-P: reset Boyd integral
            if self.active.can_be_preempted_by(new_priority, is_safety):
                should_switch = True
        if should_switch and target_cls != type(self.active):
            self._transition(t, intent, target_cls, context, params)
        self._last_intent = intent
        # Rev M: pass intent name to ControlState for urgency
        if hasattr(self.ctrl, '_intent_name'):
            self.ctrl._intent_name = self.active.name
        status = self.active.tick(context, self.ctrl)

        # Roll-out trigger on stick release
        if (isinstance(self.active, TurnBehaviour)
                and self.active._phase == TurnBehaviour.PHASE_BANKED):
            roll = sticks.get('roll', 0.0)
            if abs(roll) < 0.08:
                self.active.begin_rollout(context)

        if status in (SUCCESS, FAILURE):
            self._log(t, intent, type(self.active).__name__, f'completed:{status}')
            self._transition(t, Intent.CRUISE, CruiseBehaviour, context)
            status = RUNNING

        # Boyd EM record
        try:
            self.em_report.record(
                t=t, Ps=context.get('Ps', 0.0),
                h_agl_m=context.get('h_agl_m', 0.0),
                V_ms=context.get('V_ms', 0.0),
                bank_deg=context.get('bank_deg', 0.0),
                mode=self.active.name)
        except Exception:
            pass

        return self.active.name, intent, status

    def _transition(self, t, intent, target_cls, context, params=None):
        old_name = self.active.name
        self.active.on_exit(context, completed=False)
        self._log(t, intent, old_name, 'preempted')
        if params:
            context.update(params)
        self.active = target_cls()
        self.active.on_enter(context)
        self._log(t, intent, target_cls.name, 'entered')

    def _log(self, t, intent, behaviour, event):
        entry = {'t': round(t,2), 'intent': intent,
                 'behaviour': behaviour, 'event': event}
        self._history.append(entry)

    def behaviour_log_summary(self):
        lines = ['  BEHAVIOUR TRANSITION LOG:']
        prev  = None
        for e in self._history:
            if e['behaviour'] != prev:
                lines.append(f"  t={e['t']:6.1f}s  {e['event']:<12}"
                             f"  {e['behaviour']:<20}  intent:{e['intent']}")
                prev = e['behaviour']
        return lines


# ── MACRO ABORT TRIGGERS ─────────────────────────────────────────

MACRO_LIBRARY = {'stoop': StoopMacro, 'cf1_burst': CF1Macro,
                 'chandelle': ChandellesMacro, 'dart': DartMacro}

MACRO_ABORT_TRIGGERS = {
    'stoop': [
        (lambda ctx: ctx.get('h_agl_m', 999) < 25.0, 'hard floor 25m AGL'),
        (lambda ctx: ctx.get('V_ms', 0) > 35.0,       'overspeed'),
        (lambda ctx: ctx.get('stall_imminent', False), 'stall warning'),
    ],
    'cf1_burst': [
        (lambda ctx: ctx.get('V_ms', 0) > 10.0,       'speed too high'),
        (lambda ctx: ctx.get('stall_imminent', False), 'stall warning'),
    ],
    'chandelle': [
        (lambda ctx: ctx.get('h_agl_m', 999) < 30.0,  'altitude floor'),
        (lambda ctx: ctx.get('V_ms', 0) < 5.5,         'airspeed too low'),
    ],
    'dart': [
        (lambda ctx: ctx.get('h_agl_m', 999) < 30.0,  'hard floor 30m AGL'),
        (lambda ctx: ctx.get('stall_imminent', False), 'stall warning'),
    ],
}


def check_macro_abort(macro_name, context):
    for fn, reason in MACRO_ABORT_TRIGGERS.get(macro_name, []):
        try:
            if fn(context): return True, reason
        except Exception:
            pass
    return False, ""
