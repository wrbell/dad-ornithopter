#!/usr/bin/env python3
"""
MCO-1D Ground Station Client
=============================
Laptop-side application for real-time monitoring during TC-02/TC-03
bench tests and in-flight operations.

REQUIREMENTS: Python 3.8+ only. No external packages.

USAGE:
    python3 mco_ground_station.py [pi_ip] [port]

    Defaults: pi_ip=192.168.7.1  port=5760

PHYSICAL CONNECTION:
    1. Ethernet cable: Pi eth0 → laptop ethernet port
       Set Pi  static IP: 192.168.7.1
       Set laptop static IP: 192.168.7.2
       (One-time setup — see NETWORK SETUP below)

    2. USB-ethernet adapter on Pi side if Pi has no ethernet port
       (Pi Zero 2W needs this; Pi 3/4 have built-in ethernet)

    3. WiFi fallback: connect laptop to Pi hotspot MCO_NETWORK

NETWORK SETUP (one time, on laptop — macOS):
    System Settings → Network → Ethernet → Details
    → Configure IPv4: Manually
    → IP: 192.168.7.2   Subnet: 255.255.255.0   Router: (blank)

NETWORK SETUP (one time, on laptop — Linux):
    sudo ip addr add 192.168.7.2/24 dev eth0

NETWORK SETUP (on Pi — add to /etc/dhcpcd.conf):
    interface eth0
    static ip_address=192.168.7.1/24

WHAT THIS SHOWS:
    ┌─ FLIGHT STATE ─────────────────────────────────────────────┐
    │ Live sensor readings and optimizer states at 10 Hz          │
    └────────────────────────────────────────────────────────────┘
    ┌─ TC-02/TC-03 DATA ─────────────────────────────────────────┐
    │ Each test point displayed as it arrives from the load cell  │
    │ Running CT estimate, best config found so far               │
    └────────────────────────────────────────────────────────────┘
    ┌─ OPTIMIZER STATUS ──────────────────────────────────────────┐
    │ All six loops, aero calibration, zone remap candidates      │
    └────────────────────────────────────────────────────────────┘
    ┌─ COMMAND PROMPT ────────────────────────────────────────────┐
    │ Type commands directly — Tab completion, history            │
    └────────────────────────────────────────────────────────────┘
"""

import sys
import socket
import json
import threading
import time
import os
import readline   # gives Tab completion and history in the prompt

# ── Terminal colours (ANSI) ───────────────────────────────────────
def _c(code): return f'\033[{code}m'
RESET   = _c(0)
BOLD    = _c(1)
DIM     = _c(2)
RED     = _c(31)
GREEN   = _c(32)
YELLOW  = _c(33)
BLUE    = _c(34)
MAGENTA = _c(35)
CYAN    = _c(36)
WHITE   = _c(37)
BG_DARK = _c('48;5;234')   # very dark grey background for headers

def colour(text, *codes):
    return ''.join(codes) + text + RESET

# ── Connection ────────────────────────────────────────────────────
DEFAULT_IP   = '192.168.7.1'
DEFAULT_PORT = 5757

class GSConnection:
    """Manages the TCP connection to the Pi."""

    def __init__(self, host, port):
        self.host    = host
        self.port    = port
        self._sock   = None
        self._lock   = threading.Lock()
        self._alive  = False
        self._buf    = ''
        self.on_msg  = None   # callback: fn(dict)
        self._thread = None

    def connect(self):
        """Connect to Pi. Returns True on success."""
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.settimeout(5.0)
            s.connect((self.host, self.port))
            s.settimeout(0.5)
            self._sock  = s
            self._alive = True
            self._thread = threading.Thread(
                target=self._recv_loop, daemon=True, name='GS-Recv')
            self._thread.start()
            return True
        except Exception as e:
            return False, str(e)

    def send(self, cmd_dict):
        """Send a command dict to the Pi."""
        if not self._alive or not self._sock:
            return False
        try:
            with self._lock:
                msg = json.dumps(cmd_dict) + '\n'
                self._sock.sendall(msg.encode())
            return True
        except Exception:
            self._alive = False
            return False

    def _recv_loop(self):
        """Background thread — receive and parse messages."""
        while self._alive:
            try:
                chunk = self._sock.recv(4096).decode('utf-8', errors='ignore')
                if not chunk:
                    self._alive = False
                    break
                self._buf += chunk
                while '\n' in self._buf:
                    line, self._buf = self._buf.split('\n', 1)
                    line = line.strip()
                    if not line:
                        continue
                    try:
                        msg = json.loads(line)
                        if self.on_msg:
                            self.on_msg(msg)
                    except json.JSONDecodeError:
                        pass
            except socket.timeout:
                continue
            except Exception:
                self._alive = False
                break

    def close(self):
        self._alive = False
        if self._sock:
            try:
                self._sock.close()
            except Exception:
                pass

    @property
    def connected(self):
        return self._alive


# ── State store ───────────────────────────────────────────────────
class AppState:
    """Holds the most recent data from the Pi."""

    def __init__(self):
        self.telem        = {}
        self.tc02_points  = []    # list of tc02 test points received
        self.tc03_points  = []
        self.aero_cal     = {}
        self.lm6_zones    = {}
        self.alerts       = []    # (timestamp, msg)
        self.last_update  = None
        self._lock        = threading.Lock()

    def ingest(self, msg):
        """Process an incoming message from the Pi."""
        if not isinstance(msg, dict):
            if isinstance(msg, str):
                with self._lock:
                    self.alerts.append((time.time(), msg))
                    if len(self.alerts) > 20:
                        self.alerts = self.alerts[-20:]
            return

        t = msg.get('type', '')
        with self._lock:
            if t == 'telemetry':
                self.telem       = msg.get('data', {})
                self.last_update = time.monotonic()
            elif t in ('tc02_sweep_point', 'tc02_pitch_point',
                        'tc02_point'):
                self.tc02_points.append(msg)
                if len(self.tc02_points) > 200:
                    self.tc02_points = self.tc02_points[-200:]
            elif t in ('tc03_point',):
                self.tc03_points.append(msg)
            elif t == 'aero_cal':
                self.aero_cal = msg.get('data', {})
            elif t == 'lm6_zones':
                self.lm6_zones = msg.get('data', {})
            elif t == 'ack':
                ack_text = msg.get('msg', str(msg))
                self.alerts.append((time.time(), f"[ACK] {ack_text}"))
                if len(self.alerts) > 20:
                    self.alerts = self.alerts[-20:]
            elif t == 'alert':
                self.alerts.append((time.time(), msg.get('msg', '')))
                if len(self.alerts) > 20:
                    self.alerts = self.alerts[-20:]


# ── Display ───────────────────────────────────────────────────────
def clear():
    os.system('clear' if os.name != 'nt' else 'cls')

def _bar(value, lo, hi, width=20, fill='█', empty='░'):
    """Render a simple progress bar."""
    frac = max(0.0, min(1.0, (value - lo) / (hi - lo)))
    n    = int(frac * width)
    return fill * n + empty * (width - n)

def _fmt(val, unit='', decimals=2, warn_above=None, warn_below=None):
    """Format a value with optional colour warning."""
    if val is None:
        return colour('—', DIM)
    text = f'{val:.{decimals}f}{unit}'
    if warn_above is not None and val > warn_above:
        return colour(text, RED, BOLD)
    if warn_below is not None and val < warn_below:
        return colour(text, YELLOW)
    return text

def render_telemetry(state):
    t = state.telem
    if not t:
        return colour('  Awaiting telemetry...', DIM)

    lines = []
    lines.append(colour('  FLIGHT STATE', BOLD, CYAN))

    # Speed and height
    V    = t.get('V_ms', 0)
    h    = t.get('h_agl_m', 0)
    Ps   = t.get('Ps_measured', 0)
    bank = t.get('bank_deg', 0)
    freq = t.get('freq_hz', 0)
    lev  = t.get('lever_pct', 50)

    lines.append(
        f"  V {_fmt(V,'m/s',1)}  h {_fmt(h,'m',2)}  "
        f"Ps {_fmt(Ps,'m/s',3,warn_below=-0.5)}  "
        f"bank {_fmt(bank,'°',1)}  "
        f"freq {_fmt(freq,'Hz',2)}  "
        f"lever {_fmt(lev,'%',1)}")

    # Optimizer states
    vm  = t.get('violent_manoeuvre', False)
    vr  = t.get('violent_reason', '')
    di  = t.get('discovery_enabled', False)
    bfvc= t.get('bfvc_active', False)
    stol= t.get('stol_backside', False)
    gef = t.get('gef_hold', False)
    wf  = t.get('wfaoa_active', False)
    l6c = t.get('l6_cycle', 0)
    l6n = t.get('l6_candidates', 0)

    vm_str  = colour(f'FREEZE:{vr}', RED, BOLD) if vm else colour('normal', GREEN)
    di_str  = colour('ON', GREEN) if di else colour('off', DIM)

    lines.append(
        f"  Manoeuvre {vm_str}  "
        f"Discovery {di_str}  "
        f"L6 cycle {l6c} ({l6n} candidates)")

    flags = []
    if bfvc: flags.append(colour('BFVC', CYAN))
    if stol: flags.append(colour('STOL-BS', YELLOW))
    if gef:  flags.append(colour('GEF', MAGENTA))
    if wf:   flags.append(colour('WFAOA', BLUE))
    if flags:
        lines.append(f"  Active: {' '.join(flags)}")

    # Aero calibration
    CL  = t.get('CL_MAX', 0)
    vs  = t.get('VS_OGE_MS', 0)
    gc  = t.get('gl_conf', 0)
    sc  = t.get('stall_conf', 0)
    lines.append(
        f"\n  {colour('AERO CAL',BOLD,YELLOW)}  "
        f"CL_max {_fmt(CL,'',3)}  "
        f"Vs {_fmt(vs,'m/s',2)}  "
        f"glide obs {gc}  stall obs {sc}")

    return '\n'.join(lines)


def render_tc02(state):
    pts = state.tc02_points
    if not pts:
        return colour('  No TC-02 data yet', DIM)

    lines = [colour(f'  TC-02 TEST POINTS ({len(pts)} received)', BOLD, GREEN)]

    # Column headers
    lines.append(colour(
        f"  {'#':>3}  {'Type':<12}  {'Lift N':>8}  "
        f"{'CT':>8}  {'T/W':>6}  {'Note'}", DIM))
    lines.append(colour('  ' + '─'*65, DIM))

    W_N = 33.8   # MCO weight in N — approximate

    for i, pt in enumerate(pts[-15:], len(pts)-min(15,len(pts))+1):
        t    = pt.get('type', '')
        lift = pt.get('lift_n', pt.get('world_lift', 0))
        CT   = pt.get('CT', 0)
        TW   = pt.get('TW', lift/W_N if lift else 0)
        freq = pt.get('freq_hz', 0)

        if t == 'tc02_sweep_point':
            note = (f"sweep={pt.get('sweep_deg','?')}° "
                    f"dih={pt.get('dihedral_deg','?')}°")
        elif t == 'tc02_pitch_point':
            note = f"pitch={pt.get('pitch_deg','?')}° f={freq:.1f}Hz"
        else:
            note = f"f={freq:.1f}Hz {pt.get('profile','')}"

        tw_str = _fmt(TW, '', 2,
                       warn_above=1.0 if TW > 0 else None)
        cl_str = colour('✓', GREEN) if TW >= 1.0 else colour('✗', RED)

        lines.append(
            f"  {i:>3}  {t[5:]:<12}  {lift:>8.2f}N  "
            f"{CT:>8.4f}  {tw_str:>6}  {note}  {cl_str}")

    return '\n'.join(lines)


def render_lm6(state):
    zm = state.lm6_zones.get('zone_map', {})
    if not zm:
        return colour('  No zone map data yet', DIM)

    lines = [colour('  LEVER ZONE MAP', BOLD, MAGENTA)]
    cands = state.lm6_zones.get('candidates', {})

    for zone, modes in sorted(zm.items()):
        cand = cands.get(zone)
        cand_str = ''
        if cand:
            cand_str = colour(
                f" → candidate: {cand.get('candidate_mode','?')} "
                f"Δ={cand.get('delta',0):+.3f}", YELLOW)
        top_modes = ', '.join(list(modes)[:3])
        if len(modes) > 3:
            top_modes += f' +{len(modes)-3}'
        lines.append(f"  {zone:<18}: {top_modes}{cand_str}")

    r = state.lm6_zones
    lines.append(
        colour(f"\n  Remaps: {r.get('remaps',0)}  "
               f"Retired: {r.get('retired',0)}  "
               f"Discovered: {r.get('discovered',0)}", DIM))
    return '\n'.join(lines)


def render_alerts(state):
    if not state.alerts:
        return ''
    lines = [colour('  ALERTS', BOLD, RED)]
    for ts, msg in state.alerts[-5:]:
        t_str = time.strftime('%H:%M:%S', time.localtime(ts))
        lines.append(f"  {colour(t_str, DIM)}  {msg}")
    return '\n'.join(lines)


def render_header(conn_status):
    width = 70
    ts    = time.strftime('%H:%M:%S')
    conn  = (colour('CONNECTED', GREEN, BOLD) if conn_status
             else colour('DISCONNECTED', RED, BOLD))
    title = colour(' MCO-1D GROUND STATION ', BOLD, WHITE, BG_DARK)
    return (f"{title}  {conn}  {colour(ts, DIM)}\n"
            + colour('─' * width, DIM))



# ══════════════════════════════════════════════════════════════════
# OPERATOR COMMAND REFERENCE — complete guide to all 217 commands
# ══════════════════════════════════════════════════════════════════
#
# USAGE PATTERNS:
#   help                  — this overview
#   help pre-flight       — pre-flight checklist
#   help hover            — hover phase commands
#   help cruise           — cruise/FW phase commands
#   help optimizer        — learning and discovery
#   help safety           — fault and emergency
#   help sar              — SAR mission operations
#   help sharing          — fleet knowledge sharing
#   help <word>           — search commands by keyword
#   ? what do I type to arm the drone
#   ? how do I enable discovery mode
#   ? show me all status commands
# ══════════════════════════════════════════════════════════════════

# Structured command catalogue — all 217 commands organised by phase and task
_CMD_CATALOGUE = {

  'PRE-FLIGHT': {
    '_desc': 'Commands to run before each flight',
    'arm':                 'Arm aircraft — enables ODrives, resets learning for new flight',
    'disarm':              'Disarm — safe state, all wings idle',
    'persist_load_all':    'Restore all learning from previous flights (auto at startup)',
    'persist_status':      'Show what learning is saved on disk',
    'soft_stop_status':    'Check soft-stop (rope protection) system',
    'cg_status':           'Show centre-of-gravity estimate and stability margin',
    'failure_briefing':    'Pre-flight failure mode summary — read before TC-04',
    'fault_summary':       'Full fault and degradation state',
    'airborne_status':     'Show airborne detection state',
    'attitude_hold_status':'Show attitude hold (electronic dihedral) gains and state',
    'hover_mode_status':   'Show hover mode thresholds and calibration',
    'advanced_modes_status':'Show position hold, turn coord, crew hover, airspeed hold',
    'inbox_check':         'Scan USB/inbox for pending hypotheses or knowledge files',
  },

  'HOVER': {
    '_desc': 'Commands for hover phase (V < 3 m/s)',
    'crew_hover_enable':   'Velocity-referenced hover — stick commands speed not angle (default ON)',
    'crew_hover_disable':  'Attitude-reference hover — stick commands bank angle (classic feel)',
    'position_hold_engage':'Capture current GPS position and hold it',
    'position_hold_release':'Release position hold — return to attitude control',
    'position_hold_enable':'Enable GPS position hold system',
    'position_hold_disable':'Disable GPS position hold',
    'cg_calibrate_now':    'Trigger CG calibration pass manually',
    'hover_mode_calibrate':'Update hover frequency from TC-02 measured result',
    'airborne_reset_flight':'Reset per-flight airborne detection (called by arm automatically)',
    'gnd_clear_status':    'Ground clearance controller — wing amplitude vs AGL height',
    'gnd_state_status':    'Ground state and safe operating boundaries',
  },

  'CRUISE': {
    '_desc': 'Commands for fixed-wing cruise phase (V > 5 m/s)',
    'airspeed_hold_enable':'Hold target airspeed via frequency control (engages after stable lever)',
    'airspeed_hold_disable':'Disable airspeed hold',
    'turn_coord_enable':   'Enable automatic turn coordination (yaw bias in banked turns)',
    'turn_coord_disable':  'Disable turn coordination — manual pedal only',
    'corridor_guard_enable':'Enable conversion corridor guard (V-22 style — protects morphing)',
    'corridor_guard_disable':'Disable corridor guard (operator accepts envelope responsibility)',
    'attitude_hold_enable':'Enable electronic dihedral (hands-off stability)',
    'attitude_hold_disable':'Disable attitude hold — requires constant pilot input',
    'attitude_hold_tune':  'Tune attitude hold gain: attitude_hold_tune kp_outer 0.08',
    'strouhal_status':     'Show Strouhal number vs optimal for current airspeed',
    'h35_engage':          'Engage thrust/lift separation (stop some wing pairs for efficiency)',
    'h35_disengage':       'Return to all-flapping configuration',
    'h35_status':          'H-35 thrust/lift separation status',
    'h36_engage':          'Engage minimum-power loiter/ferry configuration',
    'h36_disengage':       'Return from loiter/ferry',
    'h36_status':          'H-36 loiter/ferry status and endurance estimate',
  },

  'MORPHING': {
    '_desc': 'Wing morphing and configuration commands',
    'h33_engage':          'Variable wing area — stop a pair: h33_engage pair=W4 state=B',
    'h33_disengage':       'Return to normal 4-pair configuration',
    'h33_status':          'H-33 variable wing area status',
    'h34_engage':          'Dynamic sweep kinematics variant: h34_engage variant=V2',
    'h34_disengage':       'Return from dynamic sweep variant',
    'h34_status':          'H-34 dynamic sweep status',
    'h36_ferry_report':    'Ferry configuration efficiency report',
    'h36_loiter_report':   'Loiter endurance prediction report',
    'h35_lift_report':     'Lift/thrust breakdown for current H-35 configuration',
    'morphing_speed_report':'Full speed leaderboard — which config is fastest at each speed',
    'morphing_speed_reset': 'Reset morphing speed leaderboard (after major rebuild)',
    'morphing_speed_start': 'Start morphing speed survey at current conditions',
    'h33_h34_h35_disengage_all': 'Emergency return to baseline — all morphing off',
  },

  'OPTIMIZER': {
    '_desc': 'Learning and discovery — how the aircraft improves',
    'discovery_enable':    'Start unconstrained discovery (enable above 15m AGL)',
    'discovery_disable':   'Stop discovery — return to normal optimizer',
    'discovery_status':    'Show discovery state, best config found, queue',
    'discovery_lock_best': 'Lock the best discovered config into permanent use',
    'mechanism_weight_report': 'Show which roll/pitch/yaw mechanisms are ranked best',
    'mechanism_weight_reset':  'Reset mechanism weights to equal (after rebuild)',
    'save_mechanism_weights':  'Save learned weights to disk now',
    'load_mechanism_weights':  'Load weights from disk (auto at startup)',
    'seed_hypotheses':     'Pre-load aerodynamic hypotheses into discovery queue',
    'hypothesis_results':  'Show which of the 17 research hypotheses were promoted',
    'hypothesis_status':   'Show hypothesis queue, evaluated count, promotions',
    'hypothesis_strouhal_targets': 'Show optimal frequency targets for current airspeed',
    'learning_history':    'Cross-flight improvement trend — what has gotten better',
    'persist_checkpoint':  'Save all learning mid-flight (without shutdown)',
    'freeze_optimizers':   'Freeze all learning loops (for baseline testing)',
    'restore_optimizers':  'Unfreeze all learning loops',
    'l5_status':           'Loop 5 yaw modality optimizer — dart/crab/split blend',
    'l9_status':           'Meta-optimizer — which constants are being tuned',
    'exploration_status':  'Full exploration system status (all loops)',
    'auto_disc_enable':    'Enable continuous autonomous discovery mode',
    'auto_disc_disable':   'Disable continuous autonomous discovery',
    'auto_disc_status':    'Show autonomous discovery state',
    'lever_learning_status': 'Show what the long-term lever learning has found',
    'polar_survey_start':  'Start autonomous in-flight polar survey',
  },

  'SAR': {
    '_desc': 'SAR mission operations',
    'crew_hover_enable':   'Velocity-referenced hover for SAR approach (default ON in hover)',
    'position_hold_engage':'Freeze aircraft position — crew operates below',
    'position_hold_release':'Release position hold when redeploying',
    'advanced_modes_status': 'Full status of all 5 advanced modes including SAR modes',
    'dart_status':         'Dart control status — pedal dwell → heading change',
    'dart_reset':          'Reset dart state after aborted manoeuvre',
    'dart_set_mode_weights': 'Override dart mode blend: dart_set_mode_weights 0.5 0.3 0.2',
    'h36_loiter_report':   'Endurance estimate for current loiter configuration',
    'cg_set_margin':       'Override stability margin for specific intent (mm)',
  },

  'SAFETY': {
    '_desc': 'Fault handling and emergency commands',
    'fault_status':        'Complete fault and degradation state',
    'fault_summary':       'Concise fault summary for pilot',
    'fault_wing_restore':  'Mark a wing healthy after physical repair or clear_errors',
    'controlled_landing_status': 'land_now mode status and descent rate settings',
    'failure_briefing':    'Full failure mode briefing (run pre-flight)',
    'query_failure':       'Look up response for a specific wing failure combination',
    'prog_sim_request':    'Request progressive failure simulation (requires approval)',
    'prog_sim_approve':    'Approve and run progressive failure simulation',
    'prog_sim_status':     'Show progressive failure simulation registry',
    'emergency_stop':      'Immediate motor stop (panel/RC also triggers this)',
    'stall_detector_status': 'Per-pair stall likelihood — warn if >0.45',
    'rope_health_status':  'Rope fatigue state for all 8 drives',
    'violent_manoeuvre_status': 'Show violent manoeuvre authority state',
    'wing_sim_request':    'Request wing failure simulation',
    'wing_sim_approve':    'Approve wing failure simulation',
    'wing_sim_status':     'Show wing simulation registry',
  },

  'FLEET SHARING': {
    '_desc': 'Sharing learning between drones',
    'knowledge_export_now':'Export everything learned to outbox (full + fleet files)',
    'knowledge_import':    'Import from another drone: knowledge_import /path/file.json',
    'merge_fleet_knowledge': 'Merge N drone files: merge_fleet_knowledge f1.json f2.json',
    'fleet_status':        'Show outbox contents and this drone learning summary',
    'gs_inject_hypothesis':'Inject your idea live: gs_inject_hypothesis name="try this" cascade_phase_deg=37',
    'gs_inject_parameter': 'Change a parameter live: gs_inject_parameter DISCOVERY_DWELL_S 30',
    'gs_inject_note':      'Record an observation: gs_inject_note "felt sluggish at 8m/s"',
    'inbox_check':         'Scan USB and inbox for pending files',
    'user_hypothesis_status': 'Show injected user hypotheses and their evaluation status',
    'persist_save_all':    'Save all learning state now (also runs at shutdown)',
    'persist_load_all':    'Reload all learning from disk',
  },

  'CERTIFICATION': {
    '_desc': 'FAA/EASA certification tools',
    'cert_status':         'Overall certification readiness summary',
    'cert_fmea':           'Run FMEA generator (13 failure modes, 0 catastrophic)',
    'cert_fha':            'Generate Functional Hazard Analysis (12 functions, ARP4754A)',
    'cert_do178c':         'Generate DO-178C software traceability matrix',
    'cert_hq_analyse':     'Run HQ analyser on latest flight log (ADS-33 criteria)',
    'cert_flutter_report': 'Flutter boundary log from TC-03',
    'cert_loads_report':   'Structural loads peak g per flight',
    'cert_rope_report':    'Rope life tracker (Miner Rule)',
  },

  'TEST CARDS': {
    '_desc': 'Commissioning test card commands',
    'tc01_start':          'Start TC-01 bench test',
    'tc01_status':         'TC-01 progress',
    'tc01_confirm':        'Confirm current TC-01 step',
    'tc01_abort':          'Abort TC-01',
    'tc02_start':          'Start TC-02 captive hover',
    'tc02_status':         'TC-02 progress',
    'tc02_continue':       'Continue to next TC-02 step',
    'tc02_abort':          'Abort TC-02',
    'tc03_start':          'Start TC-03 truck forward flight',
    'tc03_status':         'TC-03 progress',
    'tc03_continue':       'Continue to next TC-03 step',
    'tc03_abort':          'Abort TC-03',
    'tc04_start':          'Start TC-04 first free flight',
    'tc04_status':         'TC-04 progress',
    'tc04_continue':       'Continue TC-04',
    'tc04_abort':          'Abort TC-04',
    'tc_results':          'Export all TC results',
  },

  'DIAGNOSTICS': {
    '_desc': 'Detailed system diagnostics',
    'soft_stop_status':    'Soft-stop (rope protection) per-wing state',
    'pitot_status':        'Pitot airspeed sensor state',
    'motor_health':        'ODrive motor health per wing',
    'motor_peaks':         'Peak current per motor this flight',
    'servo_health_report': 'Servo tracking error history',
    'servo_bandwidth_status': 'Servo bandwidth model state',
    'stroke_position_status': 'Actual vs commanded stroke position per pair',
    'effective_aoa_status':'Effective angle of attack (geometric + induced)',
    'lift_balance_report': 'Port vs starboard lift balance',
    'trim_balance_report': 'Trim balance and asymmetry',
    'thrust_lift_status':  'Thrust/lift decomposition for current config',
    'aero_cal_status':     'Aerodynamic calibration (CL_MAX, stall speed, glide)',
    'aero_cal_reset':      'Reset aero calibration to library defaults',
    'force_waveform_status':'Instantaneous force waveform (8 wings × 50Hz)',
    'wake_timing_status':  'Wake timing from cascade state',
    'margin_status':       'Flight margin state (stall, structural, envelope)',
    'stall_detector_status': 'Per-pair stall likelihood',
    'precision_status':    'Precision metrics and lever map state',
    'ground_ops_status':   'Ground operations mode and clearances',
    'pilot_intent_status': 'Current intent and mechanism weights',
    'l4_status':           'Loop 4 optimizer status',
    'knowledge_status':    'What has been learned — knowledge state summary',
    'endurance_status':    'Honest power accounting & endurance hypotheses status (H-SERVO-1, H-DIST-1, H-SWEEP-1, H-DUTY-1)',
  },
}

# Keyword index — maps words to commands (for search)
_CMD_KEYWORDS = {}
for _section, _cmds in _CMD_CATALOGUE.items():
    for _cmd, _desc in _cmds.items():
        if _cmd.startswith('_'): continue
        words = set((_cmd + ' ' + _desc).lower().replace('_',' ').split())
        for w in words:
            if len(w) >= 3:
                _CMD_KEYWORDS.setdefault(w, set()).add(_cmd)

# Natural language intent map — plain English phrases → commands
_INTENT_MAP = [
    # Startup / arming
    (['arm','start','enable','power up','switch on','ready'],
     ['arm', 'persist_load_all', 'inbox_check']),
    (['disarm','stop','safe','shutdown','power down','switch off'],
     ['disarm', 'persist_save_all']),
    (['pre-flight','preflight','before flight','checklist'],
     ['failure_briefing', 'soft_stop_status', 'cg_status',
      'attitude_hold_status', 'advanced_modes_status']),

    # Status / what is happening
    (['status','what','how','show','display','report','check'],
     ['status', 'fault_summary', 'advanced_modes_status',
      'mechanism_weight_report']),
    (['learning','optimizer','improve','weights','mechanism'],
     ['mechanism_weight_report', 'learning_history', 'exploration_status']),
    (['discovery','explore','experiment','try'],
     ['discovery_enable', 'discovery_status', 'seed_hypotheses']),

    # SAR operations
    (['sar','rescue','hover','hold position','freeze','stop moving'],
     ['crew_hover_enable', 'position_hold_engage']),
    (['release','move','go','unfreeze','continue'],
     ['position_hold_release', 'crew_hover_enable']),

    # Speed / performance
    (['fast','speed','cruise','top speed','accelerate','sprint'],
     ['airspeed_hold_enable', 'h35_engage', 'strouhal_status',
      'morphing_speed_report']),
    (['slow','loiter','endurance','range','battery','long'],
     ['h36_engage', 'h36_loiter_report', 'airspeed_hold_enable']),

    # Hypothesis / idea
    (['idea','try','test','hypothesis','suggest','inject'],
     ['gs_inject_hypothesis', 'seed_hypotheses', 'hypothesis_status']),

    # Sharing / fleet
    (['share','export','send','other drone','fleet','merge'],
     ['knowledge_export_now', 'fleet_status', 'merge_fleet_knowledge']),
    (['import','load','receive','from','usb'],
     ['knowledge_import', 'inbox_check']),

    # Safety / fault
    (['fault','fail','damage','broken','wing','emergency','land'],
     ['fault_summary', 'controlled_landing_status', 'failure_briefing']),

    # Dart
    (['dart','turn','yaw','manoeuvre','heading'],
     ['dart_status', 'turn_coord_enable', 'l5_status']),

    # Certification
    (['cert','certif','faa','easa','compliance','document'],
     ['cert_status', 'cert_fmea', 'cert_fha', 'cert_do178c']),
]


def render_help(topic=None):
    """
    Comprehensive operator help.
    help                  — overview
    help <section>        — section detail (hover, cruise, optimizer, sar, etc.)
    help <keyword>        — search by keyword
    """
    if topic:
        topic_l = topic.lower().strip()

        # Try section match
        for section_name, section_cmds in _CMD_CATALOGUE.items():
            if topic_l in section_name.lower():
                lines = [colour(f'\n  ── {section_name} ──', BOLD, CYAN),
                         colour(f'  {section_cmds["_desc"]}', DIM)]
                for cmd, desc in section_cmds.items():
                    if cmd.startswith('_'): continue
                    lines.append(f"  {colour(cmd, '', BOLD):<40}  {desc}")
                lines.append(colour('\n  Type any command to send it to the aircraft.', DIM))
                lines.append(colour('  Use Tab to complete command names.', DIM))
                return '\n'.join(lines)

        # Keyword search
        hits = set()
        for word in topic_l.replace('-',' ').split():
            hits |= _CMD_KEYWORDS.get(word, set())
        if hits:
            lines = [colour(f'\n  ── COMMANDS matching "{topic}" ──', BOLD, CYAN)]
            for cmd in sorted(hits)[:20]:
                # Find description
                for sec_cmds in _CMD_CATALOGUE.values():
                    if cmd in sec_cmds:
                        lines.append(f"  {colour(cmd,'',BOLD):<40}  {sec_cmds[cmd]}")
                        break
            if len(hits) > 20:
                lines.append(colour(f'  ... and {len(hits)-20} more', DIM))
            return '\n'.join(lines)

        return colour(f'  No commands match "{topic}". Try: help hover, help optimizer, help sar', RED)

    # Overview
    lines = [
        colour('\n  MCO-1D GROUND STATION — COMMAND GUIDE', BOLD, CYAN),
        colour('  ─────────────────────────────────────────────────────────', DIM),
        '',
        colour('  QUICK COMMANDS (most common operations):', BOLD),
        f"  {'arm':<30}  Arm aircraft and restore prior learning",
        f"  {'disarm':<30}  Safe state — wings idle",
        f"  {'status':<30}  Full telemetry snapshot",
        f"  {'fault_summary':<30}  Fault and degradation state",
        f"  {'discovery on [h]':<30}  Start discovery mode above height h (metres)",
        f"  {'discovery off':<30}  Stop discovery",
        f"  {'crew_hover_enable':<30}  SAR hover (velocity-reference, default ON)",
        f"  {'position_hold_engage':<30}  Freeze position over casualty site",
        f"  {'knowledge_export_now':<30}  Export all learning to outbox",
        f"  {'learning_history':<30}  Cross-flight improvement trend",
        '',
        colour('  HELP BY SECTION (type: help <section>):', BOLD),
    ]
    for section, cmds in _CMD_CATALOGUE.items():
        n = sum(1 for k in cmds if not k.startswith('_'))
        lines.append(f"  {colour('help '+section.lower().replace(' ','-'),'',BOLD):<35}"
                     f"  {cmds['_desc']}  ({n} commands)")

    lines += [
        '',
        colour('  SEARCH (type: help <keyword>):', BOLD),
        '  help arm         help fault        help sar          help sharing',
        '  help position    help discovery    help dart         help morphing',
        '  help speed       help loiter       help hypothesis   help cert',
        '',
        colour('  NATURAL LANGUAGE (type: ? your question):', BOLD),
        '  ? how do I arm the drone',
        '  ? how do I start discovery mode',
        '  ? what commands are available for SAR',
        '  ? I want to share learning with another drone',
        '  ? how do I inject my own hypothesis',
        '',
        colour('  TAB COMPLETION: press Tab to complete any command name.', DIM),
        colour('  HISTORY: Up/Down arrow keys scroll through previous commands.', DIM),
        colour(f'  TOTAL COMMANDS: 217 available — all listed in help sections above.', DIM),
    ]
    return '\n'.join(lines)


def handle_natural_language(question):
    """
    Handle a ? question from the operator.
    Matches intent to relevant commands.
    Returns a helpful response string.
    """
    q = question.lower()

    # Check intent map
    best_cmds = []
    best_score = 0
    for keywords, cmds in _INTENT_MAP:
        score = sum(1 for kw in keywords if kw in q)
        if score > best_score:
            best_score = score
            best_cmds = cmds

    # Also keyword search
    search_hits = set()
    for word in q.split():
        if len(word) >= 4:
            search_hits |= _CMD_KEYWORDS.get(word, set())

    all_cmds = list(best_cmds) + [c for c in sorted(search_hits)
                                    if c not in best_cmds][:5]

    if not all_cmds:
        return (colour('  No matching commands found. Try: help', RED) + '\n' +
                colour('  Or ask more specifically: ? how do I arm', DIM))

    lines = [colour(f'\n  Relevant commands for: "{question}"', BOLD, CYAN)]
    for cmd in all_cmds[:8]:
        desc = ''
        for sec_cmds in _CMD_CATALOGUE.values():
            if cmd in sec_cmds:
                desc = sec_cmds[cmd]
                break
        lines.append(f"  {colour(cmd,'',BOLD):<40}  {desc}")

    lines.append(colour('\n  Type any command to send it. Tab completes.', DIM))
    return '\n'.join(lines)



# ── Command parser ─────────────────────────────────────────────────
# Full tab completion — all section keywords + all 217 GS commands
VALID_CMDS = (['status','aero','zones','freeze','restore','remap',
               'set','discovery','aerobatic','survey','abort',
               'clear','help','q','quit','exit'] +
              [f'help {s.lower().replace(" ","-")}'
               for s in _CMD_CATALOGUE] +
              list({c for s in _CMD_CATALOGUE.values()
                     for c in s if not c.startswith('_')}))

def parse_and_send(line, conn, state):
    """Parse a user command and send the appropriate JSON to the Pi."""
    parts = line.strip().split()
    if not parts:
        return

    cmd = parts[0].lower()

    if cmd in ('q', 'quit', 'exit'):
        return 'QUIT'

    elif cmd == 'clear':
        clear()
        return

    elif cmd == 'help':
        topic = ' '.join(parts[1:]) if len(parts) > 1 else None
        print(render_help(topic))
        return

    elif line.strip().startswith('?'):
        question = line.strip()[1:].strip()
        print(handle_natural_language(question))
        return

    elif cmd == 'status':
        conn.send({'cmd': 'status'})

    elif cmd == 'endurance_status':
        endurance_status()
        return

    elif cmd == 'aero':
        conn.send({'cmd': 'aero_cal_status'})

    elif cmd == 'zones':
        conn.send({'cmd': 'lm6_zone_report'})

    elif cmd == 'freeze':
        conn.send({'cmd': 'freeze_optimizers'})
        print(colour('  Optimizers frozen.', YELLOW))

    elif cmd == 'restore':
        conn.send({'cmd': 'restore_optimizers'})
        print(colour('  Optimizers restored.', GREEN))

    elif cmd == 'remap':
        zone = parts[1] if len(parts) > 1 else ''
        if not zone:
            print(colour('  Usage: remap <ZONE_NAME>', RED))
            return
        conn.send({'cmd': 'confirm_remap', 'zone': zone.upper()})
        print(colour(f'  Remap confirmed for {zone.upper()}', GREEN))

    elif cmd == 'set':
        if len(parts) < 3:
            print(colour('  Usage: set <KEY> <value>', RED))
            return
        key   = parts[1].upper()
        raw   = parts[2]
        # Auto-detect type
        if raw.lower() in ('true', 'false'):
            value = raw.lower() == 'true'
        else:
            try:
                value = int(raw) if '.' not in raw else float(raw)
            except ValueError:
                value = raw
        conn.send({'cmd': 'set', 'key': key, 'value': value})
        print(colour(f'  Setting {key} = {value}', CYAN))

    elif cmd == 'discovery':
        sub = parts[1].lower() if len(parts) > 1 else 'on'
        if sub == 'off':
            conn.send({'cmd': 'discovery_disable'})
            print(colour('  Discovery disabled.', DIM))
        else:
            h = float(parts[2]) if len(parts) > 2 else 30.0
            conn.send({'cmd': 'discovery_enable', 'h_agl_m': h})
            print(colour(f'  Discovery enabled at h={h}m', GREEN))

    elif cmd == 'aerobatic':
        mtype = parts[1] if len(parts) > 1 else 'combined'
        h     = float(parts[2]) if len(parts) > 2 else 50.0
        conn.send({'cmd': 'aerobatic_discovery_enable',
                   'manoeuvre_type': mtype, 'h_agl_m': h})
        print(colour(f'  Aerobatic discovery: {mtype} at h={h}m', MAGENTA))

    elif cmd == 'survey':
        h = float(parts[1]) if len(parts) > 1 else 50.0
        conn.send({'cmd': 'polar_survey_start', 'h_agl_m': h})
        print(colour(f'  Polar survey started at h={h}m', GREEN))

    elif cmd == 'abort':
        conn.send({'cmd': 'tc02_abort'})
        print(colour('  TC ABORT sent.', RED, BOLD))

    else:
        # Try sending directly to Pi — covers all 217 GS commands
        # that aren't mapped to local aliases above
        raw_cmd = parts[0]
        raw_args = parts[1:] if len(parts) > 1 else []
        # Build kwargs from key=value pairs
        kwargs = {}
        positional = []
        for arg in raw_args:
            if '=' in arg:
                k, v = arg.split('=', 1)
                try: v = float(v) if '.' in v else int(v)
                except ValueError: pass
                kwargs[k] = v
            else:
                positional.append(arg)
        if positional:
            kwargs['args'] = positional
        pkt = {'cmd': raw_cmd, **kwargs}
        conn.send(pkt)
        print(colour(f'  → {raw_cmd}', CYAN))


# ── Tab completion ─────────────────────────────────────────────────
readline.parse_and_bind('tab: complete')
readline.set_completer(
    lambda text, state: ([c for c in VALID_CMDS if c.startswith(text)]
                          + [None])[state])


# ── Display refresh loop ───────────────────────────────────────────
_REFRESH_HZ    = 2
_display_state = {'last_render': 0.0, 'dirty': True}

def display_loop(conn, state, stop_event):
    """Refresh the display panel every 0.5s in background."""
    while not stop_event.is_set():
        time.sleep(1.0 / _REFRESH_HZ)
        if not state.telem and not state.tc02_points:
            continue
        # Print status block above the prompt line
        # Use ANSI save/restore cursor to avoid clobbering input
        block = '\n'.join([
            '',
            render_header(conn.connected),
            render_telemetry(state),
            '',
            render_tc02(state),
            '',
            render_lm6(state),
            render_alerts(state),
            colour('─'*70, DIM),
            '',
        ])
        # ANSI: save cursor, move to top of terminal, print, restore
        sys.stdout.write('\033[s\033[H')
        sys.stdout.write(block)
        sys.stdout.write('\033[u')
        sys.stdout.flush()


# ── Main ───────────────────────────────────────────────────────────
def main():
    host = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_IP
    port = int(sys.argv[2]) if len(sys.argv) > 2 else DEFAULT_PORT

    print(colour(f'\n  MCO-1D Ground Station\n  Connecting to {host}:{port}...', CYAN))

    conn  = GSConnection(host, port)
    state = AppState()

    result = conn.connect()
    if result is not True:
        print(colour(f'\n  Could not connect to Pi at {host}:{port}', RED))
        print(colour('  Check: ethernet cable connected?', DIM))
        print(colour(f'  Check: Pi IP is {host}? (ping {host})', DIM))
        print(colour('  Check: gs_start() called on Pi?', DIM))
        sys.exit(1)

    print(colour('  Connected.\n', GREEN))

    # Wire inbound messages into state
    conn.on_msg = state.ingest

    # Start display refresh thread
    stop_ev = threading.Event()
    disp_th = threading.Thread(
        target=display_loop, args=(conn, state, stop_ev),
        daemon=True, name='GS-Display')
    disp_th.start()

    # Small scroll buffer so display thread doesn't overwrite commands
    print('\n' * 25)

    # Main command loop
    try:
        while True:
            try:
                prompt = colour('  mco> ', BOLD, CYAN)
                line   = input(prompt)
                r = parse_and_send(line, conn, state)
                if r == 'QUIT':
                    break
            except KeyboardInterrupt:
                print()
                break
            except EOFError:
                break
    finally:
        stop_ev.set()
        conn.close()
        print(colour('\n  Disconnected.\n', DIM))



# ── PITOT STATUS — confirms pitot is live before hover mode ──────────────────
def pitot_status():
    """
    Show pitot validity state.
    Hover mode cannot engage until pitot_valid = True.
    Blow on pitot tube — count should increment to ≥3.
    """
    try:
        import mco_wave_library as w
        valid   = w._pitot_valid
        count   = w._pitot_valid_count
        last_t  = w._pitot_last_t
        timeout = w.PITOT_VALID_TIMEOUT_S
        min_msgs= w.PITOT_VALID_MIN_MSGS
        import time
        age_s = (time.monotonic() - last_t) if last_t else None
        print(f"\n  PITOT STATUS")
        print(f"  Valid:         {valid}  ({'HOVER MODE ENABLED' if valid else 'HOVER MODE BLOCKED'})")
        print(f"  Message count: {count} / {min_msgs} required")
        print(f"  Last update:   {f'{age_s:.1f}s ago' if age_s else 'never'}")
        print(f"  Timeout:       {timeout}s")
        if not valid:
            print(f"\n  ACTION: Confirm Pixhawk is powered and MAVLink is running.")
            print(f"  Blow on pitot tube — count should reach {min_msgs}.")
            print(f"  Command 'pitot_force_valid' to override for bench testing only.")
    except Exception as e:
        print(f"  pitot_status error: {e}")

def pitot_force_valid():
    """Force pitot valid for bench testing WITHOUT pitot installed. BENCH ONLY."""
    try:
        import mco_wave_library as w
        w._pitot_valid       = True
        w._pitot_valid_count = w.PITOT_VALID_MIN_MSGS
        print("  PITOT FORCED VALID — for bench testing only. "
              "Do not fly with pitot disconnected.")
    except Exception as e:
        print(f"  pitot_force_valid error: {e}")

def pitot_force_invalid():
    """Reset pitot to invalid state."""
    try:
        import mco_wave_library as w
        w._pitot_valid       = False
        w._pitot_valid_count = 0
        print("  Pitot reset to INVALID.")
    except Exception as e:
        print(f"  pitot_force_invalid error: {e}")


# ── SECTOR ARC STATUS ─────────────────────────────────────────────────────────
def sector_arc_status():
    """
    Show sector arc symmetry results from TC-01 step 05b.
    Any wing with asymmetry >2° must have its motor drum re-seated
    before frequencies above 0.5 Hz are run.
    """
    try:
        from mco_test_cards import _tc
        result = _tc['tc01']['results'].get('05b-sector-arc', {})
        data   = result.get('data', {})
        passed = result.get('passed', False)
        print(f"\n  SECTOR ARC SYMMETRY (TC-01 step 05b)")
        print(f"  Status: {'PASS' if passed else 'NOT YET RUN or FAIL'}")
        wings = data.get('wings', [])
        for w in wings:
            note = w.get('note', '')
            err  = w.get('error', '')
            idx  = w.get('wing', '?')
            print(f"  Wing {idx}: {note or err or 'checked'}")
        if not passed:
            print(f"\n  Run TC-01 step 05b before any frequency above 0.5 Hz.")
    except Exception as e:
        print(f"  sector_arc_status error: {e}")


# ── FLUTTER STATUS — monitors stopped-wing IMU signature ─────────────────────
def flutter_status():
    """
    Show current IMU vibration RMS and flutter risk level.
    During TC-03 with stopped wings, a step-increase in RMS = flutter precursor.
    """
    try:
        import mco_wave_library as w
        latest = getattr(w, '_latest_result', {})
        imu_rms = latest.get('vibration_rms_g', None)
        baseline = latest.get('vibration_baseline_g', None)
        V_ms    = latest.get('V_ms', 0.0)
        pairs_stopped = latest.get('pairs_stopped', 0)
        print(f"\n  FLUTTER STATUS")
        print(f"  IMU vibration RMS: {f'{imu_rms:.4f} g' if imu_rms else 'no data'}")
        print(f"  Baseline:          {f'{baseline:.4f} g' if baseline else 'not set'}")
        if imu_rms and baseline:
            ratio = imu_rms / baseline if baseline > 0 else 0
            risk  = ('HIGH — ABORT SPEED STEP' if ratio > 2.0 else
                     'ELEVATED — monitor closely' if ratio > 1.3 else
                     'NORMAL')
            print(f"  RMS / baseline:    {ratio:.2f}×  → {risk}")
        print(f"  Pitot airspeed:    {V_ms:.1f} m/s")
        print(f"  Pairs stopped:     {pairs_stopped}")
        print(f"\n  During TC-03: abort if RMS rises >30% suddenly at any speed step.")
    except Exception as e:
        print(f"  flutter_status error: {e}")


# ── Register new commands ─────────────────────────────────────────────────────
_GS_COMMANDS = {k: v for k, v in globals().items() if callable(v) and
                not k.startswith('_') and k not in ('print','open','sorted')}


# ── SOFT-STOP STATUS ─────────────────────────────────────────────────────────
def soft_stop_status():
    """Show per-wing stroke soft-stop taper state."""
    try:
        from mco_wave_library import soft_stop_status as _sss
        _sss()
    except Exception as e:
        print(f"  soft_stop_status error: {e}")

def soft_stop_enable():
    """Enable stroke soft-stop system."""
    try:
        from mco_wave_library import soft_stop_enable as _sse
        _sse()
    except Exception as e:
        print(f"  soft_stop_enable error: {e}")

def soft_stop_disable():
    """Disable soft-stop — diagnostic only. Monitor rope drives closely."""
    try:
        from mco_wave_library import soft_stop_disable as _ssd
        _ssd()
    except Exception as e:
        print(f"  soft_stop_disable error: {e}")

def soft_stop_set_zone(zone_deg=10.0):
    """Set soft-stop zone depth in degrees (5-20° recommended)."""
    try:
        from mco_wave_library import soft_stop_set_zone as _ssz
        _ssz(zone_deg)
    except Exception as e:
        print(f"  soft_stop_set_zone error: {e}")

def soft_stop_set_min_vel(frac=0.35):
    """Set minimum velocity at reversal as fraction of full speed (0.1-0.6)."""
    try:
        from mco_wave_library import soft_stop_set_min_vel as _ssv
        _ssv(frac)
    except Exception as e:
        print(f"  soft_stop_set_min_vel error: {e}")


# ── CG MANAGEMENT COMMANDS ────────────────────────────────────────────────────
def cg_status():
    """Show complete CG management state — estimates, margins, cal status."""
    try:
        from mco_wave_library import cg_status as _cgs
        _cgs()
    except Exception as e:
        print(f"  cg_status error: {e}")

def cg_calibrate_now():
    """Manually trigger 90-second CG calibration pass. Fly level cruise."""
    try:
        from mco_wave_library import cg_calibrate_now as _ccn
        _ccn()
    except Exception as e:
        print(f"  cg_calibrate_now error: {e}")

def cg_reset():
    """Reset CG estimates and confidence after payload change."""
    try:
        from mco_wave_library import cg_reset as _cr
        _cr()
    except Exception as e:
        print(f"  cg_reset error: {e}")

def cg_set_margin(intent='cruise', mm=20.0):
    """Override stability margin for intent. E.g. cg_set_margin dart -5"""
    try:
        from mco_wave_library import cg_set_margin as _csm
        _csm(intent, mm)
    except Exception as e:
        print(f"  cg_set_margin error: {e}")

def airborne_status():
    """Show airborne detection confidence and signal breakdown."""
    try:
        from mco_wave_library import airborne_status as _abs
        _abs()
    except Exception as e:
        print(f"  airborne_status error: {e}")


# ── CONTROLLED LANDING / DAMAGE TOLERANCE ────────────────────────────────────
def controlled_landing_status():
    """Show complete damage tolerance state, pilot authority, and descent table."""
    try:
        from mco_wave_library import controlled_landing_status as _cls
        _cls()
    except Exception as e:
        print(f"  controlled_landing_status error: {e}")

def fault_wing_restore(wing_idx=0):
    """Restore wing to healthy state after repair. Example: fault_wing_restore 0"""
    try:
        from mco_wave_library import fault_wing_restore as _fwr
        _fwr(int(wing_idx))
    except Exception as e:
        print(f"  fault_wing_restore error: {e}")

def failure_briefing(wing_indices=None):
    """
    Show pre-flight or in-flight failure performance table.
    No args: show all combinations.
    With wing indices: failure_briefing 0 1  (W1p and W1s)
    """
    try:
        from mco_wave_library import failure_briefing as _fb
        if wing_indices:
            idx = [int(x) for x in str(wing_indices).split()]
            _fb(idx)
        else:
            _fb()
    except Exception as e:
        print(f"  failure_briefing error: {e}")


# ── CERTIFICATION TOOLS (Quarantined in firmware/archive/legacy_modules) ────
def cert_status():
    """Certification tools have been archived (unverified mock stubs)."""
    print("  Certification stubs archived under firmware/archive/legacy_modules/mco_certification_tools.py")


# ── ATTITUDE HOLD (ELECTRONIC DIHEDRAL) ──────────────────────────────────────
def attitude_hold_status():
    """Show attitude hold state, gains, and engage status."""
    try:
        from mco_wave_library import attitude_hold_status as _ahs
        _ahs()
    except Exception as e:
        print(f"  attitude_hold_status error: {e}")

def attitude_hold_enable():
    """Enable attitude hold (default on at ARM)."""
    try:
        from mco_wave_library import attitude_hold_enable as _ahe
        _ahe()
    except Exception as e:
        print(f"  attitude_hold_enable error: {e}")

def attitude_hold_disable():
    """Disable attitude hold — requires constant pilot input for stability."""
    try:
        from mco_wave_library import attitude_hold_disable as _ahd
        _ahd()
    except Exception as e:
        print(f"  attitude_hold_disable error: {e}")

def attitude_hold_reset():
    """Reset attitude hold integrators and targets (call after attitude change)."""
    try:
        from mco_wave_library import attitude_hold_reset as _ahr
        _ahr()
    except Exception as e:
        print(f"  attitude_hold_reset error: {e}")

def attitude_hold_tune(param='kp_outer', value=0.08):
    """
    Adjust an attitude hold gain in flight.
    Examples:
      attitude_hold_tune kp_outer 0.08
      attitude_hold_tune roll_ki_inner 0.02
      attitude_hold_tune authority_stick 0.15
    """
    try:
        from mco_wave_library import attitude_hold_tune as _aht
        _aht(param, value)
    except Exception as e:
        print(f"  attitude_hold_tune error: {e}")

def attitude_hold_set_intent_gain(intent='dart', scale=0.30):
    """Set gain scale for one flight intent. E.g. attitude_hold_set_intent_gain dart 0.25"""
    try:
        from mco_wave_library import attitude_hold_set_intent_gain as _ahg
        _ahg(intent, scale)
    except Exception as e:
        print(f"  attitude_hold_set_intent_gain error: {e}")


# ── ADVANCED FLIGHT CONTROL MODES ────────────────────────────────────────────
def advanced_modes_status():
    """Show status of all 5 advanced control modes and optimizer hooks."""
    try:
        from mco_wave_library import advanced_modes_status as _ams
        _ams()
    except Exception as e:
        print(f"  error: {e}")

def position_hold_enable():
    """Enable GPS position hold (loiter)."""
    try:
        from mco_wave_library import position_hold_enable as _phe
        _phe()
    except Exception as e:
        print(f"  error: {e}")

def position_hold_disable():
    """Disable GPS position hold."""
    try:
        from mco_wave_library import position_hold_disable as _phd
        _phd()
    except Exception as e:
        print(f"  error: {e}")

def position_hold_engage():
    """Manually capture current GPS position as hold target."""
    try:
        from mco_wave_library import position_hold_engage as _pheng
        _pheng()
    except Exception as e:
        print(f"  error: {e}")

def position_hold_release():
    """Release position hold."""
    try:
        from mco_wave_library import position_hold_release as _phr
        _phr()
    except Exception as e:
        print(f"  error: {e}")

def crew_hover_enable():
    """Enable velocity-referenced hover (SAR crew hover mode)."""
    try:
        from mco_wave_library import crew_hover_enable as _che
        _che()
    except Exception as e:
        print(f"  error: {e}")

def crew_hover_disable():
    """Disable crew hover — return to attitude-referenced hover."""
    try:
        from mco_wave_library import crew_hover_disable as _chd
        _chd()
    except Exception as e:
        print(f"  error: {e}")

def turn_coord_enable():
    """Enable automatic turn coordination."""
    try:
        from mco_wave_library import turn_coord_enable as _tce
        _tce()
    except Exception as e:
        print(f"  error: {e}")

def turn_coord_disable():
    """Disable turn coordination — manual pedal control only."""
    try:
        from mco_wave_library import turn_coord_disable as _tcd
        _tcd()
    except Exception as e:
        print(f"  error: {e}")

def corridor_guard_enable():
    """Enable conversion corridor guard (V-22 style envelope protection)."""
    try:
        from mco_wave_library import corridor_guard_enable as _cge
        _cge()
    except Exception as e:
        print(f"  error: {e}")

def corridor_guard_disable():
    """Disable corridor guard."""
    try:
        from mco_wave_library import corridor_guard_disable as _cgd
        _cgd()
    except Exception as e:
        print(f"  error: {e}")

def airspeed_hold_enable():
    """Enable airspeed hold mode."""
    try:
        from mco_wave_library import airspeed_hold_enable as _ashe
        _ashe()
    except Exception as e:
        print(f"  error: {e}")

def airspeed_hold_disable():
    """Disable airspeed hold."""
    try:
        from mco_wave_library import airspeed_hold_disable as _ashd
        _ashd()
    except Exception as e:
        print(f"  error: {e}")


# ── SURFACE CLASSIFIER ────────────────────────────────────────────────────────
def surface_status():
    """Show surface classification (water vs land) state."""
    try:
        from mco_wave_library import surface_status as _ss; _ss()
    except Exception as e: print(f"  error: {e}")

def surface_set_water():
    """Force water classification — enables surface effect mode."""
    try:
        from mco_wave_library import surface_set_water as _sw; _sw()
    except Exception as e: print(f"  error: {e}")

def surface_set_land():
    """Force land classification — safety default."""
    try:
        from mco_wave_library import surface_set_land as _sl; _sl()
    except Exception as e: print(f"  error: {e}")

def surface_auto():
    """Return to automatic surface classification."""
    try:
        from mco_wave_library import surface_auto as _sa; _sa()
    except Exception as e: print(f"  error: {e}")

def surface_set_land_box(lat_min=49, lon_min=-8, lat_max=61, lon_max=2):
    """Set land bounding box for GPS classifier. surface_set_land_box 49 -8 61 2"""
    try:
        from mco_wave_library import surface_set_land_box as _sb
        _sb(lat_min, lon_min, lat_max, lon_max)
    except Exception as e: print(f"  error: {e}")

# ── FLAP-GLIDE MODE ───────────────────────────────────────────────────────────
def flap_glide_enable(variant='H-FG-1'):
    """Enable flap-glide mode. variant: H-FG-1, H-FG-2, H-FG-3"""
    try:
        from mco_wave_library import flap_glide_enable as _fge; _fge(str(variant))
    except Exception as e: print(f"  error: {e}")

def flap_glide_disable():
    """Disable flap-glide — return to continuous flapping."""
    try:
        from mco_wave_library import flap_glide_disable as _fgd; _fgd()
    except Exception as e: print(f"  error: {e}")

def flap_glide_status():
    """Show flap-glide mode state and cycle metrics."""
    try:
        from mco_wave_library import flap_glide_status as _fgs; _fgs()
    except Exception as e: print(f"  error: {e}")


# ── WIG SURFACE FLOOR ─────────────────────────────────────────────────────────
def wig_floor_status():
    """Show WIG surface floor state and speed-floor curve."""
    try:
        from mco_wave_library import wig_floor_status as _wfs; _wfs()
    except Exception as e: print(f"  error: {e}")

def wig_floor_enable():
    """Enable WIG surface floor protection (default ON)."""
    try:
        from mco_wave_library import wig_floor_enable as _wfe; _wfe()
    except Exception as e: print(f"  error: {e}")

def wig_floor_disable():
    """Disable WIG floor — use for deliberate surface approach or ditching."""
    try:
        from mco_wave_library import wig_floor_disable as _wfd; _wfd()
    except Exception as e: print(f"  error: {e}")


# ── MISSION FOCUS ─────────────────────────────────────────────────────────────
def focus(name_or_phrase=''):
    """
    Set mission focus. focus / focus <name> / focus clear
    Built-in: gust_tolerance, position_hold, smooth_flight,
              crosswind_landing, endurance, sprint, sar_hover
    Any plain English phrase is matched automatically.
    """
    try:
        from mco_wave_library import focus as _f; _f(str(name_or_phrase))
    except Exception as e: print(f"  error: {e}")

def focus_status():
    """Show current focus state and scoring progress."""
    try:
        from mco_wave_library import focus_status as _fs; _fs()
    except Exception as e: print(f"  error: {e}")

def focus_clear():
    """Remove current focus — return to Ps scoring."""
    try:
        from mco_wave_library import focus_clear as _fc; _fc()
    except Exception as e: print(f"  error: {e}")

def focus_define(name='', description='', metric='imu_rms_g',
                  direction='minimize', persist=False):
    """
    Define a custom focus on any sensor field.
    focus_define name=quiet metric=imu_rms_g direction=minimize
    """
    try:
        from mco_wave_library import focus_define as _fd
        _fd(str(name), str(description), str(metric), str(direction),
            bool(persist))
    except Exception as e: print(f"  error: {e}")

def endurance_status():
    """Show honest endurance power accounting and suite status."""
    try:
        from mco_wave_library import endurance_status as _es; _es()
    except Exception as e: print(f"  error: {e}")

if __name__ == '__main__':
    main()


def display_sensing(telem: dict) -> str:
    """Display per-wing 3-DOF sensing panel."""
    lines = ["", "  ─── PER-WING SENSING ───────────────────────────────"]

    # Actual AoA (MCP3208)
    aoa = telem.get('actual_aoa_deg') or []
    eff = telem.get('effective_aoa_deg') or []
    if aoa:
        names = ['W1p','W1s','W2p','W2s','W3p','W3s','W4p','W4s']
        lines.append("  AoA (geom/eff):")
        for i in range(min(8, len(aoa))):
            g = f"{aoa[i]:.1f}°" if aoa[i] is not None else " --- "
            e = f"{eff[i]:.1f}°" if i < len(eff) and eff[i] is not None else " --- "
            if i % 2 == 0:
                lines.append(f"    {names[i]} {g}/{e}", )
            else:
                lines[-1] += f"   {names[i]} {g}/{e}"

    # Strouhal
    st = telem.get('strouhal_actual') or []
    if st:
        st_str = "  ".join(f"W{i+1}:{v:.3f}" for i,v in enumerate(st) if v)
        lines.append(f"  Strouhal: {st_str}  (target 0.20-0.40)")

    # Force waveform
    fw = telem.get('force_waveform') or {}
    if fw:
        lines.append(f"  Force: ↑{fw.get('F_total_vertical',0):.1f}N  "
                     f"→{fw.get('F_total_horizontal',0):.1f}N  "
                     f"smooth={fw.get('smoothness',0):.3f}")

    # Trim balance
    tb = telem.get('trim_balance') or {}
    if tb:
        asym = tb.get('asymmetry', 0) * 100
        bar = '█' * int(asym / 2) if asym < 50 else '!'*10
        lines.append(f"  Trim: port={tb.get('port_lift',0):.1f}N  "
                     f"stbd={tb.get('stbd_lift',0):.1f}N  "
                     f"asym={asym:.1f}% {bar}")

    # Stall detection
    sd = telem.get('stall_detection') or {}
    if sd:
        likelihoods = sd.get('pair_likelihood', [0.0]*4)
        stall_str = "  ".join(
            f"W{i+1}:{l:.2f}{'⚠' if l > 0.45 else ''}"
            for i, l in enumerate(likelihoods))
        lines.append(f"  Stall: {stall_str}")

    # Servo health
    rms = telem.get('servo_health_rms') or []
    if rms:
        worst = max((r for r in rms if r is not None), default=0)
        if worst > 0.5:
            names = ['W1p','W1s','W2p','W2s','W3p','W3s','W4p','W4s']
            rms_str = "  ".join(f"{n}:{v:.1f}°" for n,v in zip(names,rms)
                                 if v is not None and v > 0.5)
            lines.append(f"  Servo RMS (>0.5°): {rms_str}")

    # Mechanical power / rope health
    mp = telem.get('mechanical_power') or {}
    if mp and mp.get('any_elevated'):
        health = mp.get('rope_health', [1.0]*8)
        poor = [(i, h) for i,h in enumerate(health) if h < 0.8]
        if poor:
            names = ['W1p','W1s','W2p','W2s','W3p','W3s','W4p','W4s']
            rope_str = "  ".join(f"{names[i]}:{h:.2f}" for i,h in poor)
            lines.append(f"  ⚠ ROPE LOAD: {rope_str}")

    return "\n".join(lines)
