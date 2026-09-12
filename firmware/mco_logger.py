#!/usr/bin/env python3
"""
MCO-1D Flight Data Logger
==========================
Writes timestamped flight data to CSV at 50 Hz.
Runs as a thread alongside the main flight loop.

Each flight session creates a new numbered log file:
    /home/pi/logs/mco_flight_001.csv
    /home/pi/logs/mco_flight_002.csv  etc.

Logged fields (50 Hz):
    timestamp, ground_state, h_agl_m, V_ms, pitch_deg, roll_deg,
    heading_deg, sink_rate_ms, freq_hz, lever_pct, Ps_measured,
    bus_voltage_v, motor_I_A, turb_intensity, mode_id,
    w1_amp, w2_amp, w3_amp, w4_amp  (per-pair mean amplitude)

Usage:
    from mco_logger import MCOLogger
    logger = MCOLogger()
    logger.start()
    ...
    logger.log(sensors_dict, result_dict)   # call every tick
    logger.stop()                            # call at shutdown

Log files are stored on the HIGH ENDURANCE MicroSD at /home/pi/logs/
Never log to /tmp or tmpfs — data would be lost on power cycle.
"""

import os
import csv
import time
import threading
import queue
import glob


LOG_DIR      = '/home/pi/logs'
MAX_LOG_FILES= 50     # keep last 50 sessions (each ~35 MB at 50Hz/1hr)
QUEUE_MAXSIZE= 5000   # buffer ~100 seconds of data if disk is slow


class MCOLogger:
    """
    Non-blocking flight logger.
    Tick thread puts records into a queue.
    Logger thread writes queue to CSV without blocking the tick.
    """

    FIELDS = [
        'timestamp_s', 'elapsed_s', 'ground_state',
        'h_agl_m', 'V_ms', 'pitch_deg', 'roll_deg', 'heading_deg',
        'sink_rate_ms', 'accel_z_g', 'imu_rms_g',
        'freq_hz', 'lever_pct', 'Ps_measured',
        'bus_voltage_v', 'motor_I_A',
        'pitch_stick', 'roll_stick', 'yaw_stick',
        'turb_intensity', 'mode_id', 'ground_ops_mode',
        'w1_port_amp', 'w1_stbd_amp', 'w2_port_amp', 'w2_stbd_amp',
        'w3_port_amp', 'w3_stbd_amp', 'w4_port_amp', 'w4_stbd_amp',
        'discovery_active', 'n_disc_promoted',
        'l9_active', 'exploration_layer',
        # Per-wing AoA sensing (MCP3208 #1)
        'aoa_w1p', 'aoa_w1s', 'aoa_w2p', 'aoa_w2s',
        'aoa_w3p', 'aoa_w3s', 'aoa_w4p', 'aoa_w4s',
        # Effective AoA (3-DOF computed)
        'eff_aoa_w1p', 'eff_aoa_w1s', 'eff_aoa_w2p', 'eff_aoa_w2s',
        'eff_aoa_w3p', 'eff_aoa_w3s', 'eff_aoa_w4p', 'eff_aoa_w4s',
        # ODrive stroke position (radians, motor shaft)
        'pos_w1p', 'pos_w1s', 'pos_w2p', 'pos_w2s',
        'pos_w3p', 'pos_w3s', 'pos_w4p', 'pos_w4s',
        # Motor current per wing
        'i_w1p', 'i_w1s', 'i_w2p', 'i_w2s',
        'i_w3p', 'i_w3s', 'i_w4p', 'i_w4s',
        # Aero state
        'strouhal_w1', 'strouhal_w2', 'strouhal_w3', 'strouhal_w4',
        'F_vert_total', 'F_horiz_total', 'force_smoothness',
        # Trim and stall
        'port_lift_N', 'stbd_lift_N', 'trim_asymm_pct',
        'stall_w1', 'stall_w2', 'stall_w3', 'stall_w4',
        # Servo health (AoA tracking RMS per wing)
        'srv_rms_w1p', 'srv_rms_w1s', 'srv_rms_w2p', 'srv_rms_w2s',
        'srv_rms_w3p', 'srv_rms_w3s', 'srv_rms_w4p', 'srv_rms_w4s',
        # Soft-stop taper factors (per wing — for rope life and cert analysis)
        'ss_w1p', 'ss_w1s', 'ss_w2p', 'ss_w2s',
        'ss_w3p', 'ss_w3s', 'ss_w4p', 'ss_w4s',
        # Certification / structural
        'degraded_mode', 'n_active_wings',
        'cg_long_mm', 'cg_lat_mm', 'stability_margin_mm',
        'airborne', 'roll_rate_dps', 'pitch_rate_dps',
        # Attitude hold
        'ah_roll_engaged', 'ah_pitch_engaged',
        'ah_roll_bias_deg', 'ah_pitch_bias_deg', 'ah_roll_target',
        # Advanced modes
        'ph_engaged', 'ph_pos_err_m',
        'vh_active', 'tc_active', 'tc_sideslip_g',
        'ash_engaged', 'ash_speed_err', 'cc_margin',
    ]

    def __init__(self):
        self._queue   = queue.Queue(maxsize=QUEUE_MAXSIZE)
        self._alive   = False
        self._thread  = None
        self._file    = None
        self._writer  = None
        self._start_t = None
        self.filepath  = None
        self.n_records = 0
        self.n_dropped = 0

    def _next_filepath(self):
        os.makedirs(LOG_DIR, exist_ok=True)
        existing = sorted(glob.glob(f'{LOG_DIR}/mco_flight_*.csv'))
        if existing:
            last = existing[-1]
            try:
                n = int(last.split('_')[-1].split('.')[0]) + 1
            except ValueError:
                n = len(existing) + 1
        else:
            n = 1
        return f'{LOG_DIR}/mco_flight_{n:03d}.csv'

    def _prune_old(self):
        files = sorted(glob.glob(f'{LOG_DIR}/mco_flight_*.csv'))
        while len(files) > MAX_LOG_FILES:
            os.remove(files[0])
            files = files[1:]

    def start(self):
        """Open log file and start writer thread."""
        self._prune_old()
        self.filepath  = self._next_filepath()
        self._start_t  = time.monotonic()
        self._alive    = True

        self._file     = open(self.filepath, 'w', newline='', buffering=8192)
        self._writer   = csv.DictWriter(self._file, fieldnames=self.FIELDS)
        self._writer.writeheader()

        self._thread   = threading.Thread(target=self._run, daemon=True,
                                           name='mco-logger')
        self._thread.start()
        print(f"Logger started: {self.filepath}")

    def stop(self):
        """Flush and close log file."""
        self._alive = False
        if self._thread:
            self._thread.join(timeout=5.0)
        if self._file:
            self._file.flush()
            self._file.close()
            self._file = None
        print(f"Logger stopped. {self.n_records:,} records, "
              f"{self.n_dropped} dropped. File: {self.filepath}")

    def log(self, sensors: dict, result: dict):
        """
        Queue a record for logging. Non-blocking — never delays the tick.
        If queue is full, the record is dropped (counted in n_dropped).
        """
        now = time.monotonic()
        wc  = result.get('wing_commands', [{}]*8)

        row = {
            'timestamp_s':      round(now, 4),
            'elapsed_s':        round(now - self._start_t, 3),
            'ground_state':     result.get('ground_state', ''),
            'h_agl_m':          sensors.get('h_agl_m', 0),
            'V_ms':             sensors.get('V_ms', 0),
            'pitch_deg':        sensors.get('pitch_deg_imu', 0),
            'roll_deg':         sensors.get('roll_deg_imu', 0),
            'heading_deg':      sensors.get('heading_deg', 0),
            'sink_rate_ms':     sensors.get('sink_rate_ms', 0),
            'accel_z_g':        sensors.get('accel_z_g', 1),
            'imu_rms_g':        sensors.get('imu_rms_g', 0),
            'freq_hz':          result.get('freq_hz', 0),
            'lever_pct':        sensors.get('lever_pct', 50),
            'Ps_measured':      sensors.get('Ps_measured', 0),
            'bus_voltage_v':    sensors.get('bus_voltage_v', 0),
            'motor_I_A':        sensors.get('motor_I_A', 0),
            'pitch_stick':      sensors.get('pitch_stick', 0),
            'roll_stick':       sensors.get('roll_stick', 0),
            'yaw_stick':        sensors.get('yaw_stick', 0),
            'turb_intensity':   sensors.get('turb_intensity', 0),
            'mode_id':          result.get('mode_hints', {}).get('intent', ''),
            'ground_ops_mode':  sensors.get('ground_ops_mode', ''),
            'w1_port_amp':      wc[0].get('flap_amp_deg', 0) if len(wc)>0 else 0,
            'w1_stbd_amp':      wc[1].get('flap_amp_deg', 0) if len(wc)>1 else 0,
            'w2_port_amp':      wc[2].get('flap_amp_deg', 0) if len(wc)>2 else 0,
            'w2_stbd_amp':      wc[3].get('flap_amp_deg', 0) if len(wc)>3 else 0,
            'w3_port_amp':      wc[4].get('flap_amp_deg', 0) if len(wc)>4 else 0,
            'w3_stbd_amp':      wc[5].get('flap_amp_deg', 0) if len(wc)>5 else 0,
            'w4_port_amp':      wc[6].get('flap_amp_deg', 0) if len(wc)>6 else 0,
            'w4_stbd_amp':      wc[7].get('flap_amp_deg', 0) if len(wc)>7 else 0,
            'discovery_active': int(result.get('auto_disc', {}).get('active', False)),
            'n_disc_promoted':  result.get('auto_disc', {}).get('n_promoted', 0),
            'l9_active':        int(result.get('l9', {}).get('active', False)),
            'exploration_layer':result.get('exploration', {}).get('layer', 1),
            # ── Per-wing sensing (new 3-DOF fields) ─────────────────────
            **{f'aoa_w{i//2+1}{"ps"[i%2]}':   (result.get('actual_aoa_deg') or [None]*8)[i] for i in range(8)},
            **{f'eff_aoa_w{i//2+1}{"ps"[i%2]}': (result.get('effective_aoa_deg') or [None]*8)[i] for i in range(8)},
            **{f'pos_w{i//2+1}{"ps"[i%2]}':  (sensors.get('motor_pos_rad') or [None]*8)[i] for i in range(8)},
            **{f'i_w{i//2+1}{"ps"[i%2]}':    (sensors.get('motor_currents_a') or [0.0]*8)[i] for i in range(8)},
            **{f'strouhal_w{i+1}': (result.get('strouhal_actual') or [0.0]*4)[i] for i in range(4)},
            'F_vert_total':   (result.get('force_waveform') or {}).get('F_total_vertical', 0.0),
            'F_horiz_total':  (result.get('force_waveform') or {}).get('F_total_horizontal', 0.0),
            'force_smoothness':(result.get('force_waveform') or {}).get('smoothness', 1.0),
            'port_lift_N':   (result.get('trim_balance') or {}).get('port_lift', 0.0),
            'stbd_lift_N':   (result.get('trim_balance') or {}).get('stbd_lift', 0.0),
            'trim_asymm_pct':(result.get('trim_balance') or {}).get('asymmetry', 0.0),
            **{f'stall_w{i+1}': (result.get('stall_detection') or {}).get('pair_likelihood', [0.0]*4)[i] for i in range(4)},
            **{f'srv_rms_w{i//2+1}{"ps"[i%2]}': (result.get('servo_aoa_tracking_rms') or [0.0]*8)[i] for i in range(8)},
            # Soft-stop factors per wing
            **{f'ss_w{i//2+1}{"ps"[i%2]}': (result.get('wing_commands') or [{}]*8)[i].get('soft_stop_factor', 1.0) for i in range(8)},
            # Certification and structural
            'degraded_mode':      result.get('degraded_mode', 'normal'),
            'n_active_wings':     result.get('n_active_wings', 8),
            'cg_long_mm':         result.get('cg_delta_long_mm', 0.0),
            'cg_lat_mm':          result.get('cg_delta_lat_mm', 0.0),
            'stability_margin_mm':result.get('stability_margin_mm', 20.0),
            'airborne':           int(result.get('airborne', False)),
            'roll_rate_dps':      sensors.get('roll_rate_dps', 0.0),
            'pitch_rate_dps':     sensors.get('pitch_rate_dps', 0.0),
            # Attitude hold fields
            'ah_roll_engaged':    int(result.get('ah_roll_engaged', False)),
            'ah_pitch_engaged':   int(result.get('ah_pitch_engaged', False)),
            'ah_roll_bias_deg':   result.get('ah_roll_bias_deg', 0.0),
            'ah_pitch_bias_deg':  result.get('ah_pitch_bias_deg', 0.0),
            'ah_roll_target':     result.get('ah_roll_target', 0.0),
            # Advanced modes
            'ph_engaged':        int(result.get('ph_engaged', False)),
            'ph_pos_err_m':      result.get('ph_pos_err_m', 0.0),
            'vh_active':         int(result.get('vh_active', False)),
            'tc_active':         int(result.get('tc_active', False)),
            'tc_sideslip_g':     result.get('tc_sideslip_g', 0.0),
            'ash_engaged':       int(result.get('ash_engaged', False)),
            'ash_speed_err':     result.get('ash_speed_err', 0.0),
            'cc_margin':         result.get('cc_margin', 1.0),
            # Dart control
            'dart_active':       int(result.get('dart_active', False)),
            'dart_authority':    result.get('dart_authority', 0.0),
            'dart_urgency':      result.get('dart_urgency', 0.0),
            # Surface classifier + flap-glide
            'surface_type':      result.get('surface_type', 'unknown'),
            'surface_conf':      result.get('surface_conf', 0.0),
            'fg_active':         int(result.get('fg_active', False)),
            'fg_phase':          result.get('fg_phase', 'flap'),
            'fg_glide_ratio':    result.get('fg_glide_ratio', 0.0),
            # WIG floor
            'wf_active':         int(result.get('wf_active', False)),
            'wf_floor_m':        result.get('wf_floor_m', 0.0),
            'wf_margin_m':       result.get('wf_margin_m', 0.0),
            'wf_aoa_boost':      result.get('wf_aoa_boost', 0.0),
        }

        try:
            self._queue.put_nowait(row)
        except queue.Full:
            self.n_dropped += 1

    def _run(self):
        """Writer thread — drains queue to CSV."""
        while self._alive or not self._queue.empty():
            try:
                row = self._queue.get(timeout=0.05)
                self._writer.writerow(row)
                self.n_records += 1
                # Flush every 100 records (~2 seconds at 50Hz)
                if self.n_records % 100 == 0:
                    self._file.flush()
            except queue.Empty:
                continue
            except Exception as e:
                print(f"Logger write error: {e}")
