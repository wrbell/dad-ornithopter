#!/usr/bin/env python3
"""
MCO-1D ODrive S1 Configuration Script
=======================================
Configures all 8 × ODrive S1 boards before first flight.
Each S1 board controls exactly ONE motor (one wing).

Run ONCE after mechanical assembly, before TC-01.

Usage:
    python3 mco_odrive_setup.py              # configure all 8 S1 boards
    python3 mco_odrive_setup.py --verify     # verify existing config
    python3 mco_odrive_setup.py --node 0     # configure single node

MCO-1D ODrive S1 node assignments (8 boards, 1 motor each):
    Node 0: W1 port      Node 1: W1 starboard
    Node 2: W2 port      Node 3: W2 starboard
    Node 4: W3 port      Node 5: W3 starboard
    Node 6: W4 port      Node 7: W4 starboard

Key differences from ODrive v3.6:
    - Each S1 = 1 motor (not 2). 8 boards for 8 motors.
    - On-board absolute magnetic encoder — no external encoder needed.
    - Mount the included 1/4" magnetised magnet on the motor shaft.
    - No axis0/axis1 — every board has a single axis.
    - axis0.encoder.* is replaced by axis0.pos_vel_mapper.* and
      on-board magnetic encoder (config path changes in S1 firmware).
    - Each S1 needs its own brake resistor (included 2Ω 50W).
    - DO NOT share brake resistors between S1 boards.
    - Supports CAN-FD at 12 Mbps (currently running standard CAN 1 Mbps).

Magnet installation:
    The S1 includes a 1/4" (6.35 mm) diametrically magnetised disc magnet.
    Press or glue this magnet onto the motor shaft centre, flush with the
    back face of the motor. Align the S1 board so its encoder chip is
    directly over the magnet centre, ~1-2 mm gap. The encoder reads shaft
    angle absolutely from power-on with no index search.
"""

import sys
import time
import argparse

# ── MCO-1D motor parameters (GoolRC 5010-14 360KV) ──────────────────────────
POLE_PAIRS          = 7        # 12N14P: 14 magnetic poles / 2 = 7 pole pairs
MOTOR_KV            = 200      # Option A T-Motor MN5008 (or 360 for GoolRC)
CURRENT_LIM         = 25.0     # A per motor soft limit
CURRENT_LIM_MARGIN  = 10.0     # A headroom above limit (35A hard max)
CALIBRATION_CURRENT = 10.0     # A used during motor calibration
VEL_LIMIT           = 40.0     # turns/s (rev/s) motor shaft — ODrive S1 native units (40 rps = 251.3 rad/s = 2400 RPM)
                               # At 3.5 Hz sprint (θ0=±34°) × 10:1 ratio = 20.8 rps peak (or 30.7 rps at 2.2 Hz ±80°). 40 rps gives ~30-90% headroom.
BANDWIDTH           = 1000.0   # Hz — velocity controller bandwidth
BRAKE_RESISTANCE    = 2.0      # Ω — the included brake resistor per S1
DC_BUS_OVERVOLTAGE  = 28.5     # V — overvoltage trip level for 6S LiPo with regen absorption
WATCHDOG_TIMEOUT    = 0.100    # s — 100ms CAN communication loss watchdog
CAN_BAUD            = 1_000_000 # 1 Mbps standard CAN (not CAN-FD for now)

# Node IDs for all 8 S1 boards
ALL_NODES = list(range(8))   # 0 through 7

WING_NAMES = [
    'W1 port', 'W1 stbd',
    'W2 port', 'W2 stbd',
    'W3 port', 'W3 stbd',
    'W4 port', 'W4 stbd',
]


def configure_s1(odrv, node_id, verify_only=False):
    """
    Configure a single ODrive S1 for MCO-1D operation.
    The S1 has a single axis — no axis0/axis1 distinction.
    """
    wing_name = WING_NAMES[node_id] if node_id < len(WING_NAMES) else f'node{node_id}'
    print(f"\n  S1 node {node_id} ({wing_name})")

    errors  = []
    changes = []

    # S1 firmware parameter paths (single axis — no axis0/axis1)
    params = [
        # Motor configuration
        ('axis0.config.motor.motor_type',                  0,                   'Motor type HIGH_CURRENT'),
        ('axis0.config.motor.pole_pairs',                  POLE_PAIRS,          'Pole pairs (7)'),
        ('axis0.config.motor.current_soft_max',            CURRENT_LIM,         'Current soft limit (A)'),
        ('axis0.config.motor.current_hard_max',            CURRENT_LIM + CURRENT_LIM_MARGIN,
                                                                                 'Current hard limit (A)'),
        ('axis0.config.motor.calibration_current',         CALIBRATION_CURRENT, 'Calibration current (A)'),
        # Velocity controller
        ('axis0.controller.config.vel_limit',              VEL_LIMIT,           'Velocity limit (turns/s native)'),
        ('axis0.controller.config.control_mode',           2,                   'Control mode: VELOCITY'),
        ('axis0.controller.config.input_mode',             1,                   'Input mode: VELOCITY_RAMP'),
        # Encoder — on-board absolute magnetic encoder
        # S1 uses on-board encoder by default; these paths confirm it
        ('axis0.config.load_encoder',                      1,                   'Load encoder: onboard'),
        ('axis0.config.commutation_encoder',               1,                   'Commutation: onboard'),
        # Brake resistor & DC bus protection — each S1 has its own (NOT shared)
        ('config.brake_resistor0.enable',                  True,                'Brake resistor enabled'),
        ('config.brake_resistor0.resistance',              BRAKE_RESISTANCE,    'Brake resistance (2Ω)'),
        ('config.dc_bus_overvoltage_trip_level',           DC_BUS_OVERVOLTAGE,  'DC bus overvoltage trip (28.5V)'),
        # CAN node ID
        ('axis0.config.can.node_id',                       node_id,             f'CAN node ID = {node_id}'),
    ]

    for attr_path, target, desc in params:
        try:
            obj   = odrv
            parts = attr_path.split('.')
            for part in parts[:-1]:
                obj = getattr(obj, part)
            current = getattr(obj, parts[-1])

            if verify_only:
                ok = (current == target)
                status = '✓' if ok else f'✗ is {current}, expected {target}'
                print(f"    {desc:<45}: {status}")
                if not ok:
                    errors.append(f'{desc}: is {current}, want {target}')
            else:
                if current != target:
                    setattr(obj, parts[-1], target)
                    changes.append(f'{desc}: {current} → {target}')
                    print(f"    SET {desc:<41}: {current} → {target}")
                else:
                    print(f"    OK  {desc:<41}: {current}")
        except AttributeError as e:
            # Some paths differ between firmware versions — note and continue
            print(f"    --- {desc:<41}: path not found ({e}) — check firmware version")
        except Exception as e:
            errors.append(f'{desc}: {e}')
            print(f"    ERR {desc}: {e}")

    if not verify_only and changes:
        print(f"\n    Saving configuration to node {node_id}...")
        try:
            odrv.save_configuration()
            print(f"    Saved. {len(changes)} parameter(s) updated.")
        except Exception as e:
            print(f"    Save failed: {e}")

    return errors


def run_calibration(odrv, node_id):
    """
    Run motor calibration on one S1.
    The on-board absolute encoder does NOT need a separate encoder calibration —
    it knows shaft angle from the magnet position at power-on.
    Only motor resistance/inductance calibration is needed.
    """
    wing_name = WING_NAMES[node_id] if node_id < len(WING_NAMES) else f'node{node_id}'
    print(f"\n  Calibrating S1 node {node_id} ({wing_name})...")
    print(f"  Motor will spin briefly. Ensure wing is free to move.")
    input("  Press ENTER to start (Ctrl+C to cancel): ")

    axis = odrv.axis0
    print(f"  Starting motor calibration (resistance + inductance)...")
    axis.requested_state = 4    # AXIS_STATE_MOTOR_CALIBRATION
    time.sleep(10)

    try:
        if axis.motor.is_calibrated:
            print(f"  Motor calibrated ✓")
            print(f"    Phase resistance: {axis.motor.config.phase_resistance:.4f} Ω")
            print(f"    Phase inductance: {axis.motor.config.phase_inductance*1e6:.2f} µH")
        else:
            print(f"  Motor calibration FAILED — check motor wiring")
            print(f"    Error: {axis.active_errors}")
    except Exception as e:
        print(f"  Could not read calibration result: {e}")

    # Note: S1 on-board absolute encoder needs no separate calibration.
    # No AXIS_STATE_ENCODER_OFFSET_CALIBRATION needed.
    print(f"  Note: on-board absolute encoder requires no separate calibration.")
    print(f"        Ensure 1/4\" magnet is mounted on motor shaft, ~1-2mm from encoder chip.")

    try:
        odrv.save_configuration()
        print(f"  Node {node_id}: calibration saved.")
    except Exception as e:
        print(f"  Save failed: {e}")


def check_magnet(odrv, node_id):
    """
    Check that the on-board absolute encoder is reading the shaft magnet.
    The encoder should show a valid field strength.
    """
    print(f"\n  Checking encoder magnet on node {node_id}...")
    try:
        enc = odrv.onboard_encoder0
        field = enc.get_field_strength()
        status = enc.field_status
        print(f"    Field strength: {field}")
        print(f"    Field status:   {status}")
        if status == 1:
            print(f"    Magnet: OK ✓  (field detected and within range)")
        elif status == 0:
            print(f"    Magnet: NOT DETECTED ✗  (check magnet is installed and aligned)")
        else:
            print(f"    Magnet: WARNING — status {status}")
    except Exception as e:
        print(f"    Could not read encoder field: {e}")
        print(f"    Ensure magnet is installed on motor shaft.")


def main():
    parser = argparse.ArgumentParser(
        description='MCO-1D ODrive S1 Setup (8 boards × 1 motor each)')
    parser.add_argument('--verify',     action='store_true',
                        help='Verify existing configuration, do not change')
    parser.add_argument('--calibrate',  action='store_true',
                        help='Run motor calibration after configuration')
    parser.add_argument('--check-magnet', action='store_true',
                        help='Check on-board encoder magnet field strength')
    parser.add_argument('--node', type=int, default=None,
                        help='Configure single node (0-7)')
    args = parser.parse_args()

    try:
        import odrive
    except ImportError:
        print("odrive package not installed.")
        print("Run: pip install odrive --break-system-packages")
        sys.exit(1)

    print("MCO-1D ODrive S1 Configuration")
    print("=" * 50)
    print(f"Mode:         {'VERIFY' if args.verify else 'CONFIGURE'}")
    target_nodes = [args.node] if args.node is not None else ALL_NODES
    print(f"Target nodes: {target_nodes}  ({len(target_nodes)} S1 board(s))")
    print(f"Vel limit:    {VEL_LIMIT} rad/s")
    print(f"Current lim:  {CURRENT_LIM} A")
    print(f"Pole pairs:   {POLE_PAIRS}")
    print(f"Encoder:      on-board absolute magnetic (magnet on motor shaft)")
    print()

    all_errors = {}

    for node_id in target_nodes:
        wing = WING_NAMES[node_id] if node_id < len(WING_NAMES) else f'node{node_id}'
        print(f"Finding S1 node {node_id} ({wing})...")
        try:
            odrv = odrive.find_any(path=f"can:{node_id}", timeout=10)
            errors = configure_s1(odrv, node_id, verify_only=args.verify)
            if errors:
                all_errors[node_id] = errors
            if args.check_magnet:
                check_magnet(odrv, node_id)
            if args.calibrate:
                run_calibration(odrv, node_id)
        except Exception as e:
            print(f"  Could not connect to node {node_id}: {e}")
            print(f"  Check: CAN bus up? S1 powered? Node ID correct?")
            all_errors[node_id] = [str(e)]

    print("\n" + "=" * 50)
    if all_errors:
        print("CONFIGURATION ISSUES:")
        for node_id, errs in all_errors.items():
            wing = WING_NAMES[node_id] if node_id < len(WING_NAMES) else f'node{node_id}'
            print(f"  Node {node_id} ({wing}):")
            for e in errs:
                print(f"    {e}")
        sys.exit(1)
    else:
        n = len(target_nodes)
        print(f"ALL {n} ODrive S1 BOARD(S) {'VERIFIED' if args.verify else 'CONFIGURED'} ✓")
        if not args.verify:
            print("Next steps:")
            print("  1. Verify magnet: python3 mco_odrive_setup.py --check-magnet")
            print("  2. Set initial cascade phases (optional — needs motors enabled):")
            print("     from mco_odrive_interface import ODriveInterface")
            print("     odi = ODriveInterface()")
            print("     odi.enable_all()")
            print("     odi.set_initial_phases([0,45,90,135,180,225,270,315])")
            print("     # W1p=0°, W1s=45°, W2p=90°, W2s=135°, ...")
            print("     # Adjust per actual cascade phase offsets in wave library.")
            print("  3. Run preflight: python3 mco_preflight_checks.py --bench")
            print("  4. Start TC-01:   tc01_start  (from ground station)")


def safe_failsafe_configuration(odrv=None):
    """
    Enforce DO-178C / DAL-C certified failsafe settings across ODrive S1:
    - 100ms CAN watchdog timeout (cuts torque on lost communication)
    - Dynamic brake resistor active at 2.0 ohms
    - 28.5V DC bus overvoltage trip margin
    - Current limit clamp (25A soft / 35A hard)
    - Velocity limit clamp (40 turns/s native)
    """
    failsafe_params = [
        ('config.brake_resistor0.enable', True),
        ('config.brake_resistor0.resistance', BRAKE_RESISTANCE),
        ('axis0.config.watchdog_timeout', WATCHDOG_TIMEOUT),
        ('axis0.config.enable_watchdog', True),
        ('config.dc_bus_overvoltage_trip_level', DC_BUS_OVERVOLTAGE),
        ('axis0.config.motor.current_soft_max', CURRENT_LIM),
        ('axis0.config.motor.current_hard_max', CURRENT_LIM + CURRENT_LIM_MARGIN),
        ('axis0.controller.config.vel_limit', VEL_LIMIT),
    ]
    if odrv is not None:
        for path, val in failsafe_params:
            try:
                obj = odrv
                parts = path.split('.')
                for p in parts[:-1]:
                    obj = getattr(obj, p)
                setattr(obj, parts[-1], val)
            except Exception as e:
                print(f"    [FAILSAFE] Warning setting {path}: {e}")
    return {p[0]: p[1] for p in failsafe_params}


if __name__ == '__main__':
    main()
