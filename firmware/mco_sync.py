#!/usr/bin/env python3
"""
MCO Knowledge Sync — Ground Station Laptop Tool
================================================
Transfers discovered modes and validated principles between
MCO aircraft via the ground station laptop as a hub.

The laptop already talks to whichever Pi is powered on via
the ground station socket. This script handles the file-level
transfer between sessions.

Usage:
    python3 mco_sync.py pull                    # pull from connected aircraft
    python3 mco_sync.py push mco-2.local        # push to new aircraft
    python3 mco_sync.py status                  # show local knowledge files
    python3 mco_sync.py transfer mco-2.local    # pull then push in one step

Requirements on Pi:
    SSH key authentication (no password prompt)
    rsync installed (standard on Pi OS)

Setup (once):
    ssh-keygen -t ed25519 -f ~/.ssh/mco_key
    ssh-copy-id -i ~/.ssh/mco_key pi@mco-1d.local
    ssh-copy-id -i ~/.ssh/mco_key pi@mco-2.local
"""

import sys
import os
import subprocess
import json
import glob
import time
from datetime import datetime

KNOWLEDGE_DIR     = os.path.expanduser('~/mco_knowledge')
PI_KNOWLEDGE_DIR  = '/home/pi/knowledge'
PI_USER           = 'pi'
SSH_KEY           = os.path.expanduser('~/.ssh/mco_key')
DEFAULT_HOST      = 'mco-1d.local'


def ssh_opts():
    opts = ['-o', 'StrictHostKeyChecking=no',
            '-o', 'ConnectTimeout=5']
    if os.path.exists(SSH_KEY):
        opts += ['-i', SSH_KEY]
    return opts


def pull(host=DEFAULT_HOST):
    """Pull knowledge files from aircraft Pi to laptop."""
    os.makedirs(KNOWLEDGE_DIR, exist_ok=True)
    dest = os.path.join(KNOWLEDGE_DIR, host.replace('.', '_'))
    os.makedirs(dest, exist_ok=True)

    cmd = (['rsync', '-avz', '--progress']
           + [f'-e', f'ssh {" ".join(ssh_opts())}']
           + [f'{PI_USER}@{host}:{PI_KNOWLEDGE_DIR}/',
              dest + '/'])

    print(f'Pulling from {host}...')
    result = subprocess.run(cmd, capture_output=False)

    if result.returncode == 0:
        files = glob.glob(os.path.join(dest, '*.json'))
        latest = max(files, key=os.path.getmtime) if files else None
        if latest:
            # Also copy to top-level as 'latest from this host'
            dest_latest = os.path.join(KNOWLEDGE_DIR,
                f'knowledge_from_{host.split(".")[0]}_latest.json')
            import shutil
            shutil.copy2(latest, dest_latest)
            print(f'  Latest: {os.path.basename(latest)}')
            _print_knowledge_summary(latest)
        return True
    else:
        print(f'Pull failed (returncode={result.returncode})')
        print('  Check: Pi is powered on, SSH keys set up, host reachable')
        return False


def push(host, filepath=None, new_mass_kg=None,
         new_span_m=None, new_wing_area_m2=None):
    """Push knowledge file to a new aircraft Pi."""
    if filepath is None:
        # Find most recent knowledge file from any aircraft
        files = glob.glob(os.path.join(KNOWLEDGE_DIR, '**/*.json'),
                          recursive=True)
        files += glob.glob(os.path.join(KNOWLEDGE_DIR, '*.json'))
        files = [f for f in files if 'latest' not in f
                 and os.path.isfile(f)]
        if not files:
            print('No knowledge files found locally.')
            print(f'Run: python3 mco_sync.py pull  (from source aircraft)')
            return False
        filepath = max(files, key=os.path.getmtime)

    print(f'Pushing {os.path.basename(filepath)} to {host}...')
    _print_knowledge_summary(filepath)

    # Copy file to target Pi
    dest = f'{PI_USER}@{host}:{PI_KNOWLEDGE_DIR}/mco_knowledge_transfer.json'
    cmd  = (['rsync', '-avz']
            + [f'-e', f'ssh {" ".join(ssh_opts())}']
            + [filepath, dest])

    result = subprocess.run(cmd, capture_output=False)

    if result.returncode == 0:
        print(f'Transferred to {host}')
        print()
        print('On the new aircraft, at pre-flight:')
        print('  knowledge_import_usb  (if using USB path)')
        print('  OR: knowledge_import  (via ground station command)')
        print()
        if new_mass_kg:
            print(f'  With scaling: new_mass={new_mass_kg}kg '
                  f'new_span={new_span_m}m')
        return True
    else:
        print(f'Push failed (returncode={result.returncode})')
        return False


def transfer(dest_host, source_host=DEFAULT_HOST,
             new_mass_kg=None, new_span_m=None, new_wing_area_m2=None):
    """Pull from source aircraft and immediately push to destination."""
    print(f'Transferring knowledge: {source_host} → laptop → {dest_host}')
    print()

    ok = pull(source_host)
    if not ok:
        print('Pull failed — cannot transfer')
        return False

    # Find the file we just pulled
    source_dir = os.path.join(KNOWLEDGE_DIR,
        source_host.replace('.', '_'))
    files = glob.glob(os.path.join(source_dir, '*.json'))
    if not files:
        print('No files found after pull')
        return False
    latest = max(files, key=os.path.getmtime)

    print()
    return push(dest_host, latest, new_mass_kg, new_span_m, new_wing_area_m2)


def status():
    """Show all local knowledge files."""
    print('LOCAL KNOWLEDGE FILES')
    print('=' * 50)
    files = glob.glob(os.path.join(KNOWLEDGE_DIR, '**/*.json'),
                      recursive=True)
    files += glob.glob(os.path.join(KNOWLEDGE_DIR, '*.json'))
    files = sorted(set(f for f in files if os.path.isfile(f)),
                   key=os.path.getmtime, reverse=True)

    if not files:
        print(f'No knowledge files in {KNOWLEDGE_DIR}')
        print('Run: python3 mco_sync.py pull')
        return

    for f in files[:10]:
        age_s = time.time() - os.path.getmtime(f)
        age_h = age_s / 3600
        age_str = (f'{age_h:.0f}h ago' if age_h > 1
                   else f'{age_s/60:.0f}min ago')
        size_kb = os.path.getsize(f) / 1024
        print(f'  {os.path.basename(f):<45} '
              f'{size_kb:>6.1f}KB  {age_str}')
        _print_knowledge_summary(f, indent=4)


def _print_knowledge_summary(filepath, indent=2):
    """Print a one-line summary of a knowledge file."""
    try:
        with open(filepath) as f:
            k = json.load(f)
        src  = k.get('source_aircraft', {}).get('aircraft_id', '?')
        n    = k.get('n_principles', 0)
        date = k.get('exported_at_str', '?')[:10]
        expl = k.get('exploration_stats', {})
        n_tr = expl.get('n_trials', {})
        total_trials = sum(n_tr.values()) if isinstance(n_tr, dict) else 0
        sp   = ' ' * indent
        print(f'{sp}Source: {src}  '
              f'Date: {date}  '
              f'Principles: {n}  '
              f'Trials: {total_trials}')
    except Exception:
        pass


def main():
    import argparse
    parser = argparse.ArgumentParser(description='MCO Knowledge Sync')
    parser.add_argument('command',
                        choices=['pull','push','transfer','status'],
                        help='Action to perform')
    parser.add_argument('host', nargs='?', default=DEFAULT_HOST,
                        help='Target Pi hostname (e.g. mco-2.local)')
    parser.add_argument('--source', default=DEFAULT_HOST,
                        help='Source Pi for transfer command')
    parser.add_argument('--file',   default=None,
                        help='Specific knowledge file to push')
    parser.add_argument('--mass',   type=float, default=None,
                        help='New aircraft mass (kg) for scaling')
    parser.add_argument('--span',   type=float, default=None,
                        help='New aircraft wingspan (m) for scaling')
    parser.add_argument('--area',   type=float, default=None,
                        help='New aircraft wing area (m²) for scaling')
    args = parser.parse_args()

    os.makedirs(KNOWLEDGE_DIR, exist_ok=True)

    if args.command == 'pull':
        pull(args.host)
    elif args.command == 'push':
        push(args.host, args.file, args.mass, args.span, args.area)
    elif args.command == 'transfer':
        transfer(args.host, args.source, args.mass, args.span, args.area)
    elif args.command == 'status':
        status()


if __name__ == '__main__':
    main()
