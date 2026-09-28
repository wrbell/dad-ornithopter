#!/usr/bin/env python3
"""
MCO-1D Ground Station Server
==============================
Runs on the Raspberry Pi alongside the wave library.
Provides a TCP socket interface for the ground station client
(mco_ground_station.py on the laptop) and the touchscreen panel
(mco_panel.py on the Pi itself).

Protocol: newline-delimited JSON over TCP, port 5757.

Incoming (from GS client or panel):
    {"type": "cmd", "cmd": "arm"}
    {"type": "cmd", "cmd": "l9_status"}

Outgoing (to all connected clients):
    {"type": "telemetry", "data": {...}}   — every 5 ticks (10 Hz)
    {"type": "ack", "msg": "..."}          — after each command
    {"type": "status", "data": {...}}      — on mode change

Usage:
    python3 mco_gs_server.py
    (runs as part of mco_main.py — not usually started directly)
"""

import asyncio
import json
import time
import threading


GS_PORT         = 5757
GS_HOST         = '0.0.0.0'    # listen on all interfaces
TELEM_INTERVAL  = 0.1           # 10 Hz telemetry to GS
MAX_CLIENTS     = 4


class MCOGSServer:
    """
    Async TCP server. Accepts multiple simultaneous clients
    (laptop GS + Pi panel + any monitoring tools).
    Thread-safe interface to the wave library.
    """

    def __init__(self):
        self._clients    = set()
        self._telem      = {}       # latest telemetry dict
        self._lock       = threading.Lock()
        self._cmd_queue  = []       # commands waiting to be processed
        self._cmd_lock   = threading.Lock()
        self._loop       = None
        self._server     = None
        self._alive      = True

    def set_telemetry(self, data: dict):
        """Update telemetry (called from flight loop thread)."""
        with self._lock:
            self._telem = dict(data)

    def get_pending_commands(self):
        """
        Return and clear pending commands (called from flight loop).
        Returns list of command strings.
        """
        with self._cmd_lock:
            cmds = list(self._cmd_queue)
            self._cmd_queue.clear()
        return cmds

    def queue_ack(self, ack_dict: dict):
        """Queue an acknowledgement to broadcast to all clients."""
        if self._loop and self._loop.is_running():
            msg = json.dumps(ack_dict) + '\n'
            asyncio.run_coroutine_threadsafe(
                self._broadcast(msg.encode()), self._loop)

    async def _broadcast(self, data: bytes):
        """Send data to all connected clients."""
        dead = set()
        for writer in list(self._clients):
            try:
                writer.write(data)
                await writer.drain()
            except Exception:
                dead.add(writer)
        for w in dead:
            self._clients.discard(w)
            try:
                w.close()
            except Exception:
                pass

    async def _handle_client(self, reader, writer):
        """Handle a single client connection."""
        addr = writer.get_extra_info('peername')
        print(f"GS client connected: {addr}")
        self._clients.add(writer)

        # Send current telemetry immediately on connect
        with self._lock:
            telem = dict(self._telem)
        if telem:
            msg = json.dumps({'type': 'telemetry', 'data': telem}) + '\n'
            writer.write(msg.encode())

        try:
            while self._alive:
                try:
                    line = await asyncio.wait_for(reader.readline(), timeout=1.0)
                    if not line:
                        break
                    msg = json.loads(line.decode().strip())
                    if isinstance(msg, dict) and msg.get('type') == 'cmd':
                        with self._cmd_lock:
                            self._cmd_queue.append(msg)
                except asyncio.TimeoutError:
                    continue
                except json.JSONDecodeError:
                    pass
                except Exception:
                    break
        finally:
            self._clients.discard(writer)
            print(f"GS client disconnected: {addr}")
            try:
                writer.close()
            except Exception:
                pass

    async def _telem_broadcaster(self):
        """Periodically broadcast telemetry to all clients."""
        while self._alive:
            await asyncio.sleep(TELEM_INTERVAL)
            with self._lock:
                telem = dict(self._telem)
            if telem and self._clients:
                msg = json.dumps({'type': 'telemetry', 'data': telem}) + '\n'
                await self._broadcast(msg.encode())

    async def _serve(self):
        self._server = await asyncio.start_server(
            self._handle_client, GS_HOST, GS_PORT,
            limit=65536)
        print(f"GS server listening on {GS_HOST}:{GS_PORT}")
        async with self._server:
            await asyncio.gather(
                self._server.serve_forever(),
                self._telem_broadcaster(),
            )

    def start_background(self):
        """Start server in a background thread (non-blocking)."""
        def _run():
            self._loop = asyncio.new_event_loop()
            asyncio.set_event_loop(self._loop)
            try:
                self._loop.run_until_complete(self._serve())
            except Exception as e:
                print(f"GS server error: {e}")

        t = threading.Thread(target=_run, daemon=True, name='mco-gs-server')
        t.start()
        time.sleep(0.3)   # brief wait for server to bind
        return t

    def stop(self):
        self._alive = False
        if self._loop and self._server:
            asyncio.run_coroutine_threadsafe(
                self._server.wait_closed(), self._loop)


if __name__ == '__main__':
    # Standalone test
    srv = MCOGSServer()
    srv.set_telemetry({'V_ms': 0.0, 'h_agl_m': 0.0, 'test': True})
    srv.start_background()
    print("Test server running. Connect with: python3 mco_ground_station.py")
    try:
        while True:
            time.sleep(1)
            srv.set_telemetry({'V_ms': time.time() % 10,
                                'h_agl_m': 0.0,
                                'timestamp': time.time()})
            cmds = srv.get_pending_commands()
            if cmds:
                print(f"Commands received: {cmds}")
                for cmd in cmds:
                    srv.queue_ack({'type': 'ack', 'msg': f'executed: {cmd}'})
    except KeyboardInterrupt:
        srv.stop()
