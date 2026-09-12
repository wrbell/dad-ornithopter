#!/usr/bin/env python3
"""
MCO-1D WebSocket Bridge
=======================
Runs on the ground station laptop. Bridges the browser-based AI assistant
to the Pi's TCP ground station server (port 5757).

The browser cannot open raw TCP sockets for security reasons.
This bridge accepts WebSocket connections from the browser (port 5758)
and relays messages bidirectionally to the Pi's GS server (port 5757).

USAGE:
    python3 mco_ws_bridge.py [pi_ip]
    
    Defaults: pi_ip = 192.168.7.1

WHAT IT DOES:
    Browser AI assistant  ←→  WS port 5758  ←→  TCP port 5757  ←→  Pi flight controller

REQUIREMENTS:
    pip install websockets

The bridge is transparent — every JSON message from the Pi passes through
unmodified to the browser, and every JSON command from the browser passes
through unmodified to the Pi. The AI assistant sees exactly what a normal
ground station client would see.
"""

import asyncio
import json
import sys
import time
import websockets

PI_HOST    = sys.argv[1] if len(sys.argv) > 1 else '192.168.7.1'
PI_PORT    = 5757
WS_PORT    = 5758   # browser connects here
WS_HOST    = 'localhost'

# CORS origins that may connect
ALLOWED_ORIGINS = {'null', 'http://localhost:3000', 'https://claude.ai',
                    'https://claude.site'}

print(f"""
MCO-1D WebSocket Bridge
========================
Pi GS server:   tcp://{PI_HOST}:{PI_PORT}
Browser bridge: ws://{WS_HOST}:{WS_PORT}

The AI ground station assistant should connect to:
  ws://localhost:{WS_PORT}

Starting...
""")


async def bridge_client(websocket):
    """
    Handle one browser (AI assistant) connection.
    Opens a corresponding TCP connection to the Pi and relays in both directions.
    """
    peer = websocket.remote_address
    print(f"  Browser connected: {peer}")

    # Connect to Pi
    try:
        reader, writer = await asyncio.wait_for(
            asyncio.open_connection(PI_HOST, PI_PORT), timeout=5.0)
        print(f"  Pi GS connected: {PI_HOST}:{PI_PORT}")
    except Exception as e:
        err = json.dumps({'type': 'bridge_error',
                           'msg': f'Cannot reach Pi at {PI_HOST}:{PI_PORT}: {e}'})
        await websocket.send(err)
        print(f"  Pi connection failed: {e}")
        return

    # Relay Pi → Browser
    async def pi_to_browser():
        try:
            while True:
                line = await reader.readline()
                if not line:
                    break
                txt = line.decode().strip()
                if txt:
                    await websocket.send(txt)
        except Exception as e:
            try:
                await websocket.send(json.dumps(
                    {'type': 'bridge_error', 'msg': f'Pi connection lost: {e}'}))
            except Exception:
                pass

    # Relay Browser → Pi
    async def browser_to_pi():
        try:
            async for msg in websocket:
                if isinstance(msg, str):
                    writer.write((msg + '\n').encode())
                    await writer.drain()
        except Exception:
            pass

    try:
        await asyncio.gather(pi_to_browser(), browser_to_pi())
    finally:
        try: writer.close()
        except Exception: pass
        print(f"  Browser disconnected: {peer}")


async def main():
    async with websockets.serve(
        bridge_client,
        WS_HOST,
        WS_PORT,
        ping_interval=20,
        ping_timeout=10,
    ):
        print(f"Bridge ready. Open the AI assistant in Claude.ai")
        print(f"Waiting for browser connections on ws://localhost:{WS_PORT} ...\n")
        await asyncio.Future()  # run forever


if __name__ == '__main__':
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\nBridge stopped.")
