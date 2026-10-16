#!/usr/bin/env python3
"""
MCO-1D Anthropic API Proxy
===========================
Runs on the ground station laptop alongside mco_ws_bridge.py.
The browser cannot call the Anthropic API directly (CORS + key exposure).
This proxy receives POST /api/anthropic from the browser, forwards to
the Anthropic API, and returns the response.

Usage:
    python3 mco_api_proxy.py

Requirements:
    pip install aiohttp anthropic

The browser ground station connects to:
    ws://localhost:5758  (WebSocket bridge → aircraft)
    http://localhost:5759/api/anthropic  (this proxy → Claude API)

Set your Anthropic API key:
    export ANTHROPIC_API_KEY=sk-ant-...
"""

import asyncio
import json
import os
import sys
try:
    import anthropic
    HAS_ANTHROPIC = True
except ImportError:
    anthropic = None
    HAS_ANTHROPIC = False

try:
    from aiohttp import web
    HAS_AIOHTTP = True
except ImportError:
    web = None
    HAS_AIOHTTP = False

API_KEY   = os.environ.get('ANTHROPIC_API_KEY', '')
PROXY_PORT= 5759
CORS_ORIGIN = '*'   # restrict to localhost in production

client = anthropic.Anthropic(api_key=API_KEY) if (HAS_ANTHROPIC and API_KEY) else None


async def handle_anthropic(request):
    """Forward POST /api/anthropic to the Anthropic API."""
    try:
        body = await request.json()
    except Exception as e:
        return web.json_response({'error': f'Bad request: {e}'}, status=400)

    try:
        response = client.messages.create(
            model         = body.get('model', 'claude-sonnet-4-6'),
            max_tokens    = body.get('max_tokens', 1024),
            system        = body.get('system', ''),
            messages      = body.get('messages', []),
            tools         = body.get('tools', []) or [],
        )

        # Convert to dict for JSON serialisation
        result = {
            'id':           response.id,
            'type':         response.type,
            'role':         response.role,
            'content':      [block.model_dump() for block in response.content],
            'model':        response.model,
            'stop_reason':  response.stop_reason,
            'stop_sequence':response.stop_sequence,
            'usage': {
                'input_tokens':  response.usage.input_tokens,
                'output_tokens': response.usage.output_tokens,
            },
        }
        return web.json_response(result, headers={'Access-Control-Allow-Origin': CORS_ORIGIN})

    except anthropic.APIStatusError as e:
        return web.json_response(
            {'error': str(e), 'status': e.status_code},
            status=e.status_code,
            headers={'Access-Control-Allow-Origin': CORS_ORIGIN},
        )
    except Exception as e:
        return web.json_response(
            {'error': str(e)}, status=500,
            headers={'Access-Control-Allow-Origin': CORS_ORIGIN},
        )


async def handle_options(request):
    """Handle CORS preflight."""
    return web.Response(headers={
        'Access-Control-Allow-Origin':  CORS_ORIGIN,
        'Access-Control-Allow-Methods': 'POST, OPTIONS',
        'Access-Control-Allow-Headers': 'Content-Type',
    })


async def main():
    global client
    if not HAS_ANTHROPIC or not HAS_AIOHTTP:
        print("ERROR: anthropic or aiohttp not installed. Run: pip install aiohttp anthropic")
        sys.exit(1)
    if not API_KEY:
        print("ERROR: ANTHROPIC_API_KEY environment variable not set.")
        print("  export ANTHROPIC_API_KEY=sk-ant-...")
        sys.exit(1)
    if client is None:
        client = anthropic.Anthropic(api_key=API_KEY)
    app = web.Application()
    app.router.add_post('/api/anthropic', handle_anthropic)
    app.router.add_options('/api/anthropic', handle_options)

    runner = web.AppRunner(app)
    await runner.setup()
    site = web.TCPSite(runner, 'localhost', PROXY_PORT)
    await site.start()

    print(f"""
MCO-1D Anthropic API Proxy
===========================
Listening: http://localhost:{PROXY_PORT}/api/anthropic
Model:     claude-sonnet-4-6

The browser ground station sends requests here.
This proxy forwards them to the Anthropic API using your key.

Starting...
""")
    await asyncio.Event().wait()   # run forever


if __name__ == '__main__':
    asyncio.run(main())
