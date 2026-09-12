#!/usr/bin/env bash
# MCO-1D Ground Station Launcher
# Starts all three ground station components in sequence.
# Run from the laptop, with the Pi already booted and running mco_main.py.
#
# Usage:
#   bash mco_ground_station_launcher.sh [pi_ip]
#   pi_ip defaults to 192.168.7.1

PI_IP="${1:-192.168.7.1}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo ""
echo "MCO-1D Ground Station"
echo "═══════════════════════════════════════"
echo "Pi IP:  $PI_IP"
echo "Script: $SCRIPT_DIR"
echo ""

# Check API key
if [ -z "$ANTHROPIC_API_KEY" ]; then
    echo "ERROR: ANTHROPIC_API_KEY not set."
    echo "  export ANTHROPIC_API_KEY=sk-ant-..."
    exit 1
fi

# Kill any existing instances
pkill -f mco_ws_bridge.py    2>/dev/null
pkill -f mco_api_proxy.py    2>/dev/null
sleep 0.5

echo "Starting WebSocket bridge (aircraft ↔ browser)..."
python3 "$SCRIPT_DIR/mco_ws_bridge.py" "$PI_IP" &
WS_PID=$!
sleep 1

echo "Starting Anthropic API proxy (browser ↔ Claude)..."
python3 "$SCRIPT_DIR/mco_api_proxy.py" &
API_PID=$!
sleep 1

echo ""
echo "═══════════════════════════════════════"
echo "Ground station ready."
echo ""
echo "Open the AI ground station:"
echo "  → Load mco_ai_ground_station.jsx in Claude Artifacts"
echo "    or serve it from a local dev server"
echo ""
echo "The browser needs:"
echo "  ws://localhost:5758   (aircraft connection)"
echo "  http://localhost:5759 (Claude API proxy)"
echo ""
echo "Press Ctrl+C to stop all components."
echo "═══════════════════════════════════════"

# Wait for both processes
trap "kill $WS_PID $API_PID 2>/dev/null; exit 0" INT TERM
wait
