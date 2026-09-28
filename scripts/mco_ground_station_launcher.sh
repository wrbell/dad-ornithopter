#!/usr/bin/env bash
# MCO-1D Ground Station Launcher
# Starts all ground station components in sequence.
#
# Usage:
#   bash scripts/mco_ground_station_launcher.sh [pi_ip] [--gemini|--anthropic]

PI_IP="192.168.7.1"
BACKEND="gemini"

# Parse arguments
for arg in "$@"; do
    case "$arg" in
        --gemini)
            BACKEND="gemini"
            ;;
        --anthropic)
            BACKEND="anthropic"
            ;;
        *)
            if [[ "$arg" =~ ^[0-9]+\.[0-9]+\.[0-9]+\.[0-9]+$ ]]; then
                PI_IP="$arg"
            fi
            ;;
    esac
done

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
FIRMWARE_DIR="$ROOT_DIR/firmware"

echo ""
echo "MCO-1D Ground Station"
echo "═══════════════════════════════════════"
echo "Pi IP:        $PI_IP"
echo "AI Backend:   $BACKEND (default: Gemini 2.5 Flash)"
echo "Root Dir:     $ROOT_DIR"
echo "Firmware Dir: $FIRMWARE_DIR"
echo ""

# Kill any existing instances
pkill -f mco_ws_bridge.py    2>/dev/null
pkill -f mco_api_proxy.py    2>/dev/null
pkill -f mco_gemini_proxy.py 2>/dev/null
sleep 0.5

echo "Starting WebSocket bridge (aircraft ↔ browser)..."
python3 "$FIRMWARE_DIR/mco_ws_bridge.py" "$PI_IP" &
WS_PID=$!
sleep 1

# Start AI Proxy: Gemini default, Anthropic fallback
if [ "$BACKEND" = "gemini" ]; then
    if [ -n "$GEMINI_API_KEY" ]; then
        echo "Starting Google Gemini AI proxy (browser ↔ Gemini 2.5 Flash)..."
        python3 "$FIRMWARE_DIR/mco_gemini_proxy.py" &
        API_PID=$!
    else
        echo "WARNING: GEMINI_API_KEY is not set. Launching Gemini proxy in standby mode."
        echo "To enable AI copilot: export GEMINI_API_KEY='your-key'"
        python3 "$FIRMWARE_DIR/mco_gemini_proxy.py" &
        API_PID=$!
    fi
elif [ "$BACKEND" = "anthropic" ]; then
    if [ -n "$ANTHROPIC_API_KEY" ]; then
        echo "Starting Anthropic API proxy (browser ↔ Claude Sonnet)..."
        python3 "$FIRMWARE_DIR/mco_api_proxy.py" &
        API_PID=$!
    else
        echo "WARNING: ANTHROPIC_API_KEY is not set."
        python3 "$FIRMWARE_DIR/mco_api_proxy.py" &
        API_PID=$!
    fi
fi
sleep 1

echo ""
echo "═══════════════════════════════════════"
echo "Ground station ready."
echo "  WebSocket Bridge: ws://localhost:5758"
echo "  AI Proxy:         http://localhost:5759"
echo ""
echo "Press Ctrl+C to stop all components."
echo "═══════════════════════════════════════"

trap "kill $WS_PID $API_PID 2>/dev/null; exit 0" INT TERM
wait
