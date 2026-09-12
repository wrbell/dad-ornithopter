#!/bin/bash
# MCO-1D CAN-FD Interface Setup
# Run once at boot (or add to /etc/rc.local)
# Requires: MCP2518FD or similar CAN-FD hat on Pi SPI

set -e

echo "MCO-1D CAN-FD Setup"
echo "==================="

# Load CAN kernel module
if ! lsmod | grep -q "can_dev"; then
    modprobe can
    modprobe can_raw
    modprobe mcp251xfd   # MCP2518FD driver
    echo "CAN modules loaded"
fi

# Bring up CAN-FD interface at 1 Mbit/s
# Adjust 'can0' to match your interface name (ip link show)
if ip link show can0 > /dev/null 2>&1; then
    ip link set can0 down 2>/dev/null || true
    ip link set can0 up type can bitrate 1000000 dbitrate 8000000 \
        fd on restart-ms 100
    echo "CAN-FD interface can0: UP at 1Mbit/s (data 8Mbit/s)"
else
    echo "WARNING: can0 interface not found"
    echo "Available interfaces: $(ip link show | grep -o 'can[0-9]*' | tr '\n' ' ')"
    exit 1
fi

# Verify
ip -details link show can0

echo ""
echo "CAN-FD ready. Test with: candump can0"
