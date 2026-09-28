#!/bin/bash
# MCO-1D Installation Script
# Run once on a fresh Raspberry Pi OS installation
# Installs all dependencies and configures systemd services

set -e
echo "MCO-1D Installation"
echo "==================="

# Script & repo directory resolution
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

# Python packages
echo "Installing Python packages..."
pip3 install pymavlink odrive python-can pigpio pygame spidev websockets aiohttp smbus2 --break-system-packages

# Enable pigpio daemon
echo "Enabling pigpio..."
sudo systemctl enable pigpiod
sudo systemctl start pigpiod

# Create directories
mkdir -p /home/pi/logs /home/pi/knowledge

# Symlink firmware into /home/pi if running from repository
if [ -d "$REPO_DIR/firmware" ]; then
    echo "Linking firmware into /home/pi..."
    for f in "$REPO_DIR"/firmware/*.py; do
        [ -e "$f" ] && ln -sf "$f" /home/pi/
    done
    ln -sf "$SCRIPT_DIR/mco_can_setup.sh" /home/pi/mco_can_setup.sh
fi

# Install services
echo "Installing systemd services..."
sudo cp "$SCRIPT_DIR/mco-flight.service" /etc/systemd/system/
sudo cp "$SCRIPT_DIR/mco-panel.service"  /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable mco-flight mco-panel

# CAN interface on boot
echo "Configuring CAN interface..."
if ! grep -q "mco_can_setup.sh" /etc/rc.local 2>/dev/null; then
    cat >> /etc/rc.local << 'RCEOF'
# MCO-1D CAN-FD setup
/home/pi/mco_can_setup.sh &
RCEOF
fi

echo ""
echo "Installation complete."
echo "Reboot to start MCO-1D automatically."
echo "Or start now: sudo systemctl start mco-flight"
echo ""
echo "NEXT STEPS:"
echo "  1. Connect ODrives to CAN bus"
echo "  2. Run: python3 mco_odrive_setup.py --calibrate"
echo "  3. Connect Pixhawk to /dev/ttyAMA0 at 921600 baud"
echo "  4. Run: python3 mco_main.py --sim  (test without motors)"
echo "  5. Run: python3 mco_main.py        (live flight)"
