#!/bin/bash
# Pi Drone Navigation - Installation Script
# Run this script on your Raspberry Pi to install dependencies

set -e

echo "================================================"
echo "  Pi Drone Navigation - Installation Script"
echo "================================================"
echo ""

# Check if running on Raspberry Pi
if [[ -f /proc/device-tree/model ]]; then
    MODEL=$(cat /proc/device-tree/model)
    echo "Detected: $MODEL"
else
    echo "Warning: Not running on Raspberry Pi"
    echo "Some features may not work correctly"
fi

echo ""
echo "Step 1: Updating system packages..."
sudo apt-get update

echo ""
echo "Step 2: Installing system dependencies..."
sudo apt-get install -y \
    python3 \
    python3-pip \
    python3-venv \
    python3-dev \
    git \
    build-essential \
    libffi-dev \
    libssl-dev

echo ""
echo "Step 3: Creating Python virtual environment..."
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"

cd "$PROJECT_DIR"

if [[ ! -d "venv" ]]; then
    python3 -m venv venv
    echo "Virtual environment created"
else
    echo "Virtual environment already exists"
fi

echo ""
echo "Step 4: Activating virtual environment..."
source venv/bin/activate

echo ""
echo "Step 5: Upgrading pip..."
pip install --upgrade pip

echo ""
echo "Step 6: Installing Python dependencies..."
pip install -r requirements.txt

echo ""
echo "Step 7: Installing Pi Drone Navigation..."
pip install -e .

echo ""
echo "Step 8: Configuring serial ports..."

# Add user to dialout group for serial access
if ! groups | grep -q dialout; then
    sudo usermod -a -G dialout "$USER"
    echo "Added $USER to dialout group (re-login required)"
else
    echo "User already in dialout group"
fi

# Disable serial console on Raspberry Pi (required for GPS on UART)
if [[ -f /boot/cmdline.txt ]] || [[ -f /boot/firmware/cmdline.txt ]]; then
    echo ""
    echo "Note: You may need to disable the serial console"
    echo "Run: sudo raspi-config -> Interface Options -> Serial Port"
    echo "     Disable login shell, Enable serial hardware"
fi

echo ""
echo "Step 9: Creating systemd service (optional)..."

read -p "Install as systemd service? [y/N] " -n 1 -r
echo
if [[ $REPLY =~ ^[Yy]$ ]]; then
    sudo tee /etc/systemd/system/pi-drone-nav.service > /dev/null << EOF
[Unit]
Description=Pi Drone Navigation
After=network.target

[Service]
Type=simple
User=$USER
WorkingDirectory=$PROJECT_DIR
ExecStart=$PROJECT_DIR/venv/bin/python -m src.main
Restart=on-failure
RestartSec=5

[Install]
WantedBy=multi-user.target
EOF

    sudo systemctl daemon-reload
    echo "Service installed: pi-drone-nav.service"
    echo "Enable with: sudo systemctl enable pi-drone-nav"
    echo "Start with:  sudo systemctl start pi-drone-nav"
fi

echo ""
echo "================================================"
echo "  Installation Complete!"
echo "================================================"
echo ""
echo "To get started:"
echo "  1. Activate virtual environment: source venv/bin/activate"
echo "  2. Copy config/default.yaml to config.yaml"
echo "  3. Edit config.yaml with your settings"
echo "  4. Run: python -m src.main"
echo ""
echo "For simulation mode (no hardware):"
echo "  python -m src.main --simulate"
echo ""
echo "Important: Log out and back in for serial port access"
echo ""
