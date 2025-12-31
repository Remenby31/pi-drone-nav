#!/bin/bash
#
# Install Gazebo for Pi Drone Nav simulation
#
# This script installs Gazebo via Docker (recommended) or natively.
#

set -e

echo "================================================"
echo "  Gazebo Installation for Pi Drone Nav"
echo "================================================"
echo

# Check if Docker is available
if command -v docker &> /dev/null; then
    echo "[OK] Docker is installed"

    echo
    echo "Pulling Gazebo Docker image..."
    echo "(This may take a few minutes)"
    echo

    docker pull gazebo:libgazebo11

    echo
    echo "[OK] Gazebo Docker image pulled successfully"
    echo
    echo "To run Gazebo:"
    echo "  docker run -it --rm \\"
    echo "    -e DISPLAY=\$DISPLAY \\"
    echo "    -v /tmp/.X11-unix:/tmp/.X11-unix \\"
    echo "    --network host \\"
    echo "    gazebo:libgazebo11 gazebo --verbose"
    echo

else
    echo "[!] Docker not found"
    echo
    echo "Option 1: Install Docker"
    echo "  sudo pacman -S docker"
    echo "  sudo systemctl start docker"
    echo "  sudo usermod -aG docker \$USER"
    echo
    echo "Option 2: Install Gazebo natively from AUR"
    echo "  yay -S gazebo"
    echo "  (Warning: This takes 1-2 hours to compile)"
    echo
    exit 1
fi

echo "================================================"
echo "  Installation Complete"
echo "================================================"
echo
echo "Next steps:"
echo "  1. Run the simulation launcher:"
echo "     python -m simulation.launcher"
echo
echo "  2. Or run with pi_drone_nav:"
echo "     python -m simulation.adapters.run_with_sim"
echo
