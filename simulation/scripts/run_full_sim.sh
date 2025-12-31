#!/bin/bash
#
# Run full Pi Drone Nav simulation
#
# This script starts all components:
# 1. Betaflight SITL
# 2. Gazebo (optional)
# 3. pi_drone_nav with simulation adapters
#

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$(dirname "$SCRIPT_DIR")")"

cd "$PROJECT_DIR"

echo "================================================"
echo "  Pi Drone Nav - Full Simulation"
echo "================================================"
echo
echo "Project directory: $PROJECT_DIR"
echo

# Check if SITL exists
SITL_PATH="$HOME/projects/betaflight/obj/betaflight_2026.6.0-alpha_SITL"
if [ ! -f "$SITL_PATH" ]; then
    echo "[!] Betaflight SITL not found at $SITL_PATH"
    echo "    Compile with: cd ~/projects/betaflight && make SITL"
    exit 1
fi
echo "[OK] Betaflight SITL found"

# Parse arguments
USE_GAZEBO=false
HEADLESS=false
PORT=8080

while [[ $# -gt 0 ]]; do
    case $1 in
        --gazebo)
            USE_GAZEBO=true
            shift
            ;;
        --headless)
            HEADLESS=true
            shift
            ;;
        --port)
            PORT="$2"
            shift 2
            ;;
        *)
            echo "Unknown option: $1"
            exit 1
            ;;
    esac
done

echo
echo "Options:"
echo "  Gazebo: $USE_GAZEBO"
echo "  Headless: $HEADLESS"
echo "  REST API port: $PORT"
echo

# Start simulation
if [ "$USE_GAZEBO" = true ]; then
    echo "Starting with Gazebo..."
    python -m simulation.launcher --port "$PORT"
else
    echo "Starting simple simulation..."
    python -m simulation.adapters.run_with_sim --port "$PORT" -v
fi
