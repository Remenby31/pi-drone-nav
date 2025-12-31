#!/bin/bash
#
# Launch Gazebo visualization for Pi Drone Nav
#

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$(dirname "$SCRIPT_DIR")")"

echo "=============================================="
echo "  🚁 Pi Drone Nav - Gazebo Visualization"
echo "=============================================="
echo
echo "Project: $PROJECT_DIR"
echo

# Check Docker
if ! command -v docker &> /dev/null; then
    echo "ERROR: Docker not installed"
    exit 1
fi

# Detect display
if [ -z "$DISPLAY" ]; then
    export DISPLAY=:0
fi
echo "Using DISPLAY=$DISPLAY"

# Allow X11 access
echo "Enabling X11 access for Docker..."
xhost +local:root 2>/dev/null || true
xhost +SI:localuser:root 2>/dev/null || true

echo
echo "Launching Gazebo..."
echo "(This may take a moment to start)"
echo

# Run Gazebo with proper X11 forwarding
docker run -it --rm \
    --name pidrone_gazebo \
    --user root \
    -e DISPLAY=$DISPLAY \
    -e QT_X11_NO_MITSHM=1 \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -v $HOME/.Xauthority:/root/.Xauthority:ro \
    -v "$PROJECT_DIR/simulation/gazebo/worlds:/worlds:ro" \
    -v "$PROJECT_DIR/simulation/gazebo/models:/models:ro" \
    -e GAZEBO_MODEL_PATH=/models \
    --network host \
    --privileged \
    gazebo:libgazebo11 \
    gazebo --verbose /worlds/drone_test.world
