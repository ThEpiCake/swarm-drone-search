#!/usr/bin/env bash
# Single-drone SITL — M1
# Usage: ./start_sitl.sh [headless]
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PX4_DIR="$(realpath "$SCRIPT_DIR/../../../../PX4-Autopilot")"
BUILD="$PX4_DIR/build/px4_sitl_default"
HEADLESS="${1:-}"

pkill -9 -f "px4$|gz sim|gz server|MicroXRCEAgent" 2>/dev/null || true
sleep 1
find "$PX4_DIR" -name "parameters.bson" -delete 2>/dev/null || true

echo "[SITL] Starting MicroXRCEAgent..."
MicroXRCEAgent udp4 -p 8888 > /tmp/xrce_agent.log 2>&1 &

mkdir -p /tmp/px4_instance_0
echo "[SITL] Starting PX4 instance 0 (+ Gazebo)..."
(cd /tmp/px4_instance_0 && \
  ${HEADLESS:+HEADLESS=1} \
  "$BUILD/bin/px4" -i 0 "$BUILD/etc" > /tmp/px4_0.log 2>&1) &

echo "[SITL] Ready. Run: ros2 launch swarm_bringup m1_offboard.launch.py"
