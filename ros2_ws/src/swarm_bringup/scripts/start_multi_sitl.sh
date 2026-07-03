#!/usr/bin/env bash
# Multi-drone SITL — M2 (4 x PX4 instances, Gazebo Harmonic)
#
# Namespacing: instance N → ROS namespace /px4_N → topics /px4_N/fmu/...
# Spawn layout (Y axis, 3 m spacing):
#   px4_1 (0,-4.5)  px4_2 (0,-1.5)  px4_3 (0,1.5)  px4_4 (0,4.5)
#
# Usage: ./start_multi_sitl.sh [headless]

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PX4_DIR="$(realpath "$SCRIPT_DIR/../../../../PX4-Autopilot")"
BUILD="$PX4_DIR/build/px4_sitl_default"
BIN="$BUILD/bin/px4"
ETC="$BUILD/etc"
ROOTFS="$BUILD/rootfs"
[ -n "${1:-}" ] && export HEADLESS=1

echo "=== M2 Multi-drone SITL ==="
echo "PX4 : $BUILD"
echo "Headless : ${HEADLESS:-no}"

# ── Source Gazebo env (sets PX4_GZ_WORLDS, PX4_GZ_MODELS, plugins, resource path)
# shellcheck source=/dev/null
source "$ROOTFS/gz_env.sh"
echo "PX4_GZ_WORLDS : $PX4_GZ_WORLDS"
echo "PX4_GZ_MODELS : $PX4_GZ_MODELS"

# ── Cleanup ───────────────────────────────────────────────────────────────────
echo "[1/3] Killing any existing simulation..."
pkill -9 -f "px4 -i|gz sim|gz server|MicroXRCEAgent" 2>/dev/null || true
sleep 1
# Remove PX4 lock files so new instances start as servers (not clients)
rm -f /tmp/px4_lock-* 2>/dev/null || true
find "$PX4_DIR" -name "parameters.bson" -delete 2>/dev/null || true

# ── Per-instance working dirs (for state file isolation: parameters.bson, dataman)
for i in 1 2 3 4; do
    rm -rf "/tmp/px4_i_$i"
    mkdir -p "/tmp/px4_i_$i"
done

# ── DDS Agent ─────────────────────────────────────────────────────────────────
echo "[2/3] Starting MicroXRCEAgent on UDP :8888 ..."
MicroXRCEAgent udp4 -p 8888 > /tmp/xrce_agent.log 2>&1 &
sleep 0.5

# ── Helper function ────────────────────────────────────────────────────────────
# run_px4_instance <index> <pose "x,y,z,r,p,yaw"> [standalone]
run_px4_instance() {
    local idx="$1"; local pose="$2"; local standalone="${3:-}"
    local work="/tmp/px4_i_$idx"
    (
        cd "$work"
        export PX4_SIM_MODEL=gz_x500
        export GZ_IP=127.0.0.1
        export PX4_GZ_MODEL_POSE="$pose"
        [ -n "$standalone" ] && export PX4_GZ_STANDALONE=1
        "$BIN" -i "$idx" -d "$ETC" > "/tmp/px4_$idx.log" 2>&1
    ) &
}

# ── PX4 instances ─────────────────────────────────────────────────────────────
echo "[3/3] Starting instance 1 — launches Gazebo world..."
run_px4_instance 1 "0,-4.5,0,0,0,0"

echo "      Waiting for Gazebo world..."
until gz topic -l 2>/dev/null | grep -q "^/world/"; do sleep 2; done
echo "      Gazebo world ready. Settling 3 s..."
sleep 3

echo "      Spawning instance 2 at (0, -1.5)..."
run_px4_instance 2 "0,-1.5,0,0,0,0" standalone
sleep 1

echo "      Spawning instance 3 at (0,  1.5)..."
run_px4_instance 3 "0,1.5,0,0,0,0" standalone
sleep 1

echo "      Spawning instance 4 at (0,  4.5)..."
run_px4_instance 4 "0,4.5,0,0,0,0" standalone

echo ""
echo "All 4 instances launched."
echo "Topics: /px4_1/fmu/...  /px4_2/fmu/...  /px4_3/fmu/...  /px4_4/fmu/..."
echo ""
echo "Wait ~40 s for EKF2, then:"
echo "  source /opt/ros/jazzy/setup.bash && source ~/swarm_project/ros2_ws/install/setup.bash"
echo "  ros2 launch swarm_bringup m2_multi_drone.launch.py"
echo ""
echo "Logs: /tmp/px4_{1..4}.log  |  /tmp/xrce_agent.log"
