#!/usr/bin/env bash
set -u

echo "Stopping swarm simulation processes..."

patterns=(
  "MicroXRCEAgent.*udp4.*8888"
  "PX4-Autopilot/.*/px4"
  "gz sim"
  "gz server"
  "ros_gz_bridge.*parameter_bridge"
  "search_mission_controller"
  "voxel_mapper"
  "target_detector"
  "mission_dashboard"
  "slam_toolbox"
  "slam_to_px4.py"
  "slam_pose_fusion.py"
  "odom_publisher.py"
  "lidar_tilt_filter.py"
)

for sig in INT TERM KILL; do
  for pattern in "${patterns[@]}"; do
    pkill -"$sig" -f "$pattern" 2>/dev/null || true
  done

  if [[ "$sig" != "KILL" ]]; then
    sleep 2
  fi
done

rm -rf /tmp/px4_search_0 /tmp/px4_search_1 /tmp/px4_lock-* 2>/dev/null || true

echo "Cleanup complete."
if command -v ss >/dev/null 2>&1; then
  busy="$(ss -lunp 2>/dev/null | awk '/:8888/ { print }' || true)"
  if [[ -n "$busy" ]]; then
    echo "Port 8888 is still busy:"
    echo "$busy"
  else
    echo "Port 8888 is free."
  fi
fi
