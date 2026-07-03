#!/usr/bin/env bash
set -uo pipefail

usage() {
  cat <<'EOF'
Usage:
  scripts/run_search_experiment.sh <drones> <v_max_mps> [label]

Examples:
  scripts/run_search_experiment.sh 1 0.70 speed_sweep
  scripts/run_search_experiment.sh 1 1.00 speed_sweep
  scripts/run_search_experiment.sh 2 0.70 dual_validation

Environment overrides:
  WORLD=single_agent_search_room_easy
  BAG_MODE=standard           # off|minimal|standard
  RECORD_VOXEL_MAPS=1        # include voxel/free/merged maps in the bag
  BAG_START_DELAY_S=0        # seconds to wait before starting rosbag
  AUTO_STOP_ON_DONE=0        # 1: stop launch after landing or a clear failure marker
  AUTO_STOP_MAX_RUNTIME_S=3600 # with AUTO_STOP_ON_DONE=1, stop stuck runs after this wall time
  AUTO_STOP_HOVER_TICKS=1200 # with AUTO_STOP_ON_DONE=1, stop persistent emergency hover after this many controller ticks
  DISABLE_VFH=false          # true: direct waypoint following, emergency hover still active
  SHOW_DASHBOARD=true
  USE_SLAM=true
EOF
}

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  usage
  exit 0
fi

if [[ $# -lt 2 || $# -gt 3 ]]; then
  usage >&2
  exit 2
fi

drones="$1"
speed_raw="$2"
label_raw="${3:-speed_sweep}"

if [[ "$drones" != "1" && "$drones" != "2" ]]; then
  echo "ERROR: <drones> must be 1 or 2." >&2
  exit 2
fi

if ! [[ "$speed_raw" =~ ^[0-9]+([.][0-9]+)?$ ]]; then
  echo "ERROR: <v_max_mps> must be a positive number, for example 0.70 or 1.30." >&2
  exit 2
fi

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(cd "$script_dir/.." && pwd)"
cd "$repo_root"

speed_tag="$(printf "%.2f" "$speed_raw")"
label_tag="$(printf "%s" "$label_raw" | tr '[:upper:] ' '[:lower:]_' | tr -cd 'a-z0-9_-')"
if [[ -z "$label_tag" ]]; then
  label_tag="speed_sweep"
fi

run_date="$(date +%F)"
run_time="$(date +%H%M%S)"
world="${WORLD:-single_agent_search_room_easy}"
show_dashboard="${SHOW_DASHBOARD:-true}"
use_slam="${USE_SLAM:-true}"
bag_mode="$(printf "%s" "${BAG_MODE:-standard}" | tr '[:upper:]' '[:lower:]')"
record_voxel_maps="${RECORD_VOXEL_MAPS:-0}"
bag_start_delay_s="${BAG_START_DELAY_S:-0}"
auto_stop_on_done="${AUTO_STOP_ON_DONE:-0}"
auto_stop_max_runtime_s="${AUTO_STOP_MAX_RUNTIME_S:-3600}"
auto_stop_hover_ticks="${AUTO_STOP_HOVER_TICKS:-1200}"
disable_vfh="${DISABLE_VFH:-false}"

case "$bag_mode" in
  off|minimal|standard) ;;
  *)
    echo "ERROR: BAG_MODE must be off, minimal, or standard." >&2
    exit 2
    ;;
esac

base_dir="experiments/experiment_${run_date}_Ndrones_${drones}_vmax_${speed_tag}mps_${label_tag}"
exp_dir="$base_dir"
if [[ -e "$exp_dir" ]]; then
  if [[ -f "$exp_dir/logs/launch.log" || -f "$exp_dir/bags/rosbag/metadata.yaml" || -f "$exp_dir/data/metrics.csv" ]]; then
    exp_dir="${base_dir}_${run_time}"
  fi
fi

mkdir -p "$exp_dir"/{data,logs,bags,plots,screenshots,config}
mkdir -p "$exp_dir/logs"/{ros,gz}

launch_log="$exp_dir/logs/launch.log"
bag_log="$exp_dir/logs/rosbag_record.log"
bag_dir="$exp_dir/bags/rosbag"
qos_overrides="$exp_dir/config/rosbag_qos_overrides.yaml"
scan_csv="$exp_dir/data/scan_samples.csv"
metrics_csv="$exp_dir/data/metrics.csv"

if [[ "$drones" == "1" ]]; then
  launch_file="d1_single_agent_search.launch.py"
else
  launch_file="d2_dual_agent_search.launch.py"
fi

run_cmd=(
  ros2 launch swarm_bringup "$launch_file"
  "world:=$world"
  "show_dashboard:=$show_dashboard"
  "use_slam:=$use_slam"
  "v_max_mps:=$speed_tag"
  "v_max_target_mps:=$speed_tag"
  "v_max_ramp_step_mps:=0.0"
  "disable_vfh:=$disable_vfh"
)

topics=(
  /clock
  /px4_0/map_update_summary
  /px4_0/frontier_goal
  /px4_0/frontier_goal_pose
  /px4_0/frontier_path
  /px4_0/return_path
  /px4_0/drone_path
  /px4_0/range/up
  /px4_0/range/down
  /px4_0/target_found
  /px4_0/mission/result
  /fmu/out/vehicle_odometry
  /fmu/out/vehicle_local_position_v1
  /fmu/out/vehicle_status_v4
  /fmu/out/vehicle_attitude
)

if [[ "$drones" == "2" ]]; then
  topics+=(
    /shared_map_update_summary
    /px4_1/map_update_summary
    /px4_1/frontier_goal
    /px4_1/frontier_goal_pose
    /px4_1/frontier_path
    /px4_1/return_path
    /px4_1/drone_path
    /px4_1/range/up
    /px4_1/range/down
    /px4_1/target_found
    /px4_1/mission/result
    /px4_1/fmu/out/vehicle_odometry
    /px4_1/fmu/out/vehicle_local_position_v1
    /px4_1/fmu/out/vehicle_status_v4
    /px4_1/fmu/out/vehicle_attitude
  )
fi

if [[ "$bag_mode" == "minimal" ]]; then
  topics=(
    /clock
    /px4_0/map_update_summary
    /px4_0/range/up
    /px4_0/range/down
    /px4_0/target_found
    /px4_0/mission/result
    /fmu/out/vehicle_local_position_v1
    /fmu/out/vehicle_status_v4
  )

  if [[ "$drones" == "2" ]]; then
    topics+=(
      /shared_map_update_summary
      /px4_1/map_update_summary
      /px4_1/range/up
      /px4_1/range/down
      /px4_1/target_found
      /px4_1/mission/result
      /px4_1/fmu/out/vehicle_local_position_v1
      /px4_1/fmu/out/vehicle_status_v4
    )
  fi
fi

if [[ "$bag_mode" == "standard" && "$record_voxel_maps" == "1" ]]; then
  topics+=(
    /px4_0/voxel_map
    /px4_0/free_voxel_map
  )
  if [[ "$drones" == "2" ]]; then
    topics+=(
      /px4_1/voxel_map
      /px4_1/free_voxel_map
      /merged_voxel_map
    )
  fi
fi

if [[ "$bag_mode" == "off" ]]; then
  rosbag_log_artifact="disabled"
  rosbag_artifact="disabled"
else
  rosbag_log_artifact="\`logs/rosbag_record.log\`"
  rosbag_artifact="\`bags/rosbag/\`"
fi

cat > "$exp_dir/README.md" <<EOF
# Experiment ${run_date} - ${drones} drone(s) - vmax ${speed_tag} m/s - ${label_tag}

## Setup

| Field | Value |
| --- | --- |
| Date | ${run_date} |
| Start time | ${run_time} |
| Drone count | ${drones} |
| Max speed | ${speed_tag} m/s |
| Speed ramp | disabled |
| World | \`${world}\` |
| Launch file | \`${launch_file}\` |
| Label | \`${label_tag}\` |
| Bag mode | \`${bag_mode}\` |

## Command

\`\`\`bash
${run_cmd[*]}
\`\`\`

## Artifacts

| Artifact | Path |
| --- | --- |
| Launch log | \`logs/launch.log\` |
| Rosbag log | ${rosbag_log_artifact} |
| Rosbag | ${rosbag_artifact} |
| Scan CSV | \`data/scan_samples.csv\` |
| Metrics CSV | \`data/metrics.csv\` |
| Run config | \`config/run_config.env\` |

## Result

Fill after review, or use \`summary.md\` generated at shutdown as the first pass.
EOF

cat > "$exp_dir/config/run_config.env" <<EOF
RUN_DATE=${run_date}
RUN_TIME=${run_time}
DRONES=${drones}
V_MAX_MPS=${speed_tag}
V_MAX_TARGET_MPS=${speed_tag}
V_MAX_RAMP_STEP_MPS=0.0
WORLD=${world}
LABEL=${label_tag}
LAUNCH_FILE=${launch_file}
SHOW_DASHBOARD=${show_dashboard}
USE_SLAM=${use_slam}
BAG_MODE=${bag_mode}
RECORD_VOXEL_MAPS=${record_voxel_maps}
BAG_START_DELAY_S=${bag_start_delay_s}
AUTO_STOP_ON_DONE=${auto_stop_on_done}
AUTO_STOP_MAX_RUNTIME_S=${auto_stop_max_runtime_s}
AUTO_STOP_HOVER_TICKS=${auto_stop_hover_ticks}
DISABLE_VFH=${disable_vfh}
EOF

printf "%s\n" "${run_cmd[@]}" > "$exp_dir/config/run_command.argv"
if [[ "$bag_mode" == "off" ]]; then
  printf "BAG_MODE=off; no rosbag topics recorded.\n" > "$exp_dir/config/recorded_topics.txt"
else
  printf "%s\n" "${topics[@]}" > "$exp_dir/config/recorded_topics.txt"
fi
cat > "$qos_overrides" <<'EOF'
/fmu/out/vehicle_odometry: &best_effort
  reliability: best_effort
  durability: volatile
  history: keep_last
  depth: 10
/fmu/out/vehicle_local_position_v1: *best_effort
/fmu/out/vehicle_status_v4: *best_effort
/fmu/out/vehicle_attitude: *best_effort
/px4_0/drone_path: *best_effort
/px4_0/range/up: *best_effort
/px4_0/range/down: *best_effort
/px4_0/voxel_map: *best_effort
/px4_0/free_voxel_map: *best_effort
/px4_1/fmu/out/vehicle_odometry: *best_effort
/px4_1/fmu/out/vehicle_local_position_v1: *best_effort
/px4_1/fmu/out/vehicle_status_v4: *best_effort
/px4_1/fmu/out/vehicle_attitude: *best_effort
/px4_1/drone_path: *best_effort
/px4_1/range/up: *best_effort
/px4_1/range/down: *best_effort
/px4_1/voxel_map: *best_effort
/px4_1/free_voxel_map: *best_effort
/merged_voxel_map: *best_effort
EOF

# ROS setup scripts are not nounset-safe; source them with `set +u`.
set +u
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash
set -u

export ROS_LOG_DIR="$repo_root/$exp_dir/logs/ros"
export ROS_HOME="$repo_root/$exp_dir/logs/ros_home"
export GZ_LOG_PATH="$repo_root/$exp_dir/logs/gz"
mkdir -p "$ROS_HOME"

launch_pid=""
bag_pid=""
launch_tail_pid=""
monitor_pid=""
finalized=0

pid_or_group_alive() {
  local pid="${1:-}"
  [[ -n "$pid" ]] || return 1
  kill -0 -- "-$pid" 2>/dev/null || kill -0 -- "$pid" 2>/dev/null
}

send_signal_to_pid_or_group() {
  local pid="${1:-}"
  local sig="${2:-TERM}"
  [[ -n "$pid" ]] || return 0
  kill -s "$sig" -- "-$pid" 2>/dev/null || kill -s "$sig" -- "$pid" 2>/dev/null || true
}

stop_pid_or_group() {
  local pid="${1:-}"
  local name="${2:-process}"
  [[ -n "$pid" ]] || return 0
  pid_or_group_alive "$pid" || return 0

  echo "Stopping $name..."
  send_signal_to_pid_or_group "$pid" INT
  for _ in {1..20}; do
    pid_or_group_alive "$pid" || break
    sleep 0.25
  done

  if pid_or_group_alive "$pid"; then
    send_signal_to_pid_or_group "$pid" TERM
    for _ in {1..12}; do
      pid_or_group_alive "$pid" || break
      sleep 0.25
    done
  fi

  if pid_or_group_alive "$pid"; then
    send_signal_to_pid_or_group "$pid" KILL
  fi

  wait "$pid" 2>/dev/null || true
}

stop_pid_only() {
  local pid="${1:-}"
  [[ -n "$pid" ]] || return 0
  kill -0 "$pid" 2>/dev/null || return 0
  kill -TERM "$pid" 2>/dev/null || true
  wait "$pid" 2>/dev/null || true
}

count_fixed_in_launch_log() {
  local pattern="$1"
  awk -v pattern="$pattern" 'index($0, pattern) { count++ } END { print count + 0 }' "$launch_log" 2>/dev/null || printf "0"
}

max_emergency_hover_ticks_in_launch_log() {
  perl -ne '
    if (/EMERGENCY HOVER \[([0-9]+) ticks\]/) {
      $max = $1 if !defined($max) || $1 > $max;
    }
    END { print defined($max) ? $max : 0; print "\n"; }
  ' "$launch_log" 2>/dev/null || printf "0"
}

extract_scan_csv() {
  {
    echo "drone_ns,scan_t_s,alt_m,target_alt_m,layer_index,north_m,east_m,z_m,coverage_pct,layer_coverage_pct,reachable_frontier_clusters,frontier_clusters,fwd_m,vn_mps,ve_mps,vz_mps"
    perl -ne '
      next unless /SCAN t=/;
      my ($ns) = /\[(px4_[0-9]+)\.search_mission_controller\]/;
      $ns //= "px4_0";
      my ($t) = /SCAN t=([0-9]+)s/;
      my ($alt, $talt) = /alt=([0-9.]+)\/([0-9.]+)m/;
      my ($layer) = /layer=([0-9]+)/;
      my ($n, $e, $z) = /pos=\(([-0-9.]+),([-0-9.]+),([-0-9.]+)\)/;
      my ($cov) = /cov=([0-9.]+)%/;
      my ($lc) = /lc=([0-9.]+)%/;
      my ($fr, $ft) = /frontier=([0-9]+)\/([0-9]+)/;
      my ($fwd) = /fwd=([0-9.]+)m/;
      my ($vn, $ve, $vz) = /v=\(([-0-9.]+),([-0-9.]+),([-0-9.]+)\)/;
      next unless defined $t;
      for ($alt, $talt, $layer, $n, $e, $z, $cov, $lc, $fr, $ft, $fwd, $vn, $ve, $vz) { $_ = "" unless defined $_; }
      print join(",", $ns, $t, $alt, $talt, $layer, $n, $e, $z, $cov, $lc, $fr, $ft, $fwd, $vn, $ve, $vz), "\n";
    ' "$launch_log"
  } > "$scan_csv"
}

write_metrics() {
  local bag_duration=""
  local bag_messages=""
  if [[ -f "$bag_dir/metadata.yaml" ]]; then
    bag_messages="$(perl -ne 'if (/message_count:\s*([0-9]+)/) { print "$1\n"; exit }' "$bag_dir/metadata.yaml" || true)"
    bag_duration="$(perl -ne 'if (/nanoseconds:\s*([0-9]+)/) { printf "%.3f\n", $1 / 1000000000.0; exit }' "$bag_dir/metadata.yaml" || true)"
  fi

  # CSV columns: drone_ns,scan_t_s,alt_m,target_alt_m,layer_index,...,coverage_pct($9),layer_coverage_pct($10),...,fwd_m($13)
  awk -F, -v drones="$drones" -v speed="$speed_tag" -v bag_duration="$bag_duration" -v bag_messages="$bag_messages" '
    BEGIN {
      print "metric,value,unit,source,notes";
      print "drone_count," drones ",count,run_config.env,";
      print "v_max_mps," speed ",m/s,run_config.env,";
      print "v_max_target_mps," speed ",m/s,run_config.env,";
      print "v_max_ramp_step_mps,0.0,m/s per layer,run_config.env,Fixed speed run";
    }
    NR > 1 {
      count++;
      ns = $1; last_t = $2; last_cov = $9; last_lc = $10; last_fwd = $13;
      if (count == 1 || $9  + 0 > max_cov + 0) max_cov = $9;
      if ($10 != "" && (max_lc  == "" || $10 + 0 > max_lc  + 0)) max_lc  = $10;
      if ($13 != "" && (min_fwd == "" || $13 + 0 < min_fwd + 0)) min_fwd = $13;
      drone_last_t[ns] = $2; drone_last_cov[ns] = $9;
      if ($9 != "" && (drone_max_cov[ns] == "" || $9 + 0 > drone_max_cov[ns] + 0)) drone_max_cov[ns] = $9;
    }
    END {
      print "scan_lines," count ",count,scan_samples.csv,";
      print "last_logged_scan_time," last_t ",s,scan_samples.csv,";
      print "final_logged_coverage," last_cov ",percent,scan_samples.csv,";
      print "max_logged_coverage," max_cov ",percent,scan_samples.csv,";
      print "final_logged_layer_coverage," last_lc ",percent,scan_samples.csv,";
      print "max_logged_layer_coverage," max_lc ",percent,scan_samples.csv,";
      print "min_logged_forward_range," min_fwd ",m,scan_samples.csv,";
      print "last_logged_forward_range," last_fwd ",m,scan_samples.csv,";
      for (ns in drone_last_t) {
        pfx = ns; sub(/px4_/, "d", pfx); pfx = pfx "_";
        print pfx "last_scan_time," drone_last_t[ns] ",s,scan_samples.csv,";
        print pfx "final_coverage,"  drone_last_cov[ns] ",percent,scan_samples.csv,";
        print pfx "max_coverage,"    drone_max_cov[ns]  ",percent,scan_samples.csv,";
      }
      print "bag_duration," bag_duration ",s,bags/rosbag/metadata.yaml,";
      print "bag_messages," bag_messages ",count,bags/rosbag/metadata.yaml,";
    }
  ' "$scan_csv" > "$metrics_csv"
}

write_summary() {
  local end_time="$1"
  local exit_code="$2"
  local scan_lines=""
  local last_t=""
  local max_cov=""
  local min_fwd=""
  scan_lines="$(awk -F, '$1 == "scan_lines" { print $2 }' "$metrics_csv" 2>/dev/null || true)"
  last_t="$(awk -F, '$1 == "last_logged_scan_time" { print $2 }' "$metrics_csv" 2>/dev/null || true)"
  max_cov="$(awk -F, '$1 == "max_logged_coverage" { print $2 }' "$metrics_csv" 2>/dev/null || true)"
  min_fwd="$(awk -F, '$1 == "min_logged_forward_range" { print $2 }' "$metrics_csv" 2>/dev/null || true)"

  cat > "$exp_dir/summary.md" <<EOF
# Summary - ${run_date} - ${drones} drone(s) - vmax ${speed_tag} m/s

## Run Status

| Field | Value |
| --- | --- |
| End time | ${end_time} |
| Wrapper exit code | ${exit_code} |
| Drone count | ${drones} |
| Max speed | ${speed_tag} m/s |
| World | \`${world}\` |

## First-Pass Metrics

| Metric | Value |
| --- | ---: |
| SCAN samples | ${scan_lines} |
| Last logged scan time | ${last_t} s |
| Max logged coverage | ${max_cov}% |
| Min logged forward range | ${min_fwd} m |

## Notes

Review \`logs/launch.log\`, \`data/scan_samples.csv\`, and \`data/metrics.csv\` before using this run in final plots.
EOF
}

reindex_bag_if_needed() {
  if [[ -d "$bag_dir" && ! -f "$bag_dir/metadata.yaml" ]] && compgen -G "$bag_dir/*.mcap" > /dev/null; then
    echo "Reindexing rosbag metadata..."
    ros2 bag reindex "$bag_dir" >> "$bag_log" 2>&1 || true
  fi
}

finalize() {
  local exit_code="$1"
  if [[ "$finalized" == "1" ]]; then
    return
  fi
  finalized=1
  trap - INT TERM EXIT

  stop_pid_or_group "$launch_pid" "launch process group"
  stop_pid_or_group "$bag_pid" "rosbag recorder"
  stop_pid_only "$launch_tail_pid"
  stop_pid_only "$monitor_pid"
  reindex_bag_if_needed

  if [[ -x "$repo_root/scripts/finalize_search_experiment.sh" ]]; then
    "$repo_root/scripts/finalize_search_experiment.sh" "$exp_dir" "$exit_code" || true
  elif [[ -f "$launch_log" ]]; then
    extract_scan_csv || true
    write_metrics || true
    write_summary "$(date --iso-8601=seconds)" "$exit_code" || true
  fi

  echo
  echo "Experiment folder: $exp_dir"
  echo "Launch log:        $launch_log"
  if [[ "$bag_mode" == "off" ]]; then
    echo "Rosbag:            disabled (BAG_MODE=off)"
  else
    echo "Rosbag:            $bag_dir"
  fi
  echo "Metrics:           $metrics_csv"
}

on_signal() {
  local code="$1"
  finalize "$code"
  exit "$code"
}

trap 'on_signal 130' INT
trap 'on_signal 143' TERM
trap 'finalize $?' EXIT

echo "Experiment folder: $exp_dir"
echo "Running: ${run_cmd[*]}"
echo "Press Ctrl+C once to stop and finalize the experiment."

: > "$launch_log"
: > "$bag_log"
tail -n +1 -f "$launch_log" &
launch_tail_pid="$!"

if [[ "$bag_mode" == "off" ]]; then
  echo "BAG_MODE=off; rosbag recording disabled for this run." >> "$bag_log"
else
  if [[ "$bag_start_delay_s" != "0" ]]; then
    sleep "$bag_start_delay_s"
  fi

  setsid ros2 bag record \
    --storage mcap \
    --storage-preset-profile zstd_fast \
    --max-cache-size 16777216 \
    --disable-keyboard-controls \
    --include-unpublished-topics \
    --qos-profile-overrides-path "$qos_overrides" \
    --output "$bag_dir" \
    --topics "${topics[@]}" \
    >> "$bag_log" 2>&1 &
  bag_pid="$!"
fi

setsid "${run_cmd[@]}" >> "$launch_log" 2>&1 &
launch_pid="$!"

if [[ "$auto_stop_on_done" == "1" ]]; then
  (
    started_epoch="$(date +%s)"
    all_disarmed_epoch=""
    while pid_or_group_alive "$launch_pid"; do
      if [[ -f "$launch_log" ]]; then
        mission_complete_count="$(count_fixed_in_launch_log "Mission complete.")"
        landing_count="$(count_fixed_in_launch_log "Landing detected")"
        disarm_count="$(count_fixed_in_launch_log "Disarmed by landing")"
        max_hover_ticks="$(max_emergency_hover_ticks_in_launch_log)"

        if [[ "$mission_complete_count" -ge "$drones" &&
              "$landing_count" -ge "$drones" &&
              "$disarm_count" -ge "$drones" ]]; then
          echo "AUTO_STOP_ON_DONE: mission complete, all $drones drone(s) landed and disarmed; stopping launch." >> "$launch_log"
          send_signal_to_pid_or_group "$launch_pid" INT
          exit 0
        fi

        # If PX4 has already disarmed after landing, give the controller a short
        # grace window to print Mission complete before stopping the launch tree.
        if [[ "$landing_count" -ge "$drones" && "$disarm_count" -ge "$drones" ]]; then
          if [[ -z "$all_disarmed_epoch" ]]; then
            all_disarmed_epoch="$(date +%s)"
          elif (( $(date +%s) - all_disarmed_epoch >= 15 )); then
            echo "AUTO_STOP_ON_DONE: all $drones drone(s) landed/disarmed; stopping after grace period." >> "$launch_log"
            send_signal_to_pid_or_group "$launch_pid" INT
            exit 0
          fi
        else
          all_disarmed_epoch=""
        fi

        if grep -Eq "Attitude failure|Arm/OFFBOARD timeout|TEST EMERGENCY STOP" "$launch_log"; then
          echo "AUTO_STOP_ON_DONE: clear failure marker logged; stopping launch." >> "$launch_log"
          send_signal_to_pid_or_group "$launch_pid" INT
          exit 0
        fi

        if [[ "$auto_stop_hover_ticks" =~ ^[0-9]+$ &&
              "$auto_stop_hover_ticks" -gt 0 &&
              "$max_hover_ticks" -ge "$auto_stop_hover_ticks" ]]; then
          echo "AUTO_STOP_ON_DONE: emergency hover persisted for ${max_hover_ticks} ticks; stopping launch." >> "$launch_log"
          send_signal_to_pid_or_group "$launch_pid" INT
          exit 0
        fi
      fi

      if [[ "$auto_stop_max_runtime_s" =~ ^[0-9]+$ &&
            "$auto_stop_max_runtime_s" -gt 0 &&
            $(( $(date +%s) - started_epoch )) -ge "$auto_stop_max_runtime_s" ]]; then
        echo "AUTO_STOP_ON_DONE: wall-time limit ${auto_stop_max_runtime_s}s reached; stopping launch." >> "$launch_log"
        send_signal_to_pid_or_group "$launch_pid" INT
        exit 0
      fi

      sleep 2
    done
  ) &
  monitor_pid="$!"
fi

wait "$launch_pid"
launch_status="$?"

finalize "$launch_status"
exit "$launch_status"
