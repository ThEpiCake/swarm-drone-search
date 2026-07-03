#!/usr/bin/env bash
set -uo pipefail

usage() {
  cat <<'EOF'
Usage:
  scripts/run_search_experiment_series.sh [label]

Runs the speed sweep automatically:
  drone counts: 1 2
  speeds:       0.70 1.30 1.80 2.00

Environment overrides:
  DRONE_COUNTS="1 2"
  SPEEDS="0.70 1.30 1.80 2.00"
  MAX_ATTEMPTS_PER_RUN=2   # 0 means retry until success
  CONTINUE_ON_MAX_ATTEMPTS=1 # 1 means skip to the next run after the cap
  BAG_MODE=off
  SHOW_DASHBOARD=false
  USE_SLAM=true
  WORLD=single_agent_search_room_easy
EOF
}

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  usage
  exit 0
fi

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(cd "$script_dir/.." && pwd)"
cd "$repo_root"

label_raw="${1:-speed_sweep_auto}"
label_tag="$(printf "%s" "$label_raw" | tr '[:upper:] ' '[:lower:]_' | tr -cd 'a-z0-9_-')"
if [[ -z "$label_tag" ]]; then
  label_tag="speed_sweep_auto"
fi

drone_counts="${DRONE_COUNTS:-1 2}"
speeds="${SPEEDS:-0.70 1.30 1.80 2.00}"
max_attempts="${MAX_ATTEMPTS_PER_RUN:-2}"
continue_on_max_attempts="${CONTINUE_ON_MAX_ATTEMPTS:-1}"

series_dir="experiments/series_$(date +%F_%H%M%S)_${label_tag}"
mkdir -p "$series_dir"
series_csv="$series_dir/series_runs.csv"
cleanup_log="$series_dir/cleanup.log"

cat > "$series_dir/README.md" <<EOF
# Experiment Series - ${label_tag}

Automated speed sweep. A failed or timed-out attempt is documented in
\`series_runs.csv\` and the same drone-count/speed pair is retried up to
\`MAX_ATTEMPTS_PER_RUN\` times. The default is 2 attempts.

Set \`MAX_ATTEMPTS_PER_RUN=0\` to retry until success. If
\`CONTINUE_ON_MAX_ATTEMPTS=1\`, the series skips to the next run after the cap
instead of stopping the whole series. The default is to continue after the cap.
EOF

echo "drone_count,v_max_mps,attempt,runner_exit_code,mission_status,experiment_dir,started_at,ended_at" > "$series_csv"

latest_experiment_dir() {
  local drones="$1"
  local speed_tag="$2"
  local pattern="experiments/experiment_*_Ndrones_${drones}_vmax_${speed_tag}mps_${label_tag}*"
  local best=""
  local best_ts=0
  local d=""
  while IFS= read -r d; do
    [[ -d "$d" ]] || continue
    local ts=0
    ts="$(stat -c %Y "$d" 2>/dev/null || printf "0")"
    if [[ "$ts" -gt "$best_ts" ]]; then
      best_ts="$ts"
      best="$d"
    fi
  done < <(compgen -G "$pattern" || true)
  printf "%s" "$best"
}

metric_value() {
  local exp_dir="$1"
  local metric="$2"
  awk -F, -v metric="$metric" '$1 == metric { print $2; exit }' \
    "$exp_dir/data/metrics.csv" 2>/dev/null || true
}

status_is_success() {
  local status="$1"
  [[ "$status" == "completed_all_layers" || "$status" == "completed" ]]
}

for drones in $drone_counts; do
  for speed in $speeds; do
    speed_tag="$(printf "%.2f" "$speed")"
    attempt=1

    while :; do
      started_at="$(date --iso-8601=seconds)"
      echo "=== drones=${drones} speed=${speed_tag} attempt=${attempt} ==="

      "$script_dir/cleanup_sim.sh" >> "$cleanup_log" 2>&1 || true

      BAG_MODE="${BAG_MODE:-off}" \
      SHOW_DASHBOARD="${SHOW_DASHBOARD:-false}" \
      USE_SLAM="${USE_SLAM:-true}" \
      WORLD="${WORLD:-single_agent_search_room_easy}" \
      AUTO_STOP_ON_DONE=1 \
      "$script_dir/run_search_experiment.sh" "$drones" "$speed_tag" "$label_tag"
      runner_exit_code="$?"

      ended_at="$(date --iso-8601=seconds)"
      exp_dir="$(latest_experiment_dir "$drones" "$speed_tag")"
      mission_status=""
      if [[ -n "$exp_dir" ]]; then
        mission_status="$(metric_value "$exp_dir" mission_status)"
      else
        mission_status="missing_experiment_dir"
      fi

      echo "${drones},${speed_tag},${attempt},${runner_exit_code},${mission_status},${exp_dir},${started_at},${ended_at}" >> "$series_csv"

      if status_is_success "$mission_status"; then
        echo "SUCCESS drones=${drones} speed=${speed_tag} attempt=${attempt} status=${mission_status}"
        break
      fi

      echo "FAILED drones=${drones} speed=${speed_tag} attempt=${attempt} status=${mission_status}; retrying same run."
      if [[ "$max_attempts" != "0" && "$attempt" -ge "$max_attempts" ]]; then
        if [[ "$continue_on_max_attempts" == "1" ]]; then
          skipped_at="$(date --iso-8601=seconds)"
          echo "${drones},${speed_tag},${attempt},skipped,skipped_after_max_attempts,${exp_dir},${started_at},${skipped_at}" >> "$series_csv"
          echo "SKIP drones=${drones} speed=${speed_tag}: reached MAX_ATTEMPTS_PER_RUN=${max_attempts}; continuing to next run."
          break
        else
          echo "ERROR: reached MAX_ATTEMPTS_PER_RUN=${max_attempts} for drones=${drones} speed=${speed_tag}." >&2
          exit 1
        fi
      fi

      attempt=$((attempt + 1))
      sleep 5
    done
  done
done

"$script_dir/cleanup_sim.sh" >> "$cleanup_log" 2>&1 || true

echo "Series complete: $series_dir"
echo "Runs CSV:        $series_csv"
