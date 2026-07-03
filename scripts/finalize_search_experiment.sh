#!/usr/bin/env bash
set -uo pipefail

usage() {
  cat <<'EOF'
Usage:
  scripts/finalize_search_experiment.sh <experiment_dir> [wrapper_exit_code]

Rebuilds scan_samples.csv, metrics.csv, and summary.md from an existing
experiment folder. Useful after Ctrl+C, a crash, or an older wrapper run.
EOF
}

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  usage
  exit 0
fi

if [[ $# -lt 1 || $# -gt 2 ]]; then
  usage >&2
  exit 2
fi

exp_dir="${1%/}"
wrapper_exit_code="${2:-manual_postprocess}"

if [[ ! -d "$exp_dir" ]]; then
  echo "ERROR: experiment directory not found: $exp_dir" >&2
  exit 2
fi

config_file="$exp_dir/config/run_config.env"
launch_log="$exp_dir/logs/launch.log"
bag_log="$exp_dir/logs/rosbag_record.log"
bag_dir="$exp_dir/bags/rosbag"
scan_csv="$exp_dir/data/scan_samples.csv"
metrics_csv="$exp_dir/data/metrics.csv"
summary_file="$exp_dir/summary.md"

if [[ ! -f "$launch_log" ]]; then
  echo "ERROR: launch log not found: $launch_log" >&2
  exit 2
fi

mkdir -p "$exp_dir/data" "$exp_dir/logs"
: >> "$bag_log"

if [[ -f "$config_file" ]]; then
  set +u
  # shellcheck disable=SC1090
  source "$config_file"
  set -u
fi

run_date="${RUN_DATE:-unknown}"
drones="${DRONES:-unknown}"
speed_tag="${V_MAX_MPS:-unknown}"
world="${WORLD:-unknown}"
label="${LABEL:-unknown}"

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

count_fixed() {
  local pattern="$1"
  awk -v pattern="$pattern" 'index($0, pattern) { count++ } END { print count + 0 }' "$launch_log"
}

reindex_bag_if_needed() {
  if [[ ! -d "$bag_dir" || -f "$bag_dir/metadata.yaml" ]]; then
    return 0
  fi
  compgen -G "$bag_dir/*.mcap" > /dev/null || return 0

  if ! command -v ros2 >/dev/null 2>&1 && [[ -f /opt/ros/jazzy/setup.bash ]]; then
    set +u
    # shellcheck disable=SC1091
    source /opt/ros/jazzy/setup.bash
    set -u
  fi

  if command -v ros2 >/dev/null 2>&1; then
    echo "Reindexing rosbag metadata for $bag_dir..."
    ros2 bag reindex "$bag_dir" >> "$bag_log" 2>&1 || true
  fi
}

extract_target_events() {
  perl -ne '
    if (/\[TARGET FOUND\] at \(([-0-9.]+), ([-0-9.]+)\)/) {
      print "$1,$2\n";
    }
  ' "$launch_log"
}

min_emergency_forward_range() {
  perl -ne '
    if (/EMERGENCY HOVER.*fwd=([0-9.]+) m/) {
      $min = $1 if !defined($min) || $1 < $min;
    }
    END {
      print $min if defined($min);
    }
  ' "$launch_log"
}

first_event_time_s() {
  # Seconds from mission start to the first log line containing the
  # fixed-string marker. Mission start is the minimum (epoch - t) over all
  # SCAN t=Ns lines, the same time base as last_logged_scan_time.
  local marker="$1"
  perl -sne '
    if (/\[([0-9]+\.[0-9]+)\].*SCAN t=([0-9]+)s/) {
      my $start = $1 - $2;
      $mstart = $start if !defined($mstart) || $start < $mstart;
    }
    if (!defined($ev) && index($_, $marker) >= 0 && /\[([0-9]+\.[0-9]+)\]/) {
      $ev = $1;
    }
    END {
      printf "%.1f", $ev - $mstart if defined($mstart) && defined($ev);
    }
  ' -- -marker="$marker" "$launch_log"
}

extract_scan_csv
reindex_bag_if_needed

bag_duration=""
bag_messages=""
if [[ -f "$bag_dir/metadata.yaml" ]]; then
  bag_messages="$(perl -ne 'if (/message_count:\s*([0-9]+)/) { print "$1\n"; exit }' "$bag_dir/metadata.yaml" || true)"
  bag_duration="$(perl -ne 'if (/nanoseconds:\s*([0-9]+)/) { printf "%.3f\n", $1 / 1000000000.0; exit }' "$bag_dir/metadata.yaml" || true)"
fi

raw_target_detections="$(count_fixed "TargetFound @ world=")"
confirmed_targets="$(count_fixed "[TARGET FOUND]")"
target_inspections_completed="$(count_fixed "TARGET_INSPECT complete")"
mission_complete="$(count_fixed "Mission complete")"
landing_detected="$(count_fixed "Landing detected")"
disarmed_by_landing="$(count_fixed "Disarmed by landing")"
search_timeout="$(count_fixed "Search timeout")"
all_assigned_layers="$(count_fixed "all assigned layers scanned")"
emergency_hovers="$(count_fixed "EMERGENCY HOVER")"
attitude_failure="$(count_fixed "Attitude failure")"
preflight_failures="$(count_fixed "Preflight Fail")"
min_emergency_fwd="$(min_emergency_forward_range)"
time_to_first_detection_s="$(first_event_time_s "TargetFound @ world=")"
time_to_first_confirmed_s="$(first_event_time_s "[TARGET FOUND]")"

required_landings="${drones}"
[[ "$required_landings" =~ ^[0-9]+$ ]] || required_landings=1

mission_status="incomplete"
mission_note="Run stopped before a complete return and landing sequence was logged"
if [[ "$mission_complete" != "0" && "$landing_detected" != "0" && "$disarmed_by_landing" -ge "$required_landings" ]]; then
  if [[ "$search_timeout" != "0" ]]; then
    mission_status="completed_with_search_timeout"
    mission_note="Returned home after the safety timeout and landed"
  elif [[ "$all_assigned_layers" -ge "$required_landings" ]]; then
    mission_status="completed_all_layers"
    mission_note="All assigned scan layers completed, landing detected, and disarmed by landing"
  else
    mission_status="completed"
    mission_note="Mission complete, landing detected, and disarmed by landing"
  fi
elif [[ "$attitude_failure" != "0" ]]; then
  mission_status="failed_attitude_failure"
  mission_note="PX4 reported attitude failure; exclude from valid scan-time plots"
elif [[ "$emergency_hovers" != "0" ]]; then
  mission_status="incomplete_emergency_hover"
  mission_note="Emergency hover occurred before mission completion"
fi

awk -F, \
  -v drones="$drones" \
  -v speed="$speed_tag" \
  -v raw_targets="$raw_target_detections" \
  -v confirmed_targets="$confirmed_targets" \
  -v ttfd="$time_to_first_detection_s" \
  -v ttfc="$time_to_first_confirmed_s" \
  -v inspections="$target_inspections_completed" \
  -v mission_complete="$mission_complete" \
  -v landing_detected="$landing_detected" \
  -v disarmed="$disarmed_by_landing" \
  -v search_timeout="$search_timeout" \
  -v all_assigned_layers="$all_assigned_layers" \
  -v emergency_hovers="$emergency_hovers" \
  -v min_emergency_fwd="$min_emergency_fwd" \
  -v attitude_failure="$attitude_failure" \
  -v preflight_failures="$preflight_failures" \
  -v bag_duration="$bag_duration" \
  -v bag_messages="$bag_messages" \
  -v mission_status="$mission_status" \
  -v mission_note="$mission_note" '
  # CSV columns (1-indexed): drone_ns,scan_t_s,alt_m,target_alt_m,layer_index,
  #   north_m,east_m,z_m,coverage_pct,layer_coverage_pct,
  #   reachable_frontier_clusters,frontier_clusters,fwd_m,vn_mps,ve_mps,vz_mps
  BEGIN {
    print "metric,value,unit,source,notes";
    print "drone_count," drones ",count,run_config.env,";
    print "v_max_mps," speed ",m/s,run_config.env,";
    print "v_max_target_mps," speed ",m/s,run_config.env,";
    print "v_max_ramp_step_mps,0.0,m/s per layer,run_config.env,Fixed speed run";
  }
  NR > 1 {
    count++;
    ns     = $1;
    last_t = $2; last_alt = $3; target_alt = $4; last_layer = $5;
    last_cov = $9; last_lc = $10; last_fwd = $13;

    if ($3  != "" && (max_alt   == "" || $3  + 0 > max_alt   + 0)) max_alt   = $3;
    if ($5  != "" && (max_layer == "" || $5  + 0 > max_layer + 0)) max_layer = $5;
    if ($9  != "" && (max_cov   == "" || $9  + 0 > max_cov   + 0)) max_cov   = $9;
    if ($10 != "" && (max_lc    == "" || $10 + 0 > max_lc    + 0)) max_lc    = $10;
    if ($13 != "" && (min_fwd   == "" || $13 + 0 < min_fwd   + 0)) min_fwd   = $13;

    # per-drone tracking
    drone_last_t[ns]     = $2;
    drone_last_cov[ns]   = $9;
    drone_last_lc[ns]    = $10;
    drone_last_layer[ns] = $5;
    if ($9 != "" && (drone_max_cov[ns]   == "" || $9 + 0 > drone_max_cov[ns]   + 0)) drone_max_cov[ns]   = $9;
    if ($5 != "" && (drone_max_layer[ns] == "" || $5 + 0 > drone_max_layer[ns] + 0)) drone_max_layer[ns] = $5;
  }
  END {
    print "scan_lines," count ",count,scan_samples.csv,";
    print "last_logged_scan_time," last_t ",s,scan_samples.csv,";
    print "last_logged_altitude," last_alt ",m,scan_samples.csv,";
    print "max_logged_altitude," max_alt ",m,scan_samples.csv,";
    print "target_altitude," target_alt ",m,scan_samples.csv,";
    print "last_logged_layer_index," last_layer ",index,scan_samples.csv,";
    print "max_logged_layer_index," max_layer ",index,scan_samples.csv,";
    print "final_logged_coverage," last_cov ",percent,scan_samples.csv,";
    print "max_logged_coverage," max_cov ",percent,scan_samples.csv,";
    print "final_logged_layer_coverage," last_lc ",percent,scan_samples.csv,";
    print "max_logged_layer_coverage," max_lc ",percent,scan_samples.csv,";
    print "min_logged_forward_range," min_fwd ",m,scan_samples.csv,";

    # per-drone metrics (d0_*, d1_*, ...)
    for (ns in drone_last_t) {
      pfx = ns; sub(/px4_/, "d", pfx); pfx = pfx "_";
      print pfx "last_scan_time,"     drone_last_t[ns]     ",s,scan_samples.csv,";
      print pfx "final_coverage,"     drone_last_cov[ns]   ",percent,scan_samples.csv,";
      print pfx "max_coverage,"       drone_max_cov[ns]    ",percent,scan_samples.csv,";
      print pfx "final_layer_coverage," drone_last_lc[ns]  ",percent,scan_samples.csv,";
      print pfx "max_layer_index,"    drone_max_layer[ns]  ",index,scan_samples.csv,";
    }

    print "raw_target_detections," raw_targets ",count,launch.log,Detector TargetFound lines";
    print "confirmed_targets," confirmed_targets ",count,launch.log,Controller [TARGET FOUND] events";
    print "time_to_first_detection," ttfd ",s,launch.log,First detector TargetFound line relative to mission start";
    print "time_to_first_confirmed_target," ttfc ",s,launch.log,First controller [TARGET FOUND] event relative to mission start";
    print "target_inspections_completed," inspections ",count,launch.log,TARGET_INSPECT complete events";
    print "mission_complete," mission_complete ",count,launch.log,Mission complete lines";
    print "landing_detected," landing_detected ",count,launch.log,PX4 landing detected";
    print "disarmed_by_landing," disarmed ",count,launch.log,PX4 disarmed after landing";
    print "search_timeout," search_timeout ",count,launch.log,Search timeout lines";
    print "all_assigned_layers_scanned," all_assigned_layers ",count,launch.log,All assigned layers completion lines";
    print "emergency_hovers," emergency_hovers ",count,launch.log,Emergency hover events";
    print "min_emergency_forward_range," min_emergency_fwd ",m,launch.log,Minimum fwd range in EMERGENCY HOVER lines";
    print "attitude_failure," attitude_failure ",count,launch.log,PX4 attitude failure lines";
    print "preflight_failures," preflight_failures ",count,launch.log,PX4 preflight failure lines";
    print "bag_duration," bag_duration ",s,bags/rosbag/metadata.yaml,";
    print "bag_messages," bag_messages ",count,bags/rosbag/metadata.yaml,";
    print "mission_status," mission_status ",text,launch.log," mission_note;
  }
' "$scan_csv" > "$metrics_csv"

metric_value() {
  local metric="$1"
  awk -F, -v metric="$metric" '$1 == metric { print $2; exit }' "$metrics_csv"
}

scan_lines="$(metric_value scan_lines)"
last_t="$(metric_value last_logged_scan_time)"
last_alt="$(metric_value last_logged_altitude)"
max_alt="$(metric_value max_logged_altitude)"
target_alt="$(metric_value target_altitude)"
last_layer="$(metric_value last_logged_layer_index)"
max_layer="$(metric_value max_logged_layer_index)"
final_cov="$(metric_value final_logged_coverage)"
max_cov="$(metric_value max_logged_coverage)"
final_lc="$(metric_value final_logged_layer_coverage)"
max_lc="$(metric_value max_logged_layer_coverage)"
min_fwd="$(metric_value min_logged_forward_range)"
d0_max_cov="$(metric_value d0_max_coverage)"
d0_final_cov="$(metric_value d0_final_coverage)"
d0_final_lc="$(metric_value d0_final_layer_coverage)"
d0_max_layer="$(metric_value d0_max_layer_index)"
d1_max_cov="$(metric_value d1_max_coverage)"
d1_final_cov="$(metric_value d1_final_coverage)"
d1_final_lc="$(metric_value d1_final_layer_coverage)"
d1_max_layer="$(metric_value d1_max_layer_index)"

outcome="Incomplete run. Review the log before using it in plots."
if [[ "$mission_status" == "completed_all_layers" ]]; then
  outcome="Completed run. All assigned scan layers finished, and the drone returned, landed, and disarmed."
elif [[ "$mission_status" == "completed_with_search_timeout" ]]; then
  outcome="Completed run with search timeout. The drone returned home, landed, and disarmed."
elif [[ "$mission_status" == "completed" ]]; then
  outcome="Completed run. The drone returned home, landed, and disarmed."
elif [[ "$mission_status" == "failed_attitude_failure" ]]; then
  outcome="Failed run. PX4 reported attitude failure after emergency hover events, so this run should not be used as valid speed-sweep performance data."
elif [[ "$mission_status" == "incomplete_emergency_hover" ]]; then
  outcome="Incomplete run. Emergency hover was logged before a complete return and landing sequence."
fi

cat > "$summary_file" <<EOF
# Summary - ${run_date} - ${drones} drone(s) - vmax ${speed_tag} m/s

## Outcome

${outcome}

## Key Metrics

| Metric | Value |
| --- | ---: |
| SCAN samples | ${scan_lines} |
| Last logged scan time | ${last_t} s |
| Last logged altitude | ${last_alt} m |
| Max logged altitude | ${max_alt} m |
| Target altitude | ${target_alt} m |
| Last logged layer index | ${last_layer} |
| Max logged layer index | ${max_layer} |
| Final logged coverage | ${final_cov}% |
| Max logged coverage | ${max_cov}% |
| Final logged layer coverage | ${final_lc}% |
| Max logged layer coverage | ${max_lc}% |
| Min logged forward range | ${min_fwd} m |
| Emergency hover events | ${emergency_hovers} |
| Min emergency forward range | ${min_emergency_fwd} m |
| Raw target detections | ${raw_target_detections} |
| Confirmed target events | ${confirmed_targets} |
| Time to first detection | ${time_to_first_detection_s} s |
| Time to first confirmed target | ${time_to_first_confirmed_s} s |
| Completed target inspections | ${target_inspections_completed} |
| Bag duration | ${bag_duration} s |
| Bag messages | ${bag_messages} |

EOF

if [[ "${drones}" == "2" ]]; then
  cat >> "$summary_file" <<EOF
## Per-Drone Coverage

| Drone | Max Coverage | Final Coverage | Final Layer Coverage | Max Layer Index |
| --- | ---: | ---: | ---: | ---: |
| px4_0 | ${d0_max_cov}% | ${d0_final_cov}% | ${d0_final_lc}% | ${d0_max_layer} |
| px4_1 | ${d1_max_cov}% | ${d1_final_cov}% | ${d1_final_lc}% | ${d1_max_layer} |

EOF
fi

cat >> "$summary_file" <<EOF
## Target Events

EOF

target_events="$(extract_target_events)"
if [[ -n "$target_events" ]]; then
  {
    echo "| Event | Approx. position |"
    echo "| --- | --- |"
    awk -F, '{ printf "| %d | `(%s, %s)` |\n", NR, $1, $2 }' <<< "$target_events"
    echo
  } >> "$summary_file"
else
  {
    echo "No confirmed target events were logged."
    echo
  } >> "$summary_file"
fi

cat >> "$summary_file" <<EOF
## Important Events

- Mission status: \`${mission_status}\` - ${mission_note}.
- Wrapper exit code: \`${wrapper_exit_code}\`.
- Label: \`${label}\`; world: \`${world}\`.
EOF

if [[ "$search_timeout" != "0" ]]; then
  echo "- Search ended by the configured timeout." >> "$summary_file"
fi
if [[ "$emergency_hovers" != "0" ]]; then
  echo "- Emergency hover occurred ${emergency_hovers} time(s); minimum emergency forward range was ${min_emergency_fwd} m." >> "$summary_file"
fi
if [[ "$attitude_failure" != "0" ]]; then
  echo "- PX4 reported attitude failure." >> "$summary_file"
fi
if [[ "$landing_detected" != "0" && "$disarmed_by_landing" != "0" ]]; then
  echo "- PX4 reported landing detected and disarmed by landing." >> "$summary_file"
fi

cat >> "$summary_file" <<EOF

## Notes

Generated by \`scripts/finalize_search_experiment.sh\`. Use \`data/metrics.csv\` for plots, and exclude runs whose \`mission_status\` starts with \`failed\` or \`incomplete\` from valid speed comparisons.
EOF

echo "Finalized experiment: $exp_dir"
echo "Scan CSV:             $scan_csv"
echo "Metrics:              $metrics_csv"
echo "Summary:              $summary_file"
