#!/usr/bin/env bash
## @file stability_test.sh
## @brief Sample node RSS, sync statistics and resources during a long run.
## @details Logs node liveness, detector/camera/sync/localizer RSS, rates,
## valid-depth ratio, gap count and final USB kernel messages.
## @param $1 Total duration in seconds (default 600).
## @param $2 Sampling interval in seconds (default 30).
## @copyright MIT License, 2026.
# Usage: stability_test.sh [duration_seconds] [interval_seconds]
set -eo pipefail

WS="${ASTRA_RGBD_WS:-$HOME/astra_rgbd_ws}"
DURATION="${1:-600}"
INTERVAL="${2:-30}"
OUT="$WS/logs/stability_$(date +%Y%m%d_%H%M%S).log"
mkdir -p "$WS/logs"

source /opt/ros/melodic/setup.bash
if [ -f "$WS/devel/setup.bash" ]; then
  # shellcheck disable=SC1090
  source "$WS/devel/setup.bash"
fi

rss_kb() {
  local pid
  pid=$(pgrep -f "$1" | head -1 || true)
  if [ -n "$pid" ]; then
    ps -o rss= -p "$pid" 2>/dev/null | tr -d ' '
  else
    echo "0"
  fi
}

echo "stability test: $(date -Is) duration=${DURATION}s interval=${INTERVAL}s" | tee "$OUT"
START=$SECONDS
while [ $((SECONDS - START)) -lt "$DURATION" ]; do
  TS=$(date +%H:%M:%S)
  DET_RSS=$(rss_kb detector_node)
  CAM_RSS=$(rss_kb astra_camera_node)
  SYNC_RSS=$(rss_kb rgbd_sync_node)
  LOC_RSS=$(rss_kb depth_localizer_node)
  MEM=$(free -m | awk 'NR==2{print $3"/"$2"MB"}')
  STATS=$(timeout 3 rostopic echo -n 1 /perception/sync_stats 2>/dev/null | \
    tr '\n' ' ' | sed 's/  */ /g' || true)
  COLOR=$(echo "$STATS" | grep -o 'color_rate_hz: [0-9.]*' | awk '{print $2}')
  DEPTH=$(echo "$STATS" | grep -o 'depth_rate_hz: [0-9.]*' | awk '{print $2}')
  VALID=$(echo "$STATS" | grep -o 'valid_depth_ratio: [0-9.]*' | awk '{print $2}')
  GAPS=$(echo "$STATS" | grep -o 'gap_count: [0-9]*' | awk '{print $2}')
  NODES=$(rosnode list 2>/dev/null | tr '\n' ',' | sed 's/,$//')
  echo "$TS det_rss=${DET_RSS}KB cam_rss=${CAM_RSS}KB sync_rss=${SYNC_RSS}KB loc_rss=${LOC_RSS}KB mem=${MEM} color=${COLOR} depth=${DEPTH} valid=${VALID} gaps=${GAPS} nodes=${NODES}" | tee -a "$OUT"
  sleep "$INTERVAL"
done

echo "stability test end: $(date -Is)" | tee -a "$OUT"
echo "--- kernel USB messages (last 20) ---" | tee -a "$OUT"
dmesg 2>/dev/null | grep -iE "usb|disconnect|tegra-xusb" | tail -20 | tee -a "$OUT" || true
echo "Log: $OUT"