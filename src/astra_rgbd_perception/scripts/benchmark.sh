#!/usr/bin/env bash
## @file benchmark.sh
## @brief Measure topic rates, sync/detection statistics and board resources.
## @details Samples color/depth/detection/point-cloud rates, the latest sync
## and detection timings, CPU/GPU/memory and tegrastats.
## @param $1 Sampling duration per topic in seconds (default 10).
## @copyright MIT License, 2026.
# 中文：采样 topic 频率、同步/检测耗时、CPU/GPU/内存和 tegrastats。
# 用法：benchmark.sh [秒数]
# Usage: benchmark.sh [seconds]
set -eo pipefail

WS="${ASTRA_RGBD_WS:-$HOME/astra_rgbd_ws}"
DURATION="${1:-10}"
OUT="$WS/logs/benchmark_$(date +%Y%m%d_%H%M%S).txt"
mkdir -p "$WS/logs"

source /opt/ros/melodic/setup.bash
if [ -f "$WS/devel/setup.bash" ]; then
  # shellcheck disable=SC1090
  source "$WS/devel/setup.bash"
fi

{
  echo "benchmark: $(date -Is)"
  echo "duration: ${DURATION}s"
  echo
  echo "=== topic rates (last average) ==="
  for topic in \
    /camera/color/image_raw \
    /camera/depth/image_raw \
    /perception/detections_2d \
    /perception/objects_3d \
    /perception/points_xyzrgb; do
    echo "--- $topic"
    timeout "$((DURATION + 2))" rostopic hz "$topic" 2>&1 | tail -1 || true
  done

  echo
  echo "=== latest sync stats ==="
  timeout 5 rostopic echo -n 1 /perception/sync_stats 2>&1 || true

  echo
  echo "=== latest detection timing ==="
  timeout 5 rostopic echo -n 1 /perception/detections_2d 2>&1 | \
    grep -E "preprocess_ms|inference_ms|postprocess_ms|detections:" || true

  echo
  echo "=== memory ==="
  free -m

  echo
  echo "=== top processes ==="
  ps -eo pid,pcpu,pmem,rss,comm --sort=-pcpu | head -10

  echo
  echo "=== power mode / thermal / clocks ==="
  nvpmodel -q 2>/dev/null | head -6 || true
  echo "CPU freq: $(cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_cur_freq 2>/dev/null || echo n/a)"
  echo "GPU freq: $(cat /sys/devices/gpu.0/devfreq/17000000.gv11b/cur_freq 2>/dev/null || echo n/a)"
  echo "GPU load: $(cat /sys/devices/gpu.0/load 2>/dev/null || echo n/a)"
  echo "CPU temp: $(cat /sys/devices/virtual/thermal/thermal_zone0/temp 2>/dev/null || echo n/a)"

  echo
  echo "=== tegrastats (2 samples) ==="
  timeout 3 tegrastats --interval 1000 2>/dev/null | tail -2 || true
} | tee "$OUT"

echo
echo "Benchmark written to $OUT"