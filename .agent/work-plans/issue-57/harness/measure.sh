#!/usr/bin/env bash
# Scratch measurements for rolker/s57_tools#57 (not part of the package).
#
# Usage: measure.sh <s57_tools checkout> [<navigation2 checkout>]
#
# Builds harness.cpp with two copies of the layer compiled in:
#   final  <checkout>/s57_layer/src as it is in the working tree (the fix);
#   today  s57_layer/src at TODAY_REF (default afa7403, origin/rolling just
#          before #57; the layer source was the same on jazzy then), read with
#          git from the checkout and renamed to namespace s57_today.
# Both headers' private members are made protected for the test subclass.
# With a navigation2 checkout, Jazzy's InflationLayer is copied from its
# origin/jazzy and run as "jazzycopy" (only useful on Rolling; on Jazzy
# "native" is Jazzy's own).
#
# It prints the CPU, then (CORRECT=1) one summary per inflation set-up of
# the adversarial scenarios, then one line per timing run: mean, p50 and
# worst updateMap time on BEN's two costmap sizes, today beside final.
#
# The plan's figures for designs (a), (c) and (d) came from the earlier
# version of this harness: git show 021ebe4:.agent/work-plans/issue-57/harness/
# (layer_variants.patch applies to s57_layer/src at afa7403).
#
# Environment: source the ROS setup first; ROS_DOMAIN_ID must name a free
# domain (the harness creates nodes); JOBS (default 4) is the build's job
# count and the number of correctness processes; OUT (default a new
# temporary directory) is where it builds; CORRECT=1 runs the correctness
# scenarios; TIME=0 skips the timings.
set -euo pipefail
here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo="$1"
nav2="${2:-}"
today_ref="${TODAY_REF:-afa7403}"
jobs="${JOBS:-4}"
: "${ROS_DOMAIN_ID:?set ROS_DOMAIN_ID to a free domain id}"
unset DISPLAY
out="${OUT:-$(mktemp -d)}"
mkdir -p "$out/src" "$out/today"
cp "$here/harness.cpp" "$here/CMakeLists.txt" "$out/"
cp "$repo/s57_layer/src/s57_layer.cpp" "$repo/s57_layer/src/s57_layer.h" "$out/src/"
for f in s57_layer.cpp s57_layer.h; do
  git -C "$repo" show "$today_ref:s57_layer/src/$f" > "$out/today/$f"
done
sed -i 's/^private:/protected:/' "$out/src/s57_layer.h" "$out/today/s57_layer.h"
sed -i -e 's/namespace s57_layer/namespace s57_today/' -e 's/s57_layer::S57Layer/s57_today::S57Layer/' \
  -e 's/S57_LAYER_H/S57_TODAY_LAYER_H/' -e '/PLUGINLIB_EXPORT_CLASS/d' -e '/class_list_macros/d' \
  "$out/today/s57_layer.cpp" "$out/today/s57_layer.h"
kinds="native"
if [ -n "$nav2" ]; then
  mkdir -p "$out/jazzy"
  git -C "$nav2" show origin/jazzy:nav2_costmap_2d/plugins/inflation_layer.cpp > "$out/jazzy/inflation_layer.cpp"
  git -C "$nav2" show origin/jazzy:nav2_costmap_2d/include/nav2_costmap_2d/inflation_layer.hpp > "$out/jazzy/inflation_layer.hpp"
  python3 - "$out/jazzy" <<'PY'
import sys
d = sys.argv[1]
for f in ['inflation_layer.hpp', 'inflation_layer.cpp']:
    p = d + '/' + f
    s = open(p).read()
    s = s.replace('namespace nav2_costmap_2d\n{', 'namespace jazzy_inflation\n{\nusing namespace nav2_costmap_2d;', 1)
    s = s.replace('}  // namespace nav2_costmap_2d', '}  // namespace jazzy_inflation')
    s = s.replace('NAV2_COSTMAP_2D__INFLATION_LAYER_HPP_', 'JAZZY_INFLATION_LAYER_HPP_')
    s = s.replace('#include "nav2_costmap_2d/inflation_layer.hpp"', '#include "inflation_layer.hpp"')
    s = s.replace('PLUGINLIB_EXPORT_CLASS(nav2_costmap_2d::InflationLayer, nav2_costmap_2d::Layer)', '')
    s = s.replace('#include "nav2_costmap_2d/costmap_2d_ros.hpp"', '')
    if f.endswith('.hpp'):
        a = s.index('  static std::shared_ptr<nav2_costmap_2d::InflationLayer> getInflationLayer(')
        b = s.index('  // Provide a typedef to ease future code maintenance')
        s = s[:a] + s[b:]
    else:
        s = s.replace('void\nInflationLayer::onInitialize()\n{', 'void\nInflationLayer::onInitialize()\n{\n'
                      '  auto declareParameter = [this](const std::string & p, const rclcpp::ParameterValue & v) {\n'
                      '    auto n = node_.lock();\n'
                      '    if (!n->has_parameter(name_ + "." + p)) {n->declare_parameter(name_ + "." + p, v);}\n'
                      '  };', 1)
    open(p, 'w').write(s)
PY
  kinds="native jazzycopy"
fi
cmake -Wno-dev -S "$out" -B "$out/build" -DCMAKE_BUILD_TYPE=Release > "$out/cmake.log" 2>&1
cmake --build "$out/build" -j "$jobs" > "$out/build.log" 2>&1
h="$out/build/harness"
echo "built in $out; final: $repo working tree; today: $today_ref; CPU: $(lscpu | sed -n 's/^Model name: *//p'); nproc $(nproc)"
if [ "${CORRECT:-0}" = 1 ]; then
  nsetups=3
  [ -n "$nav2" ] && nsetups=5
  start=$(date +%s)
  for k in $(seq 0 $((nsetups - 1))); do
    "$h" correct "$k" > "$out/correct_$k.txt" 2>/dev/null &
    while [ "$(jobs -rp | wc -l)" -ge "$jobs" ]; do wait -n; done
  done
  wait
  cat "$out"/correct_*.txt | awk 'NR == 1 || !/scenarios x/'
  echo "correctness: $(( $(date +%s) - start )) s"
fi
[ "${TIME:-1}" = 0 ] && exit 0
# 100 global cycles at 1 Hz, 300 local at 5 Hz
for size in local global; do
  if [ "$size" = global ]; then n=100; else n=300; fi
  for kind in $kinds; do
    for hdg in 0 45; do
      for v in 2 5; do
        for code in today final; do
          "$h" time "$size" "$kind" "$hdg" "$v" "$n" "$code" 2>/dev/null | grep mean
        done
      done
    done
  done
done
for code in today final; do "$h" time global none 45 2 60 "$code" 2>/dev/null | grep mean; done
