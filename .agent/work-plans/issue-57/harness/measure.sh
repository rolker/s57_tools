#!/usr/bin/env bash
# Scratch measurements for rolker/s57_tools#57 (not part of the package).
#
# Usage: measure.sh <s57_tools checkout> [<navigation2 checkout>]
#
# Copies s57_layer/src from the checkout (rolling or jazzy: the layer source
# is the same on both at the plan commit), applies layer_variants.patch (the
# designs as a runtime switch), builds harness.cpp against the sourced ROS
# environment in a temporary directory and prints one line per run: mean, p50
# and worst updateMap time on BEN's two costmap sizes. With a navigation2
# checkout, Jazzy's InflationLayer is also copied from its origin/jazzy and
# timed as "jazzycopy" (only useful on Rolling; on Jazzy "native" is Jazzy's).
#
# Environment: source the ROS setup first; ROS_DOMAIN_ID must name a free
# domain (the harness creates nodes); JOBS (default 4) is the build's job
# count; OUT (default a new temporary directory) is where it builds;
# CORRECT=1 also runs the correctness scenarios.
set -euo pipefail
here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo="$1"
nav2="${2:-}"
: "${ROS_DOMAIN_ID:?set ROS_DOMAIN_ID to a free domain id}"
unset DISPLAY
out="${OUT:-$(mktemp -d)}"
mkdir -p "$out/src"
cp "$here/harness.cpp" "$here/CMakeLists.txt" "$out/"
cp "$repo/s57_layer/src/s57_layer.cpp" "$repo/s57_layer/src/s57_layer.h" "$out/src/"
patch -s -d "$out" -p1 < "$here/layer_variants.patch"
kinds="native"
if [ -n "$nav2" ]; then
  mkdir -p "$out/jazzy"
  git -C "$nav2" show origin/jazzy:nav2_costmap_2d/plugins/inflation_layer.cpp > "$out/jazzy/inflation_layer.cpp"
  git -C "$nav2" show origin/jazzy:nav2_costmap_2d/include/nav2_costmap_2d/inflation_layer.hpp > "$out/jazzy/inflation_layer.hpp"
  python3 - "$out/jazzy" <<'EOF'
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
EOF
  kinds="native jazzycopy"
fi
cmake -Wno-dev -S "$out" -B "$out/build" -DCMAKE_BUILD_TYPE=Release > "$out/cmake.log" 2>&1
cmake --build "$out/build" -j "${JOBS:-4}" > "$out/build.log" 2>&1
h="$out/build/harness"
echo "built in $out; CPU: $(lscpu | sed -n 's/^Model name: *//p'); nproc $(nproc)"
if [ "${CORRECT:-0}" = 1 ]; then
  "$h" correct 2>/dev/null
fi
# mode: 0 today, 3 (d), 2 (c), 1 (a); 100 global cycles at 1 Hz, 300 local at 5 Hz
for size in local global; do
  if [ "$size" = global ]; then n=100; else n=300; fi
  for kind in $kinds; do
    for hdg in 0 45; do
      for v in 2 5; do
        for mode in 0 3 2 1; do
          "$h" time "$size" "$kind" "$hdg" "$v" "$n" "$mode" 2>/dev/null | grep mean
        done
      done
    done
  done
done
"$h" time global native 45 2 40 4 2>/dev/null | grep mean
for mode in 0 3 1; do "$h" time global none 45 2 60 "$mode" 2>/dev/null | grep mean; done
