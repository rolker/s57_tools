#!/usr/bin/env bash
# Scratch measurements for rolker/s57_tools#59 (not part of the package).
#
# Usage: measure.sh <s57_tools checkout> <out dir> [after-ref] [today-ref]
#
# Builds harness.cpp with two copies of the layer compiled in (no installed
# libs57_layer is linked): today (s57_layer/src at today-ref, default
# origin/rolling, renamed to namespace s57_today) and after (s57_layer/src at
# after-ref, default HEAD, the committed fix; namespace s57_aprime, directory
# aprime/). In the plan phase the after copy was the plan's base with
# aprime.patch, the planner's draft of the fix, applied. Both headers' private
# members are made protected for the test subclass. Each copy counts its
# submap requests, the requests grid_map rejects, and the chart indices its
# SubmapIterator yields outside the chart grid, checked before
# get_cost_from_grid reads the chart's buffer; such an index is counted and
# skipped, which paints nothing, as the layer itself paints nothing from it
# (getPosition rejects the index). Then runs:
#   harness families            the deterministic families
#   harness random <n> <seed>   random cases, JOBS seeds of 2500 in parallel
#   harness band                today's request across the map-axis band, 1:12000 chart
#   harness oorreal <n> <seed>  both copies on a 1:12000-size chart's west/south edge tiles:
#                               rejected requests and chart indices outside the grid
#   harness time                generateTile time per tile, 1:12000 charts, 100-cell tiles
# Source the ROS environment first. JOBS (default 4) is the build's job count
# and the number of random-case processes; SEED0 (default 100) the seed before
# the first.
set -euo pipefail
here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo="$1"
out="$2"
after_ref="${3:-HEAD}"
today_ref="${4:-origin/rolling}"
jobs="${JOBS:-4}"
seed0="${SEED0:-100}"
mkdir -p "$out/today" "$out/aprime" "$out/build"
cp "$here/harness.cpp" "$here/CMakeLists.txt" "$out/"
for f in s57_layer.cpp s57_layer.h; do
  git -C "$repo" show "$today_ref:s57_layer/src/$f" > "$out/today/$f"
  git -C "$repo" show "$after_ref:s57_layer/src/$f" > "$out/aprime/$f"
done
echo "today: $today_ref $(git -C "$repo" rev-parse --short "$today_ref"); after: $after_ref $(git -C "$repo" rev-parse --short "$after_ref")" > "$out/refs.txt"
for v in today aprime; do
  python3 -I "$here/instrument.py" "$out/$v/s57_layer.cpp" "$v"
  sed -i 's/^private:/protected:/' "$out/$v/s57_layer.h"
  sed -i -e "s/namespace s57_layer/namespace s57_$v/" -e "s/s57_layer::S57Layer/s57_$v::S57Layer/" \
    -e "s/S57_LAYER_H/S57_${v^^}_LAYER_H/" -e '/PLUGINLIB_EXPORT_CLASS/d' -e '/class_list_macros/d' \
    "$out/$v/s57_layer.cpp" "$out/$v/s57_layer.h"
done
(cd "$out/build" && cmake .. -DCMAKE_BUILD_TYPE=Release -Wno-dev > /dev/null && make -j"$jobs" > /dev/null)
cd "$out"
./build/harness families > families.txt
for k in $(seq 1 "$jobs"); do ./build/harness random 2500 $((seed0 + k)) > "random$k.txt" & done
wait
./build/harness band > band.txt
./build/harness oorreal 300000 7 > oorreal.txt
./build/harness time > time.txt
cat refs.txt
tail -n 9 families.txt random*.txt
cat band.txt oorreal.txt time.txt
