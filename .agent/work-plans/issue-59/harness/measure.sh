#!/usr/bin/env bash
# Scratch measurements for rolker/s57_tools#59 (plan phase; not part of the package).
#
# Usage: measure.sh <s57_tools checkout at the plan's base> <out dir>
#
# Builds harness.cpp with two copies of the layer compiled in (no installed
# libs57_layer is linked): today (the checkout's s57_layer/src, renamed to
# namespace s57_today) and aprime (the same with aprime.patch applied and a
# failure counter, namespace s57_aprime). Both headers' private members are made
# protected for the test subclass. Then runs:
#   harness families            the deterministic families
#   harness random <n> <seed>   random cases, 4 seeds of 2500 in parallel
#   harness band                today's request across the map-axis band, 1:12000 chart
#   harness time                generateTile time per tile, 1:12000 charts, 100-cell tiles
# Source the ROS environment first. JOBS (default 4) is the build's job count
# and the number of random-case processes.
set -euo pipefail
here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo="$1"
out="$2"
jobs="${JOBS:-4}"
mkdir -p "$out/today" "$out/aprime" "$out/build"
cp "$here/harness.cpp" "$here/CMakeLists.txt" "$out/"
cp "$repo/s57_layer/src/s57_layer.cpp" "$repo/s57_layer/src/s57_layer.h" "$out/today/"
cp "$repo/s57_layer/src/s57_layer.cpp" "$repo/s57_layer/src/s57_layer.h" "$out/aprime/"
(cd "$out/aprime" && patch -s -p3 < "$here/aprime.patch")
python3 -I - "$out/aprime/s57_layer.cpp" <<'PY'
import sys
p = sys.argv[1]
s = open(p).read()
s = s.replace('namespace s57_layer\n{', 'extern long harness_aprime_failures;\nnamespace s57_layer\n{', 1)
s = s.replace('      if(submap_ok)\n      {\n        for(grid_map::SubmapIterator smi(overlap)',
              '      if(!submap_ok)\n        harness_aprime_failures++;\n      if(submap_ok)\n      {\n        for(grid_map::SubmapIterator smi(overlap)', 1)
open(p, 'w').write(s)
PY
for v in today aprime; do
  sed -i 's/^private:/protected:/' "$out/$v/s57_layer.h"
  sed -i -e "s/namespace s57_layer/namespace s57_$v/" -e "s/s57_layer::S57Layer/s57_$v::S57Layer/" \
    -e "s/S57_LAYER_H/S57_${v^^}_LAYER_H/" -e '/PLUGINLIB_EXPORT_CLASS/d' -e '/class_list_macros/d' \
    "$out/$v/s57_layer.cpp" "$out/$v/s57_layer.h"
done
(cd "$out/build" && cmake .. -DCMAKE_BUILD_TYPE=Release -Wno-dev > /dev/null && make -j"$jobs" > /dev/null)
cd "$out"
./build/harness families > families.txt
for k in $(seq 1 "$jobs"); do ./build/harness random 2500 $((100 + k)) > "random$k.txt" & done
wait
./build/harness band > band.txt
./build/harness time > time.txt
tail -n 5 families.txt random*.txt
cat band.txt time.txt
