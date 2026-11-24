#!/bin/bash
# usage: repeat.sh <core_ws> <out>
ws=$1; out=$2; : > "$out"
run() { # binary count
  local bin=$1 n=$2 pass=0 fail=0
  for i in $(seq 1 "$n"); do
    if timeout -s KILL 120 "$bin" > "$out.last" 2>&1 && grep -q '^\[  PASSED  \]' "$out.last" && ! grep -q '^\[  FAILED  \]' "$out.last"; then
      pass=$((pass+1))
    else
      fail=$((fail+1)); echo "FAIL $(basename "$bin") run $i:" >> "$out"; grep -E 'FAILED|Failure|Assert' "$out.last" | head -5 >> "$out"
    fi
  done
  echo "$(basename "$bin"): $pass/$n passed, $fail failed" >> "$out"
}
run "$ws/build/s57_layer/test_initialize" 50
run "$ws/build/s57_grids/test_reconfigure" 50
for t in test_depth_costs test_tide_offset test_allow_uncharted test_plugin_loading; do run "$ws/build/s57_layer/$t" 10; done
(cd "$ws/build/s57_grids" && ctest -R test_node_launch --repeat until-fail:20 2>&1 | grep -E 'tests passed|Test #' | tail -2) >> "$out"
rm -f "$out.last"
