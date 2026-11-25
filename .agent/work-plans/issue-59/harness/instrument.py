"""Instrument a copy of s57_layer.cpp for measure.sh (rolker/s57_tools#59).

Usage: instrument.py <s57_layer.cpp> <today|aprime>

Adds counters harness_<v>_requests, _failures (requests grid_map rejects),
_chart_oor (chart indices outside the grid yielded by SubmapIterator) and
_oor_requests (requests yielding at least one), and skips an out-of-range index
before get_cost_from_grid reads the chart's buffer with it.
"""
import sys

path, v = sys.argv[1], sys.argv[2]
with open(path) as f:
    s = f.read()


def rep(a, b):
    global s
    if s.count(a) != 1:
        sys.exit('pattern not found exactly once in %s: %r' % (path, a))
    s = s.replace(a, b)


rep('namespace s57_layer\n{',
    '#include "grid_map_core/GridMapMath.hpp"\n'
    'extern long harness_%s_failures, harness_%s_requests, harness_%s_chart_oor, '
    'harness_%s_oor_requests;\n'
    'namespace s57_layer\n{' % (v, v, v, v))
rep('      bool submap_ok = false;\n',
    '      bool submap_ok = false;\n'
    '      bool harness_req_oor = false;\n'
    '      harness_%s_requests++;\n' % v)
rep('      if(submap_ok)\n      {\n        for(grid_map::SubmapIterator smi(overlap)',
    '      if(!submap_ok)\n        harness_%s_failures++;\n'
    '      if(submap_ok)\n      {\n        for(grid_map::SubmapIterator smi(overlap)' % v)
rep('          grid_map::Index index = *smi;\n',
    '          grid_map::Index index = *smi;\n'
    '          if(!grid_map::checkIfIndexInRange(index, grid.second->getSize()))\n'
    '          {\n'
    '            harness_%s_chart_oor++;\n'
    '            if(!harness_req_oor)\n'
    '            {\n'
    '              harness_req_oor = true;\n'
    '              harness_%s_oor_requests++;\n'
    '            }\n'
    '            continue;\n'
    '          }\n' % (v, v))
with open(path, 'w') as f:
    f.write(s)
