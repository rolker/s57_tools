"""Mutation checks on the scratch workspace.

Each mutation edits one source file of the scratch copy, rebuilds that
package's build tree, runs the named test binaries with the scratch build
directory first on LD_LIBRARY_PATH (ldd shown), records the result, and
restores the file.

usage: [ONLY=grids] mutate.py <scratch workspace> <out file>
"""
import os
import re
import subprocess
import sys

ws, out_path = sys.argv[1:3]
src = os.path.join(ws, 'src', 's57_tools')
layer_cpp = os.path.join(src, 's57_layer', 'src', 's57_layer.cpp')
grids_cpp = os.path.join(src, 's57_grids', 'src', 'grid_publisher.cpp')
layer_build = os.path.join(ws, 'build', 's57_layer')
grids_build = os.path.join(ws, 'build', 's57_grids')
out = open(out_path, 'w')

ONLY_GRIDS = os.environ.get('ONLY') == 'grids'
OLD = ['test_depth_costs', 'test_tide_offset', 'test_allow_uncharted', 'test_plugin_loading']


def log(*a):
    print(*a, flush=True)
    print(*a, file=out, flush=True)


def run_tests(build, tests, timeout=120):
    env = dict(os.environ)
    env['LD_LIBRARY_PATH'] = build + ':' + env.get('LD_LIBRARY_PATH', '')
    results = {}
    for t in tests:
        exe = os.path.join(build, t)
        ldd = subprocess.run(['ldd', exe], env=env, capture_output=True, text=True).stdout
        lib = [l.strip() for l in ldd.splitlines() if 'libs57_layer.so' in l]
        try:
            p = subprocess.run(['timeout', '-s', 'KILL', str(timeout), exe],
                               env=env, capture_output=True, text=True)
            text = p.stdout + p.stderr
            rc = p.returncode
        except Exception as e:  # noqa
            text, rc = str(e), -1
        failed = sorted(set(re.findall(r'^\[  FAILED  \] (\S+?)(?: \(|$)', text, re.M)))
        passed = re.findall(r'^\[  PASSED  \] (\d+) test', text, re.M)
        crash = [l for l in text.splitlines() if 'Assertion' in l]
        results[t] = (rc, passed, failed, lib, crash)
    return results


def mutate(name, path, old, new, build, tests, count=1):
    pristine = open(path).read()
    assert pristine.count(old) == count, (name, pristine.count(old))
    open(path, 'w').write(pristine.replace(old, new))
    try:
        b = subprocess.run(['cmake', '--build', build, '-j4'], capture_output=True, text=True)
        if b.returncode != 0:
            log(f'## {name}: BUILD FAILED\n{b.stdout[-2000:]}{b.stderr[-2000:]}')
            return
        log(f'## {name}')
        for t, (rc, passed, failed, lib, crash) in run_tests(build, tests).items():
            log(f'  {t}: exit {rc}; passed {passed}; failed {failed}')
            if lib:
                log(f'    {lib[0]}')
            for c in crash:
                log(f'    {c[:200]}')
    finally:
        open(path, 'w').write(pristine)


# Baseline (unmutated scratch build).
for b in (['cmake', '--build', layer_build, '-j4'], ['cmake', '--build', grids_build, '-j4']):
    subprocess.run(b, capture_output=True, check=True)
log('## baseline')
for t, r in {**({} if ONLY_GRIDS else run_tests(layer_build, ['test_initialize'] + OLD)),
             **run_tests(grids_build, ['test_reconfigure'])}.items():
    log(f'  {t}: exit {r[0]}; passed {r[1]}; failed {r[2]}' + (f'\n    {r[3][0]}' if r[3] else ''))

ONLY_GRIDS = os.environ.get('ONLY') == 'grids'
if ONLY_GRIDS:
    OLD = []
# Helper hard-codes the prefix every existing test uses.
if not ONLY_GRIDS:
  mutate_layer = mutate
else:
  mutate_layer = lambda *a, **k: None
mutate_layer('helper hard-codes "chart_layer."', layer_cpp,
       'const std::string full_name = getFullName(param);',
       'const std::string full_name = "chart_layer." + param;',
       layer_build, ['test_initialize'] + OLD)

# Each read-back misnamed once.
for p in ['enabled', 'minimum_depth', 'maximum_caution_depth', 'overhead_clearance',
          'unsurveyed_cost', 'depth_costs', 'update_timeout', 'tile_size',
          'buffer_fraction', 'allow_uncharted', 'chart_datum_frame', 'sea_surface_frame',
          'tide_invalidate_threshold', 's57_grids_namespace', 'get_datasets_service']:
    mutate_layer(f'read of {p} misnamed', layer_cpp,
           f'node->get_parameter(name_+".{p}",', f'node->get_parameter(name_+".{p}_x",',
           layer_build, ['test_initialize'])

# Misnamed service and wrong topic prefix.
mutate_layer('service misnamed (+= "_x")', layer_cpp,
       '  RCLCPP_INFO_STREAM(logger_, "Using get datasets service: " << service_name);',
       '  service_name += "_x";\n  RCLCPP_INFO_STREAM(logger_, "Using get datasets service: " << service_name);',
       layer_build, ['test_initialize'] + OLD)
mutate_layer('grid topic without the namespace prefix', layer_cpp,
       's57_grids_namespace_+"datasets/"+dataset_info.topic,',
       '"datasets/"+dataset_info.topic,',
       layer_build, ['test_initialize'] + OLD)

# s57_grids fix, each part deleted.
mutate('abort_flag_ reset deleted', grids_cpp,
       '  abort_flag_ = false;\n', '', grids_build, ['test_reconfigure'])
mutate('pending_dataset_grids_.clear() deleted', grids_cpp,
       '  pending_dataset_grids_.clear();\n', '', grids_build, ['test_reconfigure'])
mutate('missing-grid check deleted', grids_cpp,
       '        if(!grid)\n        {', '        if(false)\n        {', grids_build, ['test_reconfigure'])

# Restore the build trees.
for b in (['cmake', '--build', layer_build, '-j4'], ['cmake', '--build', grids_build, '-j4']):
    subprocess.run(b, capture_output=True, check=True)
log('## done')
