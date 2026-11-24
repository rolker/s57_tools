"""Costmap hand run: Nav2's costmap with only s57_layer, s57_grids serving
two NOAA cells, compared with GDAL's own reading of the cells.

usage: costmap_run.py <label> static|rolling <robot x> <robot y> <origin x> <origin y>
                      <enc_root> <s57_grids_node> <out dir>
"""
import math
import os
import subprocess
import sys
import time

import rclpy
from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState
from nav2_msgs.msg import Costmap
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import oracle  # noqa: E402

label, mode = sys.argv[1], sys.argv[2]
rx, ry, ox, oy = map(float, sys.argv[3:7])
enc_root, grids_exe, out_dir = sys.argv[7:10]
MIN_DEPTH, MAX_CAUTION = 2.0, 6.0
SIZE_M, RES, TILE_M = 1000.0, 1.0, 100.0
STEP = 10  # sample every 10th cell in each direction

os.makedirs(out_dir, exist_ok=True)
params = os.path.join(out_dir, f'{label}_params.yaml')
open(params, 'w').write(f"""/**:
  ros__parameters:
    global_frame: map
    robot_base_frame: base_link
    rolling_window: {'true' if mode == 'rolling' else 'false'}
    width: {int(SIZE_M)}
    height: {int(SIZE_M)}
    resolution: {RES}
    origin_x: {ox}
    origin_y: {oy}
    track_unknown_space: false
    always_send_full_costmap: true
    update_frequency: 2.0
    publish_frequency: 1.0
    robot_radius: 1.0
    plugins: ["chart"]
    chart:
      plugin: "s57_layer::S57Layer"
      minimum_depth: {MIN_DEPTH}
      maximum_caution_depth: {MAX_CAUTION}
      s57_grids_namespace: "/"
""")

# Static transforms: earth -> map (ENU at oracle.LAT0, LON0), map -> base_link.
lat, lon = math.radians(oracle.LAT0), math.radians(oracle.LON0)
o = oracle._O
R = [[oracle._E[i], oracle._N[i], oracle._U[i]] for i in range(3)]
qw = math.sqrt(max(0.0, 1 + R[0][0] + R[1][1] + R[2][2])) / 2
qx = math.copysign(math.sqrt(max(0.0, 1 + R[0][0] - R[1][1] - R[2][2])) / 2, R[2][1] - R[1][2])
qy = math.copysign(math.sqrt(max(0.0, 1 - R[0][0] + R[1][1] - R[2][2])) / 2, R[0][2] - R[2][0])
qz = math.copysign(math.sqrt(max(0.0, 1 - R[0][0] - R[1][1] + R[2][2])) / 2, R[1][0] - R[0][1])

procs = []
logs = {}


def start(name, cmd):
    logs[name] = os.path.join(out_dir, f'{label}_{name}.log')
    procs.append(subprocess.Popen(cmd, stdout=open(logs[name], 'w'), stderr=subprocess.STDOUT))


start('tf_earth', ['ros2', 'run', 'tf2_ros', 'static_transform_publisher',
                   '--x', str(o[0]), '--y', str(o[1]), '--z', str(o[2]),
                   '--qx', str(qx), '--qy', str(qy), '--qz', str(qz), '--qw', str(qw),
                   '--frame-id', 'earth', '--child-frame-id', 'map'])
start('tf_robot', ['ros2', 'run', 'tf2_ros', 'static_transform_publisher',
                   '--x', str(rx), '--y', str(ry), '--frame-id', 'map', '--child-frame-id', 'base_link'])
start('s57_grids', [grids_exe, '--ros-args', '-r', '__node:=s57_grids', '-p', 'enc_root:=' + enc_root])
start('costmap', ['ros2', 'run', 'nav2_costmap_2d', 'nav2_costmap_2d',
                  '--ros-args', '-r', '__node:=costmap', '--params-file', params])

rclpy.init()
node = rclpy.create_node('costmap_hand_run')
latest = {}


def on_costmap(msg):
    latest['msg'] = msg
    latest['t'] = time.monotonic()


def spin_for(seconds):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.1)


def transition(service, tid):
    client = node.create_client(ChangeState, service)
    assert client.wait_for_service(timeout_sec=30.0), service
    req = ChangeState.Request()
    req.transition.id = tid
    fut = client.call_async(req)
    end = time.monotonic() + 60
    while not fut.done() and time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
    return fut.result().success if fut.done() else None


def log_text(name):
    return open(logs[name]).read()


result = []
try:
    print('s57_grids configure/activate:',
          transition('/s57_grids/change_state', Transition.TRANSITION_CONFIGURE),
          transition('/s57_grids/change_state', Transition.TRANSITION_ACTIVATE), flush=True)
    print('costmap configure/activate:',
          transition('/costmap/change_state', Transition.TRANSITION_CONFIGURE),
          transition('/costmap/change_state', Transition.TRANSITION_ACTIVATE), flush=True)
    topic = None
    end = time.monotonic() + 30
    while topic is None and time.monotonic() < end:
        for name, types in node.get_topic_names_and_types():
            if name.endswith('costmap_raw') and 'nav2_msgs/msg/Costmap' in types:
                topic = name
        spin_for(0.2)
    print('raw costmap topic:', topic, flush=True)
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=ReliabilityPolicy.RELIABLE)
    node.create_subscription(Costmap, topic, on_costmap, qos)
    # Wait for the grids, then for the costmap to settle.
    end = time.monotonic() + 120
    while time.monotonic() < end:
        spin_for(0.5)
        if log_text('costmap').count('Received grid map for grid') >= 1:
            break
    grids_at = time.monotonic()
    spin_for(15.0)
    msg = latest.get('msg')
    assert msg is not None, 'no costmap received'
    received = [line.split('grid: ')[-1] for line in log_text('costmap').splitlines()
                if 'Received grid map for grid' in line]
    md = msg.metadata
    mox, moy = md.origin.position.x, md.origin.position.y
    nx, ny = md.size_x, md.size_y
    data = msg.data
    result.append(f'{label}: mode {mode}, robot ({rx}, {ry}), configured origin ({ox}, {oy})')
    result.append(f'  window origin ({mox:.1f}, {moy:.1f}), {nx} x {ny} cells at {md.resolution} m;'
                  f' grids received: {received}')
    counts = {}
    for c in data:
        counts[c] = counts.get(c, 0) + 1
    result.append(f'  all cells: lethal {counts.get(254, 0)}, free {counts.get(0, 0)},'
                  f' no information {counts.get(255, 0)}, other {sum(v for k, v in counts.items() if k not in (0, 254, 255))}')

    chart = oracle.Chart([os.path.join(enc_root, c, c + '.000') for c in ('US5PSMCC', 'US5PSMCD')],
                         MIN_DEPTH, MAX_CAUTION)

    def klass(x, y):
        return chart.classify(*oracle.map_to_ll(x, y))

    def near_boundary(x, y, k, d=7.5):
        return any(klass(x + dx, y + dy) != k for dx in (-d, 0, d) for dy in (-d, 0, d) if dx or dy)

    # The west and south strip that the tile-addressing defect (#56) can
    # leave unmarked: cells whose world coordinate is below the configured
    # origin, within the first partial tile.
    def in_strip(x, y):
        west = x < ox and x >= mox and math.floor((x - ox) / TILE_M) == math.floor((mox + RES / 2 - ox) / TILE_M)
        south = y < oy and y >= moy and math.floor((y - oy) / TILE_M) == math.floor((moy + RES / 2 - oy) / TILE_M)
        return west or south

    tally = {}
    mismatches = []
    for j in range(STEP // 2, ny, STEP):
        for i in range(STEP // 2, nx, STEP):
            x, y = mox + (i + 0.5) * md.resolution, moy + (j + 0.5) * md.resolution
            k = klass(x, y)
            c = data[j * nx + i]
            got = 'lethal' if c == 254 else 'free' if c == 0 else 'noinfo' if c == 255 else 'ramp'
            strip = in_strip(x, y)
            key = (k, got, 'strip' if strip else 'main')
            tally[key] = tally.get(key, 0) + 1
            bad = (k == 'lethal' and got != 'lethal') or (k == 'deep' and got == 'lethal')
            if bad:
                mismatches.append((x, y, k, got, strip, near_boundary(x, y, k)))
    with open(os.path.join(out_dir, f'{label}_mismatches.csv'), 'w') as f:
        f.write('x,y,oracle,costmap,strip,near_boundary\n')
        for m in mismatches:
            f.write(','.join(str(v) for v in m) + '\n')
    total = sum(tally.values())
    result.append(f'  sampled cells (every {STEP}th each way): {total}')
    for key in sorted(tally):
        result.append(f'    oracle {key[0]:7s} costmap {key[1]:6s} {key[2]:5s}: {tally[key]}')
    for region in ('main', 'strip'):
        ms = [m for m in mismatches if (m[4] if region == 'strip' else not m[4])]
        nb = sum(1 for m in ms if m[5])
        result.append(f'  disagreements in {region}: {len(ms)} ({nb} within 7.5 m of a chart boundary,'
                      f' {len(ms) - nb} not)')
        for m in [m for m in ms if not m[5]][:15]:
            result.append(f'    at ({m[0]:.1f}, {m[1]:.1f}): oracle {m[2]}, costmap {m[3]}')
finally:
    for p in procs:
        if p.poll() is None:
            p.terminate()
    for p in procs:
        try:
            p.wait(timeout=10)
        except subprocess.TimeoutExpired:
            p.kill()
    node.destroy_node()
    rclpy.shutdown()
    text = '\n'.join(result)
    print(text, flush=True)
    open(os.path.join(out_dir, f'{label}_result.txt'), 'w').write(text + '\n')
