"""Reproduce the s57_grids abort_flag_ defect.

configure, activate, request the west cell, wait for "Grid ready";
deactivate, cleanup, configure, activate, request the east cell, and report
whether the node publishes it or dies. With --no-cycle, skips the
deactivate/cleanup/configure/activate in the middle (control).

usage: repro_abort.py <s57_grids_node executable> <enc_root> <log file> [--no-cycle]
"""
import math
import subprocess
import sys
import time

import rclpy
from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState
from s57_msgs.srv import GetDatasets

exe, enc_root, log_path = sys.argv[1:4]
cycle = '--no-cycle' not in sys.argv
NODE = 's57_grids_repro'

# ENU "map" at 43.07 N, 70.75 W (WGS84).
lat, lon = math.radians(43.07), math.radians(-70.75)
a, f = 6378137.0, 1 / 298.257223563
e2 = f * (2 - f)
N = a / math.sqrt(1 - e2 * math.sin(lat) ** 2)
x = N * math.cos(lat) * math.cos(lon)
y = N * math.cos(lat) * math.sin(lon)
z = N * (1 - e2) * math.sin(lat)
E = (-math.sin(lon), math.cos(lon), 0.0)
Nn = (-math.sin(lat) * math.cos(lon), -math.sin(lat) * math.sin(lon), math.cos(lat))
U = (math.cos(lat) * math.cos(lon), math.cos(lat) * math.sin(lon), math.sin(lat))
R = [[E[i], Nn[i], U[i]] for i in range(3)]  # columns: map axes in earth
tr = R[0][0] + R[1][1] + R[2][2]
qw = math.sqrt(max(0.0, 1 + tr)) / 2
qx = math.copysign(math.sqrt(max(0.0, 1 + R[0][0] - R[1][1] - R[2][2])) / 2, R[2][1] - R[1][2])
qy = math.copysign(math.sqrt(max(0.0, 1 - R[0][0] + R[1][1] - R[2][2])) / 2, R[0][2] - R[2][0])
qz = math.copysign(math.sqrt(max(0.0, 1 - R[0][0] - R[1][1] + R[2][2])) / 2, R[1][0] - R[0][1])

log = open(log_path, 'w')
tf = subprocess.Popen(
    ['ros2', 'run', 'tf2_ros', 'static_transform_publisher',
     '--x', str(x), '--y', str(y), '--z', str(z),
     '--qx', str(qx), '--qy', str(qy), '--qz', str(qz), '--qw', str(qw),
     '--frame-id', 'earth', '--child-frame-id', 'map'],
    stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
node_proc = subprocess.Popen(
    [exe, '--ros-args', '-r', '__node:=' + NODE, '-p', 'enc_root:=' + enc_root],
    stdout=log, stderr=subprocess.STDOUT)

rclpy.init()
cli = rclpy.create_node('repro_abort_client')
change = cli.create_client(ChangeState, '/' + NODE + '/change_state')
get = cli.create_client(GetDatasets, '/get_datasets')


def call(client, request, timeout=30.0):
    future = client.call_async(request)
    deadline = time.monotonic() + timeout
    while not future.done() and time.monotonic() < deadline:
        rclpy.spin_once(cli, timeout_sec=0.1)
    return future.result() if future.done() else None


def transition(tid, label):
    assert change.wait_for_service(timeout_sec=30.0), 'no change_state service'
    req = ChangeState.Request()
    req.transition.id = tid
    res = call(change, req)
    print(f'{label}: {res.success if res else "no reply"}', flush=True)


def request(lon_min, lon_max, label):
    assert get.wait_for_service(timeout_sec=30.0), 'no get_datasets service'
    req = GetDatasets.Request()
    req.bounds.min_pt.latitude, req.bounds.max_pt.latitude = 43.06, 43.11
    req.bounds.min_pt.longitude, req.bounds.max_pt.longitude = lon_min, lon_max
    req.minimum_scale = float(__import__("os").environ.get("MIN_SCALE", "0.0"))
    res = call(get, req)
    print(f'request {label}: {[d.label for d in res.datasets] if res else "no reply"}', flush=True)


def wait_for(text, timeout=90.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        log.flush()
        if text in open(log_path).read():
            return 'seen'
        if node_proc.poll() is not None:
            return f'node exited with {node_proc.returncode}'
        time.sleep(0.2)
    return 'timeout'


try:
    transition(Transition.TRANSITION_CONFIGURE, 'configure')
    transition(Transition.TRANSITION_ACTIVATE, 'activate')
    request(-70.79, -70.74, 'west')
    print('west:', wait_for('Grid ready: US5PSMCC.000'), flush=True)
    if cycle:
        transition(Transition.TRANSITION_DEACTIVATE, 'deactivate')
        transition(Transition.TRANSITION_CLEANUP, 'cleanup')
        transition(Transition.TRANSITION_CONFIGURE, 'configure')
        transition(Transition.TRANSITION_ACTIVATE, 'activate')
    request(-70.71, -70.66, 'east')
    print('east:', wait_for('Grid ready: US5PSMCD.000'), flush=True)
    time.sleep(1.0)
    print('node alive afterwards:', node_proc.poll() is None, flush=True)
finally:
    for p in (node_proc, tf):
        if p.poll() is None:
            p.terminate()
            try:
                p.wait(timeout=10)
            except subprocess.TimeoutExpired:
                p.kill()
    print('node exit code:', node_proc.returncode, flush=True)
    cli.destroy_node()
    rclpy.shutdown()
