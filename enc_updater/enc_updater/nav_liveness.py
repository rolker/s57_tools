"""
Nav-down interlock: ROS graph probe.

The updater is offline tooling, but the one thing it must never do is swap
the chart layer out from under a running navigation stack (ADR-0010 D7).
At swap time only, it probes the ROS graph with ``ros2 node list`` and
refuses to commit if any configured nav-stack node is present.

Fail-closed contract: with nodes configured, *any* probe failure (missing
``ros2``, source failure, timeout, non-zero exit) refuses the swap — an
indeterminate answer is treated as "nav may be up". An empty/omitted node
list means the interlock is not configured for this host (documented in the
README; intended for dev machines with no navigation stack) and the probe is
skipped.
"""

import subprocess
from typing import List

from . import InterlockRefusal
from .config import NavLivenessConfig


def _probe_node_list(cfg: NavLivenessConfig) -> List[str]:
    """Run ``ros2 node list`` (optionally under a sourced ROS env)."""
    if cfg.ros_setup:
        argv = ['bash', '-c',
                f'source "{cfg.ros_setup}" >/dev/null 2>&1 && ros2 node list']
    else:
        argv = ['ros2', 'node', 'list']
    try:
        result = subprocess.run(
            argv, capture_output=True, text=True, timeout=cfg.timeout)
    except (OSError, subprocess.TimeoutExpired) as e:
        raise InterlockRefusal(
            f'interlock: nav-liveness probe failed ({e}) — refusing to swap '
            '(fail closed)')
    if result.returncode != 0:
        raise InterlockRefusal(
            'interlock: nav-liveness probe exited '
            f'{result.returncode} ({result.stderr.strip()}) — refusing to swap '
            '(fail closed)')
    return [line.strip() for line in result.stdout.splitlines() if line.strip()]


def check_nav_down(cfg: NavLivenessConfig) -> None:
    """Raise InterlockRefusal if navigation is (or may be) active."""
    if not cfg.nodes:
        print('enc_updater: nav-liveness interlock not configured '
              '(nav_liveness.nodes is empty) — skipping probe')
        return
    live = _probe_node_list(cfg)
    present = sorted(set(cfg.nodes) & set(live))
    if present:
        raise InterlockRefusal(
            f'interlock: navigation active (nodes present: {present}) — '
            'refusing to swap the chart layer')
