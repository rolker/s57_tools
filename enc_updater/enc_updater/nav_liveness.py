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

Fail-open caveat: fail-closed covers probe *errors*, not a probe that
succeeds but queries the wrong DDS graph. If the probe environment's
``ROS_DOMAIN_ID`` differs from the live nav stack's, ``ros2 node list``
returns an empty list and the swap proceeds while nav is active. Pin the
domain with ``nav_liveness.ros_domain_id`` (set here on the probe's env) and
keep ``RMW_IMPLEMENTATION`` aligned — see the README nav-liveness contract.
"""

import os
import subprocess
from typing import List

from . import InterlockRefusal
from .config import NavLivenessConfig


def _probe_node_list(cfg: NavLivenessConfig) -> List[str]:
    """Run ``ros2 node list`` (optionally under a sourced ROS env)."""
    if cfg.ros_setup:
        # Pass the setup path as a positional arg ($1) rather than
        # interpolating it into the script, so an unusual path can't be
        # word-split or mis-executed by the shell.
        argv = ['bash', '-c',
                'source "$1" >/dev/null 2>&1 && ros2 node list',
                'enc_updater-probe', cfg.ros_setup]
    else:
        argv = ['ros2', 'node', 'list']
    env = os.environ.copy()
    if cfg.ros_domain_id is not None:
        # Pin the probe to the nav stack's DDS domain so a mismatched cron
        # ROS_DOMAIN_ID can't query an empty graph and let the swap fail open.
        env['ROS_DOMAIN_ID'] = str(cfg.ros_domain_id)
    try:
        result = subprocess.run(
            argv, capture_output=True, text=True, timeout=cfg.timeout, env=env)
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
    if cfg.ros_domain_id is None:
        # Nodes are configured but the DDS domain is not pinned: a cron/probe
        # ROS_DOMAIN_ID that differs from the nav stack's would query an empty
        # graph and let the swap fail *open*. Warn loudly rather than silently
        # trusting the ambient domain (see the README nav-liveness contract).
        print('enc_updater: WARNING nav_liveness.nodes is set but '
              'nav_liveness.ros_domain_id is not — the probe will use the '
              'ambient ROS_DOMAIN_ID; if it differs from the nav stack the '
              'interlock can fail OPEN. Pin ros_domain_id to the nav domain.')
    live = _probe_node_list(cfg)
    present = sorted(set(cfg.nodes) & set(live))
    if present:
        raise InterlockRefusal(
            f'interlock: navigation active (nodes present: {present}) — '
            'refusing to swap the chart layer')
