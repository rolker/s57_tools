"""Interlock tests: swap refused while nav is live; probe fails closed."""

import subprocess

from enc_updater import InterlockRefusal
from enc_updater import nav_liveness
from enc_updater.config import NavLivenessConfig
import pytest


class FakeCompleted:
    """Minimal stand-in for subprocess.CompletedProcess."""

    def __init__(self, returncode=0, stdout='', stderr=''):
        """Capture the fields check_nav_down reads."""
        self.returncode = returncode
        self.stdout = stdout
        self.stderr = stderr


def test_swap_refused_when_nav_node_present(monkeypatch):
    """A configured nav node in `ros2 node list` refuses the swap."""
    monkeypatch.setattr(
        nav_liveness.subprocess, 'run',
        lambda *a, **k: FakeCompleted(stdout='/rosout\n/bizzy/controller\n'))
    cfg = NavLivenessConfig(nodes=['/bizzy/controller'])
    with pytest.raises(InterlockRefusal, match='navigation active'):
        nav_liveness.check_nav_down(cfg)


def test_swap_proceeds_when_nav_absent(monkeypatch):
    """No configured node present: check passes silently."""
    monkeypatch.setattr(
        nav_liveness.subprocess, 'run',
        lambda *a, **k: FakeCompleted(stdout='/rosout\n'))
    nav_liveness.check_nav_down(NavLivenessConfig(nodes=['/bizzy/controller']))


def test_probe_error_fails_closed(monkeypatch):
    """A failing probe refuses the swap rather than assuming nav is down."""
    monkeypatch.setattr(
        nav_liveness.subprocess, 'run',
        lambda *a, **k: FakeCompleted(returncode=1, stderr='daemon unreachable'))
    cfg = NavLivenessConfig(nodes=['/bizzy/controller'])
    with pytest.raises(InterlockRefusal, match='fail closed'):
        nav_liveness.check_nav_down(cfg)


def test_probe_timeout_fails_closed(monkeypatch):
    """A hung probe refuses the swap (fail closed)."""
    def timeout_run(*args, **kwargs):
        """Emulate `ros2 node list` hanging past the timeout."""
        raise subprocess.TimeoutExpired(cmd='ros2', timeout=1)
    monkeypatch.setattr(nav_liveness.subprocess, 'run', timeout_run)
    cfg = NavLivenessConfig(nodes=['/bizzy/controller'])
    with pytest.raises(InterlockRefusal, match='fail closed'):
        nav_liveness.check_nav_down(cfg)


def test_empty_node_list_skips_probe(monkeypatch):
    """Interlock unconfigured (no nodes): the probe must not even run."""
    def must_not_run(*args, **kwargs):
        """Trip the test if the probe subprocess is invoked."""
        raise AssertionError('probe must not run with no nodes configured')
    monkeypatch.setattr(nav_liveness.subprocess, 'run', must_not_run)
    nav_liveness.check_nav_down(NavLivenessConfig(nodes=[]))


def test_ros_domain_id_pins_probe_env(monkeypatch):
    """ros_domain_id is exported into the probe env so it can't fail open."""
    seen = {}

    def record_run(argv, **kwargs):
        """Record the env the probe subprocess is given."""
        seen['env'] = kwargs.get('env')
        return FakeCompleted(stdout='')
    monkeypatch.setattr(nav_liveness.subprocess, 'run', record_run)
    cfg = NavLivenessConfig(nodes=['/bizzy/controller'], ros_domain_id=7)
    nav_liveness.check_nav_down(cfg)
    assert seen['env']['ROS_DOMAIN_ID'] == '7'


def test_probe_env_omits_domain_when_unset(monkeypatch):
    """With no ros_domain_id the probe inherits the ambient env unchanged."""
    seen = {}
    monkeypatch.delenv('ROS_DOMAIN_ID', raising=False)

    def record_run(argv, **kwargs):
        """Record the env the probe subprocess is given."""
        seen['env'] = kwargs.get('env')
        return FakeCompleted(stdout='')
    monkeypatch.setattr(nav_liveness.subprocess, 'run', record_run)
    nav_liveness.check_nav_down(NavLivenessConfig(nodes=['/bizzy/controller']))
    assert 'ROS_DOMAIN_ID' not in seen['env']


def test_ros_setup_wraps_probe_in_bash(monkeypatch):
    """With ros_setup configured the probe sources it before `ros2 node list`."""
    seen = {}

    def record_run(argv, **kwargs):
        """Record the argv used for the probe."""
        seen['argv'] = argv
        return FakeCompleted(stdout='')
    monkeypatch.setattr(nav_liveness.subprocess, 'run', record_run)
    cfg = NavLivenessConfig(nodes=['/bizzy/controller'],
                            ros_setup='/opt/ros/jazzy/setup.bash')
    nav_liveness.check_nav_down(cfg)
    assert seen['argv'][0] == 'bash'
    assert '/opt/ros/jazzy/setup.bash' in seen['argv'][2]
