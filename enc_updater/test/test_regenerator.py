"""Simulated-failure tests: every regeneration failure leaves the layer intact."""

import fcntl
import os

from enc_updater import regenerator
from enc_updater import registry
from enc_updater import UpdaterError
from enc_updater.config import UpdaterConfig
import pytest

MANIFEST = {'US5NH02M': {'edition': 25, 'update': 3}}

EXPORT_LINE = ('exported {path} (1024x768, 51234 cells, scale 1:20000, '
               'GGGS level 12 -> import_geotiff --level 12)')


def make_store(tmp_path):
    """Build a fake live store with an existing chart layer to protect."""
    store = tmp_path / 'store'
    chart = store / 'chart'
    chart.mkdir(parents=True)
    (chart / '12_100_200.tif').write_bytes(b'previous chart tile')
    registry.write_cells(str(chart / 'editions.json'),
                         {'US5NH02M': {'edition': 24, 'update': 9}})
    return store


def chart_snapshot(store):
    """Names + contents of the live chart layer for intactness assertions."""
    chart = store / 'chart'
    return {p.name: p.read_bytes() for p in chart.iterdir()}


def make_config(tmp_path, store):
    """Build an UpdaterConfig with tool resolution stubbed out via overrides."""
    fake_tool = tmp_path / 'fake-tool'
    fake_tool.write_text('#!/bin/sh\nexit 42\n')
    fake_tool.chmod(0o755)
    corpus = tmp_path / 'corpus'
    corpus.mkdir()
    return UpdaterConfig(
        corpus_dir=str(corpus),
        store_dir=str(store),
        cells=['US5NH02M'],
        s57_to_geotiff_bin=str(fake_tool),
        import_geotiff_bin=str(fake_tool),
    )


class FakeRunner:
    """Substitute for regenerator.run_cmd that scripts each phase's outcome."""

    def __init__(self, export_log='', fail_phase=None):
        """Fail `fail_phase` (if set); otherwise emulate export/stage/commit."""
        self.export_log = export_log
        self.fail_phase = fail_phase
        self.calls = []

    def __call__(self, argv, timeout, phase):
        """Record the call, then fail or emulate the phase's side effects."""
        self.calls.append((phase, list(argv)))
        if phase == self.fail_phase:
            raise UpdaterError(f'{phase}: simulated failure')
        if phase == 'export':
            return self.export_log
        if phase == 'stage':
            staged_chart = os.path.join(argv[argv.index('--stage') + 1], 'chart')
            os.makedirs(staged_chart, exist_ok=True)
            with open(os.path.join(staged_chart, '12_0_0.tif'), 'wb') as f:
                f.write(b'staged tile')
        return ''

    def phases(self):
        """Return the phase names invoked, in order."""
        return [phase for phase, _ in self.calls]


def no_leftover_workdirs(store):
    """Assert staging/export work dirs were cleaned from the store parent."""
    parent = store.parent
    leftovers = [p.name for p in parent.iterdir()
                 if p.name.startswith(('.enc_updater_staging', '.enc_updater_export'))]
    assert leftovers == []


@pytest.fixture(autouse=True)
def plausible_tiles(monkeypatch):
    """Give the band-1 spot check a plausible coastal range by default."""
    monkeypatch.setattr(regenerator, '_band1_min_max', lambda path: (-50.0, 5.0))


def test_export_failure_leaves_layer_intact(tmp_path, monkeypatch):
    """A failed s57_to_geotiff run must not touch the live layer."""
    store = make_store(tmp_path)
    before = chart_snapshot(store)
    runner = FakeRunner(fail_phase='export')
    monkeypatch.setattr(regenerator, 'run_cmd', runner)
    with pytest.raises(UpdaterError, match='export: simulated failure'):
        regenerator.regenerate(make_config(tmp_path, store), MANIFEST)
    assert chart_snapshot(store) == before
    assert 'commit' not in runner.phases()
    no_leftover_workdirs(store)


def test_empty_export_refuses_to_swap(tmp_path, monkeypatch):
    """An export producing no cells must never reach staging or commit."""
    store = make_store(tmp_path)
    before = chart_snapshot(store)
    runner = FakeRunner(export_log='exported 0 of 0 cell(s)\n')
    monkeypatch.setattr(regenerator, 'run_cmd', runner)
    with pytest.raises(UpdaterError, match='no cells exported'):
        regenerator.regenerate(make_config(tmp_path, store), MANIFEST)
    assert chart_snapshot(store) == before
    assert runner.phases() == ['export']
    no_leftover_workdirs(store)


def test_stage_failure_leaves_layer_intact(tmp_path, monkeypatch):
    """A failed import_geotiff --stage must not touch the live layer."""
    store = make_store(tmp_path)
    before = chart_snapshot(store)
    runner = FakeRunner(export_log=EXPORT_LINE.format(path='/x/US5NH02M.tif') + '\n',
                        fail_phase='stage')
    monkeypatch.setattr(regenerator, 'run_cmd', runner)
    with pytest.raises(UpdaterError, match='stage: simulated failure'):
        regenerator.regenerate(make_config(tmp_path, store), MANIFEST)
    assert chart_snapshot(store) == before
    assert 'commit' not in runner.phases()
    no_leftover_workdirs(store)


def test_sanity_failure_blocks_commit(tmp_path, monkeypatch):
    """Implausible band-1 values must block the swap (corrupt regeneration)."""
    store = make_store(tmp_path)
    before = chart_snapshot(store)
    runner = FakeRunner(export_log=EXPORT_LINE.format(path='/x/US5NH02M.tif') + '\n')
    monkeypatch.setattr(regenerator, 'run_cmd', runner)
    monkeypatch.setattr(regenerator, '_band1_min_max',
                        lambda path: (-50.0, 4242.0))
    with pytest.raises(UpdaterError, match='outside plausible'):
        regenerator.regenerate(make_config(tmp_path, store), MANIFEST)
    assert chart_snapshot(store) == before
    assert 'commit' not in runner.phases()
    no_leftover_workdirs(store)


def test_commit_failure_leaves_layer_intact(tmp_path, monkeypatch):
    """A failed import_geotiff --commit surfaces; the prior layer stands."""
    store = make_store(tmp_path)
    before = chart_snapshot(store)
    runner = FakeRunner(export_log=EXPORT_LINE.format(path='/x/US5NH02M.tif') + '\n',
                        fail_phase='commit')
    monkeypatch.setattr(regenerator, 'run_cmd', runner)
    with pytest.raises(UpdaterError, match='commit: simulated failure'):
        regenerator.regenerate(make_config(tmp_path, store), MANIFEST)
    assert chart_snapshot(store) == before
    no_leftover_workdirs(store)


def test_success_stages_registry_and_commits(tmp_path, monkeypatch):
    """Happy path: per-cell --level passed, registry staged, commit runs."""
    store = make_store(tmp_path)
    staged_registry = {}
    runner = FakeRunner(export_log=EXPORT_LINE.format(path='/x/US5NH02M.tif') + '\n')

    def capture_commit(argv, timeout, phase):
        """Capture the staged registry at commit time, before cleanup."""
        if phase == 'commit':
            staged_dir = argv[argv.index('--commit') + 1]
            staged_registry.update(registry.load_cells(
                registry.staged_registry_path(os.path.join(staged_dir, 'chart'))))
        return runner(argv, timeout, phase)

    monkeypatch.setattr(regenerator, 'run_cmd', capture_commit)
    regenerator.regenerate(make_config(tmp_path, store), MANIFEST)
    assert runner.phases() == ['export', 'stage', 'commit']
    stage_argv = runner.calls[1][1]
    assert stage_argv[stage_argv.index('--level') + 1] == '12'
    assert staged_registry == MANIFEST
    no_leftover_workdirs(store)


def test_dry_run_stops_before_interlock_and_commit(tmp_path, monkeypatch):
    """--dry-run validates the staged layer but never probes or commits."""
    store = make_store(tmp_path)
    before = chart_snapshot(store)
    runner = FakeRunner(export_log=EXPORT_LINE.format(path='/x/US5NH02M.tif') + '\n')
    monkeypatch.setattr(regenerator, 'run_cmd', runner)
    monkeypatch.setattr(
        regenerator.nav_liveness, 'check_nav_down',
        lambda cfg: (_ for _ in ()).throw(AssertionError('probe must not run')))
    regenerator.regenerate(make_config(tmp_path, store), MANIFEST, dry_run=True)
    assert runner.phases() == ['export', 'stage']
    assert chart_snapshot(store) == before
    no_leftover_workdirs(store)


def test_overlapping_run_refuses(tmp_path, monkeypatch):
    """A second run while the store lock is held refuses rather than racing."""
    store = make_store(tmp_path)
    before = chart_snapshot(store)
    runner = FakeRunner(export_log=EXPORT_LINE.format(path='/x/US5NH02M.tif') + '\n')
    monkeypatch.setattr(regenerator, 'run_cmd', runner)
    lock_path = os.path.join(str(store.parent), '.enc_updater.lock')
    fd = os.open(lock_path, os.O_CREAT | os.O_RDWR, 0o644)
    fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
    try:
        with pytest.raises(UpdaterError, match='runs must not overlap'):
            regenerator.regenerate(make_config(tmp_path, store), MANIFEST)
        assert runner.phases() == []  # refused before touching the pipeline
    finally:
        os.close(fd)
    assert chart_snapshot(store) == before  # live layer untouched
    no_leftover_workdirs(store)


def test_parse_export_log_skips_warnings():
    """Only `exported <cell>` lines yield (path, level); summaries do not."""
    log_text = '\n'.join([
        'warning: US4NH01M: no in-datum data (all pixels no-data); wrote empty /x/US4NH01M.tif',
        EXPORT_LINE.format(path='/x/US5NH02M.tif'),
        'exported 1 of 2 cell(s)',
    ])
    assert regenerator.parse_export_log(log_text) == [('/x/US5NH02M.tif', 12)]
