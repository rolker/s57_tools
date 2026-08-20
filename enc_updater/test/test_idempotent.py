"""End-to-end idempotency: an unchanged upstream is a no-op run."""

from enc_updater import __main__ as cli
from enc_updater import downloader
from enc_updater import InterlockRefusal
from enc_updater import regenerator
from enc_updater import registry
from enc_updater.downloader import CatalogEntry
import pytest
import yaml

CATALOG = {
    'US5NH02M': CatalogEntry(
        name='US5NH02M', edition=25, update=3,
        url='https://example.invalid/US5NH02M.zip', size=None),
}
CELLS = {'US5NH02M': {'edition': 25, 'update': 3}}


@pytest.fixture
def workspace(tmp_path, monkeypatch):
    """Config file + corpus/store where corpus, manifest and catalog agree."""
    corpus = tmp_path / 'corpus'
    corpus.mkdir()
    cell_dir = corpus / 'US5NH02M'
    cell_dir.mkdir()
    (cell_dir / 'US5NH02M.000').write_bytes(b'seeded ENC base cell data')
    registry.write_cells(registry.manifest_path(str(corpus)), dict(CELLS))
    store = tmp_path / 'store'
    (store / 'chart').mkdir(parents=True)
    registry.write_cells(registry.active_registry_path(str(store)), dict(CELLS))
    config_path = tmp_path / 'region.yaml'
    config_path.write_text(yaml.safe_dump({
        'corpus_dir': str(corpus),
        'store_dir': str(store),
        'cells': ['US5NH02M'],
    }))
    monkeypatch.setattr(downloader, 'fetch_catalog',
                        lambda url, timeout: dict(CATALOG))
    return config_path, store


def forbid_regeneration(monkeypatch):
    """Fail the test if anything past change detection runs."""
    def must_not_run(*args, **kwargs):
        """Trip the test if regeneration is attempted."""
        raise AssertionError('regeneration must not run on a no-op cycle')
    monkeypatch.setattr(regenerator, 'regenerate', must_not_run)


def test_second_run_is_noop(workspace, monkeypatch):
    """Catalog == manifest == active registry: exit 0, nothing regenerated."""
    config_path, _ = workspace
    forbid_regeneration(monkeypatch)
    assert cli.main(['--config', str(config_path)]) == 0


def test_corpus_ahead_of_layer_triggers_regeneration(workspace, monkeypatch):
    """
    Regenerate when the active layer lags the corpus despite no new download.

    That is exactly the state a prior interlock-refused run leaves behind.
    """
    config_path, store = workspace
    registry.write_cells(registry.active_registry_path(str(store)),
                         {'US5NH02M': {'edition': 24, 'update': 9}})
    ran = {}
    monkeypatch.setattr(regenerator, 'regenerate',
                        lambda cfg, manifest, dry_run=False: ran.update(ok=True))
    assert cli.main(['--config', str(config_path)]) == 0
    assert ran == {'ok': True}


def test_force_overrides_change_detection(workspace, monkeypatch):
    """--force regenerates even when everything matches."""
    config_path, _ = workspace
    ran = {}
    monkeypatch.setattr(regenerator, 'regenerate',
                        lambda cfg, manifest, dry_run=False: ran.update(ok=True))
    assert cli.main(['--config', str(config_path), '--force']) == 0
    assert ran == {'ok': True}


def test_interlock_refusal_exits_2(workspace, monkeypatch):
    """An interlock refusal is a distinct exit code for cron/monitoring."""
    config_path, _ = workspace

    def refuse(cfg, manifest, dry_run=False):
        """Emulate the nav-down interlock refusing the swap."""
        raise InterlockRefusal('interlock: navigation active')
    monkeypatch.setattr(regenerator, 'regenerate', refuse)
    assert cli.main(['--config', str(config_path), '--force']) == 2


def test_fresh_store_dir_created_up_front(workspace, monkeypatch, tmp_path):
    """
    A missing store_dir is created before any work (#39).

    The old behavior failed at the final commit, after the whole
    download/export cycle had already run.
    """
    import os
    import shutil
    config_path, store = workspace
    shutil.rmtree(store)

    def fake_regenerate(cfg, manifest, dry_run=False):
        """Stand-in swap: the store dir must already exist by now."""
        assert os.path.isdir(cfg.store_dir)
    monkeypatch.setattr(regenerator, 'regenerate', fake_regenerate)
    assert cli.main(['--config', str(config_path)]) == 0
    assert os.path.isdir(store)


def test_corrupt_manifest_is_clean_exit_1(workspace, monkeypatch, capsys):
    """A corrupt .manifest.json exits 1 with a message, never a traceback."""
    config_path, store = workspace
    corpus = store.parent / 'corpus'
    registry.manifest_path(str(corpus))
    (corpus / '.manifest.json').write_text('{not json')
    assert cli.main(['--config', str(config_path)]) == 1
    assert 'enc_updater:' in capsys.readouterr().err


def test_missing_store_parent_is_clean_refusal(workspace, tmp_path):
    """
    Only the store leaf is created; a missing parent refuses cleanly.

    An unmounted data volume or mistyped store_dir must fail loudly, not
    silently fork the pipeline onto a shadow store (makedirs would).
    """
    config_path, store = workspace
    import shutil
    import yaml as yaml_mod
    doc = yaml_mod.safe_load(config_path.read_text())
    doc['store_dir'] = str(tmp_path / 'not-mounted' / 'store')
    config_path.write_text(yaml_mod.safe_dump(doc))
    shutil.rmtree(store)
    assert cli.main(['--config', str(config_path)]) == 1
