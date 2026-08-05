"""Simulated-failure tests: a failed download leaves corpus + manifest intact."""

import io
import json
import os
import zipfile

from enc_updater import downloader
from enc_updater import registry
from enc_updater import UpdaterError
from enc_updater.config import UpdaterConfig
import pytest

CATALOG_XML = b"""<?xml version="1.0" encoding="UTF-8" ?>
<EncProductCatalog>
  <cell>
    <name>US5NH02M</name>
    <status>Active</status>
    <zipfile_location>https://www.charts.noaa.gov/ENCs/US5NH02M.zip</zipfile_location>
    <zipfile_size>SIZE_A</zipfile_size>
    <edtn>25</edtn>
    <updn>3</updn>
  </cell>
  <cell>
    <name>US4NH01M</name>
    <status>Active</status>
    <zipfile_location>https://www.charts.noaa.gov/ENCs/US4NH01M.zip</zipfile_location>
    <zipfile_size>SIZE_B</zipfile_size>
    <edtn>12</edtn>
    <updn>0</updn>
  </cell>
</EncProductCatalog>
"""


def make_cell_zip(cell):
    """Build an in-memory NOAA-style cell zip (ENC_ROOT/<cell>/<cell>.000)."""
    buffer = io.BytesIO()
    with zipfile.ZipFile(buffer, 'w') as zf:
        zf.writestr(f'ENC_ROOT/{cell}/{cell}.000', b'fake ENC base cell data')
        zf.writestr(f'ENC_ROOT/{cell}/{cell}.001', b'fake ENC update data')
    return buffer.getvalue()


def make_config(tmp_path, cells):
    """Build a minimal UpdaterConfig over tmp_path."""
    return UpdaterConfig(
        corpus_dir=str(tmp_path / 'corpus'),
        store_dir=str(tmp_path / 'store'),
        cells=cells,
        catalog_url='https://example.invalid/catalog.xml',
    )


def snapshot(corpus_dir):
    """Corpus dir listing + manifest content, for before/after comparison."""
    listing = sorted(os.listdir(corpus_dir)) if os.path.isdir(corpus_dir) else []
    manifest_file = registry.manifest_path(corpus_dir)
    manifest = None
    if os.path.exists(manifest_file):
        with open(manifest_file, encoding='utf-8') as f:
            manifest = json.load(f)['cells']
    return listing, manifest


def serve(monkeypatch, catalog, zips):
    """Point downloader._open_url at canned catalog/zip responses."""
    def fake_open_url(url, timeout):
        """Serve the canned response for url, or raise like a network error."""
        if url.endswith('catalog.xml'):
            return io.BytesIO(catalog)
        for cell, payload in zips.items():
            if url.endswith(cell + '.zip'):
                if isinstance(payload, Exception):
                    raise payload
                return io.BytesIO(payload)
        raise OSError(f'unexpected URL {url}')
    monkeypatch.setattr(downloader, '_open_url', fake_open_url)


def catalog_bytes(zip_a, zip_b, size_a=None, size_b=None):
    """CATALOG_XML with the size placeholders filled in."""
    xml = CATALOG_XML
    xml = xml.replace(b'SIZE_A', str(size_a if size_a is not None else len(zip_a)).encode())
    xml = xml.replace(b'SIZE_B', str(size_b if size_b is not None else len(zip_b)).encode())
    return xml


def test_successful_update_populates_corpus_and_manifest(tmp_path, monkeypatch):
    """Happy path: both cells install and the manifest records their editions."""
    zip_a, zip_b = make_cell_zip('US5NH02M'), make_cell_zip('US4NH01M')
    serve(monkeypatch, catalog_bytes(zip_a, zip_b),
          {'US5NH02M': zip_a, 'US4NH01M': zip_b})
    cfg = make_config(tmp_path, ['US5NH02M', 'US4NH01M'])
    changed, manifest = downloader.update_corpus(cfg)
    assert sorted(changed) == ['US4NH01M', 'US5NH02M']
    assert manifest['US5NH02M'] == {'edition': 25, 'update': 3}
    assert os.path.isfile(os.path.join(cfg.corpus_dir, 'US5NH02M', 'US5NH02M.000'))
    _, saved = snapshot(cfg.corpus_dir)
    assert saved == manifest


def test_no_change_is_noop(tmp_path, monkeypatch):
    """Manifest already matching the catalog downloads nothing."""
    zip_a, zip_b = make_cell_zip('US5NH02M'), make_cell_zip('US4NH01M')
    serve(monkeypatch, catalog_bytes(zip_a, zip_b),
          {'US5NH02M': OSError('must not download'),
           'US4NH01M': OSError('must not download')})
    cfg = make_config(tmp_path, ['US5NH02M', 'US4NH01M'])
    os.makedirs(cfg.corpus_dir)
    registry.write_cells(registry.manifest_path(cfg.corpus_dir), {
        'US5NH02M': {'edition': 25, 'update': 3},
        'US4NH01M': {'edition': 12, 'update': 0},
    })
    changed, _ = downloader.update_corpus(cfg)
    assert changed == []


def test_failed_download_leaves_corpus_untouched(tmp_path, monkeypatch):
    """A mid-fetch network failure changes nothing on disk."""
    zip_a, zip_b = make_cell_zip('US5NH02M'), make_cell_zip('US4NH01M')
    serve(monkeypatch, catalog_bytes(zip_a, zip_b),
          {'US5NH02M': OSError('connection reset')})
    cfg = make_config(tmp_path, ['US5NH02M'])
    os.makedirs(cfg.corpus_dir)
    before = snapshot(cfg.corpus_dir)
    with pytest.raises(UpdaterError, match='fetch failed'):
        downloader.update_corpus(cfg)
    assert snapshot(cfg.corpus_dir) == before


def test_size_mismatch_leaves_corpus_untouched(tmp_path, monkeypatch):
    """A truncated download (size != catalog zipfile_size) is rejected."""
    zip_a, zip_b = make_cell_zip('US5NH02M'), make_cell_zip('US4NH01M')
    serve(monkeypatch, catalog_bytes(zip_a, zip_b, size_a=len(zip_a) + 100),
          {'US5NH02M': zip_a})
    cfg = make_config(tmp_path, ['US5NH02M'])
    os.makedirs(cfg.corpus_dir)
    before = snapshot(cfg.corpus_dir)
    with pytest.raises(UpdaterError, match='size mismatch'):
        downloader.update_corpus(cfg)
    assert snapshot(cfg.corpus_dir) == before


def test_corrupt_zip_leaves_corpus_untouched(tmp_path, monkeypatch):
    """Bytes that are not a valid zip are rejected before extraction."""
    garbage = b'this is not a zip file, not even close'
    zip_b = make_cell_zip('US4NH01M')
    serve(monkeypatch, catalog_bytes(garbage, zip_b), {'US5NH02M': garbage})
    cfg = make_config(tmp_path, ['US5NH02M'])
    os.makedirs(cfg.corpus_dir)
    before = snapshot(cfg.corpus_dir)
    with pytest.raises(UpdaterError, match='not a valid zip'):
        downloader.update_corpus(cfg)
    assert snapshot(cfg.corpus_dir) == before


def test_zip_missing_cell_data_rejected(tmp_path, monkeypatch):
    """A valid zip without <cell>/<cell>.000 must not install."""
    buffer = io.BytesIO()
    with zipfile.ZipFile(buffer, 'w') as zf:
        zf.writestr('README.txt', b'wrong content')
    bad = buffer.getvalue()
    zip_b = make_cell_zip('US4NH01M')
    serve(monkeypatch, catalog_bytes(bad, zip_b), {'US5NH02M': bad})
    cfg = make_config(tmp_path, ['US5NH02M'])
    os.makedirs(cfg.corpus_dir)
    before = snapshot(cfg.corpus_dir)
    with pytest.raises(UpdaterError, match='does not contain'):
        downloader.update_corpus(cfg)
    assert snapshot(cfg.corpus_dir) == before


def test_install_double_failure_names_backup(tmp_path, monkeypatch):
    """If install and rollback both fail, the error names the preserved backup."""
    corpus = tmp_path / 'corpus'
    corpus.mkdir()
    target = corpus / 'US5NH02M'
    target.mkdir()
    (target / 'US5NH02M.000').write_text('old cell data')
    new_dir = corpus / '.download.new'
    new_dir.mkdir()
    (new_dir / 'US5NH02M.000').write_text('new cell data')

    real_rename = os.rename
    calls = {'n': 0}

    def flaky_rename(src, dst):
        """Let the first rename (target -> backup) run; fail the rest."""
        calls['n'] += 1
        if calls['n'] == 1:
            return real_rename(src, dst)
        raise OSError('disk gone')

    monkeypatch.setattr(downloader.os, 'rename', flaky_rename)
    with pytest.raises(UpdaterError, match='preserved at') as excinfo:
        downloader._install_cell(str(corpus), 'US5NH02M', str(new_dir))
    assert '.old.US5NH02M.' in str(excinfo.value)


def test_configured_cell_missing_from_catalog_errors(tmp_path, monkeypatch):
    """A configured cell absent from the catalog needs a human, not silence."""
    zip_a, zip_b = make_cell_zip('US5NH02M'), make_cell_zip('US4NH01M')
    serve(monkeypatch, catalog_bytes(zip_a, zip_b), {})
    cfg = make_config(tmp_path, ['US5ZZ99M'])
    with pytest.raises(UpdaterError, match='not in catalog'):
        downloader.update_corpus(cfg)


def test_update_replaces_previous_cell_edition(tmp_path, monkeypatch):
    """A newer edition replaces the old cell dir; old files do not linger."""
    zip_a, zip_b = make_cell_zip('US5NH02M'), make_cell_zip('US4NH01M')
    serve(monkeypatch, catalog_bytes(zip_a, zip_b), {'US5NH02M': zip_a})
    cfg = make_config(tmp_path, ['US5NH02M'])
    stale_dir = os.path.join(cfg.corpus_dir, 'US5NH02M')
    os.makedirs(stale_dir)
    stale_file = os.path.join(stale_dir, 'US5NH02M.OLD')
    with open(stale_file, 'w', encoding='utf-8') as f:
        f.write('stale edition leftover')
    registry.write_cells(registry.manifest_path(cfg.corpus_dir),
                         {'US5NH02M': {'edition': 24, 'update': 9}})
    changed, manifest = downloader.update_corpus(cfg)
    assert changed == ['US5NH02M']
    assert manifest['US5NH02M'] == {'edition': 25, 'update': 3}
    assert not os.path.exists(stale_file)
    assert os.path.isfile(os.path.join(stale_dir, 'US5NH02M.000'))
