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


def seed_corpus_cell(corpus_dir, cell):
    """Create an installed-looking corpus cell dir (<cell>/<cell>.000)."""
    cell_dir = os.path.join(corpus_dir, cell)
    os.makedirs(cell_dir, exist_ok=True)
    with open(os.path.join(cell_dir, cell + '.000'), 'wb') as f:
        f.write(b'seeded ENC base cell data')


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
    for cell in ('US5NH02M', 'US4NH01M'):
        seed_corpus_cell(cfg.corpus_dir, cell)
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


def test_missing_zipfile_size_logs_and_installs(tmp_path, monkeypatch, capsys):
    """A cell lacking zipfile_size still installs (CRC), and the skip is logged."""
    zip_a, zip_b = make_cell_zip('US5NH02M'), make_cell_zip('US4NH01M')
    serve(monkeypatch, catalog_bytes(zip_a, zip_b, size_a=''),
          {'US5NH02M': zip_a})
    cfg = make_config(tmp_path, ['US5NH02M'])
    changed, _ = downloader.update_corpus(cfg)
    assert changed == ['US5NH02M']
    assert 'no zipfile_size' in capsys.readouterr().out
    assert os.path.isfile(os.path.join(cfg.corpus_dir, 'US5NH02M', 'US5NH02M.000'))


def test_oversized_catalog_rejected(tmp_path, monkeypatch):
    """A catalog larger than the cap is refused before parsing."""
    monkeypatch.setattr(downloader, '_MAX_CATALOG_BYTES', 8)
    zip_a, zip_b = make_cell_zip('US5NH02M'), make_cell_zip('US4NH01M')
    serve(monkeypatch, catalog_bytes(zip_a, zip_b), {})
    cfg = make_config(tmp_path, ['US5NH02M'])
    with pytest.raises(UpdaterError, match='exceeds'):
        downloader.update_corpus(cfg)


def test_catalog_with_dtd_rejected(tmp_path, monkeypatch):
    """A catalog carrying a DTD/entity declaration is refused (billion-laughs)."""
    evil = (b'<?xml version="1.0"?>\n'
            b'<!DOCTYPE lolz [<!ENTITY lol "lol">]>\n'
            b'<EncProductCatalog></EncProductCatalog>\n')
    serve(monkeypatch, evil, {})
    cfg = make_config(tmp_path, ['US5NH02M'])
    with pytest.raises(UpdaterError, match='entity-expansion guard'):
        downloader.update_corpus(cfg)


def test_zip_bomb_extract_rejected(tmp_path, monkeypatch):
    """A zip whose uncompressed size exceeds the cap never extracts."""
    monkeypatch.setattr(downloader, '_MAX_ZIP_UNCOMPRESSED_BYTES', 4)
    zip_a, zip_b = make_cell_zip('US5NH02M'), make_cell_zip('US4NH01M')
    serve(monkeypatch, catalog_bytes(zip_a, zip_b), {'US5NH02M': zip_a})
    cfg = make_config(tmp_path, ['US5NH02M'])
    os.makedirs(cfg.corpus_dir)
    before = snapshot(cfg.corpus_dir)
    with pytest.raises(UpdaterError, match='zip bomb'):
        downloader.update_corpus(cfg)
    assert snapshot(cfg.corpus_dir) == before


@pytest.mark.parametrize('bad_url', [
    'file:///etc/passwd',
    'ftp://internal.host/secret',
    'gopher://169.254.169.254/',
    '/etc/passwd',
])
def test_open_url_rejects_non_http_scheme(bad_url):
    """A spoofed catalog URL with a non-http(s) scheme is refused (SSRF/LFI guard)."""
    with pytest.raises(UpdaterError, match='refusing non-http'):
        downloader._open_url(bad_url, timeout=1)


@pytest.mark.parametrize('good_url', [
    'https://charts.noaa.gov/ENCs/US5NH02M.zip',
    'http://charts.noaa.gov/ENCs/US5NH02M.zip',
])
def test_open_url_allows_http_schemes(good_url, monkeypatch):
    """http/https pass the scheme gate through to urlopen."""
    seen = {}
    monkeypatch.setattr(downloader.urllib.request, 'urlopen',
                        lambda url, timeout: seen.setdefault('url', url))
    downloader._open_url(good_url, timeout=1)
    assert seen['url'] == good_url


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


REGION_CATALOG_XML = b"""<?xml version="1.0" encoding="UTF-8" ?>
<EncProductCatalog>
  <cell>
    <name>US5INSIDE</name>
    <status>Active</status>
    <zipfile_location>https://www.charts.noaa.gov/ENCs/US5INSIDE.zip</zipfile_location>
    <edtn>2</edtn>
    <updn>0</updn>
    <cov><panel><panel_no>1</panel_no><type>E</type>
      <vertex><lat>42.95</lat><long>-70.70</long></vertex>
      <vertex><lat>42.95</lat><long>-70.60</long></vertex>
      <vertex><lat>43.05</lat><long>-70.60</long></vertex>
      <vertex><lat>43.05</lat><long>-70.70</long></vertex>
    </panel></cov>
  </cell>
  <cell>
    <name>US5OUTSIDE</name>
    <status>Active</status>
    <zipfile_location>https://www.charts.noaa.gov/ENCs/US5OUTSIDE.zip</zipfile_location>
    <edtn>1</edtn>
    <updn>0</updn>
    <cov><panel><panel_no>1</panel_no><type>E</type>
      <vertex><lat>44.95</lat><long>-68.70</long></vertex>
      <vertex><lat>44.95</lat><long>-68.60</long></vertex>
      <vertex><lat>45.05</lat><long>-68.60</long></vertex>
    </panel></cov>
  </cell>
  <cell>
    <name>US5HOLLOW</name>
    <status>Active</status>
    <zipfile_location>https://www.charts.noaa.gov/ENCs/US5HOLLOW.zip</zipfile_location>
    <edtn>1</edtn>
    <updn>0</updn>
    <cov><panel><panel_no>1</panel_no><type>I</type>
      <vertex><lat>42.95</lat><long>-70.70</long></vertex>
      <vertex><lat>42.95</lat><long>-70.60</long></vertex>
      <vertex><lat>43.05</lat><long>-70.60</long></vertex>
    </panel></cov>
  </cell>
</EncProductCatalog>
"""


def test_catalog_parse_carries_status_and_exterior_panels(tmp_path, monkeypatch):
    """fetch_catalog surfaces status + type-E panels as (lon, lat) polygons."""
    serve(monkeypatch, REGION_CATALOG_XML, {})
    catalog = downloader.fetch_catalog('https://example.invalid/catalog.xml', 5.0)
    inside = catalog['US5INSIDE']
    assert inside.status == 'Active'
    assert inside.panels == [[(-70.70, 42.95), (-70.60, 42.95),
                              (-70.60, 43.05), (-70.70, 43.05)]]
    # The type-I hole panel is not a selection polygon.
    assert catalog['US5HOLLOW'].panels == []


def region_config(tmp_path):
    """Build a region-mode UpdaterConfig over tmp_path (Shoals-shaped bbox)."""
    return UpdaterConfig(
        corpus_dir=str(tmp_path / 'corpus'),
        store_dir=str(tmp_path / 'store'),
        region=[(-70.85, 42.93), (-70.55, 42.93),
                (-70.55, 43.11), (-70.85, 43.11)],
        catalog_url='https://example.invalid/catalog.xml',
    )


def test_region_mode_selects_and_downloads_only_covering_cells(tmp_path, monkeypatch):
    """update_corpus in region mode fetches exactly the cells covering the region."""
    zip_inside = make_cell_zip('US5INSIDE')
    serve(monkeypatch, REGION_CATALOG_XML, {
        'US5INSIDE': zip_inside,
        'US5OUTSIDE': OSError('must not download'),
        'US5HOLLOW': OSError('must not download'),
    })
    cfg = region_config(tmp_path)
    changed, manifest = downloader.update_corpus(cfg)
    assert changed == ['US5INSIDE']
    assert sorted(manifest) == ['US5INSIDE']


def test_prune_removes_deselected_cell_from_corpus_and_manifest(tmp_path, monkeypatch):
    """
    A cell that leaves the selection leaves the corpus.

    Otherwise the whole-corpus export keeps feeding its stale tiles into
    every future chart layer.
    """
    zip_inside = make_cell_zip('US5INSIDE')
    serve(monkeypatch, REGION_CATALOG_XML, {'US5INSIDE': zip_inside})
    cfg = region_config(tmp_path)
    # Seed the corpus as if a pre-rescheme run had installed a now-gone cell.
    stale_dir = os.path.join(cfg.corpus_dir, 'US5LEGACY')
    os.makedirs(stale_dir)
    with open(os.path.join(stale_dir, 'US5LEGACY.000'), 'wb') as f:
        f.write(b'stale')
    registry.write_cells(registry.manifest_path(cfg.corpus_dir),
                         {'US5LEGACY': {'edition': 9, 'update': 9}})
    _, manifest = downloader.update_corpus(cfg)
    assert 'US5LEGACY' not in manifest
    assert not os.path.exists(stale_dir)
    _, saved = snapshot(cfg.corpus_dir)
    assert sorted(saved) == ['US5INSIDE']


def test_degraded_catalog_row_refuses_prune(tmp_path, monkeypatch):
    """
    An installed cell whose row is Active with unparseable coverage errors.

    Pruning on that signature would turn a transient catalog defect into
    removed navigation coverage with a successful exit.
    """
    zip_inside = make_cell_zip('US5INSIDE')
    serve(monkeypatch, REGION_CATALOG_XML, {'US5INSIDE': zip_inside})
    cfg = region_config(tmp_path)
    # US5HOLLOW's only panel is type-I, so it parses to zero usable panels
    # while its status stays Active — the degraded-row signature. Install it.
    os.makedirs(cfg.corpus_dir)
    seed_corpus_cell(cfg.corpus_dir, 'US5HOLLOW')
    registry.write_cells(registry.manifest_path(cfg.corpus_dir),
                         {'US5HOLLOW': {'edition': 1, 'update': 0}})
    before = snapshot(cfg.corpus_dir)
    with pytest.raises(UpdaterError, match='refusing to prune'):
        downloader.update_corpus(cfg)
    assert snapshot(cfg.corpus_dir) == before


def test_cells_mode_typo_fails_before_pruning(tmp_path, monkeypatch):
    """
    A config typo errors with the previous corpus and manifest intact.

    Prune runs after validation + downloads, so swapping a good cell name
    for a bad one must not destroy the good cell's data first.
    """
    serve(monkeypatch, REGION_CATALOG_XML, {})
    cfg = make_config(tmp_path, ['US5TYPO'])
    os.makedirs(cfg.corpus_dir)
    seed_corpus_cell(cfg.corpus_dir, 'US5INSIDE')
    registry.write_cells(registry.manifest_path(cfg.corpus_dir),
                         {'US5INSIDE': {'edition': 2, 'update': 0}})
    before = snapshot(cfg.corpus_dir)
    with pytest.raises(UpdaterError, match='not in catalog'):
        downloader.update_corpus(cfg)
    assert snapshot(cfg.corpus_dir) == before


def test_dry_run_does_not_prune(tmp_path, monkeypatch):
    """--dry-run previews a region change without deleting corpus data."""
    zip_inside = make_cell_zip('US5INSIDE')
    serve(monkeypatch, REGION_CATALOG_XML, {'US5INSIDE': zip_inside})
    cfg = region_config(tmp_path)
    os.makedirs(cfg.corpus_dir)
    seed_corpus_cell(cfg.corpus_dir, 'US5LEGACY')
    registry.write_cells(registry.manifest_path(cfg.corpus_dir),
                         {'US5LEGACY': {'edition': 9, 'update': 9}})
    _, manifest = downloader.update_corpus(cfg, dry_run=True)
    assert 'US5LEGACY' in manifest
    assert os.path.isdir(os.path.join(cfg.corpus_dir, 'US5LEGACY'))


def test_manifest_entry_with_missing_corpus_data_redownloads(tmp_path, monkeypatch):
    """
    A manifest entry whose corpus dir is gone counts as changed (self-heal).

    Otherwise a crash between prune steps (or a manual deletion) would leave
    the manifest claiming coverage that every future export silently lacks.
    """
    zip_a, zip_b = make_cell_zip('US5NH02M'), make_cell_zip('US4NH01M')
    serve(monkeypatch, catalog_bytes(zip_a, zip_b),
          {'US5NH02M': zip_a, 'US4NH01M': OSError('must not download')})
    cfg = make_config(tmp_path, ['US5NH02M', 'US4NH01M'])
    os.makedirs(cfg.corpus_dir)
    seed_corpus_cell(cfg.corpus_dir, 'US4NH01M')  # intact — not re-fetched
    registry.write_cells(registry.manifest_path(cfg.corpus_dir), {
        'US5NH02M': {'edition': 25, 'update': 3},   # entry, but no dir
        'US4NH01M': {'edition': 12, 'update': 0},
    })
    changed, _ = downloader.update_corpus(cfg)
    assert changed == ['US5NH02M']
    assert os.path.isfile(
        os.path.join(cfg.corpus_dir, 'US5NH02M', 'US5NH02M.000'))


def test_unsafe_catalog_cell_name_skipped(tmp_path, monkeypatch):
    """A catalog row whose name is not a plain [A-Z0-9] segment is dropped."""
    evil = REGION_CATALOG_XML.replace(b'US5OUTSIDE', b'US5/../evil')
    serve(monkeypatch, evil, {})
    catalog = downloader.fetch_catalog('https://example.invalid/catalog.xml', 5.0)
    assert 'US5/../evil' not in catalog
    assert 'US5INSIDE' in catalog
