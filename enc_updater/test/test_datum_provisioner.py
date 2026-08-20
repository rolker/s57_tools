"""
Mock-HTTP tests for datum grid provisioning (geoid + NOAA VDatum bundles).

Same substitution pattern as ``test_downloader.py``: ``downloader._open_url``
is monkeypatched to serve canned responses, so nothing touches the network.
"""

import hashlib
import io
import os
import tempfile
import zipfile

from enc_updater import datum_provisioner
from enc_updater import downloader
from enc_updater import health
from enc_updater import UpdaterError
from enc_updater.config import UpdaterConfig
import pytest

GEOID_NAME = 'us_noaa_g2018u0.tif'
GEOID_BYTES = b'fake geoid model raster bytes'
GEOID_SHA = hashlib.sha256(GEOID_BYTES).hexdigest()
BUNDLE = 'MENHMAgome23_8301'


class FakeResponse(io.BytesIO):
    """A urlopen-like response: readable, a context manager, with headers."""

    def __init__(self, data, content_length='__auto__'):
        """Wrap ``data``; expose Content-Length (len by default, None to omit)."""
        super().__init__(data)
        if content_length == '__auto__':
            content_length = len(data)
        self.headers = ({} if content_length is None
                        else {'Content-Length': str(content_length)})


def serve(monkeypatch, responses):
    """
    Point ``downloader._open_url`` at canned responses keyed by URL suffix.

    A value may be raw ``bytes`` (Content-Length auto-filled), a
    ``(bytes, content_length)`` tuple, or an ``Exception`` to raise.
    """
    def fake_open_url(url, timeout):
        for suffix, payload in responses.items():
            if url.endswith(suffix):
                if isinstance(payload, Exception):
                    raise payload
                if isinstance(payload, tuple):
                    return FakeResponse(payload[0], payload[1])
                return FakeResponse(payload)
        raise OSError(f'unexpected URL {url}')
    monkeypatch.setattr(downloader, '_open_url', fake_open_url)


def make_config(tmp_path, **overrides):
    """Build an UpdaterConfig over tmp_path with test CDN base URLs."""
    kwargs = {
        'corpus_dir': str(tmp_path / 'corpus'),
        'store_dir': str(tmp_path / 'store'),
        'cells': ['US5NH02M'],
        'geoid': str(tmp_path / 'datum' / 'geoid' / GEOID_NAME),
        'geoid_sha256': GEOID_SHA,
        'geoid_cdn_base_url': 'https://cdn.example.invalid/',
        'vdatum_dir': str(tmp_path / 'datum' / 'vdatum'),
        'vdatum_bundles': [BUNDLE],
        'vdatum_cdn_base_url': 'https://vdatum.example.invalid/',
    }
    kwargs.update(overrides)
    return UpdaterConfig(**kwargs)


def make_vdatum_zip(gtx=('mllw.gtx', 'tss.gtx'), extra=()):
    """Build an in-memory VDatum-style zip carrying ``*.gtx`` (plus any extras)."""
    buffer = io.BytesIO()
    with zipfile.ZipFile(buffer, 'w') as zf:
        for name in gtx:
            zf.writestr(name, b'fake gtx grid payload for ' + name.encode())
        for name in extra:
            zf.writestr(name, b'non-grid content')
    return buffer.getvalue()


def health_error(corpus_dir):
    """Return the recorded last_error dict from the health file, or None."""
    path = os.path.join(corpus_dir, health.HEALTH_NAME)
    if not os.path.exists(path):
        return None
    import json
    with open(path, encoding='utf-8') as f:
        return json.load(f).get('last_error')


# --- geoid ---------------------------------------------------------------

def test_geoid_provisioned_from_scratch(tmp_path, monkeypatch):
    """Happy path: the geoid is downloaded and appears at the configured path."""
    serve(monkeypatch, {GEOID_NAME: GEOID_BYTES})
    cfg = make_config(tmp_path)
    datum_provisioner.ensure_geoid(cfg)
    assert os.path.isfile(cfg.geoid)
    with open(cfg.geoid, 'rb') as f:
        assert f.read() == GEOID_BYTES


def test_geoid_idempotent_when_present(tmp_path, monkeypatch):
    """An existing geoid is trusted: no download is attempted."""
    serve(monkeypatch, {GEOID_NAME: OSError('must not download')})
    cfg = make_config(tmp_path)
    os.makedirs(os.path.dirname(cfg.geoid))
    with open(cfg.geoid, 'wb') as f:
        f.write(b'already here')
    datum_provisioner.ensure_geoid(cfg)
    with open(cfg.geoid, 'rb') as f:
        assert f.read() == b'already here'


def test_geoid_path_is_directory_fails_loud(tmp_path, monkeypatch):
    """A geoid path that exists as a directory is a hard error, not a silent skip."""
    serve(monkeypatch, {GEOID_NAME: OSError('must not download')})
    cfg = make_config(tmp_path)
    os.makedirs(cfg.geoid)  # the configured geoid path is a directory
    with pytest.raises(UpdaterError, match='not a regular file'):
        datum_provisioner.ensure_geoid(cfg)
    assert health_error(cfg.corpus_dir)['phase'] == 'provision'


def test_geoid_unset_is_noop(tmp_path, monkeypatch):
    """No geoid configured means nothing to provision."""
    serve(monkeypatch, {GEOID_NAME: OSError('must not download')})
    cfg = make_config(tmp_path, geoid=None)
    datum_provisioner.ensure_geoid(cfg)  # must not raise


def test_geoid_failed_download_leaves_no_partial(tmp_path, monkeypatch):
    """A network failure raises UpdaterError and leaves no file or temp behind."""
    serve(monkeypatch, {GEOID_NAME: OSError('connection reset')})
    cfg = make_config(tmp_path)
    with pytest.raises(UpdaterError, match='geoid fetch failed'):
        datum_provisioner.ensure_geoid(cfg)
    assert not os.path.exists(cfg.geoid)
    assert os.listdir(os.path.dirname(cfg.geoid)) == []
    assert health_error(cfg.corpus_dir)['phase'] == 'provision'


def test_geoid_sha_mismatch_rejected(tmp_path, monkeypatch):
    """A wrong SHA-256 is rejected; the temp file is removed, nothing installed."""
    serve(monkeypatch, {GEOID_NAME: GEOID_BYTES})
    cfg = make_config(tmp_path, geoid_sha256='0' * 64)
    with pytest.raises(UpdaterError, match='SHA-256 mismatch'):
        datum_provisioner.ensure_geoid(cfg)
    assert not os.path.exists(cfg.geoid)
    assert os.listdir(os.path.dirname(cfg.geoid)) == []


def test_geoid_sha_unset_fails_loud(tmp_path, monkeypatch):
    """Geoid set but geoid_sha256 unset is a hard error, not a silent skip."""
    serve(monkeypatch, {GEOID_NAME: OSError('must not download')})
    cfg = make_config(tmp_path, geoid_sha256=None)
    with pytest.raises(UpdaterError, match='geoid_sha256 is unset'):
        datum_provisioner.ensure_geoid(cfg)
    assert not os.path.exists(cfg.geoid)


def test_geoid_rejects_non_http_scheme(tmp_path):
    """A non-http(s) CDN base URL is refused by the inherited scheme guard."""
    cfg = make_config(tmp_path, geoid_cdn_base_url='file:///etc/')
    with pytest.raises(UpdaterError, match='refusing non-http'):
        datum_provisioner.ensure_geoid(cfg)
    assert not os.path.exists(cfg.geoid)
    # The temp file created before the scheme check must be cleaned up.
    assert os.listdir(os.path.dirname(cfg.geoid)) == []
    # A guard failure (not just a network error) is still recorded as a
    # provisioning failure in the health file.
    assert health_error(cfg.corpus_dir)['phase'] == 'provision'


# --- vdatum --------------------------------------------------------------

def test_vdatum_provisioned_from_scratch(tmp_path, monkeypatch):
    """Happy path: *.gtx files are extracted and the marker is written."""
    zip_bytes = make_vdatum_zip(extra=('readme.txt',))
    serve(monkeypatch, {BUNDLE + '.zip': zip_bytes})
    cfg = make_config(tmp_path)
    datum_provisioner.ensure_vdatum(cfg)
    assert os.path.isfile(os.path.join(cfg.vdatum_dir, 'mllw.gtx'))
    assert os.path.isfile(os.path.join(cfg.vdatum_dir, 'tss.gtx'))
    # Only .gtx grids are installed; the non-grid member is dropped.
    assert not os.path.exists(os.path.join(cfg.vdatum_dir, 'readme.txt'))
    assert os.path.isfile(os.path.join(cfg.vdatum_dir, f'.{BUNDLE}_installed'))


def test_vdatum_idempotent_when_marker_present(tmp_path, monkeypatch):
    """A bundle whose marker exists is not re-downloaded."""
    serve(monkeypatch, {BUNDLE + '.zip': OSError('must not download')})
    cfg = make_config(tmp_path)
    os.makedirs(cfg.vdatum_dir)
    with open(os.path.join(cfg.vdatum_dir, f'.{BUNDLE}_installed'), 'w') as f:
        f.write('')
    datum_provisioner.ensure_vdatum(cfg)  # must not raise


def test_vdatum_unset_is_noop(tmp_path, monkeypatch):
    """No bundles configured means nothing to provision."""
    serve(monkeypatch, {BUNDLE + '.zip': OSError('must not download')})
    cfg = make_config(tmp_path, vdatum_bundles=[])
    datum_provisioner.ensure_vdatum(cfg)  # must not raise


def test_vdatum_content_length_mismatch_rejected(tmp_path, monkeypatch):
    """A Content-Length that disagrees with the body is rejected; no marker."""
    zip_bytes = make_vdatum_zip()
    serve(monkeypatch, {BUNDLE + '.zip': (zip_bytes, len(zip_bytes) + 100)})
    cfg = make_config(tmp_path)
    with pytest.raises(UpdaterError, match='size mismatch'):
        datum_provisioner.ensure_vdatum(cfg)
    assert not os.path.exists(os.path.join(cfg.vdatum_dir, f'.{BUNDLE}_installed'))
    assert not os.path.exists(os.path.join(cfg.vdatum_dir, 'mllw.gtx'))


def test_vdatum_no_gtx_rejected(tmp_path, monkeypatch):
    """A zip with no *.gtx grids is an error; no marker is written."""
    zip_bytes = make_vdatum_zip(gtx=(), extra=('readme.txt', 'data.bin'))
    serve(monkeypatch, {BUNDLE + '.zip': zip_bytes})
    cfg = make_config(tmp_path)
    with pytest.raises(UpdaterError, match='no .gtx grids'):
        datum_provisioner.ensure_vdatum(cfg)
    assert not os.path.exists(os.path.join(cfg.vdatum_dir, f'.{BUNDLE}_installed'))


def test_vdatum_failed_download_leaves_no_marker(tmp_path, monkeypatch):
    """A network failure raises UpdaterError and writes no marker."""
    serve(monkeypatch, {BUNDLE + '.zip': OSError('connection reset')})
    cfg = make_config(tmp_path)
    with pytest.raises(UpdaterError, match='fetch failed'):
        datum_provisioner.ensure_vdatum(cfg)
    assert not os.path.exists(os.path.join(cfg.vdatum_dir, f'.{BUNDLE}_installed'))
    assert health_error(cfg.corpus_dir)['phase'] == 'provision'


def test_vdatum_zip_slip_rejected(tmp_path, monkeypatch):
    """A member escaping the archive (`..`) is refused before extraction."""
    buffer = io.BytesIO()
    with zipfile.ZipFile(buffer, 'w') as zf:
        zf.writestr('../escape.gtx', b'malicious grid')
    serve(monkeypatch, {BUNDLE + '.zip': buffer.getvalue()})
    cfg = make_config(tmp_path)
    with pytest.raises(UpdaterError, match='escapes archive'):
        datum_provisioner.ensure_vdatum(cfg)
    assert not os.path.exists(os.path.join(cfg.vdatum_dir, f'.{BUNDLE}_installed'))
    assert not os.path.exists(tmp_path / 'datum' / 'escape.gtx')
    # The zip-slip guard raises UpdaterError directly; it must still be
    # recorded as a provisioning failure in the health file.
    assert health_error(cfg.corpus_dir)['phase'] == 'provision'


def test_vdatum_corrupt_zip_rejected(tmp_path, monkeypatch):
    """Bytes that are not a valid zip are rejected; no marker is written."""
    serve(monkeypatch, {BUNDLE + '.zip': b'not a zip at all'})
    cfg = make_config(tmp_path)
    with pytest.raises(UpdaterError, match='not a valid zip'):
        datum_provisioner.ensure_vdatum(cfg)
    assert not os.path.exists(os.path.join(cfg.vdatum_dir, f'.{BUNDLE}_installed'))


def _fail_replace_for(monkeypatch, predicate):
    """
    Make os.replace raise for destinations matching ``predicate``.

    Selective on purpose: health.py also uses os.replace for its atomic
    write, and a blanket failure would break the very recording under test.
    """
    real_replace = os.replace

    def failing_replace(src, dst):
        if predicate(str(dst)):
            raise OSError('disk full')
        return real_replace(src, dst)
    monkeypatch.setattr(os, 'replace', failing_replace)


def test_geoid_install_failure_recorded(tmp_path, monkeypatch):
    """An OSError installing the verified geoid is recorded, not escaped."""
    serve(monkeypatch, {GEOID_NAME: GEOID_BYTES})
    cfg = make_config(tmp_path)
    _fail_replace_for(monkeypatch, lambda dst: dst == cfg.geoid)
    with pytest.raises(UpdaterError, match='geoid install'):
        datum_provisioner.ensure_geoid(cfg)
    assert not os.path.exists(cfg.geoid)
    assert health_error(cfg.corpus_dir)['phase'] == 'provision'


def test_vdatum_grid_install_failure_recorded(tmp_path, monkeypatch):
    """An OSError installing an extracted grid is recorded; no marker written."""
    serve(monkeypatch, {BUNDLE + '.zip': make_vdatum_zip()})
    cfg = make_config(tmp_path)
    _fail_replace_for(monkeypatch, lambda dst: dst.endswith('.gtx'))
    with pytest.raises(UpdaterError, match='install of'):
        datum_provisioner.ensure_vdatum(cfg)
    assert not os.path.exists(os.path.join(cfg.vdatum_dir, f'.{BUNDLE}_installed'))
    assert health_error(cfg.corpus_dir)['phase'] == 'provision'


def test_geoid_tempfile_failure_recorded(tmp_path, monkeypatch):
    """
    An OSError creating the geoid temp file is recorded, not escaped.

    Selective on the ``.geoid.`` prefix: health.py also uses mkstemp for its
    atomic write, and a blanket failure would break the recording under test.
    """
    serve(monkeypatch, {GEOID_NAME: GEOID_BYTES})
    cfg = make_config(tmp_path)
    real_mkstemp = tempfile.mkstemp

    def failing_mkstemp(*args, **kwargs):
        if kwargs.get('prefix') == '.geoid.':
            raise OSError('no space left on device')
        return real_mkstemp(*args, **kwargs)
    monkeypatch.setattr(tempfile, 'mkstemp', failing_mkstemp)
    with pytest.raises(UpdaterError, match='cannot create geoid temp file'):
        datum_provisioner.ensure_geoid(cfg)
    assert health_error(cfg.corpus_dir)['phase'] == 'provision'


def test_vdatum_tempdir_failure_recorded(tmp_path, monkeypatch):
    """An OSError creating the vdatum work dir is recorded; no marker written."""
    serve(monkeypatch, {BUNDLE + '.zip': make_vdatum_zip()})
    cfg = make_config(tmp_path)

    def failing_mkdtemp(*args, **kwargs):
        raise OSError('no space left on device')
    monkeypatch.setattr(tempfile, 'mkdtemp', failing_mkdtemp)
    with pytest.raises(UpdaterError, match='cannot create vdatum work dir'):
        datum_provisioner.ensure_vdatum(cfg)
    assert not os.path.exists(os.path.join(cfg.vdatum_dir, f'.{BUNDLE}_installed'))
    assert health_error(cfg.corpus_dir)['phase'] == 'provision'


def test_vdatum_zip_stat_failure_recorded(tmp_path, monkeypatch):
    """An OSError stat-ing the downloaded vdatum zip is recorded, not escaped."""
    serve(monkeypatch, {BUNDLE + '.zip': make_vdatum_zip()})
    cfg = make_config(tmp_path)
    real_getsize = os.path.getsize

    def failing_getsize(path):
        if str(path).endswith('.zip'):
            raise OSError('stat failed')
        return real_getsize(path)
    monkeypatch.setattr(os.path, 'getsize', failing_getsize)
    with pytest.raises(UpdaterError, match='zip stat failed'):
        datum_provisioner.ensure_vdatum(cfg)
    assert not os.path.exists(os.path.join(cfg.vdatum_dir, f'.{BUNDLE}_installed'))
    assert health_error(cfg.corpus_dir)['phase'] == 'provision'
