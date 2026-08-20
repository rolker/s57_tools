"""
Provision vertical-datum grids (geoid + NOAA VDatum) for the D7 chart export.

``s57_to_geotiff`` consumes a geoid model (``--geoid``) and a VDatum grid
directory (``--vdatum-dir``); this module downloads both on first run so an
operator never has to stage them by hand. Both entry points are idempotent —
presence is a trustworthy completeness signal because every install is an
atomic temp+rename (geoid) or a marker written *last* (VDatum), so a partial
file from an interrupted run can never masquerade as complete. Any failure
raises ``UpdaterError`` (and records it in the corpus health file) so the
caller exits 1 with a clean message rather than letting the export fail later
on absent grids.

Integrity: the geoid ``.tif`` from cdn.proj.org carries no independent declared
size and no self-CRC, so it is verified against a pinned SHA-256 from config
(``geoid_sha256``) — a wrong or corrupt file is rejected, not just a truncated
one. VDatum bundles are zips, validated by ``Content-Length`` (when the server
sends it) plus a full zip CRC pass and zip-slip/zip-bomb member checks — the
same signals ``downloader.py`` applies to ENC cell zips.

Network access goes through ``downloader._open_url`` (the scheme allow-list
guard) and ``downloader._copy_capped`` (the download-size cap), so tests
substitute the network by monkeypatching ``downloader._open_url`` exactly as
``test_downloader.py`` does.
"""

import hashlib
import os
import shutil
import tempfile
from typing import Optional
import zipfile

from . import downloader
from . import health
from . import UpdaterError

# Defense-in-depth on-the-wire caps. A geoid model is a few tens of MB and a
# VDatum bundle a few hundred MB; these ceilings sit far above any real grid
# yet bound disk if a response runs away.
_MAX_GEOID_BYTES = 512 * 1024 * 1024
_MAX_VDATUM_ZIP_BYTES = 2 * 1024 * 1024 * 1024


def _record(cfg, err: UpdaterError) -> None:
    """
    Record a provisioning failure in the health file exactly once.

    Tags the exception so that an error already recorded by ``_fail`` is not
    double-recorded when it is re-raised past an outer ``except UpdaterError``
    handler — that handler is what catches guard failures (scheme allow-list,
    size cap, zip-slip/bomb) raised directly by ``downloader`` helpers, which
    would otherwise never reach the health file.
    """
    if getattr(err, '_provision_recorded', False):
        return
    health.record_error(cfg.corpus_dir, 'provision', str(err))
    err._provision_recorded = True


def _fail(cfg, message: str) -> None:
    """Record a provisioning failure in the health file and raise UpdaterError."""
    err = UpdaterError(message)
    _record(cfg, err)
    raise err


def _remove_quietly(path: str) -> None:
    """Best-effort unlink of a temp file; a cleanup failure must not mask the real error."""
    try:
        os.unlink(path)
    except OSError:
        pass


def _sha256_file(path: str) -> str:
    """Return the hex SHA-256 of a file, read in bounded chunks."""
    digest = hashlib.sha256()
    with open(path, 'rb') as f:
        for chunk in iter(lambda: f.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def _content_length(response) -> Optional[int]:
    """Return the response's Content-Length, or None if absent/unparseable."""
    headers = getattr(response, 'headers', None)
    if headers is None:
        return None
    raw = headers.get('Content-Length')
    if raw is None:
        return None
    try:
        return int(raw)
    except (TypeError, ValueError):
        return None


def ensure_geoid(cfg) -> None:
    """
    Download ``cfg.geoid`` from the PROJ CDN if absent; verify against geoid_sha256.

    No-op when ``cfg.geoid`` is unset or a regular file already exists at the
    path (an existing geoid is trusted because it can only have arrived via the
    atomic rename below — a partial download lives at a temp path and is removed
    on any failure). If the path exists but is *not* a regular file (e.g. a
    misconfiguration pointing at a directory), fails loud rather than silently
    skipping provisioning only to fail later at export. Fails loud, too, if
    provisioning is active but ``geoid_sha256`` is unset rather than installing
    an unverified grid.
    """
    if not cfg.geoid:
        return
    if os.path.isfile(cfg.geoid):
        return
    if os.path.exists(cfg.geoid):
        _fail(cfg,
              f'provision: geoid path {cfg.geoid} exists but is not a regular '
              'file — refusing to provision over it')
    try:
        _install_geoid(cfg)
    except UpdaterError as e:
        _record(cfg, e)
        raise


def _install_geoid(cfg) -> None:
    """Fetch, verify against ``geoid_sha256``, and atomically install the geoid."""
    if not cfg.geoid_sha256:
        _fail(cfg,
              'provision: geoid provisioning is configured (geoid set) but '
              'geoid_sha256 is unset — refusing to install an unverified grid')

    filename = os.path.basename(cfg.geoid)
    url = cfg.geoid_cdn_base_url + filename
    dest_dir = os.path.dirname(cfg.geoid) or '.'
    try:
        os.makedirs(dest_dir, exist_ok=True)
    except OSError as e:
        _fail(cfg, f'provision: cannot create geoid directory {dest_dir}: {e}')

    expected = cfg.geoid_sha256.lower()
    try:
        fd, tmp = tempfile.mkstemp(prefix='.geoid.', dir=dest_dir)
    except OSError as e:
        _fail(cfg, f'provision: cannot create geoid temp file in {dest_dir}: {e}')
    try:
        try:
            with os.fdopen(fd, 'wb') as out, \
                    downloader._open_url(url, cfg.download_timeout) as response:
                downloader._copy_capped(response, out, _MAX_GEOID_BYTES, filename)
        except OSError as e:
            _fail(cfg, f'provision: geoid fetch failed ({url}): {e}')
        try:
            actual = _sha256_file(tmp)
        except OSError as e:
            _fail(cfg, f'provision: geoid {filename} read-back failed: {e}')
        if actual != expected:
            _fail(cfg,
                  f'provision: geoid {filename} SHA-256 mismatch: got {actual}, '
                  f'expected {expected}')
        try:
            os.replace(tmp, cfg.geoid)
        except OSError as e:
            _fail(cfg, f'provision: geoid install to {cfg.geoid} failed: {e}')
        tmp = None
    finally:
        if tmp is not None:
            _remove_quietly(tmp)
    print(f'enc_updater: provisioned geoid {filename}')


def ensure_vdatum(cfg) -> None:
    """
    Download and extract each configured NOAA VDatum bundle if not yet installed.

    No-op when ``cfg.vdatum_dir`` or ``cfg.vdatum_bundles`` is empty. Each
    bundle is guarded by a ``.{bundle}_installed`` marker written *last*, so a
    populated directory with no marker (e.g. an interrupted extraction)
    re-provisions rather than being trusted as complete.
    """
    if not cfg.vdatum_dir or not cfg.vdatum_bundles:
        return
    try:
        os.makedirs(cfg.vdatum_dir, exist_ok=True)
    except OSError as e:
        _fail(cfg,
              f'provision: cannot create vdatum directory {cfg.vdatum_dir}: {e}')
    for bundle in cfg.vdatum_bundles:
        marker = os.path.join(cfg.vdatum_dir, f'.{bundle}_installed')
        if os.path.exists(marker):
            continue
        try:
            _provision_vdatum_bundle(cfg, bundle, marker)
        except UpdaterError as e:
            _record(cfg, e)
            raise


def _provision_vdatum_bundle(cfg, bundle: str, marker: str) -> None:
    """Fetch one VDatum bundle zip, extract its ``*.gtx`` grids, then write the marker."""
    url = cfg.vdatum_cdn_base_url + bundle + '.zip'
    try:
        workdir = tempfile.mkdtemp(prefix=f'.vdatum.{bundle}.', dir=cfg.vdatum_dir)
    except OSError as e:
        _fail(cfg,
              f'provision: cannot create vdatum work dir in {cfg.vdatum_dir}: {e}')
    try:
        zip_path = os.path.join(workdir, bundle + '.zip')
        try:
            with downloader._open_url(url, cfg.download_timeout) as response, \
                    open(zip_path, 'wb') as out:
                declared = _content_length(response)
                downloader._copy_capped(
                    response, out, _MAX_VDATUM_ZIP_BYTES, bundle)
        except OSError as e:
            _fail(cfg, f'provision: vdatum {bundle} fetch failed ({url}): {e}')

        try:
            actual = os.path.getsize(zip_path)
        except OSError as e:
            _fail(cfg, f'provision: vdatum {bundle} zip stat failed: {e}')
        if declared is None:
            print(f'enc_updater: vdatum {bundle} response has no Content-Length '
                  '— skipping byte-count check (zip CRC still enforced)')
        elif actual != declared:
            _fail(cfg,
                  f'provision: vdatum {bundle} size mismatch: got {actual} '
                  f'bytes, Content-Length says {declared}')

        extract_root = os.path.join(workdir, 'extract')
        try:
            with zipfile.ZipFile(zip_path) as zf:
                bad = zf.testzip()
                if bad is not None:
                    _fail(cfg,
                          f'provision: vdatum {bundle} CRC failure in member {bad}')
                members = downloader._safe_members(zf)
                gtx = [m for m in members
                       if m.filename.lower().endswith('.gtx')]
                if not gtx:
                    _fail(cfg,
                          f'provision: vdatum {bundle} zip contains no .gtx grids')
                zf.extractall(extract_root, members=gtx)
        except zipfile.BadZipFile as e:
            _fail(cfg, f'provision: vdatum {bundle} is not a valid zip: {e}')

        # NOAA bundles ship bundle-prefixed grid names inside one top-level dir
        # (verified 2026-08-20: MENHMAgome23_8301/MENHMAgome23_8301_mllw.gtx),
        # so flattening to basename cannot collide across bundles and keeps the
        # *_mllw*.gtx / *_mhhw*.gtx names marine_vertical_datum scans for.
        for member in gtx:
            src = os.path.join(extract_root, member.filename)
            name = os.path.basename(member.filename)
            try:
                os.replace(src, os.path.join(cfg.vdatum_dir, name))
            except OSError as e:
                _fail(cfg,
                      f'provision: vdatum {bundle} install of {name} failed: {e}')
    finally:
        shutil.rmtree(workdir, ignore_errors=True)

    try:
        with open(marker, 'w', encoding='utf-8') as f:
            f.write('')
    except OSError as e:
        _fail(cfg, f'provision: vdatum {bundle} marker write failed: {e}')
    print(f'enc_updater: provisioned vdatum bundle {bundle} '
          f'({len(gtx)} grid(s))')
