"""
NOAA ENC catalog fetch, change detection, and corpus updates.

Catalog: ``https://charts.noaa.gov/ENCs/ENCProdCat.xml`` (format verified
live 2026-08-03) — ``<cell>`` elements carrying ``name``, ``edtn`` (edition),
``updn`` (update), ``zipfile_location`` and ``zipfile_size``. The catalog
publishes **no checksums**, so download validation is byte count against
``zipfile_size`` plus a full zip CRC pass (``ZipFile.testzip``) — the
strongest integrity signals available.

Failure contract: any failed or validation-failing download leaves the corpus
and the existing manifest untouched for that cell; already-updated cells from
the same run keep their (genuinely current) manifest entries.
"""

import dataclasses
import os
import shutil
import tempfile
from typing import Dict, List, Optional, Tuple
import urllib.request
import xml.etree.ElementTree as ET
import zipfile

from . import registry
from . import UpdaterError

# Defense-in-depth caps against a malicious or corrupt response from the
# external catalog host. NOAA cells are small (KB–few MB); these ceilings sit
# far above any real cell yet bound memory/disk if a response runs away.
_MAX_CATALOG_BYTES = 64 * 1024 * 1024                  # product catalog XML
_MAX_ZIP_DOWNLOAD_BYTES = 1024 * 1024 * 1024           # one cell zip, on the wire
_MAX_ZIP_UNCOMPRESSED_BYTES = 4 * 1024 * 1024 * 1024   # extracted, zip-bomb guard


@dataclasses.dataclass
class CatalogEntry:
    """One cell's row from the NOAA product catalog."""

    name: str
    edition: int
    update: int
    url: str
    size: Optional[int]


def _open_url(url: str, timeout: float):
    """Open a URL for reading (separate function so tests can substitute)."""
    return urllib.request.urlopen(url, timeout=timeout)


def fetch_catalog(url: str, timeout: float) -> Dict[str, CatalogEntry]:
    """Fetch and parse the product catalog into {cell name: CatalogEntry}."""
    try:
        with _open_url(url, timeout) as response:
            data = response.read(_MAX_CATALOG_BYTES + 1)
    except OSError as e:
        raise UpdaterError(f'download: catalog fetch failed ({url}): {e}')
    if len(data) > _MAX_CATALOG_BYTES:
        raise UpdaterError(
            f'download: catalog at {url} exceeds {_MAX_CATALOG_BYTES} bytes — '
            'refusing to parse (runaway or hostile response)')
    if b'<!DOCTYPE' in data or b'<!ENTITY' in data:
        # ElementTree expands internal entities; a DTD with recursive entity
        # definitions is a billion-laughs vector. The real catalog has no DTD,
        # so reject any document that carries one rather than parse it.
        raise UpdaterError(
            f'download: catalog at {url} carries a DTD/entity declaration — '
            'refusing to parse (entity-expansion guard)')
    try:
        root = ET.fromstring(data)
    except ET.ParseError as e:
        raise UpdaterError(f'download: catalog parse failed ({url}): {e}')

    entries = {}
    for cell in root.iter('cell'):
        name = cell.findtext('name')
        edition = cell.findtext('edtn')
        update = cell.findtext('updn')
        location = cell.findtext('zipfile_location')
        size = cell.findtext('zipfile_size')
        if not name or edition is None or update is None or not location:
            continue
        try:
            entries[name] = CatalogEntry(
                name=name,
                edition=int(edition),
                update=int(update),
                url=location,
                size=int(size) if size else None,
            )
        except ValueError:
            # Non-numeric edition/update/size: skip the row rather than trust it.
            continue
    if not entries:
        raise UpdaterError(f'download: catalog at {url} contained no usable cells')
    return entries


def cells_to_update(
    catalog: Dict[str, CatalogEntry],
    manifest: Dict[str, dict],
    cells: List[str],
) -> List[str]:
    """
    Return configured cells whose catalog edition/update differ from the manifest.

    A configured cell absent from the catalog is an error — either a config
    typo or a cell NOAA has withdrawn; both need a human.
    """
    missing = [c for c in cells if c not in catalog]
    if missing:
        raise UpdaterError(
            f'download: configured cell(s) not in catalog: {missing} '
            '(config typo, or withdrawn by NOAA)')
    changed = []
    for name in cells:
        have = manifest.get(name)
        entry = catalog[name]
        if (have is None or have.get('edition') != entry.edition
                or have.get('update') != entry.update):
            changed.append(name)
    return changed


def _safe_members(zf: zipfile.ZipFile) -> List[zipfile.ZipInfo]:
    """Reject zip-slip paths and cap total uncompressed size (zip-bomb guard)."""
    members = []
    total = 0
    for info in zf.infolist():
        name = info.filename
        if name.startswith(('/', '\\')) or os.path.isabs(name):
            raise UpdaterError(f'download: zip member has absolute path: {name}')
        parts = name.replace('\\', '/').split('/')
        if '..' in parts:
            raise UpdaterError(f'download: zip member escapes archive: {name}')
        total += info.file_size
        if total > _MAX_ZIP_UNCOMPRESSED_BYTES:
            raise UpdaterError(
                f'download: zip uncompressed size exceeds '
                f'{_MAX_ZIP_UNCOMPRESSED_BYTES} bytes — refusing to extract '
                '(possible zip bomb)')
        members.append(info)
    return members


def _copy_capped(src, dst, cap: int, label: str) -> None:
    """Stream src to dst, refusing more than `cap` bytes (runaway guard)."""
    remaining = cap
    while True:
        chunk = src.read(1024 * 1024)
        if not chunk:
            break
        remaining -= len(chunk)
        if remaining < 0:
            raise UpdaterError(
                f'download: {label} exceeds {cap} bytes — refusing '
                '(runaway or hostile response)')
        dst.write(chunk)


def _download_zip(entry: CatalogEntry, dest: str, timeout: float) -> None:
    """Stream one cell zip to dest and validate size + CRC."""
    try:
        with _open_url(entry.url, timeout) as response, open(dest, 'wb') as f:
            _copy_capped(response, f, _MAX_ZIP_DOWNLOAD_BYTES, entry.name)
    except OSError as e:
        raise UpdaterError(f'download: {entry.name} fetch failed ({entry.url}): {e}')
    actual = os.path.getsize(dest)
    if entry.size is not None and actual != entry.size:
        raise UpdaterError(
            f'download: {entry.name} size mismatch: got {actual} bytes, '
            f'catalog says {entry.size}')
    try:
        with zipfile.ZipFile(dest) as zf:
            bad = zf.testzip()
    except zipfile.BadZipFile as e:
        raise UpdaterError(f'download: {entry.name} is not a valid zip: {e}')
    if bad is not None:
        raise UpdaterError(f'download: {entry.name} CRC failure in member {bad}')


def _find_cell_dir(extract_root: str, cell: str) -> str:
    """Locate the extracted cell directory (NOAA zips wrap cells in ENC_ROOT/)."""
    candidates = [
        os.path.join(extract_root, 'ENC_ROOT', cell),
        os.path.join(extract_root, cell),
    ]
    for candidate in candidates:
        if os.path.isfile(os.path.join(candidate, cell + '.000')):
            return candidate
    raise UpdaterError(
        f'download: extracted zip for {cell} does not contain {cell}/{cell}.000')


def _install_cell(corpus_dir: str, cell: str, new_dir: str) -> None:
    """
    Replace corpus_dir/<cell> with new_dir; old data survives any failure.

    The extraction temp dir lives inside corpus_dir, so both renames are
    same-filesystem. The old directory is moved aside first and only removed
    once the new one is in place.
    """
    target = os.path.join(corpus_dir, cell)
    backup = None
    try:
        if os.path.exists(target):
            backup = tempfile.mkdtemp(prefix=f'.old.{cell}.', dir=corpus_dir)
            os.rmdir(backup)  # mkdtemp created it; rename needs it absent
            os.rename(target, backup)
        os.rename(new_dir, target)
    except OSError as e:
        if backup is not None and os.path.exists(backup) and not os.path.exists(target):
            try:
                os.rename(backup, target)
            except OSError as restore_err:
                # Both the install and the rollback failed: the previous cell
                # data survives only in `backup`, and `target` is now missing.
                # Name the backup dir so an operator can restore it by hand.
                raise UpdaterError(
                    f'download: installing {cell} into corpus failed ({e}); '
                    f'restoring the previous data also failed ({restore_err}) — '
                    f'the previous {cell} data is preserved at {backup}; '
                    f'move it back to {target} manually') from e
        raise UpdaterError(f'download: installing {cell} into corpus failed: {e}')
    if backup is not None:
        shutil.rmtree(backup, ignore_errors=True)


def update_corpus(cfg) -> Tuple[List[str], Dict[str, dict]]:
    """
    Bring the corpus up to the catalog; return (changed cells, manifest).

    Cells are updated one at a time; the manifest is saved after each
    successful install so a mid-run failure never misattributes what is on
    disk. Raises UpdaterError on the first failing cell.
    """
    os.makedirs(cfg.corpus_dir, exist_ok=True)
    catalog = fetch_catalog(cfg.catalog_url, cfg.download_timeout)
    manifest = registry.load_cells(registry.manifest_path(cfg.corpus_dir))
    changed = cells_to_update(catalog, manifest, cfg.cells)
    for cell in changed:
        entry = catalog[cell]
        workdir = tempfile.mkdtemp(prefix=f'.download.{cell}.', dir=cfg.corpus_dir)
        try:
            zip_path = os.path.join(workdir, cell + '.zip')
            _download_zip(entry, zip_path, cfg.download_timeout)
            extract_root = os.path.join(workdir, 'extract')
            with zipfile.ZipFile(zip_path) as zf:
                zf.extractall(extract_root, members=_safe_members(zf))
            cell_dir = _find_cell_dir(extract_root, cell)
            _install_cell(cfg.corpus_dir, cell, cell_dir)
        finally:
            shutil.rmtree(workdir, ignore_errors=True)
        manifest[cell] = {'edition': entry.edition, 'update': entry.update}
        registry.write_cells(registry.manifest_path(cfg.corpus_dir), manifest)
        print(f'enc_updater: updated {cell} to edition {entry.edition} '
              f'update {entry.update}')
    return changed, manifest
