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
import re
import shutil
import tempfile
from typing import Dict, List, Optional, Tuple
import urllib.parse
import urllib.request
import xml.etree.ElementTree as ET
import zipfile

from . import registry
from . import selection
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
    # Region-selection fields (see selection.py). status is 'Active' /
    # 'Cancelled'; panels holds the type-'E' exterior coverage polygons as
    # (lon, lat) vertex lists. Both default to the values a pre-#40 caller
    # (or test) implied, so existing constructions stay valid.
    status: str = 'Active'
    panels: List[List[Tuple[float, float]]] = dataclasses.field(
        default_factory=list)


_ALLOWED_URL_SCHEMES = ('http', 'https')

# Catalog cell names become filesystem path segments (corpus install/prune
# targets, tempdir prefixes) and — in region mode (#40) — arrive from the
# untrusted catalog rather than the operator's config. Constrain them to a
# single safe segment; real NOAA names are 8 chars of [A-Z0-9]. A row with
# any other shape is skipped at parse like the other malformed-row cases,
# so it can never reach os.path.join or shutil.rmtree.
_CELL_NAME_RE = re.compile(r'[A-Z0-9]{3,32}')


def _open_url(url: str, timeout: float):
    """Open a URL for reading (separate function so tests can substitute)."""
    # Allow-list the scheme before opening. The cell-zip URL comes from the
    # (external, untrusted) catalog; without this a spoofed catalog could point
    # `zipfile_location` at `file://` (local file exfiltration) or another
    # scheme urllib supports, turning the fetch into an SSRF/LFI vector.
    scheme = urllib.parse.urlparse(url).scheme.lower()
    if scheme not in _ALLOWED_URL_SCHEMES:
        raise UpdaterError(
            f'download: refusing non-http(s) URL {url!r} '
            f'(scheme {scheme!r} not in {_ALLOWED_URL_SCHEMES})')
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
        if not _CELL_NAME_RE.fullmatch(name):
            # A name that isn't a plain [A-Z0-9] segment can't be a real NOAA
            # cell and must never become a filesystem path component.
            continue
        try:
            entries[name] = CatalogEntry(
                name=name,
                edition=int(edition),
                update=int(update),
                url=location,
                size=int(size) if size else None,
                status=(cell.findtext('status') or 'Active').strip(),
                panels=_parse_panels(cell),
            )
        except ValueError:
            # Non-numeric edition/update/size: skip the row rather than trust it.
            continue
    if not entries:
        raise UpdaterError(f'download: catalog at {url} contained no usable cells')
    return entries


def _parse_panels(cell: ET.Element) -> List[List[Tuple[float, float]]]:
    """
    Type-'E' exterior coverage polygons of one catalog cell, as (lon, lat).

    Interior-hole panels (type 'I') are skipped — see selection.py for why
    that over-selection is harmless. A panel with a malformed vertex or fewer
    than 3 usable vertices is skipped rather than trusted. For a cell not yet
    installed that just means region selection cannot match on it; for an
    *installed* cell, an Active row with zero usable panels is refused as a
    degraded catalog by ``guard_degenerate_deselections`` rather than pruned.
    The explicit-cells mode never reads panels at all.
    """
    panels = []
    cov = cell.find('cov')
    if cov is None:
        return panels
    for panel in cov.findall('panel'):
        if (panel.findtext('type') or 'E').strip() != 'E':
            continue
        vertices = []
        for vertex in panel.findall('vertex'):
            lat = vertex.findtext('lat')
            lon = vertex.findtext('long')
            try:
                vertices.append((float(lon), float(lat)))
            except (TypeError, ValueError):
                vertices = []
                break
        if len(vertices) >= 3:
            panels.append(vertices)
    return panels


def resolve_cells(cfg, catalog: Dict[str, CatalogEntry]) -> List[str]:
    """
    Resolve the cycle's cell set: configured pins, or a region query.

    Explicit ``cells:`` mode passes the configured list through unchanged
    (``cells_to_update`` still fail-louds on a name the catalog dropped).
    ``region:`` mode derives the set from the fetched catalog's coverage
    polygons — the rescheme-proof path (#40).
    """
    if cfg.region is not None:
        return selection.select_cells(
            catalog, cfg.region, cfg.bands, cfg.max_cells)
    return list(cfg.cells)


def guard_degenerate_deselections(
    cfg,
    catalog: Dict[str, CatalogEntry],
    manifest: Dict[str, dict],
    keep: List[str],
) -> None:
    """
    Refuse to prune an installed cell whose catalog row looks broken.

    Region mode cannot ask NOAA "did you really withdraw this cell?" — but it
    can distinguish positive evidence of withdrawal (the row is gone, or its
    status is ``Cancelled``) from the signature of a degraded catalog: a row
    still ``Active`` whose coverage panels all failed to parse, or a status
    string this code doesn't recognize. Pruning on that signature would turn
    a transient upstream data defect into removed navigation coverage with a
    successful exit — so it is a hard error instead (previous layer stands;
    the next good catalog resumes normally). An Active row with *usable*
    panels that genuinely stopped intersecting the region still prunes: that
    is a real coverage/config change, not a parse failure. Cells-mode
    deselection is always an operator config edit and is never guarded.
    """
    if cfg.region is None:
        return
    for name in sorted(set(manifest) - set(keep)):
        entry = catalog.get(name)
        if entry is None:
            continue  # gone from the catalog entirely — the withdrawal path
        if entry.status == 'Active' and not entry.panels:
            raise UpdaterError(
                f'selection: installed cell {name} is Active in the catalog '
                'but its coverage panels failed to parse — refusing to prune '
                'on a degraded catalog (previous layer stands; will retry '
                'next cycle)')
        if entry.status not in ('Active', 'Cancelled'):
            raise UpdaterError(
                f'selection: installed cell {name} has unrecognized catalog '
                f'status {entry.status!r} — refusing to prune without '
                'positive evidence of withdrawal')


def prune_corpus(
    corpus_dir: str,
    keep: List[str],
    manifest: Dict[str, dict],
    dry_run: bool = False,
) -> List[str]:
    """
    Drop manifest cells (and their corpus dirs) not in this cycle's set.

    Required for correctness, not tidiness: the D7 export runs over the
    *whole corpus*, so a deselected or NOAA-withdrawn cell left on disk keeps
    contributing stale tiles to every future chart layer. Pruning the
    manifest also makes ``_need_regeneration``'s manifest-vs-active-registry
    comparison trigger the wholesale regeneration that forgets the cell.

    With ``dry_run`` nothing is deleted — the would-be prunes are printed and
    returned, keeping ``--dry-run`` free of irreversible corpus mutation (its
    whole point is previewing a candidate ``region:`` safely). The manifest
    is re-saved after every individual removal so a crash mid-prune can never
    leave the manifest claiming a cell whose corpus dir is already gone.
    Returns the (would-be) pruned names; the manifest dict is mutated and
    saved only in a real run.
    """
    stale = sorted(set(manifest) - set(keep))
    for name in stale:
        if dry_run:
            print(f'enc_updater: dry run — would prune {name} from corpus '
                  '(no longer selected / withdrawn by NOAA)')
            continue
        cell_dir = os.path.join(corpus_dir, name)
        if os.path.islink(cell_dir):
            # rmtree refuses symlinks anyway (OSError), but name the problem
            # instead of surfacing its generic message: a linked cell dir is
            # an operator arrangement this updater must not delete through
            # or silently unlink. Manifest entry stays until a human acts.
            raise UpdaterError(
                f'download: corpus entry {name} is a symlink — refusing to '
                'prune it (corpus cells must be real directories; remove '
                'the link by hand)')
        if os.path.isdir(cell_dir):
            try:
                shutil.rmtree(cell_dir)
            except OSError as e:
                raise UpdaterError(
                    f'download: pruning {name} from corpus failed: {e}')
        elif os.path.exists(cell_dir):
            # Deselected entry whose path is a stray non-directory: the
            # manifest entry still goes (the cell is deselected), but say
            # what was left behind rather than skipping it silently.
            print(f'enc_updater: warning: corpus entry {name} is not a '
                  f'directory; leaving {cell_dir} in place')
        del manifest[name]
        registry.write_cells(registry.manifest_path(corpus_dir), manifest)
        print(f'enc_updater: pruned {name} from corpus '
              '(no longer selected / withdrawn by NOAA)')
    return stale


def cells_to_update(
    catalog: Dict[str, CatalogEntry],
    manifest: Dict[str, dict],
    cells: List[str],
    corpus_dir: str,
) -> List[str]:
    """
    Return selected cells whose catalog edition/update differ from the manifest.

    A selected cell absent from the catalog is an error — either a config
    typo or a cell NOAA has withdrawn; both need a human. (Region-derived
    cells came *from* the catalog, so this only fires in explicit-cells
    mode.) A manifest entry whose corpus data is missing on disk (crash
    aftermath, manual deletion) also counts as changed — the re-download
    heals any corpus/manifest divergence instead of exporting a hole while
    the manifest claims coverage.
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
        base_file = os.path.join(corpus_dir, name, name + '.000')
        if (have is None or have.get('edition') != entry.edition
                or have.get('update') != entry.update
                or not os.path.isfile(base_file)):
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
    if entry.size is None:
        # The catalog omitted zipfile_size, so the byte-count check is skipped
        # (CRC still runs). Log it so the degraded integrity check is visible
        # rather than silently weaker.
        print(f'enc_updater: {entry.name} catalog entry has no zipfile_size — '
              'skipping byte-count check (zip CRC still enforced)')
    elif actual != entry.size:
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


def update_corpus(cfg, dry_run: bool = False) -> Tuple[List[str], Dict[str, dict]]:
    """
    Bring the corpus up to the catalog; return (changed cells, manifest).

    Cells are updated one at a time; the manifest is saved after each
    successful install so a mid-run failure never misattributes what is on
    disk. Raises UpdaterError on the first failing cell.

    Ordering is validate → download → prune, deliberately: pruning last
    means a failing run (config typo caught by ``cells_to_update``, a
    transient download failure, a degenerate-catalog refusal from
    ``guard_degenerate_deselections``) exits with the previous corpus and
    manifest fully intact — pruning first would delete valid data before
    the failure surfaced, and a rescheme whose replacement download failed
    would leave a coverage hole. ``dry_run`` reaches ``prune_corpus`` so a
    preview run never mutates the corpus.
    """
    os.makedirs(cfg.corpus_dir, exist_ok=True)
    catalog = fetch_catalog(cfg.catalog_url, cfg.download_timeout)
    manifest = registry.load_cells(registry.manifest_path(cfg.corpus_dir))
    cells = resolve_cells(cfg, catalog)
    guard_degenerate_deselections(cfg, catalog, manifest, cells)
    changed = cells_to_update(catalog, manifest, cells, cfg.corpus_dir)
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
    prune_corpus(cfg.corpus_dir, cells, manifest, dry_run=dry_run)
    return changed, manifest
