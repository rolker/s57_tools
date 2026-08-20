"""Region-config loading and validation for the ENC updater."""

import dataclasses
import os
from typing import List, Optional, Tuple

import yaml

from . import selection
from . import UpdaterError

DEFAULT_CATALOG_URL = 'https://charts.noaa.gov/ENCs/ENCProdCat.xml'

# PROJ grid CDN (geoid models) and NOAA VDatum bundle download roots. Base URLs
# end in '/' so a bundle/geoid filename appends directly (see datum_provisioner).
DEFAULT_GEOID_CDN_BASE_URL = 'https://cdn.proj.org/'
DEFAULT_VDATUM_CDN_BASE_URL = 'https://vdatum.noaa.gov/download/data/'

# Band 1 is WGS84 ellipsoidal height (up-positive). The +100 m upper bound
# deliberately admits inland/lake surfaces (e.g. Lake Massabesic sits near
# +52 m ellipsoidal); the -12000 m lower bound is deeper than any ocean.
DEFAULT_DEPTH_RANGE = (-12000.0, 100.0)


@dataclasses.dataclass
class NavLivenessConfig:
    """
    Nav-down interlock probe settings.

    An empty ``nodes`` list means the interlock is not configured for this
    host (e.g. a dev machine with no navigation stack) and the probe is
    skipped entirely. With nodes configured, any probe failure refuses the
    swap (fail closed).

    ``ros_domain_id`` pins the probe's DDS domain to the live nav stack's.
    Without it, a cron/probe environment whose ``ROS_DOMAIN_ID`` differs from
    the nav stack's queries the wrong graph, sees no nodes, and the interlock
    fails *open* (the empty result is indistinguishable from "nav down"). See
    the README nav-liveness contract for the full env-alignment requirement.
    """

    nodes: List[str] = dataclasses.field(default_factory=list)
    ros_setup: Optional[str] = None
    ros_domain_id: Optional[int] = None
    timeout: float = 20.0


@dataclasses.dataclass
class UpdaterConfig:
    """Validated region configuration (see config/region_example.yaml)."""

    corpus_dir: str
    store_dir: str
    # Cell selection: exactly one of `cells` (explicit pin list, fail-loud on
    # a name the catalog drops) or `region` (a lon/lat polygon; the cell set
    # is derived from the live catalog's coverage panels each cycle — the
    # rescheme-proof mode, #40). `bands` and `max_cells` apply to region
    # mode only; see selection.py for their semantics and defaults.
    cells: List[str] = dataclasses.field(default_factory=list)
    region: Optional[List[Tuple[float, float]]] = None
    bands: List[int] = dataclasses.field(
        default_factory=lambda: list(selection.DEFAULT_BANDS))
    max_cells: int = selection.DEFAULT_MAX_CELLS
    catalog_url: str = DEFAULT_CATALOG_URL
    geoid: Optional[str] = None
    geoid_sha256: Optional[str] = None
    geoid_cdn_base_url: str = DEFAULT_GEOID_CDN_BASE_URL
    vdatum_dir: Optional[str] = None
    vdatum_bundles: List[str] = dataclasses.field(default_factory=list)
    vdatum_cdn_base_url: str = DEFAULT_VDATUM_CDN_BASE_URL
    datum_config: Optional[str] = None
    lake_datum: Optional[float] = None
    cell_size: Optional[float] = None
    depth_range: Tuple[float, float] = DEFAULT_DEPTH_RANGE
    nav_liveness: NavLivenessConfig = dataclasses.field(default_factory=NavLivenessConfig)
    s57_to_geotiff_bin: Optional[str] = None
    import_geotiff_bin: Optional[str] = None
    download_timeout: float = 300.0
    export_timeout: float = 3600.0
    stage_timeout: float = 600.0
    commit_timeout: float = 120.0


_TOP_LEVEL_KEYS = {
    'corpus_dir', 'store_dir', 'cells', 'region', 'bands', 'max_cells',
    'catalog_url',
    'geoid', 'geoid_sha256', 'geoid_cdn_base_url',
    'vdatum_dir', 'vdatum_bundles', 'vdatum_cdn_base_url',
    'datum_config', 'lake_datum',
    'cell_size', 'depth_range', 'nav_liveness',
    's57_to_geotiff_bin', 'import_geotiff_bin',
    'download_timeout', 'export_timeout', 'stage_timeout', 'commit_timeout',
}

_NAV_KEYS = {'nodes', 'ros_setup', 'ros_domain_id', 'timeout'}


def _expand(path: Optional[str]) -> Optional[str]:
    if path is None:
        return None
    return os.path.abspath(os.path.expanduser(str(path)))


def _require_float(value, label: str) -> float:
    """
    Coerce a config value to float, raising UpdaterError (not ValueError).

    A bare ``float()`` on a mistyped YAML value (e.g. ``timeout: fast``) would
    escape ``load_config`` as an uncaught ``ValueError`` and crash with a
    traceback instead of the documented clean exit 1; route every coercion
    through here so a bad value reads as a config error like any other.
    """
    try:
        return float(value)
    except (TypeError, ValueError):
        raise UpdaterError(f'config: "{label}" must be a number, got {value!r}')


def _parse_region(raw) -> List[Tuple[float, float]]:
    """
    Normalize a config ``region`` to a lon/lat polygon (vertex list).

    Two accepted shapes: a flat 4-number bbox ``[lon_min, lat_min, lon_max,
    lat_max]`` (expanded to its corner polygon), or a polygon as a list of
    3+ ``[lon, lat]`` pairs. Coordinates are validated to lon [-180, 180] /
    lat [-90, 90] — which also enforces the planar no-antimeridian-wrap
    assumption selection.py documents.
    """
    def _coord(value, label):
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise UpdaterError(f'config: region {label} must be a number, '
                               f'got {value!r}')
        return float(value)

    def _check_lon_lat(lon, lat):
        if not -180.0 <= lon <= 180.0 or not -90.0 <= lat <= 90.0:
            raise UpdaterError(
                f'config: region coordinate ({lon}, {lat}) outside '
                'lon [-180, 180] / lat [-90, 90] — vertices are [lon, lat] '
                'pairs (x first), bbox is [lon_min, lat_min, lon_max, lat_max]')

    if (isinstance(raw, list) and len(raw) == 4
            and all(not isinstance(v, (list, tuple)) for v in raw)):
        lon_min = _coord(raw[0], 'bbox lon_min')
        lat_min = _coord(raw[1], 'bbox lat_min')
        lon_max = _coord(raw[2], 'bbox lon_max')
        lat_max = _coord(raw[3], 'bbox lat_max')
        if not (lon_min < lon_max and lat_min < lat_max):
            raise UpdaterError(
                'config: region bbox must satisfy lon_min < lon_max and '
                'lat_min < lat_max ([lon_min, lat_min, lon_max, lat_max])')
        for lon, lat in ((lon_min, lat_min), (lon_max, lat_max)):
            _check_lon_lat(lon, lat)
        return [(lon_min, lat_min), (lon_max, lat_min),
                (lon_max, lat_max), (lon_min, lat_max)]

    if (isinstance(raw, list) and len(raw) >= 3
            and all(isinstance(v, (list, tuple)) and len(v) == 2 for v in raw)):
        polygon = []
        for pair in raw:
            lon = _coord(pair[0], 'vertex lon')
            lat = _coord(pair[1], 'vertex lat')
            _check_lon_lat(lon, lat)
            polygon.append((lon, lat))
        return polygon

    raise UpdaterError(
        'config: "region" must be a bbox [lon_min, lat_min, lon_max, lat_max] '
        'or a polygon of 3+ [lon, lat] pairs')


def load_config(path: str) -> UpdaterConfig:
    """
    Load and validate a region config; raise UpdaterError on any problem.

    Unknown keys are rejected rather than ignored: a typo'd key silently
    falling back to a default is exactly how a field config drifts from what
    the operator believes it says.
    """
    try:
        with open(path, 'r', encoding='utf-8') as f:
            raw = yaml.safe_load(f)
    except (OSError, yaml.YAMLError) as e:
        raise UpdaterError(f'config: cannot read {path}: {e}')
    if not isinstance(raw, dict):
        raise UpdaterError(f'config: {path} is not a YAML mapping')

    unknown = set(raw) - _TOP_LEVEL_KEYS
    if unknown:
        raise UpdaterError(f'config: unknown key(s) {sorted(unknown)} in {path}')
    for key in ('corpus_dir', 'store_dir'):
        if key not in raw:
            raise UpdaterError(f'config: missing required key "{key}" in {path}')

    if ('cells' in raw) == ('region' in raw):
        raise UpdaterError(
            'config: exactly one of "cells" (explicit pin list) or "region" '
            f'(lon/lat area, catalog-driven selection) is required in {path}')

    cells = raw.get('cells', [])
    if 'cells' in raw and (
            not isinstance(cells, list) or not cells
            or not all(isinstance(c, str) and c for c in cells)):
        raise UpdaterError('config: "cells" must be a non-empty list of cell names')

    region = _parse_region(raw.get('region')) if 'region' in raw else None
    for key in ('bands', 'max_cells'):
        if key in raw and region is None:
            raise UpdaterError(
                f'config: "{key}" applies to region-driven selection only — '
                'it has no effect with an explicit "cells" list')
    bands = raw.get('bands', list(selection.DEFAULT_BANDS))
    if (not isinstance(bands, list) or not bands
            or not all(isinstance(b, int) and not isinstance(b, bool)
                       and 1 <= b <= 6 for b in bands)):
        raise UpdaterError(
            'config: "bands" must be a non-empty list of ENC usage bands '
            '(integers 1-6)')
    max_cells = raw.get('max_cells', selection.DEFAULT_MAX_CELLS)
    if (isinstance(max_cells, bool) or not isinstance(max_cells, int)
            or max_cells < 1):
        raise UpdaterError('config: "max_cells" must be a positive integer')

    bundles = raw.get('vdatum_bundles', []) or []
    if (not isinstance(bundles, list)
            or not all(isinstance(b, str) and b for b in bundles)):
        raise UpdaterError(
            'config: "vdatum_bundles" must be a list of NOAA bundle names')
    # A bundle name is interpolated straight into a download URL and into
    # on-disk paths (the marker file and extraction dir); a path separator or
    # ``..`` would escape vdatum_dir (or yield an opaque FileNotFoundError), so
    # reject those shapes here as a clean config error.
    illegal = [b for b in bundles if '/' in b or '\\' in b or '..' in b]
    if illegal:
        raise UpdaterError(
            f'config: "vdatum_bundles" name(s) {illegal} contain an illegal '
            "'/', '\\\\', or '..' (bundle names must be a single path segment)")

    depth_range = raw.get('depth_range', list(DEFAULT_DEPTH_RANGE))
    if (not isinstance(depth_range, (list, tuple)) or len(depth_range) != 2
            or not all(isinstance(v, (int, float)) for v in depth_range)
            or not depth_range[0] < depth_range[1]):
        raise UpdaterError('config: "depth_range" must be [min, max] with min < max')

    nav_raw = raw.get('nav_liveness', {}) or {}
    if not isinstance(nav_raw, dict):
        raise UpdaterError('config: "nav_liveness" must be a mapping')
    nav_unknown = set(nav_raw) - _NAV_KEYS
    if nav_unknown:
        raise UpdaterError(f'config: unknown nav_liveness key(s) {sorted(nav_unknown)}')
    nodes = nav_raw.get('nodes', []) or []
    if not isinstance(nodes, list) or not all(isinstance(n, str) and n for n in nodes):
        raise UpdaterError('config: nav_liveness.nodes must be a list of node names')
    domain_id = nav_raw.get('ros_domain_id')
    if domain_id is not None and (isinstance(domain_id, bool)
                                  or not isinstance(domain_id, int)
                                  or not 0 <= domain_id <= 232):
        raise UpdaterError(
            'config: nav_liveness.ros_domain_id must be an integer in [0, 232]')
    nav = NavLivenessConfig(
        nodes=list(nodes),
        ros_setup=_expand(nav_raw.get('ros_setup')),
        ros_domain_id=domain_id,
        timeout=_require_float(nav_raw.get('timeout', 20.0), 'nav_liveness.timeout'),
    )

    def _float_or_none(key):
        value = raw.get(key)
        return None if value is None else _require_float(value, key)

    return UpdaterConfig(
        corpus_dir=_expand(raw['corpus_dir']),
        store_dir=_expand(raw['store_dir']),
        cells=[str(c) for c in cells],
        region=region,
        bands=[int(b) for b in bands],
        max_cells=int(max_cells),
        catalog_url=str(raw.get('catalog_url', DEFAULT_CATALOG_URL)),
        geoid=_expand(raw.get('geoid')),
        geoid_sha256=(str(raw['geoid_sha256']) if raw.get('geoid_sha256')
                      else None),
        geoid_cdn_base_url=str(
            raw.get('geoid_cdn_base_url', DEFAULT_GEOID_CDN_BASE_URL)),
        vdatum_dir=_expand(raw.get('vdatum_dir')),
        vdatum_bundles=[str(b) for b in bundles],
        vdatum_cdn_base_url=str(
            raw.get('vdatum_cdn_base_url', DEFAULT_VDATUM_CDN_BASE_URL)),
        datum_config=_expand(raw.get('datum_config')),
        lake_datum=_float_or_none('lake_datum'),
        cell_size=_float_or_none('cell_size'),
        depth_range=(float(depth_range[0]), float(depth_range[1])),
        nav_liveness=nav,
        s57_to_geotiff_bin=_expand(raw.get('s57_to_geotiff_bin')),
        import_geotiff_bin=_expand(raw.get('import_geotiff_bin')),
        download_timeout=_require_float(
            raw.get('download_timeout', 300.0), 'download_timeout'),
        export_timeout=_require_float(
            raw.get('export_timeout', 3600.0), 'export_timeout'),
        stage_timeout=_require_float(
            raw.get('stage_timeout', 600.0), 'stage_timeout'),
        commit_timeout=_require_float(
            raw.get('commit_timeout', 120.0), 'commit_timeout'),
    )
