"""Config loading: bad values surface as UpdaterError, not raw tracebacks."""

from enc_updater import UpdaterError
from enc_updater.config import load_config
import pytest
import yaml


def _write(tmp_path, **overrides):
    """Write a minimal valid region config, applying overrides, return its path."""
    doc = {
        'corpus_dir': str(tmp_path / 'corpus'),
        'store_dir': str(tmp_path / 'store'),
        'cells': ['US5NH02M'],
    }
    doc.update(overrides)
    path = tmp_path / 'region.yaml'
    path.write_text(yaml.safe_dump(doc))
    return str(path)


def test_valid_config_loads(tmp_path):
    """A well-formed config parses into an UpdaterConfig."""
    cfg = load_config(_write(tmp_path, commit_timeout=90, lake_datum=-1.5))
    assert cfg.commit_timeout == 90.0
    assert cfg.lake_datum == -1.5


@pytest.mark.parametrize('key', [
    'download_timeout', 'export_timeout', 'stage_timeout', 'commit_timeout',
    'lake_datum', 'cell_size',
])
def test_non_numeric_float_field_is_clean_error(tmp_path, key):
    """A mistyped numeric field raises UpdaterError, not an uncaught ValueError."""
    with pytest.raises(UpdaterError, match=f'"{key}" must be a number'):
        load_config(_write(tmp_path, **{key: 'fast'}))


def test_non_numeric_nav_timeout_is_clean_error(tmp_path):
    """A mistyped nav_liveness.timeout raises UpdaterError with the nested key."""
    with pytest.raises(UpdaterError, match='"nav_liveness.timeout" must be a number'):
        load_config(_write(tmp_path, nav_liveness={'timeout': 'soon'}))


@pytest.mark.parametrize('name', ['../evil', 'sub/dir', 'back\\slash', '..'])
def test_vdatum_bundle_path_shape_rejected(tmp_path, name):
    """A bundle name with a path separator or `..` is a clean config error."""
    with pytest.raises(UpdaterError, match='illegal'):
        load_config(_write(tmp_path, vdatum_bundles=[name]))


def test_vdatum_bundle_valid_name_accepted(tmp_path):
    """A well-formed single-segment bundle name loads without complaint."""
    cfg = load_config(_write(tmp_path, vdatum_bundles=['MENHMAgome23_8301']))
    assert cfg.vdatum_bundles == ['MENHMAgome23_8301']


def _write_region(tmp_path, region, **overrides):
    """Write a config using `region` instead of `cells`, return its path."""
    doc = {
        'corpus_dir': str(tmp_path / 'corpus'),
        'store_dir': str(tmp_path / 'store'),
        'region': region,
    }
    doc.update(overrides)
    path = tmp_path / 'region.yaml'
    path.write_text(yaml.safe_dump(doc))
    return str(path)


def test_region_bbox_expands_to_corner_polygon(tmp_path):
    """A 4-number bbox normalizes to its corner polygon, ccw from lower-left."""
    cfg = load_config(_write_region(tmp_path, [-70.85, 42.93, -70.55, 43.11]))
    assert cfg.region == [(-70.85, 42.93), (-70.55, 42.93),
                          (-70.55, 43.11), (-70.85, 43.11)]
    assert cfg.cells == []
    assert cfg.bands == [4, 5, 6]
    assert cfg.max_cells == 50


def test_region_polygon_accepted(tmp_path):
    """A 3+-vertex [lon, lat] polygon loads verbatim."""
    poly = [[-70.7, 42.95], [-70.55, 42.95], [-70.6, 43.05]]
    cfg = load_config(_write_region(tmp_path, poly))
    assert cfg.region == [(-70.7, 42.95), (-70.55, 42.95), (-70.6, 43.05)]


def test_cells_and_region_together_rejected(tmp_path):
    """Exactly one selection mode: both keys at once is a clean error."""
    with pytest.raises(UpdaterError, match='exactly one'):
        load_config(_write(tmp_path, region=[-70.85, 42.93, -70.55, 43.11]))


def test_neither_cells_nor_region_rejected(tmp_path):
    """Exactly one selection mode: neither key is a clean error."""
    doc = {'corpus_dir': str(tmp_path / 'c'), 'store_dir': str(tmp_path / 's')}
    path = tmp_path / 'region.yaml'
    path.write_text(yaml.safe_dump(doc))
    with pytest.raises(UpdaterError, match='exactly one'):
        load_config(str(path))


@pytest.mark.parametrize('key,value', [('bands', [4, 5]), ('max_cells', 10)])
def test_region_only_keys_rejected_with_cells(tmp_path, key, value):
    """Reject bands / max_cells alongside `cells` — they would silently no-op."""
    with pytest.raises(UpdaterError, match='region-driven selection only'):
        load_config(_write(tmp_path, **{key: value}))


@pytest.mark.parametrize('region', [
    [-70.55, 42.93, -70.85, 43.11],          # lon_min > lon_max
    [-70.85, 43.11, -70.55, 42.93],          # lat_min > lat_max
    [-70.85, 42.93],                         # too few numbers
    [[-70.7, 42.95], [-70.55, 42.95]],       # 2-vertex polygon
    [[-70.7, 42.95], [-70.55, 42.95], [-70.6, 'north']],  # non-numeric
    [[-70.7, 42.95], [-70.55, 92.0], [-70.6, 43.0]],      # lat out of range
    'everywhere',                            # wrong type entirely
])
def test_malformed_region_rejected(tmp_path, region):
    """Every malformed region shape surfaces as a clean UpdaterError."""
    with pytest.raises(UpdaterError, match='region'):
        load_config(_write_region(tmp_path, region))


@pytest.mark.parametrize('bands', [[], [0], [7], ['4'], [True], 'all'])
def test_malformed_bands_rejected(tmp_path, bands):
    """Reject bands that are not a non-empty list of integer usage bands 1-6."""
    with pytest.raises(UpdaterError, match='"bands"'):
        load_config(_write_region(
            tmp_path, [-70.85, 42.93, -70.55, 43.11], bands=bands))


@pytest.mark.parametrize('max_cells', [0, -3, 'many', True])
def test_malformed_max_cells_rejected(tmp_path, max_cells):
    """max_cells must be a positive integer."""
    with pytest.raises(UpdaterError, match='"max_cells"'):
        load_config(_write_region(
            tmp_path, [-70.85, 42.93, -70.55, 43.11], max_cells=max_cells))
