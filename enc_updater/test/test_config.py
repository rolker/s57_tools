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
