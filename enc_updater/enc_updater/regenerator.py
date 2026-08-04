"""
Wholesale chart-layer regeneration: export, stage, sanity-check, swap.

Pipeline (ADR-0010 D7)::

    s57_to_geotiff <corpus> <export_dir>          # whole-corpus export
    import_geotiff --stage <staged> chart <tif> --level <L>   # per GeoTIFF
    (sanity check; editions.json into <staged>/chart/)
    (nav-down interlock)
    import_geotiff --commit <staged> <store>      # atomic swap

The staged and export directories are created **adjacent to the store**
(``<store parent>/.enc_updater_staging.<pid>``), never under ``/tmp``:
``--commit``'s atomic ``rename(2)`` requires the staged dir on the same
filesystem as the store (cross-device staging fails with EXDEV).
"""

import os
import re
import shutil
import subprocess
from typing import List, Optional, Tuple

from . import nav_liveness
from . import registry
from . import UpdaterError
from .config import UpdaterConfig

# Matches s57_to_geotiff's per-cell export log line, e.g.
#   exported /tmp/out/US5NH02M.tif (1024x768, 51234 cells, scale 1:20000,
#   GGGS level 12 -> import_geotiff --level 12)
EXPORT_LINE_RE = re.compile(
    r'^exported (?P<path>.+?) '
    r'\(\d+x\d+, \d+ cells, scale 1:\d+, '
    r'GGGS level (?P<level>\d+) -> import_geotiff --level \d+\)$')


def run_cmd(argv: List[str], timeout: float, phase: str) -> str:
    """Run one pipeline subprocess; return stdout, raise UpdaterError on failure."""
    try:
        result = subprocess.run(
            argv, capture_output=True, text=True, timeout=timeout)
    except (OSError, subprocess.TimeoutExpired) as e:
        raise UpdaterError(f'{phase}: {argv[0]} failed to run: {e}')
    if result.returncode != 0:
        detail = result.stderr.strip() or result.stdout.strip()
        raise UpdaterError(
            f'{phase}: {" ".join(argv)} exited {result.returncode}: {detail}')
    return result.stdout


def _resolve_tool(override: Optional[str], package: str, executable: str) -> str:
    """Resolve a lib/<pkg>/ CLI via the ament index (or a config override)."""
    if override:
        if not os.path.isfile(override):
            raise UpdaterError(f'config: {executable} override not found: {override}')
        return override
    try:
        from ament_index_python.packages import get_package_prefix
        prefix = get_package_prefix(package)
    except Exception as e:
        raise UpdaterError(
            f'regenerate: cannot locate package "{package}" for {executable} '
            f'({e}) — is the workspace sourced?')
    path = os.path.join(prefix, 'lib', package, executable)
    if not os.path.isfile(path):
        raise UpdaterError(f'regenerate: {executable} not found at {path}')
    return path


def parse_export_log(log_text: str) -> List[Tuple[str, int]]:
    """
    Extract (geotiff path, GGGS level) pairs from the export log.

    Cells the exporter warned about (empty / failed) produce no ``exported``
    line and are therefore never staged.
    """
    pairs = []
    for line in log_text.splitlines():
        match = EXPORT_LINE_RE.match(line.strip())
        if match:
            pairs.append((match.group('path'), int(match.group('level'))))
    return pairs


def _band1_min_max(path: str) -> Tuple[float, float]:
    """Band-1 (ellipsoidal height) min/max of a tile, via the GDAL bindings."""
    from osgeo import gdal
    gdal.UseExceptions()
    try:
        dataset = gdal.Open(path)
        return tuple(dataset.GetRasterBand(1).ComputeRasterMinMax(False))
    except RuntimeError as e:
        raise UpdaterError(f'sanity: cannot read staged tile {path}: {e}')


# Spot-checking this many tiles bounds sanity-check time on large regions
# while still catching systematic corruption (a bad datum/scale poisons every
# tile, not one).
_SANITY_SPOT_CHECK_TILES = 5


def sanity_check(chart_dir: str, depth_range: Tuple[float, float]) -> None:
    """Refuse a corrupt or empty staged layer (ADR-0010 D7): it must never swap in."""
    if not os.path.isdir(chart_dir):
        raise UpdaterError(f'sanity: staged chart dir missing: {chart_dir}')
    tiles = sorted(
        entry for entry in os.listdir(chart_dir) if entry.endswith('.tif'))
    if not tiles:
        raise UpdaterError(f'sanity: no staged tiles in {chart_dir}')
    for tile in tiles:
        path = os.path.join(chart_dir, tile)
        if os.path.getsize(path) == 0:
            raise UpdaterError(f'sanity: staged tile is empty: {path}')
    low, high = depth_range
    for tile in tiles[:_SANITY_SPOT_CHECK_TILES]:
        path = os.path.join(chart_dir, tile)
        tile_min, tile_max = _band1_min_max(path)
        if tile_min < low or tile_max > high:
            raise UpdaterError(
                f'sanity: {path} band-1 range [{tile_min}, {tile_max}] outside '
                f'plausible [{low}, {high}] — refusing to swap')


def _export_args(cfg: UpdaterConfig, export_dir: str) -> List[str]:
    argv = [_resolve_tool(cfg.s57_to_geotiff_bin, 's57_to_geotiff', 's57_to_geotiff'),
            cfg.corpus_dir, export_dir]
    if cfg.geoid:
        argv += ['--geoid', cfg.geoid]
    if cfg.vdatum_dir:
        argv += ['--vdatum-dir', cfg.vdatum_dir]
    if cfg.datum_config:
        argv += ['--datum-config', cfg.datum_config]
    if cfg.lake_datum is not None:
        argv += ['--lake-datum', str(cfg.lake_datum)]
    return argv


def regenerate(cfg: UpdaterConfig, manifest, dry_run: bool = False) -> None:
    """
    Run the full regeneration cycle; on any failure the live layer stands.

    With ``dry_run`` the cycle runs through export, staging, sanity check and
    registry write, then stops before the interlock and commit.
    """
    store_parent = os.path.dirname(os.path.abspath(cfg.store_dir))
    staged = os.path.join(store_parent, f'.enc_updater_staging.{os.getpid()}')
    export_dir = os.path.join(store_parent, f'.enc_updater_export.{os.getpid()}')
    for path in (staged, export_dir):
        if os.path.exists(path):
            raise UpdaterError(
                f'regenerate: work dir already exists: {path} — a previous run '
                'may still be active; remove it only if that run is dead')
    import_tool = _resolve_tool(
        cfg.import_geotiff_bin, 'marine_bathymetry_store', 'import_geotiff')
    os.makedirs(export_dir)
    try:
        log_text = run_cmd(_export_args(cfg, export_dir), cfg.export_timeout, 'export')
        print(log_text, end='')
        pairs = parse_export_log(log_text)
        if not pairs:
            raise UpdaterError('export: no cells exported — refusing to swap')
        for tif_path, level in pairs:
            argv = [import_tool, '--stage', staged, 'chart', tif_path,
                    '--level', str(level)]
            if cfg.cell_size is not None:
                argv += ['--cell-size', str(cfg.cell_size)]
            run_cmd(argv, cfg.stage_timeout, 'stage')
        chart_dir = os.path.join(staged, 'chart')
        sanity_check(chart_dir, cfg.depth_range)
        registry.write_cells(registry.staged_registry_path(chart_dir), manifest)
        if dry_run:
            print('enc_updater: dry run — staged layer validated; '
                  'skipping interlock and commit')
            return
        nav_liveness.check_nav_down(cfg.nav_liveness)
        run_cmd([import_tool, '--commit', staged, cfg.store_dir],
                cfg.commit_timeout, 'commit')
        print(f'enc_updater: chart layer swapped into {cfg.store_dir}')
    finally:
        shutil.rmtree(export_dir, ignore_errors=True)
        shutil.rmtree(staged, ignore_errors=True)
