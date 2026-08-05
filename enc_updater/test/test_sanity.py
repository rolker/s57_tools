"""Band-1 sanity check over real GeoTIFFs (all-nodata tiles are empty, not corrupt)."""

from enc_updater import regenerator
import pytest


def _write_tile(path, value, nodata=-9999.0):
    """Write a 2x2 single-band float32 GeoTIFF with every pixel set to `value`."""
    gdal = pytest.importorskip('osgeo.gdal')
    gdal.UseExceptions()
    driver = gdal.GetDriverByName('GTiff')
    dataset = driver.Create(str(path), 2, 2, 1, gdal.GDT_Float32)
    band = dataset.GetRasterBand(1)
    band.SetNoDataValue(nodata)
    band.Fill(value)
    band.FlushCache()
    dataset = None


def test_band1_min_max_returns_none_for_all_nodata_tile(tmp_path):
    """An entirely-nodata tile yields None (empty grid), not an exception."""
    tile = tmp_path / 'empty.tif'
    _write_tile(tile, value=-9999.0)  # every pixel == nodata
    assert regenerator._band1_min_max(str(tile)) is None


def test_band1_min_max_reads_valid_tile(tmp_path):
    """A tile with real data returns its (min, max) band-1 range."""
    tile = tmp_path / 'data.tif'
    _write_tile(tile, value=-3.5)
    assert regenerator._band1_min_max(str(tile)) == (-3.5, -3.5)


def test_band1_min_max_raises_on_unreadable_tile(tmp_path):
    """A file that is not a readable raster still surfaces as UpdaterError."""
    from enc_updater import UpdaterError
    junk = tmp_path / 'notaraster.tif'
    junk.write_bytes(b'definitely not a GeoTIFF')
    with pytest.raises(UpdaterError, match='cannot read staged tile'):
        regenerator._band1_min_max(str(junk))


def test_sanity_check_skips_all_nodata_tile(tmp_path):
    """An all-nodata spot-checked tile passes sanity (no range to violate)."""
    chart = tmp_path / 'chart'
    chart.mkdir()
    _write_tile(chart / '12_0_0.tif', value=-9999.0)
    # Must not raise: the empty tile is skipped, not treated as corrupt.
    regenerator.sanity_check(str(chart), (-12000.0, 100.0))


def test_sanity_check_flags_out_of_range_tile(tmp_path):
    """A tile with real data outside the plausible range still fails sanity."""
    from enc_updater import UpdaterError
    chart = tmp_path / 'chart'
    chart.mkdir()
    _write_tile(chart / '12_0_0.tif', value=5000.0)  # far above +100 m
    with pytest.raises(UpdaterError, match='outside'):
        regenerator.sanity_check(str(chart), (-12000.0, 100.0))
