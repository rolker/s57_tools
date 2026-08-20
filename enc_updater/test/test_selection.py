"""Region-driven cell selection: geometry, filters, and the fail-loud rails."""

from enc_updater import selection
from enc_updater import UpdaterError
from enc_updater.downloader import CatalogEntry
import pytest

# A unit square region around the origin, as the bbox corner polygon
# config._parse_region would produce.
REGION = [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 1.0)]


def entry(name, panels, status='Active'):
    """Build a minimal CatalogEntry carrying only what selection reads."""
    return CatalogEntry(
        name=name, edition=1, update=0,
        url=f'https://example.invalid/{name}.zip', size=None,
        status=status, panels=panels)


def square(x0, y0, x1, y1):
    """Build an axis-aligned square panel."""
    return [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]


def test_polygons_intersect_cases():
    """Containment (both directions), edge crossing, and disjointness."""
    small = square(0.4, 0.4, 0.6, 0.6)
    big = square(-1.0, -1.0, 2.0, 2.0)
    crossing = square(0.5, -0.5, 1.5, 0.5)
    disjoint = square(2.0, 2.0, 3.0, 3.0)
    assert selection.polygons_intersect(small, REGION)      # small inside region
    assert selection.polygons_intersect(REGION, small)
    assert selection.polygons_intersect(big, REGION)        # region inside big
    assert selection.polygons_intersect(crossing, REGION)   # edges cross
    assert not selection.polygons_intersect(disjoint, REGION)


def test_edge_crossing_without_contained_vertices():
    """A plus-sign overlap: edges cross but neither polygon holds a vertex of the other."""
    tall_thin = square(0.4, -1.0, 0.6, 2.0)
    assert selection.polygons_intersect(tall_thin, REGION)


def test_usage_band():
    """Band comes from the name's third character; malformed names have none."""
    assert selection.usage_band('US5PSMBE') == 5
    assert selection.usage_band('US4NH1BD') == 4
    assert selection.usage_band('USXBAD') is None
    assert selection.usage_band('U') is None


def test_select_filters_band_status_and_coverage():
    """Only Active cells in wanted bands with intersecting coverage select."""
    catalog = {
        'US5AAAAA': entry('US5AAAAA', [square(0.2, 0.2, 0.8, 0.8)]),
        'US5BBBBB': entry('US5BBBBB', [square(5.0, 5.0, 6.0, 6.0)]),   # elsewhere
        'US2CCCCC': entry('US2CCCCC', [square(0.2, 0.2, 0.8, 0.8)]),   # band 2
        'US5DDDDD': entry('US5DDDDD', [square(0.2, 0.2, 0.8, 0.8)],
                          status='Cancelled'),
        'US4EEEEE': entry('US4EEEEE', [square(5.0, 5.0, 6.0, 6.0),
                                       square(0.9, 0.9, 1.5, 1.5)]),   # 2nd panel hits
    }
    picked = selection.select_cells(catalog, REGION, bands=(4, 5, 6), max_cells=50)
    assert picked == ['US4EEEEE', 'US5AAAAA']


def test_empty_selection_is_hard_error():
    """No matching cell must fail loudly, never regenerate an empty layer."""
    catalog = {'US5BBBBB': entry('US5BBBBB', [square(5.0, 5.0, 6.0, 6.0)])}
    with pytest.raises(UpdaterError, match='no Active catalog cell'):
        selection.select_cells(catalog, REGION, bands=(4, 5, 6), max_cells=50)


def test_max_cells_cap_is_hard_error():
    """A selection over the cap names the count — a fat-fingered region fails fast."""
    catalog = {
        f'US5CAP{i:02d}': entry(f'US5CAP{i:02d}', [square(0.2, 0.2, 0.8, 0.8)])
        for i in range(4)
    }
    with pytest.raises(UpdaterError, match='matches 4 cells, over the max_cells cap'):
        selection.select_cells(catalog, REGION, bands=(4, 5, 6), max_cells=3)
