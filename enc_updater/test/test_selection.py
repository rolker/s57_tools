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


def test_select_filters_on_status_and_coverage_only():
    """Active + intersecting coverage is the whole rule; band never filters."""
    catalog = {
        'US5AAAAA': entry('US5AAAAA', [square(0.2, 0.2, 0.8, 0.8)]),
        'US5BBBBB': entry('US5BBBBB', [square(5.0, 5.0, 6.0, 6.0)]),   # elsewhere
        'US5DDDDD': entry('US5DDDDD', [square(0.2, 0.2, 0.8, 0.8)],
                          status='Cancelled'),
        'US4EEEEE': entry('US4EEEEE', [square(5.0, 5.0, 6.0, 6.0),
                                       square(0.9, 0.9, 1.5, 1.5)]),   # 2nd panel hits
    }
    picked = selection.select_cells(catalog, REGION)
    assert picked == ['US4EEEEE', 'US5AAAAA']


def test_coarse_bands_are_selected():
    """
    Overview/General/Coastal cells must come through.

    They are the ancestors a zoomed-out display upsamples from (uma-ADR-0013
    D5) and the source coverage gaps are filled from (D3); excluding them is
    what left wide views blank.
    """
    panel = [square(0.2, 0.2, 0.8, 0.8)]
    catalog = {
        'US1OVRVW': entry('US1OVRVW', panel),
        'US2GENRL': entry('US2GENRL', panel),
        'US3COAST': entry('US3COAST', panel),
        'US5HARBR': entry('US5HARBR', panel),
    }
    assert selection.select_cells(catalog, REGION) == [
        'US1OVRVW', 'US2GENRL', 'US3COAST', 'US5HARBR']


def test_no_cap_on_a_large_selection():
    """
    A large selection is downloaded, not refused.

    The operator asked for the area, and a whole scale ladder over a real
    survey area is legitimately dozens of cells.
    """
    catalog = {
        f'US5MANY{i:02d}': entry(f'US5MANY{i:02d}', [square(0.2, 0.2, 0.8, 0.8)])
        for i in range(200)
    }
    assert len(selection.select_cells(catalog, REGION)) == 200


def test_empty_selection_is_hard_error():
    """No matching cell must fail loudly, never regenerate an empty layer."""
    catalog = {'US5BBBBB': entry('US5BBBBB', [square(5.0, 5.0, 6.0, 6.0)])}
    with pytest.raises(UpdaterError, match='no Active catalog cell'):
        selection.select_cells(catalog, REGION)
