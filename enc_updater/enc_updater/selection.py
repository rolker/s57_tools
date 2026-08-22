"""
Region-driven cell selection from the NOAA product catalog.

Cell names are NOAA's implementation detail, not a deployment's: the 2026 ENC
rescheme retired legacy names out from under hard-coded cell lists (issue
#39/#40). What a deployment actually cares about is *coverage of an area*,
and the catalog the updater already fetches every cycle carries per-cell
coverage polygons — so a ``region:`` config derives the cell set fresh each
run. Reschemed, renamed, split, or newly issued cells are picked up
automatically; withdrawn cells drop out of the selection, are pruned from the
corpus (``downloader.prune_corpus``), and the wholesale chart regeneration
forgets them naturally.

Geometry notes: catalog coverage panels are polygons of lat/long vertices.
Type ``E`` panels are exterior boundaries and drive selection; the rare type
``I`` interior-hole panels (4 in the whole 2026-08 catalog) are ignored — a
region falling wholly inside a hole would spuriously select that cell, which
costs one harmless extra download, never a wrong layer. Longitudes are
treated planar (no antimeridian wrap): NOAA ENC coverage sits far from
±180°, and config validation keeps region longitudes in [-180, 180].
"""

from typing import Dict, List, Sequence, Tuple

from . import UpdaterError

Point = Tuple[float, float]


def _point_in_polygon(point: Point, polygon: Sequence[Point]) -> bool:
    """Ray-cast containment test; boundary points may land either way."""
    x, y = point
    inside = False
    j = len(polygon) - 1
    for i in range(len(polygon)):
        xi, yi = polygon[i]
        xj, yj = polygon[j]
        if (yi > y) != (yj > y):
            crossing_x = (xj - xi) * (y - yi) / (yj - yi) + xi
            if x < crossing_x:
                inside = not inside
        j = i
    return inside


def _orientation(p: Point, q: Point, r: Point) -> int:
    """Sign of the cross product (q-p) x (r-p): 1 ccw, -1 cw, 0 collinear."""
    value = (q[0] - p[0]) * (r[1] - p[1]) - (q[1] - p[1]) * (r[0] - p[0])
    if value > 0.0:
        return 1
    if value < 0.0:
        return -1
    return 0


def _on_segment(p: Point, q: Point, r: Point) -> bool:
    """Whether collinear point q lies within segment p-r's bounding box."""
    return (min(p[0], r[0]) <= q[0] <= max(p[0], r[0])
            and min(p[1], r[1]) <= q[1] <= max(p[1], r[1]))


def _segments_intersect(p1: Point, p2: Point, q1: Point, q2: Point) -> bool:
    """Whether segments p1-p2 and q1-q2 intersect (touching counts)."""
    o1 = _orientation(p1, p2, q1)
    o2 = _orientation(p1, p2, q2)
    o3 = _orientation(q1, q2, p1)
    o4 = _orientation(q1, q2, p2)
    if o1 != o2 and o3 != o4:
        return True
    if o1 == 0 and _on_segment(p1, q1, p2):
        return True
    if o2 == 0 and _on_segment(p1, q2, p2):
        return True
    if o3 == 0 and _on_segment(q1, p1, q2):
        return True
    if o4 == 0 and _on_segment(q1, p2, q2):
        return True
    return False


def polygons_intersect(a: Sequence[Point], b: Sequence[Point]) -> bool:
    """
    Whether two simple polygons overlap (containment or edge crossing).

    Covers the three cases: a vertex of one inside the other (either
    direction, which also catches full containment) or any pair of edges
    crossing. Touching boundaries count as intersecting — for cell selection
    an edge-touching cell is a legitimate pick, never a wrong one.
    """
    if any(_point_in_polygon(p, b) for p in a):
        return True
    if any(_point_in_polygon(p, a) for p in b):
        return True
    n_a, n_b = len(a), len(b)
    for i in range(n_a):
        a1, a2 = a[i], a[(i + 1) % n_a]
        for j in range(n_b):
            if _segments_intersect(a1, a2, b[j], b[(j + 1) % n_b]):
                return True
    return False


def select_cells(
    catalog: Dict[str, 'CatalogEntry'],  # noqa: F821 (see downloader)
    region: Sequence[Point],
) -> List[str]:
    """
    Every Active catalog cell whose coverage intersects ``region``.

    **At every usage band.** The coarse bands (Overview/General/Coastal) are
    the ancestors `uma-ADR-0013` D5 upsamples from for "always a valid, if
    blurry, picture", and the source D3's corollary fills coverage gaps from;
    excluding them is what leaves a zoomed-out display blank. They also cost
    almost nothing: a survey-area region matches a handful of coarse cells
    against dozens of approach and harbour ones.

    Raises UpdaterError on an empty selection: for a non-empty region that
    means the region is wrong (or NOAA coverage genuinely ends there), and
    silently regenerating an empty chart layer is exactly what the sanity
    check exists to refuse. The result is sorted for run-to-run determinism.
    """
    selected = []
    for name in catalog:
        entry = catalog[name]
        if entry.status != 'Active':
            continue
        if any(polygons_intersect(panel, region) for panel in entry.panels):
            selected.append(name)
    if not selected:
        raise UpdaterError(
            'selection: no Active catalog cell intersects the configured '
            'region — check the region coordinates (lon/lat order?)')
    return sorted(selected)
