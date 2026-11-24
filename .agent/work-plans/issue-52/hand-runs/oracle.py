"""GDAL's own reading of the chart cells, independent of the layer.

A point is expected lethal if it is in an LNDARE polygon, or in a DEPARE or
DRGARE polygon whose DRVAL1 is below the minimum depth. It is expected deep
if it is in a DEPARE/DRGARE with DRVAL1 at or above the maximum caution
depth and in no LNDARE.

The map frame is east-north-up at LAT0, LON0 on WGS84, as the hand run's
static earth -> map transform.
"""
import math

from osgeo import ogr

LAT0, LON0 = 43.07, -70.75
A, F = 6378137.0, 1 / 298.257223563
E2 = F * (2 - F)


def _ecef(lat, lon, h=0.0):
    lat, lon = math.radians(lat), math.radians(lon)
    n = A / math.sqrt(1 - E2 * math.sin(lat) ** 2)
    return ((n + h) * math.cos(lat) * math.cos(lon),
            (n + h) * math.cos(lat) * math.sin(lon),
            (n * (1 - E2) + h) * math.sin(lat))


_O = _ecef(LAT0, LON0)
_la, _lo = math.radians(LAT0), math.radians(LON0)
_E = (-math.sin(_lo), math.cos(_lo), 0.0)
_N = (-math.sin(_la) * math.cos(_lo), -math.sin(_la) * math.sin(_lo), math.cos(_la))
_U = (math.cos(_la) * math.cos(_lo), math.cos(_la) * math.sin(_lo), math.sin(_la))


def map_to_ll(x, y):
    """Map (ENU) x, y at z = 0 to latitude, longitude (degrees)."""
    p = [_O[i] + x * _E[i] + y * _N[i] for i in range(3)]
    lon = math.atan2(p[1], p[0])
    r = math.hypot(p[0], p[1])
    lat = math.atan2(p[2], r * (1 - E2))
    for _ in range(6):
        n = A / math.sqrt(1 - E2 * math.sin(lat) ** 2)
        h = r / math.cos(lat) - n
        lat = math.atan2(p[2], r * (1 - E2 * n / (n + h)))
    return math.degrees(lat), math.degrees(lon)


def ll_to_map(lat, lon):
    p = _ecef(lat, lon)
    d = [p[i] - _O[i] for i in range(3)]
    return sum(d[i] * _E[i] for i in range(3)), sum(d[i] * _N[i] for i in range(3))


class Chart:
    def __init__(self, cells, minimum_depth, maximum_caution_depth):
        self.land = ogr.Geometry(ogr.wkbMultiPolygon)
        self.shallow = ogr.Geometry(ogr.wkbMultiPolygon)
        self.deep = ogr.Geometry(ogr.wkbMultiPolygon)
        self.coverage = ogr.Geometry(ogr.wkbMultiPolygon)
        self.cells = cells
        for path in cells:
            ds = ogr.Open(path)
            for name, target in (('LNDARE', 'land'),):
                lyr = ds.GetLayerByName(name)
                for f in lyr:
                    g = f.GetGeometryRef()
                    if g and g.GetGeometryType() in (ogr.wkbPolygon, ogr.wkbMultiPolygon):
                        self.land = self.land.Union(g)
            for name in ('DEPARE', 'DRGARE'):
                lyr = ds.GetLayerByName(name)
                if lyr is None:
                    continue
                for f in lyr:
                    g = f.GetGeometryRef()
                    if not g or g.GetGeometryType() not in (ogr.wkbPolygon, ogr.wkbMultiPolygon):
                        continue
                    d = f.GetField('DRVAL1')
                    if d is None:
                        continue
                    if d < minimum_depth:
                        self.shallow = self.shallow.Union(g)
                    elif d >= maximum_caution_depth:
                        self.deep = self.deep.Union(g)
            cov = ds.GetLayerByName('M_COVR')
            for f in cov:
                if f.GetField('CATCOV') == 1:
                    self.coverage = self.coverage.Union(f.GetGeometryRef())

    def classify(self, lat, lon):
        """'lethal', 'deep', 'other' (caution band or unclassified) or 'outside'."""
        pt = ogr.Geometry(ogr.wkbPoint)
        pt.AddPoint_2D(lon, lat)
        if not self.coverage.Contains(pt):
            return 'outside'
        if self.land.Contains(pt) or self.shallow.Contains(pt):
            return 'lethal'
        if self.deep.Contains(pt):
            return 'deep'
        return 'other'
