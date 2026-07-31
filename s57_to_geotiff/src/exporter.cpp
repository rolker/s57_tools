#include "exporter.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <limits>
#include <memory>

#include "cpl_conv.h"
#include "gdal_alg.h"
#include "gdal_priv.h"
#include "ogrsf_frmts.h"

#include "marine_autonomy/gggs.h"
#include "marine_charts/s57_catalog.h"
#include "marine_charts/s57_dataset.h"
#include "marine_vertical_datum/datum_config.hpp"
#include "marine_vertical_datum/vdatum_query.hpp"

namespace s57_to_geotiff
{

namespace
{

constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

// ADR-0010 D7: 0.5 mm-at-scale resolvable ground distance drives GGGS level
// selection. NOTE this is a different quantity than
// S57Dataset::recommendedResolution()'s 0.3125 mm, which is the S52 minimum
// display-pixel size (864 lines / 270 mm) governing on-screen legibility. See
// the work plan's "Scale->level constant" note.
constexpr double kResolvableGroundFraction = 0.0005;

// Per-side raster dimension cap: a guard against a malformed scale/extent
// demanding an unbounded allocation, not a normal operational limit.
constexpr double kMaxRasterDim = 200000.0;

// A charted sounding with no CATZOC zone would otherwise get sigma 0.0, which a
// consumer could read as false certainty. Floor it at the CATZOC A1 base (0.5 m)
// so band 2 is never a hard zero for a real measurement.
constexpr double kMinSoundingSigma = 0.5;

// S-57 object class labels used here.
constexpr int kObjlDepare = 42;
constexpr int kObjlDrgare = 46;
constexpr int kObjlSoundg = 129;
constexpr int kObjlMcovr = 302;

// Iterate every feature across every layer of `dataset`, calling fn(feature).
// The feature is destroyed after fn returns, so fn must not retain it.
template<class Fn>
void forEachFeature(GDALDataset & dataset, Fn && fn)
{
  for (int li = 0; li < dataset.GetLayerCount(); ++li) {
    OGRLayer * layer = dataset.GetLayer(li);
    if (!layer) {
      continue;
    }
    layer->ResetReading();
    OGRFeature * feature = nullptr;
    while ((feature = layer->GetNextFeature()) != nullptr) {
      fn(feature);
      OGRFeature::DestroyFeature(feature);
    }
  }
}

int featureObjl(const OGRFeature * feature)
{
  int i = feature->GetFieldIndex("OBJL");
  return i < 0 ? -1 : feature->GetFieldAsInteger(i);
}

std::optional<double> fieldDouble(const OGRFeature * feature, const char * name)
{
  int i = feature->GetFieldIndex(name);
  if (i < 0 || !feature->IsFieldSetAndNotNull(i)) {
    return std::nullopt;
  }
  return feature->GetFieldAsDouble(i);
}

// The reconstructed CATZOC zones, kept alive for point-in-zone tests.
class CatzocZones
{
public:
  explicit CatzocZones(const std::vector<marine_charts::CatzocZone> & zones)
  {
    for (const auto & z : zones) {
      OGRGeometry * geometry = nullptr;
      if (OGRGeometryFactory::createFromWkb(
          z.wkb.data(), nullptr, &geometry, static_cast<int>(z.wkb.size())) == OGRERR_NONE &&
        geometry != nullptr)
      {
        geoms_.push_back({geometry, z.catzoc});
      }
    }
  }
  ~CatzocZones()
  {
    for (auto & g : geoms_) {
      OGRGeometryFactory::destroyGeometry(g.first);
    }
  }
  CatzocZones(const CatzocZones &) = delete;
  CatzocZones & operator=(const CatzocZones &) = delete;

  // CATZOC code covering (lon, lat), or 0 when no zone contains it.
  int at(double lon, double lat) const
  {
    OGRPoint point(lon, lat);
    for (const auto & g : geoms_) {
      if (g.first->Contains(&point)) {
        return g.second;
      }
    }
    return 0;
  }

private:
  std::vector<std::pair<OGRGeometry *, int>> geoms_;
};

std::string wgs84Wkt()
{
  OGRSpatialReference srs;
  srs.SetWellKnownGeogCS("WGS84");
  char * wkt = nullptr;
  srs.exportToWkt(&wkt);
  std::string out = wkt ? wkt : "";
  CPLFree(wkt);
  return out;
}

// Strip a single trailing extension (".000") from an S-57 cell label to form
// the output basename. S-57 cell names are unique across a corpus, so the
// stripped base is collision-free.
std::string baseLabel(const std::string & label)
{
  auto dot = label.rfind('.');
  return dot == std::string::npos ? label : label.substr(0, dot);
}

}  // namespace

double catzocSigma(int catzoc, double depth)
{
  const double d = std::abs(depth);
  switch (catzoc) {
    case 1:            // A1
      return 0.5 + 0.01 * d;
    case 2:            // A2
    case 3:            // B
      return 1.0 + 0.02 * d;
    case 4:            // C
      return 2.0 + 0.05 * d;
    case 5:            // D
    case 6:            // U
      return kCatzocLargeSigma;
    default:           // no CATZOC data -> no floor contribution
      return 0.0;
  }
}

bool exportCell(
  GDALDataset & dataset, double chart_scale, const DatumFn & datum,
  const std::vector<OGRGeometry *> & clip_geoms, const std::string & out_path,
  std::string & error, CellExport * stats)
{
  // A malformed cell can carry a zero/negative scale; guard before it reaches
  // gggs::Level::fromCellSize (log2 of +inf is UB) and before any allocation.
  if (!(chart_scale > 0.0)) {
    error = "non-positive chart scale";
    return false;
  }

  // --- CATZOC zones for the sigma floor -------------------------------------
  CatzocZones zones(marine_charts::readCatzocZones(&dataset));

  // --- Cell extent: prefer M_COVR, fall back to all depth features ----------
  OGREnvelope coverage;
  OGREnvelope fallback;
  forEachFeature(
    dataset, [&](OGRFeature * feature) {
      OGRGeometry * geometry = feature->GetGeometryRef();
      if (!geometry) {
        return;
      }
      OGREnvelope env;
      geometry->getEnvelope(&env);
      const int objl = featureObjl(feature);
      if (objl == kObjlMcovr) {
        int ci = feature->GetFieldIndex("CATCOV");
        if (ci >= 0 && feature->IsFieldSetAndNotNull(ci) &&
          feature->GetFieldAsInteger(ci) != 1)
        {
          return;                     // not coverage-available
        }
        coverage.Merge(env);
      } else if (objl == kObjlDepare || objl == kObjlDrgare || objl == kObjlSoundg) {
        fallback.Merge(env);
      }
    });

  const OGREnvelope & extent = coverage.IsInit() ? coverage : fallback;
  if (!extent.IsInit()) {
    error = "no M_COVR or depth features to bound the raster";
    return false;
  }

  // --- Raster geometry from the GGGS level for this scale --------------------
  const auto level = gggs::Level::fromCellSize(
    static_cast<float>(chart_scale * kResolvableGroundFraction));
  const double pixel = level.cellAngularSpan();          // degrees
  if (!(pixel > 0.0)) {
    error = "non-positive pixel size from GGGS level";
    return false;
  }
  const double min_lon = extent.MinX;
  const double max_lat = extent.MaxY;
  // Compute the dimensions in double and cap them before narrowing to int: a
  // pathological extent/pixel ratio would otherwise overflow the int cast (UB)
  // and demand an unbounded allocation.
  const double cols = std::ceil((extent.MaxX - extent.MinX) / pixel);
  const double rows = std::ceil((extent.MaxY - extent.MinY) / pixel);
  if (!(cols >= 0.0 && rows >= 0.0) || cols > kMaxRasterDim || rows > kMaxRasterDim) {
    error = "raster dimensions exceed the safety cap (malformed scale or extent)";
    return false;
  }
  const int width = std::max(1, static_cast<int>(cols));
  const int height = std::max(1, static_cast<int>(rows));
  const std::size_t n = static_cast<std::size_t>(width) * static_cast<std::size_t>(height);

  double gt[6] = {min_lon, pixel, 0.0, max_lat, 0.0, -pixel};
  const std::string wkt = wgs84Wkt();

  GDALDriver * mem_driver = GetGDALDriverManager()->GetDriverByName("MEM");
  if (!mem_driver) {
    error = "GDAL MEM driver unavailable";
    return false;
  }

  // Working raster: band 1 = depth below chart datum (positive-down), band 2 =
  // sigma. Both start as NaN so untouched pixels stay no-data.
  std::unique_ptr<GDALDataset> work(mem_driver->Create("", width, height, 2, GDT_Float64, nullptr));
  if (!work) {
    error = "failed to allocate the working raster";
    return false;
  }
  work->SetGeoTransform(gt);
  work->SetProjection(wkt.c_str());
  work->GetRasterBand(1)->Fill(kNaN);
  work->GetRasterBand(2)->Fill(kNaN);

  // --- Rasterize DEPARE/DRGARE polygons; collect SOUNDG points --------------
  struct Sounding
  {
    int col;
    int row;
    double depth;
    double sigma;
  };
  std::vector<Sounding> soundings;

  bool rasterize_ok = true;
  forEachFeature(
    dataset, [&](OGRFeature * feature) {
      OGRGeometry * geometry = feature->GetGeometryRef();
      if (!geometry) {
        return;
      }
      const int objl = featureObjl(feature);
      if (objl == kObjlDepare || objl == kObjlDrgare) {
        auto d1 = fieldDouble(feature, "DRVAL1");
        auto d2 = fieldDouble(feature, "DRVAL2");
        if (!d1 || !d2) {
          return;                     // no band midpoint without both limits
        }
        const double depth = (*d1 + *d2) / 2.0;
        const double half_band = std::max(0.0, (*d2 - *d1) / 2.0);
        OGREnvelope env;
        geometry->getEnvelope(&env);
        // CATZOC is sampled once at the polygon's bbox centroid and burned across
        // the whole area. A DEPARE that straddles two M_QUAL zones therefore gets
        // a single zone's sigma; splitting per-zone would need a polygon
        // intersection pass (left as future work).
        const int cz = zones.at((env.MinX + env.MaxX) / 2.0, (env.MinY + env.MaxY) / 2.0);
        const double sigma = std::max(half_band, catzocSigma(cz, depth));

        int bands[2] = {1, 2};
        double burn[2] = {depth, sigma};
        OGRGeometryH gh = OGRGeometry::ToHandle(geometry);
        if (GDALRasterizeGeometries(
            GDALDataset::ToHandle(work.get()), 2, bands, 1, &gh, nullptr, nullptr, burn,
            nullptr, nullptr, nullptr) != CE_None)
        {
          rasterize_ok = false;         // a dropped burn would silently lose depth pixels
        }
      } else if (objl == kObjlSoundg) {
        // S-57 encodes each sounding's depth as the point geometry's Z ordinate
        // (positive-down), not a VALSOU attribute, so we read getZ() here. This
        // deviates from plan step 5's "VALSOU" wording; geometry Z is the correct
        // S-57 source for SOUNDG.
        const OGRwkbGeometryType type = wkbFlatten(geometry->getGeometryType());
        auto add = [&](double x, double y, double z) {
          const int col = static_cast<int>(std::floor((x - min_lon) / pixel));
          const int row = static_cast<int>(std::floor((max_lat - y) / pixel));
          if (col < 0 || col >= width || row < 0 || row >= height) {
            return;
          }
          const int cz = zones.at(x, y);
          const double sounding_sigma = std::max(catzocSigma(cz, z), kMinSoundingSigma);
          soundings.push_back({col, row, z, sounding_sigma});
        };
        if (type == wkbPoint) {
          const OGRPoint * p = geometry->toPoint();
          add(p->getX(), p->getY(), p->getZ());
        } else if (type == wkbMultiPoint) {
          const OGRMultiPoint * mp = geometry->toMultiPoint();
          for (int gi = 0; gi < mp->getNumGeometries(); ++gi) {
            const OGRPoint * p = mp->getGeometryRef(gi);
            add(p->getX(), p->getY(), p->getZ());
          }
        }
      }
    });

  if (!rasterize_ok) {
    error = "failed to rasterize a DEPARE/DRGARE polygon (depth pixels dropped)";
    return false;
  }

  // --- Read working bands into memory ---------------------------------------
  std::vector<double> depth_bd(n);
  std::vector<double> sigma(n);
  if (work->GetRasterBand(1)->RasterIO(
      GF_Read, 0, 0, width, height, depth_bd.data(), width, height, GDT_Float64, 0, 0) != CE_None ||
    work->GetRasterBand(2)->RasterIO(
      GF_Read, 0, 0, width, height, sigma.data(), width, height, GDT_Float64, 0, 0) != CE_None)
  {
    error = "failed to read rasterized bands";
    return false;
  }

  // Soundings overwrite the polygon band (a discrete measurement is authoritative).
  for (const auto & s : soundings) {
    const std::size_t idx = static_cast<std::size_t>(s.row) * width + s.col;
    depth_bd[idx] = s.depth;
    sigma[idx] = s.sigma;
  }

  // --- Clip: drop pixels covered by any finer-scale footprint ---------------
  if (!clip_geoms.empty()) {
    std::unique_ptr<GDALDataset> mask(
      mem_driver->Create("", width, height, 1, GDT_Byte, nullptr));
    if (!mask) {
      error = "failed to allocate the clip mask";
      return false;
    }
    mask->SetGeoTransform(gt);
    mask->SetProjection(wkt.c_str());
    mask->GetRasterBand(1)->Fill(0);

    std::vector<OGRGeometryH> handles;
    std::vector<double> burns;
    handles.reserve(clip_geoms.size());
    burns.reserve(clip_geoms.size());
    for (OGRGeometry * g : clip_geoms) {
      if (g) {
        handles.push_back(OGRGeometry::ToHandle(g));
        burns.push_back(1.0);
      }
    }
    if (!handles.empty()) {
      int mask_band[1] = {1};
      if (GDALRasterizeGeometries(
          GDALDataset::ToHandle(mask.get()), 1, mask_band, static_cast<int>(handles.size()),
          handles.data(), nullptr, nullptr, burns.data(), nullptr, nullptr, nullptr) != CE_None)
      {
        // A failed clip mask would under-clip: stale coarser depth could leak
        // through a finer cell's footprint. Fail the cell rather than emit it.
        error = "failed to rasterize the finer-scale clip mask";
        return false;
      }
      std::vector<unsigned char> covered(n);
      if (mask->GetRasterBand(1)->RasterIO(
          GF_Read, 0, 0, width, height, covered.data(), width, height, GDT_Byte, 0, 0) != CE_None)
      {
        error = "failed to read clip mask";
        return false;
      }
      for (std::size_t i = 0; i < n; ++i) {
        if (covered[i]) {
          depth_bd[i] = kNaN;
        }
      }
    }
  }

  // --- Datum conversion per pixel: ellipsoidal height = datum_z - depth ------
  std::vector<double> out_depth(n, kNaN);
  std::vector<double> out_sigma(n, kNaN);
  long written = 0;
  for (int row = 0; row < height; ++row) {
    const double lat = max_lat - (row + 0.5) * pixel;
    for (int col = 0; col < width; ++col) {
      const std::size_t idx = static_cast<std::size_t>(row) * width + col;
      if (!std::isfinite(depth_bd[idx])) {
        continue;
      }
      const double lon = min_lon + (col + 0.5) * pixel;
      const std::optional<double> datum_z = datum(lat, lon);
      if (!datum_z) {
        continue;                     // no resolvable datum -> no-data
      }
      out_depth[idx] = *datum_z - depth_bd[idx];
      out_sigma[idx] = sigma[idx];
      ++written;
    }
  }

  // --- Write the GeoTIFF -----------------------------------------------------
  GDALDriver * tif_driver = GetGDALDriverManager()->GetDriverByName("GTiff");
  if (!tif_driver) {
    error = "GDAL GTiff driver unavailable";
    return false;
  }
  std::unique_ptr<GDALDataset> out(
    tif_driver->Create(out_path.c_str(), width, height, 2, GDT_Float64, nullptr));
  if (!out) {
    error = "failed to create " + out_path;
    return false;
  }
  out->SetGeoTransform(gt);
  out->SetProjection(wkt.c_str());
  if (out->GetRasterBand(1)->RasterIO(
      GF_Write, 0, 0, width, height, out_depth.data(), width, height, GDT_Float64, 0, 0) != CE_None ||
    out->GetRasterBand(2)->RasterIO(
      GF_Write, 0, 0, width, height, out_sigma.data(), width, height, GDT_Float64, 0, 0) != CE_None)
  {
    error = "failed to write raster bands to " + out_path;
    return false;
  }
  out->GetRasterBand(1)->SetNoDataValue(kNaN);
  out->GetRasterBand(2)->SetNoDataValue(kNaN);
  out->GetRasterBand(1)->SetDescription("depth (WGS84 ellipsoidal height, m, up-positive)");
  out->GetRasterBand(2)->SetDescription("sigma (1-sigma vertical uncertainty, m)");

  if (stats) {
    stats->width = width;
    stats->height = height;
    stats->level = level.level();
    stats->written = written;
  }
  return true;
}

namespace
{

std::shared_ptr<GDALDataset> openVector(const std::string & path)
{
  return std::shared_ptr<GDALDataset>(
    static_cast<GDALDataset *>(
      GDALOpenEx(path.c_str(), GDAL_OF_VECTOR, nullptr, nullptr, nullptr)),
    [](GDALDataset * d) {if (d) {GDALClose(d);}});
}

std::vector<OGRGeometry *> readFootprints(GDALDataset & dataset)
{
  std::vector<OGRGeometry *> footprints;
  forEachFeature(
    dataset, [&](OGRFeature * feature) {
      if (featureObjl(feature) != kObjlMcovr) {
        return;
      }
      int ci = feature->GetFieldIndex("CATCOV");
      if (ci >= 0 && feature->IsFieldSetAndNotNull(ci) &&
        feature->GetFieldAsInteger(ci) != 1)
      {
        return;                       // only coverage-available polygons clip
      }
      OGRGeometry * geometry = feature->GetGeometryRef();
      if (geometry) {
        footprints.push_back(geometry->clone());
      }
    });
  return footprints;
}

DatumFn buildDatum(const ExporterOptions & opts, std::ostream & log)
{
  marine_vertical_datum::VDatumQueryFn vquery;
  if (!opts.geoid_grid.empty()) {
    marine_vertical_datum::VDatumConfig cfg;
    cfg.geoid_grid = opts.geoid_grid;
    cfg.vdatum_grid_dir = opts.vdatum_dir;
    vquery = marine_vertical_datum::make_vdatum_query(
      cfg, [&log](const std::string & m) {log << "[vdatum] " << m << "\n";});
    if (!vquery) {
      log << "warning: VDatum query unavailable; relying on datum config / lake datum\n";
    }
  }

  std::vector<marine_vertical_datum::DatumEntry> entries;
  if (!opts.datum_config.empty()) {
    entries = marine_vertical_datum::load_datum_config(opts.datum_config);
  }

  const std::optional<double> lake = opts.lake_datum;
  return [vquery, entries, lake](double lat, double lon) -> std::optional<double> {
           std::optional<marine_vertical_datum::VDatumResult> vdatum;
           if (vquery) {
             vdatum = vquery(lat, lon);
           }
           auto resolved = marine_vertical_datum::resolve_datum(
             lat, lon, lake, std::nullopt, vdatum, entries);
           if (resolved) {
             return resolved->chart_datum_z;
           }
           return std::nullopt;
         };
}

}  // namespace

int runExport(const ExporterOptions & opts, std::ostream & log)
{
  std::error_code ec;
  std::filesystem::create_directories(opts.out_dir, ec);
  if (ec) {
    log << "error: cannot create output directory " << opts.out_dir << ": "
        << ec.message() << "\n";
    return -1;                          // fatal setup error (see header contract)
  }

  marine_charts::S57Catalog catalog(opts.enc_root);
  auto datasets = catalog.intersectingCharts(-90.0, -180.0, 90.0, 180.0);
  if (datasets.empty()) {
    log << "no charts found under " << opts.enc_root << "\n";
    return 0;
  }

  const DatumFn datum = buildDatum(opts, log);

  // Pass A: scale + coverage footprints per cell (footprints owned for the run).
  struct Cell
  {
    std::shared_ptr<marine_charts::S57Dataset> ds;
    double scale;
    std::vector<OGRGeometry *> footprints;
  };
  std::vector<Cell> cells;
  cells.reserve(datasets.size());
  for (auto & ds : datasets) {
    auto gdal = openVector(ds->filePath());
    if (!gdal) {
      log << "warning: cannot open " << ds->filePath() << "; skipping\n";
      continue;
    }
    cells.push_back({ds, ds->chartScale(), readFootprints(*gdal)});
  }

  // Pass B: export each cell, clipped by every strictly-finer cell's footprints
  // (finer = smaller scale denominator; largest scale governs, ADR-0010 D7).
  // The comparison is strict: two cells at the *same* compilation scale do not
  // clip each other. Standard ENC usage bands don't overlap at equal scale, and
  // any residual same-scale overlap is left for import_geotiff to dedup rather
  // than resolved arbitrarily here.
  int exported = 0;
  for (const Cell & cell : cells) {
    std::vector<OGRGeometry *> clip;
    for (const Cell & other : cells) {
      if (other.scale < cell.scale) {
        clip.insert(clip.end(), other.footprints.begin(), other.footprints.end());
      }
    }

    auto gdal = openVector(cell.ds->filePath());
    if (!gdal) {
      log << "warning: cannot reopen " << cell.ds->filePath() << "; skipping\n";
      continue;
    }
    const std::string out_path =
      opts.out_dir + "/" + baseLabel(cell.ds->label()) + ".tif";
    std::string error;
    CellExport stats;
    if (exportCell(*gdal, cell.scale, datum, clip, out_path, error, &stats)) {
      if (stats.written == 0) {
        log << "warning: " << cell.ds->label()
            << ": no in-datum data (all pixels no-data); wrote empty " << out_path << "\n";
      } else {
        log << "exported " << out_path << " (" << stats.width << "x" << stats.height
            << ", " << stats.written << " cells, scale 1:" << static_cast<long>(cell.scale)
            << ", GGGS level " << stats.level << " -> import_geotiff --level " << stats.level
            << ")\n";
        ++exported;
      }
    } else {
      log << "warning: " << cell.ds->label() << ": " << error << "\n";
    }
  }

  // Release cloned footprints.
  for (const Cell & cell : cells) {
    for (OGRGeometry * g : cell.footprints) {
      OGRGeometryFactory::destroyGeometry(g);
    }
  }

  log << "exported " << exported << " of " << cells.size() << " cell(s)\n";

  // A non-empty corpus that produced nothing (every cell failed or was all
  // no-data) is a failure, not a silent success: signal it so the CLI exits
  // nonzero and a caller can tell it apart from a genuinely empty corpus (which
  // returns 0 above). -1 already means "no output"; reuse it here.
  if (!cells.empty() && exported == 0) {
    log << "error: no cells exported from a non-empty corpus\n";
    return -1;
  }
  return exported;
}

}  // namespace s57_to_geotiff
