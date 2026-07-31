#ifndef S57_TO_GEOTIFF_EXPORTER_HPP
#define S57_TO_GEOTIFF_EXPORTER_HPP

#include <functional>
#include <optional>
#include <ostream>
#include <string>
#include <vector>

class GDALDataset;
class OGRGeometry;

namespace s57_to_geotiff
{

// Map an S-57 CATZOC code to a 1-sigma vertical-uncertainty floor (metres) at
// the given depth (positive-down metres below chart datum), per ADR-0010 D7 /
// the S-57 ZOC table:
//   1 (A1)      -> 0.5 m + 1 %d
//   2 (A2), 3 (B) -> 1.0 m + 2 %d
//   4 (C)       -> 2.0 m + 5 %d
//   5 (D), 6 (U) -> a large sigma (never keepout-grade)
//   0 / unknown -> 0.0 (no CATZOC floor; the caller's half-band floor still applies)
double catzocSigma(int catzoc, double depth);

// The large sigma assigned to CATZOC D/U (assessed-poor / unassessed) zones.
constexpr double kCatzocLargeSigma = 1000.0;

// Per-pixel datum lookup: (lat, lon) in degrees -> the chart datum's ellipsoidal
// height (metres, up-positive; typically negative), or nullopt where no datum is
// resolvable (the pixel becomes no-data). In the CLI this wraps the
// marine_vertical_datum precedence chain; tests inject a constant.
using DatumFn = std::function<std::optional<double>(double lat, double lon)>;

// Outcome of exporting a single cell.
struct CellExport
{
  int width = 0;
  int height = 0;
  long written = 0;   // pixels with a finite depth written to band 1
};

// Export one open S-57 vector `dataset` to a two-band GeoTIFF at `out_path`
// (band 1 = seafloor ellipsoidal height in metres up-positive, band 2 = 1-sigma
// in metres, NaN no-data; WGS84 geographic), following ADR-0010 D7.
//
// `chart_scale` is the cell's compilation scale denominator (drives GGGS level
// selection). `datum` resolves the chart datum per pixel. `clip_geoms` are the
// coverage footprints of all finer-scale cells: any pixel they cover is dropped
// so the largest scale governs (empty = keep everything). Returns false and
// fills `error` on failure; on success fills `stats` when non-null.
bool exportCell(
  GDALDataset & dataset, double chart_scale, const DatumFn & datum,
  const std::vector<OGRGeometry *> & clip_geoms, const std::string & out_path,
  std::string & error, CellExport * stats = nullptr);

// Options for a full corpus run.
struct ExporterOptions
{
  std::string enc_root;               // ENC corpus root (has CATALOG.031 or cell dirs)
  std::string out_dir;                // where the .tif files are written
  std::string geoid_grid;             // ellipsoid->NAVD88 geoid grid (e.g. g2018)
  std::string vdatum_dir;             // dir of regional NAVD88->MLLW .gtx grids
  std::string datum_config;           // optional polygon->datum YAML config
  std::optional<double> lake_datum;   // optional lake-surface ellipsoidal height (m)
};

// Discover every cell under opts.enc_root, build the datum query once, compute
// each cell's finer-scale clip, and export it. Progress and warnings go to
// `log`. Returns the number of cells written, or -1 on a fatal setup error.
int runExport(const ExporterOptions & opts, std::ostream & log);

}  // namespace s57_to_geotiff

#endif  // S57_TO_GEOTIFF_EXPORTER_HPP
