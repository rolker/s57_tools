#include <gtest/gtest.h>

#include <cmath>
#include <memory>
#include <string>

#include "cpl_conv.h"
#include "gdal_priv.h"
#include "ogrsf_frmts.h"

#include "marine_autonomy/gggs/level.h"
#include "exporter.hpp"

namespace
{

// A synthetic in-memory S-57-like cell: OGR features tagged with the OBJL codes
// the exporter switches on. No real ENC file or network needed.
class SyntheticCell
{
public:
  SyntheticCell()
  {
    GDALAllRegister();
    OGRRegisterAll();
    GDALDriver * driver = GetGDALDriverManager()->GetDriverByName("Memory");
    dataset_.reset(driver->Create("synthetic", 0, 0, 0, GDT_Unknown, nullptr));
    srs_.SetWellKnownGeogCS("WGS84");
    layer_ = dataset_->CreateLayer("features", &srs_, wkbUnknown, nullptr);
    addField("OBJL", OFTInteger);
    addField("DRVAL1", OFTReal);
    addField("DRVAL2", OFTReal);
    addField("VALSOU", OFTReal);
    addField("CATZOC", OFTInteger);
    addField("CATCOV", OFTInteger);
  }

  GDALDataset & dataset() {return *dataset_;}

  // Coverage polygon (M_COVR, CATCOV=1) bounding the cell.
  void addCoverage(double minx, double miny, double maxx, double maxy)
  {
    OGRFeature f(layer_->GetLayerDefn());
    f.SetField("OBJL", 302);
    f.SetField("CATCOV", 1);
    auto poly = makeRect(minx, miny, maxx, maxy);
    f.SetGeometry(poly.get());
    layer_->CreateFeature(&f);
  }

  // A DEPARE depth area with a [drval1, drval2] band.
  void addDepare(double minx, double miny, double maxx, double maxy,
    double drval1, double drval2)
  {
    OGRFeature f(layer_->GetLayerDefn());
    f.SetField("OBJL", 42);
    f.SetField("DRVAL1", drval1);
    f.SetField("DRVAL2", drval2);
    auto poly = makeRect(minx, miny, maxx, maxy);
    f.SetGeometry(poly.get());
    layer_->CreateFeature(&f);
  }

  // An M_QUAL quality zone carrying a CATZOC code.
  void addQuality(double minx, double miny, double maxx, double maxy, int catzoc)
  {
    OGRFeature f(layer_->GetLayerDefn());
    f.SetField("OBJL", 308);
    f.SetField("CATZOC", catzoc);
    auto poly = makeRect(minx, miny, maxx, maxy);
    f.SetGeometry(poly.get());
    layer_->CreateFeature(&f);
  }

  // A single sounding (SOUNDG) point with a positive-down depth as its Z.
  void addSounding(double lon, double lat, double depth)
  {
    OGRFeature f(layer_->GetLayerDefn());
    f.SetField("OBJL", 129);
    OGRPoint p(lon, lat, depth);
    f.SetGeometry(&p);
    layer_->CreateFeature(&f);
  }

  static std::unique_ptr<OGRPolygon> makeRect(
    double minx, double miny, double maxx, double maxy)
  {
    auto poly = std::make_unique<OGRPolygon>();
    OGRLinearRing ring;
    ring.addPoint(minx, miny);
    ring.addPoint(maxx, miny);
    ring.addPoint(maxx, maxy);
    ring.addPoint(minx, maxy);
    ring.addPoint(minx, miny);
    poly->addRing(&ring);
    return poly;
  }

private:
  void addField(const char * name, OGRFieldType type)
  {
    OGRFieldDefn def(name, type);
    layer_->CreateField(&def);
  }

  std::unique_ptr<GDALDataset> dataset_;
  OGRSpatialReference srs_;
  OGRLayer * layer_ = nullptr;
};

// Constant chart datum 30 m below the ellipsoid, so ellipsoidal height = -30 - depth.
constexpr double kDatumZ = -30.0;
s57_to_geotiff::DatumFn constantDatum()
{
  return [](double, double) -> std::optional<double> {return kDatumZ;};
}

std::string tempPath(const std::string & name)
{
  return std::string(::testing::TempDir()) + "/" + name;
}

// Sample one band of a GeoTIFF at a geographic point (nearest pixel).
double sample(const std::string & path, int band, double lon, double lat)
{
  std::unique_ptr<GDALDataset> ds(
    static_cast<GDALDataset *>(GDALOpenEx(path.c_str(), GDAL_OF_RASTER, nullptr, nullptr, nullptr)));
  EXPECT_TRUE(ds) << "cannot reopen " << path;
  double gt[6];
  ds->GetGeoTransform(gt);
  int col = static_cast<int>((lon - gt[0]) / gt[1]);
  int row = static_cast<int>((lat - gt[3]) / gt[5]);
  double value = std::nan("");
  ds->GetRasterBand(band)->RasterIO(
    GF_Read, col, row, 1, 1, &value, 1, 1, GDT_Float64, 0, 0);
  return value;
}

}  // namespace

TEST(CatzocSigma, ZocTableMapping)
{
  using s57_to_geotiff::catzocSigma;
  EXPECT_DOUBLE_EQ(catzocSigma(1, 10.0), 0.6);    // A1: 0.5 + 1%d
  EXPECT_DOUBLE_EQ(catzocSigma(2, 10.0), 1.2);    // A2: 1.0 + 2%d
  EXPECT_DOUBLE_EQ(catzocSigma(3, 10.0), 1.2);    // B:  1.0 + 2%d
  EXPECT_DOUBLE_EQ(catzocSigma(4, 10.0), 2.5);    // C:  2.0 + 5%d
  EXPECT_EQ(catzocSigma(5, 10.0), s57_to_geotiff::kCatzocLargeSigma);   // D
  EXPECT_EQ(catzocSigma(6, 10.0), s57_to_geotiff::kCatzocLargeSigma);   // U
  EXPECT_DOUBLE_EQ(catzocSigma(0, 10.0), 0.0);    // no data -> no floor
}

TEST(Exporter, BandMidpointAndHalfBandSigmaFloor)
{
  SyntheticCell cell;
  cell.addCoverage(-70.80, 43.00, -70.70, 43.10);
  cell.addDepare(-70.80, 43.00, -70.70, 43.10, 10.0, 20.0);   // midpoint 15, half-band 5

  const std::string out = tempPath("band_midpoint.tif");
  std::string error;
  s57_to_geotiff::CellExport stats;
  ASSERT_TRUE(
    s57_to_geotiff::exportCell(cell.dataset(), 20000.0, constantDatum(), {}, out, error, &stats))
    << error;
  EXPECT_GT(stats.written, 0);

  // band 1 = datum_z - midpoint = -30 - 15 = -45
  EXPECT_NEAR(sample(out, 1, -70.75, 43.05), -45.0, 1e-6);
  // band 2 = max(half-band 5, catzoc 0) = 5
  EXPECT_NEAR(sample(out, 2, -70.75, 43.05), 5.0, 1e-6);
}

TEST(Exporter, CatzocVariedSigma)
{
  SyntheticCell cell;
  cell.addCoverage(-70.80, 43.00, -70.70, 43.10);
  // Two flat-band areas (half-band 0) so sigma is governed purely by CATZOC.
  cell.addDepare(-70.80, 43.00, -70.75, 43.10, 10.0, 10.0);   // left
  cell.addDepare(-70.75, 43.00, -70.70, 43.10, 10.0, 10.0);   // right
  cell.addQuality(-70.80, 43.00, -70.75, 43.10, 1);           // left: A1
  cell.addQuality(-70.75, 43.00, -70.70, 43.10, 4);           // right: C

  const std::string out = tempPath("catzoc_varied.tif");
  std::string error;
  ASSERT_TRUE(
    s57_to_geotiff::exportCell(cell.dataset(), 20000.0, constantDatum(), {}, out, error, nullptr))
    << error;

  // Depth identical, sigma differs by zone: A1 -> 0.6, C -> 2.5.
  EXPECT_NEAR(sample(out, 1, -70.775, 43.05), -40.0, 1e-6);
  EXPECT_NEAR(sample(out, 1, -70.725, 43.05), -40.0, 1e-6);
  EXPECT_NEAR(sample(out, 2, -70.775, 43.05), 0.6, 1e-6);
  EXPECT_NEAR(sample(out, 2, -70.725, 43.05), 2.5, 1e-6);
}

TEST(Exporter, SoundingOverridesArea)
{
  SyntheticCell cell;
  cell.addCoverage(-70.80, 43.00, -70.70, 43.10);
  cell.addDepare(-70.80, 43.00, -70.70, 43.10, 10.0, 20.0);   // area midpoint 15
  cell.addQuality(-70.80, 43.00, -70.70, 43.10, 1);           // A1 everywhere
  cell.addSounding(-70.75, 43.05, 5.0);                       // shoal sounding, depth 5

  const std::string out = tempPath("sounding.tif");
  std::string error;
  ASSERT_TRUE(
    s57_to_geotiff::exportCell(cell.dataset(), 20000.0, constantDatum(), {}, out, error, nullptr))
    << error;

  // At the sounding: band1 = -30 - 5 = -35, sigma = A1 at depth 5 = 0.55.
  EXPECT_NEAR(sample(out, 1, -70.75, 43.05), -35.0, 1e-6);
  EXPECT_NEAR(sample(out, 2, -70.75, 43.05), 0.55, 1e-6);
}

TEST(Exporter, LevelSelectionFromScale)
{
  auto run = [](double scale) {
      SyntheticCell cell;
      cell.addCoverage(-70.80, 43.00, -70.70, 43.10);
      cell.addDepare(-70.80, 43.00, -70.70, 43.10, 10.0, 20.0);
      std::string error;
      s57_to_geotiff::CellExport stats;
      EXPECT_TRUE(
        s57_to_geotiff::exportCell(
          cell.dataset(), scale, constantDatum(), {},
          std::string(::testing::TempDir()) + "/level_" + std::to_string(static_cast<long>(scale)) +
          ".tif", error, &stats)) << error;
      return stats;
    };

  const auto fine = run(20000.0);      // large scale -> fine level -> small pixels
  const auto coarse = run(320000.0);   // 16x smaller scale -> coarser level -> big pixels
  EXPECT_GT(fine.width, coarse.width);
  EXPECT_GT(fine.height, coarse.height);

  // The fine raster's pixel size matches the GGGS level chosen from the scale.
  const auto level = gggs::Level::fromCellSize(static_cast<float>(20000.0 * 0.0005));
  const int expected_w =
    std::max(1, static_cast<int>(std::ceil(0.10 / level.cellAngularSpan())));
  EXPECT_EQ(fine.width, expected_w);
}

TEST(Exporter, FinerFootprintClip)
{
  SyntheticCell cell;
  cell.addCoverage(-70.80, 43.00, -70.70, 43.10);
  cell.addDepare(-70.80, 43.00, -70.70, 43.10, 10.0, 20.0);   // full-extent area

  // A finer chart covers the left half — those pixels must be clipped away.
  auto finer = SyntheticCell::makeRect(-70.80, 43.00, -70.75, 43.10);
  std::vector<OGRGeometry *> clip{finer.get()};

  const std::string out = tempPath("clip.tif");
  std::string error;
  ASSERT_TRUE(
    s57_to_geotiff::exportCell(cell.dataset(), 320000.0, constantDatum(), clip, out, error, nullptr))
    << error;

  EXPECT_TRUE(std::isnan(sample(out, 1, -70.775, 43.05)));   // clipped (left)
  EXPECT_NEAR(sample(out, 1, -70.725, 43.05), -45.0, 1e-6);  // kept (right)
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
