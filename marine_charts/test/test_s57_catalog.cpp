// Reads a real S-57 exchange-set catalogue (NOAA's CATALOG.031 for cell
// US5PSMCC, see test/data/README.md) through S57Catalog, the only user of the
// vendored ISO 8211 reader in src/iso8211.
//
// The expected values were read from the catalogue's bytes (its CATD subfields
// are ASCII), not from any ISO 8211 code. The fixture directory holds no chart
// cell, so if the reader fails to read the catalogue the constructor falls
// back to scanning for <cell>/<cell>.000, finds nothing, and these tests fail.
//
// What this does not show: reading a chart cell (cells go through GDAL's own
// OGR S-57 driver, not this reader), or the fallback scan.

#include <gtest/gtest.h>

#include <memory>
#include <string>
#include <vector>

#include "marine_charts/s57_catalog.h"
#include "marine_charts/s57_dataset.h"

namespace
{

const std::string kEncRoot = TEST_ENC_ROOT;

// CATD record of US5PSMCC\US5PSMCC.000: SLAT, WLON, NLAT, ELON.
constexpr double kSouth = 43.05;
constexpr double kWest = -70.8;
constexpr double kNorth = 43.125;
constexpr double kEast = -70.725;

TEST(S57Catalog, ReadsTheOneDatasetAndItsBoundsFromTheCatalogue)
{
  marine_charts::S57Catalog catalog(kEncRoot);

  // The catalogue lists itself, four .TXT files, and US5PSMCC .000/.001/.002;
  // only the .000 is a dataset.
  auto all = catalog.intersectingCharts(-90.0, -180.0, 90.0, 180.0);
  ASSERT_EQ(all.size(), 1u);
  EXPECT_EQ(all[0]->label(), "US5PSMCC.000");
  EXPECT_EQ(all[0]->filePath(), kEncRoot + "/US5PSMCC/US5PSMCC.000");

  auto dataset = catalog.dataset("US5PSMCC.000");
  ASSERT_NE(dataset, nullptr);
  ASSERT_TRUE(dataset->hasValidBounds());
  const auto & bounds = dataset->getBounds();
  EXPECT_DOUBLE_EQ(bounds.min_pt.latitude, kSouth);
  EXPECT_DOUBLE_EQ(bounds.min_pt.longitude, kWest);
  EXPECT_DOUBLE_EQ(bounds.max_pt.latitude, kNorth);
  EXPECT_DOUBLE_EQ(bounds.max_pt.longitude, kEast);
}

TEST(S57Catalog, SelectsTheChartOnlyForBoxesThatOverlapIt)
{
  marine_charts::S57Catalog catalog(kEncRoot);

  // A box inside the cell (Portsmouth Harbor).
  auto inside = catalog.intersectingCharts(43.07, -70.76, 43.09, -70.74);
  ASSERT_EQ(inside.size(), 1u);
  EXPECT_EQ(inside[0]->label(), "US5PSMCC.000");

  // Boxes just clear of each edge.
  EXPECT_TRUE(catalog.intersectingCharts(43.07, -70.70, 43.09, -70.68).empty());  // east
  EXPECT_TRUE(catalog.intersectingCharts(43.07, -70.85, 43.09, -70.82).empty());  // west
  EXPECT_TRUE(catalog.intersectingCharts(43.13, -70.76, 43.15, -70.74).empty());  // north
  EXPECT_TRUE(catalog.intersectingCharts(43.00, -70.76, 43.04, -70.74).empty());  // south
}

}  // namespace
