#include <cstring>
#include <iostream>
#include <stdexcept>
#include <string>

#include "exporter.hpp"

namespace
{

void usage()
{
  std::cout <<
    "usage: s57_to_geotiff <enc_root> <output_dir>\n"
    "         [--geoid <geoid_grid>] [--vdatum-dir <dir>]\n"
    "         [--datum-config <config.yaml>] [--lake-datum <metres>]\n"
    "\n"
    "  Exports each ENC cell under <enc_root> as a two-band GeoTIFF in\n"
    "  <output_dir> (band 1 = depth as WGS84 ellipsoidal height, band 2 =\n"
    "  1-sigma, NaN no-data), ready for import_geotiff into the store's chart\n"
    "  layer (ADR-0010 D7).\n"
    "\n"
    "  --geoid / --vdatum-dir: grids for the marine_vertical_datum query\n"
    "        (ellipsoid<->MLLW). Without them, datums come only from\n"
    "        --datum-config / --lake-datum, and pixels with no resolvable\n"
    "        datum are written as no-data.\n"
    "  --datum-config: polygon->datum YAML (precedence-chain fallback/override)\n"
    "  --lake-datum:   constant lake-surface ellipsoidal height (m), wins outright\n";
  std::exit(1);
}

const char * needArg(int & i, int argc, char ** argv)
{
  if (i + 1 >= argc) {
    usage();
  }
  return argv[++i];
}

}  // namespace

int main(int argc, char ** argv)
{
  s57_to_geotiff::ExporterOptions opts;
  std::string positional[2];
  int n_positional = 0;

  for (int i = 1; i < argc; ++i) {
    if (std::strcmp(argv[i], "--geoid") == 0) {
      opts.geoid_grid = needArg(i, argc, argv);
    } else if (std::strcmp(argv[i], "--vdatum-dir") == 0) {
      opts.vdatum_dir = needArg(i, argc, argv);
    } else if (std::strcmp(argv[i], "--datum-config") == 0) {
      opts.datum_config = needArg(i, argc, argv);
    } else if (std::strcmp(argv[i], "--lake-datum") == 0) {
      const char * value = needArg(i, argc, argv);
      try {
        opts.lake_datum = std::stod(value);
      } catch (const std::exception &) {
        std::cerr << "error: --lake-datum expects a number, got '" << value << "'\n";
        usage();
      }
    } else if (std::strcmp(argv[i], "--help") == 0 || std::strcmp(argv[i], "-h") == 0) {
      usage();
    } else if (argv[i][0] == '-') {
      // Reject an unrecognized dash-prefixed token as a mistyped flag before the
      // positional branch; otherwise `s57_to_geotiff --badflag out` would run with
      // enc_root="--badflag" and exit 0 with "no charts found".
      std::cerr << "error: unknown option '" << argv[i] << "'\n";
      usage();
    } else if (n_positional < 2) {
      positional[n_positional++] = argv[i];
    } else {
      usage();
    }
  }
  if (n_positional != 2) {
    usage();
  }
  opts.enc_root = positional[0];
  opts.out_dir = positional[1];

  const int exported = s57_to_geotiff::runExport(opts, std::cout);
  return exported < 0 ? 1 : 0;
}
