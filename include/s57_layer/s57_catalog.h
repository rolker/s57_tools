#ifndef S57_LAYER_S57_CATALOG_H
#define S57_LAYER_S57_CATALOG_H

#include <string>


// TODO EPSG:4978 for lat/lon to ECEF

namespace s57_layer
{

class S57Catalog
{
public:
  S57Catalog(std::string enc_root);
};

} // namespace s57_layer

#endif
