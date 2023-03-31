#include "s57_layer/s57_catalog.h"

#include <dirent.h>
#include <iostream>
#include "s57_layer/s57_dataset.h"
#include "ogrsf_frmts.h"
#include "iso8211/iso8211.h"
#include <algorithm>

namespace s57_layer
{

S57Catalog::S57Catalog(std::string enc_root)
{
  GDALAllRegister();

  OGRSpatialReference wgs84, ecef;
  wgs84.SetWellKnownGeogCS("WGS84");
  ecef.importFromEPSG(4978);
  m_WGS84ToECEF_transformation = std::shared_ptr<OGRCoordinateTransformation>(OGRCreateCoordinateTransformation(&wgs84, &ecef), OCTDestroyCoordinateTransformation);
  m_ECEFToWGS84_transformation = std::shared_ptr<OGRCoordinateTransformation>(OGRCreateCoordinateTransformation(&ecef, &wgs84), OCTDestroyCoordinateTransformation);

  // if we can't read the catalog, we'll scan for charts and query extents
  bool need_scan = true;

  std::string catalog_file = enc_root+"/CATALOG.031";
  DDFModule catalog;
  catalog.Open(catalog_file.c_str());
  if(catalog.GetFieldCount())
  {
    for(DDFRecord *record = catalog.ReadRecord(); record; record = catalog.ReadRecord())
    {
      std::string filename = record->GetStringSubfield("CATD", 0, "FILE", 0);
      if(filename.length() >= 4 && filename.compare(filename.length()-4, 4, ".000")==0)
      {
        std::replace(filename.begin(), filename.end(), '\\', '/');
        std::shared_ptr<S57Dataset> ds(new S57Dataset(enc_root+"/"+filename));
        m_datasets.push_back(ds);
        double slat = record->GetFloatSubfield("CATD", 0, "SLAT", 0);
        double wlon = record->GetFloatSubfield("CATD", 0, "WLON", 0);
        double nlat = record->GetFloatSubfield("CATD", 0, "NLAT", 0);
        double elon = record->GetFloatSubfield("CATD", 0, "ELON", 0);
        ds->setEnvelope(slat, wlon, nlat, elon);
      }
    }
    need_scan = false;
  }

  if(need_scan)
  {
    DIR *dir = opendir(enc_root.c_str());
    if (dir)
    {
      dirent* entry;
      while(entry = readdir(dir))
      {
        if(entry->d_type == DT_DIR || entry->d_type == DT_LNK)
        {
          std::string potential_chart = entry->d_name;
          if (potential_chart != "." && potential_chart != "..")
          {
            std::shared_ptr<S57Dataset> ds(new S57Dataset(enc_root+"/"+potential_chart+"/"+potential_chart+".000"));
            if(ds->getEnvelope() && ds->getEnvelope()->IsInit())
              m_datasets.push_back(ds);
          }
        }
      }
      closedir(dir);
    }
    else
    {
      std::cerr << "Unable to open derectory: " << enc_root << std::endl;
    }
  }
  std::cerr << "found " << m_datasets.size() << " charts" << std::endl;
}

bool S57Catalog::ecefToLatLong(double x, double y, double z, double &lat, double &lon)
{
  int ret = m_ECEFToWGS84_transformation->Transform(1, &x, &y, &z);
  if(ret)
  {
    lat = x;
    lon = y;
  }
  return ret;
}

bool S57Catalog::llToECEF(double lat, double lon, double &x, double &y, double &z)
{
  double alt = 0.0;
  int ret = m_WGS84ToECEF_transformation->Transform(1, &lat, &lon, &alt);
  if(ret)
  {
    x = lat;
    y = lon;
    z = alt;
  }
  return ret;
}

std::vector<std::shared_ptr<S57Dataset> > S57Catalog::intersectingCharts(double minLat, double minLon, double  maxLat, double maxLon)
{
  OGREnvelope e;
  e.Merge(minLon, minLat);
  e.Merge(maxLon, maxLat);

  std::vector<std::shared_ptr<S57Dataset> > ret;
  for(auto c: m_datasets)
  {
    auto ce = c->getEnvelope();
    if(ce && ce->Intersects(e))
      ret.push_back(c);
  }
  return ret;
}

} // namespace s57_layer
