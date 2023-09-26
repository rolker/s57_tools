#include "s57_grids/s57_catalog.h"

#include <dirent.h>
#include <iostream>
#include "s57_grids/s57_dataset.h"
#include "ogrsf_frmts.h"
#include "iso8211/iso8211.h"
#include <algorithm>

namespace s57_grids
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
        double slat = record->GetFloatSubfield("CATD", 0, "SLAT", 0);
        double wlon = record->GetFloatSubfield("CATD", 0, "WLON", 0);
        double nlat = record->GetFloatSubfield("CATD", 0, "NLAT", 0);
        double elon = record->GetFloatSubfield("CATD", 0, "ELON", 0);
        ds->setBounds(slat, wlon, nlat, elon);
        datasets_[ds->label()] = ds;
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
            if(ds->hasValidBoundsTryOpen())
              datasets_[ds->label()] = ds;
          }
        }
      }
      closedir(dir);
    }
    else
    {
      std::cerr << "Unable to open directory: " << enc_root << std::endl;
    }
  }
  std::cout << "found " << datasets_.size() << " charts" << std::endl;
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

std::vector<std::shared_ptr<S57Dataset> > S57Catalog::intersectingCharts(const geographic_msgs::BoundingBox &bounds)
{
  return intersectingCharts(bounds.min_pt.latitude, bounds.min_pt.longitude, bounds.max_pt.latitude, bounds.max_pt.longitude);
}

std::vector<std::shared_ptr<S57Dataset> > S57Catalog::intersectingCharts(double minLat, double minLon, double  maxLat, double maxLon)
{
  std::vector<std::shared_ptr<S57Dataset> > ret;
  for(auto c: datasets_)
    if (c.second->intersects(minLat, minLon, maxLat, maxLon))
      ret.push_back(c.second);
  return ret;
}

std::shared_ptr<S57Dataset> S57Catalog::dataset(std::string label) const
{
  auto di = datasets_.find(label);
  if(di != datasets_.end())
    return di->second;
  return std::shared_ptr<S57Dataset>();
}

} // namespace s57_grids
