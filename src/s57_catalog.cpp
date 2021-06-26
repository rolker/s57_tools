#include "s57_layer/s57_catalog.h"

#include <dirent.h>
#include <iostream>
#include "s57_layer/s57_dataset.h"
#include "ogrsf_frmts.h"

namespace s57_layer
{

S57Catalog::S57Catalog(std::string enc_root)
{
  GDALAllRegister();

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
          std::cerr << "valid? " << ds->valid() << std::endl;
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


} // namespace s57_layer
