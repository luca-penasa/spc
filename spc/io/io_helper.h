#ifndef SPC_IO_HELPER_H
#define SPC_IO_HELPER_H

//#include <pcl/console/print.h>
// #include <pcl/io/pcd_io.h>
//#include <pcl/point_types.h>
#include <boost/tokenizer.hpp>
//#include <pcl/point_cloud.h>
//#include <pcl/io/pcd_io.h>

#include <spc/core/common.h>

#include <spc/elements/PointCloudBase.h>

#include <string>
#include <iostream>
#include <fstream>

namespace spc
{
namespace io
{

#ifdef SPC_WITH_PCL

int loadCSVFile(const std::string &in_filename, pcl::PCLPointCloud2 &output,
                const int x_id, const int y_id, const int z_id, const int i_id,
                const int k, const std::string s);
#endif //#ifdef SPC_WITH_PCL


int loadCSVTimeSeries(const std::string in_filename,
                      const std::string separator,
                      std::vector<std::vector<float>> &tseries);

template <typename T>
int saveAsCSV(const std::string &filename, const std::string &separator,
              const std::vector<std::vector<T>> &columns,
              const int precision = 6);


#ifdef SPC_WITH_PCL
int savePCDBinaryCompressed(const std::string &filename,
                            const pcl::PCLPointCloud2 &cloud);

int savePointCloudAsPCDBinaryCompressed(PointCloudBase &cloud,  const std::string &filename);
#endif

PointCloudBase::Ptr loadPointCloud(const std::string &filename);




}// end io nspace

} // end namespace

#endif // SPC_IO_HELPER
