#ifndef SPC_IO_HELPER_H
#define SPC_IO_HELPER_H

#include <pcl/console/print.h>
// #include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <boost/tokenizer.hpp>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>

#include <string>
#include <iostream>
#include <fstream>

namespace spc
{

int 
loadCSVFile(const std::string &in_filename, sensor_msgs::PointCloud2 &output, 
						const int x_id, const int y_id, const int z_id, const int i_id, 
						const int k, const std::string s);


int
loadCSVTimeSeries(const std::string in_filename, const std::string separator, std::vector< std::vector<float> > &tseries);

template<typename T>
int
saveAsCSV(const std::string &filename, const std::string &separator, const std::vector<std::vector<T> > &columns, const int precision=6);

int
savePCDBinaryCompressed(const std::string &filename, const sensor_msgs::PointCloud2 &cloud);
}//end namespace
#endif
