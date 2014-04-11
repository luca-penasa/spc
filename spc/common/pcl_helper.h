#ifndef SPC_PCL_HELPER_H
#define SPC_PCL_HELPER_H

#include <iostream>
#include <fstream>

#include <spc/common/common.h>

#include <pcl/common/io.h>

namespace spc
{


////////////////////////// ALSO THESE THINGS ARE ~DEPRECATED
void
printHeader(const std::string &filename);


template <typename ScalarT>
std::vector<ScalarT>
readFieldToVector(const pcl::PCLPointCloud2 &cloud, const std::string &fieldname, const unsigned int count = 0) ;

template<typename ScalarT>
std::vector< std::vector<ScalarT> >
readCompleteFieldToVector(const pcl::PCLPointCloud2 &cloud, const std::string &fieldname);

pcl::PCLPointCloud2
fromStdVectorToSensor(const std::vector<std::vector<float> > & std_field, const std::string field_name );


}//end nspace


#endif // SPC_PCL_HELPER_H
