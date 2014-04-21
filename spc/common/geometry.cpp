#include "geometry.h"
#include <pcl/common/distances.h>
#include <boost/foreach.hpp>

namespace spc
{

template <typename PointT>
void 
computeDistanceFromOrigin(const pcl::PointCloud<PointT> &incloud, pcl::PointCloud<PointD> &outcloud)
{
	//get origin
	Eigen::Vector4f origin = incloud.sensor_origin_;
	pcl::PointXYZ this_origin (origin[0],origin[1],origin[2]);
	
	//resize cloud
	outcloud.width = incloud.width;
	outcloud.height = 1;
	outcloud.is_dense = false;
	outcloud.points.resize(outcloud.width * outcloud.height);
	
	//cycle on points and get distances
	for (int i =0; i < incloud.size(); ++i)	
        outcloud.points[i].distance = pcl::euclideanDistance (incloud.points[i], this_origin) ;
}

template <typename PointT>
float
getAverageDistanceFromSensor(const pcl::PointCloud<PointT> & cloud, const std::vector<int> ids)
{
    Eigen::Vector4f origin = cloud.sensor_origin_;
    pcl::PointXYZ this_origin (origin[0],origin[1],origin[2]);

    std::vector<float> dists;

    BOOST_FOREACH(int id, ids)
    {
        dists.push_back(pcl::euclideanDistance (cloud.points[id], this_origin));
    }

    float sum = std::accumulate(dists.begin(), dists.end(), 0.0);
    float mean = sum / dists.size();

    return mean;
}


template void
computeDistanceFromOrigin(const pcl::PointCloud<pcl::PointXYZ> &incloud, pcl::PointCloud<PointD> &outcloud);

template
float
getAverageDistanceFromSensor(const pcl::PointCloud<pcl::PointXYZI> & cloud, const std::vector<int> ids);

template
float
getAverageDistanceFromSensor(const pcl::PointCloud<pcl::PointXYZ> & cloud, const std::vector<int> ids);

}
