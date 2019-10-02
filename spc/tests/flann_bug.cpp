#include "flann_bug.h"





int main(int argc, char **argv)
{

    pcl::PointCloud
        <pcl::PointXYZ>::Ptr pcl_c(new pcl::PointCloud<pcl::PointXYZ>);

    pcl_c->resize(1);
    pcl_c->at(0).x = 1;
    pcl_c->at(0).y = 2;
    pcl_c->at(0).z = 22;

    pcl::PCLPointCloud2Ptr sensor_cloud (new pcl::PCLPointCloud2);

    pcl::toPCLPointCloud2(*pcl_c, *sensor_cloud);

    CloudProxy proxy(sensor_cloud);

    proxy.updateSearcher();

    pcl::PointXYZ p;
    p.x = 1;
    p.y = 2;
    p.z = 23;

    std::vector<int> ids;
    std::vector<float> dists;

    proxy.getSearcher()->nearestKSearch(p, 1, ids, dists);

    std::cout << ids.at(0) << std::endl;
    std::cout << dists.at(0) << std::endl;

    return 1;
}
