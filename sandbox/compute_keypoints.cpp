

#include <spc/methods/keypoints_extractor.h>

#include <pcl/io/file_io.h>
#include <pcl/io/pcd_io.h>
#include <spc/methods/keypoints_filter.h>

using namespace std;
int main(int argv, char ** argc)
{
    string filename = argc[1];

    cout << "computing on " << filename << endl;

    sensor_msgs::PointCloud2Ptr in_cloud (new sensor_msgs::PointCloud2);
    pcl::io::loadPCDFile(filename, *in_cloud);

    pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_cloud (new pcl::PointCloud<pcl::PointXYZI>);

    pcl::fromROSMsg(*in_cloud, *pcl_cloud);

    spc::KeypointsExtractor<pcl::PointXYZI> extractor;
    extractor.setInCloud(pcl_cloud);

    spc::KeypointsExtractor<pcl::PointXYZI>::KeypointsPtrT keys = extractor.computeKeypoints(0.05, 0);



    spc::KeypointsFilter<spc::Keypoint<pcl::PointXYZI> > filter;

    filter.setInputKeypoints(keys);
    filter.setNStd(0.05);
    filter.filterByAverages();

    *keys = filter.getCurrentKeys();

    keys->saveToFile("/home/luca/testfile.txt");




return 1;

}
