#include<pcl/point_types.h>
#include <pcl/io/pcd_io.h>
//#include <pcl/io/impl/io.hpp>

#include <pcl/kdtree/kdtree_flann.h>

#include <pcl/features/intensity_spin.h>
#include <pcl/features/impl/intensity_spin.hpp>



const int nr_intensity_bins = 5; //if using SPIN
const int nr_distance_bins = 5;  //distance bins

const int bins_number = nr_intensity_bins * nr_distance_bins;

typedef pcl::PointXYZI PointT;
typedef pcl::IntensityGradient GradientT;
typedef pcl::Histogram<bins_number> HistoT;

POINT_CLOUD_REGISTER_POINT_STRUCT(HistoT,
                                        (float[bins_number], histogram, histogram)

)



int main(int argc, char *argv[])
{
        std::string infilename = argv[1];
        std::string outfilename = argv[2];
        float radius = atof(argv[3]);

        pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);
        pcl::io::loadPCDFile(infilename.c_str(), *cloud);




        printf("Computing Spin descriptors.\n");

        pcl::PointCloud<HistoT>::Ptr cloud_spin (new pcl::PointCloud<HistoT>);
        pcl::IntensitySpinEstimation< PointT, HistoT > spi;
        spi.setInputCloud(cloud);
        spi.setNrIntensityBins(nr_intensity_bins);
        spi.setNrDistanceBins(nr_distance_bins);
        spi.setSearchMethod(pcl::search::KdTree<PointT>::Ptr (new pcl::search::KdTree<PointT>));
        spi.setRadiusSearch(radius);

        spi.compute(*cloud_spin);




        pcl::io::savePCDFile(outfilename.c_str(), *cloud_spin, true);
}

