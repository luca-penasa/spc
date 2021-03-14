#include<pcl/point_types.h>
#include <pcl/io/pcd_io.h>
//#include <pcl/io/impl/io.hpp>

#include <pcl/kdtree/kdtree_flann.h>

#include <pcl/features/intensity_spin.h>
#include <pcl/features/impl/intensity_spin.hpp>

#include <pcl/console/parse.h>

#include <spc/core/logging.h>



const int nr_intensity_bins = 5; //if using SPIN
const int nr_distance_bins = 5;  //distance bins

const int bins_number = nr_intensity_bins * nr_distance_bins;

typedef pcl::PointXYZI PointT;
typedef pcl::IntensityGradient GradientT;
typedef pcl::Histogram<bins_number> HistoT;

POINT_CLOUD_REGISTER_POINT_STRUCT(HistoT,
                                        (float[bins_number], histogram, histogram)

)


void printHelp(int argc, char **argv)
{
    std::cout << "USAGE:" << std::endl;
    std::cout << argv[0] << " incloud.pcd outspins.pcd radius [-s float sigma] [-f bool ascii] " << std::endl;
    std::cout << "-h or --help for this help!" << std::endl;
}

INITIALIZE_EASYLOGGINGPP

int main(int argc, char ** argv)
{
	START_EASYLOGGINGPP(argc, argv);
	std::cout << " QQQQQQQQQQ" << std::endl; 

    if ((argc < 4)
            | (pcl::console::find_switch(argc, argv, "-h") )
            | (pcl::console::find_switch(argc, argv, "--help")))
    {
        printHelp(argc, argv);
        return 1;
    }

        std::string infilename = argv[1];
        std::string outfilename = argv[2];
        float radius = atof(argv[3]);

        float sigma = 1.0f;
        bool ascii = true;

        pcl::console::parse_argument(argc, argv, "-s", sigma);
        std::cout << "SIGMA: " << sigma << std::endl;


        pcl::console::parse_argument(argc, argv, "-f", ascii);
        std::cout << "MODE: " << ascii << std::endl;


        pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);
        pcl::io::loadPCDFile(infilename.c_str(), *cloud);


        printf("Computing Spin descriptors.\n");

        pcl::PointCloud<HistoT>::Ptr cloud_spin (new pcl::PointCloud<HistoT>);
        pcl::IntensitySpinEstimation< PointT, HistoT > spi;
        spi.setInputCloud(cloud);
        spi.setNrIntensityBins(nr_intensity_bins);
        spi.setNrDistanceBins(nr_distance_bins);
        spi.setSmoothingBandwith(sigma);
        spi.setSearchMethod(pcl::search::KdTree<PointT>::Ptr (new pcl::search::KdTree<PointT>));
        spi.setRadiusSearch(radius);

        spi.compute(*cloud_spin);




        pcl::io::savePCDFile(outfilename.c_str(), *cloud_spin, !ascii);
}

