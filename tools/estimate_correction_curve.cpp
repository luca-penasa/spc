#include <pcl/io/pcd_io.h>
#include <pcl/console/parse.h>
#include <pcl/console/print.h>
#include <pcl/console/time.h>
#include <pcl/filters/voxel_grid.h>
#include <spc/methods/kernel_smoothing.h>
#include <spc/time_series/sparse_time_series.h>
#include <spc/common/common.h>
#include <spc/common/strings.h>
#include <spc/common/io_helper.h>

#include <math.h>


using namespace pcl;
using namespace pcl::console;

using namespace std;

//A strange point type to be used here:
struct PointXYZScalar
    {
        PCL_ADD_POINT4D;
        float Sc4laR897;

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    } EIGEN_ALIGN16;

    POINT_CLOUD_REGISTER_POINT_STRUCT   (PointXYZScalar,
                         (float, x, x)
                         (float, y, y)
                         (float, z, z)
                         (float, Sc4laR897, Sc4laR897)


    )

int checkIfFieldExists(sensor_msgs::PointCloud2 cloud, string fieldName)
{
    int status = getFieldIndex(cloud, fieldName);
    if(status == -1)
    {
        print_error("Cannot find %s scalar field to be used as intensity\n", fieldName.c_str());
    }
    return status;
}

void
printHelp(int argc, char ** argv)
{
    print_warn("USAGE: %s incloud.pcd [options]\n\n", argv[0]);
    print_info("Options are:\n");
    print_info("-if   name                specify intensity field name\n");
    print_info("-vs   float               size of voxelsize for initial downsampling\n");
    print_info("-c    float,float,float   position of the sensor for distance computing\n");
    print_info("-st   float               step size for evaluating correction curve\n");
    print_info("-bw   float               size of bandwidth for KS estimator\n\n");
    print_highlight("NOTE the output curve will be placed in incloud_correction_curve.txt\n");

}



int main (int argc, char ** argv)
{
    if (argc == 1)
    {
        printHelp(argc, argv);
        return -1;
    }

    ///////////////// UNCHANGEABLE DEFAULTS
    Eigen::Vector3f center;
    center.fill(0.0);


    ///////// PARSE THINGS
    //filename
    std::vector<int> incloud_id = parse_file_extension_argument(argc, argv, ".pcd");
    if (incloud_id.size() == 0)
    {
        print_error("Specify an input pcd file!");
        printHelp(argc, argv);
        return -1;
    }
    string incloud_pathname = argv[incloud_id[0]]; //first one is input


    //int field name
    string intensity_field_name = string("intensity"); //default value
    parse_argument(argc, argv, "-if", intensity_field_name) ; //catch a specification for the field


    //voxelgrid size for downsampling
    float vsize  = 0.05; //5 cm as default
    parse_argument(argc, argv, "-vs", vsize); //catch a specification for vsize



    //center
    parse_3x_arguments(argc, argv, "-c", center(0), center(1), center(2));

    //step at which to perform the estimation.
    float step = 0.01; //one cm as default
    parse_argument(argc, argv, "-st", step);

    //Bandwidth.
    float bandwidth = 1.0f; //one cm as default
    parse_argument(argc, argv, "-bw", bandwidth);



    //////////PRINT INFOS

    print_info("Using %s as intensity field\n", intensity_field_name.c_str());
    print_info("Downsampling with a voxelsize of %4.5f \n", vsize);
    print_info("Using as center %4.5f , %4.5f , %4.5f \n", center(0), center(1), center(2));
    print_info("Using a step for estimating the curve of %4.5f\n", step);
    print_info("Using a bandwidth for estimating the curve of %4.5f\n", bandwidth);


    /////////////////////// DO COMPUTATIONS
    //load the cloud
    sensor_msgs::PointCloud2::Ptr incloud_sns (new sensor_msgs::PointCloud2);
    io::loadPCDFile(incloud_pathname, *incloud_sns);

    assert( checkIfFieldExists(*incloud_sns, intensity_field_name) != -1);
    assert( checkIfFieldExists(*incloud_sns, string("x") ) != -1);
    assert( checkIfFieldExists(*incloud_sns, string("y") ) != -1);
    assert( checkIfFieldExists(*incloud_sns, string("z") ) != -1);

    int int_id = getFieldIndex(*incloud_sns,intensity_field_name.c_str());
    incloud_sns->fields[int_id].name = "Sc4laR897"; //do this trick for being able to use any user-supplied field




    //SUBSAMPLE THE ORIGINAL CLOUD
    pcl::VoxelGrid<sensor_msgs::PointCloud2> grid_filter;
    grid_filter.setInputCloud(incloud_sns);
    grid_filter.setLeafSize(vsize, vsize, vsize);

    sensor_msgs::PointCloud2::Ptr downsampled_cloud  (new sensor_msgs::PointCloud2);
    grid_filter.filter(*downsampled_cloud);


    //convert to pcl type
    pcl::PointCloud<PointXYZScalar>::Ptr incloud (new pcl::PointCloud<PointXYZScalar>);
    fromROSMsg(*downsampled_cloud, *incloud);


    ///// COMPUTE DISTANCES
    std::vector<float> distances (incloud->size());

    typedef typename PointCloud<PointXYZScalar>::PointType PointT;

    std::transform(incloud->begin(), incloud->end(), distances.begin(),
                   [&](PointT & point){return sqrt(
                                              (point.x - center(0)) * (point.x - center(0)) +
                                              (point.y - center(1)) * (point.y - center(1)) +
                                              (point.z - center(2)) * (point.z - center(2)) ) ;});

    // EXTRACT ALSO INTENSITIES
    vector<float> intensities (incloud->size());
    transform(incloud->begin(), incloud->end(), intensities.begin(), [&](PointT &point){return point.Sc4laR897;});


    //create the input tseries
    ll::SparseTimeSeries<float> * in_data  = new ll::SparseTimeSeries<float>(distances, intensities);


//    float min = get_min<float>(distances);
//    float max = get_max<float>(distances);


//    //the vector with distances at which to estimate the trend
//    vector<float> new_distances = subdivideRange(min, max, step);


//    print_info("MIN Distance %f, Max Distance %f, Requested Step %f \n", min, max, step);


    ll::EquallySpacedTimeSeries<float> *  series = new ll::EquallySpacedTimeSeries<float>(in_data->getMinX(), in_data->getMaxX(), step);

    //now do the estimate of the trend, using kernelsmoothing
    //now initialize a kernelsmoothing object
    ll::KernelSmoothing<float> ks;
    ks.setComputeVariance(0); //also compute the variance - support for variance computing is not complete!
    ks.setInput(in_data);
    ks.setBandwidth(bandwidth);
    ks.compute(series);



    print_info("Computing estimate... This may take a while\n");

    //get the computed values
    vector<float> new_intensities = series->getY();


    //save some data for any future use
    vector< std::vector<float> > to_save = {series->getX(), series->getY()};

    string basename = stripExtension(incloud_pathname);
    string out_name = basename + std::string("_correction_curve.txt");


    print_highlight("Saving the correction curve here: \n %s \n",  out_name.c_str());
    saveAsCSV( out_name, std::string(" "), to_save);

    return 1;
}
