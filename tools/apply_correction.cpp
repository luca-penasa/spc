#include <pcl/io/file_io.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/centroid.h>
#include <Eigen/Dense>
#include <iostream>
#include <spc/methods/strings.h>

#include <pcl/features/normal_3d.h>

#include <spc/methods/PointCloudEigenIndicesEstimator.h>
#include <spc/io/io_helper.h>

#include <spc/methods/EigenFunctionsParametrizator.h>
#include <spc/methods/IntensityCalibrationDataEstimator.h>
#include <spc/methods/IntensityCalibrationDataFilter.h>

#include <boost/spirit/home/support/detail/hold_any.hpp>
#include <spc/methods/IntensityCalibration.h>
#include <spc/elements/ICalModelFactors.h>
#include <spc/io/element_io.h>
#include <spc/methods/IntensityCalibrationApply.h>

#include <spc/elements/PointCloudPcl.h>
#include <spc/elements/ICalPHFunction.h>
using namespace std;
using namespace pcl;
using namespace pcl::console;
using namespace pcl::io;

void printHelp(int argc, char **argv)
{
    print_info("USAGE: %s cloud.pcd [cloud2.pcd ... cloudN.pcd]  model.spc\n",
               argv[0]);
    print_info("Apply the calibration to the given clouds, read the "
               "calibration model from model.spc\n");
    //    print_info("Options are:\n");
    print_info("-n normals.pcd use external normals as normals source.\n");
    print_info("-m model.* calibration model.\n");

    print_info("-h this help\n");
}

Eigen::Vector3f vector_from_string(const string line, const string separator)
{
    vector<string> strs;
    boost::split(strs, line, boost::is_any_of(" "));

    Eigen::Vector3f out;
    for (int i = 0; i < 3; ++i)
        out(i) = atof(strs.at(i).c_str());

    return out;
}

int main(int argc, char **argv)
{

    printHelp(argc, argv);
    pcl::console::TicToc tt;
    tt.tic();

    setVerbosityLevel(VERBOSITY_LEVEL::L_DEBUG);

    vector<int> tmp
        = pcl::console::parse_file_extension_argument(argc, argv, "pcd");

    std::string normal_cloud_fname, model_fname;
    pcl::console::parse_argument(argc, argv, "-n", normal_cloud_fname);
    pcl::console::parse_argument(argc, argv, "-m", model_fname);

    vector<string> clouds_fn;
    for (int i : tmp) {
        if (argv[i] != normal_cloud_fname.c_str())
            clouds_fn.push_back(argv[i]);
    }

    bool using_normals = false;
    if (!normal_cloud_fname.empty()) {
        print_info("Reading Normals from %s\n", normal_cloud_fname.c_str());
        using_normals = true;
    }

    spc::ISerializable::Ptr model = spc::io::deserializeFromFile(model_fname);
    spc::EigenFunctionBase::Ptr cal_funct = spcDynamicPointerCast
        <spc::EigenFunctionBase>(model);

    if (!cal_funct) {
        PCL_ERROR("cannot find a model\n");
        return -1;
    }

    if (clouds_fn.empty())
    {
        PCL_ERROR("No clouds to correct provided.\n");
        return -1;
    }

    for (string fname : clouds_fn) {
        pcl::console::print_info("Correcting cloud %s\n", fname.c_str());

        Eigen::Quaternionf ori;
        Eigen::Vector4f center;
        pcl::PCLPointCloud2Ptr pcl_cloud
            = pcl::PCLPointCloud2Ptr(new pcl::PCLPointCloud2);
        pcl::io::loadPCDFile(fname, *pcl_cloud, center, ori);

        spc::PointCloudPCL::Ptr asspccloud ( new spc::PointCloudPCL(pcl_cloud));

        asspccloud->setSensorPosition(center.head(3));

        if (!pcl_cloud) {
            PCL_ERROR("cannot find cloud %s\n", fname.c_str());
            return -1;
        }

        print_info("Cloud %s loaded\n Center: [%f, %f, %f]\n", fname.c_str(),
                   center(0), center(1), center(2));

        pcl::PCLPointCloud2Ptr pcl_normal_cloud
            = pcl::PCLPointCloud2Ptr(new pcl::PCLPointCloud2);



        if (using_normals) {
            print_info("Loading normals from %s\n", normal_cloud_fname.c_str());
            loadPCDFile(normal_cloud_fname, *pcl_normal_cloud);
            print_info("...Done\n", normal_cloud_fname.c_str());
        }

        spc::IntensityCalibrationApplier cal;

        if (!cal_funct)
        {
            PCL_ERROR("void pointer\n");
            return -1;
        }

        std::cout << cal_funct->getInputSize() << std::endl;
        std::cout << cal_funct->getOutputSize() << std::endl;

        cal.setCalibrationFunction(cal_funct);
        cal.setCloudToCalibrate(asspccloud);
        cal.setMaxDistanceForNormal(0.3);

        if (using_normals)
        {
            spc::PointCloudPCL::Ptr n_ptr (new spc::PointCloudPCL(pcl_normal_cloud));
            cal.setNormalsCloud(n_ptr);

        }

        cal.compute();

        string newname = fname + string("2.pcd");
        pcl::io::savePCDFile(newname, *pcl_cloud);

        return 1;
    }

    return 1;
}
