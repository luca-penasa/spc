
#include <spc/core/spc_eigen.h>

#include <spc/core/strings.h>

#include <pcl/features/normal_3d.h>

#include <spc/methods/PointCloudEigenIndicesEstimator.h>
#include <spc/io/io_helper.h>
#include <spc/methods/IntensityCalibrationDataEstimator.h>
#include <spc/methods/IntensityCalibrationDataFilter.h>

#include <boost/spirit/home/support/detail/hold_any.hpp>

#include <spc/io/element_io.h>


#include <gflags/gflags.h>
#include <spc/core/logging.h>


using namespace std;
using namespace pcl;
using namespace pcl::console;
using namespace pcl::io;



DEFINE_string(in_clouds, "", "space separated list of input clouds");
DEFINE_string(core_points_cloud, "", "file containing the core points");
DEFINE_string(normals_cloud, "", "file containing the normals");

DEFINE_string(out, "", "out file without extension");

DEFINE_double(search_radius, 0.1, "the search radius to use when normals are estimated directly");
DEFINE_double(acceptable_distance, 0.1, "maximum accettable distance for normal transferring");


DEFINE_bool(gaussian, true, "use gaussian filtering for estimating the average intensity");
DEFINE_double(gaussian_sigma, 0.1, "the spatial sigma for intensity sampling with gaussian");



int main (int argc, char ** argv)
{
    google::InitGoogleLogging(argv[0]);

    google::SetUsageMessage("computes a dataset of sampled scalar fields for calibrating the device");

    FLAGS_logtostderr = 1;
    google::ParseCommandLineFlags(&argc, &argv, true);


    std::vector<std::string> cloud_names = spc::splitStringAtSeparator(FLAGS_in_clouds);


    std::vector<CloudDataSourceOnDisk::Ptr> sources;
    for (std::string name : cloud_names)
    {
        CloudDataSourceOnDisk::Ptr s (new CloudDataSourceOnDisk(name));
        sources.push_back(s);
    }

    for (std::string s: cloud_names)
    {
        LOG(INFO) << "Required cloud "<< s;
    }


    spc::CalibrationDataEstimator calibrator;
    calibrator.setInputClouds(sources);

//    if (!FLAGS_normals_cloud.empty())
//    {
//        calibrator.setInputNormalsCloudName(FLAGS_normals_cloud);
//        calibrator.setNormalEstimationMethod(spc::CalibrationDataEstimator::PRECOMPUTED_NORMALS);
//        calibrator.setMaximumDistanceForGettingNormal(FLAGS_acceptable_distance);
//    }

    CloudDataSourceOnDisk keys(FLAGS_core_points_cloud);
    NewSpcPointCloud::Ptr key_cloud = keys.load2();

    calibrator.setInputKeypoints(key_cloud);
    calibrator.setSearchRadius(FLAGS_search_radius);
    calibrator.compute();


//    if (FLAGS_gaussian)
//    {
//        calibrator.setIntensityEstimationMethod(spc::CalibrationDataEstimator::GAUSSIAN_ESTIMATION);
//        calibrator.setIntensityGaussianSpatialSigma(FLAGS_gaussian_sigma);
//    }

//    LOG(INFO) << "going to compute";
//    calibrator.compute();
//    LOG(INFO) << "going to compute.Done";



//    spc::EigenTable::Ptr db = calibrator.getCalibrationDB();


//    spc::io::serializeToFile(db, FLAGS_out, spc::io::XML);




    return 1;
}
