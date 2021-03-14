
#include <spc/core/spc_eigen.h>

#include <spc/core/spc_strings.h>

#include <pcl/features/normal_3d.h>

#include <spc/methods/PointCloudEigenIndicesEstimator.h>
#include <spc/io/io_helper.h>
#include <spc/methods/IntensityCalibrationDataEstimator.h>
//#include <spc/methods/IntensityCalibrationDataFilter.h>

#include <boost/spirit/home/support/detail/hold_any.hpp>

#include <spc/io/element_io.h>
#include <spc/core/file_operations.h>

#include <spc/core/flagging.h>
#include <spc/core/logging.h>

#include <spc/methods/IntensityCalibratorRBF.h>

#include <spc/elements/calibration/DataHolder.h>

#include <spc/elements/NewSpcPointCloud.h>

using namespace std;
using namespace pcl;
using namespace pcl::console;
using namespace pcl::io;

DEFINE_string(in_clouds, "", "space separated list of input clouds");
DEFINE_string(core_points_cloud, "", "file containing the core points");

DEFINE_string(out, "calibration_data", "out file without extension");

DEFINE_bool(knn_intensity, true, "Whether or not to use knn search for intensity estimation - otherwise radius search will be used");
DEFINE_bool(knn_normal, true, "Whether or not to knn radius search for intensity estimation - otherwise radius search will be used");

DEFINE_int32(nn_intensity, 10, "Number of Nearest neighbors to be used when knn_intensity is true");
DEFINE_int32(nn_normal, 10, "Number of Nearest neighbors to be used when knn_normal is true");


DEFINE_int32(min_nn_intensity, 10, "Minimum number of nearest neighbors necessary to consider an intensity estimation good");
DEFINE_int32(min_nn_normal, 10, "Minimum number of nearest neighbors necessary to consider a normal estimation good");

DEFINE_double(radius_intensity, 0.1, "Search radius to be used when knn_intensity is false");
DEFINE_double(radius_normal, 0.1, "Search radius to be used when knn_normal is false");


DEFINE_double(gaussian_sigma, 0,
    "the spatial sigma for intensity sampling with gaussian - 0 for disabling intensity weighting by gaussian");


DEFINE_double(max_admissible_distance, 0.4,
    "the max admissible nighbor distance when usin knn serch instead of radius search");

//DEFINE_bool(export_as_cloud, true, "save the points as a cloud also");
DEFINE_bool (disable_spc_save, false, "disable saving as spc binary file");
DEFINE_bool(export_as_ascii, true,
	"save the data as pure ascii, e.g. for inspection");

DEFINE_bool(
	regexp, true,
	"use a regular expression to set in_clouds (found in current directory)");

DEFINE_bool(
    filter_nans, true,
    "filter nans values before saving as ascii file");


DEFINE_string(intensity_field, "intensity", "the intensity field");


INITIALIZE_EASYLOGGINGPP

int main(int argc, char ** argv)
{
	START_EASYLOGGINGPP(argc, argv);

	google::SetUsageMessage(
		"computes a dataset of sampled scalar fields for calibrating the device");

	//    FLAGS_logtostderr = 1;
    google::ParseCommandLineFlags(&argc, &argv, true);




//	LOG(INFO) << google::CommandlineFlagsIntoString();


//    log_args();

	std::string cwd = spc::getCurrentDirectory();

	LOG(INFO) << "Current directory: " << cwd;

	// in clouds to process
	std::vector<std::string> cloud_names;

	if (FLAGS_regexp) {
		LOG(INFO) << "Using regexp to match the cloud files";
		LOG(INFO) << "regex " << FLAGS_in_clouds;
		cloud_names = spc::list_files_regexp(cwd, FLAGS_in_clouds);

		LOG(INFO) << "Matched " << cloud_names.size() << " entries";
		for (auto s : cloud_names)
			LOG(INFO) << "Matched file: " << s;
	}
	else {
		cloud_names = spc::splitStringAtSeparator(FLAGS_in_clouds);
	}

	if (cloud_names.size() == 0) // we use the arguments
	{
		for (int i = 1; i < argc; ++i) {
			cloud_names.push_back(argv[i]);
		}
    }

	if (cloud_names.size() == 0) {
		LOG(FATAL) << "No point clouds to process, please specify them with "
					  "--in-clouds argument or as standard args";
	}



	//! sort the cloud_names
	std::sort(cloud_names.begin(), cloud_names.end());

	std::vector<CloudDataSourceOnDisk::Ptr> sources;
	for (std::string name : cloud_names) {
		CloudDataSourceOnDisk::Ptr s(new CloudDataSourceOnDisk(name));
		sources.push_back(s);
	}

	for (std::string s : cloud_names) {
		LOG(INFO) << "Required cloud " << s;
	}

    spc::calibration::CalibrationDataEstimator estimator;
    estimator.setInputClouds(sources);

	LOG(INFO) << "Using as intensity field: " << FLAGS_intensity_field;
    estimator.setIntensityFieldName(FLAGS_intensity_field);

	CloudDataSourceOnDisk keys(FLAGS_core_points_cloud);
	NewSpcPointCloud::Ptr key_cloud = keys.load2();

    estimator.setInputKeypoints(key_cloud);

    if (FLAGS_knn_intensity)
        estimator.getIntensitySearchParameters().setKNNSearch(FLAGS_nn_intensity);
    else
        estimator.getIntensitySearchParameters().setRadiusSearch(FLAGS_radius_intensity);

    if (FLAGS_knn_normal)
        estimator.getNormalSearchParameters().setKNNSearch(FLAGS_nn_normal);
    else
        estimator.getNormalSearchParameters().setRadiusSearch(FLAGS_radius_normal);


    estimator.setMinNumberOfPointsIntensity(FLAGS_min_nn_intensity);
    estimator.setMinNumberOfPointsNormal(FLAGS_min_nn_normal);

    estimator.setMaxAdmissibleKNNDistance(FLAGS_max_admissible_distance);

    if (FLAGS_gaussian_sigma > 0)
    {
        estimator.setGaussianWeighting(true);
        estimator.setIntensityGaussianSpatialSigma(FLAGS_gaussian_sigma);

        LOG(INFO) <<"Enabling gaussian filtering of intensity data";
    }


    else
    {
        estimator.setGaussianWeighting(false);
        LOG(INFO) <<"Disabling gaussian filtering of intensity data";
    }


    if ( estimator.compute() != 1)
	{
		LOG(FATAL) <<  "Some error during computations. Please inspect log.";
		return -1;
	}

    calibration::DataHolder::Ptr data = estimator.getCalibrationDataHolder();

    if (FLAGS_filter_nans) // this should led to a smaller database o be saved
        data->ereaseInvalidObservations(true);



    if(!FLAGS_disable_spc_save)
    {
        LOG(INFO) << "saving to file as spc";

        spc::io::serializeToFile(data, FLAGS_out, spc::io::SPC, true, google::CommandlineFlagsIntoString());
    }


	NewSpcPointCloud::Ptr outcloud(new NewSpcPointCloud);

//    if (FLAGS_filter_nans)
//        *outcloud = data->asPointCloud()->filterOutNans({ "intensity", "distance" });
//    else
    outcloud = data->asPointCloud();


//

	//    if (FLAGS_export_as_cloud)
	//    {
	//       LOG(INFO) << "saving to file as spc";

	//        std::string cloud_name = FLAGS_out + "_cloud";
	//        spc::io::serializeToFile(outcloud, cloud_name , spc::io::XML);
	//    }

	if (FLAGS_export_as_ascii) {
		std::string cloud_name = FLAGS_out + "_cloud";
        spc::io::serializeToFile(outcloud, cloud_name, spc::io::ASCII,true, google::CommandlineFlagsIntoString());
	}

	LOG(INFO) << "saved";

	return 1;
}
