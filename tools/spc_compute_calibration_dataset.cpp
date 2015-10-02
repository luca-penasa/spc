
#include <spc/core/spc_eigen.h>

#include <spc/core/strings.h>

#include <pcl/features/normal_3d.h>

#include <spc/methods/PointCloudEigenIndicesEstimator.h>
#include <spc/io/io_helper.h>
#include <spc/methods/IntensityCalibrationDataEstimator.h>
#include <spc/methods/IntensityCalibrationDataFilter.h>

#include <boost/spirit/home/support/detail/hold_any.hpp>

#include <spc/io/element_io.h>
#include <spc/core/file_operations.h>

#include <spc/core/flagging.h>
#include <spc/core/logging.h>

#include <spc/methods/IntensityCalibratorRBF.h>
using namespace std;
using namespace pcl;
using namespace pcl::console;
using namespace pcl::io;

DEFINE_string(in_clouds, "", "space separated list of input clouds");
DEFINE_string(core_points_cloud, "", "file containing the core points");

DEFINE_string(out, "calibration_data", "out file without extension");

DEFINE_double(search_radius, 0.1,
	"the search radius to use when normals are estimated directly");

DEFINE_double(gaussian_sigma, 0.1,
	"the spatial sigma for intensity sampling with gaussian");

DEFINE_bool(export_as_cloud, true, "save the points as a cloud also");

DEFINE_bool(export_as_ascii, true,
	"save the data as pure ascii, e.g. for inspection");

DEFINE_bool(
	regexp, true,
	"use a regular expression to set in_clouds (found in current directory)");


DEFINE_string(intensity_field, "intensity", "the intensity field");

int main(int argc, char** argv)
{
	google::InitGoogleLogging(argv[0]);

	gflags::SetUsageMessage(
		"computes a dataset of sampled scalar fields for calibrating the device");

	//    FLAGS_logtostderr = 1;
	gflags::ParseCommandLineFlags(&argc, &argv, true);

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

	spc::calibration::CalibrationDataEstimator calibrator;
	calibrator.setInputClouds(sources);

	LOG(INFO) << "Using as intensity field: " << FLAGS_intensity_field;
	calibrator.setIntensityFieldName(FLAGS_intensity_field);

	CloudDataSourceOnDisk keys(FLAGS_core_points_cloud);
	NewSpcPointCloud::Ptr key_cloud = keys.load2();

	calibrator.setInputKeypoints(key_cloud);
	calibrator.setNormalEstimationSearchRadius(FLAGS_search_radius);
	if ( calibrator.compute() != 1)
	{
		LOG(FATAL) <<  "Some error during computations. Please inspect log.";
		return -1;
	}

	calibration::DataHolder::Ptr data = calibrator.getCalibrationDataHolder();

	LOG(INFO) << "saving to file as spc";

	spc::io::serializeToFile(data, FLAGS_out, spc::io::SPC);

	NewSpcPointCloud::Ptr outcloud(new NewSpcPointCloud);

	*outcloud = data->asPointCloud()->filterOutNans({ "intensity", "distance" });

	//    if (FLAGS_export_as_cloud)
	//    {
	//       LOG(INFO) << "saving to file as spc";

	//        std::string cloud_name = FLAGS_out + "_cloud";
	//        spc::io::serializeToFile(outcloud, cloud_name , spc::io::XML);
	//    }

	if (FLAGS_export_as_ascii) {
		std::string cloud_name = FLAGS_out + "_cloud";
		spc::io::serializeToFile(outcloud, cloud_name, spc::io::ASCII);
	}

	LOG(INFO) << "saved";

	return 1;
}
