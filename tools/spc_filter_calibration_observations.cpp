#include <spc/core/flagging.h>
#include <spc/core/logging.h>

#include <spc/io/element_io.h>

#include <spc/elements/calibration/DataHolder.h>

DEFINE_string(in, "", "input calibration data to be filtered");

DEFINE_string(out, "calibration_data_filtered.spc",
	"output (filtered) observations data");

namespace spc {
namespace calibration {
	class FilterCalibrationObservations {
	public:
		//! in the case we implement new strategies
		enum filter_strategy { SQUARES = 0 };

		FilterCalibrationObservations() {}
		~FilterCalibrationObservations() {}

		int loadData(const std::string& infile)
		{
			spc::ISerializable::Ptr file = spc::io::deserializeFromFile(infile);

			if (!file) {
				LOG(ERROR) << "cannot load file " << infile;
				return -1;
			}

			in_data_ = spcDynamicPointerCast<spc::calibration::DataHolder>(file);

			if (!in_data_) {
				LOG(ERROR) << "Cannot cast file to a data holder type";
				return -1;
			}

			return 1;
		}

		int filter() { return 1; }

	private:
		calibration::DataHolder::Ptr in_data_;
		filter_strategy strategy_ = SQUARES;
	};
} // end calib
} // end spc

int main(int argc, char** argv)
{
	google::InitGoogleLogging(argv[0]);

	gflags::SetUsageMessage("Filter the observations for calibration");

	gflags::ParseCommandLineFlags(&argc, &argv, true);

	FLAGS_logtostderr = 1;

	FLAGS_colorlogtostderr = 1;

	spc::calibration::FilterCalibrationObservations filter;
	if (filter.loadData(FLAGS_in) == -1)
		LOG(FATAL) << "Problems loading the in file " << FLAGS_in
				   << " check settings and log";

	LOG(INFO) << "data loaded from " << FLAGS_in;

	return 1;
}
