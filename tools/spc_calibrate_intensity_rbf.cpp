#include <spc/core/strings.h>

#include <spc/methods/RBFModelEstimator.hpp>

#include <spc/io/element_io.h>
#include <spc/elements/EigenTable.h>

#include <gflags/gflags.h>
#include <spc/core/logging.h>

#include <spc/elements/NewSpcPointCloud.h>
#include <spc/methods/IntensityCalibratorRBF.h>

using namespace spc;
using Eigen::Matrix;


DEFINE_string(out, "rbf_model", "Out Filename.");


//DEFINE_string(weights_field, "intensity_std", "Used weights for solving the problem");

DEFINE_double(sigma, 0, "Sigma value for the kernel of RBF. If 0 it will be automatically chosen");
DEFINE_double(lambda, 0.01, "Lambda value for regularization (smoothness) of RBF");

DEFINE_bool(angles, true, "If calibrate the model to account for scattering angles");
DEFINE_uint64(poly_order, 1 , "polynomial order for the polynomial part of RBF");
DEFINE_string(n_nodes, "6,6", "Number of splits on each dimension (one for each predictor)"
              "The total numbe of nodes will be the product of all these splits");

DEFINE_string(input_database, "", "an input file containing the variables to be used for calibrating the model");

int main (int argc, char ** argv)
{
    google::InitGoogleLogging(argv[0]);

    google::SetUsageMessage("Compute a rbf model for predicting a given scalar field (e.g. intensity) as function of any other scalar fields."
                            "call as: " + std::string(argv[0]) + " database.spc [or xml/json] [...]");

    FLAGS_logtostderr = 1;
    google::ParseCommandLineFlags(&argc, &argv, true);


    std::string datadb = FLAGS_input_database;

    LOG(INFO) << "working on file " << datadb;




    LOG(INFO) << "calibrating a linear model";

    ISerializable::Ptr o =  spc::io::deserializeFromFile(datadb);

    CHECK(o != NULL) << "cannot deserialize the file";

    calibration::CalibrationDataHolder::Ptr cal_data = spcDynamicPointerCast<calibration::CalibrationDataHolder> (o);


    LOG(INFO) << "going do perform calibration";
    calibration::IntensityCalibratorRBF calibrator;
    calibrator.setCalibrationData(cal_data);

    LOG(INFO) << "computing";

    calibrator.calibrate();

    LOG(INFO) << "Saving the model to " << FLAGS_out;
    spc::io::serializeToFile(calibrator.getModel(), FLAGS_out, spc::io::XML);





    return 1;
}
