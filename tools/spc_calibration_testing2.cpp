

//#include "ceres/ceres.h"
//#include "glog/logging.h"

//#include <spc/io/element_io.h>
//#include <spc/elements/EigenTable.h>

//#include <spc/io/AsciiEigenTableWriter.h>

//#include <spc/calibration/CalibrationFactors.h>

#include <spc/core/strings.h>


#include <spc/ceres_calibration/CalibrationManager.h>

#include <spc/ceres_calibration/LinearCalibrator.h>

#include <spc/methods/RBFModelEstimator.hpp>

#include <spc/core/flagging.h>

//#include <gflags/gflags.h>
//#include <glog/logging.h>

using namespace spc;
using Eigen::Matrix;


//DEFINE_bool(linear, true, "linear way");


DEFINE_string(out, "model.xml", "Out Filename.");
DEFINE_string(distance_field_name, "distance", "The field name of the distance field");
DEFINE_string(intensity_field_name, "intensity", "The field name of the intensity field");
DEFINE_string(angle_field_name, "angle", "The field name of the angle field");
DEFINE_string(intensity_std_field_name, "intensity_std", "The field name of standard deviation of intensity");

DEFINE_bool(only_distance, false, "Calibrate both distance and angle effects");
DEFINE_bool(weights, false, "Use weights for solving the problem");

DEFINE_double(sigma, 0, "Sigma value for the kernel of RBF. If 0 it will be automatically chosen");
DEFINE_double(lambda, 0.01, "Lambda value for regularization (smoothness) of RBF");


DEFINE_uint64(poly_order, 1 , "polynomial order for the polynomial part of RBF");
DEFINE_uint64(n_distance_nodes, 6 , "Number of nodes in the distance space \n The total number of nodes"
                                    "will be n_distance_nodes * n_angle_nodes. So be careful.");
DEFINE_uint64(n_angle_nodes, 6 , "Number of nodes in the angle space");


int main (int argc, char ** argv)
{
    google::InitGoogleLogging(argv[0]);

//    google::FLAGS_logtostderr = 1;

    google::ParseCommandLineFlags(&argc, &argv, true);




    CHECK(argc == 2) << "please provide an argument";

    std::string datadb = argv[1];

    LOG(INFO) << "working on file " << datadb;


    CalibratorManager man;
    man.readFile(datadb);
    man.setSolverOptions();

    man.setUpProblem();

    std::cout << "n pars blocks " << man.getNumberOfParameterBlocks() << std::endl;

    for (int i =0 ; i < man.getNumberOfParameterBlocks(); ++i)
    {
        std::cout << "par block " << i << " has size " << man.getSizeOfParameterBlock(i) << std::endl;
    }


    man.solve();
    man.printFullReport();
    man.printAllParameters();

    man.savePrediction("/home/luca/test.txt");


    return 1;
}
