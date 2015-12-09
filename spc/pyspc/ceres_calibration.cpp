
#include "eigen_numpy.h"


#include "ceres_calibration.h"


#include "elements.h"
#include <spc/core/spc_eigen.h>

#include "eigen_numpy.h"


#include <spc/ceres_calibration/SimpleCalibration.h>

namespace spc
{

void test(const Eigen::VectorXd &e)
{
    LOG(INFO) << e;
}

#include <boost/python.hpp>

using namespace boost::python;




BOOST_PYTHON_MODULE(ceres_calibration)
{

    def("test", &test);

//    def("predict_intensity" )


    def("near_distance_effect", &near_distance_effect_);

    def("predict_intensity_no_angle", &predict_intensity_no_angle_);


    class_<SimpleCalibration> ("SimpleCalibrator")
            .def ("initProblem", &SimpleCalibration::initProblem)
            .def ("solve", &SimpleCalibration::solve)

            .def("predict_no_angle", &SimpleCalibration::predict_no_angle)
//            .def ("testSet", &SimpleCalibration::testSet)

            .def("setNearDistanceLowerBounds", &SimpleCalibration::setNearDistanceLowerBounds)
            .def("setNearDistanceUpperBounds", &SimpleCalibration::setNearDistanceUpperBounds)


            .def ("setNearDistancePars", &SimpleCalibration::setNearDistancePars)
            .def ("getNearDistancePars", &SimpleCalibration::getNearDistancePars)

            .def ("setMaterialPars", &SimpleCalibration::setMaterialPars)
            .def ("getMaterialPars", &SimpleCalibration::getMaterialPars)

            .def ("setRoughnessPars", &SimpleCalibration::setRoughnessPars)
            .def ("getRoughnessPars", &SimpleCalibration::getRoughnessPars)

            .def ("setR", &SimpleCalibration::setR)
            .def ("setI", &SimpleCalibration::setI)
            .def ("setAngle", &SimpleCalibration::setAngle)

            ;
}
}// end nspace
