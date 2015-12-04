
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



    class_<SimpleCalibration> ("SimpleCalibrator")
            .def ("initProblem", &SimpleCalibration::initProblem)
            .def ("solve", &SimpleCalibration::solve)

            .def ("testSet", &SimpleCalibration::testSet)


            .def ("setNearDistancePars", &SimpleCalibration::setNearDistancePars)
            .def ("setMaterialPars", &SimpleCalibration::setMaterialPars)
            .def ("setRoughnessPars", &SimpleCalibration::setRoughnessPars)

            .def ("setR", &SimpleCalibration::setR)
            .def ("setI", &SimpleCalibration::setI)
            .def ("setAngle", &SimpleCalibration::setAngle)

            ;
}
}// end nspace
