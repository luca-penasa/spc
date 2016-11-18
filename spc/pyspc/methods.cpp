#include "io.h"

#include "eigen_numpy.h"

#include <spc/io/element_io.h>


#include <boost/python/manage_new_object.hpp>
#include <boost/python/return_value_policy.hpp>
#include <spc/methods/Sift1d.h>
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>

namespace spc
{

#include <boost/python.hpp>
using namespace boost::python;

typedef std::vector<std::vector<Eigen::VectorXd>> vofvdoublev;

BOOST_PYTHON_MODULE(methods)

{
	typedef RBFModelEstimator<float> RBFModelEstimatorF;


    class_<vofvdoublev > ("vofvdoubev")
             .def(vector_indexing_suite<vofvdoublev >());

         class_<std::vector<Eigen::VectorXd> > ("vdoublev")
             .def(vector_indexing_suite<std::vector<Eigen::VectorXd> >());

	class_<RBFModelEstimatorF>("RBFModelEstimator")
			.def("setPoints", &RBFModelEstimatorF::setPoints)
			.def("getPoints", &RBFModelEstimatorF::getPoints)
			.def("setInputValues", &RBFModelEstimatorF::setInputValues)
            .def("getInputValues", &RBFModelEstimatorF::getInputValues)

            .def("autosetScales", &RBFModelEstimatorF::autosetScales)

			.def("autosetSigma", &RBFModelEstimatorF::autosetSigma)
			.def("autosetNodes", &RBFModelEstimatorF::autosetNodes)
            .def("setNodes", &RBFModelEstimatorF::setNodes)


			.def("setLambda", &RBFModelEstimatorF::setLambda)
			.def("getLambda", &RBFModelEstimatorF::getLambda)
			.def("getNumberOfPoints", &RBFModelEstimatorF::getNumberOfPoints)
			.def("initProblem", &RBFModelEstimatorF::initProblem)
			.def("solveProblem", &RBFModelEstimatorF::solveProblem)
			.def("getModel", &RBFModelEstimatorF::getModel)
			;


    class_<Sift1d> ("Sift1d")
            .def ("setInputSignal", &Sift1d::setInputSignal)
            .def ("compute", &Sift1d::compute)
            ;

    def("gaussWindow", spc::Sift1d::gaussian_window);
    def("blur", spc::Sift1d::blur);
    def("fftshift", spc::Sift1d::fftshift<Eigen::VectorXd>);





}

} // end nspace
