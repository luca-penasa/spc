#include "io.h"

#include "eigen_numpy.h"

#include <spc/io/element_io.h>


#include <boost/python/manage_new_object.hpp>
#include <boost/python/return_value_policy.hpp>


namespace spc
{

#include <boost/python.hpp>
using namespace boost::python;




BOOST_PYTHON_MODULE(methods)

{
	typedef RBFModelEstimator<float> RBFModelEstimatorF;


	class_<RBFModelEstimatorF>("RBFModelEstimator")
			.def("setPoints", &RBFModelEstimatorF::setPoints)
			.def("getPoints", &RBFModelEstimatorF::getPoints)
			.def("setInputValues", &RBFModelEstimatorF::setInputValues)
			.def("autosetSigma", &RBFModelEstimatorF::autosetSigma)
			.def("autosetNodes", &RBFModelEstimatorF::autosetNodes)
			.def("autosetScales", &RBFModelEstimatorF::autosetScales)
			.def("setLambda", &RBFModelEstimatorF::setLambda)
			.def("getLambda", &RBFModelEstimatorF::getLambda)
			.def("getNumberOfPoints", &RBFModelEstimatorF::getNumberOfPoints)
			.def("initProblem", &RBFModelEstimatorF::initProblem)
			.def("solveProblem", &RBFModelEstimatorF::solveProblem)
			.def("getModel", &RBFModelEstimatorF::getModel)
			;





}

} // end nspace
