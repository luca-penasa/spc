#include "pymm.h"

#include "eigen_numpy.h"

//typedef spc::RBFModel<float>::Ptr ModelPtrT ;
// Tag the OpenSceneGraph ref_ptr type as a smart pointer.
//namespace boost {
//  namespace python {
//	template <class T> struct pointee< spcSharedPtrMacro<T> >
//	{ typedef T type; };
//  }}


namespace spc
{

#include <boost/python.hpp>
using namespace boost::python;
typedef RBFModelEstimator<float> RBFModelEstimatorF;


void PrintMat(const Eigen::MatrixXf &m) {
	std::cout << "Matrix : " << m << std::endl;
}

void TestClass::PrintMat(const Eigen::MatrixXf &m) {
	std::cout << "Matrix : " << m << std::endl;
}




//// Declare the actual type
//BOOST_PYTHON_MODULE(Node) {
//  class_<Node, ref_ptr<Node> >("Node")
//    ;
//}




typedef spc::RBFModel<float> RBFModelF;


BOOST_PYTHON_MODULE(pyspc)
{

	boost::numpy::initialize();
	SetupEigenConverters();


	class_<RBFModelF,RBFModelF::Ptr>("RBFModel")
			.def("getCoefficients", &RBFModelF::getCoefficients)
			.def("getDimensionality", &RBFModelF::getDimensionality)
//			.def("op2", &spc::RBFModel<float>::op2)
//			.def("oper", static_cast< float const (RBFModelF::*)(&RBFModelEstimator::PointT ) > ( &RBFModelF::operator() ))

//relevant:
// http://stackoverflow.com/questions/6050996/boost-python-overloaded-functions-with-default-arguments-problem
//			.def("__call__", &spc::RBFModel<float>::operator() )

			;

	class_<World>("World")
			.def("greet", &World::greet)
			.def("set", &World::set)
			.def("__call__", &World::operator () )

			;

	class_<TestClass>("TestClass")
			.def("PrintMat", &TestClass::PrintMat);
//ase");


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


	def("PrintMat", PrintMat);
}

} // end nspace
