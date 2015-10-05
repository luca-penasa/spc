#include "io.h"

#include "eigen_numpy.h"

#include <spc/io/element_io.h>


#include <boost/python/manage_new_object.hpp>
#include <boost/python/return_value_policy.hpp>
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>
#include <boost/python.hpp>


namespace spc
{

using namespace boost::python;



typedef spc::RBFModel<float> RBFModelF;

BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(op, RBFModelF::operator(), 1, 1)


template<typename T>
void wrap_std_vector(const std::string name)
{
	typedef std::vector <T> VT;
	class_< VT>(name.c_str())
			.def(vector_indexing_suite<VT >())
			;
}

BOOST_PYTHON_MODULE(elements)
{
	implicitly_convertible<RBFModelF::Ptr,ISerializable::Ptr>();
	implicitly_convertible<RBFModelF::Ptr,ElementBase::Ptr>();
	implicitly_convertible<ElementBase::Ptr,ISerializable::Ptr>();

	wrap_std_vector<ElementBase::Ptr> ("VectorOfElementBasePtr");

	class_<ISerializable, ISerializable::Ptr, boost::noncopyable>("ISerializable", no_init)
			.def("isSerializable", &ISerializable::isSerializable)
			.def("isAsciiSerializable", &ISerializable::isAsciiSerializable);
			;

	class_<ElementBase, ElementBase::Ptr, boost::noncopyable, bases<ISerializable> >("ElementBase", no_init)
			.def("getPtr", &ElementBase::getPtr)
			.def("clone", pure_virtual(&ElementBase::clone))
			.def("getChilds", &ElementBase::getChilds)
			.def("addChild", &ElementBase::addChild)
			.def("getElementName", &ElementBase::getElementName)
			.def("setElementName", &ElementBase::setElementName)
			.def("removeChild", &ElementBase::removeChild)
			.def("setParent", &ElementBase::getParent)
			;

	class_<RBFModelF,RBFModelF::Ptr, bases<ElementBase> , boost::noncopyable>("RBFModel")
			.def("getCoefficients", &RBFModelF::getCoefficients)
			.def("getDimensionality", &RBFModelF::getDimensionality)
			.def("__call__",  static_cast<float (RBFModelF::*)(const RBFModelF::PointT &) const >  (&RBFModelF::operator() ), op() )
			.def("clone", &RBFModelF::clone)
			.def("__call__",  static_cast<float (RBFModelF::*)(const RBFModelF::PointT &) const >  (&RBFModelF::operator() ), op() )
			;
}


//BOOST_PYTHON_MODULE(methods)

//{
//	typedef RBFModelEstimator<float> RBFModelEstimatorF;


//	class_<RBFModelEstimatorF>("RBFModelEstimator")
//			.def("setPoints", &RBFModelEstimatorF::setPoints)
//			.def("getPoints", &RBFModelEstimatorF::getPoints)
//			.def("setInputValues", &RBFModelEstimatorF::setInputValues)
//			.def("autosetSigma", &RBFModelEstimatorF::autosetSigma)
//			.def("autosetNodes", &RBFModelEstimatorF::autosetNodes)
//			.def("autosetScales", &RBFModelEstimatorF::autosetScales)
//			.def("setLambda", &RBFModelEstimatorF::setLambda)
//			.def("getLambda", &RBFModelEstimatorF::getLambda)
//			.def("getNumberOfPoints", &RBFModelEstimatorF::getNumberOfPoints)
//			.def("initProblem", &RBFModelEstimatorF::initProblem)
//			.def("solveProblem", &RBFModelEstimatorF::solveProblem)
//			.def("getModel", &RBFModelEstimatorF::getModel)
//			;





//}

} // end nspace
