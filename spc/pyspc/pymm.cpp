#include "pymm.h"

#include "eigen_numpy.h"

#include <spc/io/element_io.h>

//typedef spc::RBFModel<float>::Ptr ModelPtrT ;
// Tag the OpenSceneGraph ref_ptr type as a smart pointer.
//namespace boost {
//  namespace python {
//	template <class T> struct pointee< spcSharedPtrMacro<T> >
//	{ typedef T type; };
//  }}


#include <boost/python/manage_new_object.hpp>
#include <boost/python/return_value_policy.hpp>


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


int pass_ptr(ISerializable::Ptr const & ptr)
{
    std::cout << ptr << std::endl;
}

RBFModelF::Ptr getPtr()
{
    return RBFModelF::Ptr(new RBFModelF);
}


BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(op, RBFModelF::operator(), 1, 1)



BOOST_PYTHON_MODULE(pyspc)
{

    def("pass_ptr", &pass_ptr);
    def("getPtr", &getPtr);


	boost::numpy::initialize();
	SetupEigenConverters();

    enum_<spc::io::ARCHIVE_TYPE>("ARCHIVE_TYPE")
        .value("XML", spc::io::XML)
        .value("SPC", spc::io::SPC)
            .value("JSON", spc::io::JSON)
            .value("ASCII", spc::io::ASCII)

        ;

    def("serializeToFile", &spc::io::serializeToFile);
    def("deserializeFromFile", &spc::io::deserializeFromFile);





    class_<ISerializable, ISerializable::Ptr, boost::noncopyable>("ISerializable", no_init)
            .def("isSerializable", &ISerializable::isSerializable)
            .def("isAsciiSerializable", &ISerializable::isAsciiSerializable);
            ;


    class_<ElementBase, ElementBase::Ptr, boost::noncopyable, bases<ISerializable> >("ElementBase", no_init)
            .def("getPtr", &ElementBase::getPtr)
            .def("clone", &ElementBase::clone)
            .def("clone2", &ElementBase::clone2)

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

			;

	class_<World>("World")
			.def("greet", &World::greet)
			.def("set", &World::set)
//			.def("__call__", &World::operator () )

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

implicitly_convertible<RBFModelF::Ptr,ISerializable::Ptr>();
//        register_ptr_to_python< ElementBase::Ptr >();
//        register_ptr_to_python< ISerializable::Ptr >();
//        register_ptr_to_python< RBFModelF::Ptr >();
}

} // end nspace
