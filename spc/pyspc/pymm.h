#ifndef PYMM_H
#define PYMM_H
//#include <spc/micmac/mm_utils.h>
#include <boost/python.hpp>
#include <boost/numpy.hpp>

//#include <Eigen/Eigen>?

#include <spc/core/spc_eigen.h>



#include <spc/methods/RBFModelEstimator.h>

// here is a small python interface for some common functions
namespace spc
{

class TestClass
{
public:
void PrintMat(const Eigen::MatrixXf& m);

};

void PrintMat(const Eigen::MatrixXf& m);



struct World
{
	void set(std::string msg) { this->msg = msg; }
	std::string greet() { return msg; }
	std::string msg;
};

//char const *pythonStr2cstr(boost::python::str string);

//boost::numpy::ndarray fromImageToModelSpace(boost::numpy::ndarray pts2d,
//                                            boost::python::str nuage_filename);

//boost::numpy::ndarray
//fromModelToImageSpace(boost::numpy::ndarray pts3d,
//                      boost::python::str orientation_filename);
}

#endif // PYMM_H
