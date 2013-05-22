#ifndef PYMM_H
#define PYMM_H
#include <spc/micmac/mm_utils.h>
#include <boost/python.hpp>
#include <boost/numpy.hpp>

//here is a small python interface for some common functions
namespace spc
{

char const * pythonStr2cstr(boost::python::str string);

boost::numpy::ndarray fromImageToModelSpace(boost::numpy::ndarray pts2d,
                                                      boost::python::str nuage_filename);

boost::numpy::ndarray fromModelToImageSpace(boost::numpy::ndarray pts3d,
                                                      boost::python::str orientation_filename);
}

#endif // PYMM_H
