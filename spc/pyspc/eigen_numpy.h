#ifndef _EIGEN_NUMPY_H
#define _EIGEN_NUMPY_H


// boost
#include <boost/version.hpp>
#include <boost/config.hpp>

#include <memory>

// this comes from here https://github.com/mapnik/mapnik/blob/804c90d46e0f1efd888bf75e103bc5396ddf35da/bindings/python/boost_std_shared_shim.hpp
// solves an issue when compiling with clang!

#if defined( BOOST_NO_CXX11_SMART_PTR )
namespace boost{


template<class T> T * get_pointer( std::unique_ptr<T> const& p )
{
	return p.get();
}

template<class T> T * get_pointer( std::shared_ptr<T> const& p )
{
	return p.get();
}


} // end nspace
#endif




//#if BOOST_VERSION < 105300 || defined BOOST_NO_CXX11_SMART_PTR

//// https://github.com/mapnik/mapnik/issues/2022
//#include <memory>

//namespace boost {
//template<class T> const T* get_pointer(std::shared_ptr<T> const& p)
//{
//	return p.get();
//}

//template<class T> T* get_pointer(std::shared_ptr<T>& p)
//{
//	return p.get();
//}
//} // namespace boost

//#endif


#include <boost/python.hpp>
#include <boost/numpy.hpp>


void SetupEigenConverters();

#endif
