#include "io.h"

//#include "eigen_numpy.h"

#include <spc/io/element_io.h>


#include <boost/python/manage_new_object.hpp>
#include <boost/python/return_value_policy.hpp>


namespace spc
{

#include <boost/python.hpp>
using namespace boost::python;



BOOST_PYTHON_MODULE(io)
{
	boost::numpy::initialize();
//	SetupEigenConverters();

    enum_<spc::io::ARCHIVE_TYPE>("ARCHIVE_TYPE")
        .value("XML", spc::io::XML)
        .value("SPC", spc::io::SPC)
            .value("JSON", spc::io::JSON)
			.value("ASCII", spc::io::ASCII)
			;


    def("serializeToFile", &spc::io::serializeToFile);
    def("deserializeFromFile", &spc::io::deserializeFromFile);

	def("getV", &spc::getV);

}




} // end nspace
