#ifndef ELEMENT_IO_H
#define ELEMENT_IO_H
#include <spc/elements/element_base.h>
#include <spc/elements/salvable_object.h>
#include <spc/elements/spcSerializableContainer.h>


//#include <boost/property_tree/ptree.hpp>
//#include <boost/property_tree/xml_parser.hpp>
#include <boost/serialization/shared_ptr.hpp>


#include <pcl/console/print.h>

//#include <boost/foreach.hpp>

namespace spc
{




class spcElementSerializer
{
public:
    spcElementSerializer();


    ///save into an archive
    bool save(const std::string &fname, const spcSerializableObject::ConstPtr obj)
    {
        try {
        std::ofstream ofs(fname);
        boost::archive::xml_oarchive oa(ofs);
        oa << BOOST_SERIALIZATION_NVP(obj);
        ofs.close();
        } catch (...) {
            /// any exception is catched here!
            pcl::console::print_warn("Cannot save to the achive. Something went wrong.");
            return false;
        }

        return true; //todo add error handling here
    }

    ///load from an archive
    bool load(const std::string &fname, spc::spcSerializableObject::Ptr &obj)
    {


        try {
            spc::spcSerializableObject::Ptr tmp;
            std::ifstream ifs(fname);
            boost::archive::xml_iarchive ia(ifs);

            ia >> BOOST_SERIALIZATION_NVP(tmp);

            ifs.close();
            obj = tmp;


        } catch (...) {
            /// any exception is catched here!
            pcl::console::print_warn("Cannot read the achive! Sure about the format?");
            return false;
        }


        return true; //todo add error handling
    }
};


}

#endif // ELEMENT_IO_H
