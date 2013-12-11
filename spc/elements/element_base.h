#ifndef ELEMENT_BASE_H
#define ELEMENT_BASE_H

#include <Eigen/Geometry>
#include <Eigen/Core>
#include <boost/smart_ptr.hpp>

#include <iostream>
#include <sstream>

#include <spc/elements/salvable_object.h>

#include <fstream>
#include <iostream>
#include <boost/archive/xml_iarchive.hpp>
#include <boost/archive/xml_oarchive.hpp>
#include <boost/serialization/string.hpp> // so that boost know how to serialize std::string
#include <boost/serialization/export.hpp>


using namespace Eigen;
namespace spc
{


class spcElementBase: public spcSerializableObject
{
public:
    typedef typename boost::shared_ptr<spcElementBase> Ptr;
    typedef typename boost::shared_ptr<const spcElementBase> ConstPtr;


    spcElementBase () {}


protected:
    friend class boost::serialization::access;

    template <class Archive>
    void serialize(Archive &ar, const unsigned int version)
    {
        ar & boost::serialization::make_nvp("spcSerializableObject", boost::serialization::base_object<spc::spcSerializableObject> (*this));
    }

    std::string getClassName()
    {
        return typeid(this).name();
    }



};

}//end nspace



#endif // ELEMENT_BASE_H
