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

using namespace Eigen;
namespace spc
{

class spcCommon
{
public:
    spcCommon() {}
    ///
    /// \brief __virtual__ we need to have at least 1 virtual function implemented
    /// here so we will able to dynamic cast from any object that inherit from spcCommon
    /// I mean the extern cc-types implemented in qGEO as ccHObject types.
    /// \return
    ///
    virtual int __virtual__() {}
};

class spcElementBase: virtual public spcCommon, public spcSalvableObject
{
public:
    typedef typename boost::shared_ptr<spcElementBase> Ptr;
    typedef typename boost::shared_ptr<const spcElementBase> ConstPtr;



    /// each element must have an hopefully-unique name
    spcElementBase(const std::string name);

    /// retrun the name
    virtual std::string getSPCClassName() {return class_name_;}


private:

    friend class boost::serialization::access;

    template <class Archive>
    void serialize(Archive &ar, const unsigned int version)
    {
        /// for now we dont need to call base-classes serialization methods
        ar & BOOST_SERIALIZATION_NVP(class_name_);
    }


    std::string class_name_;
};

}//end nspace

#endif // ELEMENT_BASE_H
