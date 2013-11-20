#ifndef ELEMENT_BASE_H
#define ELEMENT_BASE_H

#include <Eigen/Geometry>
#include <Eigen/Core>
#include <boost/smart_ptr.hpp>

#include <iostream>
#include <sstream>
#include <fstream>

#include <spc/elements/salvable_object.h>

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
    /// I mean the externd cc-types implemented in qGEO as ccHObject types.
    /// \return
    ///
    virtual int __virtual__() {}
};

class spcElementBase: virtual public spcCommon
{
public:
    spcElementBase();

    virtual std::string getSPCClassName() = 0 ;

    virtual int toAsciiMeOnly(std::stringstream &stream) {/*nothing by def*/}
};

}//end nspace

#endif // ELEMENT_BASE_H
