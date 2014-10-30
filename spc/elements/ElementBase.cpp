#include "ElementBase.h"
//#include <fstream>

#include <pcl/console/print.h>
#include <spc/elements/UniversalUniqueID.h>
#include <spc/elements/SerializableInterface.h>
namespace spc
{

DtiClassType ElementBase::Type ("ElementBase"); // it is a root


void ElementBase::update()
{
    // do stuff in subclasses
    pcl::console::print_debug(
        "Called update in base method, update() not implemented in %s",
        this->getType()->getClassName().c_str());
    modified_ = false;
}



UniversalUniqueID ElementBase::getUniversalUUID() const
{
    return universal_id_;
}



} // end nspace


#include <spc/core/spc_cereal.hpp>
SPC_CEREAL_REGISTER_TYPE(spc::ElementBase)
