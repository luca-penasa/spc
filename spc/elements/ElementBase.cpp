#include "ElementBase.h"
//#include <fstream>

#include <pcl/console/print.h>

namespace spc
{

void ElementBase::modified()
{
    modified_ = true;
}

void ElementBase::update()
{
    // do stuff in subclasses
    pcl::console::print_debug(
        "Called update in base method, update() not implemented in %s",
        this->getClassName().c_str());
    modified_ = false;
}

bool ElementBase::isSPCSerializable() const
{
    return true;
}

bool ElementBase::canBeSavedAsAscii() const
{
    return false;
}

UniversalUniqueID ElementBase::getUniversalUUID() const
{
    return universal_id_;
}

} // end nspace
