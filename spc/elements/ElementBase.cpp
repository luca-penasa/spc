#include "ElementBase.h"
//#include <fstream>

#include <pcl/console/print.h>


namespace spc
{

void spcObject::modified()
{
    modified_ = true;
}

void spcObject::update()
{
    // do stuff in subclasses
    pcl::console::print_debug("Called update in base method, update() not implemented in %s", this->getClassName().c_str());
    modified_ = false;
}

bool spcObject::isSPCSerializable() const
{
    return true;
}

bool spcObject::canBeSavedAsAscii() const
{
    return false;
}

UniversalUniqueID spcObject::getUniversalUUID() const
{
    return universal_id_;
}







}//end nspace


