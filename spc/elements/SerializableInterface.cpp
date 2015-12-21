#include "SerializableInterface.h"
//#include <pcl/console/print.h>
#include <spc/core/logging.h>

namespace spc
{

ISerializable::ISerializable()
{
}

int ISerializable::toAsciiStream(std::ostream &stream) const
{
    LOG(WARNING) << "ascii serialization in base class has been called but nothing done. "
                    "this method must be overridden as needed";

    return -1;
}

EigenTablePtr ISerializable::asEigenTable() const
{
    LOG(INFO) << "Transformation to eigen table object not implemented for this object";
    return nullptr;
}

}//end nspace
