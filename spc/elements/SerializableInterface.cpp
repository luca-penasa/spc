#include "SerializableInterface.h"
//#include <pcl/console/print.h>
namespace spc
{

ISerializable::ISerializable()
{
}

EigenTablePtr ISerializable::asEigenTable() const
{
    LOG(INFO) << "Transformation to eigen table object not implemented for this object";
    return nullptr;
}

}//end nspace
