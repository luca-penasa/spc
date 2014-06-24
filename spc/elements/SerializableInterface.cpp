#include "SerializableInterface.h"
#include <pcl/console/print.h>
namespace spc
{

ISerializable::ISerializable()
{
}

EigenTablePtr ISerializable::asEigenTable() const
{
    pcl::console::print_warn("Transformation to eigen table object not implemented for this object\n");
}

}//end nspace
