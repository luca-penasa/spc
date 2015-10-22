#include "ElementsFactory.h"
#include <spc/elements/ElementBase.h>
#include <spc/elements/Attitude.h>
namespace spc
{


ElementBasePtr ElementsFactory::create(const std::string name)
{
    ElementBasePtr ptr;

    if (name == spc::Attitude::Type.getClassName())
        ptr = ElementBasePtr(new spc::Attitude);

    else
        LOG(INFO) << "Requested object not found in factory";
    return ptr;
}



}
