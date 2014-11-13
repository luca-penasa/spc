#include "CloudDataSourceOnDisk.h"

namespace spc
{


DtiClassType CloudDataSourceOnDisk::Type ("CloudDataSourceOnDisk", &ElementBase::Type);






}//end nspace


#include <spc/core/spc_cereal.hpp>
SPC_CEREAL_REGISTER_TYPE(spc::CloudDataSourceOnDisk)
