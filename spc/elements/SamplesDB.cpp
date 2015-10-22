#include <spc/elements/SamplesDB.h>
//#include <pcl/console/print.h>


namespace spc
{
DtiClassType SamplesDB::Type ("SamplesDB", &ElementBase::Type);

}

#include <spc/core/spc_cereal.hpp>
SPC_CEREAL_REGISTER_TYPE(spc::SamplesDB)
