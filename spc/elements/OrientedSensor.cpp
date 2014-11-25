#include "OrientedSensor.h"

namespace spc
{

DtiClassType OrientedSensor::Type = DtiClassType("OrientedSensor", &ElementBase::Type);





}

#include <spc/core/spc_cereal.hpp>
SPC_CEREAL_REGISTER_TYPE(spc::OrientedSensor)
