#include "CalibrationDataHolder.h"

namespace spc
{
namespace calibration
{

DtiClassType CalibrationDataHolder::Type ("CalibrationDataHolder", &ElementBase::Type);

CalibrationDataHolder::CalibrationDataHolder()
{
}

}
}
#include <spc/core/spc_cereal.hpp>
SPC_CEREAL_REGISTER_TYPE(spc::calibration::CalibrationDataHolder)
