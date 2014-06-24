#ifndef CALIBRATION_MODEL_BASE_H
#define CALIBRATION_MODEL_BASE_H

#include <spc/elements/macros.h>
#include <spc/elements/ElementBase.h>

namespace spc
{

class CalibrationModelBase : public ElementBase
{
public:
    SPC_OBJECT(CalibrationModelBase)
    EXPOSE_TYPE
    CalibrationModelBase();

    virtual float getDistanceCorrection(const float &distance) = 0;
};

} // end nspace

#endif // CALIBRATION_MODEL_BASE_H
