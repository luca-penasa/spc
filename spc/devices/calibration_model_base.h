#ifndef CALIBRATION_MODEL_BASE_H
#define CALIBRATION_MODEL_BASE_H

#include <spc/common/macros.h>
#include <spc/elements/spcObject.h>

namespace spc
{

class CalibrationModelBase: public spcObject
{
public:
    SPC_OBJECT(CalibrationModelBase)


    CalibrationModelBase();

    virtual float getDistanceCorrection(const float &distance) = 0;


};


}//end nspace

#endif // CALIBRATION_MODEL_BASE_H
