#ifndef CALIBRATIONFUNCTION_H
#define CALIBRATIONFUNCTION_H

#include <spc/elements/EigenFunctionBase.h>

namespace spc
{


class CalibrationFunction: public ElementBase
{
public:
    SPC_OBJECT(CalibrationFunction)
    EXPOSE_TYPE

    virtual std::vector<std::string> getFieldsRequiredForCalibration() const = 0;

    virtual std::vector<std::string> getFieldsRequiredForApply() const = 0;


};


}//end nspace

#endif // CALIBRATIONFUNCTION_H
