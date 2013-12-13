#ifndef CALIBRATION_MODEL_BASE_H
#define CALIBRATION_MODEL_BASE_H

#include <boost/shared_ptr.hpp>

namespace spc
{

class CalibrationModelBase
{
public:
    typedef boost::shared_ptr<CalibrationModelBase> Ptr;
    typedef boost::shared_ptr<const CalibrationModelBase> ConstPtr;


    CalibrationModelBase();

    virtual float getDistanceCorrection(const float &distance) = 0;

};


}//end nspace

#endif // CALIBRATION_MODEL_BASE_H
