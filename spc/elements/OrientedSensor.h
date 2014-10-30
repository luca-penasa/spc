#ifndef ORIENTEDSENSOR_H
#define ORIENTEDSENSOR_H
#include <spc/elements/ElementBase.h>


namespace spc
{

class OrientedSensor: public ElementBase
{
public:

    SPC_OBJECT(OrientedSensor)
    EXPOSE_TYPE

    OrientedSensor()
    {

    }

    OrientedSensor(Eigen::Vector4f sensor_position,
                   Eigen::Quaternionf sensor_orientation )
    {
        sensor_position_ = sensor_position;
        sensor_orientation_ = sensor_orientation;
    }

    spcSetMacro(Position, sensor_position_, Eigen::Vector4f)
    spcGetMacro(Position, sensor_position_, Eigen::Vector4f)

    spcGetMacro(Orientation, sensor_orientation_, Eigen::Quaternionf)
    spcSetMacro(Orientation, sensor_orientation_, Eigen::Quaternionf)


protected:
    Eigen::Vector4f sensor_position_;
    Eigen::Quaternionf sensor_orientation_;
};

}//end nspace

#endif // ORIENTEDSENSOR_H