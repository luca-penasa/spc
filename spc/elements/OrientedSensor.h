#ifndef ORIENTEDSENSOR_H
#define ORIENTEDSENSOR_H
#include <spc/elements/ElementBase.h>


namespace spc
{

class OrientedSensor: public ElementBase
{
public:

    SPC_ELEMENT(OrientedSensor)
    EXPOSE_TYPE

    OrientedSensor()
    {
        sensor_orientation_.coeffs().fill(spcNANMacro);
        sensor_position_.fill(spcNANMacro);
    }

    OrientedSensor(const OrientedSensor & other) : ElementBase(other)
    {
        sensor_position_ = other.sensor_position_;
        sensor_orientation_ = other.sensor_orientation_;
    }

    OrientedSensor(const Eigen::Vector4f & sensor_position,
                   const Eigen::Quaternionf & sensor_orientation )
    {
        sensor_position_ = sensor_position;
        sensor_orientation_ = sensor_orientation;
    }

    spcSetMacro(Position, sensor_position_, Eigen::Vector4f)
    spcGetMacro(Position, sensor_position_, Eigen::Vector4f)

    spcGetMacro(Orientation, sensor_orientation_, Eigen::Quaternionf)
    spcSetMacro(Orientation, sensor_orientation_, Eigen::Quaternionf)

    private:
        friend class cereal::access;

        template <class Archive> void serialize(Archive &ar)
        {
            ar(cereal::base_class<spc::ElementBase> (this),
               CEREAL_NVP(sensor_orientation_),
               CEREAL_NVP(sensor_position_) );
        }


protected:
    Eigen::Vector4f sensor_position_ ;
    Eigen::Quaternionf sensor_orientation_;
};

}//end nspace

#endif // ORIENTEDSENSOR_H
