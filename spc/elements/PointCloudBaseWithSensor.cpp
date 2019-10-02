
#include "PointCloudBaseWithSensor.h"
#include <spc/elements/OrientedSensor.h>


PointCloudBaseWithSensor::PointCloudBaseWithSensor()
    : sensor_(new OrientedSensor)
{
}

PointCloudBaseWithSensor::PointCloudBaseWithSensor(const PointCloudBaseWithSensor &other)
{
    sensor_ = spcDynamicPointerCast<OrientedSensor>(other.sensor_->clone());
}

OrientedSensor PointCloudBaseWithSensor::getSensor() const
{
    return *sensor_;
}

void PointCloudBaseWithSensor::setSensor(const OrientedSensor &sensor) const
{
    *sensor_ = sensor;
}
