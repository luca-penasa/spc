#pragma once
#ifndef POINTCLOUDBASEWITHSENSOR_H
#define POINTCLOUDBASEWITHSENSOR_H

#include <spc/elements/PointCloudBase.h>

namespace spc {

spcFwdDeclSharedPtr(OrientedSensor)

    class PointCloudBaseWithSensor : public PointCloudBase {

public:
    spcTypedefSharedPtrs(PointCloudBaseWithSensor)

        EXPOSE_TYPE

        PointCloudBaseWithSensor();

    PointCloudBaseWithSensor(const PointCloudBaseWithSensor& other);

    virtual OrientedSensor getSensor() const override;

    virtual void setSensor(const OrientedSensor& sensor) const override;

protected:
    OrientedSensorPtr sensor_;
};
} // end nspace
#endif // POINTCLOUDBASEWITHSENSOR_H
