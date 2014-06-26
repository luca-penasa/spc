#ifndef TRANSFERFIELDNN_H
#define TRANSFERFIELDNN_H

#include <spc/elements/PointCloudBase.h>

#include <spc/methods/IntensityCalibrationDataEstimator.h>
namespace spc
{
class PointCloudHelpers
{
public:

    static void transferNormals(PointCloudBase::Ptr from,
                                PointCloudBase::Ptr to,
                                const float &max_distance = 0.1);

    static void computeScatteringAngle(PointCloudBase::Ptr cloud, const std::string fieldname = "angle");

    static void computeDistanceFromSensor(PointCloudBase::Ptr cloud, std::string fieldname = "distance");



};
}

#endif // TRANSFERFIELDNN_H
