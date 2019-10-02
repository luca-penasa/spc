#ifndef TRANSFERFIELDNN_H
#define TRANSFERFIELDNN_H

#include <spc/elements/PointCloudBase.h>

#include <spc/methods/IntensityCalibrationDataEstimator.h>
namespace spc
{
class PointCloudHelpers
{
public:
#ifdef SPC_WITH_PCL

    static int transferNormals(PointCloudBase::Ptr from,
                                PointCloudBase::Ptr to,
                                const float &max_distance = 0.1);
#endif
    static int computeScatteringAngle(PointCloudBase::Ptr cloud,
                                      const std::string angle_fieldname = "angle");

    static int computeDistanceFromSensor(PointCloudBase::Ptr cloud,
                                         std::string fieldname = "distance");


    static int transferFieldsNN(PointCloudBase::Ptr from,
                                PointCloudBase::Ptr to,
                                const float &max_distance,
                                std::vector<std::string> fields);

};
}

#endif // TRANSFERFIELDNN_H
