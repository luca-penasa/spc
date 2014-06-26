#pragma once
#ifndef INTENSITYCALIBRATIONAPPLY_H
#define INTENSITYCALIBRATIONAPPLY_H

#include <spc/elements/PointCloudBase.h>
#include <spc/elements/EigenFunctionBase.h>

#include <spc/methods/TransferFieldNN.h>
namespace spc
{
class IntensityCalibrationApplier
{
public:
    SPC_OBJECT(IntensityCalibrationApplier)

    enum DETRENDING_METHOD {
        DIVIDE = 0,
        SUBTRACT
    };

    IntensityCalibrationApplier();

    void setDistanceField(const std::string &_arg)
    {
        distance_field_name_ = _arg;
    }

    void setAngleField(const std::string &_arg)
    {
        angle_field_name_ = _arg;
    }

    void setIntensityField(const std::string &_arg)
    {
        intensity_field_name_ = _arg;
    }

    int compute();

    void setNormalsCloud(PointCloudBase::Ptr n_cloud)
    {
        cloud_with_normals_ = n_cloud;
    }

    void setMaxDistanceForNormal(const float &d)
    {
        max_distance_normals_ = d;
    }

    void setCloudToCalibrate( PointCloudBase::Ptr c)
    {
        cloud_ = c;
    }

    void setCalibrationFunction(const EigenFunctionBase::Ptr cal_function)
    {
        cal_function_ = cal_function;
    }


protected:
    PointCloudBase::Ptr cloud_;

    PointCloudBase::Ptr cloud_with_normals_;

    EigenFunctionBase::Ptr cal_function_;

    std::string distance_field_name_ = "distance";
    std::string angle_field_name_ = "angle";
    std::string intensity_field_name_ = "intensity";

    DETRENDING_METHOD method_ = SUBTRACT;


    float max_distance_normals_ = 0.1;
};

} // end nspace

#endif // INTENSITYCALIBRATIONAPPLY_H
