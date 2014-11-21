#ifndef INTENSITYCALIBRATORRBF_H
#define INTENSITYCALIBRATORRBF_H
#include <spc/methods/RBFModelEstimator.hpp>
#include <spc/elements/PointCloudBase.h>
#include <spc/elements/EigenTable.h>
#include <spc/methods/IntensityCalibrationDataEstimator.h>
namespace spc
{

class IntensityCalibratorRBF
{


public:
    IntensityCalibratorRBF();

    void extractKeypoints()
    {
        CalibrationDataEstimator extractor;
//        extractor.setInputClouds(in_cloud_names_);

    }

protected:
    std::vector<std::string> in_cloud_names_;

    PointCloudBase::ConstPtr keypoints_on_mat1_;
    PointCloudBase::ConstPtr keypoints_on_other_materials_;


   EigenTable::Ptr data_on_mat1_;
   EigenTable::Ptr data_on_varous_materials_;
};
}

#endif // INTENSITYCALIBRATORRBF_H
