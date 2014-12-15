#ifndef SPC_CALIBRATION_DATA_ESTIMATOR_H
#define SPC_CALIBRATION_DATA_ESTIMATOR_H


#include <spc/elements/CloudDataSourceOnDisk.h>

#include <spc/elements/calibration/DataHolder.h>
#include <spc/elements/Kernels.hpp>

namespace spc
{
namespace calibration
{

class CalibrationDataEstimator
{
public:
    spcTypedefSharedPtrs(CalibrationDataEstimator)


    CalibrationDataEstimator();

    void setInputClouds(std::vector<CloudDataSourceOnDisk::Ptr> cloud_names);

    void setInputKeypoints(NewSpcPointCloud::ConstPtr kpoints);

    void setNormalEstimationSearchRadius(const float rad);

    void setIntensityGaussianSpatialSigma(const float rad)
    {
        intensity_estimation_spatial_sigma_ = rad;
        kernel_->setScale(rad);
    }

    void setMaterialsFieldName(const std::string material_fname)
    {
        material_field_name_ = material_fname;
    }

    void extractDataForKeypointAndCloud(calibration::Observation::Ptr data_holder,
                                               NewSpcPointCloud::Ptr cloud);

    void computeDerivedData();

    int compute();

    static float getMinimumAngleBetweenVectors(const Eigen::Vector3f x_,
                                               const Eigen::Vector3f y_);

    static void decompressMatches(const std::vector<std::pair<size_t, float>> &matches, std::vector<size_t> &ids, Eigen::VectorXf &sq_distances)
    {
        for (std::pair<size_t, float> pair: matches)
        {
            ids.push_back(pair.first);
            sq_distances.push_back(pair.second);
        }
    }

    calibration::DataHolder::Ptr getCalibrationDataHolder() const
    {
        return calibration_data_;
    }


private:
    /////////////////// STRING STUFF //////////////////////////////
    //! list of pcd files to use for calibration
    std::vector<CloudDataSourceOnDisk::Ptr> input_clouds_ondisk_;

    ////////////////// CONFIGURATIONS //////////////////////
    float normal_estimation_search_radius_;

    /////////////// USED WHEN GAUSSIAN ESTIMATION /////////////
    float intensity_estimation_spatial_sigma_ = 0.1;

    ////////////// OUTPUT DATABASE /////////////////////////
    calibration::DataHolder::Ptr calibration_data_;

    RBFBase<float>::Ptr kernel_ ;

    std::string intensity_field_name_ = "intensity";

    std::string material_field_name_ = "material";

    size_t min_number_of_points_for_normal_estimation_ = 10;

    size_t min_number_of_points_for_intensity_estimation_ = 20;
};
}
}
#endif // INTENSITYAUTOCALIBRATOR_H
