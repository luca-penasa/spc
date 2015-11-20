#ifndef SPC_CALIBRATION_DATA_ESTIMATOR_H
#define SPC_CALIBRATION_DATA_ESTIMATOR_H

#include <spc/elements/CloudDataSourceOnDisk.h>

#include <spc/elements/calibration/DataHolder.h>
#include <spc/elements/Kernels.hpp>

namespace spc {
namespace calibration {

template <typename ScalarT>
struct CalibrationSearchParameters {
    enum search_t { RADIUS,
                    KNN };

    void setKNNSearch(const size_t& n_neighbors)
    {
        search_type_ = KNN;
        n_neighbors_ = n_neighbors;
    }

    void setRadiusSearch(const ScalarT& search_radius)
    {
        search_type_ = RADIUS;
        sq_search_radius_ = search_radius * search_radius;
        radius_ = search_radius;
    }

    bool isKNN() const
    {
        return KNN == search_type_;
    }

    ScalarT getSearchRadius() const
    {
        return radius_;
    }

    ScalarT getSquaredSearchRadius() const
    {
        return sq_search_radius_;
    }

    size_t getNumberOfNeighbors() const
    {
        return n_neighbors_;
    }

    search_t search_type_ = RADIUS;

    ScalarT sq_search_radius_ = 0.01;
    ScalarT radius_ = 0.1;
    size_t n_neighbors_ = 10;

    inline bool operator==(const CalibrationSearchParameters<ScalarT>& other)
    {

        if (search_type_ != other.search_type_)
            return false;

        if (search_type_ == KNN)
            return n_neighbors_ == other.n_neighbors_;

        else if (search_type_ == RADIUS)
            return (sq_search_radius_ == other.sq_search_radius_) && (radius_ == other.radius_);

        else
            return false;
    }

    inline bool operator!=(const CalibrationSearchParameters<ScalarT>& other)
    {
        return !(*this == other);
    }
};

template <typename ScalarT>
std::ostream& operator<<(std::ostream& os, const CalibrationSearchParameters<ScalarT>& obj)
{

    os << "search method ";
    if (obj.isKNN())
        os << "KNN: " << obj.getNumberOfNeighbors();
    else
        os << "RADIUS: " << obj.getSearchRadius();

    return os;
}

class CalibrationDataEstimator {
public:
    spcTypedefSharedPtrs(CalibrationDataEstimator)

    CalibrationDataEstimator();

    void setInputClouds(std::vector<CloudDataSourceOnDisk::Ptr> cloud_names);

    void setInputKeypoints(const NewSpcPointCloud::Ptr kpoints);

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

    calibration::DataHolder::Ptr getCalibrationDataHolder() const
    {
        return calibration_data_;
    }

    spcSetMacro(IntensityFieldName, intensity_field_name_, std::string)
    spcGetMacro(IntensityFieldName, intensity_field_name_, std::string)

    spcSetMacro(MaterialFieldName, material_field_name_, std::string)
    spcGetMacro(MaterialFieldName, material_field_name_, std::string)

    CalibrationSearchParameters<float>& getNormalSearchParameters()
    {
        return normal_search_pars_;
    }

    CalibrationSearchParameters<float>& getIntensitySearchParameters()
    {
        return intensity_search_pars_;
    }

    spcGetMacro(GaussianWeighting, gaussian_weighting_, bool)
    spcSetMacro(GaussianWeighting, gaussian_weighting_, bool)

    spcGetMacro(MinNumberOfPointsIntensity, min_number_of_points_intensity_, size_t)
    spcSetMacro(MinNumberOfPointsIntensity, min_number_of_points_intensity_, size_t)

    spcGetMacro(MinNumberOfPointsNormal, min_number_of_points_normal_, size_t)
    spcSetMacro(MinNumberOfPointsNormal, min_number_of_points_normal_, size_t)

    spcGetMacro(MaxAdmissibleKNNDistance, max_admissible_knn_distance_,float)
    spcSetMacro(MaxAdmissibleKNNDistance, max_admissible_knn_distance_, float)

    private:


    std::vector<CloudDataSourceOnDisk::Ptr> input_clouds_ondisk_;

    float intensity_estimation_spatial_sigma_ = 0.1;

    calibration::DataHolder::Ptr calibration_data_;

    RBFBase<float>::Ptr kernel_;

    size_t min_number_of_points_intensity_ = 10;
    size_t min_number_of_points_normal_ = 10;

    float max_admissible_knn_distance_ = 0.5;

    std::string intensity_field_name_ = "intensity";

    std::string material_field_name_ = "material";

    CalibrationSearchParameters<float> normal_search_pars_;
    CalibrationSearchParameters<float> intensity_search_pars_;

    bool gaussian_weighting_ = false;


    std::map<CloudDataSourceOnDisk::Ptr, Eigen::Vector3f> cloud_to_sensor_position_;
};
}
}
#endif // INTENSITYAUTOCALIBRATOR_H
