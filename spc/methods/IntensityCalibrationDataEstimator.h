#ifndef SPC_CALIBRATION_DATA_ESTIMATOR_H
#define SPC_CALIBRATION_DATA_ESTIMATOR_H

#include <pcl/search/flann_search.h>
#include <pcl/search/impl/flann_search.hpp>
#include <pcl/common/centroid.h>
#include <pcl/common/distances.h>
#include <pcl/console/time.h>

#include <pcl/io/pcd_io.h>
#include <spc/methods/PointCloudEigenIndicesEstimator.h>
#include <boost/any.hpp>
#include <spc/core/std_helpers.hpp>


#include <spc/elements/EigenTable.h>

#include <spc/elements/CloudDataSourceOnDisk.h>

#include <spc/elements/calibration/CalibrationKeypoint.h>

namespace spc
{


class CalibrationDataEstimator
{
public:
    spcTypedefSharedPtrs(CalibrationDataEstimator)


    enum INTENSITY_ESTIMATION_METHOD {
        SIMPLE_AVERAGE = 0,
        GAUSSIAN_ESTIMATION
    };

    CalibrationDataEstimator();

    void setInputClouds(std::vector<CloudDataSourceOnDisk::Ptr> cloud_names);

    void setInputKeypoints(PointCloudBase::ConstPtr kpoints);

    int compute();

//    void getNearestNormal(const Eigen::Vector3f &point, float &nx, float &ny,
//                          float &nz, float sq_dist_limit, float &eigenratio);



    void setIntensityEstimationMethod(const INTENSITY_ESTIMATION_METHOD met)
    {
        intensity_estimation_method_ = met;
    }

    EigenTable::Ptr getCalibrationDB();

    void setSearchRadius(const float rad);

    void setIntensityGaussianSpatialSigma(const float rad)
    {
        intensity_estimation_spatial_sigma_ = rad;
    }

    /** \brief The bilateral filter Gaussian distance kernel.
      * \param[in] sq_dist the spatial distance (distance or intensity)
      * \param[in] sigma standard deviation
      */
    static inline float kernel(const float &sq_dist, const float &sigma)
    {
        return (exp(-sq_dist / (2 * sigma * sigma)));
    }

    //! compute the gaussian smoothed value for a given point
    static float computeGaussianSmoothedIntensity(const PointCloudBase::ConstPtr in_cloud,
        const std::vector<int> &indices, const std::vector<float> &sq_distances,
        const float &kernel_sigma, float &intensity_std, const bool exclude_first_id = false);

    static float getAverageIntensity(const PointCloudBase::ConstPtr cloud,
                                     std::vector<int> ids,
                                     float &std);

    // on current cloud!
    void computeSampleParameters(const size_t core_point_id,
                                        const float search_radius);

    static float getMinimumAngleBetweenVectors(const Eigen::Vector3f x_,
                                               const Eigen::Vector3f y_);

    size_t getNumberOfClouds() const
    {
        return input_clouds_ondisk_.size();
    }

    void setMaximumDistanceForGettingNormal(const float &val)
    {
        maximum_squared_distance_for_normal_getting_ = val * val;
    }

    void fillCalibrationDataAtkeypointForCloud(calibration::PerCloudCalibrationData::Ptr data_holder,
                                               PointCloudBase::Ptr cloud)

    {
        PointCloudBase::SearcherT::ConstPtr searcher = cloud->getSearcher();

        std::vector<std::pair<size_t, float>> matches;
        searcher->radiusSearch(data_holder->parent_keypoint->position, intensity_estimation_spatial_sigma_ * 4, matches);



    }



private:
    /////////////////// STRING STUFF //////////////////////////////
    //! list of pcd files to use for calibration
    std::vector<CloudDataSourceOnDisk::Ptr> input_clouds_ondisk_;

    //! the core points
    PointCloudBase::ConstPtr keypoints_cloud_;

    //////////////////// CURRENT SENSOR STUFF /////////////////
    //! current sensor position
    Eigen::Vector4f current_sensor_center_;

    ////////////////// CONFIGURATIONS //////////////////////
    INTENSITY_ESTIMATION_METHOD intensity_estimation_method_;

    float normal_estimation_search_radius_;

    /////////////// USED WHEN GAUSSIAN ESTIMATION /////////////
    float intensity_estimation_spatial_sigma_;

    ////////////// USED ONLY WITH PRECOMPUTED NORMALS ////////////
    float maximum_squared_distance_for_normal_getting_;

    ////////////// OUTPUT DATABASE /////////////////////////
    EigenTable::Ptr db_;

    std::vector<PointSet<>> accumulators_;


    std::vector<calibration::CalibrationKeyPoint::Ptr> keypoints_;
};
}
#endif // INTENSITYAUTOCALIBRATOR_H
