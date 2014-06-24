#ifndef SPC_CALIBRATION_DATA_ESTIMATOR_H
#define SPC_CALIBRATION_DATA_ESTIMATOR_H

//#include <vector>
//#include <string>

//#include <spc/elements/clouds/generic_cloud.h>

#include <pcl/search/flann_search.h>
#include <pcl/search/impl/flann_search.hpp>
#include <pcl/common/centroid.h>
#include <pcl/common/distances.h>
#include <pcl/console/time.h>

#include <pcl/io/pcd_io.h>
#include <spc/methods/PointCloudEigenIndicesEstimator.h>
#include <spc/methods/geometry.h>
#include <spc/elements/Plane.h>

#include <boost/any.hpp>
#include <spc/methods/std_helpers.hpp>
//#include <boost/spirit/home/support/detail/hold_any.hpp>

//#include <spc/elements/SamplesDB.h>
//#include <spc/elements/Fields.h>

//#include <spc/elements/Fields.h>

#include <spc/elements/EigenTable.h>

namespace spc
{

class CalibrationDataEstimator
{
public:
    SPC_OBJECT(CalibrationDataEstimator)

    enum NORMAL_COMPUTATION_METHOD {
        FULL_NORMALS_ESTIMATION = 0,
        PRECOMPUTED_NORMALS
    };

    enum INTENSITY_ESTIMATION_METHOD {
        SIMPLE_AVERAGE = 0,
        GAUSSIAN_ESTIMATION
    };

    CalibrationDataEstimator();

    void setInputClouds(std::vector<std::string> cloud_names);

    void setInputSamples(std::string core_points_name);

    int compute();

    void getNearestNormal(const pcl::PointXYZI &point, float &nx, float &ny,
                          float &nz, float sq_dist_limit, float &eigenratio);

    void setNormalEstimationMethod(const NORMAL_COMPUTATION_METHOD met)
    {
        normal_estimation_method_ = met;
    }

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
    static float computeGaussianSmoothedIntensity(const pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud,
        const std::vector<int> &indices, const std::vector<float> &sq_distances,
        const float &kernel_sigma, float &intensity_std, const bool exclude_first_id = false);

    static float getAverageIntensity(const pcl::PointCloud
                                     <pcl::PointXYZI> &cloud,
                                     std::vector<int> ids,
                                     float &std);

    // on current cloud!
    void computeSampleParameters(const size_t core_point_id,
                                        const float search_radius);

    static float getMinimumAngleBetweenVectors(const Eigen::Vector3f x_,
                                               const Eigen::Vector3f y_);



    void loadCloudAndCreateSearcher(const std::string fname);

    void loadKeypointsCloud();

    void setInputNormalsCloudName(const std::string fname)
    {
        input_normal_surface_file_ = fname;
    }

    void loadInputNormalsCloud();

    size_t getNumberOfClouds() const
    {
        return input_fnames_.size();
    }

    void setSurfaceForNormalCloud(pcl::PointCloud
                                  <pcl::PointNormal>::Ptr in_surf_normal_cloud)
    {
        surface_for_normal_cloud_ = in_surf_normal_cloud;
    }

    void setMaximumDistanceForGettingNormal(const float &val)
    {
        maximum_squared_distance_for_normal_getting_ = val * val;
    }

    float getEigenRatio(const pcl::PointNormal &point, float &eigen_ratio);


    void setAppendEigenRatio(const bool & val)
    {
        append_eigen_ratio_ = val;
    }

private:
    /////////////////// STRING STUFF //////////////////////////////
    //! list of pcd files to use for calibration
    std::vector<std::string> input_fnames_;

    //! the core points pcd file
    std::string input_core_points_fname_;

    //! this store the filename of the input cloud for normals
    std::string input_normal_surface_file_;

    //! name of the cloud currently under computation
    std::string current_cloud_name_;

    //////////////////// CLOUDS STUFF /////////////////////

    //! here will be store the core points cloud
    pcl::PointCloud<pcl::PointXYZI>::Ptr core_points_cloud_;

    //! these pointers will be dynamically changed during execution
    pcl::PointCloud<pcl::PointXYZI>::Ptr current_point_cloud_;

    //! if you want you can use also precomputed normals. You just need to set
    // an input cloud here
    pcl::PointCloud<pcl::PointNormal>::Ptr surface_for_normal_cloud_;

    ///////////////////// FLANN INDEXES //////////////////////

    //! the current searcher - updated during execution
    pcl::search::FlannSearch<pcl::PointXYZI>::Ptr current_cloud_searcher_;

    //! a searcher for the input cloud with the normals
    pcl::search::FlannSearch
        <pcl::PointNormal>::Ptr surface_for_normal_cloud_searcher_;

    //////////////////// CURRENT SENSOR STUFF /////////////////
    //! current sensor position
    Eigen::Vector4f current_sensor_center_;

    //! current sensor orientation - not really needed here
    Eigen::Quaternionf current_sensor_orientation_;

    ////////////////// CONFIGURATIONS //////////////////////
    NORMAL_COMPUTATION_METHOD normal_estimation_method_;

    INTENSITY_ESTIMATION_METHOD intensity_estimation_method_;

    float normal_estimation_search_radius_;

    /////////////// USED WHEN GAUSSIAN ESTIMATION /////////////
    float intensity_estimation_spatial_sigma_;

    ///////////////// CURRENT CLOUD ID //////////
    size_t current_cloud_id_;

    ////////////// USED ONLY WITH PRECOMPUTED NORMALS ////////////
    float maximum_squared_distance_for_normal_getting_;
    bool append_eigen_ratio_ = true;

    ////////////// OUTPUT DATABASE /////////////////////////
    EigenTable::Ptr db_;

};
}
#endif // INTENSITYAUTOCALIBRATOR_H
