#ifndef SPC_CALIBRATION_DATA_ESTIMATOR_H
#define SPC_CALIBRATION_DATA_ESTIMATOR_H

//#include <vector>
//#include <string>

//#include <spc/elements/generic_cloud.h>

#include <pcl/search/flann_search.h>
#include <pcl/search/impl/flann_search.hpp>
#include <pcl/common/centroid.h>
#include <pcl/common/distances.h>
#include <pcl/console/time.h>

#include <pcl/io/pcd_io.h>
#include <spc/methods/compute_eigen_indices.h>

#include <spc/common/geometry.h>
#include <spc/elements/plane.h>

#include <spc/common/common_includes.h>


#include <boost/any.hpp>
#include <spc/common/std_helpers.hpp>
//#include <boost/spirit/home/support/detail/hold_any.hpp>

#include <spc/calibration/CorePointData.h>
#include <spc/calibration/CalibrationDataDB.h>

namespace spc
{



class CalibrationDataEstimator
{
public:

    enum NORMAL_COMPUTATION_METHOD {FULL_NORMALS_ESTIMATION = 0, PRECOMPUTED_NORMALS};

    enum INTENSITY_ESTIMATION_METHOD {SIMPLE_AVERAGE = 0, GAUSSIAN_ESTIMATION};

    CalibrationDataEstimator();

    void setInputClouds(std::vector<std::string> cloud_names);


    void setInputCorePoints(std::string core_points_name);


    int compute()
    {
        if (normal_estimation_method_ == PRECOMPUTED_NORMALS)
        {
            loadInputNormalsCloud();
        }

        //load the core points cloud
        this->loadCorePointsCloud();

        //set up a db
        db_ = CalibrationDataDB(); //ensure is clean, we should reset instead

        current_cloud_id_ = 0;
        BOOST_FOREACH (std::string fname, input_fnames_)
        {
            current_cloud_name_ = fname;
            pcl::console::print_info("started working on %s ", fname.c_str());

            //load the cloud and create what we need
            this->loadCloudAndCreateSearcher(fname);

            //now we cycle on the core points for the current cloud
            for (size_t i = 0; i < core_points_cloud_->size(); ++i)
            {
                //just some verbosity
                if (i % 100 == 0)
                {
                    pcl::console::print_info("core # %i\n",i);
                }
                CorePointData::Ptr measurement = this->computeCorePointParameters(i, normal_estimation_search_radius_);
                db_.pushBack(measurement);
            }
            current_cloud_id_ += 1;
        }




    }

    void getNearestNormal(const pcl::PointXYZI &point, float &nx, float &ny, float &nz, float sq_dist_limit );

    void setNormalEstimationMethod(const NORMAL_COMPUTATION_METHOD met)
    {
        normal_estimation_method_ = met;
    }

    void setIntensityEstimationMethod(const INTENSITY_ESTIMATION_METHOD met)
    {
        intensity_estimation_method_ = met;
    }

    CalibrationDataDB getCalibrationDB();


    void setSearchRadius(const float rad);

    void setIntensityGaussianSpatialSigma(const float rad)
    {
        intensity_estimation_spatial_sigma_ = rad;
    }

    /** \brief The bilateral filter Gaussian distance kernel.
      * \param[in] sq_dist the spatial distance (distance or intensity)
      * \param[in] sigma standard deviation
      */
    static inline float kernel ( const float &sq_dist, const float &sigma)
    {
        return (exp (- sq_dist/(2*sigma*sigma)));
    }

    //! compute the gaussian smoothed value for a given point
    static float computeGaussianSmoothedIntensity(const pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud,
                                                  const std::vector<int> &indices,
                                                  const std::vector<float> &sq_distances,
                                                  const float &kernel_sigma,
                                                  const bool exclude_first_id = false)
    {
        assert(indices.size() == sq_distances.size());
        assert(kernel_sigma != 0.0f);
        assert(in_cloud);

        if (indices.size() == 1)
            return in_cloud->at(indices.at(0)).intensity;

        if ((indices.size() == 1) & (exclude_first_id))
            return std::numeric_limits<float>::quiet_NaN();

        if (indices.size() == 0)
            return std::numeric_limits<float>::quiet_NaN();

        // compute weights
        std::vector<float> weights (indices.size());
        for (size_t i = 0; i < indices.size(); ++i)
            weights.at(i) = kernel(sq_distances.at(i), kernel_sigma);

        // now compute the weighted average
        float sum = 0;
        float w_sum = 0;

        if (!exclude_first_id)
        {
            for (size_t i = 0; i < indices.size(); ++i)
            {
                sum += weights.at(i) * in_cloud->at(indices.at(i)).intensity;
                w_sum += weights.at(i);
            }
        }
        else
        {
            // now compute the weighted average
            float sum = 0;
            float w_sum = 0;
            for (size_t i = 1; i < indices.size(); ++i)
            {
                sum += weights.at(i) * in_cloud->at(indices.at(i)).intensity;
                w_sum += weights.at(i);
            }
        }


            return sum/w_sum;
        }



        //on current cloud!
        CorePointData::Ptr computeCorePointParameters(const size_t core_point_id,
                                                      const float search_radius );


        static float getMinimumAngleBetweenVectors(const Eigen::Vector3f x_, const Eigen::Vector3f y_);

        static float getAverageIntensity(const pcl::PointCloud<pcl::PointXYZI> & cloud, std::vector<int> ids);

        void loadCloudAndCreateSearcher(const std::string fname);

        void loadCorePointsCloud();

        void setInputNormalsCloudName(const std::string fname)
        {
            input_normal_surface_file_ = fname;
        }

        void loadInputNormalsCloud();

        size_t getNumberOfClouds() const
        {
            return input_fnames_.size();
        }

        void setSurfaceForNormalCloud( pcl::PointCloud<pcl::PointNormal>::Ptr in_surf_normal_cloud )
        {
            surface_for_normal_cloud_ = in_surf_normal_cloud;
        }

        void setMaximumDistanceForGettingNormal(const float &val)
        {
            maximum_squared_distance_for_normal_getting_ = val*val;
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

        //! if you want you can use also precomputed normals. You just need to set an input cloud here
        pcl::PointCloud<pcl::PointNormal>::Ptr surface_for_normal_cloud_;


        ///////////////////// FLANN INDEXES //////////////////////

        //! the current searcher - updated during execution
        pcl::search::FlannSearch<pcl::PointXYZI>::Ptr current_cloud_searcher_;

        //! a searcher for the input cloud with the normals
        pcl::search::FlannSearch<pcl::PointNormal>::Ptr surface_for_normal_cloud_searcher_;


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


        ////////////// OUTPUT DATABASE /////////////////////////
        CalibrationDataDB db_;




    };





}
#endif // INTENSITYAUTOCALIBRATOR_H
