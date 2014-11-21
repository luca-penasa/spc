#include <spc/methods/IntensityCalibrationDataEstimator.h>
#include <pcl/filters/bilateral.h>

#include <pcl/surface/mls.h>

#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/stats.hpp>

#include <boost/accumulators/statistics/mean.hpp>
#include <boost/accumulators/statistics/sum.hpp>
#include <boost/accumulators/statistics/weighted_mean.hpp>
#include <boost/accumulators/statistics/weighted_variance.hpp>
#include <boost/accumulators/statistics/variance.hpp>

#include <boost/foreach.hpp>
#include <boost/range/combine.hpp>

#include <boost/range/combine.hpp>

#include <boost/tuple/tuple.hpp>


#include <spc/elements/calibration/CalibrationKeypoint.h>
namespace spc
{

CalibrationDataEstimator::CalibrationDataEstimator()
    : normal_estimation_search_radius_(0.1),      
      maximum_squared_distance_for_normal_getting_(0.1),
      intensity_estimation_method_(SIMPLE_AVERAGE),
      intensity_estimation_spatial_sigma_(0.1), db_(new EigenTable)
{

    db_->addNewComponent("n_neighbors", 1);
    db_->addNewComponent("normal", 3);
    db_->addNewComponent("lambdas", 3);
    db_->addNewComponent("position", 3);
    db_->addNewComponent("core_id", 1);
    db_->addNewComponent("cloud_id", 1);
    db_->addNewComponent("distance", 1);
    db_->addNewComponent("intensity", 1);
    db_->addNewComponent("angle", 1);
    db_->addNewComponent("intensity_std", 1);
    db_->addNewComponent("eigen_ratio", 1);
}

void CalibrationDataEstimator::setInputClouds(std::vector
                                              <CloudDataSourceOnDisk> cloud_names)
{
    input_clouds_ondisk_ = cloud_names;
}

void CalibrationDataEstimator::setInputKeypoints(PointCloudBase::ConstPtr kpoints)
{
    keypoints_cloud_ = kpoints;
}

int CalibrationDataEstimator::compute()
{
    // init the keypoints
    for (int i = 0 ; i < keypoints_cloud_->getNumberOfPoints(); ++i)
    {
        k = calibration::CalibrationKeyPoint(keypoints_cloud_->getPoint(i));
        keypoints_.push_back(k);
    }

    for (CloudDataSourceOnDisk source: input_clouds_ondisk_)
    {



        if (!source.exists())
        {
            LOG(WARNING) << "the cloud " << source.getFilename() << " does not exists on disk";
            continue;
        }

        // load the cloud
        PointCloudBase::Ptr cloud = spc::io::loadPointCloud(source.getFilename());

        for (calibration::CalibrationKeyPointPtr keypoint: keypoints_)
        {
            data = calibration::PerCloudCalibrationData(source, keypoint);
        }

        //fill calibration data for this keypoint and cloud


    }

//    // load the core points cloud
//    this->loadKeypointsCloud();

//    current_cloud_id_ = 0;
//    for(CloudDataSourceOnDisk cloud_on_disk: input_clouds_ondisk_)
//    {
//        LOG(INFO) << "working on " << cloud.getFilename();

//        // load the cloud and create what we need
//        this->loadCloudAndCreateSearcher(fname);
//        PointCloudBase::Ptr cloud = cloud_on_disk.load();

//#ifdef USE_OPENMP
//#pragma omp parallel for
//#endif
//        // now we cycle on the core points for the current cloud
//        for (size_t i = 0; i < keypoints_(); ++i)
//        {
//            this->computeSampleParameters(i, normal_estimation_search_radius_);
//        }
//        DLOG(INFO) << "cycling on cores. Done";

//        current_cloud_id_ += 1;
//        current_cloud_searcher_.reset();
//        current_point_cloud_.reset();
//    }
}




void CalibrationDataEstimator::setSearchRadius(const float rad)
{
    normal_estimation_search_radius_ = rad;
}

float CalibrationDataEstimator::computeGaussianSmoothedIntensity(const PointCloudBase::ConstPtr in_cloud,
        const std::vector<int> &indices, const std::vector<float> &sq_distances,
        const float &kernel_sigma, float &intensity_std,
        const bool exclude_first_id)
{
//    assert(indices.size() == sq_distances.size());
//    assert(kernel_sigma != 0.0f);
//    assert(in_cloud);

//    if (indices.size() == 1)
//        return in_cloud->at(indices.at(0)).intensity;

//    if ((indices.size() == 1) & (exclude_first_id))
//        return spcNANMacro;

//    if (indices.size() == 0)
//        return spcNANMacro;

//    // extract intensities
//    std::vector<float> intensities(indices.size());
//    std::transform(indices.begin(), indices.end(), intensities.begin(),
//                   [&](int id) { return in_cloud->at(id).intensity; });

//    // compute weights
//    std::vector<float> weights(indices.size());
//    std::transform(
//                sq_distances.begin(), sq_distances.end(), weights.begin(),
//                [&](const float &sq_dist) { return kernel(sq_dist, kernel_sigma); });

//    // now compute stats
//    namespace bacc = boost::accumulators;

//    // weighted mean/var accumulator
//    bacc::accumulator_set
//            <float, bacc::features<bacc::tag::weighted_mean, bacc::tag::weighted_variance>, float>
//            acc;

//    typedef boost::tuple<float &, float &> ref_tuple;
//    for(ref_tuple tup: boost::combine(intensities, weights))
//    {
//        float a = tup.get<0>();
//        float b = tup.get<1>();
//        acc(a, bacc::weight = b);
//    }

//    intensity_std = std::sqrt(bacc::variance(acc));

//    return bacc::mean(acc);
}

void
CalibrationDataEstimator::computeSampleParameters(const size_t core_point_id,
                                                  const float search_radius)
{


//    size_t counter = core_point_id + current_cloud_id_
//            * core_points_cloud_->size();

//    // get the point coordinates from core point cloud
//    pcl::PointXYZI point = core_points_cloud_->at(core_point_id);

//    std::vector<float> dists;
//    std::vector<int> ids;

//    current_cloud_searcher_->radiusSearch(point, search_radius, ids, dists);

//    ////////////////////////////////// NORMAL ESTIMATION
//    ////////////////////////////////



//    // compute normal and average distance
//    float nx(spcNANMacro);
//    float ny(spcNANMacro);
//    float nz(spcNANMacro);
//    float lam0(spcNANMacro);
//    float lam1(spcNANMacro);
//    float lam2(spcNANMacro);
//    float eigen_ratio;

//    if (normal_estimation_method_ == FULL_NORMALS_ESTIMATION) {
//        spc::computePointNormal(*current_point_cloud_, ids, nx, ny, nz, lam0,
//                                lam1, lam2);

//    } else if (normal_estimation_method_ == PRECOMPUTED_NORMALS) {
//        getNearestNormal(point, nx, ny, nz,
//                         maximum_squared_distance_for_normal_getting_, eigen_ratio);


//    }

//    /////////////////////// DISTANCE ESTIMATION ////////////////////////////

//    Eigen::Vector4f c; // tmp_var
//    c.fill(spcNANMacro);

//    if (ids.size() > 0)
//    {
//        pcl::compute3DCentroid(*current_point_cloud_, ids, c);
//    }

//    Eigen::Vector3f pos = current_sensor_center_.head(3);
//    Eigen::Vector3f ray = c.head(3) - pos;

//    float distance = ray.norm();

//    Eigen::Vector3f n_vec;
//    n_vec << nx, ny, nz;

//    Eigen::Vector3f lam_vec;
//    lam_vec << lam0, lam1, lam2;

//    //////////////////////// INTENSITY ESTIMATION ////////////////////////
//    float intensity = spcNANMacro;
//    float intensity_std = spcNANMacro;


//    if (intensity_estimation_method_ == SIMPLE_AVERAGE)
//        intensity = this->getAverageIntensity(*current_point_cloud_, ids,
//                                              intensity_std);

//    else if (intensity_estimation_method_ == GAUSSIAN_ESTIMATION)
//    {


//        float full_radius = intensity_estimation_spatial_sigma_ * 4;

//        std::vector<float> dists_int;
//        std::vector<int> ids_int;

//        current_cloud_searcher_->radiusSearch(point, full_radius, ids_int,
//                                              dists_int);


//        intensity = CalibrationDataEstimator::computeGaussianSmoothedIntensity(
//                    current_point_cloud_, ids_int, dists_int,
//                    intensity_estimation_spatial_sigma_, intensity_std);

//    }

//    float angle
//            = CalibrationDataEstimator::getMinimumAngleBetweenVectors(n_vec, ray);

//    db_->atScalar("n_neighbors", counter) = ids.size();
//    db_->atVector("normal", counter) = n_vec;
//    db_->atVector("lambdas", counter) = lam_vec;
//    db_->atVector("position", counter) = c.head(3);
//    db_->atScalar("core_id", counter) = core_point_id;
//    db_->atScalar("cloud_id", counter) = current_cloud_id_;
//    db_->atScalar("distance", counter) = distance;
//    db_->atScalar("intensity", counter) = intensity;
//    db_->atScalar("angle", counter) = angle;
//    db_->atScalar("intensity_std", counter) = intensity_std;
//    db_->atScalar("eigen_ratio", counter) = eigen_ratio;
}

float CalibrationDataEstimator::getMinimumAngleBetweenVectors(const Eigen::Vector3f x_,
                                                              const Eigen::Vector3f y_)
{
    // ensure they are normalized
    Eigen::Vector3f x = x_ / x_.norm();
    Eigen::Vector3f y = y_ / y_.norm();

    float cosTheta = x.dot(y);
    float theta = acos(std::min(fabs(cosTheta), 1.0));

    theta *= 180.0 / M_PI;

    return theta;
}

float CalibrationDataEstimator::getAverageIntensity(const PointCloudBase::ConstPtr
                                                   cloud,
                                                    std::vector<int> ids,
                                                    float &std)
{
//    if (ids.size() == 0)
//        return spcNANMacro;

//    namespace bacc = boost::accumulators;

//    std::vector<float> intensities(ids.size());
//    std::transform(ids.begin(), ids.end(), intensities.begin(),
//                   [&cloud](int id) { return cloud.at(id).intensity; });

//    bacc::accumulator_set
//            <float, bacc::features<bacc::tag::mean, bacc::tag::variance>> acc;
//    std::for_each(intensities.begin(), intensities.end(),
//                  boost::bind<void>(boost::ref(acc), _1));

//    std = std::sqrt(bacc::variance(acc));
//    return bacc::mean(acc);
}






EigenTable::Ptr CalibrationDataEstimator::getCalibrationDB()
{
    return db_;
}

} // end nspace
