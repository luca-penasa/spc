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
      intensity_estimation_spatial_sigma_(0.1),
      kernel_(new GaussianRBF<float>(0.1)),
      calibration_data_(new calibration::CalibrationDataHolder)

{


}

void CalibrationDataEstimator::setInputClouds(std::vector<CloudDataSourceOnDisk::Ptr> cloud_names)
{
    input_clouds_ondisk_ = cloud_names;
}

void CalibrationDataEstimator::setInputKeypoints(NewSpcPointCloud::ConstPtr kpoints)
{
    calibration_data_->initFromCloud(kpoints);
}

int CalibrationDataEstimator::compute()
{
    for (CloudDataSourceOnDisk::Ptr source: input_clouds_ondisk_)
    {
        // load the cloud
        NewSpcPointCloud::Ptr cloud = source->load2();

        if (cloud == NULL)
        {
            LOG(ERROR) << "some error loading the cloud. see log please";
            continue;
        }

        if (calibration_data_->getData().empty())
        {
            LOG(ERROR) << "no keypoints to be processed";
            return -1;
        }

        for (calibration::CalibrationKeyPoint::Ptr keypoint: calibration_data_->getData())
        {
            calibration::PerCloudCalibrationData::Ptr data = keypoint->newPerCloudData(source);

            extractDataForKeypointAndCloud(data, cloud);
        }
    }

    computeDerivedData();

    return 1;

}




void CalibrationDataEstimator::setNormalEstimationSearchRadius(const float rad)
{
    normal_estimation_search_radius_ = rad;
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

void CalibrationDataEstimator::extractDataForKeypointAndCloud(calibration::PerCloudCalibrationData::Ptr data_holder, NewSpcPointCloud::Ptr cloud)

{
    //! extract and save the sensor position
    data_holder->sensor_position = cloud->getSensor()->getPosition().head(3);

    LOG(INFO) <<"senso position: " << data_holder->sensor_position;

    NewSpcPointCloud::SearcherT::Ptr searcher = cloud->getSearcher();

    LOG(INFO) << "going to do matches" << std::endl;
    std::vector<std::pair<size_t, float>> matches;

    // these points will be used for normal estimation
    LOG(INFO) << "point " << data_holder->parent_keypoint->original_position;
    searcher->radiusSearch(data_holder->parent_keypoint->original_position, normal_estimation_search_radius_, matches);

    LOG(INFO) << "number of matches " << matches.size();

    std::vector<size_t> ids;
    Eigen::VectorXf sq_distances;

    decompressMatches(matches, ids, sq_distances);

    NewSpcPointCloud extract = cloud->fromIds(ids, {"position"}); // extract only the position

    LOG(INFO) << "new cloud has "<< extract.getNumberOfPoints() << " points";

    // cumulate the cloud. Normal estimation will be performed only at the end
    data_holder->extract_for_normal_ = extract;

    // NOW INTENSITY STUFF
    // now repeat the search for the intensity estimation

    std::vector<std::pair<size_t, float>> matches_int;
    searcher->radiusSearch(data_holder->parent_keypoint->original_position, intensity_estimation_spatial_sigma_ * 4, matches_int);


    LOG(INFO) << "found matches:" << matches_int.size();
    std::vector<size_t> ids_int;
    Eigen::VectorXf sq_distances_int;

    decompressMatches(matches_int, ids_int, sq_distances_int);

    LOG(INFO) << "extractin fields " ;


    NewSpcPointCloud intensities = cloud->fromIds(ids_int, {intensity_field_name_});

    LOG(INFO) << "int cloud is of size " << intensities.getNumberOfPoints();
    Eigen::VectorXf ints = intensities.getFieldByName(intensity_field_name_);

    LOG(INFO) << "ints computed are " << ints.rows();

    LOG(INFO) << "doing weighting " ;

    Eigen::VectorXf weights = kernel_->eval(sq_distances_int);

    LOG(INFO) << "weights computed are " << weights.rows();


    float avg_intensity = ints.cwiseProduct(weights).sum() / weights.sum();
    Eigen::VectorXf diff = ints.array() - avg_intensity;
    diff = diff.cwiseProduct(diff); // squared
    float std_intensity = sqrt(diff.cwiseProduct(weights).sum() / weights.sum()); // standard deviation

    data_holder->intensity = avg_intensity;
    data_holder->intensity_std = std_intensity;
    data_holder->n_neighbors_intensity = ints.rows();

    LOG(INFO) << "avg is " << avg_intensity;
    LOG(INFO) << "std is " << std_intensity;
}

void CalibrationDataEstimator::computeDerivedData()
{

    for (calibration::CalibrationKeyPoint::Ptr keypoint: calibration_data_->getData())
    {
        for (calibration::PerCloudCalibrationData::Ptr data_holder: keypoint->per_cloud_data)
        {
            LOG(INFO) << "extrat for normas has size " << data_holder->extract_for_normal_.getNumberOfPoints();
            keypoint->cumulative_set.concatenate(data_holder->extract_for_normal_);
        }

        LOG(INFO) << "cumulative set has " << keypoint->cumulative_set.getNumberOfPoints();

        if (keypoint->cumulative_set.getNumberOfPoints() >= min_number_of_points_for_normal_estimation_)
        {

            Eigen::Vector3f lambdas;
            NewSpcPointCloud::EigenPlaneT plane = keypoint->cumulative_set.fitPlane(lambdas);

            keypoint->fitting_plane = Plane::fromEigenHyperplane3f(plane);
            keypoint->lambdas = lambdas;
            keypoint->eigen_ratio = lambdas(0) / lambdas.sum();

            LOG(INFO) << "found normal: " << keypoint->fitting_plane.getNormal().transpose() << "with lambdas: " << lambdas.transpose();


            Eigen::Vector3f centroid = keypoint->cumulative_set.getCentroid();
            //! project the centroid onto the fitting plane to get a better estimate of the position
            Eigen::Vector3f newpos = keypoint->fitting_plane.projectPointOnPlane(centroid);
            LOG(INFO) << "new pos is " << newpos.transpose() << " original was " <<keypoint->original_position.transpose();
            LOG(INFO) << "centroid was" << centroid;
            keypoint->post_position = newpos;

            //! no we can compute distance and angle
            for (calibration::PerCloudCalibrationData::Ptr per_cloud: keypoint->per_cloud_data)
            {
                Eigen::Vector3f ray = keypoint->post_position - per_cloud->sensor_position;

                per_cloud->angle = getMinimumAngleBetweenVectors(keypoint->fitting_plane.getNormal(), ray);
                per_cloud->distance = sqrt(ray.dot(ray));

                LOG(INFO) << "angle " << per_cloud->angle << " and distance " << per_cloud->distance;

            }
        }
    }
}









} // end nspace
