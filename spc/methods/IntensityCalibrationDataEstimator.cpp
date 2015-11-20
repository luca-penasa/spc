#include <spc/methods/IntensityCalibrationDataEstimator.h>
//#include <pcl/filters/bilateral.h>

//#include <pcl/surface/mls.h>

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

#include <spc/elements/calibration/KeyPoint.h>

namespace spc {
namespace calibration {
CalibrationDataEstimator::CalibrationDataEstimator()
    : intensity_estimation_spatial_sigma_(0.1)
    , kernel_(new GaussianRBF<float>(0.1))
    , calibration_data_(new calibration::DataHolder)

{
}

void CalibrationDataEstimator::setInputClouds(std::vector<CloudDataSourceOnDisk::Ptr> cloud_names)
{
    input_clouds_ondisk_ = cloud_names;
}

void CalibrationDataEstimator::setInputKeypoints(const NewSpcPointCloud::Ptr kpoints)
{
    calibration_data_->initFromCloud(kpoints, material_field_name_);
}

int CalibrationDataEstimator::compute()
{

    LOG(INFO) << "Using as search pars for intensity "  << this->getIntensitySearchParameters();
    LOG(INFO) << "Using as search pars for normal estiamtion "  << this->getNormalSearchParameters();
    LOG(INFO) << "Gaussian weighting is "  << gaussian_weighting_;

    for (CloudDataSourceOnDisk::Ptr source : input_clouds_ondisk_)
    {
        LOG(INFO) << "started loading the cloud";
        NewSpcPointCloud::Ptr cloud = source->load2();
        LOG(INFO) << "Done";

        cloud_to_sensor_position_[source] = cloud->getSensor()->getPosition().head(3);

        LOG(INFO) << "cloud loaded: " << source->getFilename();

        if (cloud == NULL) {
            LOG(ERROR) << "some error loading the cloud. see log please";
            continue;
        }

        if (calibration_data_->getData().empty()) {
            LOG(ERROR) << "no keypoints to be processed";
            return -1;
        }

        if (!cloud->hasField(intensity_field_name_))
        {
            LOG(ERROR) << "No field named " << intensity_field_name_  << " in the cloud. Check your settings!";
            return -1;
        }

        LOG(INFO) << "Working on cloud " << source->getFilename();

#ifdef USE_OPENMP
#pragma omp parallel for shared(cloud)
#endif
        for (int i = 0; i < calibration_data_->getData().size(); ++i) {
            calibration::KeyPoint::Ptr keypoint = calibration_data_->getData().at(i);

            // create a new observation
            Observation::Ptr cdata (new Observation(source, keypoint));

            extractDataForKeypointAndCloud(cdata, cloud);

            if (cdata->hasValidIntensity()) // do not push back observations with invalid intensities
                keypoint->observations.push_back(cdata);
        }

//        LOG(INFO) << "cloud referenced by a number: " << cloud.use_count();

//        cloud.reset();
    }

    computeDerivedData();

    DLOG(INFO) << "done computing derived data";

    return 1;
}

float CalibrationDataEstimator::getMinimumAngleBetweenVectors(const Eigen::Vector3f x_,
                                                              const Eigen::Vector3f y_)
{
    // ensure they are normalized
    Eigen::Vector3f x = x_ / x_.norm();
    Eigen::Vector3f y = y_ / y_.norm();

    float cosTheta = x.dot(y);
    float theta = acos(std::min(std::abs(cosTheta), 1.0f));

    theta *= 180.0 / M_PI;

    return theta;
}

void CalibrationDataEstimator::extractDataForKeypointAndCloud(calibration::Observation::Ptr data_holder,
                                                              NewSpcPointCloud::Ptr cloud)

{
//    //! extract and store the sensor position
//    data_holder->sensor_position = cloud->getSensor()->getPosition().head(3);



//    DLOG(INFO) << "sensor position found: " << data_holder->sensor_position;

    NewSpcPointCloud::SearcherT::Ptr searcher = cloud->getSearcher();

    std::vector<size_t> ids;
    std::vector<float> sq_distances;

    /////// do first search for the normal estimation!

    if (normal_search_pars_.isKNN())
    {
        DLOG(INFO) << "going to do neighbors search" << std::endl;
        bool status = searcher->knnSearch(data_holder->parent_keypoint->original_position, normal_search_pars_.getNumberOfNeighbors(), ids, sq_distances);
        DCHECK(status) << "Status false for current neighbors search";

        std::vector<float> filtered_sq_distances;
        std::vector<size_t> filtered_ids;

        for (size_t i = 0; i < ids.size(); ++i)
        {
            if (sq_distances.at(i) < max_admissible_knn_distance_*max_admissible_knn_distance_) // push back
            {
                   filtered_sq_distances.push_back(sq_distances.at(i));
                   filtered_ids.push_back(ids.at(i));
            }
        }

        ids = filtered_ids;
        sq_distances = filtered_sq_distances;

    }
    else // do radius search
    {
        DLOG(INFO) << "going to do search" << std::endl;
        searcher->radiusSearch(data_holder->parent_keypoint->original_position, normal_search_pars_.getSquaredSearchRadius(), ids, sq_distances);
        DLOG(INFO) << "number of neighbors for normal " << ids.size();
//        LOG(INFO) << "found " << ids.size() << " neighbors";
    }

    if (ids.size() > 0) // extract the corresponding points only if we have neighbors
    {
        NewSpcPointCloud extract = cloud->fromIds(ids, { "position" }); // extract only the position

        // cumulate the cloud. Normal estimation will be performed only at the end
        data_holder->extract_for_normal_ = extract;
    }

    /////////////// SECOND SEARCH FOR INTENSITY, but only if the search parameters are different
    std::vector<size_t> ids_int;
    std::vector<float> dists_sq_int;

    if (normal_search_pars_ != intensity_search_pars_) // in this case redo the search
    {

        DLOG(INFO) << "search parameters were different, redoing the search";

        if (normal_search_pars_.isKNN())
        {
            DLOG(INFO) << "going to do neighbors search" << std::endl;
            bool status = searcher->knnSearch(data_holder->parent_keypoint->original_position, normal_search_pars_.getNumberOfNeighbors(), ids_int, dists_sq_int);
            DCHECK(status) << "Status false for current neighbors search";

            std::vector<float> filtered_sq_distances;
            std::vector<size_t> filtered_ids;

            for (size_t i = 0; i < ids_int.size(); ++i)
            {
                if (dists_sq_int.at(i) < max_admissible_knn_distance_*max_admissible_knn_distance_) // push back
                {
                       filtered_sq_distances.push_back(dists_sq_int.at(i));
                       filtered_ids.push_back(ids_int.at(i));
                }
            }

            ids_int = filtered_ids;
            dists_sq_int = filtered_sq_distances;
        }
        else // do radius search
        {
            DLOG(INFO) << "going to do search" << std::endl;
            searcher->radiusSearch(data_holder->parent_keypoint->original_position, normal_search_pars_.getNumberOfNeighbors(), ids_int, dists_sq_int);
            DLOG(INFO) << "number of neighbors for normal " << ids.size();
        }
    }

    else // just do a copy of the previous found values
    {
        ids_int = ids;
        dists_sq_int = sq_distances;
    }

    if (ids_int.size() < min_number_of_points_intensity_) // skip, nothing to do here
    {
        return;
    }

    ///// extract intensities and compute average and standard deviation
    NewSpcPointCloud intensities = cloud->fromIds(ids_int, { intensity_field_name_ });

    Eigen::VectorXf ints = intensities.getFieldByName(intensity_field_name_);

    float avg_intensity, std_intensity;

    if (gaussian_weighting_ == true) // d gaussian weighting
    {
        Eigen::Map<Eigen::VectorXf> asmap(dists_sq_int.data(), dists_sq_int.size(), 1);
        Eigen::VectorXf weights = kernel_->eval(asmap);


        avg_intensity = ints.cwiseProduct(weights).sum() / weights.sum();
        Eigen::VectorXf diff = ints.array() - avg_intensity;
        diff = diff.cwiseProduct(diff); // squared
        std_intensity = sqrt(diff.cwiseProduct(weights).sum() / weights.sum()); // standard deviation
    }
    else // pure average intensity
    {
        avg_intensity = ints.sum() / ints.rows() - 1;
        Eigen::VectorXf diff = ints.array() - avg_intensity;
        diff = diff.cwiseProduct(diff); // squared
        std_intensity  = sqrt(diff.sum() / diff.rows() - 1 );
    }


    // add these infos in the observation!
    data_holder->intensity = avg_intensity;
    data_holder->intensity_std = std_intensity;
    data_holder->n_neighbors_intensity = ints.rows();

    DLOG(INFO) << "done extraction data for given keypoint";
}

void CalibrationDataEstimator::computeDerivedData()
{

    LOG(INFO) << "Computing derived fields";
#ifdef USE_OPENMP
#pragma omp parallel for
#endif
    for (int i = 0; i < calibration_data_->getData().size(); ++i) {

        calibration::KeyPoint::Ptr keypoint = calibration_data_->getData().at(i);
        for (calibration::Observation::Ptr data_holder : keypoint->observations)
        {
            DLOG(INFO) << "extract for normas has size " << data_holder->extract_for_normal_.getNumberOfPoints();
            keypoint->cumulative_set.concatenate(data_holder->extract_for_normal_);
        }

        if (keypoint->cumulative_set.getNumberOfPoints() >= min_number_of_points_normal_)
        {
            Eigen::Vector3f lambdas;
            NewSpcPointCloud::EigenPlaneT plane = keypoint->cumulative_set.fitPlane(lambdas);

            float lambdas_sum = lambdas.sum();

            keypoint->fitting_plane = Plane::fromEigenHyperplane3f(plane);
            keypoint->lambdas = lambdas;
            keypoint->eigen_ratio = lambdas(0) / lambdas_sum;
            keypoint->s1 = (-4 * lambdas(2) - 2 * lambdas(1)) / lambdas_sum + 3;
            keypoint->s2 = (2 * lambdas(2) + 4 * lambdas(1)) / lambdas_sum - 2;

            Eigen::Vector3f centroid = keypoint->cumulative_set.getCentroid();
            //! project the centroid onto the fitting plane to get a better estimate of the position
            Eigen::Vector3f newpos = keypoint->fitting_plane.projectPointOnPlane(centroid);
//            LOG(INFO) << "new pos is " << newpos.transpose() << " original was " <<keypoint->original_position.transpose() << " c: " << centroid.transpose();
            keypoint->post_position = newpos;

            keypoint->center_to_new_center = (newpos - keypoint->original_position).norm();



            //! no we can compute distance and angle
            for (calibration::Observation::Ptr per_cloud : keypoint->observations) {
                Eigen::Vector3f ray = keypoint->post_position - cloud_to_sensor_position_.at(per_cloud->getCloud());

                per_cloud->angle = getMinimumAngleBetweenVectors(keypoint->fitting_plane.getNormal(), ray);
                per_cloud->distance = sqrt(ray.dot(ray));

            }
        }
    }
}
}

} // end nspace
