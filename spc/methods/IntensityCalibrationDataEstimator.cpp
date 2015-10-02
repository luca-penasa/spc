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

#include <spc/elements/calibration/KeyPoint.h>
namespace spc {
namespace calibration {
	CalibrationDataEstimator::CalibrationDataEstimator()
		: normal_estimation_search_radius_(0.1)
		, intensity_estimation_spatial_sigma_(0.1)
		, kernel_(new GaussianRBF<float>(0.1))
		, calibration_data_(new calibration::DataHolder)

	{
	}

	void CalibrationDataEstimator::setInputClouds(std::vector<CloudDataSourceOnDisk::Ptr> cloud_names)
	{
		input_clouds_ondisk_ = cloud_names;
	}

	void CalibrationDataEstimator::setInputKeypoints(NewSpcPointCloud::ConstPtr kpoints)
    {
		calibration_data_->initFromCloud(kpoints, material_field_name_);
	}

	int CalibrationDataEstimator::compute()
	{
		for (CloudDataSourceOnDisk::Ptr source : input_clouds_ondisk_) {
			// load the cloud

			NewSpcPointCloud::Ptr cloud = source->load2();

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

//				calibration::Observation::Ptr data = keypoint->newObservationOnCloud(source);
				extractDataForKeypointAndCloud(cdata, cloud);

				// if it has valid intensity data we keep it else it is not useful at all..
				if (cdata->hasValidIntensity())
				{
//					LOG(INFO) << "pushed";
					keypoint->observations.push_back(cdata);
				}

			}

			LOG(INFO) << "cloud referenced by a number: " << cloud.use_count();

			cloud.reset();

		}

		computeDerivedData();

		DLOG(INFO) << "done computing derived data";

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
		float theta = acos(std::min(std::abs(cosTheta), 1.0f));

		theta *= 180.0 / M_PI;

		return theta;
	}

	void CalibrationDataEstimator::extractDataForKeypointAndCloud(calibration::Observation::Ptr data_holder, NewSpcPointCloud::Ptr cloud)

	{
		//! extract and save the sensor position
		data_holder->sensor_position = cloud->getSensor()->getPosition().head(3);

		DLOG(INFO) << "sensor position: " << data_holder->sensor_position;

		NewSpcPointCloud::SearcherT::Ptr searcher = cloud->getSearcher();

		DLOG(INFO) << "going to do matches" << std::endl;
		std::vector<std::pair<size_t, float> > matches;

		// these points will be used for normal estimation
		searcher->radiusSearch(data_holder->parent_keypoint->original_position, normal_estimation_search_radius_, matches);

		DLOG(INFO) << "number of matches for normal " << matches.size();

		std::vector<size_t> ids;
		Eigen::VectorXf sq_distances;

		decompressMatches(matches, ids, sq_distances);

		NewSpcPointCloud extract = cloud->fromIds(ids, { "position" }); // extract only the position

		// cumulate the cloud. Normal estimation will be performed only at the end
		data_holder->extract_for_normal_ = extract;

		std::vector<std::pair<size_t, float> > matches_int;
		searcher->radiusSearch(data_holder->parent_keypoint->original_position, intensity_estimation_spatial_sigma_ * 3, matches_int);

		DLOG(INFO) << "number of matches for intensity " << matches_int.size();


		if (matches_int.size() >= min_number_of_points_for_intensity_estimation_) {

			DLOG(INFO) << "NN are enough";

			//    DLOG(INFO) << "found matches for intensity estimation:" << matches_int.size();
			std::vector<size_t> ids_int;
			Eigen::VectorXf sq_distances_int;

			decompressMatches(matches_int, ids_int, sq_distances_int);

			NewSpcPointCloud intensities = cloud->fromIds(ids_int, { intensity_field_name_ });

			Eigen::VectorXf ints = intensities.getFieldByName(intensity_field_name_);

			Eigen::VectorXf weights = kernel_->eval(sq_distances_int);

			float avg_intensity = ints.cwiseProduct(weights).sum() / weights.sum();
			Eigen::VectorXf diff = ints.array() - avg_intensity;
			diff = diff.cwiseProduct(diff); // squared
			float std_intensity = sqrt(diff.cwiseProduct(weights).sum() / weights.sum()); // standard deviation

			data_holder->intensity = avg_intensity;
			data_holder->intensity_std = std_intensity;
			data_holder->n_neighbors_intensity = ints.rows();
		}



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
			for (calibration::Observation::Ptr data_holder : keypoint->observations) {
				//            DLOG(INFO) << "extract for normas has size " << data_holder->extract_for_normal_.getNumberOfPoints();
				keypoint->cumulative_set.concatenate(data_holder->extract_for_normal_);
			}

			if (keypoint->cumulative_set.getNumberOfPoints() >= min_number_of_points_for_normal_estimation_) {

				Eigen::Vector3f lambdas;
				NewSpcPointCloud::EigenPlaneT plane = keypoint->cumulative_set.fitPlane(lambdas);

				float lambdas_sum = lambdas.sum();

				keypoint->fitting_plane = Plane::fromEigenHyperplane3f(plane);
				keypoint->lambdas = lambdas;
				keypoint->eigen_ratio = lambdas(0) / lambdas_sum;

				//						DLOG(INFO) << "found normal: " << keypoint->fitting_plane.getNormal().transpose() << " with lambdas: " << lambdas.transpose();

				// computing s1 and s2 indexes
				// lambdas are in increasing order as thy come back from eigen
				keypoint->s1 = (-4 * lambdas(2) - 2 * lambdas(1)) / lambdas_sum + 3;
				keypoint->s2 = (2 * lambdas(2) + 4 * lambdas(1)) / lambdas_sum - 2;

				Eigen::Vector3f centroid = keypoint->cumulative_set.getCentroid();
				//! project the centroid onto the fitting plane to get a better estimate of the position
				Eigen::Vector3f newpos = keypoint->fitting_plane.projectPointOnPlane(centroid);
				//						LOG(INFO) << "new pos is " << newpos.transpose() << " original was " <<keypoint->original_position.transpose() << " c: " << centroid.transpose();
				keypoint->post_position = newpos;

				keypoint->center_to_new_center = (newpos - keypoint->original_position).norm();

				//! no we can compute distance and angle
				for (calibration::Observation::Ptr per_cloud : keypoint->observations) {
					Eigen::Vector3f ray = keypoint->post_position - per_cloud->sensor_position;

					per_cloud->angle = getMinimumAngleBetweenVectors(keypoint->fitting_plane.getNormal(), ray);
					per_cloud->distance = sqrt(ray.dot(ray));

					//                LOG(INFO) << "angle " << per_cloud->angle << " and distance " << per_cloud->distance;
				}
            }
        }

		//    LOG(INFO) << "Derived data computed!";
	}
}

} // end nspace
