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
namespace spc
{

CalibrationDataEstimator::CalibrationDataEstimator()
    : normal_estimation_search_radius_(0.1),
      normal_estimation_method_(FULL_NORMALS_ESTIMATION),
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
                                              <std::string> cloud_names)
{
    input_fnames_ = cloud_names;
}

void CalibrationDataEstimator::setInputSamples(std::string core_points_name)
{
    input_core_points_fname_ = core_points_name;
}

int CalibrationDataEstimator::compute()
{

    if (normal_estimation_method_ == PRECOMPUTED_NORMALS) {
        loadInputNormalsCloud();
    }

    // load the core points cloud
    this->loadKeypointsCloud();

    current_cloud_id_ = 0;
    for(std::string fname: input_fnames_)
    {

        current_point_cloud_.reset();
        current_cloud_searcher_.reset();

        current_cloud_name_ = fname;
        pcl::console::print_info("started working on %s ", fname.c_str());

        // load the cloud and create what we need
        this->loadCloudAndCreateSearcher(fname);

        // now we cycle on the core points for the current cloud
        for (size_t i = 0; i < core_points_cloud_->size(); ++i) {
            // just some verbosity
            if (i % 100 == 0) {
                pcl::console::print_info("core # %i\n", i);
            }
            this->computeSampleParameters(i, normal_estimation_search_radius_);
        }
        current_cloud_id_ += 1;

        current_cloud_searcher_.reset();
        current_point_cloud_.reset();
    }
}

void CalibrationDataEstimator::getEigenRatio(const pcl::PointNormal &point, float &eigen_ratio)
{
    eigen_ratio = point.curvature;
}

void CalibrationDataEstimator::getNearestNormal(const pcl::PointXYZI &point,
                                                float &nx, float &ny, float &nz,
                                                float sq_dist_limit, float &eigenratio)
{
    pcl::PointNormal p_no;
    p_no.x = point.x;
    p_no.y = point.y;
    p_no.z = point.z;
    std::vector<int> ids;
    std::vector<float> sq_dists;
    surface_for_normal_cloud_searcher_->nearestKSearch(p_no, 1, ids, sq_dists);

    if (ids.empty()) {
        nx = spcNANMacro;
        ny = spcNANMacro;
        nz = spcNANMacro;
        return;
    }

    if ((sq_dists.at(0) >= sq_dist_limit)) {
        nx = spcNANMacro;
        ny = spcNANMacro;
        nz = spcNANMacro;
        return;
    }

    // get the normal for that point
    p_no = surface_for_normal_cloud_->at(ids.at(0));
    nx = p_no.normal_x;
    ny = p_no.normal_y;
    nz = p_no.normal_z;

    if (append_eigen_ratio_)
        eigenratio = p_no.curvature;
    else
        eigenratio = spcNANMacro;

    return;
}

void CalibrationDataEstimator::setSearchRadius(const float rad)
{
    normal_estimation_search_radius_ = rad;
}

float CalibrationDataEstimator::computeGaussianSmoothedIntensity(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud,
    const std::vector<int> &indices, const std::vector<float> &sq_distances,
    const float &kernel_sigma, float &intensity_std,
    const bool exclude_first_id)
{
    assert(indices.size() == sq_distances.size());
    assert(kernel_sigma != 0.0f);
    assert(in_cloud);

    if (indices.size() == 1)
        return in_cloud->at(indices.at(0)).intensity;

    if ((indices.size() == 1) & (exclude_first_id))
        return spcNANMacro;

    if (indices.size() == 0)
        return spcNANMacro;

    // extract intensities
    std::vector<float> intensities(indices.size());
    std::transform(indices.begin(), indices.end(), intensities.begin(),
                   [&](int id) { return in_cloud->at(id).intensity; });

    // compute weights
    std::vector<float> weights(indices.size());
    std::transform(
        sq_distances.begin(), sq_distances.end(), weights.begin(),
        [&](const float &sq_dist) { return kernel(sq_dist, kernel_sigma); });

    // now compute stats
    namespace bacc = boost::accumulators;

    // weighted mean/var accumulator
    bacc::accumulator_set
        <float, bacc::features<bacc::tag::weighted_mean, bacc::tag::weighted_variance>, float>
    acc;

    typedef boost::tuple<float &, float &> ref_tuple;
    for(ref_tuple tup: boost::combine(intensities, weights))
    {
        float a = tup.get<0>();
        float b = tup.get<1>();
        acc(a, bacc::weight = b);
    }

    intensity_std = std::sqrt(bacc::variance(acc));

    return bacc::mean(acc);
}

void
CalibrationDataEstimator::computeSampleParameters(const size_t core_point_id,
                                                  const float search_radius)
{

    size_t counter = core_point_id + current_cloud_id_
                                     * core_points_cloud_->size();

    // get the point coordinates from core point cloud
    pcl::PointXYZI point = core_points_cloud_->at(core_point_id);

    std::vector<float> dists;
    std::vector<int> ids;

    current_cloud_searcher_->radiusSearch(point, search_radius, ids, dists);

    ////////////////////////////////// NORMAL ESTIMATION
    ////////////////////////////////
    // compute normal and average distance
    float nx(spcNANMacro);
    float ny(spcNANMacro);
    float nz(spcNANMacro);
    float lam0(spcNANMacro);
    float lam1(spcNANMacro);
    float lam2(spcNANMacro);
    float eigen_ratio;

    if (normal_estimation_method_ == FULL_NORMALS_ESTIMATION) {
        spc::computePointNormal(*current_point_cloud_, ids, nx, ny, nz, lam0,
                                lam1, lam2);

    } else if (normal_estimation_method_ == PRECOMPUTED_NORMALS) {
        getNearestNormal(point, nx, ny, nz,
                         maximum_squared_distance_for_normal_getting_, eigen_ratio);


    }

    /////////////////////// DISTANCE ESTIMATION ////////////////////////////
    Eigen::Vector4f _c(spcNANMacro, spcNANMacro, spcNANMacro,
                       spcNANMacro); // tmp_var

    if (ids.size() > 0) {
        pcl::compute3DCentroid(*current_point_cloud_, ids, _c);
    }

    Eigen::Vector3f c(_c(0), _c(1), _c(2));
    Eigen::Vector3f pos
        = Eigen::Vector3f(current_sensor_center_(0), current_sensor_center_(1),
                          current_sensor_center_(2));
    Eigen::Vector3f ray = c - pos;

    float distance = ray.norm();

    auto n_vec = Eigen::Vector3f(nx, ny, nz);
    //    auto lambdas = Eigen::Vector3f(lam0, lam1, lam2);

    //////////////////////// INTENSITY ESTIMATION ////////////////////////
    float intensity = spcNANMacro;
    float intensity_std = spcNANMacro;

    if (intensity_estimation_method_ == SIMPLE_AVERAGE)
        intensity = this->getAverageIntensity(*current_point_cloud_, ids,
                                              intensity_std);

    else if (intensity_estimation_method_ == GAUSSIAN_ESTIMATION) {
        // we redo a neighbors search

        float full_radius = intensity_estimation_spatial_sigma_ * 4;

        std::vector<float> dists_int;
        std::vector<int> ids_int;

        current_cloud_searcher_->radiusSearch(point, full_radius, ids_int,
                                              dists_int);

        intensity = CalibrationDataEstimator::computeGaussianSmoothedIntensity(
            current_point_cloud_, ids_int, dists_int,
            intensity_estimation_spatial_sigma_, intensity_std);
    }

    float angle
        = CalibrationDataEstimator::getMinimumAngleBetweenVectors(n_vec, ray);




    db_->atScalar("n_neighbors", counter) = ids.size();
    db_->atVector("normal", counter) = Eigen::Vector3f(nx, ny, nz);
    db_->atVector("lambdas", counter) = Eigen::Vector3f(lam0, lam1, lam2);
    db_->atVector("position", counter) = c;
    db_->atScalar("core_id", counter) = core_point_id;
    db_->atScalar("cloud_id", counter) = current_cloud_id_;
    db_->atScalar("distance", counter) = distance;
    db_->atScalar("intensity", counter) = intensity;
    db_->atScalar("angle", counter) = angle;
    db_->atScalar("intensity_std", counter) = intensity_std;
    db_->atScalar("eigen_ratio", counter) = eigen_ratio;

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

float CalibrationDataEstimator::getAverageIntensity(const pcl::PointCloud
                                                    <pcl::PointXYZI> &cloud,
                                                    std::vector<int> ids,
                                                    float &std)
{
    if (ids.size() == 0)
        return spcNANMacro;

    namespace bacc = boost::accumulators;

    std::vector<float> intensities(ids.size());
    std::transform(ids.begin(), ids.end(), intensities.begin(),
                   [&cloud](int id) { return cloud.at(id).intensity; });

    bacc::accumulator_set
        <float, bacc::features<bacc::tag::mean, bacc::tag::variance>> acc;
    std::for_each(intensities.begin(), intensities.end(),
                  boost::bind<void>(boost::ref(acc), _1));

    std = std::sqrt(bacc::variance(acc));
    return bacc::mean(acc);
}

void
CalibrationDataEstimator::loadCloudAndCreateSearcher(const std::string fname)
{

    pcl::console::print_info("loading cloud\n");

    pcl::PCLPointCloud2::Ptr in_cloud(new pcl::PCLPointCloud2);

    Eigen::Vector4f center;
    Eigen::Quaternionf orientation;

    // LOAD
    pcl::io::loadPCDFile(fname, *in_cloud, center, orientation);

    // convert to a PointCloud type object
    pcl::PointCloud
        <pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromPCLPointCloud2(*in_cloud, *cloud);

    pcl::console::print_info("cloud loaded\n");
    // TODO check that the intensity fields exists, please

    pcl::console::print_info("creating flann index\n");
    // now create the searcher

    pcl::console::print_info("flann index ok\n");
    current_point_cloud_.swap(cloud);

    pcl::search::FlannSearch<pcl::PointXYZI>::Ptr searcher(
        new pcl::search::FlannSearch<pcl::PointXYZI>);
    searcher->setInputCloud(current_point_cloud_);

    current_cloud_searcher_.swap(searcher);

    current_sensor_center_ = center;
    current_sensor_orientation_ = orientation;
}

void CalibrationDataEstimator::loadKeypointsCloud()
{
    pcl::console::print_info("loading the core points cloud\n");
    pcl::PointCloud
        <pcl::PointXYZI>::Ptr in_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::io::loadPCDFile(input_core_points_fname_, *in_cloud);
    core_points_cloud_ = in_cloud;
    pcl::console::print_info("core point cloud ok\n");

    size_t req_size = getNumberOfClouds() * in_cloud->size();

    db_->resize(req_size);

}

void CalibrationDataEstimator::loadInputNormalsCloud()
{
    pcl::console::print_info("loading cloud with normals\n");
    pcl::PointCloud
        <pcl::PointNormal>::Ptr in_cloud(new pcl::PointCloud<pcl::PointNormal>);
    pcl::io::loadPCDFile(input_normal_surface_file_, *in_cloud);
    surface_for_normal_cloud_ = in_cloud;
    pcl::console::print_info("cloud with normals loaded\n");

    // we also create a searcher for this cloud
    pcl::search::FlannSearch<pcl::PointNormal>::Ptr searcher(
        new pcl::search::FlannSearch<pcl::PointNormal>);
    searcher->setInputCloud(in_cloud);

    surface_for_normal_cloud_searcher_ = searcher;
}

EigenTable::Ptr CalibrationDataEstimator::getCalibrationDB()
{
    return db_;
}

} // end nspace
