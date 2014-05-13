#include <spc/calibration/CalibrationDataEstimator.h>
#include <pcl/filters/bilateral.h>

#include <pcl/surface/mls.h>

namespace spc
{

CalibrationDataEstimator::CalibrationDataEstimator(): normal_estimation_search_radius_(0.1),
    normal_estimation_method_(FULL_NORMALS_ESTIMATION),
    maximum_squared_distance_for_normal_getting_(0.1),
    intensity_estimation_method_(SIMPLE_AVERAGE),
    intensity_estimation_spatial_sigma_(0.1)
{
}

void CalibrationDataEstimator::setInputClouds(std::vector<std::string> cloud_names)
{
    input_fnames_ = cloud_names;
}

void CalibrationDataEstimator::setInputCorePoints(std::string core_points_name)
{
    input_core_points_fname_ = core_points_name;
}

void CalibrationDataEstimator::getNearestNormal(const pcl::PointXYZI &point, float &nx, float &ny, float &nz, float sq_dist_limit)
{
    pcl::PointNormal p_no;
    p_no.x = point.x;
    p_no.y = point.y;
    p_no.z = point.z;
    std::vector<int> ids;
    std::vector<float> sq_dists;
    surface_for_normal_cloud_searcher_->nearestKSearch(p_no, 1, ids, sq_dists );

    if (ids.empty() )
    {
        nx = NAN;
        ny = NAN;
        nz = NAN;
        return;
    }

    if ( (sq_dists.at(0) >= sq_dist_limit))
    {
        nx = std::numeric_limits<float>::quiet_NaN();
        ny = std::numeric_limits<float>::quiet_NaN();
        nz = std::numeric_limits<float>::quiet_NaN();
        return;
    }


    //get the normal for that point
    p_no = surface_for_normal_cloud_->at(ids.at(0));
    nx = p_no.normal_x;
    ny = p_no.normal_y;
    nz = p_no.normal_z;

    return;
}

void CalibrationDataEstimator::setSearchRadius(const float rad)
{
    normal_estimation_search_radius_ = rad;
}

CorePointData::Ptr CalibrationDataEstimator::computeCorePointParameters(const size_t core_point_id, const float search_radius)
{
    CorePointData::Ptr out (new CorePointData);

    //get the point coordinates from core point cloud
    pcl::PointXYZI point = core_points_cloud_->at(core_point_id);

    std::vector<float> dists;
    std::vector<int> ids;

    current_cloud_searcher_->radiusSearch(point, search_radius, ids, dists );

    ////////////////////////////////// NORMAL ESTIMATION /////////////////////////////
    //compute normal and average distance
    float nx(NAN);
    float ny(NAN);
    float nz(NAN);
    float lam0(NAN);
    float lam1(NAN);
    float lam2(NAN);


    if (normal_estimation_method_ == FULL_NORMALS_ESTIMATION)
    {
        spc::computePointNormal (*current_point_cloud_, ids,
                                 nx, ny, nz, lam0, lam1, lam2);

    }
    else if (normal_estimation_method_ == PRECOMPUTED_NORMALS)
    {
        getNearestNormal(point, nx, ny, nz, maximum_squared_distance_for_normal_getting_);
    }

    /////////////////////// DISTANCE ESTIMATION ////////////////////////////
    Eigen::Vector4f _c(NAN, NAN, NAN, NAN); //tmp_var

    if (ids.size() > 0)
    {
        pcl::compute3DCentroid(*current_point_cloud_, ids, _c);
    }


    Eigen::Vector3f c (_c(0),_c(1),_c(2));
    Eigen::Vector3f pos = Eigen::Vector3f(current_sensor_center_(0), current_sensor_center_(1), current_sensor_center_(2));
    Eigen::Vector3f ray = c - pos;

    float distance = ray.norm();

    auto n_vec = Eigen::Vector3f(nx, ny, nz);
    auto lambdas = Eigen::Vector3f(lam0, lam1, lam2);


    //////////////////////// INTENSITY ESTIMATION ////////////////////////
    float intensity = NAN;

    if (intensity_estimation_method_ == SIMPLE_AVERAGE)
        intensity = this->getAverageIntensity(*current_point_cloud_, ids);

    else if (intensity_estimation_method_ == GAUSSIAN_ESTIMATION)
    {
        // we redo a neighbors search

        float full_radius = intensity_estimation_spatial_sigma_ * 4;

        std::vector<float> dists_int;
        std::vector<int> ids_int;

        current_cloud_searcher_->radiusSearch(point, search_radius, ids_int, dists_int );

        intensity = CalibrationDataEstimator::computeGaussianSmoothedIntensity(current_point_cloud_, ids_int, dists_int, intensity_estimation_spatial_sigma_);
    }


    ////////////////////  SAVE ALL THE STUFF /////////////////////////

    //number and which neighbors used for this dataset
    out->value("n_neighbors") = ids.size();
    out->value("neighbors") = ids;
    out->value("neighbors_dists") = dists;

    //normal and goodnees of fit
    out->value("normal") = n_vec;
    out->value("lambdas") = lambdas;

    //the local centroid
    out->value("centroid") = c;


    //position of sensor
    out->value("sensor_position") = pos;

    // ray from sensor to the center of mass of the core point
    out->value("ray") = ray;

    // the id of the core point
    out->value("core_id") = core_point_id;

    // cloud on which it was computed
    out->value("cloud_name") = current_cloud_name_;

    //a progressive id for this cloud
    out->value("cloud_id") = current_cloud_id_;

    //the average distance of the core point (its center of mass) from the sensor
    out->value("distance") = distance;

    // the local average intensity
    out->value("intensity") = intensity;

    //the scattering angle
    out->value("angle") = CalibrationDataEstimator::getMinimumAngleBetweenVectors(n_vec, ray);

    return out;

}

float CalibrationDataEstimator::getMinimumAngleBetweenVectors(const Vector3f x_, const Vector3f y_)
{
    //ensure they are normalized
    Eigen::Vector3f x = x_ / x_.norm();
    Eigen::Vector3f y = y_ / y_.norm();

    float cosTheta = x.dot(y);
    float theta = acos(std::min(fabs(cosTheta),1.0)) ;

    theta *= 180.0/M_PI;

    return theta;
}

float CalibrationDataEstimator::getAverageIntensity(const pcl::PointCloud<pcl::PointXYZI> &cloud, std::vector<int> ids)
{
    if (ids.size() == 0)
        return NAN;

    float sum = 0;
    BOOST_FOREACH(int id, ids)
            sum += cloud.at(id).intensity ;

    sum /= ids.size();

    return sum;
}

void CalibrationDataEstimator::loadCloudAndCreateSearcher(const std::string fname)
{
    pcl::console::print_info("loading cloud\n");

    pcl::PCLPointCloud2::Ptr in_cloud (new pcl::PCLPointCloud2);

    Eigen::Vector4f center;
    Eigen::Quaternionf orientation;

    //LOAD
    pcl::io::loadPCDFile(fname, *in_cloud, center, orientation);

    //convert to a PointCloud type object
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromPCLPointCloud2(*in_cloud, *cloud);

    pcl::console::print_info("cloud loaded\n");
    //TODO check that the intensity fields exists, please

    pcl::console::print_info("creating flann index\n");
    //now create the searcher
    pcl::search::FlannSearch<pcl::PointXYZI>::Ptr searcher (new pcl::search::FlannSearch<pcl::PointXYZI>);
    searcher->setInputCloud(cloud);

    pcl::console::print_info("flann index ok\n");
    current_point_cloud_ = cloud;
    current_cloud_searcher_ = searcher;

    current_sensor_center_ = center;
    current_sensor_orientation_ = orientation;
}

void CalibrationDataEstimator::loadCorePointsCloud()
{
    pcl::console::print_info("loading the core points cloud\n");
    pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::io::loadPCDFile(input_core_points_fname_, *in_cloud);
    core_points_cloud_ = in_cloud;
    pcl::console::print_info("core point cloud ok\n");
}

void CalibrationDataEstimator::loadInputNormalsCloud()
{
    pcl::console::print_info("loading cloud with normals\n");
    pcl::PointCloud<pcl::PointNormal>::Ptr in_cloud (new pcl::PointCloud<pcl::PointNormal>);
    pcl::io::loadPCDFile(input_normal_surface_file_, *in_cloud);
    surface_for_normal_cloud_ = in_cloud;
    pcl::console::print_info("cloud with normals loaded\n");

    // we also create a searcher for this cloud
    pcl::search::FlannSearch<pcl::PointNormal>::Ptr searcher (new pcl::search::FlannSearch<pcl::PointNormal>);
    searcher->setInputCloud(in_cloud);

    surface_for_normal_cloud_searcher_ = searcher;
}

CalibrationDataDB CalibrationDataEstimator::getCalibrationDB()
{
    return db_;
}

} //end nspace



