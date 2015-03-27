#include <spc/methods/PointCloudEigenIndicesEstimator.h>
namespace spc
{

//////////////////////////////////////////////////////////////////////////////////////////////////////
inline void solvePlaneParametersEigen(const Eigen::Matrix3f &covariance_matrix,
                                      float &nx, float &ny, float &nz,
                                      float &lam0, float &lam1, float &lam2)
{
    Eigen::Matrix3f eigen_vectors;
    Eigen::Vector3f eigen_values;

    Eigen::SelfAdjointEigenSolver
        <Eigen::Matrix3f> eigensolver(covariance_matrix);

    if (eigensolver.info() != Eigen::Success) {
        nx = ny = nz = lam0 = lam1 = lam2 = std::numeric_limits
            <float>::quiet_NaN();
        return;
    }

    eigen_vectors = eigensolver.eigenvectors();
    eigen_values = eigensolver.eigenvalues();

    // first column, no?
    nx = eigen_vectors(0, 0);
    ny = eigen_vectors(1, 0);
    nz = eigen_vectors(2, 0);

    lam0 = eigen_values(0);
    lam1 = eigen_values(1);
    lam2 = eigen_values(2);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT>
void computePointNormal(const pcl::PointCloud<PointInT> &cloud,
                        const std::vector<int> &indices, float &nx, float &ny,
                        float &nz, float &lam0, float &lam1, float &lam2)
{

    Eigen::Matrix3f covariance_matrix;

    if (indices.size() < 3 || computeCovMat(cloud, indices, covariance_matrix)
                              == 0) {
        nx = ny = nz = std::numeric_limits<float>::quiet_NaN();
        lam0 = lam1 = lam2 = std::numeric_limits<float>::quiet_NaN();
        return;
    }

    // Get the plane normal and surface curvature
    solvePlaneParametersEigen(covariance_matrix, nx, ny, nz, lam0, lam1, lam2);
}
//////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
void flipNormal(const PointT &point, float vp_x, float vp_y, float vp_z,
                float &nx, float &ny, float &nz)
{

    // See if we need to flip any plane normals
    vp_x -= point.x;
    vp_y -= point.y;
    vp_z -= point.z;

    // Dot product between the (viewpoint - point) and the plane normal
    float cos_theta = (vp_x * nx + vp_y * ny + vp_z * nz);

    // Flip the plane normal
    if (cos_theta < 0) {
        nx *= -1;
        ny *= -1;
        nz *= -1;
    }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT>
void computeEigIndices(const pcl::PointCloud<PointInT> &in_cloud,
                       const int &point_id, float &id_0, float &id_1,
                       float &id_2)
{
    float sum = in_cloud[point_id].lam0 + in_cloud[point_id].lam1
                + in_cloud[point_id].lam2;
    if (sum == 0) {
        id_0 = id_1 = id_2 = 0.0;
    } else {
		id_0 = std::abs(in_cloud[point_id].lam0 / sum);
		id_1 = std::abs(in_cloud[point_id].lam1 / sum);
		id_2 = std::abs(in_cloud[point_id].lam2 / sum);
    }
}

template <typename PointInT, typename PointOutT>
void computeDispersionIndices(const pcl::PointCloud<PointInT> &in_cloud,
                              pcl::PointCloud<PointOutT> &out_cloud,
                              const int &n_threads)
{
    out_cloud.resize(in_cloud.size());
#ifdef USE_OPENMP
#pragma omp parallel for shared(out_cloud) num_threads(n_threads)
#endif
    for (int i = 0; i < in_cloud.size(); ++i) {
        computeEigIndices<PointNormalEigs>(in_cloud, i, out_cloud[i].id0,
                                           out_cloud[i].id1, out_cloud[i].id2);
    }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
int computeCovMat(const pcl::PointCloud<PointT> &in_cloud,
                  const std::vector<int> &indices, Eigen::Matrix3f &covmat)
{
    covmat = Eigen::Matrix3f::Zero();
    int n_points = indices.size();

    float mean_x = 0, mean_y = 0, mean_z = 0;
    for (int i = 0; i < n_points; ++i) {
        mean_x += in_cloud[indices[i]].x;
        mean_y += in_cloud[indices[i]].y;
        mean_z += in_cloud[indices[i]].z;
    }

    mean_x /= n_points;
    mean_y /= n_points;
    mean_z /= n_points;

    for (int i = 0; i < n_points; ++i) {
        covmat(0, 0) += (in_cloud[indices[i]].x - mean_x)
                        * (in_cloud[indices[i]].x - mean_x);
        covmat(0, 1) += (in_cloud[indices[i]].x - mean_x)
                        * (in_cloud[indices[i]].y - mean_y);
        covmat(0, 2) += (in_cloud[indices[i]].x - mean_x)
                        * (in_cloud[indices[i]].z - mean_z);

        covmat(1, 1) += (in_cloud[indices[i]].y - mean_y)
                        * (in_cloud[indices[i]].y - mean_y);
        covmat(1, 2) += (in_cloud[indices[i]].y - mean_y)
                        * (in_cloud[indices[i]].z - mean_z);

        covmat(2, 2) += (in_cloud[indices[i]].z - mean_z)
                        * (in_cloud[indices[i]].z - mean_z);
    }

    covmat(1, 0) = covmat(0, 1);
    covmat(2, 0) = covmat(0, 2);
    covmat(2, 1) = covmat(1, 2);

    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) {
            covmat(i, j) /= (n_points - 1);
        }

    return n_points;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointOutT>
int computeNormalsAndEigenvalues(const typename pcl::PointCloud<PointInT>::Ptr
                                 &in_cloud,
                                 const float &radius, const int &n_threads,
                                 pcl::PointCloud<PointOutT> &out_cloud)
{
    size_t n_points = in_cloud->size();
    // resize the out cloud
    out_cloud.resize(n_points);

    // build the kdtree
    pcl::KdTreeFLANN<PointInT> flann;
    flann.setInputCloud(in_cloud);

    // place for neighs ids and distances
    std::vector<int> indices;
    std::vector<float> distances;
#ifdef USE_OPENMP
#pragma omp parallel for shared(out_cloud) private(indices, distances)         \
    num_threads(n_threads)
#endif
    for (int i = 0; i < n_points; ++i) {
        // get neighbors for this point
        flann.radiusSearch((*in_cloud)[i], radius, indices, distances);

        // estimate normal and eigenvalues
        computePointNormal(*in_cloud, indices, out_cloud[i].normal_x,
                           out_cloud[i].normal_y, out_cloud[i].normal_z,
                           out_cloud[i].lam0, out_cloud[i].lam1,
                           out_cloud[i].lam2);

        flipNormal
            <PointInT>((*(in_cloud))[i], 0.0, 0.0, 0.0, out_cloud[i].normal_x,
                       out_cloud[i].normal_y, out_cloud[i].normal_z);
    }

    return 1;
}

template int computeNormalsAndEigenvalues<pcl::PointXYZ, PointNormalEigs>(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr &in_cloud, const float &radius,
    const int &n_threads, pcl::PointCloud<PointNormalEigs> &out_cloud);

template void computeDispersionIndices(const pcl::PointCloud
                                       <PointNormalEigs> &in_cloud,
                                       pcl::PointCloud
                                       <PointEigIndices> &out_cloud,
                                       const int &n_threads);

template int computeCovMat(const pcl::PointCloud<pcl::PointXYZI> &in_cloud,
                           const std::vector<int> &indices,
                           Eigen::Matrix3f &covmat);

template void computePointNormal(const pcl::PointCloud<pcl::PointXYZI> &cloud,
                                 const std::vector<int> &indices, float &nx,
                                 float &ny, float &nz, float &lam0, float &lam1,
                                 float &lam2);

} // end nspace
