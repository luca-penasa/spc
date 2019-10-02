#ifndef SPC_COMPUTE_EIGEN_INDICES_H
#define SPC_COMPUTE_EIGEN_INDICES_H

#include <spc/core/spc_eigen.h>

#ifdef SPC_WITH_PCL

#include <pcl/features/feature.h>
#include <pcl/point_cloud.h>

// a struct for storing point normals plus the eigenvalues
// we are not interested in eigenvectors, so we discard them
struct PointNormalEigs
{
    PCL_ADD_NORMAL4D;
    float lam0;
    float lam1;
    float lam2;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(
    PointNormalEigs,
    (float, normal_x, normal_x)(float, normal_y, normal_y)(
        float, normal_z, normal_z)(float, lam0, lam0)(float, lam1,
                                                      lam1)(float, lam2, lam2))

// a struct for storing point indices of local variance on the planar surface
// they are ratio of the first two eigvalues (lam0, lam1) on the sum of lam0,
// lam1, lam2
struct PointEigIndices
{
    float id0;
    float id1;
    float id2;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(PointEigIndices,
                                  (float, id0, id0)(float, id1, id1)(float, id2,
                                                                     id2))
namespace spc
{

inline void
    ///
    /// \brief solvePlaneParametersEigen
    /// \param covariance_matrix
    /// \param nx
    /// \param ny
    /// \param nz
    /// \param lam0
    /// \param lam1
    /// \param lam2
    ///
    solvePlaneParametersEigen(const Eigen::Matrix3f &covariance_matrix,
                              float &nx, float &ny, float &nz, float &lam0,
                              float &lam1, float &lam2);



template <typename PointInT>
///
/// \brief computePointNormal
/// \param cloud
/// \param indices
/// \param nx
/// \param ny
/// \param nz
/// \param lam0
/// \param lam1
/// \param lam2
///
void computePointNormal(const pcl::PointCloud<PointInT> &cloud,
                        const std::vector<int> &indices, float &nx, float &ny,
                        float &nz, float &lam0, float &lam1, float &lam2);



template <typename PointT>
///
/// \brief flipNormal
/// \param point
/// \param vp_x
/// \param vp_y
/// \param vp_z
/// \param nx
/// \param ny
/// \param nz
///
void flipNormal(const PointT &point, float vp_x, float vp_y, float vp_z,
                float &nx, float &ny, float &nz);



template <typename PointT>
///
/// \brief computeEigIndices
/// \param in_cloud
/// \param point_id
/// \param id_0
/// \param id_1
/// \param id_2
///
void computeEigIndices(const pcl::PointCloud<PointT> &in_cloud,
                       const int &point_id, float &id_0, float &id_1,
                       float &id_2);

template <typename PointInT, typename PointOutT>
///
/// \brief computeDispersionIndices
/// \param in_cloud
/// \param out_cloud
/// \param n_threads
///
void computeDispersionIndices(const pcl::PointCloud<PointInT> &in_cloud,
                              pcl::PointCloud<PointOutT> &out_cloud,
                              const int &n_threads);

template <typename PointT>
///
/// \brief computeCovMat
/// \param in_cloud
/// \param indices
/// \param covmat
/// \return
///
int computeCovMat(const pcl::PointCloud<PointT> &in_cloud,
                  const std::vector<int> &indices, Eigen::Matrix3f &covmat);

template <typename PointInT, typename PointOutT>
///
/// \brief computeNormalsAndEigenvalues
/// \param in_cloud
/// \param radius
/// \param n_threads
/// \param out_cloud
/// \return
///
int computeNormalsAndEigenvalues(const typename pcl::PointCloud<PointInT>::Ptr
                                 &in_cloud,
                                 const float &radius, const int &n_threads,
                                 pcl::PointCloud<PointOutT> &out_cloud);


}

#endif
#endif // COMPUTEEIGENINDICES_H
