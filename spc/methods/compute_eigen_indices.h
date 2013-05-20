#ifndef SPC_COMPUTE_EIGEN_INDICES_H
#define SPC_COMPUTE_EIGEN_INDICES_H

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

POINT_CLOUD_REGISTER_POINT_STRUCT   (PointNormalEigs,

                                     (float, normal_x, normal_x)
                                     (float, normal_y, normal_y)
                                     (float, normal_z, normal_z)
                                     (float, lam0, lam0)
                                     (float, lam1, lam1)
                                     (float, lam2, lam2))


// a struct for storing point indices of local variance on the planar surface
// they are ratio of the first two eigvalues (lam0, lam1) on the sum of lam0, lam1, lam2
struct PointEigIndices
{
    float id0;
    float id1;
    float id2;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT   (PointEigIndices,
                                     (float, id0, id0)
                                     (float, id1, id1)
                                     (float, id2, id2))


namespace ll {

            inline void
            solvePlaneParametersEigen (const Eigen::Matrix3f &covariance_matrix,
                                       float &nx, float &ny, float &nz,
                                       float &lam0, float &lam1, float &lam2
                                       );



            template<typename PointInT>
            void
            computePointNormal (const pcl::PointCloud<PointInT> &cloud, const std::vector<int> &indices,
                                float &nx, float &ny, float &nz,
                                float &lam0, float &lam1, float &lam2);


            template <typename PointT>
            void
            flipNormal (const PointT &point, float vp_x, float vp_y, float vp_z,
                        float &nx, float &ny, float &nz);



            template <typename PointT>
            void
            computeEigIndices(const pcl::PointCloud<PointT> &in_cloud,
                              const int &point_id,
                              float &id_0,
                              float &id_1,
                              float &id_2);


            template <typename PointInT, typename PointOutT>
            void
            computeDispersionIndices(const pcl::PointCloud<PointInT> &in_cloud,
                                              pcl::PointCloud<PointOutT> &out_cloud,
                                              const int &n_threads);

            template<typename PointT>
            int
            computeCovMat(const pcl::PointCloud<PointT> &in_cloud,
                          const std::vector<int> &indices,
                          Eigen::Matrix3f &covmat);

            template<typename PointInT, typename PointOutT>
            int
            computeNormalsAndEigenvalues(const typename pcl::PointCloud<PointInT>::Ptr &in_cloud,
                                const float &radius,
                                const int &n_threads,
                                pcl::PointCloud<PointOutT> &out_cloud);



}


#endif // COMPUTEEIGENINDICES_H
