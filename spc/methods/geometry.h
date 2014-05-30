#ifndef SPC_GEOMETRY_H
#define SPC_GEOMETRY_H

#include <pcl/point_cloud.h>

// Specialize point types
#include <spc/elements/point_types.h>
namespace spc
{
/** \brief Compute distances from origin for a given cloud.
 *
 * As origin the origin attribute of the cloud is used
 *
 * \param[in] incloud the name of the file to load
 * \param[out] outcloud resultant point cloud dataset
 *
 */

template <typename PointT>
void computeDistanceFromOrigin(const pcl::PointCloud<PointT> &incloud,
                               pcl::PointCloud<PointD> &outcloud);

template <typename PointT>
float getAverageDistanceFromSensor(const pcl::PointCloud<PointT> &cloud,
                                   const std::vector<int> ids);

} // end nspace
#endif
