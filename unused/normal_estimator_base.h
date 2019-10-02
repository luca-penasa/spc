#ifndef NORMAL_ESTIMATOR_BASE_H
#define NORMAL_ESTIMATOR_BASE_H

#include <spc/stratigraphy/stratigraphic_model_base.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace spc
{

class NormalEstimatorBase
{
public:
    typedef typename pcl::PointCloud<pcl::PointXYZ> cloudT;
    typedef typename boost::shared_ptr<NormalEstimatorBase> Ptr;

    NormalEstimatorBase();



};

}//end nspace

#endif // NORMAL_ESTIMATOR_BASE_H
