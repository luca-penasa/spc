#ifndef SINGLE_PLANE_MODEL_FROM_ONE_CLOUD_ESTIMATOR_H
#define SINGLE_PLANE_MODEL_FROM_ONE_CLOUD_ESTIMATOR_H

#include <spc/geology/normal_estimator_base.h>
#include <pcl/features/normal_3d.h>
#include <spc/geology/single_plane_stratigraphic_model.h>



namespace spc
{

class SinglePlaneModelFromOneCloudEstimator: public NormalEstimatorBase
{
public:
    SinglePlaneModelFromOneCloudEstimator();

    /// from the NormalEstimatorBase
    void estimate(SinglePlaneStratigraphicModel  &model)
    {
        Vector4f params;
        float curv;
        pcl::computePointNormal(*input_cloud_, params, curv);

        model.setParameters(params);

    }

    ///set input cloud
    void setInputCloud(cloudT::Ptr in_cloud)
    {
        input_cloud_ = in_cloud;
    }


protected:
    cloudT::Ptr input_cloud_;
};

} //end nspace

#endif // SINGLE_PLANE_MODEL_FROM_ONE_CLOUD_ESTIMATOR_H
