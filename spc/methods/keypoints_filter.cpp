#include "keypoints_filter.h"
namespace spc
{

template<typename KeypointT>
KeypointsFilter<KeypointT>::KeypointsFilter()
{
    n_stds_for_avg_ = 1.0;
}



//force instantiation
template class KeypointsFilter<Keypoint<pcl::PointXYZI> >;

}//end nspace
