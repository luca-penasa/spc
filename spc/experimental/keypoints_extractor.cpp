#include "keypoints_extractor.h"

namespace spc
{
template <typename PointInT> KeypointsExtractor<PointInT>::KeypointsExtractor()
{

    use_only_full_size_ = true;
}

template class KeypointsExtractor<pcl::PointXYZI>;
} // end namespace
