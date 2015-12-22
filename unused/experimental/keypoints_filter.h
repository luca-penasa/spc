#ifndef KEYPOINTS_FILTER_H
#define KEYPOINTS_FILTER_H

#include <spc/experimental/keypoints_extractor.h>
namespace spc
{

template <typename KeypointT> class KeypointsFilter
{
public:
    KeypointsFilter();

    void setInputKeypoints(const typename Keypoints<KeypointT>::Ptr keys)
    {
        keypoints_ = keys;

        // make a copy in current keys
        for (int i = 0; i < keys->size(); ++i)
            current_keys_.push_back(keys->at(i));
    }

    void filterByAverages()
    {
        Keypoints<KeypointT> new_keys;

        for (int i = 0; i < current_keys_.size(); ++i) {
            typename KeypointT::Ptr key = current_keys_.at(i);

            float w_avg, b_avg, w_std, b_std;
            w_avg = key->avg_int_white_;
            b_avg = key->avg_int_black_;
            w_std = key->std_int_white_;
            b_std = key->std_int_black_;

            if ((w_avg - w_std * n_stds_for_avg_)
                > (b_avg + b_std * n_stds_for_avg_)) // it is good!
            {
                new_keys.push_back(key);
            }
        }

        current_keys_ = new_keys;
    }

    void setNStd(float n)
    {
        n_stds_for_avg_ = n;
    }

    Keypoints<KeypointT> getCurrentKeys()
    {
        return current_keys_;
    }

private:
    float n_stds_for_avg_;

    typename Keypoints<KeypointT>::Ptr keypoints_;

    Keypoints<KeypointT> current_keys_;
};

} // end nspace

#endif // KEYPOINTS_FILTER_H
