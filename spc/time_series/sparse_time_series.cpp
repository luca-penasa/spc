#include <spc/time_series/sparse_time_series.h>

namespace spc
{


template <typename ScalarT>
SparseTimeSeries<ScalarT>::SparseTimeSeries(vector<ScalarT> x_, vector<ScalarT> y_)
{
    assert(x_.size() == y_.size()); //must have same size
    x = x_; y = y_;
}


///// INSTANTIATIONS //////////
template class SparseTimeSeries<float>;
template class SparseTimeSeries<double>;



}
