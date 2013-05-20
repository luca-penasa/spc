#include <spc/time_series/sparse_time_series.h>

template <typename ScalarT>
ll::SparseTimeSeries<ScalarT>::SparseTimeSeries(vector<ScalarT> x_, vector<ScalarT> y_)
{
    assert(x_.size() == y_.size()); //must have same size
    x = x_; y = y_;
}


///// INSTANTIATIONS //////////
template class ll::SparseTimeSeries<float>;
template class ll::SparseTimeSeries<double>;
