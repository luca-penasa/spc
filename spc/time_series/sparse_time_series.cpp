#include <spc/time_series/sparse_time_series.h>

namespace spc
{


template <typename ScalarT>
SparseTimeSeries<ScalarT>::SparseTimeSeries(const vector<ScalarT> &x, const vector<ScalarT> &y)
{
    assert(x.size() == y.size()); //must have same size
    x_ = x; this->y_ = y;
}


///// INSTANTIATIONS //////////
template class SparseTimeSeries<float>;
template class SparseTimeSeries<double>;



}
