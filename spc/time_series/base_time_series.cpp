#include <spc/time_series/base_time_series.h>

namespace spc {


template <typename ScalarT>
GenericTimeSeries<ScalarT>::GenericTimeSeries(): y(0)
{

}

template<typename ScalarT>
auto
GenericTimeSeries<ScalarT>::getY() const-> vector<ScalarT>
{
    return y;
}


///INSTANTIATIONS
template class GenericTimeSeries<float>;
template class GenericTimeSeries<double>;





}

