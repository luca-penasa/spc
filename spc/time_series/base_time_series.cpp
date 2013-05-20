#include <spc/time_series/base_time_series.h>

template <typename ScalarT>
ll::GenericTimeSeries<ScalarT>::GenericTimeSeries(): y(0)
{

}

template<typename ScalarT>
auto
ll::GenericTimeSeries<ScalarT>::getY() -> vector<ScalarT>
{
    return y;
}


///INSTANTIATIONS
template class ll::GenericTimeSeries<float>;
template class ll::GenericTimeSeries<double>;
