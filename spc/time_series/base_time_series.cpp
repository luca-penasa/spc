#include <spc/time_series/base_time_series.h>

namespace spc {


template<typename ScalarT>
void GenericTimeSeries<ScalarT>::fillRandomY(ScalarT min, ScalarT max)
{

    boost::random::random_device rng;
    boost::random::uniform_real_distribution<> index_dist(min, max);

    BOOST_FOREACH (ScalarT& y, this->getY())
    {
        y = index_dist(rng) ;
    }
}

///INSTANTIATIONS
template class GenericTimeSeries<float>;
template class GenericTimeSeries<double>;







}

