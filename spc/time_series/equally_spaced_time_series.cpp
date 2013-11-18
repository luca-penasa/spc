#include <spc/time_series/equally_spaced_time_series.h>

namespace spc
{


template <typename ScalarT>
EquallySpacedTimeSeries<ScalarT>::EquallySpacedTimeSeries(): x_start(0.0f), x_step(1.0f)
{

}

template<typename ScalarT>
EquallySpacedTimeSeries<ScalarT>::EquallySpacedTimeSeries(vector <ScalarT> y_, ScalarT x_step_, ScalarT x_start_)
{
    y = y_;
    x_start = x_start_;
    x_step = x_step_;
}

template<typename ScalarT>
EquallySpacedTimeSeries<ScalarT>::EquallySpacedTimeSeries(ScalarT x_step_, ScalarT x_start_, size_t size)
{
    x_start = x_start_;
    x_step = x_step_;
    resize(size);
    fill();
}


template<typename ScalarT>
EquallySpacedTimeSeries<ScalarT>::EquallySpacedTimeSeries(ScalarT x_min_, ScalarT x_max_, ScalarT step_)
{
    x_start = x_min_;
    x_step = step_;
    ScalarT range = x_max_ - x_min_;

    size_t n_samples = std::ceil(range / step_); // approximate to the upper integer

    resize(n_samples);
    fill();

}

template<typename ScalarT>
auto
EquallySpacedTimeSeries<ScalarT>::getX() const -> vector<ScalarT>
{
    std::vector<ScalarT> x(y.size());
    int counter = 0;
    for (auto &x_pos: x)
        x_pos = counter++ * x_step + x_start;

    return x;

}

/// INSTANTIATIONS
template class EquallySpacedTimeSeries<float>;
template class EquallySpacedTimeSeries<double>;


}
