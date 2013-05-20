#include <spc/time_series/equally_spaced_time_series.h>


template <typename ScalarT>
ll::EquallySpacedTimeSeries<ScalarT>::EquallySpacedTimeSeries(): x_start(0.0f), x_step(1.0f)
{

}

template<typename ScalarT>
ll::EquallySpacedTimeSeries<ScalarT>::EquallySpacedTimeSeries(vector <ScalarT> y_, ScalarT x_step_, ScalarT x_start_)
{
    y = y_;
    x_start = x_start_;
    x_step = x_step_;
}

template<typename ScalarT>
ll::EquallySpacedTimeSeries<ScalarT>::EquallySpacedTimeSeries(ScalarT x_step_, ScalarT x_start_, size_t size)
{
    x_start = x_start_;
    x_step = x_step_;
    resize(size);
    fill();
}


template<typename ScalarT>
ll::EquallySpacedTimeSeries<ScalarT>::EquallySpacedTimeSeries(ScalarT x_min_, ScalarT x_max_, ScalarT step_)
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
ll::EquallySpacedTimeSeries<ScalarT>::getX() -> vector<ScalarT>
{
    std::vector<ScalarT> x(y.size());
    int counter = 0;
    for (auto &x_pos: x)
        x_pos = counter++ * x_step + x_start;

    return x;

}

/// INSTANTIATIONS
template class ll::EquallySpacedTimeSeries<float>;
template class ll::EquallySpacedTimeSeries<double>;
