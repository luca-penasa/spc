#include <spc/elements/TimeSeriesEquallySpaced.h>

namespace spc
{
DtiClassType TimeSeriesEquallySpaced::Type ("TimeSeriesEquallySpaced", &TimeSeriesBase::Type);


TimeSeriesEquallySpaced::TimeSeriesEquallySpaced(const TimeSeriesEquallySpaced &other)
{
    x_start = other.x_start;
    x_step = other.x_step;
}

TimeSeriesEquallySpaced::TimeSeriesEquallySpaced(std::vector<ScalarT> y_, ScalarT x_step_,
                                       ScalarT x_start_)
{
    y_ = y_;
    x_start = x_start_;
    x_step = x_step_;
}

//TimeSeriesEquallySpaced::TimeSeriesEquallySpaced(ScalarT x_step_, ScalarT x_start_,
//                                       size_t size)
//{
//    x_start = x_start_;
//    x_step = x_step_;
//    resize(size);
//    this->fill();
//}

TimeSeriesEquallySpaced::TimeSeriesEquallySpaced(ScalarT x_min_, ScalarT x_max_,
                                       ScalarT step_)
{
    x_start = x_min_;
    x_step = step_;
    ScalarT range = x_max_ - x_min_;

    size_t n_samples
        = std::ceil(range / step_); // approximate to the upper integer

    resize(n_samples);
    this->fill();
}

TimeSeriesBase::VectorT TimeSeriesEquallySpaced::getX() const
{
    VectorT x(this->y_.size());

#ifdef USE_OPENMP
#pragma omp parallel for
#endif
    for (int i = 0 ; i < x.size(); ++i)
    {
        x(i) = i * x_step + x_start;
    }
    return x;
}


}


#include <spc/core/spc_cereal.hpp>
SPC_CEREAL_REGISTER_TYPE(spc::TimeSeriesEquallySpaced)
