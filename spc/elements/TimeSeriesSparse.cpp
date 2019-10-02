#include <spc/elements/TimeSeriesSparse.h>
#include <algorithm>

namespace spc
{
DtiClassType TimeSeriesSparse::Type ("TimeSeriesSparse:", &TimeSeriesBase::Type);



void TimeSeriesSparse::resize(size_t size_)
{
    x_.resize(size_);
    this->y_.resize(size_);
}



TimeSeriesSparse::ScalarT TimeSeriesSparse::getMinX() const
{
        return x_.minCoeff();

}

TimeSeriesSparse::ScalarT TimeSeriesSparse::getMaxX() const
{

        return x_.maxCoeff();
}

}// end nspace
#include <spc/core/spc_cereal.hpp>
SPC_CEREAL_REGISTER_TYPE(spc::TimeSeriesSparse)
