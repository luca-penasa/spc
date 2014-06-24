#include <spc/elements/TimeSeriesSparse.h>
#include <algorithm>

namespace spc
{
DtiClassType TimeSeriesSparse::Type ("TimeSeriesSparse:", &TimeSeriesBase::Type);

TimeSeriesSparse::TimeSeriesSparse(const std::vector<TimeSeriesSparse::ScalarT> &x, const std::vector<TimeSeriesSparse::ScalarT> &y)
{
    assert(x.size() == y.size()); // must have same size
    x_ = x;
    this->y_ = y;
}

void TimeSeriesSparse::resize(size_t size_)
{
    x_.resize(size_);
    this->y_.resize(size_);
}

TimeSeriesSparse::ScalarT TimeSeriesSparse::getMinX() const
{
    if (!x_.empty())
        return *std::min_element(x_.begin(), x_.end());
    else
        return std::numeric_limits<ScalarT>::quiet_NaN();
}

TimeSeriesSparse::ScalarT TimeSeriesSparse::getMaxX() const
{
    if (!x_.empty())
        return *std::max_element(x_.begin(), x_.end());
    else
        return std::numeric_limits<ScalarT>::quiet_NaN();
}

}
