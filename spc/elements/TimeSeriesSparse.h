#ifndef SPC_SPARSE_TIME_SERIES_H
#define SPC_SPARSE_TIME_SERIES_H

#include "TimeSeriesBase.h"
#include <algorithm>

#include <cereal/types/vector.hpp>
namespace spc
{

using namespace std;
template <typename ScalarT>
///
/// \brief The SparseTimeSeries class represents a time series that is sparse.
/// for each y value in y vector a correspondent x value is defined.
/// use getX() and getY() to get back the corresponding vectors.
/// They must have the same size!
/// \ingroup time_series
///
class TimeSeriesSparse : public TimeSeriesBase<ScalarT>
{
public:
    SPC_OBJECT(TimeSeriesSparse)

    typedef typename TimeSeriesBase<ScalarT>::VectorT VectorT;

public:
    ///
    /// \brief SparseTimeSeries def const. does nothing
    ///
    TimeSeriesSparse()
    {
    }

    ///
    /// \brief SparseTimeSeries constructor
    /// x_ and y_ must have the same size
    /// \param x_ vector of x positions
    /// \param y_ vector of y values
    ///
    TimeSeriesSparse(const vector<ScalarT> &x, const vector<ScalarT> &y)
    {
        assert(x.size() == y.size()); // must have same size
        x_ = x;
        this->y_ = y;
    }

    ///
    /// \brief getX get x positions
    /// \return the vector of x positions
    /// reimplemented from base class
    ///
    VectorT getX() const
    {
        return x_;
    }

    VectorT &getX()
    {
        return x_;
    }

    ScalarT &getX(size_t id)
    {
        return x_.at(id);
    }

    ScalarT getX(size_t id) const
    {
        return x_.at(id);
    }

    ///
    /// \brief resize the x and y vectors
    /// \param size_ number of samples
    /// reimplemented from base class
    ///
    void resize(size_t size_)
    {
        x_.resize(size_);
        this->y_.resize(size_);
    }

    ///
    /// \brief getMinX get min value of x
    /// \return the minimum value o the x vector
    /// reimplemented from base class nan if void
    ///
    virtual ScalarT getMinX() const
    {
        if (!x_.empty())
            return *std::min_element(x_.begin(), x_.end());
        else
            return std::numeric_limits<ScalarT>::quiet_NaN();
    }

    ///
    /// \brief getMaxX get max value of x
    /// \return the maximum value o the x vector
    /// reimplemented from base class nan if void
    ///
    virtual ScalarT getMaxX() const
    {
        if (!x_.empty())
            return *std::max_element(x_.begin(), x_.end());
        else
            return std::numeric_limits<ScalarT>::quiet_NaN();
    }

protected:
    ///
    /// \brief x is the vector containing the x positions of the time series
    /// added to the base class attributes
    ///
    vector<ScalarT> x_;

private:
    friend class cereal::access;

    template <class Archive> void serialize(Archive &ar)
    {
        ar(cereal::base_class<TimeSeriesBase<ScalarT>>(this),
           CEREAL_NVP(x_));
    }
};

} // end namespace sp
#endif
