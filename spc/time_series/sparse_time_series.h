#ifndef SPC_SPARSE_TIME_SERIES_H
#define SPC_SPARSE_TIME_SERIES_H

#include <spc/time_series/base_time_series.h>
#include <algorithm>
namespace  spc {

using namespace std;
template <typename ScalarT>
///
/// \brief The SparseTimeSeries class represents a time series that is sparse.
/// for each y value in y vector a correspondent x value is defined.
/// use getX() and getY() to get back the corresponding vectors.
/// They must have the same size!
/// \ingroup time_series
///
class SparseTimeSeries: public GenericTimeSeries<ScalarT>
{
public:

    typedef boost::shared_ptr<SparseTimeSeries<ScalarT> > Ptr;
    typedef boost::shared_ptr< const SparseTimeSeries<ScalarT> > ConstPtr;

    typedef typename GenericTimeSeries<ScalarT>::VectorT VectorT;

public:


    ///
    /// \brief SparseTimeSeries def const. does nothing
    ///
    SparseTimeSeries()
    {

    }

    ///
    /// \brief SparseTimeSeries constructor
    /// x_ and y_ must have the same size
    /// \param x_ vector of x positions
    /// \param y_ vector of y values
    ///
    SparseTimeSeries(vector<ScalarT> x, vector<ScalarT> y);


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
    /// reimplemented from base class
    ///
    virtual ScalarT getMinX() const
    {
        return *std::min_element(x_.begin(), x_.end());
    }

    ///
    /// \brief getMaxX get max value of x
    /// \return the maximum value o the x vector
    /// reimplemented from base class
    ///
    virtual ScalarT getMaxX()  const
    {
        return *std::max_element(x_.begin(), x_.end());
    }

protected:
    ///
    /// \brief x is the vector containing the x positions of the time series
    /// added to the base class attributes
    ///
    vector<ScalarT> x_;

};


} //end namespace sp
#endif
