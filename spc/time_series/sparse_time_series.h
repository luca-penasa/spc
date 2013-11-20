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
    using GenericTimeSeries<ScalarT>::y;

public:


    ///
    /// \brief SparseTimeSeries def const. does nothing
    ///
    SparseTimeSeries();

    ///
    /// \brief SparseTimeSeries constructor
    /// x_ and y_ must have the same size
    /// \param x_ vector of x positions
    /// \param y_ vector of y values
    ///
    SparseTimeSeries(vector<ScalarT> x_, vector<ScalarT> y_);

    virtual std::string getSPCClassName()
    {
        std::string name = "SparseTimeSeries";
        return name;
    }


    ///
    /// \brief getX get x positions
    /// \return the vector of x positions
    /// reimplemented from base class
    ///
    auto getX() const -> vector<ScalarT> {return x;}

    ///
    /// \brief resize the x and y vectors
    /// \param size_ number of samples
    /// reimplemented from base class
    ///
    void resize(size_t size_) {x.resize(size_); y.resize(size_);}

    ///
    /// \brief getMinX get min value of x
    /// \return the minimum value o the x vector
    /// reimplemented from base class
    ///
    virtual auto getMinX() -> ScalarT const { return *std::min_element(x.begin(), x.end());}

    ///
    /// \brief getMaxX get max value of x
    /// \return the maximum value o the x vector
    /// reimplemented from base class
    ///
    virtual auto getMaxX() -> ScalarT const { return *std::max_element(x.begin(), x.end());}

protected:
    ///
    /// \brief x is the vector containing the x positions of the time series
    /// added to the base class attributes
    ///
    vector<ScalarT> x;

};


} //end namespace sp
#endif
