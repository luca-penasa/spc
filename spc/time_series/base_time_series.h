#ifndef SPC_BASE_TIME_SERIES_H
#define SPC_BASE_TIME_SERIES_H


#include <string>
#include <vector>
#include <assert.h>
#include <spc/elements/element_base.h>

#include <boost/random/random_device.hpp>

#include <boost/random/uniform_real_distribution.hpp>


namespace spc
{

///
/// \defgroup time_series Time Series Objects
///
using namespace std;
template <typename ScalarT>
///
/// \brief The GenericTimeSeries class is a Generic class for time series-like (TS) objects.
/// \ingroup time_series
///
class GenericTimeSeries: public spcElementBase
{
public:

    GenericTimeSeries() {}

    typedef boost::shared_ptr<GenericTimeSeries<ScalarT> > Ptr;
    typedef boost::shared_ptr<const GenericTimeSeries<ScalarT> > ConstPtr;

    typedef std::vector<ScalarT> VectorT;
    typedef ScalarT ScaT;

    void fill(const ScalarT value_ = std::numeric_limits<ScalarT>::quiet_NaN())
    {
        std::fill(this->y_.begin(), this->y_.end(), value_);
    }

    /// by default will generate in -1 to 1 range
    //void fillRandomY(ScalarT min=-1, ScalarT max=1);


    ///
    /// \brief getNumberOfSamples in time series
    /// \return the number of samples
    ///

    size_t getNumberOfSamples() const {return y_.size();}

    ///
    /// \brief getX positions
    /// \return a vector of x positions
    ///
    virtual vector<ScalarT> getX() const = 0;

    ///
    /// \brief getY values
    /// \return a vector of the y values associated with x
    ///
    VectorT getY() const
    {
        return y_;
    }

    VectorT &getY()
    {
        return y_;
    }

    ScalarT &getY(size_t id)
    {
        return y_.at(id);
    }

    ///
    /// \brief setY values
    ///
    void setY(vector<ScalarT> y) {y_ = y;}

    ///
    /// \brief Provide a way to resize the time series
    /// \param size_ number of samples
    /// It resizes the vector of y values
    ///
    virtual void resize(size_t size_) = 0;

    ///
    /// \brief getMinX for the lowest value of the x positions
    /// \return the min value of the x positions
    /// Must be implemented in each derived class
    ///
    virtual ScalarT getMinX() const = 0 ;

    ///
    /// \brief getMaxX for the higher value of the x positions
    /// \return the max value of the x positions
    /// Must be implemented in each derived class
    ///
    virtual ScalarT getMaxX() const = 0;

protected:
    ///
    /// \brief y is the vector of values of the TS
    ///
    std::vector<ScalarT> y_;

};



} //end namespace spc
#endif // GENERICTIMESERIES_H
