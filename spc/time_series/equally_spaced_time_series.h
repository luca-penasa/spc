#ifndef SPC_EQUALLY_SPACED_TIME_SERIES_H
#define SPC_EQUALLY_SPACED_TIME_SERIES_H

#include <spc/time_series/base_time_series.h>
#include <algorithm>
#include <limits>

namespace spc
{

using namespace std;
template <typename ScalarT = float >
///
/// \brief The EquallySpacedTimeSeries class is made up of a set of ordered y values equally spaced.
///
/// X positions are specified using a x_start and an x_step attribute you can setup
/// using setXStart() and setXStep(). x_start is the x coordinate for the first sample of y
/// other positions may be explicitely computed using getX() method with the formula
/// \f[
/// x[i] = x\_start + x\_step * i
/// \f]
/// \ingroup time_series
///
class EquallySpacedTimeSeries: public GenericTimeSeries<ScalarT>
{
    using GenericTimeSeries<ScalarT>::m_name;
    using GenericTimeSeries<ScalarT>::y;

public:



    ///
    /// \brief EquallySpacedTimeSeries default constructor
    ///
    EquallySpacedTimeSeries();

    ///
    /// \brief EquallySpacedTimeSeries copy const
    /// \param other
    ///
    EquallySpacedTimeSeries(const EquallySpacedTimeSeries &other);


    ///
    /// \brief EquallySpacedTimeSeries constructor with args
    /// \param y_ the data vector
    /// \param x_step_ x sampling step
    /// \param x_start_ x position of the first sample
    ///
    EquallySpacedTimeSeries(vector <ScalarT> y_, ScalarT x_step_ = 1.0, ScalarT x_start_ = 0.0);

    ///
    /// \brief EquallySpacedTimeSeries constructors, initializing as vector of nans
    /// \param x_step_ sampling step
    /// \param x_start_ x position of the first sample of y
    /// \param size number of samples reuired.
    ///
    EquallySpacedTimeSeries(ScalarT x_step_, ScalarT x_start_, size_t size);

    ///
    /// \brief EquallySpacedTimeSeries constructor, intialize a series filled with nans
    /// \param x_min_ min x position
    /// \param x_max_ max x position
    /// \param step_ requested sampling step
    ///
    EquallySpacedTimeSeries(ScalarT x_min_, ScalarT x_max_, ScalarT step_);

    template<typename NScalarT>
    EquallySpacedTimeSeries( const EquallySpacedTimeSeries<NScalarT> &other)
    {
        this->y.clear();
        std::vector<NScalarT> other_y = other.getY();
        std::vector<ScalarT> this_y (other_y.begin(), other_y.end());
        this->setY(this_y);
        this->x_start = (ScalarT) other.getXStart();
        this->x_step = (ScalarT) other.getXStep();
    }

    virtual std::string getSPCClassName()
    {
        std::string name = "EquallySpacedTimeSeries";
        return name;


    }

    ///
    /// \brief getXStart
    /// \return the x position of the first sample
    ///
    ScalarT getXStart() const {return x_start;}

    ///
    /// \brief getXStep
    /// \return the sampling step of the time series
    ///
    ScalarT getXStep() const {return x_step;}

    ///
    /// \brief getX
    /// \return a vector of the x positions
    ///
    virtual auto getX() const -> vector<ScalarT>;

    ///
    /// \brief resize the y vector to the given size
    /// If longer it will be truncated
    /// \param size_ number of samples
    ///
    void resize(size_t size_) { y.resize(size_); }

    ///
    /// \brief fill the time series with a value
    /// \param value_ the const value to use for y-filling
    ///
    void fill(const ScalarT value_ = std::numeric_limits<ScalarT>::quiet_NaN()) {std::fill(y.begin(), y.end(), value_);}

    ///
    /// \brief setXStep set sampling step
    /// \param step sampling step
    ///
    void setXStep(ScalarT step) {x_step = step;}

    ///
    /// \brief setXStart set the x position of the first y sample
    /// \param start the x position
    ///
    void setXStart(ScalarT start) {x_start = start;}

    virtual auto getMinX() -> ScalarT const {return x_start;}

    virtual auto getMaxX() -> ScalarT const {return x_start + x_step * this->getNumberOfSamples();}



private:
    ///
    /// \brief x_start holds the x position of first sampl
    ///
    ScalarT x_start;

    ///
    /// \brief x_step holds the sampling step of the time series
    ///
    ScalarT x_step;
};



}//end spc namespace

#endif // EQUALLYSPACEDTIMESERIES_H
