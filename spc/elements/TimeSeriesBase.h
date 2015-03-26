#ifndef SPC_BASE_TIME_SERIES_H
#define SPC_BASE_TIME_SERIES_H


#include <spc/elements/ElementBase.h>
#include <cereal/types/vector.hpp>
#include <spc/elements/EigenTable.h>

namespace spc
{


	class SPC_LIB_API TimeSeriesBase : public ElementBase
{
public:
    spcTypedefSharedPtrs(TimeSeriesBase)
    EXPOSE_TYPE
    TimeSeriesBase()
    {
    }
    typedef float ScalarT;
    typedef Eigen::Matrix<ScalarT, -1, 1> VectorT;


    void fill(const ScalarT value_ = std::numeric_limits<ScalarT>::quiet_NaN());

    /// by default will generate in -1 to 1 range
    // void fillRandomY(ScalarT min=-1, ScalarT max=1);

    ///
    /// \brief getNumberOfSamples in time series
    /// \return the number of samples
    ///

    size_t getNumberOfSamples() const
    {
        return y_.size();
    }

    ///
    /// \brief getX positions
    /// \return a vector of x positions
    ///
    virtual VectorT getX() const = 0;

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

    template<class VEC>
    void getY(VEC & out) const
    {
        out.resize(this->getNumberOfSamples());
#ifdef USE_OPENMP
#pragma omp parallel for
#endif
        for (int i = 0 ; i < this->getNumberOfSamples(); ++i)
        {
            out[i] = y_(i);
        }
    }

    template<class VEC>
    void getX(VEC & out) const
    {
        VectorT x = this->getX();

        out.resize(this->getNumberOfSamples());
#ifdef USE_OPENMP
#pragma omp parallel for
#endif
        for (int i = 0 ; i < this->getNumberOfSamples(); ++i)
        {
            out[i] = (typename VEC::value_type) x(i);
        }
    }

    ScalarT &getY(size_t id)
    {
        return y_(id);
    }

    ///
    /// \brief setY values
    ///
    void setY(VectorT y)
    {
        y_ = y;
    }

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
    virtual ScalarT getMinX() const = 0;

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
    VectorT y_;

private:
    friend class cereal::access;

    template <class Archive> void serialize(Archive &ar)
    {
        ar(cereal::base_class<spc::ElementBase>(this), CEREAL_NVP(y_));
    }

    // ElementBase interface

    // ISerializable interface
public:
    virtual bool isAsciiSerializable() const
    {
        return true;
    }

    // ISerializable interface
public:
    virtual EigenTable::Ptr asEigenTable() const;
};


} // end namespace spc
#endif // GENERICTIMESERIES_H
