#pragma once
#ifndef POINTSET_H
#define POINTSET_H

#include <spc/elements/templated/PointSetBase.h>
#include <spc/core/spc_cereal.hpp>
namespace spc
{

template<typename ScalarT = float, size_t DIM = 3>
class PointSet: public PointSetBase<ScalarT, DIM>
{
public:

    using typename PointSetBase<ScalarT, DIM>::IndexT;
    using typename PointSetBase<ScalarT, DIM>::PointT;

    typedef PointSet<ScalarT, DIM> self_t;

    //! copy const
    PointSet( const self_t & other) : PointSetBase<ScalarT, DIM>(other)
    {
        // copy the data
        data_ = other.data_;
    }

    PointSet(const PointSetBase<ScalarT, DIM> & other)
    {
        this->resize(other.getNumberOfPoints());
        for (int i = 0; i < other.getNumberOfPoints(); ++i)
        {
            this->setPoint(i, other.getPoint(i));
        }
    }

    //!def const
    PointSet()
    {

    }

    ~PointSet()
    {
        // nothing to do
    }

public:
    virtual PointT getPoint(const IndexT id) const override
    {
        return data_.row(id);
    }

    virtual IndexT getNumberOfPoints() const override
    {
        return data_.rows();
    }



    virtual void addPoint(const PointT &p) override
    {

        this->resize(getNumberOfPoints() + 1);
        data_.row(data_.rows() - 1) = p; // append it
    }


    virtual void setPoint(const IndexT id, const PointT &p) override
    {
        data_.row(id) = p;
    }

    virtual void resize(const IndexT size) override
    {
        data_.conservativeResize(size, DIM);
    }

    virtual PointSet operator +(const PointSet &other) const
    {
        PointSet out(*this);
        out.resize(out.getNumberOfPoints() + other.getNumberOfPoints());
        out.data_.bottomRows(out.getNumberOfPoints() - this->getNumberOfPoints()) = other.data_;
        return out;
    }


    virtual void operator += (const PointSet &other)
    {
        size_t old_size = this->getNumberOfPoints();
        this->resize(old_size + other.getNumberOfPoints());

        data_.bottomRows(this->getNumberOfPoints() - old_size) = other.data_;
    }

protected:
    Eigen::Matrix<ScalarT, -1, DIM> data_;

    // PointSetBase interface
public:
    virtual Eigen::Matrix<ScalarT, -1, DIM> asEigenMatrix() const override
    {
        return data_;
    }

private:
    friend class cereal::access;

    template <class Archive> void serialize(Archive &ar, const std::uint32_t version)
    {
        ar(data_);
    }
};

}

#endif // POINTSET_H
