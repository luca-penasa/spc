#ifndef POINTSET_H
#define POINTSET_H

#include <spc/templated/PointSetBase.h>

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
        return data_.size();
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



    virtual void resize(IndexT size)
    {
        data_.conservativeResize(size, DIM);
    }




protected:
    Eigen::Matrix<ScalarT, -1, DIM> data_;


    // PointSetBase interface
public:
    virtual Eigen::Matrix<ScalarT, -1, DIM> asEigenMatrix() const override
    {
        return data_;
    }
};

}

#endif // POINTSET_H
