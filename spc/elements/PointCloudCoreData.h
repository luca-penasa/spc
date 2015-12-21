#pragma once

#ifndef SPC_POINTCLOUDCOREDATA_H
#define SPC_POINTCLOUDCOREDATA_H

#include <spc/core/spc_cereal.hpp>
#include <spc/core/spc_eigen.h>
#include <spc/elements/ElementBase.h>


namespace spc
{





class ScalarField: public ElementBase
{
public:

    spcTypedefSharedPtrs(ScalarField)
    spcAddClone(ScalarField)
//    EXPOSE_TYPE




};



template <typename ScalarT, int DIM>
class CoreFieldBase: public ScalarField
{
public:

    spcTypedefSharedPtrs(CoreFieldBase)


    typedef Eigen::Matrix<ScalarT, DIM, 1> VectorT;
    typedef Eigen::Matrix<ScalarT, -1, DIM, Eigen::RowMajor> MatrixT;

    typedef Eigen::Map<VectorT> VectorMapT;
    typedef Eigen::Map<MatrixT> MatrixMapT;

    virtual size_t size() const = 0;
    virtual void resize (const size_t & n) = 0;


    virtual size_t dim() const
    {
        return DIM;
    }



    virtual VectorMapT row(const size_t id)
    {
        return VectorMapT(data() + id*DIM, DIM);
    }

    MatrixMapT matrix()
    {
        return MatrixMapT(data(), size(), DIM);
    }

    virtual ScalarT * data() const = 0;


};



template<typename ScalarT, int DIM>
class CoreFieldEigen: public CoreFieldBase<ScalarT, DIM>
{
public:

    typedef typename CoreFieldBase<ScalarT, DIM>::MatrixT MatrixT;
    typedef typename CoreFieldBase<ScalarT, DIM>::VectorT VectorT;
    typedef typename CoreFieldBase<ScalarT, DIM>::VectorMapT VectorMapT;
    typedef typename CoreFieldBase<ScalarT, DIM>::MatrixMapT MatrixMapT;


    CoreFieldEigen()
    {        
    }

    // CoreFieldBase interface
    virtual size_t size() const override
    {
        return matrix_.rows();
    }

    virtual void resize(const size_t &n) override
    {
        matrix_.conservativeResize(n, Eigen::NoChange);
    }


    virtual ScalarT *data() const override
    {
        matrix_.data();
    }



protected:
    MatrixT matrix_;
};

template<typename ScalarT>
class CoreFieldEigen<ScalarT, 1>: public CoreFieldBase<ScalarT, 1>
{
public:

    typedef typename Eigen::Matrix<ScalarT, -1, 1> MatrixT;
    typedef typename Eigen::Map<MatrixT> MatrixMapT;

    CoreFieldEigen()
    {
    }

    // CoreFieldBase interface
    virtual size_t size() const override
    {
        return matrix_.rows();
    }

    virtual void resize(const size_t &n) override
    {
        matrix_.conservativeResize(n);
    }

    ScalarT & row(const size_t &id)
    {
        return *(data() + id);
    }

     MatrixMapT matrix()
     {
         return MatrixMapT(data(), size());
     }


    virtual ScalarT *data() const override
    {
        matrix_.data();
    }

protected:
   MatrixT matrix_;
};



}// end nspace


#endif // POINTCLOUDCOREDATA_H
