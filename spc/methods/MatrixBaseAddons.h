

double mean() const
{
    if ( size()==0) throw std::runtime_error("mean: Empty container.");
    return derived().sum()/static_cast<double>(size());
}

 bool empty() const { return this->size() == 0; }

/** Computes a row with the mean values of each column in the matrix and the associated vector with the standard deviation of each column.
 * \sa mean,meanAndStdAll \exception std::exception If the matrix/vector is empty.
  * \param unbiased_variance Standard deviation is sum(vals-mean)/K, with K=N-1 or N for unbiased_variance=true or false, respectively.
  */
template <class VEC>
void meanAndStd(
        VEC &outMeanVector,
        VEC &outStdVector,
        const bool unbiased_variance = true ) const
{
    const size_t N = rows();
    const double N_inv = 1.0/N;
    const double N_ = unbiased_variance ? (N>1 ? 1.0/(N-1) : 1.0) : 1.0/N;

    outMeanVector.resize(cols());
    outStdVector.resize(cols());

    for (Index i=0;i<cols();i++)
    {
        outMeanVector(i)= this->col(i).sum() * N_inv;
        outStdVector(i) = std::sqrt( (this->col(i).array() - outMeanVector(i)).square().sum() * N_ );
    }

}

EIGEN_STRONG_INLINE
Transpose<Derived>
T()
{
    return this->transpose();
}
EIGEN_STRONG_INLINE
size_t nonZeroCount()
{
    return ((*this).array() != 0).count();
}

EIGEN_STRONG_INLINE
Eigen::Matrix<Scalar, -1, 1>
nonZeroCoeffs()
{
    size_t nz = this->nonZeroCount();
    Eigen::Matrix<Scalar, -1, 1> outv(nz);
    size_t counter = 0;
    for (int i = 0; i < size(); ++i)
    {
        Scalar v= this->array()(i);
        if (v != 0.0 )
            outv(counter ++) = v;
    }

    return outv;


}

//! compatibility with other vector implementations - std::vector, QVector etc

inline Scalar at(uint i, uint j) const
{
    return this->operator()(i,j);
}

inline Scalar& at(uint i, uint j)
{
    return this->operator()(i,j);
}

inline Scalar at(uint i) const
{
    return this->operator[](i);
}

inline Scalar& at(uint i)
{
    return this->operator[](i);

}
//inline RealScalar squaredLength() const { return squaredNorm(); }
//inline RealScalar length() const { return norm(); }
//inline RealScalar invLength(void) const { return fast_inv_sqrt(squaredNorm()); }


//template<typename OtherDerived>
//inline Scalar squaredDistanceTo(const MatrixBase<OtherDerived>& other) const
//{
//    return (derived() - other.derived()).squaredNorm();
//}


//inline void scaleTo(RealScalar l) { RealScalar vl = norm(); if (vl>1e-9) derived() *= (l/vl); }

//inline Transpose<Derived> transposed() {return this->transpose();}

//inline const Transpose<Derived> transposed() const {return this->transpose();}

//inline uint minComponentId(void) const { int i; this->minCoeff(&i); return i; }

//inline uint maxComponentId(void) const { int i; this->maxCoeff(&i); return i; }

//template<typename OtherDerived>
//void makeFloor(const MatrixBase<OtherDerived>& other) { derived() = derived().cwiseMin(other.derived()); }

//template<typename OtherDerived>
//void makeCeil(const MatrixBase<OtherDerived>& other) { derived() = derived().cwiseMax(other.derived()); }

//const CwiseUnaryOp<internal::scalar_add_op<Scalar>, Derived>
//operator+(const Scalar& scalar) const
//{ return CwiseUnaryOp<internal::scalar_add_op<Scalar>, Derived>(derived(), internal::scalar_add_op<Scalar>(scalar)); }


//friend const CwiseUnaryOp<internal::scalar_add_op<Scalar>, Derived>
//operator+(const Scalar& scalar, const MatrixBase<Derived>& mat)
//{ return CwiseUnaryOp<internal::scalar_add_op<Scalar>, Derived>(mat.derived(), internal::scalar_add_op<Scalar>(scalar)); }



