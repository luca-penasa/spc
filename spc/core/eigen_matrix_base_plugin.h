
//! yes we add iterators! straight from mrpt project
typedef Scalar* iterator;
typedef const Scalar* const_iterator;

EIGEN_STRONG_INLINE iterator begin() { return derived().data(); }
EIGEN_STRONG_INLINE iterator end() { return (&(derived().data()[size()-1]))+1; }
EIGEN_STRONG_INLINE const_iterator begin() const { return derived().data(); }
EIGEN_STRONG_INLINE const_iterator end() const { return (&(derived().data()[size()-1]))+1; }

//! end iterators

EIGEN_STRONG_INLINE size_t maxID() const
{
    size_t id;
    derived().maxCoeff(id);
    return id;
}


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

//EIGEN_STRONG_INLINE
//Eigen::Matrix<Scalar, -1, 1>
//nonNaNCoeffs()
//{

//    size_t nz = ((*this).array() != std::numeric_limits<Scalar>::quiet_NaN()).count()
//    Eigen::Matrix<Scalar, -1, 1> outv(nz);
//    size_t counter = 0;
//    for (int i = 0; i < size(); ++i)
//    {
//        Scalar v= this->array()(i);
//        if (v != 0.0 )
//            outv(counter ++) = v;
//    }

//    return outv;

//}

//template<typename OtherDerived>
EIGEN_STRONG_INLINE
Eigen::Matrix<bool, RowsAtCompileTime, ColsAtCompileTime>
finiteness() const
{
    return unaryExpr([](const float &x){return std::isfinite(x);}).template cast<bool>();
}


EIGEN_STRONG_INLINE
void push_back(Scalar val)
{
    const Index N = size();
    derived().conservativeResize(N+1);
    coeffRef(N) = val;
}

EIGEN_STRONG_INLINE
Eigen::Matrix<Scalar, -1, 1>
only_finites() const
{
    Eigen::Matrix<bool, RowsAtCompileTime, ColsAtCompileTime> bools = finiteness();

    return select(bools);
}


template<typename OtherDerived>
Eigen::Matrix<Scalar, -1, 1>
select(const OtherDerived & selection_matrix) const
{
    DCHECK(selection_matrix.rows() == rows() && selection_matrix.cols() == cols());

    size_t n = selection_matrix.template cast<int>().array().sum();
    Eigen::Matrix<Scalar, -1, 1> out(n);

    size_t counter = 0;
    for (int i = 0 ; i < size(); ++i)
    {
        if(selection_matrix(i))
            out(counter++, 0) = array()(i);
    }
    return out;
}

/** Remove rows of the matrix. The unsafe version assumes that, the indices are sorted in ascending order. */
EIGEN_STRONG_INLINE void unsafeRemoveRows(const std::vector<size_t> &idxs)
{
    size_t k = 1;
    for (std::vector<size_t>::reverse_iterator it = idxs.rbegin(); it != idxs.rend(); ++it, ++k)
    {
        const size_t nR = rows() - *it - k;
        if( nR > 0 )
            derived().block(*it,0,nR,cols()) = derived().block(*it+1,0,nR,cols()).eval();
    }
    derived().conservativeResize(rows()-idxs.size(),NoChange);
}


/** Remove rows of the matrix. */
 EIGEN_STRONG_INLINE void removeRows(const std::vector<size_t> &idxsToRemove)
  {
  std::vector<size_t> idxs = idxsToRemove;
  std::sort( idxs.begin(), idxs.end() );
  std::vector<size_t>::iterator itEnd = std::unique( idxs.begin(), idxs.end() );
  idxs.resize( itEnd - idxs.begin() );

  unsafeRemoveRows( idxs );
  }

Eigen::Matrix<Scalar, -1, 1> unique() const
{
    Derived tmp = derived();

    std::sort(tmp.data(), tmp.data() + tmp.size());
    auto last = std::unique(tmp.data(), tmp.data() + tmp.size());

    size_t n_elements = last - tmp.data();

    Eigen::Matrix<Scalar, -1, 1> out;

    //! flatten it
    Eigen::Map<Eigen::Matrix<Scalar, -1, 1>> v(tmp.data(),tmp.size());
    out =  v.head(n_elements);
    return out;

}



Eigen::Matrix<Scalar, ColsAtCompileTime, ColsAtCompileTime>
getSampleCovMatAndAvg(Eigen::Matrix<Scalar, ColsAtCompileTime, 1> &avg) const
{
    typedef Scalar ScalarT;
    //    typedef Eigen::Matrix<typename MatrixT::Scalar, MatrixT::ColsAtCompileTime, 1> VectorT;
    typedef Eigen::Matrix<ScalarT, ColsAtCompileTime, ColsAtCompileTime> CovMatT;

    avg = derived().colwise().mean();
    auto centered = derived().rowwise() - avg.transpose();
    CovMatT cov = (centered.adjoint() * centered) / ScalarT(derived().rows() - 1);
    return cov;
}



// Eigen::Hyperplane<Scalar, ColsAtCompileTime>
// fitHyperplane()
// {
//     typedef Scalar ScalarT;
//     typedef Eigen::Hyperplane<ScalarT, ColsAtCompileTime> planeT;
//     typedef Eigen::Matrix<ScalarT, ColsAtCompileTime, ColsAtCompileTime> CovMatT;
//     typedef typename Eigen::Matrix<ScalarT, ColsAtCompileTime, 1> VectorT;

//     VectorT avg;
//     CovMatT covmat = derived().getSampleCovMatAndAvg(avg);

//     planeT plane;

//     SelfAdjointEigenSolver<CovMatT> eig(covmat);
//     plane.normal() = eig.eigenvectors().col(0);
//     plane.offset() = - plane.normal().dot(avg);

//     return plane;
// }




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



