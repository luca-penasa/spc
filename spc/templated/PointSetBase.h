#ifndef SIMPLEPOINTSET_H
#define SIMPLEPOINTSET_H
#include <spc/core/spc_eigen.h>

namespace spc
{

template<typename ScalarT = float, size_t DIM = 3>
class PointSetBase
{
public:
    typedef size_t IndexT;
    typedef Eigen::Matrix<ScalarT, DIM, 1> PointT;

    typedef Eigen::Transform<ScalarT, PointT::RowsAtCompileTime, Eigen::Affine, Eigen::AutoAlign> TransformT;

    typedef PointSetBase<ScalarT, DIM> self_t;

    spcTypedefSharedPtrs(self_t)


    //! def const
    PointSetBase() {}

    //! copy const
    PointSetBase(const self_t & other)
    {
        //! no need to do anything
    }

    ~PointSetBase()
    {
        // no need to d anything
    }


    virtual PointT getPoint(const IndexT id) const  = 0;

    virtual void setPoint(const IndexT id, const PointT &p) = 0;

    virtual void addPoint(const PointT & p) = 0;

    virtual IndexT getNumberOfPoints() const = 0;

    virtual void resize(IndexT size) = 0;


    virtual Eigen::Matrix<ScalarT, -1, DIM> asEigenMatrix() const
    {
        DLOG(WARNING) << "creating an eigen matrix copying the data. This means the method was not reimplemented in derived classes";
        Eigen::Matrix<ScalarT, -1, DIM> mat;
        mat.resize(getNumberOfPoints(), DIM);
        for (int i = 0; i < getNumberOfPoints(); ++i)
             mat.row(i)=this->getPoint(i);

             return mat;
    }


    virtual PointT getCentroid() const
    {
        PointT c = PointT::Zero();
        for (size_t i = 0 ; i < getNumberOfPoints(); ++i)
            c += this->getPoint(i);

        c /= getNumberOfPoints();
        return c;
    }

    template<typename OutPointSet> OutPointSet
    transform(const TransformT &T) const
    {
        OutPointSet outset;
        outset.resize(getNumberOfPoints());
        for (IndexT i = 0; i < getNumberOfPoints(); ++i)
        {
            outset.setPoint(i, T * this->getPoint(i));
        }

        return outset;
    }



    Eigen::Hyperplane<ScalarT, DIM>
    fitHyperplane() const
    {
//        typedef typename MatrixT::Scalar ScalarT;
//        typedef Eigen::Hyperplane<typename MatrixT::Scalar, MatrixT::ColsAtCompileTime> planeT;
//        typedef Eigen::Matrix<ScalarT, MatrixT::ColsAtCompileTime, MatrixT::ColsAtCompileTime> CovMatT;
//        typedef typename Eigen::Matrix<ScalarT, MatrixT::ColsAtCompileTime, 1> VectorT;

        PointT avg;
        Eigen::Matrix<ScalarT, DIM, DIM> covmat = this->asEigenMatrix().getSampleCovMatAndAvg(avg);

        Eigen::Hyperplane<ScalarT, DIM> plane;

        Eigen::SelfAdjointEigenSolver<Eigen::Matrix<ScalarT, DIM, DIM>> eig(covmat);
        plane.normal() = eig.eigenvectors().col(0);
        plane.offset() = - plane.normal().dot(avg);

        return plane;
    }
};



typedef PointSetBase<float, 3> PointCloudXYZBase;


}
#endif // SIMPLEPOINTSET_H
