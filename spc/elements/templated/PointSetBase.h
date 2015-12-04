#ifndef SIMPLEPOINTSET_H
#define SIMPLEPOINTSET_H
#include <spc/core/spc_eigen.h>
#include <spc/core/nanoflann_adapters.hpp>

#include <nanoflann.hpp>

namespace spc
{

template<typename ScalarT = float, size_t DIM = 3>
struct BoundingBox
{
    typedef Eigen::Matrix<ScalarT, DIM,1> PointT;

    PointT min_;
    PointT max_;
};


template<typename ScalarT = float, size_t DIM = 3>
class PointSetBase
{
public:
    typedef size_t IndexT;
    typedef Eigen::Matrix<ScalarT, DIM, 1> PointT;
    typedef Eigen::Matrix<ScalarT, -1, DIM> MatrixT;

    typedef Eigen::Transform<ScalarT, PointT::RowsAtCompileTime, Eigen::Affine, Eigen::AutoAlign> TransformT;

    typedef PointSetBase<ScalarT, DIM> self_t;

    typedef BoundingBox<ScalarT, DIM> BBT;

// this stuff may go in a dedicate class in a future
    typedef NanoFlannEigenMatrixAdaptor<MatrixT> SearcherT;
//    typedef spcSharedPtrMacro<SearcherT> SearcherPtrT;
//    typedef spcSharedPtrMacro<const SearcherT> SearcherConstPtrT;


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

    virtual void resize(const IndexT size) = 0;


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
        DLOG(INFO) << "getting centroid with n points: " << this->getNumberOfPoints();
        for (size_t i = 0 ; i < getNumberOfPoints(); ++i)
            c += this->getPoint(i);

        DLOG(INFO) << "Sum is " << c.transpose();

        c /= getNumberOfPoints();
        return c;
    }

    virtual BBT getBB() const
    {
          BBT out;
          if (this->getNumberOfPoints() >= 1)
          {
              out.min_ = this->asEigenMatrix().colwise().minCoeff();
              out.max_ = this->asEigenMatrix().colwise().maxCoeff();
          }

          else
          {
              out.min_.fill(std::numeric_limits<ScalarT>::quiet_NaN());
              out.max_.fill(std::numeric_limits<ScalarT>::quiet_NaN());
          }
          return out;
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
        PointT avg;
        Eigen::Matrix<ScalarT, DIM, DIM> covmat = this->asEigenMatrix().getSampleCovMatAndAvg(avg);

        Eigen::Hyperplane<ScalarT, DIM> plane;

        Eigen::SelfAdjointEigenSolver<Eigen::Matrix<ScalarT, DIM, DIM>> eig(covmat);
        plane.normal() = eig.eigenvectors().col(0);
        plane.offset() = - plane.normal().dot(avg);

        return plane;
    }

    void updateSearcher()
    {
        DLOG(INFO) << "updating the searcher";
        searcher_ = typename SearcherT::Ptr (new SearcherT(this->asEigenMatrix(), 50));
        DLOG(INFO) << "done";
    }

    typename SearcherT::ConstPtr getSearcher()
    {
        if (searcher_ == NULL)
            updateSearcher();

        return searcher_;
    }



protected:
    typename SearcherT::Ptr searcher_;

};



typedef PointSetBase<float, 3> PointCloudXYZBase;


}
#endif // SIMPLEPOINTSET_H
