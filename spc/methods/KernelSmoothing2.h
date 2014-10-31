#ifndef VOMBAT_KERNELSMOOTHING2_H
#define VOMBAT_KERNELSMOOTHING2_H

#include <spc/elements/TimeSeriesSparse.h>
#include <spc/elements/TimeSeriesEquallySpaced.h>


#include <submodules/nanoflann/include/nanoflann.hpp>

#include <pcl/console/print.h>
#include <boost/foreach.hpp>

#include <spc/elements/Kernels.hpp>

namespace spc
{

/**
 * KernelSmoothing performs kernel smoothing over a N-dimensional space.
 *  notice we dont copy the input data. so you nay cause problems if
 * you delete the data and try to call operator ()
 */
template <typename ScalarT>
class KernelSmoothing
{
public:
    typedef std::pair<size_t, ScalarT> MatchT;
    typedef std::vector<MatchT> MatchSetT;

    typedef Eigen::Matrix<ScalarT, -1, 1> VectorT;


    typedef Eigen::Matrix<ScalarT, -1, -1> MatrixT;


    typedef nanoflann::KDTreeEigenMatrixAdaptor<Eigen::MatrixXf> NanoFlannIndexT;

    KernelSmoothing(): kernel_(new GaussianRBF<ScalarT>),
        points_(NULL),
        values_(NULL)
    {
    }

    void setInputPoints(const MatrixT & points)
    {
        points_ = &points;
    }

    void setValues(const VectorT & values)
    {
        values_ = &values;
    }

    //! you can chose between spc kernels
    void setKernel(const RBF_FUNCTION ker)
    {
        kernel_ = kernel_from_enum<ScalarT>(ker);
    }

    //! is the bandwith of the estimator
    void setKernelSigma(const ScalarT sigma)
    {
        kernel_->setSigma(sigma);
    }

    typename BasicRadialBasisFunction<ScalarT>::Ptr getKernel() const
    {
        return kernel_;
    }


    /**
     * @brief operator () is a batch operator, parallelized over openmp
     * @param [in] eval_points
     * @param [out] outvector
     */
    void operator ()(const MatrixT &eval_points, VectorT &outvector)
    {
        outvector.resize(eval_points.rows());
        if (!initFlann())
        {
            LOG(WARNING) << "Problem initializing flann. returning a vector of nans";
            outvector.fill(std::numeric_limits<ScalarT>::quiet_NaN());
        }

#ifdef USE_OPENMP
#pragma omp parallel for
#endif
        for (int i = 0; i < eval_points.rows(); ++i)
            outvector(i) = single_eval(eval_points.row(i));
    }

//    /**
//     * @brief operator () is a single-call oprator. If you need to do multiple calls
//     * please use the batch operator.
//     * @param eval_point
//     * @return
//     */
//    ScalarT
//    operator ()(const VectorT &eval_point)
//    {
//        if (!initFlann())
//        {
//            LOG(WARNING) << "Problem initializing flann. check log";
//            return std::numeric_limits<ScalarT>::quiet_NaN();
//        }

//        return single_eval(eval_point);
//    }




protected:
    int initFlann()
    {
        if (nanoflann_index_)
        {
            LOG(WARNING) << "Flann was yet initialized. We are going to do a new init";
        }

        if (!points_)
        {
            LOG(ERROR) << "Please provide points";
            return -1;
        }

        if (!values_)
        {
            LOG(ERROR) << "Please provide values";
            return -1;
        }


        if (points_->rows() != values_->rows())
        {
            LOG(ERROR) << "points and values must have the same number of rows";
            return -1;
        }

        nanoflann_index_ = spcSharedPtrMacro<NanoFlannIndexT> (new NanoFlannIndexT(points_->cols(), *points_, 10));
        nanoflann_index_->index->buildIndex();


        if(nanoflann_index_)
            return 1;
        else
            return -1; // just to be sure

    }


    int radiusSearch(const VectorT &position, const ScalarT &sq_radius,
                     MatchSetT &matches) const
    {
        nanoflann::SearchParams params;
        size_t nMatches = nanoflann_index_->index->radiusSearch(position.data(), sq_radius, matches, params);

        DLOG(INFO) << "Found " << nMatches << "neighbors";
        return nMatches;
    }

    ScalarT
    single_eval (const VectorT &eval_point) const
    {

        MatchSetT matches;
        this->radiusSearch(eval_point, kernel_->getSupportRegionSquared(),  matches);

        if (matches.size() == 0)
            return std::numeric_limits<ScalarT>::quiet_NaN();

        ScalarT sum = 0;
        ScalarT val = 0;
        for (MatchT &m: matches)
        {
            m.second = kernel_->operator() (m.second);
            sum += m.second;
            val += values_->operator ()(m.first) * m.second;
        }

        return val / sum;
    }




    // flann index
    spcSharedPtrMacro<NanoFlannIndexT> nanoflann_index_ ;

    ScalarT kernel_raidius_; /**< AKA the bandwidth of the estimator */

    const MatrixT * points_;
    const VectorT * values_;

    typename BasicRadialBasisFunction<ScalarT>::Ptr kernel_;
};






} // end nspace

#endif // KERNELSMOOTHING2_H
