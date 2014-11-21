#ifndef VOMBAT_KERNELSMOOTHING2_H
#define VOMBAT_KERNELSMOOTHING2_H

#include <spc/elements/TimeSeriesSparse.h>
#include <spc/elements/TimeSeriesEquallySpaced.h>
#include <spc/core/nanoflann_adapters.hpp>

#include <nanoflann.hpp>

#include <pcl/console/print.h>
#include <boost/foreach.hpp>

#include <spc/elements/RBFKernelFactory.h>

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


    typedef NanoFlannEigenMatrixAdaptor<Eigen::MatrixXf> NanoFlannIndexT;

    KernelSmoothing(const Eigen::Ref<const MatrixT> & points,
                    const Eigen::Ref<const VectorT> & values): kernel_(new EpanechnikovRBF<ScalarT>),
        points_(points), values_(values)
    {
        init();
    }

    //! you can chose between spc rbf-kernels
    void setKernel(const typename RBFKernelFactory<ScalarT>::RBF_FUNCTION ker)
    {
        kernel_ = RBFKernelFactory<ScalarT>::create(ker);
    }

    //! is the bandwith of the estimator
    void setKernelSigma(const ScalarT sigma)
    {
        kernel_->setScale(sigma);
    }

    typename RBFBase<ScalarT>::Ptr getKernel() const
    {
        return kernel_;
    }


    /**
     * @brief operator () is a batch operator, parallelized over openmp
     * @param [in] eval_points
     * @param [out] outvector
     */
    int operator ()(const Eigen::Ref<const MatrixT> &eval_points, VectorT & outvector)
    {
        if (eval_points.cols() != points_.cols())
        {
            LOG(ERROR) << "eval_points must have the same dimensionality of the points_";
            return -1;
        }

        outvector.resize(eval_points.rows());
        LOG(INFO) << "going to compute kernel smoothing";
#ifdef USE_OPENMP
#pragma omp parallel for
#endif
        for (int i = 0; i < eval_points.rows(); ++i)
            outvector(i) = single_eval(eval_points.row(i));
        LOG(INFO) << "going to compute kernel smoothing. Done";
        return 1;
    }



protected:
    int init()
    {
        if (index_)
        {
            LOG(WARNING) << "Flann was yet initialized. We are going to do a new init";
        }

        if (points_.rows() != values_.rows())
        {
            LOG(ERROR) << "points and values must have the same number of rows";
            return -1;
        }

        if (!kernel_)
        {
            LOG(ERROR) << "kernel not set";
            return -1;
        }


        // extract the support region

        if (kernel_->isCompact())
            search_support_ = kernel_->getSupport();
        else
            search_support_ = kernel_->getScale() * noncompact_support_multiplier_;

        search_support_squared_ = search_support_ * search_support_;

        index_ = NanoFlannIndexT::Ptr (new NanoFlannIndexT(points_, 10));

        if(index_)
            return 1;
        else
            return -1; // just to be sure
    }



    ScalarT
    single_eval (const VectorT &eval_point) const
    {

        nanoflann::SearchParams pars;
        pars.sorted = false; // we get some speed-up not sorting them maybe
        //        LOG(INFO) << "eval point " << eval_point.transpose();
        MatchSetT matches;
        index_->radiusSearch(eval_point, search_support_squared_,  matches, pars);

        //        LOG(INFO) << "N matches "<< matches.size();

        if (matches.size() == 0)
            return std::numeric_limits<ScalarT>::quiet_NaN();

        ScalarT sum = 0;
        ScalarT val = 0;
        for (MatchT &m: matches)
        {
            m.second = kernel_->eval(m.second);
            sum += m.second;
            val += values_(m.first) * m.second;
        }

        return val / sum;
    }




    // flann index
    typename NanoFlannIndexT::Ptr index_;
    //    ScalarT kernel_raidius_; /**< AKA the bandwidth of the estimator */

    const Eigen::Ref<const MatrixT>  points_;
    const Eigen::Ref<const VectorT> values_;

    typename RBFBase<ScalarT>::Ptr kernel_;


    ScalarT search_support_;
    ScalarT search_support_squared_;

    ScalarT noncompact_support_multiplier_ = 4;
};






} // end nspace

#endif // KERNELSMOOTHING2_H
