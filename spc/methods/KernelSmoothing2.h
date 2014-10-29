#ifndef VOMBAT_KERNELSMOOTHING2_H
#define VOMBAT_KERNELSMOOTHING2_H

#include <spc/elements/TimeSeriesSparse.h>
#include <spc/elements/TimeSeriesEquallySpaced.h>


#include <submodules/nanoflann/include/nanoflann.hpp>

#include <pcl/console/print.h>
#include <boost/foreach.hpp>

namespace spc
{
class KernelSmoothing2
{

public:

    typedef float ScalarT;
    typedef TimeSeriesSparse SparseT;
    typedef spcSharedPtrMacro<SparseT> SparsePtrT;

    typedef TimeSeriesEquallySpaced EquallyT;
    typedef spcSharedPtrMacro<EquallyT> EquallyPtrT;

typedef nanoflann::KDTreeEigenMatrixAdaptor<Eigen::MatrixXf> NanoFlannIndexT;

    KernelSmoothing2();

    void setInputSeries(SparsePtrT in)
    {
        sparse_ = in;
    }

    void setOutputSeriesBlank(EquallyPtrT out)
    {
        out_series_ = out;
    }

    void setStep(ScalarT step)
    {
        step_ = step;
    }

    void setBandwidth(ScalarT bandwidth)
    {
        bandwidth_ = bandwidth;
    }

    EquallyPtrT getOutputSeries() const
    {
        return out_series_;
    }

    int compute();

protected:
    void initFlann();

    void extractVectors();

    SparsePtrT sparse_;
    EquallyPtrT out_series_;



    ScalarT step_ = 1.0;
    ScalarT bandwidth_ = 1.0;

    std::vector<ScalarT> x_;
    std::vector<ScalarT> y_;

    spcSharedPtrMacro<NanoFlannIndexT> nanoflann_index_ ;

    // compute gaussian weights on a vector
    inline void gaussian(const std::vector<ScalarT> &values,
                         std::vector<ScalarT> &gaussian_values)
    {
        spcForEachMacro(float val, values)
        {
            gaussian_values.push_back(gaussian(val));
        }
    }

    // compute gaussian weights on a single numeric value
    inline ScalarT gaussian(const ScalarT &value)
    {
        return 1.0 / sqrt(2.0 * M_PI) * exp(-0.5 * value);
    }

//    void initKDTree();

    int radiusSearch(const ScalarT &position, const ScalarT &radius,
                     std::vector<std::pair<size_t, ScalarT> > &matches);

    int evaluateKS(const ScalarT &position, ScalarT &value);
};

} // end nspace

#endif // KERNELSMOOTHING2_H
