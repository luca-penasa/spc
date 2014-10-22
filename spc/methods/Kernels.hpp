#ifndef KERNELS_HPP
#define KERNELS_HPP

#include <spc/elements/macros_ptr.h>
#include <Eigen/Core>
namespace spc {

enum KERNELS {KERNEL_GAUSSIAN};

/** most kernels use a sigma parameter controlling the kernel' s size
 * this parameter is often used squared so we keep it (also) squared.
 * for kernel not using it just leave it uninitialized using the def-constructor of the
 * class.
 **/
template <typename T>
class BasicKernel
{
public:

    spcTypedefSharedPtrs(BasicKernel<T>)

    BasicKernel(): sigma_(0), sigma_squared_(0)
    {

    }

    BasicKernel(const T & sigma)
    {
        setSigma(sigma);
    }


    void setSigma(const T & sigma)
    {
        sigma_squared_ = sigma * sigma;
        sigma_ = sigma;
    }

    T getSigma() const
    {
        return sigma_;
    }

    virtual T operator ()(const T& squared_x) const = 0;


protected:
    T sigma_squared_ ;
    T sigma_;
};



template <typename T>
class GaussianKernel: public BasicKernel<T>
{
public:

    using BasicKernel<T>::sigma_squared_;

    GaussianKernel (const T& sigma): BasicKernel<T>(sigma) {}

    // BasicKernel interface
    virtual T operator ()(const T &squared_x) const
    {
        return exp(-(squared_x / sigma_squared_));
    }
};


}
#endif // KERNELS_HPP
