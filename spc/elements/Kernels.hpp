#ifndef KERNELS_HPP
#define KERNELS_HPP

#include <spc/elements/ElementBase.h>
#include <spc/core/spc_eigen.h>
namespace spc {

enum RBF_FUNCTION {RBF_GAUSSIAN};

/** most rbf functinos use a kernel radius parameter controlling the size
 * this parameter is often used squared so we keep it (also) squared.
 * for kernel not using it just leave it uninitialized using the def-constructor of the
 * class.
 *
 * \note derived classes MUST implement the evalSquared function only. Most of the
 * rbf actually works with squared distances so its ok.
 *
 **/

template <typename T>
class BasicRadialBasisFunction: public ElementBase
{
public:
    SPC_OBJECT(BasicRadialBasisFunction<T>)

    /** the scale is acually the scale of the basis fuction
     * bigger scale -> bigger basis function
     **/
    BasicRadialBasisFunction(const T & scale = 1): scale_(scale)
    {
        updateSecondary();
    }

    /** argument is squared!
     * in fact most rbf use a squared distance measure in the formula
     **/
    virtual T eval(const T& squared_x) const = 0;


    /** is important to know what the support for this RBF is
     * by default is infinity. So please implement it in derived classes
     * the returned valued should be the squared distance at which the RBF becomes
     * zero. For example for a gaussian rbf its infinity.
     *
     */
    virtual T support() const
    {
        return std::numeric_limits<T>::infinity();
    }

    void updateSecondary()
    {
        scale_inv_ = 1 / scale_;
        scale_squared_ = scale_ * scale_;
        scale_squared_inv = 1 / scale_squared_;
    }

protected:

    T scale_ = 1;


    /** we call it scale
    http://en.wikipedia.org/wiki/Radial_basis_function
    (and http://en.wikipedia.org/wiki/Kernel_smoother)

    these are ancillary variables used for efficiency
    **/
    T scale_inv_ ;
    T scale_squared_;
    T scale_squared_inv;

private:
    friend class cereal::access;

    template <class Archive> void serialize(Archive &ar)
    {
        ar(cereal::base_class<ElementBase>(this),
           CEREAL_NVP(scale_),
           CEREAL_NVP(scale_inv_),
           CEREAL_NVP(scale_squared_),
           CEREAL_NVP(scale_squared_inv));
    }
};



template <typename T>
class GaussianRBF: public BasicRadialBasisFunction<T>
{
public:

    using BasicRadialBasisFunction<T>::scale_squared_inv;

    GaussianRBF (const T& sigma): BasicRadialBasisFunction<T>(sigma) {}

    GaussianRBF (): BasicRadialBasisFunction<T>() {}

    // BasicKernel interface
    virtual inline T eval(const T &squared_x) const
    {
        return exp(-(squared_x * scale_squared_inv));
    }

    virtual void setSigma(const T &sigma)
    {
        BasicRadialBasisFunction<T>::setSigma(sigma);
        support_region_ = sigma * 4;
        support_region_squared_ = support_region_ * support_region_;
    }

private:
    friend class cereal::access;

    template <class Archive> void serialize(Archive &ar)
    {
        ar(cereal::base_class<BasicRadialBasisFunction<T> >(this));
    }

};


template<typename ScalarT>
typename BasicRadialBasisFunction<ScalarT>::Ptr kernel_from_enum(const RBF_FUNCTION &kernel)
{
    if (kernel == RBF_GAUSSIAN)
        return BasicRadialBasisFunction<ScalarT>::Ptr(new GaussianRBF<ScalarT>);
    else
    {
        LOG(WARNING) << "Requested kernel not found";
        return NULL;
    }

}





} // end nspace






#endif // KERNELS_HPP
