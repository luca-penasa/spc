#ifndef KERNELS_HPP
#define KERNELS_HPP

#include <spc/elements/ElementBase.h>
#include <spc/core/spc_eigen.h>
namespace spc {


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
class RBFBase: public ElementBase
{
public:
    SPC_OBJECT(RBFBase<T>)

    RBFBase(): scale_(1)
    {
        updateSecondary();
    }

    /** the scale is acually the scale of the basis fuction
     * bigger scale -> bigger basis function
     **/
    RBFBase(const T & scale): scale_(scale)
    {
        updateSecondary();
    }

    T getScale() const
    {
        return scale_;
    }

    /** by default support is infinite
     * restrict it for derived classes which need it
     **/
    T getSupport() const
    {
        return std::numeric_limits<T>::infinity();
    }

    bool isCompact() const
    {
        return std::isfinite(this->getSupport());
    }

    void setScale(const T & scale)
    {
        scale_ = scale;
        updateSecondary();
    }

    /** argument is squared!
     * in fact most rbf use a squared distance measure in the formula
     **/
    virtual T eval(const T& squared_x) const = 0;

    void updateSecondary()
    {
        scale_inv_ = 1 / scale_;
        scale_squared_ = scale_ * scale_;
        scale_squared_inv_ = 1 / scale_squared_;
        scale_squared_inv_neg_ = - scale_squared_inv_;
    }

protected:
    T scale_ = 1;

    /** we call it scale
     * this parameter is often know in other ways...
     * e.g. wikipedia use an epsilon in this page
     * http://en.wikipedia.org/wiki/Radial_basis_function
     * in our case: epsilon = 1/scale
     * these are ancillary variables, precomputed used for efficiency
    **/
    T scale_inv_ ;
    T scale_squared_;
    T scale_squared_inv_;
    T scale_squared_inv_neg_;

private:
    friend class cereal::access;

    template <class Archive> void serialize(Archive &ar)
    {
        ar(cereal::base_class<ElementBase>(this),
           CEREAL_NVP(scale_),
           CEREAL_NVP(scale_inv_),
           CEREAL_NVP(scale_squared_),
           CEREAL_NVP(scale_squared_inv_),
           CEREAL_NVP(scale_squared_inv_neg_));
    }
};



template <typename T>
class GaussianRBF: public RBFBase<T>
{
public:

    using RBFBase<T>::scale_squared_inv_neg_;

    /** in the case of Gaussian the scale correspond to the sigma
     **/
    GaussianRBF (const T& sigma): RBFBase<T>(sigma) {}

    /** def const
     **/
    GaussianRBF(): RBFBase<T>() {}

    // BasicKernel interface
    virtual inline T eval(const T &squared_x) const
    {
        return exp(squared_x * scale_squared_inv_neg_);
    }


private:
    friend class cereal::access;

    template <class Archive> void serialize(Archive &ar)
    {
        ar(cereal::base_class<RBFBase<T> >(this));
    }

};



template <typename T>
class MultiquadricRBF: public RBFBase<T>
{
public:

    using RBFBase<T>::scale_squared_inv_;

    /** in the case of Gaussian the scale correspond to the sigma
     **/
     MultiquadricRBF (const T& sigma): RBFBase<T>(sigma) {}

//    // BasicKernel interface
//    virtual inline T eval(const T &squared_x) const
//    {
//        return sqrt(1+ squared_x * scale_squared_inv_);
//    }


private:
    friend class cereal::access;

    template <class Archive> void serialize(Archive &ar)
    {
        ar(cereal::base_class<RBFBase<T> >(this));
    }

};










} // end nspace






#endif // KERNELS_HPP
