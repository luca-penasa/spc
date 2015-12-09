#ifndef KERNELS_HPP
#define KERNELS_HPP

#include <spc/elements/ElementBase.h>
#include <spc/core/spc_eigen.h>
#include <spc/core/fastapprox.h>
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
//	SPC_ELEMENT(RBFBase<T>)
	spcTypedefSharedPtrs(RBFBase<T>)

    EXPOSE_TYPE

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


    RBFBase(const RBFBase & other): ElementBase(other)
    {
        scale_ = other.scale_;
        updateSecondary();
    }



    T getScale() const
    {
        return scale_;
    }

    /** by default support is infinite
     * restrict it for derived classes which need it
     **/
    virtual T getSupport() const
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
    virtual T eval_squared(const T& squared_x) const = 0;


    virtual Eigen::Matrix<T, -1, 1> eval_squared(const Eigen::Matrix<T, -1, 1> &squared_xs) const
    {
        Eigen::Matrix<T, -1, 1> w;
        w.resize(squared_xs.rows());

#ifdef USE_OPENMP
#pragma omp parallel for
#endif
        for (int i = 0; i < squared_xs.rows(); ++i)
        {

            w(i) = this->eval_squared( squared_xs(i));
        }

        return w;

    }

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

    SPC_ELEMENT(GaussianRBF)
    EXPOSE_TYPE

    using RBFBase<T>::scale_squared_inv_neg_;


    /** in the case of Gaussian the scale correspond to the sigma
     **/
    GaussianRBF (const T& sigma): RBFBase<T>(sigma) {}

    GaussianRBF(const GaussianRBF & other): RBFBase<T> (other)
    {

    }

    /** def const
     **/
    GaussianRBF(): RBFBase<T>() {}

    // BasicKernel interface
    virtual inline T eval_squared(const T &squared_x) const override
    {
        return exp(squared_x * 0.5* scale_squared_inv_neg_);
    }




private:
    friend class cereal::access;

    template <class Archive> void serialize(Archive &ar)
    {
        ar(cereal::base_class<RBFBase<T> >(this));
    }

};

template <typename T>
class GaussianApproxRBF: public RBFBase<T>
{
public:

    SPC_ELEMENT(GaussianApproxRBF)

    EXPOSE_TYPE

    using RBFBase<T>::scale_squared_inv_neg_;

    /** in the case of Gaussian the scale correspond to the sigma
     **/
    GaussianApproxRBF (const T& sigma): RBFBase<T>(sigma) {}

    GaussianApproxRBF(const GaussianApproxRBF & other): RBFBase<T> (other)
    {

    }




    /** def const
     **/
    GaussianApproxRBF(): RBFBase<T>() {}

    // BasicKernel interface
    virtual inline T eval_squared(const T &squared_x) const override
    {
        return fastexp(squared_x * 0.5 * scale_squared_inv_neg_);
    }


private:
    friend class cereal::access;

    template <class Archive> void serialize(Archive &ar)
    {
        ar(cereal::base_class<RBFBase<T> >(this));
    }

};


/** this kernel is defined as (from wikipedia)
 * K(u) = \frac{3}{4}(1-u^2) \,\mathbf{1}_{\{|u|\leq1\}}
 **/
template <typename T>
class EpanechnikovRBF: public RBFBase<T>
{
public:

    SPC_ELEMENT(EpanechnikovRBF)
EXPOSE_TYPE
    using RBFBase<T>::scale_squared_inv_;
    using RBFBase<T>::scale_;


    /** in the case of Gaussian the scale correspond to the sigma
     **/
    EpanechnikovRBF (const T& sigma): RBFBase<T>(sigma) {}

    EpanechnikovRBF(const EpanechnikovRBF & other): RBFBase<T> (other)
    {

    }

    /** def const
     **/
    EpanechnikovRBF(): RBFBase<T>() {}

    // BasicKernel interface
    virtual inline T eval_squared(const T &squared_x) const override
    {
        if ((squared_x * scale_squared_inv_) >= 1)
            return 0;
        else
            return ratio_ - ratio_ * squared_x * scale_squared_inv_;
    }

    virtual T getSupport() const override
    {
        return scale_; // that is the support for this kernel (1 * scale_)
    }


private:
    friend class cereal::access;

    template <class Archive> void serialize(Archive &ar)
    {
        ar(cereal::base_class<RBFBase<T> >(this));
    }

    T ratio_ = 0.75; /**< the ratio 3/4 used int this kernel */

};



template <typename T>
class MultiquadricRBF: public RBFBase<T>
{
public:
EXPOSE_TYPE

SPC_ELEMENT(MultiquadricRBF)
    using RBFBase<T>::scale_squared_inv_;

    /** in the case of Gaussian the scale correspond to the sigma
     **/
     MultiquadricRBF (const T& sigma): RBFBase<T>(sigma) {}

     MultiquadricRBF(): RBFBase<T>() {}


     MultiquadricRBF(const MultiquadricRBF & other): RBFBase<T> (other)
     {

     }


//     virtual ElementBase::Ptr clone() const override
//     {
//         return ElementBase::Ptr(new MultiquadricRBF(*this));
//     }


    // BasicKernel interface
    virtual inline T eval_squared(const T &squared_x) const override
    {
        return sqrt(1+ squared_x * scale_squared_inv_);
    }


private:
    friend class cereal::access;

    template <class Archive> void serialize(Archive &ar)
    {
        ar(cereal::base_class<RBFBase<T> >(this));
    }

};



template <typename T>
class PolyharmonicRBF: public RBFBase<T>
{
public:

    EXPOSE_TYPE

    SPC_ELEMENT(PolyharmonicRBF)

     PolyharmonicRBF(const size_t k = 1): RBFBase<T>(), k_(k) {}


     PolyharmonicRBF(const PolyharmonicRBF & other): RBFBase<T> (other)
     {
        k_ = other.k_;
     }


//     virtual ElementBase::Ptr clone() const override
//     {
//         return ElementBase::Ptr(new PolyharmonicRBF(*this));
//     }



     /**
     * @brief eval_squared
     * @param squared_x
     * @return
     * implemented this way for now, we must seek for faster ways to do this,
     * maybe k could be a template arg
     */
    virtual inline T eval_squared(const T &squared_x) const override
    {

//         LOG(INFO) << "in dist: " << std::sqrt(squared_x);
        if (k_ % 2 == 0)
        {
            T val = std::sqrt(squared_x);
            return std::pow(val, k_) * std::log(val);
        }

        else // its odd
        {
            T val = std::sqrt(squared_x);
            return std::pow(val, k_);
        }
    }


private:
    friend class cereal::access;

    template <class Archive> void serialize(Archive &ar)
    {
        ar(cereal::base_class<RBFBase<T> >(this),
           CEREAL_NVP(k_));

    }

    size_t k_;

};






} // end nspace






#endif // KERNELS_HPP
