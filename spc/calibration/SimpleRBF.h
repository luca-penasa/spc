#ifndef SIMPLERBF_H
#define SIMPLERBF_H


#include <Eigen/Eigen>
#include <iostream>
#include <spc/elements/macros.h>
namespace spc
{

// define a custom template unary functor
template<typename Scalar >
struct CwiseGaussian
{
    CwiseGaussian(const Scalar& sigma) : sigma_(sigma) {}

    const Scalar operator()(const Scalar& x) const
    {
        return exp(-(sigma_*x) * (sigma_*x));
    }

    Scalar sigma_;

};


class SimpleRBF
{
public:
    SimpleRBF();

    void setKnots(const Eigen::MatrixXf & knots)
    {
        knots_ = knots;
    }

    void setParameters(const Eigen::VectorXf &pars)
    {

        if (pars.rows() != knots_.rows())
        {
            std::cout << "pars must be of equal size of knots" << std::endl;
            return;
        }

        pars_ = pars;
    }

    float operator ()(const Eigen::VectorXf &x) const
    {
        if (this->getDimensions() != x.rows())
        {
            std::cout << "dimensionality of rbf is different from provided point" << std::endl;
            return spcNANMacro;
        }

        if (pars_.rows() != knots_.rows())
        {
            std::cout << "knots or parameters not correctly setted" << std::endl;
            return spcNANMacro;
        }

        Eigen::VectorXf w = this->getWeightsAtPoint(x);
        return w.cwiseProduct(pars_).sum();
    }

    Eigen::VectorXf batchEval(Eigen::MatrixXf &points) const
    {

    }


    size_t getDimensions( ) const
    {
        return knots_.cols();
    }

    size_t getNumberOfKnots() const
    {
        return knots_.rows();
    }

    void setSigma(const float & s)
    {
        sigma_ = s;
    }


    // gives back the distance of x from the knots
    Eigen::VectorXf getDistancesFromKnots (const Eigen::VectorXf & x) const;

    Eigen::VectorXf getWeightsAtPoint(const Eigen::VectorXf &x) const;

protected:
    Eigen::MatrixXf knots_;
    Eigen::VectorXf pars_;

    float sigma_ = 1.0f; // used in kernels with sigmas, as gaussian


};


} //end nspace
#endif // SIMPLERBF_H
