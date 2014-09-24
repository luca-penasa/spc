#ifndef SIMPLERBF_H
#define SIMPLERBF_H


#include <Eigen/Eigen>
#include <iostream>
#include <spc/elements/macros.h>
#include <spc/elements/ElementBase.h>

#include <spc/calibration/RBFParameters.h>
namespace  spc {


template <typename ScalarT = double>
class RBFEvaluator: public ElementBase
{
public:



    //    typedef double ScalarT;
    typedef Eigen::Matrix<ScalarT,-1,1> VectorT;
    typedef Eigen::Matrix<ScalarT,-1,-1> MatrixT;

    typedef  RBFParameters<ScalarT> ParameterT;
    typedef typename RBFParameters<ScalarT>::Ptr ParameterPtrT;


    SPC_OBJECT(RBFEvaluator)
    //    EXPOSE_TYPE


    RBFEvaluator()
    {

    }

    RBFEvaluator(const ParameterPtrT pars)
    {
        parameters_ = pars;
    }


    ScalarT operator ()(const VectorT &x) const
    {

        if (!parameters_->isValid())
        {
            std::cout << "parameters seems to be not valid\n" << std::endl;
            return std::numeric_limits<ScalarT>::quiet_NaN();
        }

        return singleEval(x);
    }



public:
    VectorT batchEval(const MatrixT &points) const
    {

        VectorT out(points.rows());

        if (!parameters_->isValid())
        {
            std::cout << "parameters seems to be not valid\n" << std::endl;
            out = VectorT::Constant(points.rows(), std::numeric_limits<ScalarT>::quiet_NaN());
            return out;
        }


#ifdef USE_OPENMP // more horse-power!
#pragma omp parallel for
#endif
        for (int i = 0 ; i < points.rows(); ++i)
        {
            out(i) = operator ()(points.row(i));
        }

        return out;
    }

    ParameterPtrT getParameters () const
    {
        return parameters_;
    }


    // gives back the distance of x from the knots
    VectorT getSquaredDistancesFromNodes (const VectorT & x) const
    {


        MatrixT diff  = parameters_->getNodes().rowwise() - x.transpose();




        //        VectorT d(diff.rows());
        //        for (int i = 0; i < diff.rows(); ++i)
        //        {
        //            ScalarT sq_sum(0);

        //            for (int j =0; j < diff.cols(); ++j)
        //            {

        //                sq_sum += diff(i,j) * diff(i,j);
        //            }

        //            std::cout << "squared sum is " << sq_sum << std::endl;
        //           d(i) = sqrt(sq_sum);


        //        }

        return  diff.rowwise().squaredNorm();

        //        std::cout << "distances are" << std::endl;
        //        for (int i =0; i < d.rows(); ++i)
        //        {
        //            std::cout << d(i) << std::endl;
        //        }

//        return d;
    }

    VectorT getWeightsAtPoint(const VectorT &x) const
    {
        VectorT sq_dist = getSquaredDistancesFromNodes(x);
        VectorT w (sq_dist.rows());
        for (int i =0; i < sq_dist.rows(); ++i)
        {
            w(i) = parameters_->getKernel()->operator ()(sq_dist(i));
        }
        return w;

        //        return dist;
    }

protected:
    ParameterPtrT parameters_ = ParameterPtrT(new ParameterT);


protected:
    ScalarT singleEval(const VectorT &x) const
    {
        VectorT w = this->getWeightsAtPoint(x);
        return w.cwiseProduct(parameters_->getCoefficients() ).sum();
    }
};


} //end nspace
#endif // SIMPLERBF_H
