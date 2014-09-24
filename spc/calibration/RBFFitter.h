#ifndef FITRBF_H
#define FITRBF_H

#include <spc/calibration/RBFEvaluator.h>

namespace spc
{

template <typename ScalarT>
class RBFFitter: public ElementBase
{
public:

    //    typedef double ScalarT;
    typedef Eigen::Matrix<ScalarT,-1,1> VectorT;
    typedef Eigen::Matrix<ScalarT,-1,-1> MatrixT;

    typedef  RBFParameters<ScalarT> ParametersT;
    typedef typename RBFParameters<ScalarT>::Ptr ParametersPtrT;


    typedef  RBFEvaluator<ScalarT> EvaluatorT;
    typedef typename RBFEvaluator<ScalarT>::Ptr EvaluatorPtrT;

    typedef  RBFKernel<ScalarT> KernelT;
    typedef typename RBFKernel<ScalarT>::Ptr KernelPtrT;

    SPC_OBJECT(RBFFitter)
//    EXPOSE_TYPE


    RBFFitter()

    {

    }


    spcSetMacro(Points, points_, Eigen::MatrixXf)

    spcGetMacro(Points, points_, Eigen::MatrixXf)

    spcSetMacro(Values, values_, Eigen::VectorXf)

    spcGetMacro(Values, values_, Eigen::VectorXf)

    spcGetObjectMacro(Kernel, kernel_, typename KernelT)
    spcSetObjectMacro(Kernel, kernel_, typename KernelT)

    spcGetObjectMacro(Parameters, parameters_, typename ParametersT)
    spcSetObjectMacro(Parameters, parameters_, typename ParametersT)

    spcGetObjectMacro(Evaluator, evaluator_, typename EvaluatorT)
    spcSetObjectMacro(Evaluator, evaluator_, typename  EvaluatorT)


    protected:
        //! points in n-dim space
        MatrixT points_;

    //! scalar value for each point in points_
    VectorT values_;

    //! it also contains the parameters
    EvaluatorPtrT evaluator_ = EvaluatorPtrT(new EvaluatorT);

    //! but we provide a pointer to the parameters here for simplicity
    ParametersPtrT parameters_ = evaluator_->getParameters();

    //! also a shortcut for the kernel used
    KernelPtrT kernel_ = parameters_->getKernel();
};

}//end nspace

#endif // FITRBF_H
