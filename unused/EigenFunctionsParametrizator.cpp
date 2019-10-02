#include "EigenFunctionsParametrizator.h"
//#include <spc/elements/ICalPHFunction.h>


int spc::EigenFunctionsParametrizator::compute()
{
    if (function_->getInputSize() != vars_.cols()) {
        LOG(ERROR) <<
                    "Wrong dimension of independent variables";
        return -1;
    }

    if (vars_.rows() != b_.size()) {
        LOG(ERROR) << "Dependent and independent variables "
                                  "must have the same numbe of elements";
        return -1;
    }

    if (function_->isA(&EigenLinearFunctionBase::Type)) {
        // we go with linear least squares
        EigenLinearFunctionBase::Ptr lin_f = spcDynamicPointerCast
                <EigenLinearFunctionBase>(function_);




        Eigen::MatrixXf A(vars_.rows(), lin_f->getNumberOfCoefficients());
        for (int i = 0; i < vars_.rows(); ++i)
            A.row(i) = lin_f->getRowOfObservationsMatrix(vars_.row(i));

        std::cout << A.row(0) << std::endl;

        // solve it!
        Eigen::VectorXf coeffs  = A.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b_);

        lin_f->setCoefficients(coeffs);

        prediction_ = A * coeffs;


        return 1;

    } else {
        // we must go non-linear. Still not implemented
        LOG(ERROR) << "Your are trying to parametrize a "
                                  "nonlinear function.\n Nonlinear solver "
                                  "is not implemented yet\n";
        return -1;
    }
}
