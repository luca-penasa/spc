#include "RBFParameters.h"
namespace spc
{
//template<>
//DtiClassType RBFParameters<double>::Type ("RBFParameters", &ElementBase::Type);


//template class RBFParameters<double>;


void setCoefficients(const Eigen::Matrix<double, -1, 1> &coeffs)
{
    coeffs_ = coeffs;
}


void setCoefficients(const Eigen::Matrix<Eigen::Jet<double, 4>, -1, 1> &coeffs)
{
    coeffs_ = coeffs;
}





}//end nspace

