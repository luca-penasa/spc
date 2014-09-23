#include "SimpleRBF.h"

namespace
spc
{

SimpleRBF::SimpleRBF()
{
}

Eigen::VectorXf SimpleRBF::getDistancesFromKnots(const Eigen::VectorXf &x) const
{
    Eigen::MatrixXf diff  = knots_.rowwise() - x.transpose();
    Eigen::VectorXf d = diff.rowwise().norm();
    return d;
}

Eigen::VectorXf SimpleRBF::getWeightsAtPoint(const Eigen::VectorXf &x) const
{
    Eigen::VectorXf dist = getDistancesFromKnots(x);
    return dist.unaryExpr(CwiseGaussian<float>(sigma_));

    //        return dist;
}

}//end nspace
