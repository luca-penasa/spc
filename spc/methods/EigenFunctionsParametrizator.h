#ifndef EIGENFUNCTIONSPARAMETRIZATOR_H
#define EIGENFUNCTIONSPARAMETRIZATOR_H

#include <spc/elements/EigenFunctionBase.h>
#include <spc/elements/EigenLinearFunctionBase.h>
#include <pcl/console/print.h>

namespace spc
{

class EigenFunctionsParametrizator
{
public:
    EigenFunctionsParametrizator()
    {
    }

    virtual void setFunction(const EigenFunctionBase::Ptr f)
    {
        function_ = f;
    }

    virtual void setY(const Eigen::VectorXf &y)
    {
        b_ = y;
    }

    virtual void setVariables(const Eigen::MatrixXf &vars)
    {
        vars_ = vars;
    }

    virtual Eigen::VectorXf getPrediction() const
    {
        return prediction_;
    }

    virtual int compute();

protected:
    EigenFunctionBase::Ptr function_;
    Eigen::VectorXf b_;
    Eigen::MatrixXf vars_;

    Eigen::VectorXf prediction_ ;
};

} // end nspace

#endif // EIGENFUNCTIONSPARAMETRIZATOR_H
