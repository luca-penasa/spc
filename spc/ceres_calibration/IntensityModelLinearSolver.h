#ifndef INTENSITYMODELLINEARSOLVER_H
#define INTENSITYMODELLINEARSOLVER_H

#include <spc/ceres_calibration/IntensityModelLinear.h>
#include <spc/elements/PointCloudBase.h>
namespace spc
{


class IntensityModelLinearSolver
{
public:
    IntensityModelLinearSolver();

    void setInputModel(IntensityModelLinear::Ptr mod)
    {
        model_ = mod;
    }

    void setPoints(const Eigen::MatrixXf &points)
    {
        points_ = points;
    }

    void setValues(const Eigen::VectorXf &v)
    {
        values_ = v;
    }

    void updateAMatrix()
    {
        size_t n =model_->getAMatrixRow(points_.row(0)).rows();
        A_.resize(points_.rows(), n);

        for (int i = 0; i < points_.rows(); ++i)
        {
            A_.row(i) = model_->getAMatrixRow(points_.row(i));
        }
    }

    void solve()
    {
        x_  = A_.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(values_);
    }






protected:

    IntensityModelLinear::Ptr model_;
    Eigen::MatrixXf points_;
    Eigen::VectorXf values_;

    Eigen::MatrixXf A_;
    Eigen::VectorXf x_;


};


}
#endif // INTENSITYMODELLINEARSOLVER_H
