#ifndef SIMPLECALIBRATION_H
#define SIMPLECALIBRATION_H

#include <spc/core/spc_eigen.h>
#include "ceres/ceres.h"

#include <spc/core/macros.h>
using ceres::AutoDiffCostFunction;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solver;
using ceres::Solve;




template <typename T>
T near_distance_effect(const T &R, const T * const pars)
{
    T rd = pars[0];
    T d = pars[1];
    T D = pars[2];
    T Sd = pars[3];
    T f = pars[4];

    T upper = -T(2) * rd*rd * pow((R + d), T(2))   ;
    T lower = D*D * pow(((T(1)-Sd/f) * R + d - ((d*Sd)/f) + Sd), T(2));

    return T(1) - exp(upper/lower);
}


template <typename T>
T distance_effect(const T &R)
{
    return T(1) / R*R;
}


template <typename T>
T material_effect(const T * const pars)
{
    T p = pars[0];
    return p;

}


template <typename T>
T angle_effect(const T & rad_angle, const T * const pars)
{
    T n = pars[0];
    return T(1) - n + n * cos(rad_angle);
}

template<typename T>
T predict_intensity(const T & R,
                    const T & angle,
                    const T * const near_distance_pars,
                    const T * const material_pars,
                    const T * const roughness_pars
                    )
{
    return material_effect(material_pars) * distance_effect(R) * near_distance_effect(R, near_distance_pars) * angle_effect(angle / T(180 * M_PI), roughness_pars);
}


struct CalibrationResidual {
    CalibrationResidual(double distance,
                        double angle,
                        double intensity)
    {
        distance_ = distance;
        angle_ = angle;
        intensity_ = intensity;
    }

    template <typename T> bool operator()(const T * const near_distance_pars,
                                          const T * const material_pars,
                                          const T * const roughness_pars,
                                          T* residual
                                          ) const {
        residual[0] = T(intensity_) - predict_intensity(T(distance_), T(angle_), near_distance_pars, material_pars, roughness_pars);
        return true;
    }

private:
    double distance_;
    double intensity_;
    double angle_;
};



class SimpleCalibration
{
public:
    SimpleCalibration()
    {
    }

    ~SimpleCalibration()
    {
        delete problem_;
    }

    void initProblem()
    {

        problem_ = new Problem;
        for (int i = 0; i < R_.rows(); ++i)
        {
          problem_->AddResidualBlock(
              new AutoDiffCostFunction<CalibrationResidual, 1,  5, 1, 1>(
                  new CalibrationResidual(R_(i), a_(i), I_(i))),
              NULL,
              near_distance_pars.data(), material_pars.data(), roughness_pars.data());
        }
    }


    void testSet(const Eigen::VectorXf & testv)
    {
        LOG(INFO) << testv;
    }

    void solve()
    {
        Solver::Options options;
        options.max_num_iterations = 25;
        options.linear_solver_type = ceres::DENSE_QR;
        options.minimizer_progress_to_stdout = true;

        Solver::Summary summary;
        Solve(options, problem_, &summary);
        std::cout << summary.FullReport() << "\n";
//        std::cout << "Initial m: " << 0.0 << " c: " << 0.0 << "\n";
//        std::cout << "Final   m: " << m << " c: " << c << "\n";
    }


    spcSetMacro(R, R_, Eigen::VectorXd)
    spcGetMacro(R, R_, Eigen::VectorXd)
    spcSetMacro(Angle, a_, Eigen::VectorXd)
    spcGetMacro(Angle, a_, Eigen::VectorXd)
    spcSetMacro(I, I_, Eigen::VectorXd)
    spcGetMacro(I, I_, Eigen::VectorXd)


    Eigen::VectorXd R_;
    Eigen::VectorXd a_;
    Eigen::VectorXd I_;

    spcSetMacro(NearDistancePars, near_distance_pars, Eigen::VectorXd)
    spcGetMacro(NearDistancePars, near_distance_pars, Eigen::VectorXd)

    spcSetMacro(MaterialPars, material_pars, Eigen::VectorXd)
    spcGetMacro(MaterialPars, material_pars, Eigen::VectorXd)

    spcSetMacro(RoughnessPars, roughness_pars, Eigen::VectorXd)
    spcGetMacro(RoughnessPars, roughness_pars, Eigen::VectorXd)

    Eigen::VectorXd  near_distance_pars;
    Eigen::VectorXd  material_pars;
    Eigen::VectorXd  roughness_pars;

    Problem * problem_;
};

#endif // SIMPLECALIBRATION_H
