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

    T upper = -T(2) * pow(rd, T(2)) * pow((R + d), T(2))   ;
    T lower = pow(D,T(2)) * pow(((T(1)-Sd/f) * R + d - ((d*Sd)/f) + Sd), T(2));

    return T(1) - exp(upper/lower);
}


//Eigen::VectorXf near_distance_effect_(const Eigen::VectorXf & R, const Eigen::VectorXf &pars)
//{
//    Eigen::VectorXf out;
//    out.resize(R.rows());

//    for (int i = 0 ; i < R.rows(); ++i)
//    {
//        out(i) = near_distance_effect(R(i), pars.data());
//    }

//    return out;
//}




Eigen::VectorXd near_distance_effect_(const Eigen::VectorXd & R, const Eigen::VectorXd & pars)
{
    Eigen::VectorXd out;
    out.resize(R.rows());
    for (int i = 0; i < R.rows(); ++i)
        out(i) = near_distance_effect(R(i), pars.data());

    return out;
}


template <typename T>
T distance_effect(const T &R)
{
    return T(1) / pow(R, T(2));
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

template<typename T>
T predict_intensity_no_angle(const T & R,
                             const T * const near_distance_pars,
                             const T * const material_pars
                             )
{
    return material_effect(material_pars) * distance_effect(R) * near_distance_effect(R, near_distance_pars);
}


Eigen::VectorXd predict_intensity_no_angle_(const Eigen::VectorXd & R,
                                            const Eigen::VectorXd & near_distance_pars,
                                            const Eigen::VectorXd & material_pars)
{
    Eigen::VectorXd out;
    out.resize(R.rows());
    for (int i = 0 ; i < R.rows(); ++i)
    {
        out(i) = predict_intensity_no_angle(R(i), near_distance_pars.data(), material_pars.data());
    }

    return out;
}


struct CalibrationResidualNoAngle {
    CalibrationResidualNoAngle(double distance,
                               double intensity)
    {
        distance_ = distance;
        intensity_ = intensity;
    }

    template <typename T> bool operator()(const T * const near_distance_pars,
                                          const T * const material_pars,
                                          T* residual
                                          ) const {
        residual[0] = T(intensity_) - predict_intensity_no_angle(T(distance_), near_distance_pars, material_pars);
        return true;
    }

private:
    double distance_;
    double intensity_;
};


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
        LOG(INFO) << "called residual operator () "<< residual[0];
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
        if (problem_ != nullptr)
            delete problem_;
    }

    Eigen::VectorXd predict_no_angle(const Eigen::VectorXd & R) const
    {
//        Eigen::VectorXd out;
//        out.resize(R.rows());
//        for (int i = 0 ; i < R.rows(); ++i)
//            out(i) = predict_intensity_no_angle(R(i),
//                                                near_distance_pars.data(),
//                                                material_pars.data());

        Eigen::VectorXd out;
        out.resize(R.rows());
        for (int i = 0 ; i < R.rows(); ++i)
        {
            out(i) = predict_intensity_no_angle(R(i), near_distance_pars.data(), material_pars.data());
        }

        return out;

//        return out;
    }

    void initProblem()
    {

        problem_ = new Problem;
        for (int i = 0; i < R_.rows(); ++i)
        {

            if (!no_angle)
            {
                problem_->AddResidualBlock(
                            new AutoDiffCostFunction<CalibrationResidual, 1,  5, 1, 1>(
                                new CalibrationResidual(R_(i), a_(i), I_(i))),
                            nullptr,
                            near_distance_pars.data(), material_pars.data(), roughness_pars.data());


            }

            else // no angle estimation
            {
                problem_->AddResidualBlock(
                            new AutoDiffCostFunction<CalibrationResidualNoAngle, 1,  5, 1>(
                                new CalibrationResidualNoAngle(R_(i), I_(i))),
                            nullptr,
                            near_distance_pars.data(), material_pars.data());
            }





        }



        if (use_bounds == true)
        {
            for (int i = 0; i < near_distance_pars.rows(); ++i)
            {
                problem_->SetParameterLowerBound(near_distance_pars.data(), i, near_distance_lower_bounds(i));
                problem_->SetParameterUpperBound(near_distance_pars.data(), i, near_distance_upper_bounds(i));
            }
        }

        LOG(INFO) << "Near distance pars (at init): " << near_distance_pars << "\n"
                  << "Material pars (at init): " << material_pars << "\n"
                  << "Roughness pars (at init): " << roughness_pars;
    }


    void testSet(const Eigen::VectorXf & testv)
    {
        LOG(INFO) << testv;
    }

    void solve()
    {
        Solver::Options options;
        options.max_num_iterations = 100;
        options.linear_solver_type = ceres::DENSE_QR;
        options.minimizer_progress_to_stdout = true;
        options.eta = 1e4;
        Solver::Summary summary;
        Solve(options, problem_, &summary);
        std::cout << summary.FullReport() << "\n";

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

    spcSetMacro(NearDistanceUpperBounds, near_distance_upper_bounds, Eigen::VectorXd)
    spcGetMacro(NearDistanceUpperBounds, near_distance_upper_bounds, Eigen::VectorXd)

    spcSetMacro(NearDistanceLowerBounds, near_distance_lower_bounds, Eigen::VectorXd)
    spcGetMacro(NearDistanceLowerBounds, near_distance_lower_bounds, Eigen::VectorXd)

    Eigen::VectorXd near_distance_upper_bounds;
    Eigen::VectorXd near_distance_lower_bounds;

    Eigen::VectorXd  near_distance_pars;
    Eigen::VectorXd  material_pars;
    Eigen::VectorXd  roughness_pars;

    Problem * problem_ = nullptr;


    bool no_angle = true; // dont use angle information

    bool use_bounds = true;
};

#endif // SIMPLECALIBRATION_H
