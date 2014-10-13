#ifndef MODELINGFUNCTIONS_H
#define MODELINGFUNCTIONS_H

#include <algorithm>


#include <Eigen/Eigen>

#include <spc/calibration/Observations.h>
#include <spc/calibration/BasicResidualBlock.h>

//#include <spc/calibration/IntensityModelFixedPars.h>

#include <spc/calibration/ParametersBlock.h>

namespace spc {
using Eigen::Matrix;


template <typename T>
T angle_law(const Observation &ob, const BasicResidualBlock & res_block,  T const * const * parameters)
{
    T p =  res_block.remapFromPointerAndName(parameters, "angle_cos_proportion")(0);

    T angle_rad = T(ob.angle) / M_PI * T(180); // as radians

    T slope = res_block.remapFromPointerAndName(parameters, "angle_slope")(0);

    return (T(1)-p) * (slope * angle_rad + T(1)) + p * ( cos(angle_rad) );

}

template<typename T>
T distance_law(const Observation &ob, const BasicResidualBlock & res_block,  T const * const * parameters)
{
    T exponential = res_block.remapFromPointerAndName(parameters, "distance_exponential")(0);

    T distance = T(ob.distance);

    return T(1) / distance;
}

template <typename T>
T overall_multiplier(const Observation &ob, const BasicResidualBlock & res_block,  T const * const * parameters)
{
    return res_block.remapFromPointerAndName(parameters, "overall_multiplier" ) (0);
}

template <typename T>
T overall_shift(const Observation &ob, const BasicResidualBlock & res_block,  T const * const * parameters)
{
    return res_block.remapFromPointerAndName(parameters, "overall_shift" ) (0);
}

template <typename T>
T compute_rbf(const Matrix<T, -1, 1> &coefficients,
              const Matrix<T, -1, -1> &nodes,
              const T &sigma,
              const Matrix<T, -1, 1> &point)
{

    // compute squared distances of point from nodes
    Matrix<T, -1, -1> diff  = nodes.rowwise() - point.transpose();
    Matrix<T, -1, 1> sq_dist =  diff.rowwise().squaredNorm();

    // transform these sq_dist in weights
    Matrix<T, -1, 1> weights(sq_dist.rows());

    for (int i = 0; i < sq_dist.rows(); ++i)
        weights(i)  = exp(- sq_dist(i) / (sigma * sigma)  );

    // compute the RBF value
    return weights.cwiseProduct(coefficients ).sum();
}

template <typename T>
T predict_intensities(const Observation &ob, const BasicResidualBlock & res_block,  T const * const * parameters)
{

    Eigen::Matrix<T, -1, -1> c_dist =  res_block.remapFromPointerAndName(parameters, "coeff_distance");

    Eigen::Matrix<T, -1, -1> c_angle = res_block.remapFromPointerAndName(parameters, "coeff_angle");

    Eigen::Matrix<T, 1, 1> edist;
    edist << T(ob.distance);

    Eigen::Matrix<T, 1, 1> eang;
    eang<< T(ob.angle);

    Eigen::Matrix<T, -1, -1> knots_dist =   res_block.remapFromPointerAndName(parameters, "knots_distance");
    Eigen::Matrix<T, -1, -1> knots_angle =  res_block.remapFromPointerAndName(parameters, "knots_angle");

    T sigma_distance =   res_block.remapFromPointerAndName(parameters, "sigma_distance")(0);
    T sigma_angle =   res_block.remapFromPointerAndName(parameters, "sigma_angle")(0);

    T d_effect = compute_rbf<T>(c_dist, knots_dist, sigma_distance, edist );
    T a_effect = compute_rbf<T>(c_angle, knots_angle, sigma_angle, eang );

    T standard_laws_effect = angle_law(ob, res_block, parameters) *
            distance_law(ob, res_block, parameters) *
            overall_multiplier(ob, res_block, parameters)
            ;


    T prediction = d_effect * a_effect + standard_laws_effect + overall_shift(ob, res_block, parameters);

    return prediction;
}

}
#endif // MODELINGFUNCTIONS_H
