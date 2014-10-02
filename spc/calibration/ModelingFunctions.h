#ifndef MODELINGFUNCTIONS_H
#define MODELINGFUNCTIONS_H

#include <algorithm>


#include <Eigen/Eigen>

#include <spc/calibration/Observations.h>
#include <spc/calibration/ResidualBlocks.h>

#include <spc/calibration/IntensityModelFixedPars.h>

namespace spc {
using Eigen::Matrix;




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
T predict_intensities(const Observation &ob, const IntensityModelFixedPars &fixed_pars,  T const * const * parameters)
{
    typedef Eigen::Matrix<T,-1,1> MatrixType;
    typedef Eigen::Map<const MatrixType> MapType;

    const T * coeffs_dist = parameters[0];
    const T * coeffs_angle = parameters[1];

    MapType c_dist  (coeffs_dist, fixed_pars.getNumberOfDistanceKnots());
    MapType c_angle  (coeffs_angle, fixed_pars.getNumberOfAngleKnots());


    Eigen::Matrix<T, 1, 1> edist;
    edist << T(ob.distance);

    Eigen::Matrix<T, 1, 1> eang;
    eang<< T(ob.angle);


    T d_effect = compute_rbf<T>(c_dist, (fixed_pars.knots_dist)->template cast<T>(), T(fixed_pars.sigma_dist), edist );
    T a_effect = compute_rbf<T>(c_angle, (fixed_pars.knots_angle)->template cast<T>(), T(fixed_pars.sigma_angle), eang );


    T prediction = d_effect * a_effect;

    return prediction;
}

}
#endif // MODELINGFUNCTIONS_H
