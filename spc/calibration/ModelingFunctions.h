#ifndef MODELINGFUNCTIONS_H
#define MODELINGFUNCTIONS_H

#include <algorithm>


#include <Eigen/Eigen>

#include <spc/calibration/Observations.h>
#include <spc/calibration/BasicResidualBlock.h>

#include <spc/calibration/IntensityModelFixedPars.h>

#include <spc/calibration/ParametersHolder.h>

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
T predict_intensities(const Observation &ob, const BasicResidualBlock & res_block,  T const * const * parameters)
{

    Eigen::Matrix<T, -1, -1> c_dist =  res_block.remapFromPointerAndName(parameters, "coeff_distance");

//    std::cout << "C_DIST\n" << std::endl;

//    for (int i = 0; i <c_dist.size(); ++i)
//    {
//        std::cout << " " << c_dist.array()(i) << std::endl;
//    }



    Eigen::Matrix<T, -1, -1> c_angle = res_block.remapFromPointerAndName(parameters, "coeff_angle");

//    std::cout << "C_ANGLE\n" << c_angle << std::endl;

//    std::cout << "C_ANGLE\n" << std::endl;

//    for (int i = 0; i <c_angle.size(); ++i)
//    {
//        std::cout << " " << c_angle.array()(i) << std::endl;
//    }



    Eigen::Matrix<T, 1, 1> edist;
    edist << T(ob.distance);

    Eigen::Matrix<T, 1, 1> eang;
    eang<< T(ob.angle);

    Eigen::Matrix<T, -1, -1> knots_dist =   res_block.remapFromPointerAndName(parameters, "knots_distance");
    Eigen::Matrix<T, -1, -1> knots_angle =  res_block.remapFromPointerAndName(parameters, "knots_angle");

//    std::cout << "remapping sigma distance " << std::endl;
    T sigma_distance =   res_block.remapFromPointerAndName(parameters, "sigma_distance")(0);
//    std::cout << "remapping sigma angle " << std::endl;
    T sigma_angle =   res_block.remapFromPointerAndName(parameters, "sigma_angle")(0);

//    std::cout << "done" << std::endl;

    T d_effect = compute_rbf<T>(c_dist, knots_dist, sigma_distance, edist );
    T a_effect = compute_rbf<T>(c_angle, knots_angle, sigma_angle, eang );


    T prediction = d_effect * a_effect;

    return prediction;
}

}
#endif // MODELINGFUNCTIONS_H
