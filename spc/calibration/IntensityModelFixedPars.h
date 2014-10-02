#ifndef INTENSITYMODELFIXEDPARS_H
#define INTENSITYMODELFIXEDPARS_H

#include <spc/calibration/SampledData.h>

namespace spc
{

class IntensityModelFixedPars
{
public:

    IntensityModelFixedPars(size_t n_dist, size_t n_angle)
    {
        n_dist_pars = n_dist;
        n_ang_pars = n_angle;
    }



    const size_t getNumberOfDistanceKnots() const
    {
        return knots_dist->rows();
    }

    const size_t getNumberOfAngleKnots() const
    {
        return knots_angle->rows();
    }

    void initFromData(const SampledData &data)
    {
        knots_dist =  new Eigen::MatrixXd(Eigen::VectorXd::LinSpaced (n_dist_pars, data.d_.minCoeff(), data.d_.maxCoeff()));

        knots_angle =  new Eigen::MatrixXd(Eigen::VectorXd::LinSpaced (n_ang_pars, 0, 90));


        std::cout << "node for distance " << std::endl;
        std::cout << *knots_dist << std::endl;

        std::cout << "node for angle " << std::endl;
        std::cout << *knots_angle << std::endl;


        sigma_dist = knots_dist->operator() (1) - knots_dist->operator ()(0);
        sigma_angle= knots_angle->operator() (1) - knots_angle->operator ()(0);
    }

    double sigma_dist;
    double sigma_angle;

    Eigen::MatrixXd * knots_dist;
    Eigen::MatrixXd * knots_angle;


    size_t n_dist_pars = 6;
    size_t n_ang_pars = 4;


};



}
#endif // INTENSITYMODELFIXEDPARS_H
