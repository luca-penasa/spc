#ifndef CALIBRATIONMANAGER_H
#define CALIBRATIONMANAGER_H

#include <spc/elements/ElementBase.h>
#include <spc/calibration/Observations.h>
#include <spc/calibration/CalibrationFactors.h>
#include <spc/calibration/ResidualBlocks.h>
#include <spc/elements/EigenTable.h>
#include <ceres/ceres.h>

#include <spc/io/AsciiEigenTableWriter.h>


#include <spc/calibration/SampledData.h>
#include <spc/calibration/ModelingFunctions.h>


#include <spc/calibration/ParametersHolder.h>
namespace spc
{

class CalibratorManager
{
public:
    CalibratorManager(char ** argv);

    void setSolverOptions();

    void setUpFixedPars()
    {
        fixed_pars_.initFromData(samples_);
    }

    void setUpInitParametersBlocks()
    {

        std::cout << "setting up blocks" << std::endl;

        dist_pars_ei_ = Eigen::VectorXd::Ones(fixed_pars_.n_dist_pars);
        angle_pars_ei_ = Eigen::VectorXd::Ones(fixed_pars_.n_ang_pars);

        std::cout << "init pars - dist: " << std::endl;
        std::cout << dist_pars_ei_ << std::endl;

        std::cout << "init pars - ang: " << std::endl;
        std::cout << angle_pars_ei_ << std::endl;

        parameters_.push_back(dist_pars_ei_.data());
        parameters_.push_back(angle_pars_ei_.data());


    }

    void addResidualBlock(BasicResidualBlock &block)
    {

        ceres::CostFunction * cost = block.getMyCost();
        problem_.AddResidualBlock(cost, NULL, this->getParameters());
    }

    void setUpProblem()
    {

        IntensityModelingFunctorMultiResiduals * f  =
                new IntensityModelingFunctorMultiResiduals(samples_.obs_, &fixed_pars_);

        this->addResidualBlock(*f);
//        for (int j = 0 ; j < obs_.size(); ++j)
//        {
//            Observation* ob = &obs_.at(j);
//            IntensityModelingFunctor * f = new IntensityModelingFunctor( ob, &fixed_pars_);

//            this->addResidualBlock(*f);
//        }

        //////// SMOOTHNESS CONSTRAIN ///////////
        FlatParametersConstrain * smoothness = new FlatParametersConstrain(0.03 * samples_.obs_.size(), &fixed_pars_);
        this->addResidualBlock(*smoothness);



    }

    size_t getNumberOfParameterBlocks()
    {
        return problem_.NumParameterBlocks();
    }

    size_t getSizeOfParameterBlock(size_t id)
    {
        std::vector<double *> pars;
        problem_.GetParameterBlocks(&pars);
        return problem_.ParameterBlockSize(pars.at(id));
    }

    void readFile(const std::string & fname);

    void solve();

    void printFullReport();

    void printAllParameters();

    double * getParametersBlock(const size_t block_id)
    {
        std::vector<double *> blocks;
        problem_.GetParameterBlocks(&blocks);

        return blocks.at(block_id);
    }

    void savePrediction(const std::string outfname) const;

    std::vector<double *> getParameters()
    {
        std::vector<double *> out;
        for (std::vector<double> v: parameters_)
        {
            out.push_back(&v[0]);
        }
        return out;
    }

    ceres::Solver::Options options_;


    ceres::Solver::Summary summary_;


    ceres::Problem problem_;

    SampledData samples_;

    IntensityModelFixedPars fixed_pars_ ;


    ParametersHolder parameters_;

    Eigen::VectorXd dist_pars_ei_;
    Eigen::VectorXd angle_pars_ei_ ;

};

}//end nspace
#endif // CALIBRATIONMANAGER_H
