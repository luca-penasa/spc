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


#include <spc/calibration/ParametersBlock.h>
namespace spc
{

class CalibratorManager
{
public:
    CalibratorManager(char ** argv);

    void setSolverOptions();

    void addResidualBlock(BasicResidualBlock &block)
    {

        ceres::CostFunction * cost = block.getMyCost();


        std::vector<double *> parameters = block.getMyActiveParameters();


        for (ParameterBlock * b: block.getMyBlocks())
        {
            std::string name = b->getBlockName();
            std::cout << "my block: " << name << std::endl;
        }
        std::cout << "adding " << parameters.size() << "blocks to problem" << std::endl;
        problem_.AddResidualBlock(cost, NULL, parameters);


    }

    void setUpProblem()
    {

        IntensityModelingFunctorMultiResiduals * f  =
                new IntensityModelingFunctorMultiResiduals(samples_.obs_);



        this->addResidualBlock(*f);

        predictor_block_ = f;


//        //////// SMOOTHNESS CONSTRAIN /////////// ->  keep near zero the RBF parameters!
//        FlatParametersConstrain * smoothness = new FlatParametersConstrain(1 * samples_.obs_.size(),
//        f->getParametersBlocksByName({"coeff_angle", "coeff_distance"}));

//        this->addResidualBlock(*smoothness);


    }

    BasicResidualBlock * getPredictorBlock() const
    {
        return predictor_block_;
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



    BasicResidualBlock * predictor_block_;
    ceres::Solver::Options options_;


    ceres::Solver::Summary summary_;


    ceres::Problem problem_;

    SampledData samples_;




};

}//end nspace
#endif // CALIBRATIONMANAGER_H
