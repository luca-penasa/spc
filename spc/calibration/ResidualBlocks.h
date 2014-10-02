#ifndef RESIDUALBLOCKS_H
#define RESIDUALBLOCKS_H

#include <spc/calibration/CalibrationFactors.h>
#include <spc/calibration/Observations.h>
#include <spc/calibration/ModelingFunctions.h>
#include <ceres/ceres.h>

#include <spc/calibration/IntensityModelFixedPars.h>

namespace spc
{

class BasicResidualBlock
{
public:
    virtual std::vector<size_t> getSizesOfParametersBlocks() const = 0;

    virtual size_t getNumberOfResiduals() const = 0;

    virtual ceres::CostFunction * getMyCost()  = 0;

    virtual void addParameters (std::vector<std::vector<double>> & overall_pars) = 0;







};

class FlatParametersConstrain: public BasicResidualBlock
{
public:

    typedef ceres::DynamicAutoDiffCostFunction< FlatParametersConstrain, 4> mycostT;

    FlatParametersConstrain(const double alpha, spc::IntensityModelFixedPars const * fixed_pars ): alpha_(alpha)
    {
        fixed_pars_ = fixed_pars;
    }

    ceres::CostFunction * getMyCost()
    {
        mycostT * cost  =new mycostT ( this );

        for (size_t s: getSizesOfParametersBlocks())
        {
            cost->AddParameterBlock(s);
        }

        cost->SetNumResiduals(getNumberOfResiduals());

        return cost;
    }

    virtual void addParameters(std::vector<std::vector<double>> & overall_pars)
    {
        // we do no need to add parameters really.
    }

    void addWeightBlock( size_t block_id)
    {

        weigthed_blocks_.push_back(block_id);
    }

    std::vector<size_t> weigthed_blocks_;

    template<typename T>
    bool operator()(T const* const* parameters, T* residual)
    {
        size_t d_s = fixed_pars_->getNumberOfDistanceKnots();
        size_t a_s = fixed_pars_->getNumberOfAngleKnots();

        for (int i = 0; i < d_s; ++i)
        {
            residual[i] = alpha_ * parameters[0][i];
        }

        for (int i = 0; i < a_s; ++i)
        {
            residual[i + d_s] = alpha_ * parameters[1][i];
        }

        return true;
    }

    size_t getNumberOfResiduals() const
    {
        return fixed_pars_->getNumberOfAngleKnots() + fixed_pars_->getNumberOfDistanceKnots();
    }


    // BasicResidualBlock interface
    std::vector<size_t> getSizesOfParametersBlocks() const
    {
        return {fixed_pars_->getNumberOfDistanceKnots(), fixed_pars_->getNumberOfAngleKnots()};
    }

    double alpha_;
    const spc::IntensityModelFixedPars * fixed_pars_;


};





//class IntensityModelingFunctor: public BasicResidualBlock
//{
//public:
//    typedef ceres::DynamicAutoDiffCostFunction< IntensityModelingFunctor, 4> mycostT;

//    IntensityModelingFunctor(spc::Observation * ob,
//                             spc::IntensityModelFixedPars * fixed_pars                             )


//    {
//        observation_ = ob;
//        fixed_pars_ = fixed_pars;
//    }

//    template<typename T>
//    bool operator()(T const* const* parameters, T* residual) const
//    {

//        T prediction = predict_intensities(*observation_, *fixed_pars_, parameters);
//        residual[0] = T(observation_->intensity) - prediction;


//        return true;
//    }

//private:
//    spc::Observation * observation_;

//    spc::IntensityModelFixedPars * fixed_pars_;


//    // BasicResidualBlock interface
//public:
//    std::vector<size_t> getSizesOfParametersBlocks() const
//    {
//        return {fixed_pars_->n_dist_pars, fixed_pars_->n_ang_pars};
//    }
//    size_t getNumberOfResiduals() const
//    {
//        return 1;
//    }
//    ceres::CostFunction *getMyCost()
//    {

//        mycostT * cost  =new mycostT ( this );

//        for (size_t s: getSizesOfParametersBlocks())
//        {
//            cost->AddParameterBlock(s);
//        }

//        cost->SetNumResiduals(getNumberOfResiduals());

//        return cost;
//    }
//};


class IntensityModelingFunctorMultiResiduals: public BasicResidualBlock
{
public:
    typedef ceres::DynamicAutoDiffCostFunction< IntensityModelingFunctorMultiResiduals, 4> mycostT;

    IntensityModelingFunctorMultiResiduals(std::vector<spc::Observation> &ob,
                                           spc::IntensityModelFixedPars * fixed_pars)


    {
        observations_ = &ob;
        fixed_pars_ = fixed_pars;
    }

    template<typename T>
    bool operator()(T const* const* parameters, T* residual) const
    {

        size_t i = 0;
        for (const Observation & ob : *observations_)
        {
            T prediction = predict_intensities(ob, *fixed_pars_, parameters);
            residual[i++] = T(ob.intensity) - prediction;
        }

        return true;
    }

private:
    std::vector<spc::Observation> * observations_;

    spc::IntensityModelFixedPars * fixed_pars_;


    // BasicResidualBlock interface
public:
    std::vector<size_t> getSizesOfParametersBlocks() const
    {
        return {fixed_pars_->n_dist_pars, fixed_pars_->n_ang_pars};
    }
    size_t getNumberOfResiduals() const
    {
        return observations_->size();
    }
    ceres::CostFunction *getMyCost()
    {

        mycostT * cost  =new mycostT ( this );

        for (size_t s: getSizesOfParametersBlocks())
        {
            cost->AddParameterBlock(s);
        }

        cost->SetNumResiduals(getNumberOfResiduals());

        return cost;
    }


};

}//end nspace

#endif // RESIDUALBLOCKS_H
