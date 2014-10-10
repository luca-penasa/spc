#ifndef RESIDUALBLOCKS_H
#define RESIDUALBLOCKS_H

//#include <spc/calibration/BasicResidualBlock.h>
//#include <spc/calibration/Observations.h>
#include <spc/calibration/ModelingFunctions.h>
namespace spc
{

class FlatParametersConstrain: public BasicResidualBlock
{
public:

    typedef ceres::DynamicAutoDiffCostFunction< FlatParametersConstrain, 4> mycostT;

    FlatParametersConstrain(const double alpha,
                            std::vector<ParameterBlock *> to_flatten

                            ): alpha_(alpha)
    {

//        initMyBlocks();

        blocks_= to_flatten;
        // now create the cost object
        cost_  =new mycostT ( this );


        size_t n_res = 0;
        for (ParameterBlock * block: blocks_)
        {
            cost_->AddParameterBlock(block->getBlockSize());
            n_res += block->getBlockSize();
        }

        cost_->SetNumResiduals(n_res);

        updateNameToBlock();
    }

    ceres::CostFunction * getMyCost()
    {
        return cost_;
    }




    template<typename T>
    bool operator()(T const* const* parameters, T* residual)
    {

        size_t current_residual = 0;
        for (int i = 0 ; i < blocks_.size(); ++i)
        {
            ParameterBlock * block = blocks_.at(i);

            Eigen::Matrix<T, -1, -1> mat = this->remapFromPointerAndBlock<T>(parameters, block);

            for (int j = 0; j < mat.size(); ++j)
            {
                T value = mat.array()(j);
                residual[current_residual++] = value * T(alpha_);
            }
        }
        return true;
    }



    double alpha_;

    mycostT * cost_;


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

    IntensityModelingFunctorMultiResiduals(std::vector<spc::Observation> &ob)


    {
        observations_ = &ob;

          cost_  =new mycostT ( this );

          initMyBlocks();

          size_t n_res = 0;
          for (ParameterBlock * block: blocks_)
              if (block->isEnabled())
                cost_->AddParameterBlock(block->getBlockSize());

          cost_->SetNumResiduals(observations_->size());

          updateNameToBlock();

    }

    template<typename T>
    bool operator()(T const* const* parameters, T* residual) const
    {

        size_t i = 0;
        for (const Observation & ob : *observations_)
        {
            T prediction = predict_intensities(ob, *this, parameters);
            residual[i++] = T(ob.intensity) - prediction;
        }

        return true;
    }

private:
    std::vector<spc::Observation> * observations_;


    // BasicResidualBlock interface
public:


    void initMyBlocks()
    {

        ParameterBlock * knots_angle  = new ParameterBlock(5, "knots_angle");
        knots_angle->getData()(0) = 0;
        knots_angle->getData()(1) = 25;
        knots_angle->getData()(2) = 50;
        knots_angle->getData()(3) = 75;
        knots_angle->getData()(4) = 90;
        knots_angle->disable();





        ParameterBlock * knots_distance  = new ParameterBlock(5, "knots_distance");
        knots_distance->getData()(0) = 0;
        knots_distance->getData()(1) = 35;
        knots_distance->getData()(2) = 70;
        knots_distance->getData()(3) = 105;
        knots_distance->getData()(4) = 140;
        knots_distance->disable();


        ParameterBlock * sigma_angle = new ParameterBlock(1, "sigma_angle");
        sigma_angle->disable();
        sigma_angle->getData()(0) = 25;

        ParameterBlock * sigma_distance = new ParameterBlock(1, "sigma_distance");
        sigma_distance->disable();
        sigma_distance->getData()(0) = 70;

        ParameterBlock * coeff_angle = new ParameterBlock(5, "coeff_angle");
        ParameterBlock * coeff_distance = new ParameterBlock(5, "coeff_distance");

        blocks_.push_back(knots_angle);
        blocks_.push_back(knots_distance);
        blocks_.push_back(sigma_angle);
        blocks_.push_back(sigma_distance);
        blocks_.push_back(coeff_angle);
        blocks_.push_back(coeff_distance);


//        parameter_descriptor_->pushBack(blocks_);





    }



    ceres::CostFunction *getMyCost()
    {
        return cost_;
    }


    mycostT * cost_;



};

}//end nspace

#endif // RESIDUALBLOCKS_H
