#ifndef RESIDUALBLOCKS_H
#define RESIDUALBLOCKS_H

//#include <spc/ceres_calibration/BasicResidualBlock.h>
//#include <spc/ceres_calibration/Observations.h>
#include <spc/ceres_calibration/ModelingFunctions.h>
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

        //initMyBlocks();

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
            if (ob.distance >=40)
            {
            T prediction = predict_intensities(ob, *this, parameters);
            residual[i++] = T(ob.intensity) - prediction;
            }
            else
            {
                residual[i++] = T(0);
            }
        }

        return true;
    }

private:
    std::vector<spc::Observation> * observations_;


    // BasicResidualBlock interface
public:


    void initMyBlocks()
    {


        ///// standard laws-parameters

        ParameterBlock *angle_cos_proportion  = new ParameterBlock(1, "angle_cos_proportion");
//        angle_cos_proportion->disable();

        ParameterBlock *angle_slope  = new ParameterBlock(1, "angle_slope");
        angle_slope->getData()(0) = 0;
//        angle_slope->disable();

        ParameterBlock *distance_exponential  = new ParameterBlock(1, "distance_exponential");
        distance_exponential->getData()(0) = 2;

//        distance_exponential->disable();


        ParameterBlock *overall_multiplier  = new ParameterBlock(1, "overall_multiplier");

        ParameterBlock *overall_shift  = new ParameterBlock(1, "overall_shift");
        overall_shift->getData()(0) = 1;

        
//        ParameterBlock * per_cloud_multiplier = new ParameterBlock()

        blocks_.push_back(overall_shift);

        blocks_.push_back(angle_cos_proportion);
        blocks_.push_back(angle_slope);
        blocks_.push_back(distance_exponential);

        blocks_.push_back(overall_multiplier);







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
        sigma_distance->getData()(0) = 35;

        ParameterBlock * coeff_angle = new ParameterBlock(5, "coeff_angle");
        ParameterBlock * coeff_distance = new ParameterBlock(5, "coeff_distance");

        //        blocks_.push_back(knots_angle);
        //        blocks_.push_back(knots_distance);
        //        blocks_.push_back(sigma_angle);
        //        blocks_.push_back(sigma_distance);
        //        blocks_.push_back(coeff_angle);
        //        blocks_.push_back(coeff_distance);


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
