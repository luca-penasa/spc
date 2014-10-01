#ifndef RESIDUALBLOCKS_H
#define RESIDUALBLOCKS_H

#include <spc/calibration/CalibrationFactors.h>
#include <spc/calibration/Observations.h>

namespace spc
{



struct SmoothnessConstrain
{
    SmoothnessConstrain(const double alpha, spc::FixedModelPars const * fixed_pars ): alpha_(alpha)
    {
        fixed_pars_ = fixed_pars;
    }

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

    template <typename T>
    T const * const *
    getParameters();




    double alpha_;
    const spc::FixedModelPars * fixed_pars_;

};





struct ResidualFunctor
{
    ResidualFunctor(spc::Observation * ob,
                    spc::FixedModelPars * fixed_pars)


    {
        observation_ = ob;
        fixed_pars_ = fixed_pars;
    }

    template<typename T>
    bool operator()(T const* const* parameters, T* residual) const
    {

        T prediction = predict_intensities(*observation_, *fixed_pars_, parameters);
        residual[0] = T(observation_->intensity) - prediction;


        return true;
    }

private:
    spc::Observation * observation_;

    spc::FixedModelPars * fixed_pars_;

};

}//end nspace

#endif // RESIDUALBLOCKS_H
