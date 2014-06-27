#pragma once
#ifndef EIGENLINARFUNCTIONBASE_H
#define EIGENLINARFUNCTIONBASE_H

#include <spc/elements/EigenFunctionBase.h>

namespace spc
{

//! a polynomial function in n vars from R^n to R
//!
class EigenLinearFunctionBase : public EigenFunctionBase
{
public:
    SPC_OBJECT(EigenLinearFunctionBase)
    EXPOSE_TYPE

    EigenLinearFunctionBase(const size_t &in_dimension)
    {
        output_size_ = 1; // only R^n -> R
        input_size_ = in_dimension;
    }

    virtual Eigen::VectorXf getCoefficients() const
    {
        return coefficients_;
    }

    virtual size_t getNumberOfCoefficients()
    {
        return coefficients_.size();
    }


    virtual void setCoefficients(const Eigen::VectorXf &c)
    {
        coefficients_ = c;
    }

    //! this virtual method is what actually determines the shape of your linear
    //(polynomial) function
    virtual Eigen::VectorXf getRowOfObservationsMatrix(const Eigen::VectorXf &v)
        = 0;

    virtual Eigen::VectorXf operator()(const Eigen::VectorXf &v)
    {
        assert(v.size() == getInputSize());

        Eigen::VectorXf obs_row = this->getRowOfObservationsMatrix(v);

        float value = obs_row.dot(coefficients_);

        Eigen::VectorXf asv(1);
        asv << value;
        return asv;
    }

protected:
    Eigen::VectorXf coefficients_;


private:
    friend class cereal::access;

    template <class Archive> void serialize(Archive &ar)
    {
        ar(cereal::base_class<EigenFunctionBase>(this), CEREAL_NVP(coefficients_));
    }
};

}//end nspace
#endif
