#pragma once
#ifndef ICALPHFUNCTION_H
#define ICALPHFUNCTION_H
#include <spc/elements/EigenLinearFunctionBase.h>
#include <spc/elements/CalibrationFunction.h>
namespace spc
{



class ICalPHFunction : public CalibrationFunction
{
public:
    SPC_OBJECT(ICalPHFunction)
    EXPOSE_TYPE

    ICalPHFunction();

public:
    virtual Eigen::VectorXf operator()(const Eigen::VectorXf &v)
    {
        assert(v.size() >= 2);
        assert(coefficients_.size() == getNumberOfCoefficients());

        float r = v(0);
        float cosw = v(1);
        Eigen::VectorXf obs_row = getRowOfObservationsMatrix(v);

        float value = obs_row.dot(getCoefficients());

        Eigen::VectorXf asv(1);
        asv << value;
        return asv;
    }

    virtual size_t getNumberOfCoefficients() const
    {
        return (overall_degree_ + 1) * (parameters_degree_ + 1);
    }

    virtual Eigen::VectorXf getRowOfObservationsMatrix(const Eigen::VectorXf &v)
    {
        Eigen::VectorXf r_c = getRContrib(v(0));
        Eigen::VectorXf cosw_c = getCosWContrib(v(1));

        Eigen::VectorXf vec = r_c.array() * cosw_c.array();

        return vec;
    }

private:
    Eigen::VectorXf getRContrib(const float &r) const
    {
        Eigen::VectorXf v(getNumberOfCoefficients());
        for (int i = 0; i < overall_degree_ + 1; ++i) {
            for (int j = 0; j < parameters_degree_ +1; ++j) {
                v(i *(parameters_degree_+1) + j) = pow(r, j);
            }
        }

        return v;
    }

    Eigen::VectorXf getCosWContrib(const float &cosw) const
    {
        Eigen::VectorXf v(getNumberOfCoefficients());
        for (int i = 0; i < overall_degree_ +1; ++i) {
            for (int j = 0; j < parameters_degree_+1; ++j) {
                v(i *(parameters_degree_+1) + j) = pow(cosw, i);

            }
        }
        return v;
    }

protected:
    size_t overall_degree_ = 3;
    size_t parameters_degree_ = 3;

private:
    friend class cereal::access;

    template <class Archive> void serialize(Archive &ar)
    {
        ar(cereal::base_class<EigenFunctionBase>(this), CEREAL_NVP(coefficients_));
    }

};
} // end nspace
#endif // ICALPHFUNCTION_H
