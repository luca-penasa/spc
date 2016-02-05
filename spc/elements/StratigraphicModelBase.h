#pragma once
#ifndef STRATIGRAPHICMODELBASE_H
#define STRATIGRAPHICMODELBASE_H

#include <spc/elements/VariableScalarFieldBase.h>

namespace spc
{

class StratigraphicModelBase : public VariableScalarFieldBase
{
public:
    spcTypedefSharedPtrs(StratigraphicModelBase)
    EXPOSE_TYPE
    StratigraphicModelBase()
    {
    }

    StratigraphicModelBase(const StratigraphicModelBase & other): VariableScalarFieldBase(other)
    {
        stratigraphic_shift_ = other.getStratigraphicShift();
    }
    ~StratigraphicModelBase()
    {

    }

    virtual float predictStratigraphicPosition(const Eigen::Vector3f & pos) const = 0;


    void addShift(const float shift)
    {
        stratigraphic_shift_ += shift;
    }

    spcSetGetMacro(ElasticParameter, elastic_parameter_, float)

    spcSetGetMacro(StratigraphicShift, stratigraphic_shift_, float)

    spcSetGetMacro(IsElastic, is_elastic_, bool)

    spcSetGetMacro(IsFreezed, is_freezed_, bool)



private:
    friend class cereal::access;

    template <class Archive> void serialize(Archive &ar, std::uint32_t const version)
    {
        ar(cereal::base_class<spc::VariableScalarFieldBase>(this),
           CEREAL_NVP(stratigraphic_shift_));


        if (version >= 3)
        {
            CEREAL_NVP(elastic_parameter_);
            CEREAL_NVP(is_elastic_);
            CEREAL_NVP(is_freezed_);

        }


    }

protected:
    float stratigraphic_shift_ = 0.0f;

    float elastic_parameter_ = 1;

    bool is_elastic_ = false;

    bool is_freezed_ = false;
};
} // end nspace

CEREAL_CLASS_VERSION(spc::StratigraphicModelBase, 3)

#endif // STRATIGRAPHICMODELBASE_H
