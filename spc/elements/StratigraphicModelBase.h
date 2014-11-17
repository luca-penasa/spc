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


    float getStratigraphicShift() const
    {
        return stratigraphic_shift_;
    }

    void setStratigraphicShift(const float stratigraphic_shift)
    {
        stratigraphic_shift_ = stratigraphic_shift;
    }

    void addShift(const float shift)
    {
        stratigraphic_shift_ += shift;
    }

private:
    friend class cereal::access;

    template <class Archive> void serialize(Archive &ar)
    {
        ar(cereal::base_class<spc::VariableScalarFieldBase>(this),
           CEREAL_NVP(stratigraphic_shift_));
    }

protected:
    float stratigraphic_shift_ = 0.0f;
};

} // end nspace
#endif // STRATIGRAPHICMODELBASE_H
