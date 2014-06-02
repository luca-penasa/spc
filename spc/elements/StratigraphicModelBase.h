#ifndef STRATIGRAPHICMODELBASE_H
#define STRATIGRAPHICMODELBASE_H

#include <spc/elements/VariableScalarFieldBase.h>

namespace spc
{

class StratigraphicModelBase : public VariableScalarFieldBase
{
public:
    SPC_OBJECT(StratigraphicModelBase)

    StratigraphicModelBase()
    {
    }

private:
    friend class cereal::access;

    template <class Archive> void serialize(Archive &ar)
    {
        ar(cereal::base_class<spc::VariableScalarFieldBase>(this));
    }

};

} // end nspace
#endif // STRATIGRAPHICMODELBASE_H
