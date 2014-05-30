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
};

} // end nspace
#endif // STRATIGRAPHICMODELBASE_H
