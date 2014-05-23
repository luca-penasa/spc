#ifndef STRATIGRAPHICMODELBASE_H
#define STRATIGRAPHICMODELBASE_H

#include <spc/scalar_fields_generators/DynamicScalarFieldGenerator.h>

namespace spc
{

class StratigraphicModelBase: public DynamicScalarFieldGenerator
{
public:
    SPC_OBJECT(StratigraphicModelBase)

    StratigraphicModelBase() {}
};

} //end nspace
#endif // STRATIGRAPHICMODELBASE_H
