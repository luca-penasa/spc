#include "ICalPHFunction.h"
namespace spc
{

DtiClassType ICalPHFunction::Type = DtiClassType("ICalPHFunction", &EigenLinearFunctionBase::Type);

ICalPHFunction::ICalPHFunction(): EigenLinearFunctionBase(2)
{
}

}//end nspace
