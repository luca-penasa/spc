#include "StratigraphicConstrain.h"
#include <list>
namespace spc
{

DtiClassType StratigraphicConstrain::Type ("StratigraphicConstrain", &ElementBase::Type);



} // end nspace

namespace cereal
{
template <class Archive>
struct specialize<Archive, spc::StratigraphicConstrain, cereal::specialization::member_load_save> {};
// cereal no longer has any ambiguity when serializing MyDerived
}


#include <spc/core/spc_cereal.hpp>
SPC_CEREAL_REGISTER_TYPE(spc::StratigraphicConstrain)
