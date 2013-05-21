#include "stratigraphic_model_base.h"

namespace spc
{

template <typename ScalarT>
StratigraphicModelBase<ScalarT>::StratigraphicModelBase()
{
}


/// INST

template class StratigraphicModelBase<float>;
template class StratigraphicModelBase<double>;
} //end nspace
