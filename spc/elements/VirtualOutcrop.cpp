#include "VirtualOutcrop.h"
#include <spc/methods/StratigraphicSolver.h>


namespace spc
{

DtiClassType VirtualOutcrop::Type ("VirtualOutcrop", &ElementBase::Type);

void VirtualOutcrop::solveStratigraphy()
{
    LOG(INFO) << "solve called!";

    spc::StratigraphicSolver solver_;

//    if ( findElementsThatAre(&spc::StratigraphicModelBase::Type).size() == 0)
//    {
//        LOG(WARNING) << "tried to solve stratigraphic constrains but no models are present in this outcrop";
//        return;
//    }

    solver_.reset();

    solver_.setInputFromChildren(getPtr());


    //        for (spc::ElementBase::Ptr el: findElementsThatAre(&spc::StratigraphicModelBase::Type))
    //        {
    //            solver_.addStratigraphicModel(spcDynamicPointerCast<spc::StratigraphicModelBase>(el));
    //            solver_.extractInputFromChildrens();
    //        }

    solver_.solve();


    LOG(INFO)<< "solve done";
}





}//end nspace



#include <spc/core/spc_cereal.hpp>
SPC_CEREAL_REGISTER_TYPE(spc::VirtualOutcrop)
