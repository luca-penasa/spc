#pragma once
#ifndef VIRTUALOUTCROP_H
#define VIRTUALOUTCROP_H
#include <spc/elements/ElementBase.h>
#include <spc/methods/StratigraphicSolver.h>
namespace spc
{

class VirtualOutcrop: public ElementBase
{
public:
    SPC_ELEMENT(VirtualOutcrop)
    EXPOSE_TYPE

    VirtualOutcrop()
    {
    }

    ~VirtualOutcrop()
    {

    }

    VirtualOutcrop(const VirtualOutcrop & other): ElementBase(other)
    {

    }

    void solveStratigraphy()
    {
        LOG(INFO) << "solve called!";

       if ( findElementsThatAre(&spc::StratigraphicModelBase::Type).size() == 0)
       {
           LOG(WARNING) << "tried to solve stratigraphic constrains but no models are present in this outcrop";
           return;
       }

        solver_.clear();
        for (spc::ElementBase::Ptr el: findElementsThatAre(&spc::StratigraphicModelBase::Type))
        {
            solver_.addStratigraphicModel(spcDynamicPointerCast<spc::StratigraphicModelBase>(el));
            solver_.extractInputFromChildrens();
        }

        solver_.solve();


        LOG(INFO)<< "solve done";
    }



private:
    friend class cereal::access;

    template <class Archive> void serialize(Archive &ar, const std::uint32_t version)
    {
        ar(cereal::base_class<spc::ElementBase>(this));
    }


protected:
    spc::StratigraphicSolver solver_;

};

}

#endif // VIRTUALOUTCROP_H
