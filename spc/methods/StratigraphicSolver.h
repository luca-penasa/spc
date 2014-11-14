#ifndef STRATIGRAPHICSOLVER_H
#define STRATIGRAPHICSOLVER_H

//#include <spc/elements/StratigraphicPositionableElement.h>
#include <spc/elements/StratigraphicModelBase.h>
#include <spc/elements/StratigraphicConstrain.h>
#include <spc/elements/StratigraphicPositionableElement.h>


namespace spc
{

class StratigraphicSolver
{
public:
    StratigraphicSolver();

    void extractInputFromChildrens()
    {

        constrains_.clear();
        positionables_.clear();

        for(StratigraphicModelBase::Ptr mod: models_)
        {
            std::vector<spc::ElementBase::Ptr> pos = getChildsThatAre(&spc::StratigraphicPositionableElement::Type);

            for (spc::ElementBase::Ptr el: pos)
            {
                spc::StratigraphicPositionableElement::Ptr p = spcDynamicPointerCast<spc::StratigraphicPositionableElement>(el);
                if (p->getManual()) //! < only elements set to manual can provide a constrain
                {
                    positionables_.push_back(p);
                }
            }

            std::vector<spc::ElementBase::Ptr> constrains = getChildsThatAre(&spc::StratigraphicConstrain::Type);


            for (ElementBase::Ptr el: constrains)
            {
                spc::StratigraphicConstrain::Ptr p = spcDynamicPointerCast<StratigraphicConstrain>(el);




            }



        }
    }





protected:
    std::vector<StratigraphicModelBase::Ptr> models_;
    std::vector<StratigraphicConstrain::Ptr> constrains_;
    std::vector<StratigraphicPositionableElement::Ptr> positionables_;







};

}

#endif // STRATIGRAPHICSOLVER_H
