#ifndef STRATIGRAPHICSOLVER_H
#define STRATIGRAPHICSOLVER_H

#include <spc/elements/SelectionRubberband.h>
#include <spc/elements/StratigraphicModelBase.h>
#include <spc/elements/StratigraphicConstrain.h>
#include <spc/elements/CloudDataSourceOnDisk.h>

namespace spc
{

class StratigraphicSolver
{
public:
    StratigraphicSolver();

    void addAreaOfInteres(SelectionRubberband::Ptr area)
    {
        areas_.push_back(area);
        LOG(INFO) << "added new interest area to the sover";
    }

    void addStratigraphicModel(StratigraphicModelBase::Ptr model)
    {
        models_.push_back(model);
        LOG(INFO) << "added new stratigraphic model to the solver";
    }

    void addStratigraphicConstrain(StratigraphicConstrain::Ptr constrain)
    {
        constrains_.push_back(constrain);
        LOG(INFO) << "added new stratigraphic constrain between areas of interest";
    }

    void assignModelToAreas()
    {
        for (StratigraphicModelBase::Ptr model: models_)
        {
            for (SelectionRubberband::Ptr area: areas_)

            {
//               area->contains(model->get)
            }
        }
    }



protected:
    std::vector<SelectionRubberband::Ptr> areas_;
    std::vector<StratigraphicModelBase::Ptr> models_;
    std::vector<StratigraphicConstrain::Ptr> constrains_;


    std::map<SelectionRubberband::Ptr, StratigraphicModelBase::Ptr> area_to_model_;


};

}

#endif // STRATIGRAPHICSOLVER_H
