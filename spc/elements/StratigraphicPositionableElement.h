#pragma once
#ifndef STRATIGRAPHICPOSITIONABLEELEMENT_H
#define STRATIGRAPHICPOSITIONABLEELEMENT_H

#include <spc/elements/MovableElement.h>
#include <spc/elements/StratigraphicModelBase.h>


namespace spc
{


class StratigraphicPositionableElement: public Point3D
{
public:

    SPC_ELEMENT(StratigraphicPositionableElement)
    EXPOSE_TYPE

    StratigraphicPositionableElement()
    {
    }

    StratigraphicPositionableElement(const StratigraphicPositionableElement & other): Point3D(other)
    {
        stratigraphic_position_ = other.stratigraphic_position_;
        manual_ = other.manual_;
        strat_model_ = other.strat_model_;
    }

    ~StratigraphicPositionableElement()
    {

    }


    StratigraphicPositionableElement(const float x, const float y, const float z): Point3D(x,y,z)
    {

    }

    StratigraphicPositionableElement(const Eigen::Vector3f point): Point3D(point)
    {

    }



    void modelFromParent()
    {
        if (this->getParent()!=NULL && this->getParent()->isA(&spc::StratigraphicModelBase::Type))
        {
            LOG(INFO) << "lading parent as model";
            strat_model_ = spcDynamicPointerCast<spc::StratigraphicModelBase>(this->getParent());
        }
    }



    spcGetObjectMacro(StratigraphicModel, strat_model_, StratigraphicModelBase)


    void setStratigraphicModel(StratigraphicModelBase::Ptr mod)
    {
        if (mod == NULL)
            this->setManual(true);
        else
            strat_model_ = mod;
    }



    spcGetMacro(Manual, manual_, bool)
    spcSetMacro(Manual, manual_, bool)

    float getSquaredResidual() const
    {
        if (!getManual())
            return 0;
        else
        {
            float out = strat_model_->predictStratigraphicPosition(this->getPosition()) - getStratigraphicPosition();
            return out*out;
        }
    }

    float getStratigraphicPosition() const
    {
        if (manual_ | strat_model_ == nullptr)
            return stratigraphic_position_;
        else
            return strat_model_->predictStratigraphicPosition(this->getPosition());
    }



    void setStratigraphicPosition(const float &sp)
    {
        stratigraphic_position_ = sp;

        LOG(INFO) << "new stratigraphic position manually set to " << stratigraphic_position_ ;
    }





private:
    float stratigraphic_position_ = 0;

    bool manual_ = true;

    StratigraphicModelBase::Ptr strat_model_;


private:
    friend class cereal::access;

    template <class Archive> void serialize(Archive &ar, const std::uint32_t version)
    {
        ar(cereal::base_class<Point3D>(this),
           CEREAL_NVP(stratigraphic_position_),
           CEREAL_NVP(strat_model_),
           CEREAL_NVP(manual_));
    }

};


}// end nspace
#endif // STRATIGRAPHICPOSITIONABLEELEMENT_H


