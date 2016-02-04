#pragma once
#ifndef STRATIGRAPHICPOSITIONABLEELEMENT_H
#define STRATIGRAPHICPOSITIONABLEELEMENT_H

#include <spc/elements/GeologicalElement.h>
#include <spc/elements/StratigraphicModelBase.h>

namespace spc {
/**
 * @brief The StratigraphicPositionableElement class
 *  \todo we should have a "Nominal Stratigraphic Position" and a "Modeled Stratigraphic Position",
 *  \todo also supporting the EXTENDED types must be done
 */
class StratigraphicPositionableElement : public GeologicalElement {
public:
    spcTypedefSharedPtrs(StratigraphicPositionableElement)
        EXPOSE_TYPE

        enum POSITIONABLE_TYPE { SINGLE_POSITION = 0,
            EXTENDED };

    StratigraphicPositionableElement()
    {
    }

    StratigraphicPositionableElement(const StratigraphicPositionableElement& other)
    {
        user_stratigraphic_position_ = other.user_stratigraphic_position_;
        manual_ = other.manual_;
        strat_model_ = other.strat_model_;
    }

    ~StratigraphicPositionableElement()
    {
    }

    bool hasModel() const
    {
        if (strat_model_ == nullptr)
            return false;
        else
            return true;
    }

    void modelFromParent()
    {
        if (this->getParent() != nullptr && this->getParent()->isA(&spc::StratigraphicModelBase::Type)) {
            LOG(INFO) << "loading parent as model";
            strat_model_ = spcDynamicPointerCast<spc::StratigraphicModelBase>(this->getParent());
        }
    }

    spcGetObjectMacro(StratigraphicModel, strat_model_, StratigraphicModelBase)

        void setStratigraphicModel(const StratigraphicModelBase::Ptr& mod)
    {
        if (mod == nullptr)
            return;
        else
            strat_model_ = mod;
    }

    spcSetGetMacro(Manual, manual_, bool)

        spcGetMacro(PositionableType, positionable_type_, POSITIONABLE_TYPE)

            spcSetGetMacro(UserStratigraphicPosition, user_stratigraphic_position_, float)

                float getSquaredResidual() const
    {
        if (!getManual())
            return 0;
        else {
            float out = predictStratigraphicPositionFromModel() - getStratigraphicPosition();
            return out * out;
        }
    }

    /** a stratigraphic positionable element needs a method to predict its position
     * in stratigraphy
     **/
    virtual float predictStratigraphicPositionFromModel() const = 0;

    float getStratigraphicPosition() const
    {
        if (getManual()) // in manual mode always give the user chosen sp
            return user_stratigraphic_position_;

        else if (!hasModel()) // if we dont have a model just return nan;
            return spcNANMacro;

        else // we are not in manual and we have a strat model.
            return predictStratigraphicPositionFromModel();
    }


protected:
    float user_stratigraphic_position_ = spcNANMacro;

    bool manual_ = true;

    StratigraphicModelBase::Ptr strat_model_ = nullptr;

    POSITIONABLE_TYPE positionable_type_ = SINGLE_POSITION; // by default



private:


    friend class cereal::access;

    template <class Archive>
    void serialize(Archive& ar, const std::uint32_t version)
    {
        ar(cereal::base_class<GeologicalElement>(this),
            CEREAL_NVP(user_stratigraphic_position_),
            CEREAL_NVP(strat_model_),
            CEREAL_NVP(manual_));

        if (version >= 2) {
            ar(CEREAL_NVP(positionable_type_));
        }
    }
};

} // end nspace

CEREAL_CLASS_VERSION(spc::StratigraphicPositionableElement, 2)

#endif // STRATIGRAPHICPOSITIONABLEELEMENT_H
