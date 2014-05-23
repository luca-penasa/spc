#pragma once
#ifndef SAMPLE_H
#define SAMPLE_H

#include <spc/elements/movable_element.h>
#include <spc/elements/UniversalUniqueObject.h>

#include <spc/scalar_fields_generators/StratigraphicModelBase.h>

#include <spc/tables/UnorderedDataTable.h>
#include <spc/common/macros.h>

namespace spc
{

class Sample: public PositionableElement, public UniversalUniqueObject
{

public:
    SPC_OBJECT(Sample)

    Sample(): stratigraphic_position_(spcNANMacro)
    {
        initDataRecord();
    }


    Sample(const float x, const float y, const float z): PositionableElement(x,y,z)
    {
        initDataRecord();
    }

    Sample(const Eigen::Vector3f v): PositionableElement(v)
    {
        initDataRecord();
    }

    void initDataRecord()
    {
        data_record_ = UnorderedDataRecord::Ptr(new UnorderedDataRecord);
    }


    // strat pos
    spcSetMacro(StratigraphicPosition, stratigraphic_position_, float)
    spcGetMacro(StratigraphicPosition, stratigraphic_position_, float)

    // a connected stratigraphic model
    spcSetObjectMacro(ReferenceModel, reference_model_, StratigraphicModelBase)
    spcGetObjectMacro(ReferenceModel, reference_model_, StratigraphicModelBase)

    // a connected data rcord
    spcSetObjectMacro(DataRecord, data_record_, UnorderedDataRecord)
    spcGetObjectMacro(DataRecord, data_record_, UnorderedDataRecord)

    int updateCurrentStratigraphicPositionWithModel()
    {
        stratigraphic_position_ = reference_model_->getScalarFieldValue(getPosition());
        this->modified();
        return 1;
    }

private:
    friend class cereal::access;

    template <class Archive>
    void serialize( Archive & ar )
    {
        ar( make_nvp("PositionableElement", cereal::base_class<spc::PositionableElement>( this )),
            make_nvp("UniversalUniqueObject", cereal::base_class<spc::UniversalUniqueObject>( this )));
        // to be completed with members
    }

protected:
    float stratigraphic_position_;

    StratigraphicModelBase::Ptr reference_model_;

    UnorderedDataRecord::Ptr data_record_;





};

} //end nspace

#endif // SAMPLE_H
