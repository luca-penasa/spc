#pragma once
#ifndef ELEMENTWITHVARIANTPROPERTIES_H
#define ELEMENTWITHVARIANTPROPERTIES_H
#include <spc/elements/VariantPropertiesRecord.h>
namespace spc
{

class ElementWithVariantProperties
{
public:
    spcTypedefSharedPtrs(ElementWithVariantProperties)

    ElementWithVariantProperties();

    VariantPropertiesRecord getVariantPropertiesRecord() const
    {
        return properties_;
    }

    VariantPropertiesRecord &getVariantPropertiesRecord()
    {
        return properties_;
    }



private:
    //! variant properties for the object
    VariantPropertiesRecord properties_;

private:
    friend class cereal::access;

    template <class Archive> void serialize(Archive &ar)
    {
        ar(CEREAL_NVP(properties_));
    }
};

} // end nspace

#endif // ELEMENTWITHVARIANTPROPERTIES_H
