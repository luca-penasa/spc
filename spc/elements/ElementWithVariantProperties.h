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

    template <typename T> void setVariantPropertyValue(const std::string name, const T &value)
    {
        this->getVariantPropertiesRecord().property(name) = value;
    }

    VariantProperty::VarianT &variantPropertyValue(const std::string &name)
    {
        return this->getVariantPropertiesRecord().property(name).value();
    }

    template <typename T> T variantPropertyValue(const std::string name) const
    {
        return boost::get<T>(this->getVariantPropertiesRecord().property(name).value());
    }

    bool hasProperty(const std::string prop_name)
    {
        return this->getVariantPropertiesRecord().hasPropertyWithName(prop_name);
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
