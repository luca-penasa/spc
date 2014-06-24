#pragma once
#ifndef SPC_OBJECT_H
#define SPC_OBJECT_H

#include <spc/elements/macros.h>
#include <cereal/cereal.hpp>
#include <spc/elements/UniversalUniqueID.h>
#include <spc/elements/SerializableInterface.h>
#include <spc/elements/ElementWithVariantProperties.h>

namespace spc
{

class ElementBase : public ISerializable, public ElementWithVariantProperties
{
public:
    SPC_OBJECT(ElementBase)
    EXPOSE_TYPE
    ElementBase() : modified_(false)
    {
    }

    // this may be useful in a future
    virtual void modified();

    virtual void update();

    /// this should also be present in the interface for elements with variant
    /// properties
    virtual bool hasVariantProperties() const
    {
        return true;
    }

    virtual UniversalUniqueID getUniversalUUID() const;

protected:
    bool modified_;
    UniversalUniqueID universal_id_;

private:
    friend class cereal::access;

    template <class Archive> void serialize(Archive &ar)
    {
        ar(cereal::base_class<ElementWithVariantProperties>(this),
           CEREAL_NVP(modified_), CEREAL_NVP(universal_id_));
    }

    // SerializableInterface interface
public:
    virtual bool isSerializable() const
    {
        return true;
    }
    virtual bool isAsciiSerializable() const
    {
        return false;
    }

    // ElementWithVariantProperties interface
public:
    virtual bool hasVariantProperties()
    {
        return true;
    }
};

} // end nspace

#endif // ELEMENT_BASE_H
