#pragma once
#ifndef SPC_OBJECT_H
#define SPC_OBJECT_H

#include <spc/elements/macros.h>
#include <cereal/cereal.hpp>
#include <spc/elements/UniversalUniqueID.h>

namespace spc
{

class ElementBase
{
public:
    SPC_OBJECT(ElementBase)

    ElementBase() : modified_(false)
    {
    }

    // this may be useful in a future
    virtual void modified();

    virtual void update();

    /**
     * @brief isSPCSerializable
     * This may be overloaded for forcing an spcSerializableObject to be
     * non-serializable
     * Normally ALL spcObjects are serializable.
     * So def is true
     * @return true if serializable
     */
    virtual bool isSPCSerializable() const;

    virtual bool canBeSavedAsAscii() const;

    virtual UniversalUniqueID getUniversalUUID() const;

protected:
    bool modified_;
    UniversalUniqueID universal_id_;

private:
    friend class cereal::access;

    template <class Archive> void serialize(Archive &ar)
    {
        ar(CEREAL_NVP(modified_), CEREAL_NVP(universal_id_));
    }
};

} // end nspace

#endif // ELEMENT_BASE_H
