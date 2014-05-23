#pragma once
#ifndef UNIVERSALUNIQUEOBJECT_H
#define UNIVERSALUNIQUEOBJECT_H

#include <spc/elements/UniversalUniqueID.h>

namespace cereal
{
class access;
}

namespace spc
{

class UniversalUniqueID;

class UniversalUniqueObject
{
public:
    UniversalUniqueObject() {}

    // the uuid element
    spcGetMacro(UniversalUUID, universal_id_, UniversalUniqueID)


protected:
    UniversalUniqueID universal_id_;

private:
    friend class cereal::access;


    template <class Archive>
    void serialize( Archive & ar )
    {
        ar( CEREAL_NVP(universal_id_) );
    }

};





} // end nspace
#endif // UNIVERSALUNIQUEOBJECT_H
