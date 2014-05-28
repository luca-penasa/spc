#ifndef SPC_SALVABLE_OBJECT_H
#define SPC_SALVABLE_OBJECT_H
#include <spc/common/macros.h>
#include <ostream>
#include <iostream>
//#include <spc/io/element_io.h>

namespace spc
{

class spcSerializableObject
{
public:
    SPC_OBJECT(spcSerializableObject)

    spcSerializableObject();

    /**
     * @brief isSPCSerializable
     * This may be overloaded for forcing an spcSerializableObject to be non-serializable
     * Normally ALL spcSerializableObjects are serializable.
     * So def is true
     * @return true if serializable
     */
    virtual bool isSPCSerializable()  const
    {
        return true;
    }

    virtual bool canBeSavedAsAscii() const
    {
        return false;
    }


    template<class Archive>
    void serialize(Archive & archive)
    {
        // nothing for now
    }


};



}//end nspace



#endif // SALVABLE_OBJECT_H
