#ifndef ELEMENT_BASE_H
#define ELEMENT_BASE_H

#include <spc/common/macros.h>

#include <spc/elements/ModificableElement.h>
#include <spc/elements/SerializableObject.h>

#include <cereal/cereal.hpp>

namespace spc
{


class spcObject: public spcSerializableObject, public ModificableElement
{
public:


    spcTypedefSharedPtrs(spcObject)

    spcObject () {}

    virtual std::string getClassName() const = 0;

private:
    friend class cereal::access;

    template <class Archive>
    void serialize( Archive & ar)
    {
        //nothing for now
    }
};

}//end nspace



#endif // ELEMENT_BASE_H
