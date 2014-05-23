#ifndef SPCSERIALIZABLECONTAINER_H
#define SPCSERIALIZABLECONTAINER_H

#include "SerializableObject.h"
#include <vector>

namespace spc
{


class spcSerializableContainer: public spcSerializableObject
{

public:

    SPC_OBJECT(spcSerializableContainer)
public:
    spcSerializableContainer()
    {

    }

    void push_back(spc::spcSerializableObject * el)
    {
        data_.push_back(el);
    }

    inline size_t getNumberOfElements() const
    {
        return data_.size();
    }

    spc::spcSerializableObject * at(size_t id) const
    {
        return data_.at(id);
    }



protected:

    std::vector<spc::spcSerializableObject *> data_;

};


}//end nspace

#endif // SPCSERIALIZABLECONTAINER_H
