#ifndef SPCSERIALIZABLECONTAINER_H
#define SPCSERIALIZABLECONTAINER_H

#include "SerializableObject.h"
#include <vector>
#include <cereal/cereal.hpp>

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

    void push_back(spc::spcSerializableObject::Ptr el)
    {
        data_.push_back(el);
    }

    inline size_t getNumberOfElements() const
    {
        return data_.size();
    }

    spc::spcSerializableObject::Ptr at(size_t id) const
    {
        return data_.at(id);
    }

private:
    friend class cereal::access;

    template <class Archive>
    void serialize( Archive & ar )
    {
        ar( cereal::base_class<spcSerializableObject>( this ),
            CEREAL_NVP(data_));
    }



protected:

    std::vector<spc::spcSerializableObject::Ptr> data_;

};


}//end nspace

#endif // SPCSERIALIZABLECONTAINER_H
