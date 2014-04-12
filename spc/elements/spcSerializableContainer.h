#ifndef SPCSERIALIZABLECONTAINER_H
#define SPCSERIALIZABLECONTAINER_H

#include "salvable_object.h"
//#include <boost/serialization/shared_ptr.hpp>
//#include <boost/serialization/vector.hpp>

namespace spc
{


class spcSerializableContainer: public spcSerializableObject
{

public:
    typedef boost::shared_ptr<spcSerializableContainer> Ptr;
    typedef boost::shared_ptr<const spcSerializableContainer> ConstPtr;
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
//    friend class boost::serialization::access;

//    template <class Archive>
//    void serialize(Archive &ar, const unsigned int version)
//    {
//        ar & BOOST_SERIALIZATION_NVP(data_);
//        ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(spcSerializableObject);
//    }


    std::vector<spc::spcSerializableObject *> data_;

};


}//end nspace

#endif // SPCSERIALIZABLECONTAINER_H
