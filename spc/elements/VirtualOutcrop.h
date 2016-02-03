#pragma once
#ifndef VIRTUALOUTCROP_H
#define VIRTUALOUTCROP_H
#include <spc/elements/ElementBase.h>
namespace spc
{

class VirtualOutcrop: public ElementBase
{
public:
    SPC_ELEMENT(VirtualOutcrop)
    EXPOSE_TYPE

    VirtualOutcrop()
    {
    }

    ~VirtualOutcrop()
    {

    }

    VirtualOutcrop(const VirtualOutcrop & other): ElementBase(other)
    {

    }

    void solveStratigraphy();



private:
    friend class cereal::access;

    template <class Archive> void serialize(Archive &ar, const std::uint32_t version)
    {
        ar(cereal::base_class<spc::ElementBase>(this));
    }





};

}

#endif // VIRTUALOUTCROP_H
