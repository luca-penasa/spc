#pragma once
#ifndef SAMPLE_H
#define SAMPLE_H

#include <spc/elements/StratigraphicPositionableElement.h>
#include <spc/elements/VariantPropertiesRecord.h>
#include <spc/core/macros.h>
#include <spc/elements/ElementWithVariantProperties.h>

namespace spc
{

class Sample : public StratigraphicPositionableElement
{

public:
    SPC_ELEMENT(Sample)
    EXPOSE_TYPE
    Sample()
    {
    }

    Sample(const Sample &other) : StratigraphicPositionableElement(other)
    {

    }

    ~Sample()
    {

    }

    Sample(const float x, const float y, const float z)
        : StratigraphicPositionableElement(x, y, z)
    {
    }

    Sample(const Eigen::Vector3f v) : StratigraphicPositionableElement(v)
    {
    }

private:
    friend class cereal::access;

    template <class Archive> void serialize(Archive &ar)
    {
        ar(cereal::base_class<StratigraphicPositionableElement>(this));
    }
};

} // end nspace

#endif // SAMPLE_H
