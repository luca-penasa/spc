#pragma once
#ifndef SAMPLE_H
#define SAMPLE_H

#include <spc/elements/MovableElement.h>
#include <spc/elements/VariantPropertiesRecord.h>
#include <spc/core/macros.h>
#include <spc/elements/ElementWithVariantProperties.h>

namespace spc
{

class Sample : public Point3D
{

public:
    SPC_ELEMENT(Sample)
    EXPOSE_TYPE
    Sample()
    {
    }

    Sample(const float x, const float y, const float z)
        : Point3D(x, y, z)
    {
    }

    Sample(const Eigen::Vector3f v) : Point3D(v)
    {
    }

private:
    friend class cereal::access;

    template <class Archive> void serialize(Archive &ar)
    {
        ar(cereal::base_class<Point3D>(this));
    }
};

} // end nspace

#endif // SAMPLE_H
