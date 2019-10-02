#pragma once
#ifndef GEOMETRICELEMENT3DBASE_H
#define GEOMETRICELEMENT3DBASE_H

#include <spc/core/ElementBase.h>
#include <spc/core/spc_eigen.h>

namespace spc {
/**
 * @brief The GeometricElement3DBase class is an element which has
 * three-d meaning and possibly a representation in R^3
 */
class GeometricElement3DBase : public ElementBase {
public:
    spcTypedefSharedPtrs(GeometricElement3DBase)
        EXPOSE_TYPE_BASE

        typedef Eigen::Affine3f TransformT;

    GeometricElement3DBase()
    {
    }

    // copy const
    GeometricElement3DBase(const GeometricElement3DBase& other)
        : ElementBase(other)
    {
    }

    virtual void applyTransform(const TransformT& transform) = 0;

private:
    friend class cereal::access;

    template <class Archive>
    void serialize(Archive& ar, const std::uint32_t version)
    {
        ar(cereal::base_class<ElementBase>(this)); // nothing for now
    }
};

} // end nspace

#endif // GEOMETRICELEMENT3DBASE_H
