#ifndef GEOMETRICELEMENT3DBASE_H
#define GEOMETRICELEMENT3DBASE_H

#include <spc/elements/ElementBase.h>

namespace spc
{
/**
 * @brief The GeometricElement3DBase class is an element which has
 * three-d meanin and possibly representation in R^3
 */
class GeometricElement3DBase: public ElementBase
{
public:

    spcTypedefSharedPtrs(GeometricElement3DBase)
    EXPOSE_TYPE_BASE

    typedef Eigen::Affine3f TransformT;

    GeometricElement3DBase()
    {
//        transform_ = TransformT::Identity();
    }

    // copy const
    GeometricElement3DBase(const GeometricElement3DBase & other): ElementBase(other)
    {
//        transform_ = TransformT::Identity();
    }

//    void setTransform(const TransformT  &transform)
//    {
////        transform_ = transform;
//    }

    virtual void applyTransform(const TransformT  &transform) = 0;





private:
    friend class cereal::access;

    template <class Archive> void serialize(Archive &ar, const std::uint32_t version)
    {
        ar(cereal::base_class<ElementBase>(this)); // nothing for now
    }

//protected:
//    TransformT transform_;


};


}// end nspace

#endif // GEOMETRICELEMENT3DBASE_H
