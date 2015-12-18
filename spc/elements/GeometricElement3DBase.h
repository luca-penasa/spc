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


    GeometricElement3DBase();

    // copy const
    GeometricElement3DBase(const GeometricElement3DBase & other): ElementBase(other)
    {

    }


private:
    friend class cereal::access;

    template <class Archive> void serialize(Archive &ar, const std::uint32_t version)
    {
        ar(cereal::base_class<ElementBase>(this)); // nothing for now
    }
};


}// end nspace

#endif // GEOMETRICELEMENT3DBASE_H
