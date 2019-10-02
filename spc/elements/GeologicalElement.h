#ifndef GEOLOGICALELEMENT_H
#define GEOLOGICALELEMENT_H

#include <spc/elements/GeometricElement3DBase.h>

namespace spc
{
class GeologicalElement: public GeometricElement3DBase
{
public:

    spcTypedefSharedPtrs(GeologicalElement)
    EXPOSE_TYPE


    GeologicalElement();


private:
    friend class cereal::access;

    template <class Archive> void serialize(Archive &ar, std::uint32_t const version)
    {
            ar(cereal::base_class<GeometricElement3DBase>(this));
    }


};




}// end nspace

#endif // GEOLOGICALELEMENT_H
