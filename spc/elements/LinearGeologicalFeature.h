#ifndef LINEARGEOLOGICALFEATURE_H
#define LINEARGEOLOGICALFEATURE_H

#include <spc/elements/PolyLine3D.h>
namespace spc
{

class LinearGeologicalFeature: public PolyLine3D
{
public:

    SPC_ELEMENT(LinearGeologicalFeature)
    EXPOSE_TYPE

    LinearGeologicalFeature();

private:
    friend class cereal::access;

    template <class Archive>
    void serialize(Archive& ar, const std::uint32_t version)
    {
        ar(cereal::base_class<spc::PolyLine3D>(this));
    }
};


}// end nspace
#endif // LINEARGEOLOGICALFEATURE_H
