#ifndef LINEARGEOLOGICALFEATURE_H
#define LINEARGEOLOGICALFEATURE_H

#include <spc/elements/PolyLine3D.h>
#include <spc/elements/StratigraphicPositionableElement.h>
namespace spc
{

class LinearGeologicalFeature: public StratigraphicPositionableElement
{
public:

    SPC_ELEMENT(LinearGeologicalFeature)
    EXPOSE_TYPE

    LinearGeologicalFeature();


    PolyLine3D & getPolyline()
    {
        return polyline_;
    }

    spcSetMacro(Polyline, polyline_, PolyLine3D)




private:
    friend class cereal::access;

    template <class Archive>
    void serialize(Archive& ar, const std::uint32_t version)
    {
        ar(cereal::base_class<spc::StratigraphicPositionableElement>(this));
        ar(CEREAL_NVP(polyline_));
    }

protected:
    PolyLine3D polyline_;

    // StratigraphicPositionableElement interface
public:
    virtual float predictStratigraphicPositionFromModel() const override;

    // GeometricElement3DBase interface
public:
    virtual void applyTransform(const TransformT &transform) override;
};




}// end nspace
#endif // LINEARGEOLOGICALFEATURE_H
