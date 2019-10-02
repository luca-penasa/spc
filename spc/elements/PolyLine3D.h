#pragma once
#ifndef POLYLINE3D_H
#define POLYLINE3D_H

#include <spc/elements/templated/Polyline.hpp>

#include <spc/elements/GeometricElement3DBase.h>


namespace spc
{


class PolyLine3D: public PolyLine<float, 3>, public GeometricElement3DBase
{

public:
    SPC_ELEMENT(PolyLine3D)
    EXPOSE_TYPE


    PolyLine3D(): PolyLine<float, 3>()
    {

    }

    PolyLine3D(const PolyLine3D &other): PolyLine<float, 3>(other)
    {

    }

    PolyLine3D(const PointSetT &other): PolyLine<float, 3>(other)
    {

    }

    // GeometricElement3DBase interface
public:
    virtual void applyTransform(const GeometricElement3DBase::TransformT &transform) override;


private:
    friend class cereal::access;

    template <class Archive>
    void serialize(Archive& ar, const std::uint32_t version)
    {
        if (version == 0 ) // we migth still have older versions around
        {
            ar(data_);
        }
        else
        {
            ar(cereal::base_class<spc::GeometricElement3DBase>(this));
            ar(cereal::base_class<PolyLine<float, 3>>(this));
        }


    }
};

//void PolyLine3D::applyTransform(const TransformT &transform);

//class PolyLine2D: public PolyLine<float, 2>
//{

//};

//CEREAL_CLASS_VERSION(PolyLine3D, 2)



//typedef  PolyLine<float, 3> PolyLine3D;
typedef PolyLine<float, 2> PolyLine2D;


}
#endif // POLYLINE3D_H
