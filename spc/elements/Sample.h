#pragma once
#pragma once
#ifndef SAMPLE_H
#define SAMPLE_H

#include <spc/elements/StratigraphicPositionableElement.h>
#include <spc/elements/MovableElement.h>

namespace spc {

class Sample :  public StratigraphicPositionableElement {

public:
    SPC_ELEMENT(Sample)
    EXPOSE_TYPE
    Sample()
        : StratigraphicPositionableElement()
    {
    }

    Sample(const Sample& other)
        : StratigraphicPositionableElement(other)
    {
    }

    ~Sample()
    {
    }

    Sample(const float x, const float y, const float z)
        :point_(x, y, z)
    {
    }

    Eigen::Vector3f getPosition()
    {
        return point_.getPosition();
    }

    void setPosition(const Eigen::Vector3f &p)
    {
        point_.setPosition(p);
    }


    Sample(const Eigen::Vector3f v)
        : point_(v)
    {
    }

    virtual float predictStratigraphicPositionFromModel() const override
    {
        spc::StratigraphicModelBase::Ptr model = this->getStratigraphicModel();
        if (model != nullptr)
            return model->predictStratigraphicPosition(point_.getPosition());
        else
            return spcNANMacro; // returning nan
    }

private:
    friend class cereal::access;

    template <class Archive>
    void serialize(Archive& ar, const std::uint32_t version)
    {
        ar(cereal::base_class<StratigraphicPositionableElement>(this));
        ar(point_);

    }




protected:
    Point3D point_;



    // GeometricElement3DBase interface
public:
    virtual void applyTransform(const TransformT &transform) override;
};




} // end nspace

#endif // SAMPLE_H
