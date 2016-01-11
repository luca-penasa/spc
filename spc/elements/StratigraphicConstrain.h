#pragma once
#ifndef STRATIGRAPHICLINK_H
#define STRATIGRAPHICLINK_H

#include<spc/elements/MovableElement.h>
#include <spc/elements/templated/PointSet.h>

#include <spc/elements/StratigraphicPositionableElement.h>

#include <spc/elements/templated/PolyLine3D.h>

#include <cereal/cereal.hpp>
namespace spc
{


class StratigraphicConstrain: public ElementBase
{
public:

    SPC_ELEMENT(StratigraphicConstrain)
    EXPOSE_TYPE

    StratigraphicConstrain()
    {

    }

    void addVertex(StratigraphicPositionableElement::Ptr v)
    {
        vertices_.push_back(v);
        updatePointSetRepresentation();
    }

    size_t getNumberOfConstrains() const
    {
        return vertices_.size();
    }

    void updatePointSetRepresentation()
    {

        polyline_rep_.resize(vertices_.size());

        size_t id = 0;
        for (Point3D::Ptr p: vertices_)
        {
            polyline_rep_.setPoint(id++, p->getPosition());
        }
    }

    std::vector<StratigraphicPositionableElement::Ptr> getVertices() const
    {
        return vertices_;
    }


    PolyLine3D getPolyLineRep() const
    {
        return polyline_rep_;
    }

//    float getSquaredResidual() const
//    {
//        float sqres;
//        StratigraphicPositionableElement::Ptr thisvert = vertices_.at(0);
//        for(int i = 1 ; i < vertices_.size(); ++i)
//        {
//            StratigraphicPositionableElement::Ptr nextvert = vertices_.at(i);
//            float diff = thisvert->getStratigraphicPosition() - nextvert->getStratigraphicPosition();
//            sqres += diff*diff;

//            thisvert = nextvert;
//        }

//        return sqres;
//    }

protected:
    std::vector<StratigraphicPositionableElement::Ptr> vertices_;

    // a representation os polylin of the points defining the constrain
    PolyLine3D polyline_rep_;

private:
    friend class cereal::access;




    template <class Archive> void load(Archive &ar)
    {
        ar(cereal::base_class<ElementBase>(this), CEREAL_NVP(vertices_));
        updatePointSetRepresentation();

    }

    template <class Archive> void save(Archive &ar) const
    {
        ar(cereal::base_class<ElementBase>(this), CEREAL_NVP(vertices_));
    }

};



}

#endif // STRATIGRAPHICLINK_H
