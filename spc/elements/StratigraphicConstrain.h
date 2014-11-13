#ifndef STRATIGRAPHICLINK_H
#define STRATIGRAPHICLINK_H

#include<spc/elements/MovableElement.h>
#include <spc/elements/templated/PointSet.h>

#include <spc/elements/templated/PolyLine3D.h>
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

    void addVertex(Point3D::Ptr v)
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


    PolyLine3D getPolyLineRep() const
    {
        return polyline_rep_;
    }


protected:
    std::vector<Point3D::Ptr> vertices_;

    // a representation os polylin of the points defining the constrain
    PolyLine3D polyline_rep_;

private:
    friend class cereal::access;

    template <class Archive> void serialize(Archive &ar)
    {
        ar(cereal::base_class<ElementBase>(this), CEREAL_NVP(vertices_));
    }

};

}

#endif // STRATIGRAPHICLINK_H
