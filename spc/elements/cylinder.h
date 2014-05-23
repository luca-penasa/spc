#ifndef SPC_CYLINDER_H
#define SPC_CYLINDER_H

#include <spc/elements/movable_element.h>
#include <spc/elements/normal3d.h>
#include <spc/elements/generic_cloud.h>

namespace spc {


///
/// \brief The Cylinder class represent a cylinder in space
/// witht the base locate at the center of the point provided with the movable element
/// and with length and radius as in parameters
///
class Cylinder: public PositionableElement
{
public:
SPC_OBJECT(Cylinder)

    /// def constrcutor
    Cylinder()
    {

    }

    Cylinder(const Cylinder & cyl) : PositionableElement(cyl)
    {
        //TODO we'll need to copy some other staff
    }

    Cylinder(const Eigen::Vector3f dir);


    spcNormal3D getDirection() const
    {
        return direction_;
    }

    void setDirection(spcNormal3D dir)
    {
        direction_ = dir;
    }

    void setDirection(Eigen::Vector3f dir)
    {
        direction_.setNormal(dir);
    }


    void setLength(float len)
    {
        length_ = len;
    }

    void setRadius(float rad)
    {
        radius_ = rad;
    }

    /// distance is always positive, clearly
    /// this is the distance of your point from the axis of the cyclinder
    void getPointToCylinderDistances(const Eigen::Vector3f point, float &to_axis, float & to_origin) const;

    /// the test for inside/outside from the cyclinder.
    bool isPointWithinCylinder(const Eigen::Vector3f point) const;

    std::vector<int> getIndicesInside(spcGenericCloud::ConstPtr cloud);

private:
    spcNormal3D direction_;

    float length_;

    float radius_;

private:
    friend class cereal::access;

    template <class Archive>
    void sserialize( Archive & ar )
    {
        ar( CEREAL_NVP(direction_),
            CEREAL_NVP(length_),
            CEREAL_NVP(radius_));
    }
};


}//end nspace


#endif // CYLINDER_H
