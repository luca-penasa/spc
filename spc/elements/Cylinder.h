#ifndef SPC_CYLINDER_H
#define SPC_CYLINDER_H

#include <spc/elements/MovableElement.h>
#include <spc/elements/Normal3D.h>
#include <spc/elements/PointCloudBase.h>

namespace spc
{

///
/// \brief The Cylinder class represent a cylinder in space
/// witht the base locate at the center of the point provided with the movable
/// element
/// and with length and radius as in parameters
///
class Cylinder : public Point3D
{
public:
    SPC_OBJECT(Cylinder)
EXPOSE_TYPE

    /// def constrcutor
    Cylinder()
    {
    }

    Cylinder(const Cylinder &cyl) : Point3D(cyl)
    {
        // TODO we'll need to copy some other staff
    }

    Cylinder(const Eigen::Vector3f dir);

    Normal3D getDirection() const
    {
        return direction_;
    }

    void setDirection(Normal3D dir)
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
    void getPointToCylinderDistances(const Eigen::Vector3f point,
                                     float &to_axis, float &to_origin) const;

    /// the test for inside/outside from the cyclinder.
    bool isPointWithinCylinder(const Eigen::Vector3f point) const;

    std::vector<int> getIndicesInside(PointCloudBase::ConstPtr cloud);

private:
    Normal3D direction_;

    float length_;

    float radius_;

private:
    friend class cereal::access;

    template <class Archive> void sserialize(Archive &ar)
    {
        ar(cereal::base_class<spc::Point3D>(this),
           CEREAL_NVP(direction_), CEREAL_NVP(length_), CEREAL_NVP(radius_));
    }
};

} // end nspace

#endif // CYLINDER_H
