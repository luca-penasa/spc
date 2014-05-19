#ifndef SPC_CYLINDER_H
#define SPC_CYLINDER_H

#include <Eigen/Dense>
#include <Eigen/Core>

#include <Eigen/Geometry>


#include <spc/elements/movable_element.h>
#include <spc/elements/normal3d.h>

namespace spc {

///
/// \brief The Cylinder class represent a cylinder in space
/// witht the base locate at the center of the point provided with the movable element
/// and with length and radius as in parameters
///
class Cylinder: public PositionableElement
{
public:
    typedef boost::shared_ptr<Cylinder> Ptr;
    typedef boost::shared_ptr<const Cylinder> ConstPtr;

    /// def constrcutor
    Cylinder()
    {

    }

    Cylinder(const Cylinder & cyl) : PositionableElement(cyl)
    {
        //TODO we'll need to copy some other staff
    }

    Cylinder(const Vector3f dir)
    {
        direction_ = spcNormal3D(dir);
    }


    spcNormal3D getDirection() const
    {
        return direction_;
    }

    void setDirection(spcNormal3D dir)
    {
        direction_ = dir;
    }

    void setDirection(Vector3f dir)
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
    void getPointToCylinderDistances(const Vector3f point, float &to_axis, float & to_origin) const
    {
        // we project the point onto the direction of the cyclinder
        Vector3f point_proj = direction_.projectPoint(point);

        //the distance from the cylinder origin along the direction
        auto to_point = point_proj - this->getPosition();
        to_origin = to_point.dot(direction_.getUnitNormal()); //signed dist

        //now the distance from the cyclinder axis
        Vector3f point_to_ax = point - point_proj;
        to_axis = point_to_ax.norm(); //unsigned dist
    }

    /// the test for inside/outside from the cyclinder.
    bool isPointWithinCylinder(const Vector3f point) const
    {
        float radial;
        float along;



        this->getPointToCylinderDistances(point, radial, along);

        std::cout << "distance from axis " << radial << std::endl;
        std::cout << "distance from origin " << along << std::endl;
        if ((radial > radius_) | (along > length_) | (along <= 0))
            return false;
        else
            return true;
    }

    std::vector<int> getIndicesInside(spcGenericCloud::ConstPtr cloud)
    {
        std::vector<int> ids;
        auto siz = cloud->size();
        for (int i = 0; i < siz; ++i)
        {
            Vector3f p = cloud->getPoint(i);
            if (this->isPointWithinCylinder(p))
                ids.push_back(i);
        }

        return ids;
    }






private:
    spcNormal3D direction_;

    float length_;

    float radius_;
};


}//end nspace


#endif // CYLINDER_H
