#include "cylinder.h"



spc::Cylinder::Cylinder(const Eigen::Vector3f dir)
{
    direction_ = spcNormal3D(dir);
}





void spc::Cylinder::getPointToCylinderDistances(const Eigen::Vector3f point, float &to_axis, float &to_origin) const
{
    // we project the point onto the direction of the cyclinder
    Eigen::Vector3f point_proj = direction_.projectPoint(point);

    //the distance from the cylinder origin along the direction
    auto to_point = point_proj - this->getPosition();
    to_origin = to_point.dot(direction_.getUnitNormal()); //signed dist

    //now the distance from the cyclinder axis
    Eigen::Vector3f point_to_ax = point - point_proj;
    to_axis = point_to_ax.norm(); //unsigned dist
}

bool spc::Cylinder::isPointWithinCylinder(const Eigen::Vector3f point) const
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

std::vector<int> spc::Cylinder::getIndicesInside(spcGenericCloud::ConstPtr cloud)
{
    std::vector<int> ids;
    auto siz = cloud->size();
    for (int i = 0; i < siz; ++i)
    {
        Eigen::Vector3f p = cloud->getPoint(i);
        if (this->isPointWithinCylinder(p))
            ids.push_back(i);
    }

    return ids;
}



