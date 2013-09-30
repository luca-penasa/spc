#include <spc/geology/stratigraphic_normal_model.h>
//#include <ccPlane.h>
//#include <ccHObjectCaster.h>

//TMP
//#include <iostream>

#include<pcl/features/normal_3d.h>


namespace spc
{

StratigraphicNormalModel::StratigraphicNormalModel(): GeologicPlane(), stratigraphic_shift_(0.0f)
{
    //nothing for now
}

float StratigraphicNormalModel::getStratigraphicPosition(const Eigen::Vector3f point)
{
    float dist = this->getDistanceFromOrigin();
    return point.dot(getNormal()) - dist + stratigraphic_shift_ ;
}

Eigen::Vector3f StratigraphicNormalModel::getStratigraphicNormal(const Eigen::Vector3f point)
{
    return getNormal();
}


}//end nspace
