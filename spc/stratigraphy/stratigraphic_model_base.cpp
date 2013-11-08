#include "stratigraphic_model_base.h"

namespace spc
{


StratigraphicModelBase::StratigraphicModelBase()
{
}

std::vector<float> StratigraphicModelBase::getStratigraphicPositions(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const std::vector<int> &indices)
{
    std::vector<float> out;
    for (int id : indices)
    {
        pcl::PointXYZ p = cloud->at(id);
        out.push_back(getStratigraphicPosition(Vector3f(p.x, p.y, p.z)));
    }

    return out;

}

std::vector<float> StratigraphicModelBase::getStratigraphicPositions(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    std::vector<float> out;
    for (pcl::PointXYZ p: *cloud)
    {
        pcl::PointXYZ point;
        out.push_back(getStratigraphicPosition(Vector3f(p.x, p.y, p.z)));
    }

    return out;

}


/// INST


} //end nspace
