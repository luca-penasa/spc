#include "stratigraphic_model_base.h"

namespace spc
{


spcStratigraphicModelBase::spcStratigraphicModelBase()
{
}

std::vector<float> spcStratigraphicModelBase::getStratigraphicPositions(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const std::vector<int> &indices)
{
    std::vector<float> out;
    for (int id : indices)
    {
        pcl::PointXYZ p = cloud->at(id);
        out.push_back(getStratigraphicPosition(Vector3f(p.x, p.y, p.z)));
    }

    return out;

}

std::vector<float> spcStratigraphicModelBase::getStratigraphicPositions(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    std::vector<float> out;
    for (pcl::PointXYZ p: *cloud)
    {
        pcl::PointXYZ point;
        out.push_back(getStratigraphicPosition(Vector3f(p.x, p.y, p.z)));
    }

    return out;

}

std::vector<float> spcStratigraphicModelBase::getStratigraphicPositions(spc::spcGenericCloud *cloud)
{
    std::vector<float> out;
    int n = cloud->getSize();

    out.resize(n);

    float x, y, z, sp;
    for (int i = 0 ; i < n; i++)
    {
        cloud->getPoint(i, x, y, z);
        out.at(i) = getStratigraphicPosition(Vector3f(x,y,z));
    }

    return out;
}


std::vector<float> spcStratigraphicModelBase::getStratigraphicPositions(spcGenericCloud *cloud, const std::vector<int> &indices)
{
    std::vector<float> out;
    out.resize(indices.size());

    float x, y, z, sp;
    int counter  =0;
    for (int i: indices)
    {
        cloud->getPoint(i, x, y, z);
        out.at(counter++) = getStratigraphicPosition(Vector3f(x,y,z));
    }

    return out;
}

std::vector<float> spcStratigraphicModelBase::getStratigraphicPositions(spcGenericCloud::Ptr cloud, const std::vector<int> &indices)
{
    std::vector<float> out;
    out.resize(indices.size());

    float x, y, z, sp;
    int counter  =0;
    for (int i: indices)
    {
        cloud->getPoint(i, x, y, z);
        out.at(counter++) = getStratigraphicPosition(Vector3f(x,y,z));
    }

    return out;
}



} //end nspace
