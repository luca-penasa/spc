#include "DynamicScalarFieldGenerator.h"

namespace spc
{


DynamicScalarFieldGenerator::DynamicScalarFieldGenerator()
{
}

std::vector<float> DynamicScalarFieldGenerator::getScalarFieldValues(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const std::vector<int> &indices) const
{
    std::vector<float> out;
    spcForEachMacro (int id , indices)
    {
        pcl::PointXYZ p = cloud->at(id);
        out.push_back(this->getScalarFieldValue(Vector3f(p.x, p.y, p.z)));
    }

    return out;

}

std::vector<float> DynamicScalarFieldGenerator::getScalarFieldValues(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) const
{
    std::vector<float> out;
    spcForEachMacro(pcl::PointXYZ p, *cloud)
    {
        pcl::PointXYZ point;
        out.push_back(this->getScalarFieldValue(Vector3f(p.x, p.y, p.z)));
    }

    return out;

}

std::vector<float> DynamicScalarFieldGenerator::getScalarFieldValues(spc::spcGenericCloud *cloud) const
{
    std::vector<float> out;
    int n = cloud->size();

    out.resize(n);

    float x, y, z;
    for (int i = 0 ; i < n; i++)
    {
        cloud->getPoint(i, x, y, z);
        out.at(i) = this->getScalarFieldValue(Vector3f(x,y,z));
    }

    return out;
}


std::vector<float> DynamicScalarFieldGenerator::getScalarFieldValues(spcGenericCloud *cloud, const std::vector<int> &indices) const
{
    std::vector<float> out;
    out.resize(indices.size());

    float x, y, z;
    int counter  =0;
    spcForEachMacro (int i, indices)
    {
        cloud->getPoint(i, x, y, z);
        out.at(counter++) = this->getScalarFieldValue(Vector3f(x,y,z));
    }

    return out;
}

std::vector<float> DynamicScalarFieldGenerator::getScalarFieldValues(spcGenericCloud::Ptr cloud, const std::vector<int> &indices) const
{
    std::vector<float> out;
    out.resize(indices.size());

    float x, y, z;
    int counter  =0;
    spcForEachMacro (int i, indices)
    {
        cloud->getPoint(i, x, y, z);
        out.at(counter++) = this->getScalarFieldValue(Vector3f(x,y,z));
    }

    return out;
}

std::vector<float> DynamicScalarFieldGenerator::getScalarFieldValues(spcGenericCloud::Ptr cloud) const
{
    std::vector<float> out;
    out.resize(cloud->size());

    float x, y, z;

    for (int i = 0; i < cloud->size(); ++i)
    {
        cloud->getPoint(i, x, y, z);
        out.at(i) = this->getScalarFieldValue(Vector3f(x,y,z));
    }

    return out;
}




} //end nspace
