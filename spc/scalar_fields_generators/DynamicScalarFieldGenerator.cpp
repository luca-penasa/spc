#include "DynamicScalarFieldGenerator.h"

namespace spc
{


DynamicScalarFieldGenerator::DynamicScalarFieldGenerator()
{
}

std::vector<float> DynamicScalarFieldGenerator::getScalarFieldValues(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const std::vector<int> &indices)
{
    std::vector<float> out;
    BOOST_FOREACH (int id , indices)
    {
        pcl::PointXYZ p = cloud->at(id);
        out.push_back(getScalarFieldValue(Vector3f(p.x, p.y, p.z)));
    }

    return out;

}

std::vector<float> DynamicScalarFieldGenerator::getScalarFieldValues(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    std::vector<float> out;
    BOOST_FOREACH(pcl::PointXYZ p, *cloud)
    {
        pcl::PointXYZ point;
        out.push_back(getScalarFieldValue(Vector3f(p.x, p.y, p.z)));
    }

    return out;

}

std::vector<float> DynamicScalarFieldGenerator::getScalarFieldValues(spc::spcGenericCloud *cloud)
{
    std::vector<float> out;
    int n = cloud->size();

    out.resize(n);

    float x, y, z;
    for (int i = 0 ; i < n; i++)
    {
        cloud->getPoint(i, x, y, z);
        out.at(i) = getScalarFieldValue(Vector3f(x,y,z));
    }

    return out;
}


std::vector<float> DynamicScalarFieldGenerator::getScalarFieldValues(spcGenericCloud *cloud, const std::vector<int> &indices)
{
    std::vector<float> out;
    out.resize(indices.size());

    float x, y, z;
    int counter  =0;
    BOOST_FOREACH (int i, indices)
    {
        cloud->getPoint(i, x, y, z);
        out.at(counter++) = getScalarFieldValue(Vector3f(x,y,z));
    }

    return out;
}

std::vector<float> DynamicScalarFieldGenerator::getScalarFieldValues(spcGenericCloud::Ptr cloud, const std::vector<int> &indices)
{
    std::vector<float> out;
    out.resize(indices.size());

    float x, y, z;
    int counter  =0;
    BOOST_FOREACH (int i, indices)
    {
        cloud->getPoint(i, x, y, z);
        out.at(counter++) = getScalarFieldValue(Vector3f(x,y,z));
    }

    return out;
}

std::vector<float> DynamicScalarFieldGenerator::getScalarFieldValues(spcGenericCloud::Ptr cloud)
{
    std::vector<float> out;
    out.resize(cloud->size());

    float x, y, z;

    for (int i = 0; i < cloud->size(); ++i)
    {
        cloud->getPoint(i, x, y, z);
        out.at(i) = getScalarFieldValue(Vector3f(x,y,z));
    }

    return out;
}




} //end nspace
