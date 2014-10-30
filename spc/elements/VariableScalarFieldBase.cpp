#include "VariableScalarFieldBase.h"

namespace spc
{


DtiClassType
VariableScalarFieldBase::Type ("VariableScalarFieldBase", &ElementBase::Type);

VariableScalarFieldBase::VariableScalarFieldBase()
{
}

VariableScalarFieldBase::VectorT VariableScalarFieldBase::getScalarFieldValues(
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
    const std::vector<int> &indices) const
{
    VectorT out(indices.size());

    for (int i = 0 ; i < indices.size(); ++i)
    {
        pcl::PointXYZ p = cloud->at(indices.at(i));
        out.at(i) =this->getScalarFieldValue(Vector3f(p.x, p.y, p.z));
    }

    return out;
}

VariableScalarFieldBase::VectorT VariableScalarFieldBase::getScalarFieldValues(
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) const
{
    VectorT out(cloud->size());

    for (int i = 0 ; i < cloud->size(); ++i)
    {
        pcl::PointXYZ p = cloud->at(i);
        out.at(i) = this->getScalarFieldValue(Vector3f(p.x, p.y, p.z));
    }

    return out;
}

VariableScalarFieldBase::VectorT VariableScalarFieldBase::getScalarFieldValues(spc::PointCloudBase
                                                  *cloud) const
{
    VectorT out;
    int n = cloud->size();

    out.resize(n);

    float x, y, z;
    for (int i = 0; i < n; i++) {
        cloud->getPoint(i, x, y, z);
        out.at(i) = this->getScalarFieldValue(Vector3f(x, y, z));
    }

    return out;
}

VariableScalarFieldBase::VectorT VariableScalarFieldBase::getScalarFieldValues(
    PointCloudBase *cloud, const std::vector<int> &indices) const
{
    VectorT out;
    out.resize(indices.size());

    float x, y, z;
    int counter = 0;
    for(int i: indices)
    {
        cloud->getPoint(i, x, y, z);
        out.at(counter++) = this->getScalarFieldValue(Vector3f(x, y, z));
    }

    return out;
}

VariableScalarFieldBase::VectorT VariableScalarFieldBase::getScalarFieldValues(
    PointCloudBase::Ptr cloud, const std::vector<int> &indices) const
{
    VectorT out;
    out.resize(indices.size());

    float x, y, z;
    int counter = 0;
    for(int i: indices)
    {
        cloud->getPoint(i, x, y, z);
        out.at(counter++) = this->getScalarFieldValue(Vector3f(x, y, z));
    }

    return out;
}

VariableScalarFieldBase::VectorT VariableScalarFieldBase::getScalarFieldValues(
    PointCloudBase::Ptr cloud) const
{
    VectorT out;
    out.resize(cloud->size());

    float x, y, z;

    for (int i = 0; i < cloud->size(); ++i) {
        cloud->getPoint(i, x, y, z);
        out.at(i) = this->getScalarFieldValue(Vector3f(x, y, z));
    }

    return out;
}

} // end nspace


#include <spc/core/spc_cereal.hpp>
SPC_CEREAL_REGISTER_TYPE(spc::VariableScalarFieldBase)
