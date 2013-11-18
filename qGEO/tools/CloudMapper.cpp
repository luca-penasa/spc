#include "CloudMapper.h"

template<>
void CloudWrapper<ccPointCloud>::getPoint(int id, float &x, float &y, float &z)
{
    assert(id < in_cloud->size());
    CCVector3 point;
    in_cloud->getPoint(id, point);
    x = point.x;
    y = point.y;
    z = point.z;
}

template<>
void CloudWrapper<ccPointCloud>::getFieldValue(const int id, const std::string fieldname, float &val)
{
    int field_id = in_cloud->getScalarFieldIndexByName(fieldname.c_str());

    if (field_id != -1)
    {
        CCLib::ScalarField * field = in_cloud->getScalarField(field_id);
        val = field->getValue(id);
    }
}


template<>
void CloudWrapper<pcl::PointCloud<pcl::PointXYZ> >::getFieldValue(const int id, const std::string fieldname, float &val)
{
    /// NOTHING
    std::cout << "error in CloudMapper field does not exists!" << std::endl;
    return;
}

template<>
int CloudWrapper<ccPointCloud>::getSize() const
{
    return in_cloud->size();
}

template<>
int CloudWrapper<pcl::PointCloud<pcl::PointXYZ>>::getSize() const
{
    return in_cloud->size();
}

template<>
void CloudWrapper<pcl::PointCloud<pcl::PointXYZ>>::getPoint(int id, float &x, float &y, float &z)
{
    assert(id < in_cloud->size());
//also assert we have xyz fields!!!
    pcl::PointXYZ point= in_cloud->at(id);
    x = point.x;
    y = point.y;
    z = point.z;
}
