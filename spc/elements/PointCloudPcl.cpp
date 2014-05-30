#include "PointCloudPcl.h"
#include <boost/make_shared.hpp>

namespace spc
{

template <typename PointT> PointCloudPcl<PointT>::PointCloudPcl(CloudPtrT cloud)
{
    cloud_ = cloud;
}

template <typename PointT>
PointCloudPcl<PointT>::PointCloudPcl(const pcl::PointCloud<PointT> &cloud)
{
    cloud_ = boost::make_shared<pcl::PointCloud<PointT>>(cloud);
}

template <typename PointT>
void PointCloudPcl
    <PointT>::getPoint(const int id, float &x, float &y, float &z) const
{
    x = cloud_->at(id).x;
    y = cloud_->at(id).y;
    z = cloud_->at(id).z;
}

template <typename PointT>
void PointCloudPcl<PointT>::setPoint(const int id, const float x, const float y,
                                   const float z)
{

    try
    {

        PointT p = cloud_->at(id);
        p.x = x;
        p.y = y;
        p.z = z;

        std::cout << "setting" << std::endl;
        cloud_->at(id) = p;
        std::cout << "first done" << std::endl;
    }
    catch (const std::exception &exc)
    {
        /// any exception is catched here!
        std::cerr << exc.what() << std::endl;
        return;
    }

    //        cloud_->at(id).y = y;
    //        cloud_->at(id).z = z;
}

template <typename PointT>
void PointCloudPcl<PointT>::getFieldValue(const int id,
                                        const std::string fieldname, float &val)
{

    std::vector<pcl::PCLPointField> fields;
    int distance_idx = pcl::getFieldIndex(*cloud_, fieldname, fields);
    if (distance_idx == -1) {
        PCL_WARN("[GenericCloud] Unable to find field name in point type.\n");
        return;
    }

    const uint8_t *pt_data = reinterpret_cast
        <const uint8_t *>(&cloud_->points[id]);

    memcpy(&val, pt_data + fields[distance_idx].offset, sizeof(float));
}

template <typename PointT>
bool PointCloudPcl<PointT>::hasField(const std::string fieldname)
{
    std::vector<pcl::PCLPointField> fields;
    int distance_idx = pcl::getFieldIndex(*cloud_, fieldname, fields);
    if (distance_idx == -1)
        return false;
    else
        return true;
}

template <typename PointT> int PointCloudPcl<PointT>::size() const
{
    return cloud_->size();
}

template <typename PointT> void PointCloudPcl<PointT>::resize(size_t s)
{
    cloud_->resize(s);
    cloud_->reserve(s);
}

template class PointCloudPcl<pcl::PointXYZ>;
template class PointCloudPcl<pcl::PointXYZI>;
}
