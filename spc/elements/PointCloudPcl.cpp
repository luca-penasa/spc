#include "PointCloudPcl.h"
#include <boost/make_shared.hpp>

namespace spc
{


DtiClassType PointCloudPCL::Type = DtiClassType("PointCloudPCL", &PointCloudBaseWithSensor::Type);

PointCloudPCL::PointCloudPCL(pcl::PCLPointCloud2Ptr cloud)
{
    cloud_ = cloud;

    xyz_offsets_ << cloud_->fields[pcl::getFieldIndex (*cloud_, "x")].offset,
      cloud_->fields[pcl::getFieldIndex (*cloud_, "y")].offset,
      cloud_->fields[pcl::getFieldIndex (*cloud_, "z")].offset;

    DLOG(INFO) << "PointCloudPCL sucessfully created";
}



void PointCloudPCL::getFieldValue(const int id,
                                  const std::string fieldname, float &val) const
{
    if (!hasField(fieldname)) {
        PCL_WARN("[GenericCloud] Unable to find field %s in point type.\n", fieldname.c_str());
        return;
    }

    int field_id = pcl::getFieldIndex(*cloud_, fieldname );

    int value_pos = cloud_->fields[field_id].offset + cloud_->point_step * id;
    memcpy(&val, &cloud_->data[value_pos], sizeof(float));
}

void PointCloudPCL::setFieldValue(const int id, const std::string fieldname,const float &val)
{
    int field_id = pcl::getFieldIndex(*cloud_, fieldname);

    if (!hasField(fieldname)) {
        PCL_WARN("[GenericCloud] Unable to find field %s in point type.\n", fieldname.c_str());
        return;
    }

    int value_pos = cloud_->fields[field_id].offset + cloud_->point_step * id;
    memcpy( &cloud_->data[value_pos], &val, sizeof(float));

}



bool PointCloudPCL::hasField(const std::string fieldname) const
{
    int field_id = pcl::getFieldIndex(*cloud_, fieldname);
    if (field_id >= 0 )
        return true;
    else
        return false;
}

int PointCloudPCL::size() const
{
    return cloud_->height * cloud_->width;
}

void PointCloudPCL::resize(size_t s)
{
    // we are assuming the cloud is not dense
    cloud_->data.resize(s * cloud_->point_step);
    cloud_->width += cloud_->point_step * s;
}


}
