#ifdef SPC_WITH_PCL


#include "PointCloudPcl.h"
#include <boost/make_shared.hpp>
#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string/split.hpp>


namespace spc
{


//DtiClassType PointCloudPCL::Type = DtiClassType("PointCloudPCL", &PointCloudBaseWithSensor::Type);

//PointCloudPCL::PointCloudPCL(pcl::PCLPointCloud2Ptr cloud)
//{
//    cloud_ = cloud;
//    DLOG(INFO) << "PointCloudPCL sucessfully created";
//}



//void PointCloudPCL::getFieldValue(const IndexT id,
//                                  const std::string fieldname, float &val) const
//{
//    if (!hasField(fieldname))
//    {
//		DLOG(WARNING) <<"Unable to find field: " << fieldname;
//        return;
//    }

//    int field_id = pcl::getFieldIndex(*cloud_, fieldname );

//    int value_pos = cloud_->fields[field_id].offset + cloud_->point_step * id;
//    memcpy(&val, &cloud_->data[value_pos], sizeof(float));
//}

//void PointCloudPCL::setFieldValue(const IndexT id, const std::string fieldname, const float &val)
//{
//    int field_id = pcl::getFieldIndex(*cloud_, fieldname);

//    if (!hasField(fieldname)) {
//		DLOG(WARNING) << "Unable to find field: " << fieldname;
//        return;
//    }

//    int value_pos = cloud_->fields[field_id].offset + cloud_->point_step * id;
//    memcpy( &cloud_->data[value_pos], &val, sizeof(float));
//}



//bool PointCloudPCL::hasField(const std::string fieldname) const
//{
//    int field_id = pcl::getFieldIndex(*cloud_, fieldname);
//    if (field_id >= 0 )
//        return true;
//    else
//        return false;
//}

//size_t PointCloudPCL::getNumberOfPoints() const
//{
//    return cloud_->height * cloud_->width;
//}

//void PointCloudPCL::resize(const IndexT s)
//{
//    // we are assuming the cloud is not dense
//    cloud_->data.resize(s * cloud_->point_step);
//    cloud_->width = s;
//    cloud_->height = 1; // force to one.
//}

//std::vector<std::string> PointCloudPCL::getFieldNames() const
//{
//    std::string names = pcl::getFieldsList(*cloud_);
//    std::vector<std::string> fields, out;
//    boost::split(fields, names, boost::is_any_of(" "), boost::token_compress_on);

//    for (auto f: fields)
//    {
//        if (f != "_")
//            out.push_back(f);
//    }

//    for (auto f: out)
//        DLOG(INFO) << "FIELD found: " << f;

//    return  out;
//}

//void PointCloudPCL::addField(const std::string &name)
//{
//    if (this->hasField(name))
//    {
//        DLOG(WARNING) << "field yet exists: " << name;
//            return;
//    }

//    pcl::PointCloud<PointScalar> newcloud;
//    newcloud.resize(this->getNumberOfPoints());

//    pcl::PCLPointCloud2 c, out;
//    pcl::toPCLPointCloud2(newcloud, c);

//    c.fields[0].name = name;

//    pcl::concatenateFields(*cloud_, c, out);

//    *cloud_ = out;
//}

//bool PointCloudPCL::getRGBField(const PointCloudBase::COLORS_ENUM &color, VectorXf &vector, const std::vector<IndexT> &indices) const
//{
//    LOG(WARNING) << "cannot get rgb fields in PointCloudPlc. Virtual method not implemented";
//}


}

#endif // SPC_WITH_PCL
