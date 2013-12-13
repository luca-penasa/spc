#ifndef VOMBAT_SPCPCLCLOUD_H
#define VOMBAT_SPCPCLCLOUD_H


#include <spc/elements/generic_cloud.h>

namespace spc
{

template <typename PointT>
class spcPCLCloud: public spcGenericCloud
{
public:

    typedef boost::shared_ptr<spcPCLCloud<PointT> > Ptr;
    typedef boost::shared_ptr<const spcPCLCloud<PointT> > ConstPtr;

    typedef boost::shared_ptr<pcl::PointCloud<PointT> > CloudPtrT;

    spcPCLCloud(CloudPtrT cloud)
    {
        cloud_ = cloud;
    }

    spcPCLCloud(const pcl::PointCloud<PointT> &cloud)
    {
        cloud_ = boost::make_shared<pcl::PointCloud<PointT> > (cloud);
    }

    virtual void getPoint(const int id, float &x, float &y, float &z) const
    {
        x = cloud_->at(id).x;
        y = cloud_->at(id).y;
        z = cloud_->at(id).z;
    }

    //// we assume here that is a float the value to be extracted
    virtual void getFieldValue(const int id, const std::string fieldname, float &val)
    {

        std::vector<pcl::PCLPointField> fields;
        int distance_idx = pcl::getFieldIndex (*cloud_, fieldname, fields);
        if (distance_idx == -1)
        {
          PCL_WARN ("[spc::GenericCloud] Unable to find field name in point type.\n");
          return;
        }

        const uint8_t* pt_data = reinterpret_cast<const uint8_t*> (&cloud_->points[id]);

        memcpy (&val, pt_data + fields[distance_idx].offset, sizeof (float));
    }

    virtual bool hasField(const std::string fieldname)
    {
        std::vector<pcl::PCLPointField> fields;
        int distance_idx = pcl::getFieldIndex (*cloud_, fieldname, fields);
        if (distance_idx == -1)
            return false;
        else
            return true;
    }



    virtual int size() const
    {
        return cloud_->size();
    }

    virtual void resize(size_t s)
    {
        cloud_->resize(s);
    }


private:
    // the actual data
    CloudPtrT cloud_;
};



}//end nspace
#endif // SPCPCLCLOUD_H
