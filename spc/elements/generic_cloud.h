#ifndef GENERIC_CLOUD_H
#define GENERIC_CLOUD_H

#include "element_base.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/io/io.h>
namespace spc
{




class spcGenericCloud: public spcElementBase
{
public:

    typedef typename boost::shared_ptr<spcGenericCloud> Ptr;
    typedef typename boost::shared_ptr<const spcGenericCloud> ConstPtr;


    spcGenericCloud();


//    virtual int toStreamMeOnly(std::stringstream &stream) {/*nothing for now*/}


    /// a generic cloud must implement these method
    virtual void getPoint(const int id, float &x, float &y, float &z) const = 0  ;

    virtual void getFieldValue(const int id, const std::string fieldname, float &val) = 0;

    virtual int getSize() const = 0 ;

    virtual bool hasField(const std::string fieldname) = 0;


    /// some always-working methods
    virtual Vector3f getPoint (const int id)
    {
        float x,y,z;
        getPoint(id, x, y, z);
        return Vector3f(x, y, z);
    }


    virtual std::vector<float> getField(const std::string fieldname, std::vector<int> indices)
    {
        std::vector<float> out;

        if (!hasField(fieldname))
        {
            pcl::console::print_warn("[Error in %s] asked for field %s", getClassName().c_str(), fieldname.c_str());
            return out;
        }

        float val;
        for (int i : indices)
        {
            getFieldValue(i, fieldname, val);
            out.push_back(val);
        }

        return out;


    }




    pcl::PointCloud<pcl::PointXYZ> applyTransform(const Transform<float, 3, Affine, AutoAlign> &T)
    {
        pcl::PointCloud<pcl::PointXYZ> cloud;
        int size = getSize();


        for (int i = 0; i < size; ++i)
        {
            Vector3f point = T * getPoint(i);
            pcl::PointXYZ p(point(0), point(1), point(2));
            cloud.push_back(p);
        }

        return cloud;
    }


};

template <typename PointT>
class spcPCLCloud: public spcGenericCloud
{
public:

    typedef boost::shared_ptr<spcPCLCloud<PointT> > Ptr;
    typedef boost::shared_ptr<const spcPCLCloud<PointT> > ConstPtr;

    typedef boost::shared_ptr<pcl::PointCloud<PointT> > CloudPtrT;

    // the actual data
    CloudPtrT cloud_;

    spcPCLCloud(CloudPtrT cloud)
    {
        cloud_ = cloud;
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



    virtual int getSize() const
    {
        return cloud_->size();
    }
};


class spcPCLPointCloud2: public pcl::PCLPointCloud2 , public spcGenericCloud
{
public:
    spcPCLPointCloud2() {}

    spcPCLPointCloud2(const spcPCLPointCloud2 &other): pcl::PCLPointCloud2(other)
    {


    }

    virtual void getPoint(const int id, float &x, float &y, float &z)
    {

    }

    virtual void getFieldValue(const int id, const std::string fieldname, float &val)
    {

    }


    virtual int getSize() const
    {
        return width * height; //should be a valid size
    }

protected:
    int offset_;
    int x_stride_;
    int y_stride_;
    int z_stride_;


};


}//end nspace

#endif // GENERIC_CLOUD_H
