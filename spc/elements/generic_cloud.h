#ifndef GENERIC_CLOUD_H
#define GENERIC_CLOUD_H

#include "element_base.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
//#include <pcl/PCLPointCloud2.h>
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

    virtual void setPoint(const int id, const float x, const float y, const float z) = 0;

    virtual void getFieldValue(const int id, const std::string fieldname, float &val) = 0;

    virtual int size() const = 0 ;

    virtual bool hasField(const std::string fieldname) = 0;

    virtual void resize(size_t s) = 0;


    /// some always-working methods
    virtual Vector3f getPoint (const int id) const
    {
        float x,y,z;
        getPoint(id, x, y, z);
        return Vector3f(x, y, z);
    }


    virtual std::vector<float> getField(const std::string fieldname, std::vector<int> indices) ;

    virtual std::vector<float> getField(const std::string fieldname)
    {
        std::vector<float> out;

        if (!hasField(fieldname))
        {
            pcl::console::print_warn("[Error in %s] asked for field %s", getClassName().c_str(), fieldname.c_str());
            return out;
        }

        float val;
        for (int i = 0; i < size(); ++i)
        {
            getFieldValue(i, fieldname, val);
            out.push_back(val);
        }

        return out;

    }




    pcl::PointCloud<pcl::PointXYZ> applyTransform(const Transform<float, 3, Affine, AutoAlign> &T)
    {
        pcl::PointCloud<pcl::PointXYZ> cloud;



        for (int i = 0; i < size(); ++i)
        {
            Vector3f point = T * getPoint(i);
            pcl::PointXYZ p(point(0), point(1), point(2));
            cloud.push_back(p);
        }

        return cloud;
    }


};




}//end nspace

#endif // GENERIC_CLOUD_H
