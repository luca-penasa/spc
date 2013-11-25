#ifndef GENERIC_CLOUD_H
#define GENERIC_CLOUD_H

#include "element_base.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
namespace spc
{




class spcGenericCloud: public spcElementBase, public SalvableObject
{
public:
    spcGenericCloud();

    virtual std::string getSPCClassName()
    {
        std::string name = "GenericCloud";
        return name;
    }

    virtual int toAsciiMeOnly(std::stringstream &stream) {/*nothing for now*/}


    /// a generic cloud must implement these method
    virtual void getPoint(const int id, float &x, float &y, float &z) = 0 ;

    virtual void getFieldValue(const int id, const std::string fieldname, float &val) = 0;

    virtual int getSize() const = 0 ;


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


class spcPCLXYZCloud: public pcl::PointCloud<pcl::PointXYZ>, public spcGenericCloud
{
public:
    spcPCLXYZCloud()
    {

    }

    spcPCLXYZCloud (const pcl::PointCloud<pcl::PointXYZ> & other): pcl::PointCloud<pcl::PointXYZ> (other)
    {

    }

    virtual void getPoint(const int id, float &x, float &y, float &z)
    {
        x = at(id).x;
        y = at(id).y;
        z = at(id).z;
    }

    virtual void getFieldValue(const int id, const std::string fieldname, float &val)
    {

    }

    virtual int getSize() const
    {
        return size();
    }
};


}//end nspace

#endif // GENERIC_CLOUD_H
