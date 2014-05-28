#ifndef GENERIC_CLOUD_H
#define GENERIC_CLOUD_H

#include <spc/elements/ElementBase.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/io.h>

#include <spc/methods/common.h>


namespace spc
{

class spcGenericCloud
{

public:

    spcTypedefSharedPtrs(spcGenericCloud)

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
    virtual Eigen::Vector3f getPoint (const int id) const;


    virtual std::vector<float> getField(const std::string fieldname, std::vector<int> indices) ;

    virtual std::vector<float> getField(const std::string fieldname)
    {
        std::vector<float> out;

        if (!hasField(fieldname))
        {
            pcl::console::print_warn("[Error in point cloud] asked for field %s", fieldname.c_str());
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


    pcl::PointCloud<pcl::PointXYZ> applyTransform(const Eigen::Transform<float, 3, Eigen::Affine, Eigen::AutoAlign> &T);


};




}//end nspace

#endif // GENERIC_CLOUD_H
