#ifndef GENERIC_CLOUD_H
#define GENERIC_CLOUD_H

#include <spc/elements/ElementBase.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/io.h>

#include <spc/core/common.h>
#include <pcl/search/impl/flann_search.hpp>
#include <boost/make_shared.hpp>

#include <spc/elements/OrientedSensor.h>


#include <spc/elements/templated/PointSetBase.h>


namespace spc
{



class PointCloudBase : public ElementBase, public PointCloudXYZBase
{
public:

    using PointSetBase::IndexT;
    using PointSetBase::PointT;



    SPC_OBJECT(PointCloudBase)
    EXPOSE_TYPE
    PointCloudBase();

    virtual int getNearestPointID(const PointT query,
                                  float &sq_distance);


    //! sensor info access
    virtual OrientedSensor getSensor() const  = 0;

    virtual void setSensor(const OrientedSensor &sensor) const = 0;

    virtual void getPoint(const IndexT id, float &x, float &y, float &z) const
    {
        getFieldValue(id, "x", x);
        getFieldValue(id, "y", y);
        getFieldValue(id, "z", z);
    }

    virtual void setPoint(const IndexT id, const float x, const float y,
                          const float z)
    {
        setFieldValue(id, "x", x);
        setFieldValue(id, "y", y);
        setFieldValue(id, "z", z);
    }

    /// a generic cloud must implement these method
    virtual void getNormal(const IndexT id, float &x, float &y, float &z) const
    {
        getFieldValue(id, "normal_x", x);
        getFieldValue(id, "normal_y", y);
        getFieldValue(id, "normal_z", z);
    }

    virtual void setNormal(const IndexT id, const float x, const float y,
                           const float z)
    {
        setFieldValue(id, "normal_x", x);
        setFieldValue(id, "normal_y", y);
        setFieldValue(id, "normal_z", z);
    }

    virtual pcl::PointCloud<pcl::PointXYZ>::Ptr getAsPclXyz()
    {
        if (!xyz_representation_)
            updateXYZRepresentation();

        return xyz_representation_;
    }

    void updateFlannSearcher();

    virtual void updateXYZRepresentation();

    pcl::search::FlannSearch<pcl::PointXYZ>::Ptr getFlannSearcher()
    {
        if (!searcher_)
            updateFlannSearcher();

        return searcher_;
    }

    virtual void getFieldValue(const IndexT id, const std::string fieldname,
                               float &val) const = 0;

    virtual void setFieldValue(const IndexT, const std::string fieldname,
                               const float &val) = 0;

    virtual void addField(const std::string &name) = 0;

    virtual void addFields(const std::vector<std::string> field_names,
                           const Eigen::MatrixXf &data);

    virtual size_t getNumberOfPoints() const = 0;

    virtual bool hasField(const std::string fieldname) const = 0;

    virtual std::vector<std::string> getFieldNames() const = 0;

    virtual void resize(const IndexT s) = 0;

    virtual PointT getPoint(const IndexT id) const;

    virtual Eigen::Vector3f getNormal(const IndexT id) const;

//    template<class VEC>
    virtual std::vector<float> getField(const std::string fieldname,
                                        std::vector<IndexT> indices);


    virtual void getField(const std::string fieldname, const std::vector<IndexT> indices, Eigen::VectorXf & out);


    virtual std::vector<float> getField(const std::string fieldname);

    virtual bool getField(const std::string fieldname, Eigen::VectorXf &vector);

    pcl::PointCloud<pcl::PointXYZ>
    applyTransform(const Eigen::Transform
                   <float, 3, Eigen::Affine, Eigen::AutoAlign> &T);

    // PointSetBase interface
public:
    virtual void addPoint(const PointT &p) override
    {
        IndexT id = this->getNumberOfPoints();
        this->resize(id + 1);
        this->setPoint(id, p);
    }

    virtual void setPoint(const IndexT id, const PointT &p) override
    {
        this->setPoint(id,  p(0), p(1), p(2));
    }


    virtual pcl::PCLPointCloud2Ptr asPCLData() const ;

    //! reurns true if ALL the fields exists
    bool hasFields(const std::vector<std::string> &field_names) const;





protected:
    pcl::search::FlannSearch<pcl::PointXYZ>::Ptr searcher_;

    pcl::PointCloud<pcl::PointXYZ>::Ptr xyz_representation_;




};// end class


class PointCloudBaseWithSensor: public PointCloudBase
{

public:
    SPC_OBJECT(PointCloudBaseWithSensor)
    EXPOSE_TYPE
    PointCloudBaseWithSensor(): sensor_(new OrientedSensor)
    {
    }

    // PointCloudBase interface
public:
    virtual OrientedSensor getSensor() const
    {
        return *sensor_;
    }

    virtual void setSensor(const OrientedSensor &sensor) const
    {
        *sensor_ = sensor;
    }

protected:
    OrientedSensor::Ptr sensor_;
};


} // end nspace

#endif // GENERIC_CLOUD_H
