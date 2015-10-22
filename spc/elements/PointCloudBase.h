#ifndef GENERIC_CLOUD_H
#define GENERIC_CLOUD_H

#include <spc/elements/ElementBase.h>

#ifdef SPC_WITH_PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/io.h>
#include <pcl/search/impl/flann_search.hpp>
#endif


#include <spc/core/common.h>
#include <boost/make_shared.hpp>
#include <spc/elements/OrientedSensor.h>
#include <spc/elements/templated/PointSet.h>


namespace spc
{



class PointCloudBase : public ElementBase, public PointCloudXYZBase
{
public:

    using PointSetBase::IndexT;
    using PointSetBase::PointT;

//    SPC_ELEMENT(PointCloudBase)

    spcTypedefSharedPtrs(PointCloudBase)
    EXPOSE_TYPE
    PointCloudBase();

    PointCloudBase(const PointCloudBase &other): ElementBase(other), PointCloudXYZBase(other)
    {
#ifdef SPC_WITH_PCL

        searcher_ = other.searcher_;
        xyz_representation_ = other.xyz_representation_;
#endif
    }

    ~PointCloudBase()
    {

    }
#ifdef SPC_WITH_PCL

    virtual int getNearestPointID(const PointT query,
                                  float &sq_distance);

#endif
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

#ifdef SPC_WITH_PCL
    virtual pcl::PointCloud<pcl::PointXYZ>::Ptr getAsPclXyz()
    {
        if (!xyz_representation_)
            updateXYZRepresentation();

        return xyz_representation_;
    }
#endif
    PointSet<> extractAsPointSet(const std::vector<size_t> &ids = std::vector<size_t>()) const
    {
        PointSet<> out;

        if (ids.size() == 0)
        {
            out.resize(getNumberOfPoints());
#ifdef USE_OPENMP
#pragma omp parallel for
#endif
            for (int i = 0; i < getNumberOfPoints(); ++i)
            {
                out.setPoint(i, this->getPoint(i));
            }
        }

        else
        {
            out.resize(ids.size());
#ifdef USE_OPENMP
#pragma omp parallel for
#endif
            for (int i = 0; i < ids.size(); ++i)
            {
                out.setPoint(i, this->getPoint(ids.at(i)));
            }
        }

        return out;
    }


    template<typename SPCCloudT>
    PointCloudBase::Ptr extractIDS(const std::vector<size_t> &ids) const
    {
        PointCloudBase::Ptr out (new SPCCloudT());

        out->resize(ids.size());

        float v;
        for (std::string fname: this->getFieldNames())
        {
           out->addField(fname);

            for (int i = 0; i < ids.size(); ++i)
            {
                this->getFieldValue(ids.at(i), fname, v);
                out->setFieldValue(i, fname, v);
            }

        }

        return out;
    }

#ifdef SPC_WITH_PCL
    void updateFlannSearcher();

    virtual void updateXYZRepresentation();

    pcl::search::FlannSearch<pcl::PointXYZ>::Ptr getFlannSearcher()
    {
        if (!searcher_)
            updateFlannSearcher();

        return searcher_;
    }
#endif

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

    virtual std::vector<float> getField(const std::string fieldname,
                                        std::vector<IndexT> indices);


    virtual void getField(const std::string fieldname, const std::vector<IndexT> indices, Eigen::VectorXf & out);


    virtual std::vector<float> getField(const std::string fieldname);

    virtual bool getField(const std::string fieldname, Eigen::VectorXf &vector);

#ifdef SPC_WITH_PCL

    pcl::PointCloud<pcl::PointXYZ>
    applyTransform(const Eigen::Transform
                   <float, 3, Eigen::Affine, Eigen::AutoAlign> &T);

#endif
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

#ifdef SPC_WITH_PCL
    virtual pcl::PCLPointCloud2Ptr asPCLData() const ;
#endif

    //! reurns true if ALL the fields exists
    bool hasFields(const std::vector<std::string> &field_names) const;

protected:
#ifdef SPC_WITH_PCL
    pcl::search::FlannSearch<pcl::PointXYZ>::Ptr searcher_;

    pcl::PointCloud<pcl::PointXYZ>::Ptr xyz_representation_;
#endif



};// end class


class PointCloudBaseWithSensor: public PointCloudBase
{

public:
    spcTypedefSharedPtrs(PointCloudBaseWithSensor)
    EXPOSE_TYPE
    PointCloudBaseWithSensor(): sensor_(new OrientedSensor)
    {
    }

    PointCloudBaseWithSensor(const PointCloudBaseWithSensor & other)
    {
        sensor_ = spcDynamicPointerCast<OrientedSensor>(other.sensor_->clone());
    }

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
