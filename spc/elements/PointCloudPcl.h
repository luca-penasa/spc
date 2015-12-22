#pragma once
#ifndef SPC_POINTCLOUDPCL_H
#define SPC_POINTCLOUDPCL_H

#include <spc/elements/PointCloudBaseWithSensor.h>
#include <spc/elements/point_types.h>
//#include <spc/elements/templated/PointSetBase.h>

#ifdef SPC_WITH_PCL


namespace spc
{

//! In several places here we assume that the fields are FLOATS!
//! \todo implement a field-type aware class!
class PointCloudPCL : public PointCloudBaseWithSensor
{

    using PointCloudBase::IndexT;


public:
    SPC_ELEMENT(PointCloudPCL)
    EXPOSE_TYPE

    //! def const
    PointCloudPCL()
    {

        pcl::PointCloud<pcl::PointXYZ> cloud;

        pcl::PCLPointCloud2Ptr c(new pcl::PCLPointCloud2);
        pcl::toPCLPointCloud2(cloud, *c );


        cloud_ = c;
    }


    //! copy const
    PointCloudPCL (const PointCloudPCL &other): PointCloudBaseWithSensor(other)
    {
        cloud_ = other.cloud_;
    }

    //! wrap a PCL point cloud
    PointCloudPCL(pcl::PCLPointCloud2Ptr cloud);

    //! wrap a PCL point clou of any type
    template <typename PointT>
    PointCloudPCL(const pcl::PointCloud<PointT> &cloud)
    {
        pcl::PCLPointCloud2Ptr c(new pcl::PCLPointCloud2);
        pcl::toPCLPointCloud2(cloud, *c );

        cloud_ = c;
    }

    //! destructor. smart pointers take care of themselves
    ~PointCloudPCL()
    {

    }


    //// we assume here that is a float the value to be extracted
    virtual void getFieldValue(const IndexT id, const std::string fieldname,
                               float &val) const override;

    virtual void setFieldValue(const IndexT id, const std::string fieldname,const float &val) override;


    virtual bool hasField(const std::string fieldname) const override;

    virtual size_t getNumberOfPoints() const override;

    virtual void resize(const IndexT s) override;


public:
    virtual std::vector<std::string> getFieldNames() const override;


    virtual void addField(const std::string &name) override;


    virtual pcl::PCLPointCloud2Ptr asPCLData() const override
    {
        return cloud_;
    }


protected:
    pcl::PCLPointCloud2Ptr cloud_;

//    Eigen::Array3i xyz_offsets_;



};

} // end nspace


#endif
#endif //#ifdef SPC_WITH_PCL
