#pragma once
#ifndef SPC_POINTCLOUDPCL_H
#define SPC_POINTCLOUDPCL_H

#include <spc/elements/PointCloudBase.h>
#include <spc/elements/point_types.h>
#include <spc/templated/PointSetBase.h>

namespace spc
{

class PointCloudPCL : public PointCloudBaseWithSensor
{

    using PointCloudBase::IndexT;


public:
    SPC_OBJECT(PointCloudPCL)
    EXPOSE_TYPE

    PointCloudPCL(pcl::PCLPointCloud2Ptr cloud);

    template <typename PointT>
    PointCloudPCL(const pcl::PointCloud<PointT> &cloud)
    {
        pcl::PCLPointCloud2Ptr c(new pcl::PCLPointCloud2);
        pcl::toPCLPointCloud2(cloud, *c );

        cloud_ = c;
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

    Eigen::Array3i xyz_offsets_;



};

} // end nspace


#endif
