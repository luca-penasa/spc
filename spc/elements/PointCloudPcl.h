#ifndef VOMBAT_SPCPCLCLOUD_H
#define VOMBAT_SPCPCLCLOUD_H

#include <spc/elements/PointCloudBase.h>
#include <spc/elements/point_types.h>

namespace spc
{

class PointCloudPCL : public PointCloudBase
{
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
    virtual void getFieldValue(const int id, const std::string fieldname,
                               float &val) const override;

    virtual void setFieldValue(const int id, const std::string fieldname,const float &val);


    virtual bool hasField(const std::string fieldname) const;

    virtual int size() const override;

    virtual void resize(size_t s) override;

private:


protected:
    pcl::PCLPointCloud2Ptr cloud_;

    Eigen::Array3i xyz_offsets_;

public:
    virtual std::vector<std::string> getFieldNames()
    {
        std::string names = pcl::getFieldsList(*cloud_);
        std::vector<std::string> fields;
        boost::split(fields, names, boost::is_any_of(" "), boost::token_compress_on);

        return  fields;
    }

public:
    virtual void addField(const std::string &name)
    {
        pcl::PointCloud<PointScalar> newcloud;
        newcloud.resize(cloud_->width * cloud_->height);

        pcl::PCLPointCloud2 c, out;
        pcl::toPCLPointCloud2(newcloud, c);

        c.fields[0].name = name;

        pcl::concatenateFields(*cloud_, c, out);

        *cloud_ = out;
    }




    // ElementBase interface
};

} // end nspace
#endif // SPCPCLCLOUD_H
