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

    spcPCLCloud(CloudPtrT cloud);

    spcPCLCloud(const pcl::PointCloud<PointT> &cloud);

    virtual void getPoint(const int id, float &x, float &y, float &z) const;


    virtual void setPoint(const int id, const float x, const float y, const float z);



    //// we assume here that is a float the value to be extracted
    virtual void getFieldValue(const int id, const std::string fieldname, float &val);

    virtual bool hasField(const std::string fieldname);



    virtual int size() const;

    virtual void resize(size_t s);


protected:
    // the actual data
    CloudPtrT cloud_;
};



}//end nspace
#endif // SPCPCLCLOUD_H
