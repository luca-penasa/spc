#ifndef CLOUDDATASOURCEONDISK_H
#define CLOUDDATASOURCEONDISK_H

#include <spc/elements/ElementBase.h>
#include <boost/filesystem.hpp>

//#include <spc/elements/PointCloudBase.h>
#include <spc/elements/PointCloudPcl.h>
#include <spc/elements/NewSpcPointCloud.h>
#include <pcl/io/pcd_io.h>
#include <spc/io/io_helper.h>

namespace spc
{


class CloudDataSourceOnDisk: public ElementBase
{
public:

    typedef boost::filesystem::path PathT;

    SPC_ELEMENT(CloudDataSourceOnDisk)
    EXPOSE_TYPE

    CloudDataSourceOnDisk(const std::string &fname)
    {
        if (PathT(fname).extension() != ".pcd")
        {
            LOG(ERROR) << "You created a cloudOnDisk ref to a non pcd file.";
        }

        filename_ = fname;

        setElementName(fname);
    }

    //def const
    CloudDataSourceOnDisk()
    {

    }

    bool exists() const
    {
        return boost::filesystem::exists(filename_);
    }

    std::string getExtension() const
    {
        PathT p(filename_);
        return p.extension().string();
    }


    spcSetMacro(Filename, filename_, std::string)
    spcGetMacro(Filename, filename_, std::string)

    PointCloudBase::Ptr load() const
    {

        if (!exists())
        {
            LOG(ERROR) << "Data source does not exists. maybe you moved it?  Please relocate also the reference here.";
            return NULL;
        }

        if (getExtension() != ".pcd")
        {
            LOG(ERROR) << "We are pointing to a non .pcd file!";
            return NULL;
        }

        return io::loadPointCloud(filename_);
    }

    NewSpcPointCloud::Ptr load2() const
    {
        NewSpcPointCloud::Ptr out = NewSpcPointCloud::fromPointCloudBase(*io::loadPointCloud(filename_));
        return out;
    }

protected:
    std::string filename_;

private:
    friend class cereal::access;

    template <class Archive> void serialize(Archive &ar)
    {
        ar(cereal::base_class<spc::ElementBase>(this), CEREAL_NVP(filename_));
    }


};

}// end nspace

#endif // CLOUDDATASOURCEONDISK_H
