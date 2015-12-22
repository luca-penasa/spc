#include "CloudDataSourceOnDisk.h"

//#include <spc/elements/PointCloudBase.h>
#include <spc/elements/PointCloudPcl.h>
#include <spc/elements/NewSpcPointCloud.h>

//#include <pcl/io/pcd_io.h>
#include <spc/io/io_helper.h>

#include <boost/filesystem.hpp>

namespace spc {

using PathT = boost::filesystem::path;

DtiClassType CloudDataSourceOnDisk::Type("CloudDataSourceOnDisk", &ElementBase::Type);

PointCloudBase::Ptr CloudDataSourceOnDisk::load() const
{

    if (!exists()) {
        LOG(ERROR) << "Data source does not exists. maybe you moved it?  Please relocate also the reference here.";
        return NULL;
    }

    return io::loadPointCloud(filename_);
}

NewSpcPointCloud::Ptr CloudDataSourceOnDisk::load2() const
{
    NewSpcPointCloud::Ptr out = NewSpcPointCloud::fromPointCloudBase(*io::loadPointCloud(filename_));
    return out;
}

bool CloudDataSourceOnDisk::exists() const
{
    return boost::filesystem::exists(filename_);
}

std::string CloudDataSourceOnDisk::getExtension() const
{
    PathT p(filename_);
    return p.extension().string();
}

} //end nspace

#include <spc/core/spc_cereal.hpp>
SPC_CEREAL_REGISTER_TYPE(spc::CloudDataSourceOnDisk)
