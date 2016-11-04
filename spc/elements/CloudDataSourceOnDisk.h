#pragma once
#ifndef CLOUDDATASOURCEONDISK_H
#define CLOUDDATASOURCEONDISK_H

#include <spc/core/ElementBase.h>

namespace spc {

spcFwdDeclSharedPtr(PointCloudBase)
    spcFwdDeclSharedPtr(NewSpcPointCloud)

        class CloudDataSourceOnDisk : public ElementBase {
public:
    SPC_ELEMENT(CloudDataSourceOnDisk)
    EXPOSE_TYPE

    CloudDataSourceOnDisk(const std::string& fname)
    {
        filename_ = fname;
        setElementName(fname);
    }

    //def const
    CloudDataSourceOnDisk()
    {
    }

    bool exists() const;

    std::string getExtension() const;

    spcSetMacro(Filename, filename_, std::string)
    spcGetMacro(Filename, filename_, std::string)

    PointCloudBasePtr load() const;

    NewSpcPointCloudPtr load2() const;

protected:
    std::string filename_;

private:
    friend class cereal::access;

    template <class Archive>
    void serialize(Archive& ar, const std::uint32_t version)
    {
        ar(cereal::base_class<spc::ElementBase>(this), CEREAL_NVP(filename_));
    }
};

} // end nspace

#endif // CLOUDDATASOURCEONDISK_H
