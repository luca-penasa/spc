#include "NewSpcPointCloud.h"

namespace spc
{

DtiClassType NewSpcPointCloud::Type ("NewSpcPointCloud", &ElementBase::Type);


NewSpcPointCloud::NewSpcPointCloud(): sensor_(new OrientedSensor)
{
}

NewSpcPointCloud::Ptr NewSpcPointCloud::fromPointCloudBase(const PointCloudBase &other)
{
    NewSpcPointCloud::Ptr out(new NewSpcPointCloud);

    OrientedSensor::Ptr s = out->getSensor();
    s->setPosition(other.getSensor().getPosition());
    s->setOrientation(other.getSensor().getOrientation());

    out->conservativeResize(other.getNumberOfPoints());
    if (other.hasFields({"x", "y", "z"}))
    {
        out->addNewField("position", 3);

        for (int i = 0; i < other.getNumberOfPoints(); ++i)
        {
            Eigen::Vector3f p = other.getPoint(i);
            out->getFieldByName("position").row(i) = p;
        }
    }

    if (other.hasFields({"normal_x", "normal_y", "normal_z"}))
    {
        out->addNewField("normal", 3);

        for (int i = 0; i < other.getNumberOfPoints(); ++i)
        {
            Eigen::Vector3f p = other.getNormal(i);
            out->getFieldByName("normal").row(i) = p;
        }
    }

    for (std::string fname: other.getFieldNames())
    {
        if (fname == "x" | fname == "y" | fname == "z" | fname == "normal_x"| fname == "normal_y" | fname == "normal_z")
        {
            continue;
        }
        LOG(INFO) << "going to add field " << fname;

        out->addNewField(fname, 1);

        for (int i  =0; i < other.getNumberOfPoints(); ++i)
        {
            float val;
            other.getFieldValue(i, fname, val);
            out->getFieldByName(fname).row(i)(0) =  val;
        }


    }

    LOG(INFO) << "copy done";

    return out;
}




}


#include <spc/core/spc_cereal.hpp>
SPC_CEREAL_REGISTER_TYPE(spc::NewSpcPointCloud)
