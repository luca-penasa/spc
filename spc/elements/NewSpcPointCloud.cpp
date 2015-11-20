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
//        LOG(INFO) << "going to add field " << fname;

        out->addNewField(fname, 1);

        for (int i  =0; i < other.getNumberOfPoints(); ++i)
        {
            float val;
            other.getFieldValue(i, fname, val);
            out->getFieldByName(fname).row(i)(0) =  val;
        }


    }

//    LOG(INFO) << "copy done";

    return out;
}

NewSpcPointCloud NewSpcPointCloud::fromIds(const std::vector<size_t> &ids, const std::vector<std::string> &fields) const
{
    NewSpcPointCloud out;

    if (!fields.empty())
    {

        size_t ncols = 0;
        for (const std::string &fname : fields)
        {
            FieldLabel label = labels_.getLabelByName(fname);
            ncols += label.dimensionality_;
            out.addNewField(fname, label.dimensionality_);
        }

        out.conservativeResize(ids.size());

        size_t counter = 0;
        for (const size_t  &id: ids)
        {
            for (const std::string f: fields)
            {
                out.getFieldByName(f).row(counter) =  this->getFieldByName(f).row(id);
            }

            counter++;
        }

        return out;

    }


    else // cpy everything
    {
        out.fields_.conservativeResize(ids.size(), fields_.cols());
        out.field_to_col_ = field_to_col_;
        out.labels_ = this->labels_;
        *out.sensor_  = *sensor_;

        size_t counter = 0;
        for (const size_t &id: ids)
        {
            out.fields_.row(counter++) =  fields_.row(id);
        }

        return out;


    }

}

void NewSpcPointCloud::addNewField(const std::string &name, size_t dim)
{
//    DLOG(INFO) <<"adding new field wih name " << name << " and dim " << dim;
    FieldLabel newf(name, dim);
    fields_.conservativeResize(getNumberOfPoints(), fields_.cols() + dim);
    labels_.push_back(newf);

    field_to_col_[name] = fields_.cols() - dim;

    this->getFieldByName(name).fill(spcNANMacro);

//    DLOG(INFO) << "now dimensions are " << fields_.rows() << " x " << fields_.cols();
}

NewSpcPointCloud NewSpcPointCloud::filterOutNans(const std::vector<std::string> &fields) const
{
    Eigen::Matrix<bool, -1, 1> to_keep(fields_.rows());
    to_keep.fill(true); // none (rows) to remove at beginning

    for (const std::string f : fields) {
        to_keep.array() *= this->getFieldByName(f).finiteness().rowwise().prod().array();
    }

    std::vector<size_t> good_ids;
    for (int id = 0; id < fields_.rows(); ++id) {
        if (to_keep(id) == true) {
            good_ids.push_back(id);
        }
    }

    return this->fromIds(good_ids);
}


NewSpcPointCloud::EigenPlaneT NewSpcPointCloud::fitPlane(Vector3f &eigenvalues) const
{
    Eigen::VectorXf avg;

    Eigen::Matrix<ScalarT, -1, -1> covmat =this->getFieldByName("position").getSampleCovMatAndAvg(avg);

    EigenPlaneT plane;

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix<ScalarT, 3, 3>> eig(covmat);
    plane.normal() = eig.eigenvectors().col(0);
    eigenvalues = eig.eigenvalues();
    plane.offset() = - plane.normal().dot(avg);

    return plane;
}

void NewSpcPointCloud::concatenate(const NewSpcPointCloud &other)
{

    for (int i = 0 ; i < other.labels_.size(); ++i)
    {
        FieldLabel lab = other.labels_.getLabel(i);
        if (!labels_.hasLabel(lab))
        {
            LOG(WARNING) << "nothin done. some fields missing";
            return;
        }
    }

    size_t old_size = this->getNumberOfPoints();
    this->conservativeResize(old_size + other.getNumberOfPoints());

    this->fields_.bottomRows(this->getNumberOfPoints() - old_size) = other.fields_;
}




}


#include <spc/core/spc_cereal.hpp>
SPC_CEREAL_REGISTER_TYPE(spc::NewSpcPointCloud)
