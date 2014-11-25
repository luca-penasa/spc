#ifndef NEWSPCPOINTCLOUD_H
#define NEWSPCPOINTCLOUD_H
#include <spc/elements/ElementBase.h>
#include <map>
#include <spc/elements/PointCloudBase.h>

#include <spc/elements/OrientedSensor.h>

#include <spc/core/spc_eigen.h>

namespace spc
{

class FieldLabel
{
public:

    FieldLabel(): field_name_("None"), dimensionality_(0)
    {

    }

    FieldLabel(const std::string & name, const size_t dim)
    {
        field_name_ = name;
        dimensionality_ = dim;
    }

    bool operator== (const FieldLabel &other) const
    {
        return (this->field_name_ == other.field_name_) && (this->dimensionality_ == other.dimensionality_);
    }

    std::string field_name_;
    size_t dimensionality_;

private:
    friend class cereal::access;

    template <class Archive> void serialize(Archive &ar)
    {
        ar(CEREAL_NVP(field_name_),
           CEREAL_NVP(dimensionality_)
           );
    }




};

class LabelsContainer
{
public:
    bool hasField(const std::string &name) const
    {
        for (const FieldLabel& label: labels_)
        {
            if (label.field_name_ ==name)
                return true;

        }
        return false;
    }

    FieldLabel getLabelByName(const std::string name) const
    {
        for (const FieldLabel& label: labels_)
        {
            if (label.field_name_ ==name)
                return label;
        }
    }

    bool hasLabel(const FieldLabel & label2) const
    {
        for (const FieldLabel& label: labels_)
        {
            if (label == label2)
                return true;

        }
        return false;
    }

    void push_back(const FieldLabel & lab)
    {
        labels_.push_back(lab);
    }


    size_t size() const
    {
        return labels_.size();
    }

    FieldLabel &getLabel(const size_t id)
    {
        return labels_.at(id);
    }

    FieldLabel getLabel(const size_t id) const
    {
        return labels_.at(id);
    }

    std::vector<FieldLabel> &getLabels()
    {
        return labels_;
    }

    std::vector<FieldLabel> getLabels() const
    {
        return labels_;
    }

private:
    friend class cereal::access;

    template <class Archive> void serialize(Archive &ar)
    {
            ar(CEREAL_NVP(labels_));
    }

    std::vector<FieldLabel> labels_;
};



class NewSpcPointCloud: public ElementBase
{
public:

    SPC_ELEMENT(NewSpcPointCloud)
    EXPOSE_TYPE

    typedef float ScalarT;
    typedef Eigen::Matrix<ScalarT, Eigen::Dynamic, Eigen::Dynamic > MatrixT;
    typedef Eigen::Block<MatrixT> BlockT;

    typedef Eigen::Hyperplane<ScalarT, 3> EigenPlaneT;

    typedef const Eigen::Block<const MatrixT> ConstBlockT;

    typedef NanoFlannEigenBlockAdaptor<MatrixT> SearcherT;



    NewSpcPointCloud();

    //! static from old types
    static NewSpcPointCloud::Ptr fromPointCloudBase(const spc::PointCloudBase &other);


    //! by default it will copy ALL the fields for the requested ids.
    //! it is often better to provide a fields list to copy
    NewSpcPointCloud fromIds(const std::vector<size_t> &ids, const std::vector<std::string> &fields = {})
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
            out.fields_.conservativeResize(ids.size(), this->getData().cols());

            out.labels_ = this->labels_;

            size_t counter = 0;
            for (const size_t &id: ids)
            {
                out.fields_.row(counter++) =  this->getData().row(id);
            }

            return out;


        }

    }

    BlockT getData()
    {
        return fields_.block(0, 0, fields_.rows(), fields_.cols());
    }



    void addNewField(const std::string & name, size_t dim = 1)
    {
        LOG(INFO) <<"adding new field wih name " << name << " and dim " << dim;
        FieldLabel newf(name, dim);
        fields_.conservativeResize(getNumberOfPoints(), fields_.cols() + dim);
        labels_.push_back(newf);

        field_to_col_[name] = fields_.cols() - dim;

        this->getFieldByName(name).fill(spcNANMacro);

        LOG(INFO) << "now dimensions are " << fields_.rows() << " x " << fields_.cols();
    }

    size_t getNumberOfPoints() const
    {
        return fields_.rows();
    }

    BlockT getFieldByName(const std::string &name)
    {
        size_t id = field_to_col_.at(name);
        size_t dim =  labels_.getLabelByName(name).dimensionality_;
        return fields_.block(0, id, getNumberOfPoints(), dim);
    }

    ConstBlockT getFieldByName(const std::string &name) const
    {
        size_t id = field_to_col_.at(name);
        size_t dim =  labels_.getLabelByName(name).dimensionality_;
        return fields_.block(0, id, getNumberOfPoints(), dim);
    }


    void conservativeResize(const size_t n_points)
    {
        fields_.resize(n_points, fields_.cols());
    }


    void updateSearcher(const std::string & on_field = "position")
    {
        if (searcher_ == NULL)
            searcher_ = typename SearcherT::Ptr (new SearcherT(this->getFieldByName("position"), 20));
    }

    void resetSearcher()
    {
        searcher_ = NULL;
    }

    Eigen::Vector3f getCentroid()
    {
        return getFieldByName("position").colwise().sum() / getNumberOfPoints();
    }

    SearcherT::Ptr getSearcher(const std::string & on_field = "position")
    {
        updateSearcher(on_field);
        return searcher_;
    }

    OrientedSensor::Ptr getSensor()
    {
        return sensor_;
    }

    EigenPlaneT fitPlane(Eigen::Vector3f &eigenvalues) const
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

    //! only if all the fields are present in both concatenation will work fine
    void concatenate (const NewSpcPointCloud &other)
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

    virtual bool isAsciiSerializable() const override
    {
        return true;
    }

    virtual int toAsciiStream(std::ostream &stream) const override
    {
        for (const FieldLabel & label: labels_.getLabels())
        {
            stream << label.field_name_ << ":" << label.dimensionality_ << " ";
        }
        stream << "\n";
        stream << fields_;

        return 1;
    }

private:
    friend class cereal::access;

    template <class Archive> void serialize(Archive &ar)
    {
        ar(cereal::base_class<spc::ElementBase> (this),
           CEREAL_NVP(fields_),
           CEREAL_NVP(labels_),
           CEREAL_NVP(field_to_col_),
//           CEREAL_NVP(searcher_),
           CEREAL_NVP(sensor_)
           );
    }


protected:
    MatrixT fields_;

    LabelsContainer labels_;

    std::map<std::string, size_t> field_to_col_;

    SearcherT::Ptr searcher_;

    OrientedSensor::Ptr sensor_;



};

}

#endif // NEWSPCPOINTCLOUD_H
