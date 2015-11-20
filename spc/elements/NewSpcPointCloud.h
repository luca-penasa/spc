#ifndef NEWSPCPOINTCLOUD_H
#define NEWSPCPOINTCLOUD_H
#include <spc/elements/ElementBase.h>
#include <map>
#include <spc/elements/PointCloudBase.h>

#include <spc/elements/OrientedSensor.h>

#include <spc/core/spc_eigen.h>

namespace spc {

class FieldLabel {
public:
    FieldLabel()
        : field_name_("None")
        , dimensionality_(0)
    {
    }

    FieldLabel(const std::string& name, const size_t dim)
    {
        field_name_ = name;
        dimensionality_ = dim;
    }

    bool operator==(const FieldLabel& other) const
    {
        return (this->field_name_ == other.field_name_) && (this->dimensionality_ == other.dimensionality_);
    }

    std::string field_name_;
    size_t dimensionality_;

private:
    friend class cereal::access;

    template <class Archive>
    void serialize(Archive& ar)
    {
        ar(CEREAL_NVP(field_name_),
            CEREAL_NVP(dimensionality_));
    }
};

class LabelsContainer {
public:
    bool hasField(const std::string& name) const
    {
        for (const FieldLabel& label : labels_) {
            if (label.field_name_ == name)
                return true;
        }
        return false;
    }

    FieldLabel getLabelByName(const std::string name) const
    {
        for (const FieldLabel& label : labels_) {
            if (label.field_name_ == name)
                return label;
        }
    }

    bool hasLabel(const FieldLabel& label2) const
    {
        for (const FieldLabel& label : labels_) {
            if (label == label2)
                return true;
        }
        return false;
    }

    void push_back(const FieldLabel& lab)
    {
        labels_.push_back(lab);
    }

    size_t size() const
    {
        return labels_.size();
    }

    FieldLabel& getLabel(const size_t id)
    {
        return labels_.at(id);
    }

    FieldLabel getLabel(const size_t id) const
    {
        return labels_.at(id);
    }

    std::vector<FieldLabel>& getLabels()
    {
        return labels_;
    }

    std::vector<FieldLabel> getLabels() const
    {
        return labels_;
    }

private:
    friend class cereal::access;

    template <class Archive>
    void serialize(Archive& ar)
    {
        ar(CEREAL_NVP(labels_));
    }

    std::vector<FieldLabel> labels_;
};

class NewSpcPointCloud : public ElementBase {
public:
    SPC_ELEMENT(NewSpcPointCloud)
    EXPOSE_TYPE

    typedef float ScalarT; // using float, useful if we templatize the class
    //! \todo we should move to a rowmajor format for the matrix.
    //! but we must also take care of the correspondent serialization function for cereal.
    //! that will create bad stuff on reloading. (I verified this!!!)
    typedef Eigen::Matrix<ScalarT, Eigen::Dynamic, Eigen::Dynamic> MatrixT;
    typedef Eigen::Block<MatrixT> BlockT;

    typedef Eigen::Hyperplane<ScalarT, 3> EigenPlaneT;

    typedef const Eigen::Block<const MatrixT> ConstBlockT;

    typedef NanoFlannEigenBlockAdaptor<MatrixT> SearcherT;

    NewSpcPointCloud();

    //! static from old types
    static NewSpcPointCloud::Ptr fromPointCloudBase(const spc::PointCloudBase& other);

    //! by default it will copy ALL the fields for the requested ids.
    //! it is often better to provide a fields list to copy
    NewSpcPointCloud fromIds(const std::vector<size_t>& ids, const std::vector<std::string>& fields = {}) const;

    BlockT getData()
    {
        return fields_.block(0, 0, fields_.rows(), fields_.cols());
    }

    LabelsContainer getLabels() const
    {
        return labels_;
    }

    void addNewField(const std::string& name, size_t dim = 1);

    size_t getNumberOfPoints() const
    {
        return fields_.rows();
    }

    BlockT getFieldByName(const std::string& name)
    {
        size_t id = field_to_col_.at(name);
        size_t dim = labels_.getLabelByName(name).dimensionality_;
        return fields_.block(0, id, getNumberOfPoints(), dim);
    }

    ConstBlockT getFieldByName(const std::string& name) const
    {
        size_t id = field_to_col_.at(name);
        size_t dim = labels_.getLabelByName(name).dimensionality_;
        return fields_.block(0, id, getNumberOfPoints(), dim);
    }

    MatrixT getFieldAsMatrixByName(const std::string& name) const
    {
        return getFieldByName(name);
    }

    NewSpcPointCloud filterOutNans(const std::vector<std::string>& fields) const;

    void conservativeResize(const size_t n_points)
    {
        size_t before = fields_.rows();
        fields_.conservativeResize(n_points, fields_.cols());
        fields_.bottomRows(fields_.rows() - before).fill(spcNANMacro);
    }

    void updateSearcher(const std::string& on_field = "position")
    {
        if (searcher_ == NULL)
            searcher_ = SearcherT::Ptr(new SearcherT(this->getFieldByName("position"), 20));
    }

    void resetSearcher()
    {
        searcher_ = NULL;
    }

    Eigen::Vector3f getCentroid()
    {
        return getFieldByName("position").colwise().sum() / getNumberOfPoints();
    }

    SearcherT::Ptr getSearcher(const std::string& on_field = "position")
    {
        updateSearcher(on_field);
        return searcher_;
    }

    OrientedSensor::Ptr getSensor()
    {
        return sensor_;
    }

    //! \note returns eigenvalues in increasing order
    EigenPlaneT fitPlane(Eigen::Vector3f& eigenvalues) const;

    bool hasField(const std::string& fname) const
    {
        return labels_.hasField(fname);
    }

    //! only if all the fields are present in both concatenation will work fine
    void concatenate(const NewSpcPointCloud& other);

    virtual bool isAsciiSerializable() const override
    {
        return true;
    }

    virtual int toAsciiStream(std::ostream& stream) const override
    {
        for (const FieldLabel& label : labels_.getLabels()) {
            for (size_t i = 0; i < label.dimensionality_; ++i) {
                stream << label.field_name_ << ":" << i << " ";
            }
            //            stream << label.field_name_ << ":" << label.dimensionality_ ;
        }
        stream << "\n";
        stream << fields_;

        return 1;
    }

private:
    friend class cereal::access;

    template <class Archive>
    void serialize(Archive& ar)
    {
        ar(cereal::base_class<spc::ElementBase>(this),
            CEREAL_NVP(fields_),
            CEREAL_NVP(labels_),
            CEREAL_NVP(field_to_col_),
            //           CEREAL_NVP(searcher_),
            CEREAL_NVP(sensor_));
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
