#ifndef NEWSPCPOINTCLOUD_H
#define NEWSPCPOINTCLOUD_H
#include <spc/elements/ElementBase.h>
#include <map>
#include <spc/elements/PointCloudBase.h>

#include <spc/elements/OrientedSensor.h>

#include <spc/core/spc_eigen.h>

namespace spc
{

struct FieldLabel
{


    FieldLabel(const std::string & name, const size_t dim)
    {
        field_name_ = name;
        dimensionality_ = dim;
    }

    bool operator== (const FieldLabel &other)
    {
        return (this->field_name_ == other.field_name_) && (this->dimensionality_ == other.dimensionality_);
    }



    std::string field_name_;
    size_t dimensionality_;
};

struct LabelsContainer: std::vector<FieldLabel>
{
    bool hasField(const std::string &name) const
    {
        for (const FieldLabel& label: *this)
        {
            if (label.field_name_ ==name)
                return true;

        }
        return false;
    }

    FieldLabel getLabelByName(const std::string name) const
    {
        for (const FieldLabel& label: *this)
        {
            if (label.field_name_ ==name)
                return label;
        }
    }

};



class NewSpcPointCloud
{
public:

    typedef float ScalarT;
    typedef Eigen::Matrix<ScalarT, Eigen::Dynamic, Eigen::Dynamic > MatrixT;
    typedef Eigen::Block<MatrixT> BlockT;

    typedef const Eigen::Block<const MatrixT> ConstBlockT;

    typedef NanoFlannEigenMatrixAdaptor<MatrixT> SearcherT;

    spcTypedefSharedPtrs(NewSpcPointCloud)


    NewSpcPointCloud();

    //! static from old types
    static NewSpcPointCloud::Ptr fromPointCloudBase(const spc::PointCloudBase &other);


    NewSpcPointCloud fromIds(const std::vector<size_t> &ids, const std::vector<std::string> &fields = {"position"})
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

            size_t counter;
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

            size_t counter;
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


    void updateSearcher()
    {
        if (searcher_ == NULL)
            searcher_ = typename SearcherT::Ptr (new SearcherT(this->getFieldByName("position"), 20));
    }

    Eigen::Vector3f getCentroid()
    {
        return getFieldByName("position").colwise().sum() / getNumberOfPoints();
    }

    SearcherT::Ptr getSearcher()
    {
        updateSearcher();
        return searcher_;
    }

    OrientedSensor::Ptr getSensor()
    {
        return sensor_;
    }

    Eigen::Hyperplane<ScalarT, 3> fitPlane(Eigen::Vector3f &eigenvalues) const
    {
        Eigen::VectorXf avg;
        Eigen::Matrix<ScalarT, -1, -1> covmat =this->getFieldByName("position").getSampleCovMatAndAvg(avg);

        Eigen::Hyperplane<ScalarT, 3> plane;

        Eigen::SelfAdjointEigenSolver<Eigen::Matrix<ScalarT, 3, 3>> eig(covmat);
        plane.normal() = eig.eigenvectors().col(0);
        eigenvalues = eig.eigenvalues();
        plane.offset() = - plane.normal().dot(avg);

        return plane;
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
