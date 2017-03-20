#ifndef DYNAMIC_SCALAR_FIELD_EVALUATOR_H
#define DYNAMIC_SCALAR_FIELD_EVALUATOR_H

#include <spc/elements/VariableScalarFieldBase.h>

namespace spc
{

template <typename ScalarT, typename idtype = size_t>
class DynamicScalarFieldEvaluator
{
public:

    typedef Eigen::Matrix<ScalarT, -1, 1> VectorT;
    DynamicScalarFieldEvaluator()
    {
    }

    void setGenerator(VariableScalarFieldBase::ConstPtr model)
    {
        // ensure also the indices vector is clear
        model_ = model;
    }

    void setInputCloud(spc::PointCloudBase::ConstPtr in_cloud)
    {
        in_cloud_ = in_cloud;
    }

    void setIndices(const std::vector<idtype> indices)
    {
        indices_ = indices;
    }

    int compute()
    {
        if (!model_) {
            LOG(ERROR) << "Model not found. set one before to compute";
            return -1;
        }

        if (!in_cloud_) {
            LOG(ERROR) << "in cloud not found. set one before to compute";
            return -1;
        }

        // populate indices if needed
        if (indices_.empty()) {
            // fill it with all the ids
            for (int i = 0; i < in_cloud_->getNumberOfPoints(); ++i)
                indices_.push_back(i);
        }

        output_.resize(indices_.size());

//#ifdef USE_OPENMP
//#pragma omp parallel for
//#endif
        for (int i = 0; i < indices_.size(); ++i) {
            int id = indices_.at(i);
            Eigen::Vector3f point = in_cloud_->getPoint(id);
            float val = model_->getScalarFieldValue(point);

//		LOG(INFO) << "point " << point << " value " << val;
            output_(i) = val;
        }

        //    output_ = out; // do a copy

        return 1; // to confirm everything is fine
    }

    Eigen::VectorXf getOutput()
    {
        return output_;
    }

private:
    ///
    /// \brief model_ is a pointer to a stratigraphic model that implements the
    /// virtual methods of
    /// a StratigraphicModelBase
    ///
    VariableScalarFieldBase::ConstPtr model_;

    //! \brief in_cloud_ is the input cluod on which to compute stratigraphic
    //positions
    spc::PointCloudBase::ConstPtr in_cloud_;

    //! \brief indices_ the set of int indices for which to compute the values
    //position
    std::vector<idtype> indices_;

    //! \brief output_ it the results of all computations
    VectorT output_;
};

template<typename ScalarT, typename idtype = size_t>
Eigen::Matrix<ScalarT, -1, 1> evaluate_dynamic_scalar_field_generator(const spc::PointCloudBase::ConstPtr in_cloud,
                                                                      const spc::VariableScalarFieldBase::ConstPtr model,
                                                                      const std::vector<idtype> & indices = std::vector<idtype>())
{

    LOG(INFO) << "evaluating the scalar field";
    spc::DynamicScalarFieldEvaluator<ScalarT, idtype> eval;
    eval.setInputCloud(in_cloud);
    eval.setIndices(indices);
    eval.setGenerator(model);
    eval.compute();
    LOG(INFO) << "evaluating the scalar field. Done";
    LOG(INFO) << "Values are: \n" <<eval.getOutput().transpose() ;


    return eval.getOutput();
}

} // end nspace

#endif // guard



