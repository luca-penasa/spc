#ifndef DYNAMIC_SCALAR_FIELD_EVALUATOR_H
#define DYNAMIC_SCALAR_FIELD_EVALUATOR_H

#include <spc/elements/VariableScalarFieldBase.h>

namespace spc
{


class DynamicScalarFieldEvaluator
{
public:
    DynamicScalarFieldEvaluator();

    void setGenerator(DynamicScalarFieldGenerator::ConstPtr model)
    {
        //ensure also the indices vector is clear
        model_ = model;
    }

    void setInputCloud( spc::spcGenericCloud::ConstPtr in_cloud) {in_cloud_ = in_cloud;}

    void setIndices(const std::vector<int> indices) {indices_ = indices;}

    int compute();

    std::vector<float> getOutput()
    {
        return output_;
    }

private:
    ///
    /// \brief model_ is a pointer to a stratigraphic model that implements the virtual methods of
    /// a StratigraphicModelBase
    ///
    DynamicScalarFieldGenerator::ConstPtr  model_;


    //! \brief in_cloud_ is the input cluod on which to compute stratigraphic positions
    spc::spcGenericCloud::ConstPtr in_cloud_;


    //! \brief indices_ the set of int indices for which to compute the strat position
    std::vector<int> indices_;


    //! \brief output_ it the results of all computations
    std::vector<float> output_;


};

} //end nspace

#endif // guard

