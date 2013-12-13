#ifndef STRATIGRAPHIC_EVALUATOR_H
#define STRATIGRAPHIC_EVALUATOR_H

#include <spc/stratigraphy/stratigraphic_model_base.h>



namespace spc
{


class StratigraphicEvaluator
{
public:
    StratigraphicEvaluator();

    void setInputModel(spcStratigraphicModelBase::ConstPtr model)
    {
        //ensure also the indices vector is clear
        model_ = model;
        indices_.clear();
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
    spcStratigraphicModelBase::ConstPtr  model_;


    //! \brief in_cloud_ is the input cluod on which to compute stratigraphic positions
    spc::spcGenericCloud::ConstPtr in_cloud_;


    //! \brief indices_ the set of int indices for which to compute the strat position
    std::vector<int> indices_;


    //! \brief output_ it the results of all computations
    std::vector<float> output_;


};

} //end nspace
#endif // STRATIGRAPHIC_EVALUATOR_H


