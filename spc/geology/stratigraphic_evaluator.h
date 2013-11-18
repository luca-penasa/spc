#ifndef STRATIGRAPHIC_EVALUATOR_H
#define STRATIGRAPHIC_EVALUATOR_H

#include <spc/stratigraphy/stratigraphic_model_base.h>



namespace spc
{


class StratigraphicEvaluator
{
public:
    StratigraphicEvaluator();

    void setInputModel(StratigraphicModelBase * model)
    {
        //ensure also the indices vector is clear
        model_ = model;
        indices_.clear();
    }

    void setInputCloud( const pcl::PointCloud<pcl::PointXYZ>::Ptr  in_cloud) {in_cloud_ = in_cloud;}

    void setIndices(const std::vector<int> indices) {indices_ = indices;}

    int compute()
    {
        //clear the output
        output_.clear();

        //populate indices if needed
        if (indices_.empty())
        {
            //fill it with all the ids
            for (int i = 0 ; i < in_cloud_->size(); ++i)
                indices_.push_back(i);
        }

        std::vector<float> out;
        out.resize(indices_.size());

        //now perform computations
//#idfef USE_OPENMP
//#pragma omp parallel for shared (out)
//#endif
        for (int i = 0 ; i < indices_.size(); ++i)
        {
            int id = indices_.at(i);
            pcl::PointXYZ point = in_cloud_->at(id);
            Eigen::Vector3f p(point.x, point.y, point.z);

            out.at(i) = model_->getStratigraphicPosition(p);

        }


        output_ = out; //do a copy

        return 1; //to confirm everything is fine
    }

    std::vector<float> getOutput()
    {
        return output_;
    }

private:
    ///
    /// \brief model_ is a pointer to a stratigraphic model that implements the virtual methods of
    /// a StratigraphicModelBase
    ///
    StratigraphicModelBase * model_;


    //! \brief in_cloud_ is the input cluod on which to compute stratigraphic positions
    pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_;


    //! \brief indices_ the set of int indices for which to compute the strat position
    std::vector<int> indices_;


    //! \brief output_ it the results of all computations
    std::vector<float> output_;


};

} //end nspace
#endif // STRATIGRAPHIC_EVALUATOR_H


