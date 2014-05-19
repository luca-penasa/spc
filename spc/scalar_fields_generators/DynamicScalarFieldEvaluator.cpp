#include "DynamicScalarFieldEvaluator.h"
namespace spc
{

DynamicScalarFieldEvaluator::DynamicScalarFieldEvaluator()
{
}

int DynamicScalarFieldEvaluator::compute()
{
    //clear the output
    output_.clear();

    if (!model_)
    {
        pcl::console::print_error("Model not found. set one before to compute");
        return -1;
    }

    std::cout << "HERE" << model_ << std::endl;

    if (!in_cloud_)
    {
        pcl::console::print_error("in cloud not found. set one before to compute");
        return -1;
    }


    //populate indices if needed
    if (indices_.empty())
    {
        //fill it with all the ids
        for (int i = 0 ; i < in_cloud_->size(); ++i)
            indices_.push_back(i);
    }

    std::vector<float> out(indices_.size());
//    out.resize(indices_.size());

    //now perform computations
    //#idfef USE_OPENMP
    //#pragma omp parallel for shared (out)
    //#endif
    for (int i = 0 ; i < indices_.size(); ++i)
    {
        int id = indices_.at(i);
        Eigen::Vector3f point = in_cloud_->getPoint(id);
        out.at(i) = model_->getScalarFieldValue(point);

    }


    output_ = out; //do a copy

    return 1; //to confirm everything is fine
}

}//end nspace
