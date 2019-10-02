#ifndef ESTIMATE_AVERAGE_SAMPLING_STEP_H
#define ESTIMATE_AVERAGE_SAMPLING_STEP_H

#ifdef SPC_WITH_PCL

#include <spc/elements/PointCloudBase.h>
#include <pcl/filters/filter.h>
#include <pcl/search/search.h>

#include <pcl/filters/voxel_grid.h>

#include <pcl/search/kdtree.h>

#include <pcl/io/pcd_io.h>

#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/variance.hpp>
#include <boost/accumulators/statistics/mean.hpp>

#include <boost/bind.hpp>
#include <boost/ref.hpp>

namespace spc
{

using namespace pcl; // so when it will be merged in PCL no problem
using namespace boost::accumulators;
///
template <typename PointT>
class EstimateAverageSamplingStep : public Filter<PointT>
{
    using Filter<PointT>::input_;
    using Filter<PointT>::filter_name_;

    using Filter<PointT>::getClassName;

    typedef typename Filter<PointT>::PointCloud PointCloud;
    typedef typename Filter<PointT>::PointCloudPtr PointCloudPtr;
    typedef typename pcl::search::Search<PointT>::Ptr SearcherPtr;

public:
    typedef spcSharedPtrMacro<EstimateAverageSamplingStep<PointT>> Ptr;
    typedef spcSharedPtrMacro
        <const EstimateAverageSamplingStep<PointT>> ConstPtr;

    EstimateAverageSamplingStep()
        : step_avg_(spcNANMacro), step_std_(spcNANMacro)
    {
        filter_name_ = "EstimateAverageSamplingStep";
    }

    void setInputCloud(const PointCloudPtr &cloud)
    {
        input_ = cloud;
    }

    float getMean()
    {
        return step_avg_;
    }

    float getStd()
    {
        return step_std_;
    }

    virtual void applyFilter(PointCloud &output)
    {
        // do nohing for now
    }

    void applyFilter()
    {
        // In case a search method has not been given, initialize it using some
        // defaults
        if (!tree_) {
            // For organized datasets, use an OrganizedDataIndex
            tree_.reset(new pcl::search::KdTree<PointT>(false));
        }

        if (!input_) {
            PCL_ERROR("[pcl::EstimateAverageSamplingStep::applyFilter] Need an "
                      "input cloud befere continuing.\n");
            return;
        }

        tree_->setInputCloud(input_);

        std::vector<float> all_dists(input_->size());

#ifdef USE_OPENMP
#pragma omp parallel for shared(all_dists)
#endif
        for (int i = 0; i < input_->size(); ++i) {
            PointT point = input_->at(i);
            std::vector<int> ids;
            std::vector<float> sq_dists;

            tree_->nearestKSearch(point, 2, ids, sq_dists);

            all_dists.at(i) = sqrt(sq_dists.at(1));
        }

        // now we use boost for getting mean and variance
        accumulator_set<float, stats<tag::mean, tag::variance>> acc;

        std::for_each(all_dists.begin(), all_dists.end(),
                      boost::bind<void>(boost::ref(acc), _1));

        step_avg_ = mean(acc);
        step_std_ = sqrt(variance(acc));

        //        std::cout << step_std_ << " " << step_avg_ << std::endl;
    }

protected:
    /**
     * @brief squared_sigma_ for the gaussian kernel
     */
    float step_avg_;

    float step_std_;

    /** \brief A pointer to the spatial search object. */
    SearcherPtr tree_;
};

} // end nspace

#endif // CLOUD_GAUSSIAN_CONVOLVER_H
#endif
