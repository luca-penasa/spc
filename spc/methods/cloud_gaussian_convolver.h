#ifndef CLOUD_GAUSSIAN_CONVOLVER_H
#define CLOUD_GAUSSIAN_CONVOLVER_H

#include <spc/elements/generic_cloud.h>

#include <pcl/filters/filter.h>
#include <pcl/search/search.h>

#include <pcl/filters/voxel_grid.h>

#include <pcl/search/kdtree.h>

#include <pcl/io/pcd_io.h>

namespace spc
{

using namespace pcl; // so when it will be merged in PCL no problem

///
/// \brief Convolve an existent scalar field with a gaussian kernel
template <typename PointT>
class GaussianConvolver: public Filter<PointT>
{
    using Filter<PointT>::input_;
    using Filter<PointT>::filter_name_;

    using Filter<PointT>::getClassName;

    typedef typename Filter<PointT>::PointCloud PointCloud;
    typedef typename Filter<PointT>::PointCloudPtr PointCloudPtr;
    typedef typename pcl::search::Search<PointT>::Ptr SearcherPtr;

public:

    typedef boost::shared_ptr< GaussianConvolver<PointT> > Ptr;
    typedef boost::shared_ptr< const GaussianConvolver<PointT> > ConstPtr;

    GaussianConvolver(): filter_field_name_("intensity"), squared_sigma_(1.0f), down_width_(0.1f), support_factor_(2.0)
    {
        filter_name_ = "GaussianConvolver";
    }

    void setInputCloud(const PointCloudPtr &cloud)
    {
        input_ = cloud;
        downsampled_.reset(); //reset its downsampled version
    }

    std::vector<float> getOutput()
    {
        return smoothed_values_;
    }

    void setKernelSigma(const float sigma)
    {
        squared_sigma_ = sigma * sigma; //we keep it as squared so it is pre-computed
        computeSupportSize();
    }

    void setSupportFactor(const float fact)
    {
        support_factor_ = fact;
        computeSupportSize();
    }



    void setFieldName(const std::string fieldname)
    {
        filter_field_name_ = fieldname;
    }

    void setDownsapleWidth(const float value)
    {
        down_width_ = value;
        downsampled_.reset();
    }

    virtual void
    applyFilter (PointCloud &output)
    {
        //do nohing for now
    }

    void applyFilter ()
    {
        // In case a search method has not been given, initialize it using some defaults
        if (!tree_)
        {
            // For organized datasets, use an OrganizedDataIndex
            tree_.reset (new pcl::search::KdTree<PointT> (false));
        }

        if (!input_)
        {
            PCL_ERROR ("[pcl::GaussianConvolver::applyFilter] Need an input cloud befere continuing.\n");
            return;
        }

//        if (input_->is_dense) # this is simply wrong -> check out the meaning of is_dense please. You mean isOrganized maybe
//        {
//            PCL_ERROR ("[pcl::%s::applyFilter] This method ony works on sparse clouds.\n", getClassName ().c_str ());
//            return;
//        }

        if (!downsampled_)
            prepareDownsampledVersion();



//was for debug
//        pcl::io::savePCDFileBinary("/home/luca/tmp.pcd", *downsampled_);




        tree_->setInputCloud (downsampled_);


        // Copy the input data into the output

        smoothed_values_.resize(input_->size());

#ifdef DUSE_OPENMP
    #pragma omp parallel for shared (smoothed_values_)
#endif
        for (int i = 0 ; i < input_->size(); ++i)
        {
            PointT point = input_->at(i);
            std::vector<int> ids;
            std::vector<float> sq_dists;

            tree_->radiusSearch(point, support_size_, ids, sq_dists);


            std::vector<float> values = getFieldValuesInDownsampled(ids);

            // now compute weights
            std::vector<float> w(values.size());

            for (int ix = 0 ; ix < ids.size(); ++ix)
                w.at(ix) = exp (- (sq_dists.at(ix))/(2*squared_sigma_));

            std::vector<float> mult(ids.size());
            std::transform(values.begin(), values.end(), w.begin(), mult.begin(), std::multiplies<float>());
            float sum = std::accumulate(mult.begin(), mult.end(), 0.0f);
            float sum_w = std::accumulate(w.begin(), w.end(), 0.0f);

            smoothed_values_.at(i) = sum/sum_w;

        }

    }


private:

    /** \brief The Gaussian distance kernel.
      * \param[in] x the spatial distance
      * \param[in] sigma standard deviation of the Gaussian function
      */
    inline double
    kernel (double x, double sigma)
    {
        return (exp (- (x*x)/(2*sigma*sigma)));
    }

    inline std::vector<float>
    getFieldValuesInDownsampled(const std::vector<int> ids)
    {
        std::vector<pcl::PCLPointField> fields;
        int distance_idx = pcl::getFieldIndex (*input_, filter_field_name_, fields);
        if (distance_idx == -1)
        {
            PCL_WARN ("[pcl::%s::applyFilter] Unable to find field name in point type.\n", getClassName ().c_str ());
            return std::vector<float>(0); //a zero length vector
        }

        std::vector<float> out;
        out.resize(ids.size());

        for (int i = 0 ; i < ids.size(); ++i)
        {
            const uint8_t* pt_data = reinterpret_cast<const uint8_t*> (&downsampled_->points[ids[i]]);
            memcpy (&out[i], pt_data + fields[distance_idx].offset, sizeof (float));
        }

        return out;
    }

    /** \brief Prepare a downsampled version of the input cloud
     */
    void prepareDownsampledVersion()
    {

        if (!downsampled_)
            downsampled_ = PointCloudPtr(new PointCloud);

        pcl::VoxelGrid<PointT> gridder;
        gridder.setInputCloud(input_);
        gridder.setDownsampleAllData(true); //downsample also fields
        gridder.setLeafSize(down_width_, down_width_, down_width_);
        gridder.filter(*downsampled_);
    }

    void computeSupportSize()
    {
        support_size_ = support_factor_ * sqrt(squared_sigma_);
    }


protected:
    /**
     * @brief squared_sigma_ for the gaussian kernel
     */
    float squared_sigma_;

    /**
     * @brief down_width_ leaf size for voxelgridding the input cloud and its fields
     */
    float down_width_;
    /**
     * @brief support_ the search radius size that will be used will be
     * given by support_ * sigma, a two-sigma support is pretty common
     */
    float support_factor_;

    /**
     * @brief support_size_ is the actual size of support region in world units
     */
    float support_size_;

    /**
     * @brief smoothed_values_ is the output
     */
    std::vector<float> smoothed_values_;

    /**
     * @brief field_name_ the name of the field to be smoothed out
     */
    std::string filter_field_name_;

    /**
     * @brief downsampled_ keeps a downsampled version of the input cloud
     */
    PointCloudPtr downsampled_;

    /** \brief A pointer to the spatial search object. */
    SearcherPtr tree_;

};

}//end nspace

#endif // CLOUD_GAUSSIAN_CONVOLVER_H
