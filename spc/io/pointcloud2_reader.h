#ifndef POINTCLOUD2_READER_H
#define POINTCLOUD2_READER_H

#include <spc/common/common.h>


#ifdef QGEO
#include <ccPointCloud.h>
#endif
namespace spc
{

class PointCloud2Reader
{
public:
    union rgb_type{
    struct {
      u_int8_t r;
      u_int8_t g;
      u_int8_t b;
    };
    float rgb;
    };

    PointCloud2Reader() {}

    PointCloud2Reader(pcl::PCLPointCloud2::Ptr in_cloud) {setInputCloud(in_cloud);}

#ifdef QGEO
    PointCloud2Reader(ccPointCloud * in_cloud) {setInputCloud(in_cloud);}
#endif

    void setInputCloud(pcl::PCLPointCloud2::Ptr in_cloud) {resetInputs(); in_cloud_ = in_cloud;}

#ifdef QGEO
    void setInputCloud(ccPointCloud * in_cloud) {resetInputs(); in_cloud_cc_ = in_cloud;}
#endif

    void setIndices(std::vector<int> indices) {indices_ = indices;}

#ifdef QGEO
    void resetInputs() {in_cloud_ = boost::shared_ptr<pcl::PCLPointCloud2>(); in_cloud_cc_ = 0;}
#else
    void resetInputs() {in_cloud_ = boost::shared_ptr<pcl::PCLPointCloud2>();}
#endif
    template <typename ScalarT>
    typename std::vector<ScalarT> * getScalarFieldAsStdVector(std::string &field_name);

    std::vector<rgb_type> getRGB();

    size_t getNumberOfPoints();



private:

     void fillIndicesIfNeeded()
     {
         size_t n_points = this->getNumberOfPoints();
         if (indices_.size() == 0)
         {
             //we fill in the vector with all the ids!
             indices_.resize(n_points);
             for (int i = 0; i < n_points; ++i)
                 indices_.at(i) = i;
         }
     }

    pcl::PCLPointCloud2::Ptr in_cloud_;

#ifdef QGEO
    ccPointCloud * in_cloud_cc_;
#endif
    std::vector<int> indices_;
};

}

#endif // POINTCLOUD2_READER_H
