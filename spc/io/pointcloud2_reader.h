#ifndef POINTCLOUD2_READER_H
#define POINTCLOUD2_READER_H

#include <sensor_msgs/PointCloud2.h>


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

    PointCloud2Reader(sensor_msgs::PointCloud2::Ptr in_cloud) {setInputCloud(in_cloud);}

    void setInputCloud(sensor_msgs::PointCloud2::Ptr in_cloud) {in_cloud_ = in_cloud;}

    void setIndices(std::vector<int> indices) {indices_ = indices;}

    template <typename ScalarT>
    auto getScalarFieldAsStdVector(std::string &field_name) -> std::vector<ScalarT> * ;

     std::vector<rgb_type> getRGB();



private:

     void fillIndicesIfNeeded()
     {
         size_t n_points = in_cloud_->width * in_cloud_->height;
         if (indices_.size() == 0)
         {
             //we fill in the vector with all the ids!
             indices_.resize(n_points);
             for (int i = 0; i < n_points; ++i)
                 indices_.at(i) = i;
         }
     }

    sensor_msgs::PointCloud2::Ptr in_cloud_;

    std::vector<int> indices_;
};

}

#endif // POINTCLOUD2_READER_H
