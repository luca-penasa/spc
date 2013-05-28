#ifndef CLOUD_SLICER_H
#define CLOUD_SLICER_H

#include <pcl/point_cloud.h>
#include <pcl/common/eigen.h>
#include <pcl/point_types.h>
#include <sensor_msgs/PointCloud2.h>

namespace spc
{



class CloudSerializedSlicerOnField
{
public:   
    CloudSerializedSlicerOnField();

    void
    setInputCloud(sensor_msgs::PointCloud2 * in_cloud) {in_cloud_ = in_cloud;}

    std::vector<sensor_msgs::PointCloud2::Ptr>
    getSlicedClouds() {return all_small_clouds_;}

    void
    setSliceWidth(float slice_width) {slice_width_ = slice_width;}

    void
    setSliceStep(float slice_step) {slice_step_ = slice_step;}

    void
    setFieldName(std::string field_name) ;

    int compute();


private:
    sensor_msgs::PointCloud2 * in_cloud_;
    float slice_width_;
    float slice_step_;
    int field_id_;
    std::string field_name_;

    std::vector<std::vector<int>> all_indices_;
    std::vector<sensor_msgs::PointCloud2::Ptr> all_small_clouds_;

    int
    updateSecondaryAttributes();
};




} //end nspace
#endif // CLOUD_SLICER_H
