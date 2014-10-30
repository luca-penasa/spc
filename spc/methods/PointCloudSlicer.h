#ifndef CLOUD_SLICER_H
#define CLOUD_SLICER_H

#include <pcl/point_cloud.h>
#include <pcl/common/eigen.h>
#include <pcl/point_types.h>
//#include <spc/io/pointcloud2_reader.h>

namespace spc
{

class CloudSerializedSlicerOnField
{
public:
    CloudSerializedSlicerOnField()
    {
    }

//    void setInputReader(spc::PointCloud2Reader *reader)
//    {
//        in_reader_ = reader;
//    }

    void setSliceWidth(float slice_width)
    {
        slice_width_ = slice_width;
    }

    void setSliceStep(float slice_step)
    {
        slice_step_ = slice_step;
    }

    void setFieldName(std::string field_name);

    std::vector<std::vector<int>> getOutputIndices()
    {
        return all_indices_;
    }

    int compute();

private:
//    spc::PointCloud2Reader *in_reader_;
    float slice_width_;
    float slice_step_;
    int field_id_;
    std::string field_name_;

    std::vector<std::vector<int>> all_indices_;
};

} // end nspace
#endif // CLOUD_SLICER_H
