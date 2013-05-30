#include "pointcloud2_reader.h"
#include <pcl/io/io.h>

namespace spc
{



template <typename ScalarT>
auto PointCloud2Reader::getScalarFieldAsStdVector(std::string &field_name) -> std::vector<ScalarT> *
{
    int id = pcl::getFieldIndex(*in_cloud_, field_name);
    if (id == -1)
    {
        pcl::console::print_error("Requested scalar field in pointcloud2reader not exists!");
        return 0; //return a null pointer
    }

    //now we get all the infos about this field
    sensor_msgs::PointField field = in_cloud_->fields.at(id);

    size_t offset = field.offset;
    size_t point_step = in_cloud_->point_step;
    size_t size = pcl::getFieldSize(field.datatype);

    this->fillIndicesIfNeeded();

    std::vector<ScalarT> * out_vector = new std::vector<ScalarT>(indices_.size());

    if (size == sizeof(ScalarT)) //simple copy!
    {
        for (auto i: indices_)
            memcpy(out_vector->data() + i, in_cloud_->data.data() + i*point_step + offset, sizeof(ScalarT));

    }
    else //we MUST get a cast to make the data fit!
    {

        if (size == 4) //float or int
        {
            float data;
            for (auto i: indices_)
            {
                memcpy(&data, in_cloud_->data.data() + i*point_step + offset, sizeof(float));
                out_vector->at(i) = (ScalarT) data;
            }
        }
        else if (size == 8) //double
        {
            double data;
            for (auto i: indices_)
            {
                memcpy(&data, in_cloud_->data.data() + i*point_step + offset, sizeof(double));
                out_vector->at(i) = (ScalarT) data;
            }
        }
        else //something went wrong
        {
            pcl::console::print_error("something wrong in the pointcloud2 reader! - wrong sized of fields");
            return 0;
        }

    }


    return out_vector;


}

std::vector< PointCloud2Reader::rgb_type > PointCloud2Reader::getRGB()
{
    this->fillIndicesIfNeeded();
    std::vector< PointCloud2Reader::rgb_type > rgb_out(indices_.size());
    std::string name = "rgb";
    std::vector<float> out = *this->getScalarFieldAsStdVector<float>(name);
    memcpy(rgb_out.data(), out.data(), sizeof(float) * indices_.size());
    return rgb_out;
}




} //end nspace
