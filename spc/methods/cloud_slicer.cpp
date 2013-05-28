#include <spc/methods/cloud_slicer.h>
#include <pcl/features/normal_3d.h>
#include <spc/common/point_types.h>
#include <pcl/search/flann_search.h>
#include <algorithm>
#include <spc/common/common.h>

#include <pcl/filters/extract_indices.h>

struct PointScalarPointRep: public pcl::PointRepresentation<PointScalar>
{
    PointScalarPointRep() { this->nr_dimensions_ = 1; }
    /** \brief Empty destructor */
    virtual ~PointScalarPointRep() {}
    virtual void copyToFloatArray (const PointScalar& p, float* out) const { memcpy(out, &p.scalar, sizeof(float)); }
};


namespace spc
{


CloudSerializedSlicerOnField::CloudSerializedSlicerOnField()
{
}


int CloudSerializedSlicerOnField::updateSecondaryAttributes()
{

    //get the index of the field
    field_id_ = pcl::getFieldIndex(*in_cloud_, field_name_);
    if (field_id_ == -1)
    {
        pcl::console::print_error("Error. You must specify a valid field name to be used\n");
        return (-1);
    }
}

int
CloudSerializedSlicerOnField::compute()
{
    updateSecondaryAttributes();

    //we transform the whole cloud to a simple cloud with just a X coordinate
    int point_step = in_cloud_->point_step;
    sensor_msgs::PointField field = in_cloud_->fields.at(field_id_);
    sensor_msgs::PointCloud2::Ptr in_cloud_ptr = boost::make_shared<sensor_msgs::PointCloud2> (*in_cloud_);
    int field_offset = field.offset;

    pcl::PointCloud<PointScalar>::Ptr scalar_cloud (new pcl::PointCloud<PointScalar>);
    scalar_cloud->resize(in_cloud_->width * in_cloud_->height);

    for (int i = 0; i < in_cloud_->width * in_cloud_->height; ++i )
    {
        memcpy(&scalar_cloud->at(i).scalar, in_cloud_->data.data() + i * point_step + field_offset, sizeof(float));
    }

    pcl::search::FlannSearch<PointScalar> search_tree;
    PointScalarPointRep::Ptr rep (new PointScalarPointRep);
    search_tree.setPointRepresentation( rep );
    search_tree.setInputCloud(scalar_cloud);

    // get max and min of this scalar field
    PointScalar min = *std::min_element(scalar_cloud->begin(), scalar_cloud->end(), [&](const PointScalar &a, const PointScalar &b){return a.scalar < b.scalar;} );
    PointScalar max = *std::max_element(scalar_cloud->begin(), scalar_cloud->end(), [&](const PointScalar &a, const PointScalar &b){return a.scalar < b.scalar;} );

    float min_s = min.scalar;
    float max_s = max.scalar;

    //now subdivide this range in equally spaced positions.
    auto intervals = subdivideRange<float>( min.scalar, max.scalar, slice_step_);

    std::vector<std::vector<int>> all_indices;
    std::vector<sensor_msgs::PointCloud2::Ptr> all_small_clouds;

    for (auto pos: intervals)
    {
        std::vector<int> indices;
        std::vector<float> dist;
        PointScalar point;
        point.scalar = pos;
        search_tree.radiusSearch(point, (double) slice_width_ / 2.0f, indices, dist);
        all_indices_.push_back(indices);

        pcl::ExtractIndices<sensor_msgs::PointCloud2> filter;
        filter.setInputCloud(in_cloud_ptr);
        filter.setIndices(boost::make_shared<std::vector<int>> (indices));

        sensor_msgs::PointCloud2::Ptr small_cloud (new sensor_msgs::PointCloud2);
        filter.filter(*small_cloud);

        all_small_clouds_.push_back(small_cloud);
    }


    return 1;
}



void
CloudSerializedSlicerOnField::setFieldName(std::string field_name)
{
    field_name_ = field_name;
}







} //end nspace
