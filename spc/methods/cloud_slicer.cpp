#include <spc/methods/cloud_slicer.h>
#include <pcl/features/normal_3d.h>
#include <spc/common/point_types.h>
#include <pcl/search/flann_search.h>
#include <pcl/search/impl/flann_search.hpp>
//#include <pcl/search/impl/search.hpp>
#include <algorithm>
#include <spc/common/common.h>

#include <pcl/search/impl/search.hpp>

#include <pcl/filters/extract_indices.h>

//forced instantiation
template class pcl::search::FlannSearch<PointScalar>;
template class pcl::search::Search<PointScalar>;

struct PointScalarPointRep: public pcl::PointRepresentation<PointScalar>
{
    PointScalarPointRep() { this->nr_dimensions_ = 1; }
    /** \brief Empty destructor */
    virtual ~PointScalarPointRep() {}
    virtual void copyToFloatArray (const PointScalar& p, float* out) const { memcpy(out, &p.scalar, sizeof(float)); }
};


namespace spc
{

int
CloudSerializedSlicerOnField::compute()
{

    if (!in_reader_)
    {
        pcl::console::print_error("Specify a valid reader!");
        return -1;
    }

    std::vector<float> * sf = in_reader_->getScalarFieldAsStdVector<float>(field_name_);

    if (!sf)
    {
        pcl::console::print_error("Specify a valid scalar field name!");
        return -1;
    }


    pcl::PointCloud<PointScalar>::Ptr scalar_cloud (new pcl::PointCloud<PointScalar>);
    scalar_cloud->resize(sf->size());



    for (int i = 0 ; i < sf->size(); ++i)
    {
        PointScalar p;
        p.scalar = sf->at(i);
        scalar_cloud->at(i) = p;
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


    for (auto pos: intervals)
    {
        std::vector<int> indices;
        std::vector<float> dist;
        PointScalar point;
        point.scalar = pos;
        search_tree.radiusSearch(point, (double) slice_width_ / 2.0f, indices, dist);
        all_indices_.push_back(indices);       
    }


    return 1;
}



void
CloudSerializedSlicerOnField::setFieldName(std::string field_name)
{
    field_name_ = field_name;
}







} //end nspace
