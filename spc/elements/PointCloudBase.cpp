#include <spc/elements/PointCloudBase.h>

namespace spc
{

DtiClassType PointCloudBase::Type = DtiClassType("PointCloudBase", &ElementBase::Type);
DtiClassType PointCloudBaseWithSensor::Type = DtiClassType("PointCloudBaseWithSensor", &PointCloudBase::Type);


PointCloudBase::PointCloudBase()
{

}

int PointCloudBase::getNearestPointID(const PointCloudBase::PointT query, float &sq_distance)
{
    std::vector<float> sq_dists;
    std::vector<int> ids;
    pcl::PointXYZ p(query(0), query(1), query(2));
    getFlannSearcher()->nearestKSearch(p, 1, ids, sq_dists);

    sq_distance = sq_dists.at(0);
    return ids.at(0);
}

void PointCloudBase::updateFlannSearcher()
{
    DLOG(INFO)  << "updating flann index";
    pcl::search::FlannSearch
                <pcl::PointXYZ>::Ptr s (new pcl::search::FlannSearch<pcl::PointXYZ>);

    s->setInputCloud(getAsPclXyz());
    DLOG(INFO)  << "updating flann index. Done";


    searcher_ = s;
}

void PointCloudBase::updateXYZRepresentation()
{
    DLOG(INFO) << "updating XYZ representation of cloud ";      

    DLOG(INFO) << "got as pcl data ";

     pcl::PCLPointCloud2Ptr c = this->asPCLData();

     pcl::PointCloud<pcl::PointXYZ>::Ptr c2(new  pcl::PointCloud<pcl::PointXYZ>);
     pcl::fromPCLPointCloud2(*c, *c2);

    xyz_representation_ = c2;
    DLOG(INFO) << "updating XYZ representation of cloud. Done ";

}

void PointCloudBase::addFields(const std::vector<std::string> field_names,
                               const Eigen::MatrixXf &data)
{
    CHECK(field_names.size() == data.cols()) << "the field names and the column of data must be equal";
    CHECK (this->getNumberOfPoints() == data.rows()) << "the number of rows of data must be equal to the size of the cloud";

    IndexT field_counter = 0;
    for (std::string fname: field_names)
    {
        this->addField(fname);

#ifdef USE_OPENMP
#pragma omp parallel for
#endif
        for (int i = 0 ; i < data.rows(); ++i)
        {
            this->setFieldValue(i, fname, data( i, field_counter));
        }
        DLOG(INFO) << "field " << fname << " added";
        field_counter++;
    }


}

std::vector<float> PointCloudBase::getField(const std::string fieldname)
{

    PCL_DEBUG("Asked for field %s\n", fieldname.c_str());
    std::vector<float> out;

    if (!hasField(fieldname)) {
        pcl::console::print_warn(
                    "[Error in point cloud] asked for field %s", fieldname.c_str());
        return out;
    }

    float val;
    for (IndexT i = 0; i < getNumberOfPoints(); ++i) {
        getFieldValue(i, fieldname, val);
        out.push_back(val);
    }

    return out;
}

bool PointCloudBase::getField(const std::string fieldname, Eigen::VectorXf &vector)
{
    std::vector<float> stdvect = this->getField(fieldname);
    vector = Eigen::Map<Eigen::VectorXf>(stdvect.data(), stdvect.size());
    return true;
}

PointCloudBase::PointT PointCloudBase::getPoint(const IndexT id) const
{
    float x, y, z;
    getPoint(id, x, y, z);
    Eigen::Vector3f p;
    p << x, y, z;
    return p;
}

Eigen::Vector3f PointCloudBase::getNormal(const IndexT id) const
{
    float x, y, z;
    getNormal(id, x, y, z);
    return Eigen::Vector3f(x, y, z);
}

pcl::PointCloud<pcl::PointXYZ>
PointCloudBase::applyTransform(const Eigen::Transform
                               <float, 3, Eigen::Affine, Eigen::AutoAlign> &T)
{

    DLOG(INFO) << "applying transformation";
    pcl::PointCloud<pcl::PointXYZ> cloud;

    for (int i = 0; i < getNumberOfPoints(); ++i) {
        Eigen::Vector3f point = T * getPoint(i);
        pcl::PointXYZ p(point(0), point(1), point(2));
        cloud.push_back(p);
    }

    DLOG(INFO) << "applying transformation. Done. Cloud returned";

    return cloud;
}

pcl::PCLPointCloud2Ptr PointCloudBase::asPCLData() const
{
    // basic info from this
    std::vector<std::string> field_names = this->getFieldNames();

    IndexT n_fields = field_names.size();
    IndexT n_points = this->getNumberOfPoints();

    pcl::PCLPointCloud2::Ptr cloud (new pcl::PCLPointCloud2);

    cloud->width = 1;
    cloud->height = this->getNumberOfPoints();

    cloud->point_step =sizeof(float) * n_fields;
    cloud->row_step =sizeof(float) * n_fields;

    IndexT total_size = sizeof(float) * n_points * n_fields;
    cloud->data.resize(total_size);

    // set up fields
    std::vector<pcl::PCLPointField> fields;


    int counter = 0;
    for (std::string fname: field_names)
    {
        pcl::PCLPointField field;
        field.name = fname;
        field.offset = counter++ * sizeof(float);
        field.datatype = 7; // float, it would be 8 if it were double
        field.count = 1;

        fields.push_back(field);
    }


    DLOG(INFO) << "going to do hard conversion of cloud \n"
                  "This means no better method was found \n"
                  "Please reimplement this method in derived point cloud classes";
    float value;
#ifdef USE_OPENMP
#pragma omp parallel for private(value)
#endif
    for (int j = 0;  j < n_points; ++j)
    {
        for (int i = 0; i < n_fields; ++i)
        {
            std::string fname = field_names.at(i);
            this->getFieldValue(j, fname, value);
            memcpy(&cloud->data[0] + sizeof(float) *n_fields * j + i * sizeof(float),  &value, sizeof(float));
        }

    }


    cloud->fields = fields;
    DLOG(INFO) << "Done";

    return cloud;
}

bool PointCloudBase::hasFields(const std::vector<std::string> &field_names) const
{
    for (auto s: field_names)
        if (!hasField(s))
            return false;

    return true;
}

std::vector<float> PointCloudBase::getField(const std::string fieldname,
                                                 std::vector<IndexT> indices)
{
    std::vector<float> out;

    if (!hasField(fieldname)) {
        pcl::console::print_warn("[Error in generic_cloud] asked for field %s",
                                 fieldname.c_str());
        return out;
    }

    float val;
    for(int i: indices)
    {
        getFieldValue(i, fieldname, val);
        out.push_back(val);
    }

    return out;
}

void PointCloudBase::getField(const std::string fieldname, const std::vector<IndexT> indices, Eigen::VectorXf &out)
{
    if (!hasField(fieldname)) {
        pcl::console::print_warn("[Error in generic_cloud] asked for field %s",
                                 fieldname.c_str());
        return ;
    }

    out.resize(indices.size());


    float val;
    for (int i = 0 ; i < indices.size(); ++i)
    {
        getFieldValue(indices.at(i), fieldname, val);
        out(i) = val;
    }

}



} // end nspace
