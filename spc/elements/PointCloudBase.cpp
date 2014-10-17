#include <spc/elements/PointCloudBase.h>
namespace spc
{

DtiClassType PointCloudBase::Type = DtiClassType("PointCloudBase", &ElementBase::Type);

PointCloudBase::PointCloudBase()
{
    sensor_position_ << spcNANMacro, spcNANMacro, spcNANMacro;
}

int PointCloudBase::getNearestPointID(const Eigen::Vector3f query, float &sq_distance)
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

    pcl::search::FlannSearch
                <pcl::PointXYZ>::Ptr s (new pcl::search::FlannSearch<pcl::PointXYZ>);

    s->setInputCloud(getAsPclXyz());

    searcher_ = s;
}

void PointCloudBase::updateXYZRepresentation()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    cloud->resize(this->size());

    for (int i = 0; i < this->size(); ++i) {
        Eigen::Vector3f p = getPoint(i);
        cloud->at(i).x = p(0);
        cloud->at(i).y = p(1);
        cloud->at(i).z = p(2);
    }
    xyz_representation_ = cloud;
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
    for (int i = 0; i < size(); ++i) {
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

Eigen::Vector3f PointCloudBase::getPoint(const int id) const
{
    float x, y, z;
    getPoint(id, x, y, z);
    return Eigen::Vector3f(x, y, z);
}

Eigen::Vector3f PointCloudBase::getNormal(const int id) const
{
    float x, y, z;
    getNormal(id, x, y, z);
    return Eigen::Vector3f(x, y, z);
}

pcl::PointCloud<pcl::PointXYZ>
PointCloudBase::applyTransform(const Eigen::Transform
                               <float, 3, Eigen::Affine, Eigen::AutoAlign> &T)
{
    pcl::PointCloud<pcl::PointXYZ> cloud;

    for (int i = 0; i < size(); ++i) {
        Eigen::Vector3f point = T * getPoint(i);
        pcl::PointXYZ p(point(0), point(1), point(2));
        cloud.push_back(p);
    }

    return cloud;
}

std::vector<float> spc::PointCloudBase::getField(const std::string fieldname,
                                                  std::vector<int> indices)
{
    std::vector<float> out;

    if (!hasField(fieldname)) {
        pcl::console::print_warn("[Error in generic_cloud] asked for field %s",
                                 fieldname.c_str());
        return out;
    }

    float val;
    spcForEachMacro(int i, indices)
    {
        getFieldValue(i, fieldname, val);
        out.push_back(val);
    }

    return out;
}

} // end nspace
