#include <spc/elements/PointCloudBase.h>
namespace spc
{

PointCloudBase::PointCloudBase()
{
}

Eigen::Vector3f PointCloudBase::getPoint(const int id) const
{
    float x, y, z;
    getPoint(id, x, y, z);
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
