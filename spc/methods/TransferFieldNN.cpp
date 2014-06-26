#include "TransferFieldNN.h"
namespace spc {

void PointCloudHelpers::transferNormals(PointCloudBase::Ptr from, PointCloudBase::Ptr to, const float &max_distance)
{
    to->addField("normal_x");
    to->addField("normal_y");
    to->addField("normal_z");

    float d;
    for (int i = 0; i < to->size(); ++i) {

        if (i %100 == 0)
            std::cout << i << std::endl;
        int nn_id = from->getNearestPointID(to->getPoint(i), d);
        if (sqrt(d) > max_distance)
            to->setNormal(i, spcNANMacro, spcNANMacro, spcNANMacro);
        else {
            Eigen::Vector3f n = from->getNormal(nn_id);
            to->setNormal(i, n(0), n(1), n(2));
        }
    }



}

void PointCloudHelpers::computeScatteringAngle(PointCloudBase::Ptr cloud, const std::string fieldname)
{
    if( !cloud->hasField("normal_x") )
    {
        PCL_ERROR("no normals in the cloud.\n");
        return;
    }

    Eigen::Vector3f position = cloud->getSensorPosition();
    if (pcl_isnan(position(0)))
    {
        PCL_ERROR("no valid sensor position in the cloud.\n");
        return;
    }

    cloud->addField(fieldname);

    for (int i  = 0; i < cloud->size(); ++i)
    {
        Eigen::Vector3f p = cloud->getPoint(i);
        Eigen::Vector3f n = cloud->getNormal(i);
        Eigen::Vector3f ray = p - position;

        ray.normalize();
        n.normalize();

        float cosTheta = ray.dot(n);
        float theta = acos(std::min(fabs(cosTheta), 1.0));

        theta *= 180.0 / M_PI;

        cloud->setFieldValue(i, "angle",theta);
    }



}

void PointCloudHelpers::computeDistanceFromSensor(PointCloudBase::Ptr cloud, std::string fieldname)
{
    Eigen::Vector3f position = cloud->getSensorPosition();
    if (pcl_isnan(position(0)))
    {
        PCL_ERROR("no valid sensor position in the cloud.\n");
        return;
    }

    cloud->addField(fieldname);

    for (int i  = 0; i < cloud->size(); ++i)
    {
        Eigen::Vector3f p = cloud->getPoint(i);

        Eigen::Vector3f d = p -position;

        cloud->setFieldValue(i, "distance", d.norm());
    }


}




}
