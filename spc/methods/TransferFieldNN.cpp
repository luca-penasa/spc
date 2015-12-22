#include "TransferFieldNN.h"
#include <spc/elements/OrientedSensor.h>
namespace spc {
#ifdef SPC_WITH_PCL

int PointCloudHelpers::transferNormals(PointCloudBase::Ptr from, PointCloudBase::Ptr to, const float &max_distance)
{
    to->addField("normal_x");
    to->addField("normal_y");
    to->addField("normal_z");

    float d;
#ifdef USE_OPENMP
#pragma omp parallel for private(d)
#endif
    for (int i = 0; i < to->getNumberOfPoints(); ++i)
    {
        int nn_id = from->getNearestPointID(to->getPoint(i), d);
        if (sqrt(d) > max_distance)
            to->setNormal(i, spcNANMacro, spcNANMacro, spcNANMacro);
        else {
            Eigen::Vector3f n = from->getNormal(nn_id);
            to->setNormal(i, n(0), n(1), n(2));
        }
    }

    return 1;



}

int PointCloudHelpers::transferFieldsNN(PointCloudBase::Ptr from, PointCloudBase::Ptr to,
                                        const float &max_distance,
                                        std::vector<std::string> fields)
{
    if (!from->hasFields(fields))
    {
        LOG(WARNING) << "one or more fields missing in input cloud. Cannot copy the fields";
        return -1;
    }


    CHECK_GT(max_distance, 0);


    for (std::string name: fields)
    {
        to->addField(name);
    }


    from->updateFlannSearcher();

    float value;
    float d;


#ifdef USE_OPENMP
#pragma omp parallel for private(value, d)
#endif
    for (int i = 0; i < to->getNumberOfPoints(); ++i)
    {

        // nearest point in from
        int nn_id = from->getNearestPointID(to->getPoint(i), d);

        if (sqrt(d) > max_distance)
        {
            for (std::string sname: fields)
            {
                to->setFieldValue(i, sname, spcNANMacro);
            }
        }

        else
        {
            for (std::string sname: fields)
            {
                from->getFieldValue(nn_id,sname, value);
                to->setFieldValue(i, sname, value);
            }
        }
    }

    return 1;
}


int PointCloudHelpers::computeScatteringAngle(PointCloudBase::Ptr cloud, const std::string angle_fieldname)
{
    if( !cloud->hasField("normal_x") )
    {
        LOG(WARNING) << "Normals must be present in input cloud. Cannot compue angles";
        return -1;
    }

    Eigen::Vector4f position = cloud->getSensor().getPosition();

    if (position == Eigen::Vector4f::Zero())
    {
        LOG(WARNING) << "looks like the sensor position is [0,0,0], going ahead considering this as sensor position";
    }

    if (pcl_isnan(position(0)) || pcl_isnan(position(1)) ||pcl_isnan(position(2)))
    {
        LOG(WARNING) << "sensor position contains NANS. Not valid.";
        return -1;
    }

    cloud->addField(angle_fieldname);

#ifdef USE_OPENMP
#pragma omp parallel for
#endif
    for (int i  = 0; i < cloud->getNumberOfPoints(); ++i)
    {
        Eigen::Vector3f p = cloud->getPoint(i);
        Eigen::Vector3f n = cloud->getNormal(i);
        Eigen::Vector3f ray = p - position.head(3);

        ray.normalize();
        n.normalize();

        float cosTheta = ray.dot(n);
		float theta = acos(std::min((float) std::abs(cosTheta), 1.0f));

        theta *= 180.0 / M_PI;

        cloud->setFieldValue(i, angle_fieldname,theta);
    }

    return 1;



}

int PointCloudHelpers::computeDistanceFromSensor(PointCloudBase::Ptr cloud, std::string fieldname)
{
    Eigen::Vector3f position = cloud->getSensor().getPosition().head(3);


    if (position == Eigen::Vector3f::Zero())
    {
        LOG(WARNING) << "looks like the sensor position is [0,0,0], going ahead considering this as sensor position";
    }

    if (pcl_isnan(position(0)) || pcl_isnan(position(1)) ||pcl_isnan(position(2)))
    {
        LOG(WARNING) << "sensor position contains NANS. Not valid.";
        return -1;
    }


    cloud->addField(fieldname);


#ifdef USE_OPENMP
#pragma omp parallel for
#endif

    for (int i  = 0; i < cloud->getNumberOfPoints(); ++i)
    {
        Eigen::Vector3f p = cloud->getPoint(i);
        Eigen::Vector3f d = p -position;
        cloud->setFieldValue(i, fieldname, d.norm());
    }

    return 1;

}

#endif


}
