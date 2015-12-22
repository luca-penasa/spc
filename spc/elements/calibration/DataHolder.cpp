#include "DataHolder.h"

#include <list>

#include <spc/elements/NewSpcPointCloud.h>
#include <spc/elements/calibration/KeyPoint.h>
#include <spc/elements/calibration/Observation.h>
#include <spc/elements/CloudDataSourceOnDisk.h>
#include <spc/elements/OrientedSensor.h>

namespace spc
{
namespace calibration
{

DtiClassType DataHolder::Type ("DataHolder", &ElementBase::Type);

DataHolder::DataHolder()
{
}

NewSpcPointCloudPtr DataHolder::asPointCloud() const
{
    std::vector<CloudDataSourceOnDiskPtr> sources = getDataSources();
    std::map<CloudDataSourceOnDiskPtr, size_t> source_to_id;

    size_t id = 0;
    for (CloudDataSourceOnDiskPtr s: sources)
        source_to_id[s] = id++;



    spc::NewSpcPointCloud::Ptr out(new spc::NewSpcPointCloud);

	size_t total_number= getTotalNumberOfObservations();

    DLOG(INFO) << "we have a total number of entries: " << total_number;

    out->addNewField("post_position", 3);
    out->addNewField("original_position", 3);


    out->addNewField("distance", 1);
    out->addNewField("angle", 1);
    out->addNewField("intensity", 1);

    out->addNewField("n_neighbors", 1);
    out->addNewField("intensity_std", 1);
    out->addNewField("eigen_ratio", 1);

    out->addNewField("material", 1);
    out->addNewField("keypoint_id", 1);
    out->addNewField("source_id", 1);
    out->addNewField("progressive_id", 1);

    out->addNewField("normal", 3);
    out->addNewField("lambdas", 3);


	out->addNewField("s1", 1);
	out->addNewField("s2", 1);

	out->addNewField("center_to_new_center", 1);




    out->conservativeResize(total_number);

    size_t counter = 0;
    size_t keypoint_id = 0;

    for (calibration::KeyPoint::Ptr keypoint: getData())
    {
		for (calibration::Observation::Ptr data_holder: keypoint->observations)
        {
            out->getFieldByName("n_neighbors")(counter, 0) = data_holder->n_neighbors_intensity;
            out->getFieldByName("normal").row(counter) = keypoint->fitting_plane.getNormal();
            out->getFieldByName("lambdas").row(counter ) = keypoint->lambdas;
            out->getFieldByName("post_position").row(counter ) = keypoint->post_position;
            out->getFieldByName("original_position").row(counter ) = keypoint->original_position;
            out->getFieldByName("intensity")(counter, 0) = data_holder->intensity;
            out->getFieldByName("intensity_std")(counter, 0) = data_holder->intensity_std;
            out->getFieldByName("eigen_ratio")(counter, 0) = keypoint->eigen_ratio;
            out->getFieldByName("distance")(counter, 0) = data_holder->distance;
            out->getFieldByName("angle")(counter, 0) = data_holder->angle;
            out->getFieldByName("keypoint_id")(counter, 0) = (float) keypoint_id;
            out->getFieldByName("progressive_id")(counter, 0) = (float) counter;
            out->getFieldByName("source_id")(counter, 0) = (float) source_to_id.at(data_holder->cloud);
            out->getFieldByName("material")(counter, 0) = (float) data_holder->parent_keypoint->material_id;

			out->getFieldByName("s1")(counter, 0) = (float) keypoint->s1;
			out->getFieldByName("s2")(counter, 0) = (float) keypoint->s2;
			out->getFieldByName("center_to_new_center")(counter, 0) = (float) keypoint->center_to_new_center;

            counter++;
        }

        keypoint_id++;
    }

    return out;
}

std::vector<Observation::Ptr> DataHolder::getAllObservations() const
{
    std::vector<Observation::Ptr> out ;
    for (KeyPointPtr kpoint: getData())
    {
        for (Observation::Ptr pcd: kpoint->observations)
        {
            out.push_back(pcd);
        }
    }

    return out;

}

void spc::calibration::DataHolder::updateCloudIDsInObservations()
{

    DLOG(INFO) << "Updating cloud ids in observations";
    std::list<CloudDataSourceOnDiskPtr> clouds;
    for (Observation::Ptr ob: this->getAllObservations())
        clouds.push_back(ob->getCloud());

    clouds.sort();
    clouds.unique();

    LOG(INFO) << "Found " << clouds.size() << " unique clouds";

    std::map<CloudDataSourceOnDiskPtr, size_t> to_ids;

    size_t id = 0;
    for (CloudDataSourceOnDiskPtr c: clouds)
        to_ids[c] = id++;

#pragma parallel for private(to_ids)
    for (Observation::Ptr ob: this->getAllObservations())
        ob->cloud_id = to_ids[ob->getCloud()];
}

KeyPoint::Ptr DataHolder::newKeypoint(const Eigen::Vector3f &pos, const size_t material_id)
{
    KeyPoint::Ptr kp(new KeyPoint(pos, material_id));
    keypoints_.push_back(kp);
    return kp;
}

size_t DataHolder::getTotalNumberOfObservations() const
{
    size_t total_number=0;
    for (KeyPoint::Ptr keypoint: getData())
        total_number += keypoint->observations.size();

    return total_number;
}

void DataHolder::ereaseInvalidObservations(const bool consider_angle)
{
    for (KeyPoint::Ptr kp: keypoints_)
        kp->removeInvalidObservations(consider_angle);
}

DataHolder::Ptr DataHolder::getValidKeypoints() const
{
    DataHolder::Ptr out(new DataHolder());
    for (KeyPoint::Ptr kp: keypoints_)
    {
        if (kp->getNumberOfObservations() > 0)
            out->appendKeypoint(kp);
    }

    return out;
}

DataHolder::Ptr DataHolder::getKeypointsOnMaterial(const size_t mat_id) const
{
    DataHolder::Ptr out(new DataHolder());
    for (KeyPoint::Ptr kp: keypoints_)
    {
        if (kp->material_id == mat_id)
            out->appendKeypoint(kp);
    }
    return out;
}

Eigen::VectorXi DataHolder::getUniqueMaterials() const
{
    Eigen::VectorXi out;
    for (calibration::KeyPoint::Ptr keypoint: keypoints_)
    {
        out.push_back( keypoint->material_id );
    }

    return out.unique();
}

std::vector<CloudDataSourceOnDiskPtr> DataHolder::getDataSources() const
{
    std::vector<CloudDataSourceOnDiskPtr> sources;

    for (KeyPoint::Ptr kpoint : keypoints_)
    {
        for (Observation::Ptr per_cloud: kpoint->observations)
        {
            CloudDataSourceOnDiskPtr cloud = per_cloud->cloud;

            if (std::find(sources.begin(), sources.end(), cloud) == sources.end())
            {
                sources.push_back(cloud);
            }
        }
    }

    return sources;
}

//std::vector<KeyPointPtr> DataHolder::getData() const
//{
//    return keypoints_;
//}

void DataHolder::initFromCloud(const NewSpcPointCloudPtr pointset,
                               const std::string &material_field_name)
{
    if (!pointset->hasField(material_field_name))
    {
        LOG(WARNING) << "No material field "<< material_field_name <<" found in keypoints. All keypoints will be considered of the same material";
        for (int i = 0 ; i < pointset->getNumberOfPoints(); ++i)
            this->newKeypoint(pointset->getFieldByName("position").row(i), 0);
    }
    else
    {
        LOG(WARNING) << "Using the field " << material_field_name << " as field for materials. Remember that all integers are ok as material index but -1 "
                                                                     " will be considered as keypoints on unprecised and undifferentiated materials";
        for (int i = 0 ; i < pointset->getNumberOfPoints(); ++i)
            this->newKeypoint(pointset->getFieldByName("position").row(i), (int) pointset->getFieldByName(material_field_name).row(i)(0));
    }

}

}
}
#include <spc/core/spc_cereal.hpp>
SPC_CEREAL_REGISTER_TYPE(spc::calibration::DataHolder)
