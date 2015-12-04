#include "DataHolder.h"

namespace spc
{
namespace calibration
{

DtiClassType DataHolder::Type ("DataHolder", &ElementBase::Type);

DataHolder::DataHolder()
{
}

NewSpcPointCloud::Ptr DataHolder::asPointCloud() const
{
    std::vector<CloudDataSourceOnDisk::Ptr> sources = getDataSources();
    std::map<CloudDataSourceOnDisk::Ptr, size_t> source_to_id;

    size_t id = 0;
    for (CloudDataSourceOnDisk::Ptr s: sources)
        source_to_id[s] = id++;



    NewSpcPointCloud::Ptr out(new NewSpcPointCloud);

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

void spc::calibration::DataHolder::updateCloudIDsInObservations()
{

    DLOG(INFO) << "Updating cloud ids in observations";
    std::list<CloudDataSourceOnDisk::Ptr> clouds;
    for (Observation::Ptr ob: this->getAllObservations())
        clouds.push_back(ob->getCloud());

    clouds.sort();
    clouds.unique();

    LOG(INFO) << "Found " << clouds.size() << " unique clouds";

    std::map<CloudDataSourceOnDisk::Ptr, size_t> to_ids;

    size_t id = 0;
    for (CloudDataSourceOnDisk::Ptr c: clouds)
        to_ids[c] = id++;

#pragma parallel for private(to_ids)
    for (Observation::Ptr ob: this->getAllObservations())
        ob->cloud_id = to_ids[ob->getCloud()];
}

}
}
#include <spc/core/spc_cereal.hpp>
SPC_CEREAL_REGISTER_TYPE(spc::calibration::DataHolder)
