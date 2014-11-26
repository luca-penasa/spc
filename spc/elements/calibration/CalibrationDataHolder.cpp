#include "CalibrationDataHolder.h"

namespace spc
{
namespace calibration
{

DtiClassType CalibrationDataHolder::Type ("CalibrationDataHolder", &ElementBase::Type);

CalibrationDataHolder::CalibrationDataHolder()
{
}

NewSpcPointCloud::Ptr CalibrationDataHolder::asPointCloud() const
{
    std::vector<CloudDataSourceOnDisk::Ptr> sources = getDataSources();
    std::map<CloudDataSourceOnDisk::Ptr, size_t> source_to_id;

    size_t id = 0;
    for (CloudDataSourceOnDisk::Ptr s: sources)
        source_to_id[s] = id++;



    NewSpcPointCloud::Ptr out(new NewSpcPointCloud);

    size_t total_number= getTotalNumberOfEntries();

    DLOG(INFO) << "we have a total number of entries: " << total_number;

    out->addNewField("distance", 1);
    out->addNewField("angle", 1);
    out->addNewField("intensity", 1);


    out->addNewField("material", 1);
    out->addNewField("keypoint_id", 1);
    out->addNewField("source_id", 1);
    out->addNewField("progressive_id", 1);



    out->addNewField("normal", 3);
    out->addNewField("position", 3);

    out->addNewField("lambdas", 3);
    out->addNewField("n_neighbors", 1);
    out->addNewField("intensity_std", 1);
    out->addNewField("eigen_ratio", 1);




    out->conservativeResize(total_number);

    size_t counter = 0;
    size_t keypoint_id = 0;
    for (calibration::CalibrationKeyPoint::Ptr keypoint: getData())
    {
        for (calibration::PerCloudCalibrationData::Ptr data_holder: keypoint->per_cloud_data)
        {
            out->getFieldByName("n_neighbors")(counter, 0) = data_holder->n_neighbors_intensity;
            out->getFieldByName("normal").row(counter) = keypoint->fitting_plane.getNormal();
            out->getFieldByName("lambdas").row(counter ) = keypoint->lambdas;
            out->getFieldByName("position").row(counter ) = keypoint->post_position;
            out->getFieldByName("intensity")(counter, 0) = data_holder->intensity;
            out->getFieldByName("intensity_std")(counter, 0) = data_holder->intensity_std;
            out->getFieldByName("eigen_ratio")(counter, 0) = keypoint->eigen_ratio;
            out->getFieldByName("distance")(counter, 0) = data_holder->distance;
            out->getFieldByName("angle")(counter, 0) = data_holder->angle;
            out->getFieldByName("keypoint_id")(counter, 0) = (float) keypoint_id;
            out->getFieldByName("progressive_id")(counter, 0) = (float) counter;
            out->getFieldByName("source_id")(counter, 0) = (float) source_to_id.at(data_holder->cloud);
            out->getFieldByName("material")(counter, 0) = (float) data_holder->parent_keypoint->material_id;

            counter++;
        }

        keypoint_id++;
    }

    return out;
}

}
}
#include <spc/core/spc_cereal.hpp>
SPC_CEREAL_REGISTER_TYPE(spc::calibration::CalibrationDataHolder)
