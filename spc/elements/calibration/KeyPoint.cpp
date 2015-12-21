#include "KeyPoint.h"
#include <spc/core/cerea_requested_types.hpp>
#include <spc/elements/calibration/Observation.h>

namespace spc
{
namespace calibration
{


KeyPoint::KeyPoint(const Vector3f &pos, const size_t mat_id)
{
//    LOG(INFO) << "creating a new calibration keypoint at "<< pos.transpose();
    original_position = pos;
    material_id = mat_id;
    cumulative_set.addNewField("position", 3);
    lambdas.fill(spcNANMacro);

    material_id = mat_id;

}

void KeyPoint::removeInvalidObservations(const bool &consider_angle)
{

    std::vector<Observation::Ptr> good ;
    for (Observation::Ptr d: observations)
    {
        if (d->isValid(consider_angle))
            good.push_back(d);
    }

    observations = good;


}

//Observation::Ptr
//KeyPoint::newObservationOnCloud(CloudDataSourceOnDisk::Ptr cloud)
//{
//    Observation::Ptr cdata (new Observation(cloud, shared_from_this()));
//	// we want to add data only if valid
//	//observations.push_back(cdata);
//    return cdata;
//}

// it is a root

}
}
