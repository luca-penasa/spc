#pragma once
#ifndef PERCLOUDDATA_H
#define PERCLOUDDATA_H


#include <spc/elements/NewSpcPointCloud.h>


namespace spc
{
spcFwdDeclSharedPtr(CloudDataSourceOnDisk)

namespace calibration
{

//class KeyPoint;
//typedef spcSharedPtrMacro<KeyPoint> CalibrationKeyPointPtr;

spcFwdDeclSharedPtr(KeyPoint);
//spcFwdDeclSharedPtr(KeyPoint);

class Observation: public std::enable_shared_from_this<Observation>
{
public:
	spcTypedefSharedPtrs(Observation)

    //! so that cereal does not complain. should not be used
	Observation()
    {

    }

    Observation(CloudDataSourceOnDiskPtr ref_cloud,KeyPointPtr parent);
    KeyPointPtr getParent() const
    {
        return parent_keypoint;
    }

    CloudDataSourceOnDiskPtr getCloud() const
    {
        return cloud;
    }


    bool hasValidDistance() const
    {
        return std::isfinite(distance);
    }

    bool hasValidAngle() const
    {
        return std::isfinite(angle);
    }

    bool hasValidIntensity() const
    {
        return std::isfinite(intensity);
    }

	Eigen::VectorXf getAsEigenPoint(const bool also_angle = true) const
	{
		Eigen::VectorXf v(1);
		v(0) = distance;

		if (also_angle)
		{
			v.conservativeResize(2);
			v(1) = angle;
		}

		return v;
	}



    //! consider_angle says if also a finite angle must be present to consider the data "good"
    bool isValid(bool consider_angle) const
    {
		if (consider_angle == true)
            return hasValidAngle() && hasValidDistance() && hasValidIntensity();
        else
            return hasValidDistance() && hasValidIntensity();
    }

    //////////////////////// THE DATA
    CloudDataSourceOnDiskPtr cloud;

    KeyPointPtr parent_keypoint;

    size_t n_neighbors_intensity = 0;
    float distance = spcNANMacro;
    float angle= spcNANMacro;
    float intensity= spcNANMacro;
    float intensity_std= spcNANMacro;

    float intensity_corrected = spcNANMacro; // we don't save this value

    size_t cloud_id = spcNANMacros;  // we do not save this,
                                    // but the data holder can promply recompute it
    // when serializing

//    Eigen::Vector3f sensor_position;

    spc::NewSpcPointCloud extract_for_normal_;

private:
    friend class cereal::access;

    template <class Archive> void serialize(Archive &ar, const std::uint32_t version)
    {
        ar(CEREAL_NVP(cloud),
           CEREAL_NVP(parent_keypoint),
           CEREAL_NVP(n_neighbors_intensity),
           CEREAL_NVP(distance),
           CEREAL_NVP(angle),
           CEREAL_NVP(intensity),
           CEREAL_NVP(intensity_std),
//           CEREAL_NVP(sensor_position),
           CEREAL_NVP(extract_for_normal_)
           );
    }



};
}
}




#endif
