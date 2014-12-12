#ifndef PERCLOUDDATA_H
#define PERCLOUDDATA_H

#include <spc/elements/ElementBase.h>
#include <spc/elements/CloudDataSourceOnDisk.h>
namespace spc
{
namespace calibration
{

class CalibrationKeyPoint;
typedef spcSharedPtrMacro<CalibrationKeyPoint> CalibrationKeyPointPtr;

class Observation: public std::enable_shared_from_this<Observation>
{
public:
	spcTypedefSharedPtrs(Observation)

    //! so that cereal does not complain. should not be used
	Observation()
    {

    }

	Observation(CloudDataSourceOnDisk::Ptr ref_cloud, CalibrationKeyPointPtr parent);
    CalibrationKeyPointPtr getParent() const
    {
        return parent_keypoint;
    }

    CloudDataSourceOnDisk::Ptr getCloud() const
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
        if (consider_angle = true)
            return hasValidAngle() && hasValidDistance() && hasValidIntensity();
        else
            return hasValidDistance() && hasValidIntensity();
    }

    //////////////////////// THE DATA
    CloudDataSourceOnDisk::Ptr cloud;

    CalibrationKeyPointPtr parent_keypoint;

    size_t n_neighbors_intensity = 0;
    float distance = spcNANMacro;
    float angle= spcNANMacro;
    float intensity= spcNANMacro;
    float intensity_std= spcNANMacro;

	float intensity_corrected = spcNANMacro; // we don't save this value											 // when serializing

    Eigen::Vector3f sensor_position;

    NewSpcPointCloud extract_for_normal_;

private:
    friend class cereal::access;

    template <class Archive> void serialize(Archive &ar)
    {
        ar(CEREAL_NVP(cloud),
           CEREAL_NVP(parent_keypoint),
           CEREAL_NVP(n_neighbors_intensity),
           CEREAL_NVP(distance),
           CEREAL_NVP(angle),
           CEREAL_NVP(intensity),
           CEREAL_NVP(intensity_std),
           CEREAL_NVP(sensor_position),
           CEREAL_NVP(extract_for_normal_)
           );
    }



};
}
}




#endif
