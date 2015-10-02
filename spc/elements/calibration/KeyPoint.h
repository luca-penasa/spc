
#ifndef CALIBRATION_KEYPOINT_H
#define CALIBRATION_KEYPOINT_H
#include <spc/elements/ElementBase.h>
#include <spc/elements/calibration/Observation.h>
#include <spc/elements/Plane.h>
namespace spc
{
namespace calibration
{

class KeyPoint: public std::enable_shared_from_this<KeyPoint>
{
public:
	spcTypedefSharedPtrs(KeyPoint)

	KeyPoint()
    {
        lambdas.fill(spcNANMacro);
		original_position.fill(spcNANMacro);
		post_position.fill(spcNANMacro);
        material_id = std::numeric_limits<int>::quiet_NaN();

        //! \todo add nan init for other members
    }

	KeyPoint(const KeyPoint & other)
    {

    }

	KeyPoint(const Eigen::Vector3f &pos, const size_t mat_id);

	void removeInvalidObservations(const bool &consider_angle)
    {

		std::vector<Observation::Ptr> good ;
		for (Observation::Ptr d: observations)
        {
            if (d->isValid(consider_angle))
                good.push_back(d);
        }

		observations = good;


    }

	size_t getNumberOfObservations() const
    {
		return observations.size();
    }

//	Observation::Ptr newObservationOnCloud(CloudDataSourceOnDisk::Ptr cloud);

    Plane fitting_plane;

    Eigen::Vector3f original_position;
    Eigen::Vector3f post_position;
    float eigen_ratio;
    Eigen::Vector3f lambdas;
	int material_id; //! < a material id of -1 will be considered as unknwon material

	// these two are indexes derived from the eigenvalues
	float s1 = spcNANMacro;
	float s2 = spcNANMacro;

	float center_to_new_center = spcNANMacro;

	std::vector<Observation::Ptr> observations;

    NewSpcPointCloud cumulative_set;

	float intensity_expected = spcNANMacro;

private:
    friend class cereal::access;

	template <class Archive> void serialize(Archive &ar, std::uint32_t const version)
    {
        ar(
           CEREAL_NVP(fitting_plane),
           CEREAL_NVP(original_position),
           CEREAL_NVP(post_position),
           CEREAL_NVP(eigen_ratio),
           CEREAL_NVP(lambdas),
		   CEREAL_NVP(observations),
           CEREAL_NVP(cumulative_set),
           CEREAL_NVP(material_id));

		if (version >=2)
		{
			CEREAL_NVP(s1);
			CEREAL_NVP(s2);
			CEREAL_NVP(center_to_new_center);
		}
    }
};

}//end cal





}// end spc

CEREAL_CLASS_VERSION( spc::calibration::KeyPoint, 2 )
#endif
