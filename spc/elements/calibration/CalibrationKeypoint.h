
#ifndef CALIBRATION_KEYPOINT_H
#define CALIBRATION_KEYPOINT_H
#include <spc/elements/ElementBase.h>
#include <spc/elements/calibration/CalibrationObservation.h>
#include <spc/elements/Plane.h>
namespace spc
{
namespace calibration
{

class CalibrationKeyPoint: public std::enable_shared_from_this<CalibrationKeyPoint>
{
public:
    spcTypedefSharedPtrs(CalibrationKeyPoint)

    //! just for cereal to not complain
    CalibrationKeyPoint()
    {
        lambdas.fill(spcNANMacro);
        material_id = std::numeric_limits<int>::quiet_NaN();

        //! \todo add nan init for other members
    }

    CalibrationKeyPoint(const CalibrationKeyPoint & other)
    {

    }

    CalibrationKeyPoint(const Eigen::Vector3f &pos, const size_t mat_id);

    void removeInvalidEntries(const bool &consider_angle)
    {

		std::vector<Observation::Ptr> good ;
		for (Observation::Ptr d: per_cloud_data)
        {
            if (d->isValid(consider_angle))
                good.push_back(d);
        }

        per_cloud_data = good;


    }

    size_t getNumberOfEntries() const
    {
        return per_cloud_data.size();
    }

	Observation::Ptr newPerCloudData(CloudDataSourceOnDisk::Ptr cloud);

    Plane fitting_plane;

    Eigen::Vector3f original_position;
    Eigen::Vector3f post_position;
    float eigen_ratio;
    Eigen::Vector3f lambdas;
	int material_id; //! < a material id of -1 will be considered as unknwon material

	std::vector<Observation::Ptr> per_cloud_data;

    NewSpcPointCloud cumulative_set;

	float intensity_expected = spcNANMacro;

private:
    friend class cereal::access;

    template <class Archive> void serialize(Archive &ar)
    {
        ar(
           CEREAL_NVP(fitting_plane),
           CEREAL_NVP(original_position),
           CEREAL_NVP(post_position),
           CEREAL_NVP(eigen_ratio),
           CEREAL_NVP(lambdas),
           CEREAL_NVP(per_cloud_data),
           CEREAL_NVP(cumulative_set),
           CEREAL_NVP(material_id));
    }
};


}//end cal

}// end spc


#endif
