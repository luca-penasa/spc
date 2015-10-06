#ifndef CALIBRATIONDATAHOLDER_H
#define CALIBRATIONDATAHOLDER_H
#include <spc/elements/calibration/KeyPoint.h>
#include <spc/elements/NewSpcPointCloud.h>

namespace spc
{
namespace calibration {


class DataHolder: public ElementBase
{
public:

	SPC_ELEMENT(DataHolder)
    EXPOSE_TYPE

	DataHolder();

	DataHolder(const DataHolder & other)
	{
		keypoints_ = other.keypoints_;
	}

	std::vector<Observation::Ptr> getAllObservations() const
	{
		std::vector<Observation::Ptr> out ;
		for (CalibrationKeyPointPtr kpoint: getData())
		{
			for (Observation::Ptr pcd: kpoint->observations)
			{
				out.push_back(pcd);
			}
		}

		return out;

	}


	KeyPoint::Ptr newKeypoint(const Eigen::Vector3f &pos, const size_t material_id)
    {
		KeyPoint::Ptr kp(new KeyPoint(pos, material_id));
        keypoints_.push_back(kp);
        return kp;
    }

	void appendKeypoint(const KeyPoint::Ptr kpoint)
    {
        keypoints_.push_back(kpoint);
    }

	//! this will create an empty keypoint for each point in pointset
	//! if the pointset contains the scalar field specified with material_field_name
	//! the material information will be copied in the newly created keypoints.
	void initFromCloud(const NewSpcPointCloud::Ptr pointset, const std::string &material_field_name)
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

	size_t getTotalNumberOfObservations() const
    {
        size_t total_number=0;
		for (KeyPoint::Ptr keypoint: getData())
				total_number += keypoint->observations.size();

        return total_number;
    }



	void ereaseInvalidObservations(const bool consider_angle)
    {
		for (KeyPoint::Ptr kp: keypoints_)
			kp->removeInvalidObservations(consider_angle);
    }

    //! returns a cleaned version of the CalibrationDataHolder
    //! no data copies are actually made. so only the pointers are copied into the new
    //! holder
	DataHolder::Ptr getValidKeypoints() const
    {
		DataHolder::Ptr out(new DataHolder());
		for (KeyPoint::Ptr kp: keypoints_)
        {
			if (kp->getNumberOfObservations() > 0)
                out->appendKeypoint(kp);
        }

        return out;
    }

	DataHolder::Ptr getKeypointsOnMaterial(const size_t mat_id) const
    {
		DataHolder::Ptr out(new DataHolder());
		for (KeyPoint::Ptr kp: keypoints_)
        {
           if (kp->material_id == mat_id)
               out->appendKeypoint(kp);
        }
        return out;
    }

    //! this will report a vector of the ids of the materialis into the dataset
	//! notice that it will ALSO report uncategorized materials
	Eigen::VectorXi getUniqueMaterials() const
    {
        Eigen::VectorXi out;
		for (calibration::KeyPoint::Ptr keypoint: keypoints_)
        {
           out.push_back( keypoint->material_id );
        }

        return out.unique();
    }





	std::vector<KeyPoint::Ptr> getData() const
    {
        return keypoints_;
    }

	std::vector<KeyPoint::Ptr> &getData()
    {
        return keypoints_;
    }

    std::vector<CloudDataSourceOnDisk::Ptr> getDataSources() const
    {
        std::vector<CloudDataSourceOnDisk::Ptr> sources;

		for (KeyPoint::Ptr kpoint : keypoints_)
        {
			for (Observation::Ptr per_cloud: kpoint->observations)
            {
                CloudDataSourceOnDisk::Ptr cloud = per_cloud->cloud;

                if (std::find(sources.begin(), sources.end(), cloud) == sources.end())
                {
                    sources.push_back(cloud);
                }
            }
        }

        return sources;
    }


    NewSpcPointCloud::Ptr asPointCloud() const;
protected:
	std::vector<KeyPoint::Ptr> keypoints_;

private:
    friend class cereal::access;

    template <class Archive> void serialize(Archive &ar)
    {
        ar(cereal::base_class<spc::ElementBase>(this),
           CEREAL_NVP(keypoints_)
           );
    }
};


}
}

#endif // CALIBRATIONDATAHOLDER_H
