#pragma once
#ifndef CALIBRATIONDATAHOLDER_H
#define CALIBRATIONDATAHOLDER_H

#include <spc/core/macros_ptr.h>
#include <spc/elements/ElementBase.h>

namespace spc
{

spcFwdDeclSharedPtr(NewSpcPointCloud)
spcFwdDeclSharedPtr(CloudDataSourceOnDisk)


namespace calibration {

spcFwdDeclSharedPtr(KeyPoint)
spcFwdDeclSharedPtr(Observation)
spcFwdDeclSharedPtr(DataHolder)



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

    std::vector<ObservationPtr> getAllObservations() const;

    void updateCloudIDsInObservations();


    KeyPointPtr newKeypoint(const Eigen::Vector3f &pos, const size_t material_id);

    void appendKeypoint(const KeyPointPtr kpoint)
    {
        keypoints_.push_back(kpoint);
    }

	//! this will create an empty keypoint for each point in pointset
	//! if the pointset contains the scalar field specified with material_field_name
	//! the material information will be copied in the newly created keypoints.
    void initFromCloud(const NewSpcPointCloudPtr pointset,
                       const std::string &material_field_name);

    size_t getTotalNumberOfObservations() const;



    void ereaseInvalidObservations(const bool consider_angle);

    //! returns a cleaned version of the CalibrationDataHolder
    //! no data copies are actually made. so only the pointers are copied into the new
    //! holder
    DataHolderPtr getValidKeypoints() const;

    DataHolderPtr getKeypointsOnMaterial(const size_t mat_id) const;

    //! this will report a vector of the ids of the materialis into the dataset
	//! notice that it will ALSO report uncategorized materials
    Eigen::VectorXi getUniqueMaterials() const;





    std::vector<KeyPointPtr> getData() const
    {
        return keypoints_;
    }

    std::vector<KeyPointPtr> &getData()
    {
        return keypoints_;
    }

    std::vector<CloudDataSourceOnDiskPtr> getDataSources() const;


    NewSpcPointCloudPtr asPointCloud() const;
protected:
    std::vector<KeyPointPtr> keypoints_;

private:
    friend class cereal::access;

    template <class Archive> void serialize(Archive &ar, const std::uint32_t version)
    {
        ar(cereal::base_class<spc::ElementBase>(this),
           CEREAL_NVP(keypoints_)
           );
    }
};


}
}

#endif // CALIBRATIONDATAHOLDER_H
