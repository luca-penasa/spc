#ifndef CALIBRATIONDATAHOLDER_H
#define CALIBRATIONDATAHOLDER_H
#include <spc/elements/calibration/CalibrationKeypoint.h>
#include <spc/elements/NewSpcPointCloud.h>
namespace spc
{
namespace calibration {


class CalibrationDataHolder: public ElementBase
{
public:

    SPC_ELEMENT(CalibrationDataHolder)
    EXPOSE_TYPE

    CalibrationDataHolder();


    CalibrationKeyPoint::Ptr newKeypoint(const Eigen::Vector3f &pos, size_t material_id)
    {
        CalibrationKeyPoint::Ptr kp(new CalibrationKeyPoint(pos, material_id));
        keypoints_.push_back(kp);
        return kp;
    }

    void appendKeypoint(CalibrationKeyPoint::Ptr kpoint)
    {
        keypoints_.push_back(kpoint);
    }

    void initFromCloud(const NewSpcPointCloud::ConstPtr pointset, const std::string material_field_name)
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

    size_t getTotalNumberOfEntries() const
    {
        size_t total_number=0;
        for (calibration::CalibrationKeyPoint::Ptr keypoint: getData())
        {
                total_number += keypoint->per_cloud_data.size();
        }

        return total_number;
    }

    Eigen::Matrix<float, -1, 1> getDistanceForKPointsOnMaterial(const size_t mat_id) const
    {

    }

    void ereaseInvalidPerCloudEntries(const bool consider_angle)
    {
        for (CalibrationKeyPoint::Ptr kp: keypoints_)
            kp->removeInvalidEntries(consider_angle);
    }

    //! returns a cleaned version of the CalibrationDataHolder
    //! no data copies are actually made. so only the pointers are copied into the new
    //! holder
    CalibrationDataHolder::Ptr getValidKeypoints() const
    {
        CalibrationDataHolder::Ptr out(new CalibrationDataHolder());
        for (CalibrationKeyPoint::Ptr kp: keypoints_)
        {
            if (kp->getNumberOfEntries() > 0)
                out->appendKeypoint(kp);
        }

        return out;
    }

    CalibrationDataHolder::Ptr getKeypointsOnMaterial(const size_t mat_id) const
    {
        CalibrationDataHolder::Ptr out(new CalibrationDataHolder());
        for (CalibrationKeyPoint::Ptr kp: keypoints_)
        {
           if (kp->material_id == mat_id)
               out->appendKeypoint(kp);
        }
        return out;
    }

    //! this will report a vector of the ids of the materialis into the dataset
    //! notice that it will not report uncategorized materials
    Eigen::VectorXi getDefinedMaterials() const
    {
        Eigen::VectorXi out;
        for (calibration::CalibrationKeyPoint::Ptr keypoint: keypoints_)
        {
           out.push_back( keypoint->material_id );
        }

        return out.unique();
    }





    std::vector<CalibrationKeyPoint::Ptr> getData() const
    {
        return keypoints_;
    }

    std::vector<CalibrationKeyPoint::Ptr> &getData()
    {
        return keypoints_;
    }

    std::vector<CloudDataSourceOnDisk::Ptr> getDataSources() const
    {
        std::vector<CloudDataSourceOnDisk::Ptr> sources;

        for (CalibrationKeyPoint::Ptr kpoint : keypoints_)
        {
            for (PerCloudCalibrationData::Ptr per_cloud: kpoint->per_cloud_data)
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
    std::vector<CalibrationKeyPoint::Ptr> keypoints_;

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
