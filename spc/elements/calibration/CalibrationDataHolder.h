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

    void initFromCloud(const NewSpcPointCloud::ConstPtr pointset, std::string material_field_name)
    {
        if (!pointset->hasField(material_field_name))
        {
            LOG(WARNING) << "No material field "<< material_field_name <<" found in keypoints. All keypoints will be considered of the same material";
            for (int i = 0 ; i < pointset->getNumberOfPoints(); ++i)
                this->newKeypoint(pointset->getFieldByName("position").row(i), 0);
        }
        else
        {
            LOG(INFO) << "Using the field " << material_field_name << " as field for materials";
            for (int i = 0 ; i < pointset->getNumberOfPoints(); ++i)
                this->newKeypoint(pointset->getFieldByName("position").row(i), (size_t) pointset->getFieldByName(material_field_name).row(i)(0));
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
