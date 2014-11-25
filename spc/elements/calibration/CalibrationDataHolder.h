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



    CalibrationKeyPoint::Ptr newKeypoint(const Eigen::Vector3f &pos)
    {
        CalibrationKeyPoint::Ptr kp(new CalibrationKeyPoint(pos));
        getData().push_back(kp);
        return kp;
    }

    void initFromCloud(const NewSpcPointCloud::ConstPtr pointset)
    {
        for (int i = 0 ; i < pointset->getNumberOfPoints(); ++i)
            this->newKeypoint(pointset->getFieldByName("position").row(i));
    }

    std::vector<CalibrationKeyPoint::Ptr> getData() const
    {
        return keypoints_;
    }

    std::vector<CalibrationKeyPoint::Ptr> &getData()
    {
        return keypoints_;
    }


     NewSpcPointCloud::Ptr asPointCloud() const
     {
         NewSpcPointCloud::Ptr out(new NewSpcPointCloud);

         size_t total_number=0;
         for (calibration::CalibrationKeyPoint::Ptr keypoint: getData())
         {
             for (calibration::PerCloudCalibrationData::Ptr data_holder: keypoint->per_cloud_data)
             {
                 total_number += keypoint->per_cloud_data.size();
             }
         }


         out->addNewField("n_neighbors", 1);
         out->addNewField("normal", 3);
         out->addNewField("lambdas", 3);
         out->addNewField("position", 3);
         out->addNewField("core_id", 1);
         //    out->addNewField("cloud_id", 1);
         out->addNewField("distance", 1);
         out->addNewField("intensity", 1);
         out->addNewField("angle", 1);
         out->addNewField("intensity_std", 1);
         out->addNewField("eigen_ratio", 1);



         out->conservativeResize(total_number);

         size_t counter = 0;
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

                 counter++;
             }
         }

         return out;
     }
protected:
     std::vector<CalibrationKeyPoint::Ptr> keypoints_;

private:
    friend class cereal::access;

    template <class Archive> void serialize(Archive &ar)
    {
        ar(cereal::base_class<spc::ElementBase>(this)
           ,
           CEREAL_NVP(keypoints_)
           );
    }
};


}
}

#endif // CALIBRATIONDATAHOLDER_H
