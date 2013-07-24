#ifndef SIMPLE_INTENSITY_CALIBRATION_MODEL_H
#define SIMPLE_INTENSITY_CALIBRATION_MODEL_H

#include <pcl/point_cloud.h>

template <typename PointInT>
class SimpleIntensityCalibrationModel
{

    enum modeltypeT {KS, POLYNOMIAL};

    typedef typename pcl::PointCloud<PointInT> CloudT;
    typedef typename CloudT::Ptr CloudPtrT;
public:
    SimpleIntensityCalibrationModel()
    {
        modeltype_ = KS;
    }

    void setModelType(const modeltypeT type)
    {
        modeltype_ = type;
    }

    void setInputCloud(const CloudPtrT cloud)
    {
        in_cloud_ = cloud;
    }

    void setKSStep(const float step)
    {
        ks_step_ = step;
    }

    void setKSBandwidth(const float bandwidth)
    {
        ks_bandwidth_ = bandwidth;
    }



private:
    CloudPtrT in_cloud_;
    modeltypeT modeltype_;

    float ks_step_, ks_bandwidth_;


};

#endif // SIMPLE_INTENSITY_CALIBRATION_MODEL_H
