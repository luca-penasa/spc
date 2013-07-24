#ifndef INTENSITY_DROP_CORRECTION_H
#define INTENSITY_DROP_CORRECTION_H

#include <pcl/point_cloud.h>
#include <pcl/common/io.h>


class DropModel
{
    typedef typename boost::shared_ptr<DropModel> Ptr;
public:
    virtual float getCorrectionAt(float x) = 0;
};

class StairstepDropModel: public DropModel
{
public:
    StairstepDropModel() {}

    float drop_position_;
    float drop_magnitude_;

    float getCorrectionAt(float x)
    {
        if (x<=drop_position_)
            return 0.0f;
        else
            return drop_magnitude_;
    }
};

template <typename PointInT>
class IntensityDropCorrection
{
    typedef typename pcl::PointCloud<PointInT> CloudT;
    typedef typename CloudT::Ptr CloudPtrT;
public:
    IntensityDropCorrection();

    void setInputCloud(const CloudPtrT cloud)
    {
        in_cloud_ = cloud;
    }

    void setDropModel(DropModel::Ptr model)
    {
        model_ = model;
    }

    void compute()
    {
        out_cloud_ = CloudT::Ptr(new CloudT);



        for (int i = 0 ; i < in_cloud_->size(); ++i)
        {
            PointInT point = in_cloud_->at(i);
            float position = point.id;
            float intens = point.intensity;
            float correction = model_->getCorrectionAt(position);

//            float new_value = Intensity + correction;

            out_cloud_->push_back(point);
        }
    }


    auto getOutputCloud() -> CloudPtrT
    {
        return out_cloud_;
    }

private:
    CloudPtrT in_cloud_, out_cloud_;
    DropModel::Ptr model_;
};

#endif // INTENSITY_DROP_CORRECTION_H
