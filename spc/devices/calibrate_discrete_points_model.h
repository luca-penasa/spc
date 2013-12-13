#ifndef CALIBRATE_MODEL_H
#define CALIBRATE_MODEL_H

#include <spc/devices/discrete_points_calibration_model.h>
#include <spc/elements/generic_cloud.h>
#include <spc/methods/kernelsmoothing2.h>

namespace spc
{


class CalibrateDiscretePointsModel: public spcElementBase
{
public:
    CalibrateDiscretePointsModel(): model_(new DiscretePointsCalibrationModel)
    {

    }

    void setDistanceField(const std::string d_field)
    {
        distace_field_ = d_field;
    }

    void setIntensityField(const std::string i_field)
    {
        intensity_field_ = i_field;
    }

    void setSamplingStep(const float step)
    {
        s_step_ = step;
    }

    void setBandwidth(const float band)
    {
        bandwidth_ = band;
    }

    void setInputCloud(spcGenericCloud::Ptr cloud)
    {
        cloud_ = cloud;
    }

    int compute() const
    {
        if (!cloud_)
        {
            pcl::console::print_error("[Error in %s] you should set an input cluod!", getClassName().c_str());
            return -1;
        }

        if (!cloud_->hasField(distace_field_) | !cloud_->hasField(intensity_field_))
        {
            pcl::console::print_error("[Error in %s] Cannot find the distance and/or the distance fields you requested!", getClassName().c_str());
            return -1;
        }


        std::vector<float> dist = cloud_->getField(distace_field_);
        std::vector<float> intens = cloud_->getField(intensity_field_);

        SparseTimeSeries<float>::Ptr serie(new SparseTimeSeries<float>(dist, intens));

        KernelSmoothing2<float> ks;

        ks.setBandwidth(bandwidth_);
        ks.setStep(s_step_);
        ks.setInputSeries(serie);
        ks.compute();

        EquallySpacedTimeSeries<float>::Ptr out = ks.getOutputSeries();

        model_->setDiscretePoints(out);

    }


    DiscretePointsCalibrationModel::Ptr getModel() const
    {
        return model_;
    }


protected:
    DiscretePointsCalibrationModel::Ptr model_;
    spcGenericCloud::Ptr cloud_;

    std::string distace_field_;
    std::string intensity_field_;


    float s_step_;
    float bandwidth_;





};



}//end nspace
#endif // CALIBRATE_MODEL_H
