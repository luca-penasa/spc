#ifndef CALIBRATE_MODEL_H
#define CALIBRATE_MODEL_H

#include <spc/experimental/ICalModelDiscretePoints.h>
#include <spc/elements/PointCloudPcl.h>
#include <spc/methods/KernelSmoothing2.h>

namespace spc
{

class CalibrateDiscretePointsModel : public ElementBase
{
public:
    CalibrateDiscretePointsModel() : model_(new ModelDiscretePoints)
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

    void setInputCloud(PointCloudBase::Ptr cloud)
    {
        cloud_ = cloud;
    }

    int compute() const
    {
        if (!cloud_) {
            pcl::console::print_error(
                "[Error in %s] you should set an input cluod!",
                this->getType()->getClassName().c_str());
            return -1;
        }

        if (!cloud_->hasField(distace_field_)
            | !cloud_->hasField(intensity_field_)) {
            pcl::console::print_error("[Error in %s] Cannot find the distance "
                                      "and/or the distance fields you "
                                      "requested!",
                                      this->getType()->getClassName().c_str());
            return -1;
        }

        std::vector<float> dist = cloud_->getField(distace_field_);
        std::vector<float> intens = cloud_->getField(intensity_field_);

        TimeSeriesSparse::Ptr serie(new TimeSeriesSparse(dist, intens));

//        KernelSmoothing2 ks;

//        ks.setBandwidth(bandwidth_);
//        ks.setStep(s_step_);
//        ks.setInputSeries(serie);
//        ks.compute();

//        TimeSeriesEquallySpaced::Ptr out = ks.getOutputSeries();

//        model_->setDiscretePoints(out);
    }

    ModelDiscretePoints::Ptr getModel() const
    {
        return model_;
    }

protected:
    ModelDiscretePoints::Ptr model_;
    PointCloudBase::Ptr cloud_;

    std::string distace_field_;
    std::string intensity_field_;

    float s_step_;
    float bandwidth_;
};

} // end nspace
#endif // CALIBRATE_MODEL_H
