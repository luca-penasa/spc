#ifndef TIME_SERIES_GENERATOR_H
#define TIME_SERIES_GENERATOR_H

#include <spc/core/common.h>

#include <spc/elements/TimeSeriesEquallySpaced.h>
//#include <spc/io/pointcloud2_reader.h>
#include <spc/elements/PointCloudBase.h>

#include <spc/elements/VariableScalarFieldBase.h>

#include <spc/methods/KernelSmoothing2.h>

#include <spc/methods/DynamicScalarFieldEvaluator.h>

#include <spc/elements/SelectionRubberband.h>
#include <spc/methods/SelectionExtractor.h>

namespace spc
{

///
/// \brief The TimeSeriesGenerator class estimates time series starting from two
/// fields of a cloud
/// the results is always a TimeSeries equally spaced. If you want to evaluate
/// non-equally spaced time series
/// have a look directly to the KernelSmoothing method that permits to do that.
///
/// You can also chose to set as input a stratigraphic model + a cloud as a
/// generic cloud
///
class TimeSeriesGenerator
{
public:
    typedef float ScalarT;
    typedef spc::TimeSeriesEquallySpaced OutSeriesT;
    typedef spcSharedPtrMacro<OutSeriesT> OutSeriesPtrT;

    typedef Eigen::Matrix<ScalarT, -1, 1> VectorT;

    ///
    /// \brief TimeSeriesGenerator def constructor
    ///
    TimeSeriesGenerator() : min_x_(0.0), max_x_(0.0)
    {
    }

    ///! set the stratigraphic model to use for interpretation
    /// you can set a field with setXFieldName if if the stratigraphic positions
    /// were pre-computed
    void setStratigraphicModel(VariableScalarFieldBase::Ptr model)
    {
        model_ = model;
    }

    ///
    /// \brief setInputCloud the input cloud
    /// \param cloud
    ///
    void setInputCloud(const PointCloudBase::Ptr cloud)
    {
        in_cloud_ = cloud;
    }

    ///
    /// \brief setBandwidth
    /// \param bandwidth is the bandwidth used in the KernelSmoothing method
    ///
    void setBandwidth(ScalarT bandwidth)
    {
        bandwidth_ = bandwidth;
    }

    ///
    /// \brief setSamplingStep
    /// \param sampling_step is the step distance at which evaluate the time
    /// series values
    ///
    void setSamplingStep(ScalarT sampling_step)
    {
        sampling_step_ = sampling_step;
    }

    ///
    /// \brief setXFieldName
    /// \param field_name the field used as independen variable - x -
    /// if you prefer you can use a model and estimate the X field on the fly
    /// notice that if a model is set it is always used
    ///
    void setXFieldName(std::string field_name)
    {
        x_field_name_ = field_name;
    }

    void setMinMaxLog(const float min, const float max)
    {
        min_x_ = min;
        max_x_ = max;
    }

    ///
    /// \brief setLogFieldName
    /// \param field_name the name of the field to be logged
    /// \note this var is mandatory
    ///
    void setYFieldName(const std::string &field_name)
    {
        y_field_name_ = field_name;
    }

    ///
    /// \brief setIndices use only these points for estimatig the time series!
    /// if not given all the points will be used!
    /// \param indices
    ///
    void setIndices(const std::vector<size_t> &indices)
    {
        indices_ = indices;
    }

    ///
    /// \brief getOutputSeries
    /// \return the resulting series
    ///
    inline OutSeriesPtrT getOutputSeries() const
    {
        if (!out_series_)
            LOG(WARNING) << "out series is void. Null pointer returned!";

        return out_series_;
    }

    void setSelection(spc::SelectionRubberband::Ptr sel)
    {
        selection_ = sel;
    }

    SelectionRubberband::Ptr getSelection() const
    {
        return selection_;
    }

    ///
    /// \brief compute make computations
    /// \return status -1 if something wrong!
    ///
    int compute();

    ///
    /// \brief setFixedMinMax permits to create a bunch of ts all with the same
    /// extension - where nothing can be said you'll find nans
    /// \param min minx value of the time series
    /// \param max maxx value of the time series
    ///
    void setFixedMinMax(ScalarT min, ScalarT max)
    {
        min_x_ = min;
        max_x_ = max;
    }

protected:
    void fillIndicesIfNeeded();

    void extractFields()
    {

        if (selection_ != NULL)
        {
            LOG(INFO) << "Extracting selection";
            SelectionExtractor<Eigen::Vector3f, size_t> extractor;
            extractor.setSelection(selection_);
            extractor.setInputSet(in_cloud_);
            extractor.compute();
            indices_ = extractor.getInsideIds();

            LOG(INFO) << "extracted the " << extractor.getPercentageInside() << "% of points";
        }
        else
        {
            LOG(INFO) << "no selection found";
        }

        fillIndicesIfNeeded();

        if (do_autocalibration_)
        {

            LOG(INFO) << "going to compute an automatic calibration";
            // get the fields used in the autocalibration process
            if (in_cloud_->hasField(distance_field_name_))
            {
                in_cloud_->getField(distance_field_name_, indices_, distance_field_);
                LOG(INFO) << "distance field found";
            }

            if (in_cloud_->hasField(angle_field_name_))
            {
                in_cloud_->getField(angle_field_name_, indices_, angle_field_);
                LOG(INFO) << "angle field found";
            }

            LOG(INFO) << "fields for auto calibration extracted from cloud";

        }



        if (!x_field_name_.empty())
            in_cloud_->getField(x_field_name_, indices_, x_field_);

        else // we should have a stratigrahic model
        {
            x_field_ = evaluate_dynamic_scalar_field_generator<float,size_t>(in_cloud_,  model_, indices_);
        }

        in_cloud_->getField(y_field_name_, indices_, y_field_);
    }
public:
    void setDoAutoCalibration(const bool & status)
    {
        do_autocalibration_ = status;
    }
protected:
    SelectionRubberband::Ptr selection_;

    VariableScalarFieldBase::Ptr model_;

    PointCloudBase::Ptr in_cloud_;

    ScalarT sampling_step_;

    ScalarT bandwidth_;

    std::string x_field_name_;

    std::string y_field_name_;

    OutSeriesPtrT out_series_;

    std::vector<size_t> indices_;

    ScalarT min_x_;

    ScalarT max_x_;

// autocalibration stuff
    bool do_autocalibration_ = false;

    VectorT distance_field_;
    VectorT angle_field_;

    std::string distance_field_name_ = "distance";
    std::string angle_field_name_ = "angle";


    int n_distance_splits_ = 4;
    int n_angle_splits_ = 4;

private:
    ///
    /// \brief x_data field
    ///
    VectorT x_field_;

    ///
    /// \brief y_data field
    ///
    VectorT y_field_;





};

} // end nspace

#endif // TIME_SERIES_GENERATOR_H
