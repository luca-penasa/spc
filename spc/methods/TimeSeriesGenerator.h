#ifndef TIME_SERIES_GENERATOR_H
#define TIME_SERIES_GENERATOR_H

#include <spc/methods/common.h>

#include <spc/elements/TimeSeriesEquallySpaced.h>
#include <spc/io/pointcloud2_reader.h>
#include <spc/elements/PointCloudBase.h>

#include <spc/elements/VariableScalarFieldBase.h>

#include <spc/methods/KernelSmoothing2.h>

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
    typedef spc::TimeSeriesEquallySpaced<ScalarT> OutSeriesT;
    typedef spcSharedPtrMacro<OutSeriesT> OutSeriesPtrT;

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

    void setMinMaxLog(float min, float max)
    {
        min_x_ = min;
        max_x_ = max;
    }

    ///
    /// \brief setLogFieldName
    /// \param field_name the name of the field to be logged
    /// \note this var is mandatory
    ///
    void setYFieldName(std::string field_name)
    {
        y_field_name_ = field_name;
    }

    ///
    /// \brief setIndices use only these points for estimatig the time series!
    /// if not given all the points will be used!
    /// \param indices
    ///
    void setIndices(std::vector<int> indices)
    {
        indices_ = indices;
    }

    ///
    /// \brief getOutputSeries
    /// \return the resulting series
    ///
    inline OutSeriesPtrT getOutputSeries()
    {
        if (!out_series_)
            pcl::console::print_error(
                "out series is void. Null pointer returned!");

        return out_series_;
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

    void doExtractFields()
    {
        fillIndicesIfNeeded();

        std::vector<float> x_d;

        if (!x_field_name_.empty())
            x_d = in_cloud_->getField(x_field_name_, indices_);

        else // we should have a stratigrahic model
            x_d = model_->getScalarFieldValues(in_cloud_, indices_);

        std::vector<float> y_d = in_cloud_->getField(y_field_name_, indices_);

        x_field_ = x_d;
        y_field_ = y_d;
    }

    VariableScalarFieldBase::Ptr model_;

    PointCloudBase::Ptr in_cloud_;

    ScalarT sampling_step_;

    ScalarT bandwidth_;

    std::string x_field_name_;

    std::string y_field_name_;

    OutSeriesPtrT out_series_;

    std::vector<int> indices_;

    ScalarT min_x_;

    ScalarT max_x_;

private:
    // these vars are for internal use only - used to temporarly store the data

    ///
    /// \brief x_data field
    ///
    std::vector<ScalarT> x_field_;

    ///
    /// \brief y_data field
    ///
    std::vector<ScalarT> y_field_;

    KernelSmoothing2<ScalarT> ks_;
};

} // end nspace

#endif // TIME_SERIES_GENERATOR_H
