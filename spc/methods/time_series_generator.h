#ifndef TIME_SERIES_GENERATOR_H
#define TIME_SERIES_GENERATOR_H

#include <sensor_msgs/PointCloud2.h>
#include <spc/time_series/equally_spaced_time_series.h>

namespace spc
{

template <typename ScalarT>
///
/// \brief The TimeSeriesGenerator class estimates time series starting from two fields of a cloud
/// the results is always a TimeSeries equally spaced. If you want to evaluate non-equally spaced time series
/// have a look directly to the KernelSmoothing method that permits to do that.
///
class TimeSeriesGenerator
{
public:
    ///
    /// \brief TimeSeriesGenerator def constructor
    ///
    TimeSeriesGenerator() {}

    ///
    /// \brief setInputCloud
    /// \param in_cloud
    ///
    void setInputCloud(sensor_msgs::PointCloud2::Ptr in_cloud) {in_cloud_ = in_cloud;}

    ///
    /// \brief setBandwidth
    /// \param bandwidth is the bandwidth used in the KernelSmoothing method
    ///
    void setBandwidth(ScalarT bandwidth) {bandwidth_ = bandwidth;}

    ///
    /// \brief setSamplingStep
    /// \param sampling_step is the step distance at which evaluate the time series values
    ///
    void setSamplingStep(ScalarT sampling_step) {sampling_step_ = sampling_step;}

    ///
    /// \brief setXFieldName
    /// \param field_name the field used as independen variable - x -
    ///
    void setXFieldName(std::string field_name) {x_field_name_ = field_name;}

    ///
    /// \brief setYFieldName
    /// \param field_name the field name to be used as dependent var - y -
    ///
    void setYFieldName(std::string field_name) {y_field_name_ = field_name;}

    ///
    /// \brief setIndices use only these points for estimatig the time series!
    /// if not given all the points will be used!
    /// \param indices
    ///
    void setIndices(std::vector<int> indices) {indices_ = indices;}

    ///
    /// \brief getOutputSeries
    /// \return the resulting series
    ///
    auto getOutputSeries() -> EquallySpacedTimeSeries<ScalarT> {return out_series_;}

    ///
    /// \brief compute make computations
    /// \return status -1 if something wrong!
    ///
    int compute();


private:
    ///
    /// \brief in_cloud_ the input cloud
    ///
    sensor_msgs::PointCloud2::Ptr in_cloud_;

    ///
    /// \brief sampling_step_
    ///
    ScalarT sampling_step_;

    ///
    /// \brief bandwidth_
    ///
    ScalarT bandwidth_;

    ///
    /// \brief x_field_name_
    ///
    std::string x_field_name_;

    ///
    /// \brief y_field_name_
    ///
    std::string y_field_name_;

    ///
    /// \brief out_series_
    ///
    EquallySpacedTimeSeries<ScalarT> out_series_;

    ///
    /// \brief indices_
    ///
    std::vector<int> indices_;

    ///
    /// \brief x_data
    ///
    std::vector<ScalarT> x_data;

    ///
    /// \brief y_data
    ///
    std::vector<ScalarT> y_data;

};

}//end nspace

#endif // TIME_SERIES_GENERATOR_H
