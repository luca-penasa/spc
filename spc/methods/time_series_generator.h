#ifndef TIME_SERIES_GENERATOR_H
#define TIME_SERIES_GENERATOR_H

#include <sensor_msgs/PointCloud2.h>
#include <spc/time_series/equally_spaced_time_series.h>
#include <spc/io/pointcloud2_reader.h>
#include <spc/elements/generic_cloud.h>

#include <spc/stratigraphy/stratigraphic_model_base.h>

namespace spc
{

template <typename ScalarT>
///
/// \brief The TimeSeriesGenerator class estimates time series starting from two fields of a cloud
/// the results is always a TimeSeries equally spaced. If you want to evaluate non-equally spaced time series
/// have a look directly to the KernelSmoothing method that permits to do that.
///
/// You can also chose to set as input a stratigraphic model + a cloud as a generic cloud
///
class TimeSeriesGenerator
{
public:
    ///
    /// \brief TimeSeriesGenerator def constructor
    ///
    TimeSeriesGenerator(): in_reader_(0), in_cloud_(0), model_(0)
    {
        f_min = 0 ; f_max = 0;
    }

    void setStratigraphicModel(StratigraphicModelBase * model)
    {
        model_ = model;
    }

    void setInputCloud(spcGenericCloud * cloud)
    {
        in_cloud_ = cloud;
    }




    ///
    /// \brief setInputCloud
    /// \param in_cloud is a sensor msgs cloud
    /// \note this is deprecated, please use the setInputCloud and setStratigraphicModel way
    void setInputReader(spc::PointCloud2Reader * reader) {in_reader_ = reader;}

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

    ///
    /// \brief setFixedMinMax permits to create a bunch of ts all with the same extension - where nothing can be said you'll find nans
    /// \param min minx value of the time series
    /// \param max maxx value of the time series
    ///
    void setFixedMinMax(ScalarT min, ScalarT max) {f_min = min; f_max=max;}


    void fillIndicesIfNeeded();

private:

    StratigraphicModelBase * model_;

    spcGenericCloud * in_cloud_;

    PointCloud2Reader * in_reader_;

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

    ///
    /// \brief f_min
    ///
    ScalarT f_min;

    ///
    /// \brief f_max
    ///
    ScalarT f_max;

};

}//end nspace

#endif // TIME_SERIES_GENERATOR_H
