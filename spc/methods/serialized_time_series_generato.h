#ifndef SERIALIZED_TIME_SERIES_GENERATOR_H
#define SERIALIZED_TIME_SERIES_GENERATOR_H

#include <spc/methods/cloud_slicer.h>
#include <spc/time_series/equally_spaced_time_series.h>
#include <spc/io/pointcloud2_reader.h>

namespace spc
{

template<typename ScalarT>
class SerializedTimeSeriesGenerator
{
public:
    typedef std::vector<int> Indices;
    typedef std::vector<Indices> IndicesContainer;

public:
    SerializedTimeSeriesGenerator() {};

    void setInputReader(spc::PointCloud2Reader * reader) {in_reader_ = reader;}

    void setIndices(IndicesContainer all_indices) {all_indices_ = all_indices;}

    void setSamplingStep(ScalarT step) {sampling_step_ = step;}

    void setBandwidth(ScalarT band) {bandwidth_ = band;}

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


    int compute();

    auto getOutput() -> std::vector<EquallySpacedTimeSeries<ScalarT> > {return output_;}

private:
//    sensor_msgs::PointCloud2::Ptr in_cloud_ ;
    spc::PointCloud2Reader * in_reader_;
    IndicesContainer all_indices_;
    ScalarT sampling_step_;
    ScalarT bandwidth_;

    std::string x_field_name_;
    std::string y_field_name_;

    std::vector<EquallySpacedTimeSeries<ScalarT> > output_;
};

}//end nspace
#endif // SERIALIZED_TIME_SERIES_GENERATO_H
