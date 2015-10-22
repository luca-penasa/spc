#ifndef SERIALIZED_TIME_SERIES_GENERATOR_H
#define SERIALIZED_TIME_SERIES_GENERATOR_H

#include <spc/methods/PointCloudSlicer.h>
#include <spc/elements/TimeSeriesEquallySpaced.h>
//#include <spc/io/pointcloud2_reader.h>
#include <spc/methods/TimeSeriesGenerator.h>

namespace spc
{

class SerializedTimeSeriesGenerator
{
public:
    typedef float ScalarT;

    typedef std::vector<size_t> Indices;
    typedef std::vector<Indices> IndicesContainer;

    typedef spc::TimeSeriesEquallySpaced OutSeriesT;
    typedef spcSharedPtrMacro<OutSeriesT> OutSeriesPtrT;

public:
    SerializedTimeSeriesGenerator()
    {
    }

    void setIndices(IndicesContainer all_indices)
    {
        all_indices_ = all_indices;
    }

    void setSamplingStep(ScalarT step)
    {
        sampling_step_ = step;
    }

    void setBandwidth(ScalarT band)
    {
        bandwidth_ = band;
    }

    ///
    /// \brief setXFieldName
    /// \param field_name the field used as independen variable - x -
    ///
    void setXFieldName(std::string field_name)
    {
        x_field_name_ = field_name;
    }

    ///
    /// \brief setYFieldName
    /// \param field_name the field name to be used as dependent var - y -
    ///
    void setYFieldName(std::string field_name)
    {
        y_field_name_ = field_name;
    }

    int compute();

    std::vector<OutSeriesPtrT> getOutput()
    {
        return output_;
    }

private:
    //    pcl::PCLPointCloud2::Ptr in_cloud_ ;
    IndicesContainer all_indices_;
    ScalarT sampling_step_;
    ScalarT bandwidth_;

    std::string x_field_name_;
    std::string y_field_name_;

    std::vector<OutSeriesPtrT> output_;
};

} // end nspace
#endif // SERIALIZED_TIME_SERIES_GENERATO_H
