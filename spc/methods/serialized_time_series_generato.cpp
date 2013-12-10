#include "serialized_time_series_generato.h"

namespace spc
{

int
SerializedTimeSeriesGenerator::compute()
{
    for (auto id_list: all_indices_)
    {
        TimeSeriesGenerator generator;
        generator.setBandwidth(bandwidth_);
        generator.setIndices(id_list);
        generator.setSamplingStep(sampling_step_);
//        generator.setInputReader(in_reader_);
        generator.setXFieldName(x_field_name_);
        generator.setLogFieldName(y_field_name_);
        if (!generator.compute())
            return -1;

        OutSeriesPtrT ts = generator.getOutputSeries();
        output_.push_back(ts);
    }
}




}//end nspace


