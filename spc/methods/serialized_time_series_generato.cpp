#include "serialized_time_series_generato.h"
#include <spc/methods/time_series_generator.h>

namespace spc
{

template <typename ScalarT>
int
SerializedTimeSeriesGenerator<ScalarT>::compute()
{
    for (auto id_list: all_indices_)
    {
        TimeSeriesGenerator<ScalarT> generator;
        generator.setBandwidth(bandwidth_);
        generator.setIndices(id_list);
        generator.setSamplingStep(sampling_step_);
        generator.setInputReader(in_reader_);
        generator.setXFieldName(x_field_name_);
        generator.setYFieldName(y_field_name_);
        if (!generator.compute())
            return -1;

        auto out = generator.getOutputSeries();
        output_.push_back(out);
    }
}


//instantiate
template class SerializedTimeSeriesGenerator<float>;
template class SerializedTimeSeriesGenerator<double>;

}//end nspace


