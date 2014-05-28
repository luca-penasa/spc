#include "TimeSeriesMultiGenerator.h"

namespace spc
{

int
SerializedTimeSeriesGenerator::compute()
{
    spcForEachMacro(std::vector<int> id_list, all_indices_)
    {
        TimeSeriesGenerator generator;
        generator.setBandwidth(bandwidth_);
        generator.setIndices(id_list);
        generator.setSamplingStep(sampling_step_);
//        generator.setInputReader(in_reader_);
        generator.setXFieldName(x_field_name_);
        generator.setYFieldName(y_field_name_);
        if (!generator.compute())
            return -1;

        OutSeriesPtrT ts = generator.getOutputSeries();
        output_.push_back(ts);
    }

    return 1;
}




}//end nspace


