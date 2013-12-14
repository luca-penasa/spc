#include <spc/methods/time_series_generator.h>
#include <spc/io/pointcloud2_reader.h>
#include <spc/methods/kernel_smoothing.h>
#include <spc/time_series/sparse_time_series.h>
#include <pcl/console/print.h>
#include <spc/methods/kernelsmoothing2.h>

namespace spc
{


int
TimeSeriesGenerator::compute()
{



    if (y_field_name_.empty())
    {
        pcl::console::print_error("No scalar field name specified as values");
        return -1;
    }

    if(!in_cloud_ | !model_)
    {
        pcl::console::print_error("No model and/or input cloud");
        return -1;
    }

    doExtractFields();

    if ((x_field_.empty()) || (y_field_.empty()))
    {
        pcl::console::print_error("No suitable scalar field for computing the series. Wrong names?");
        return -1;
    }

    SparseTimeSeries<ScalarT>::Ptr series (new SparseTimeSeries<ScalarT> (x_field_, y_field_)); //the input series

    KernelSmoothing2<ScalarT> ks;

    //and also init the output series
    if (min_x_ != max_x_)

    {
        pcl::console::print_warn("instantiating!");
        //fixed min-max length series
        out_series_ = EquallySpacedTimeSeries<ScalarT>::Ptr(new EquallySpacedTimeSeries<ScalarT> (min_x_, max_x_, sampling_step_));

    }
    else
    {
        out_series_ = EquallySpacedTimeSeries<ScalarT>::Ptr(new EquallySpacedTimeSeries<ScalarT> (get_min(x_field_), get_max(x_field_), sampling_step_));
        ks.setStep(sampling_step_);
    }


    ks.setOutputSeriesBlank(out_series_);
    ks.setInputSeries(series);
    ks.setBandwidth(bandwidth_);

    ks.compute();


    return 1;

}



void TimeSeriesGenerator::fillIndicesIfNeeded()
{
    if (indices_.size() == 0)
        for (int i = 0; i < in_cloud_->size(); ++i)
            indices_.push_back(i);

}


}//end nspace
