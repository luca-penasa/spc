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

    std::vector<ScalarT> x_field;
    std::vector<ScalarT> y_field;

    if (y_field_name_.empty())
    {
        pcl::console::print_error("No scalar field name specified as values");
        return -1;
    }


    else if (in_cloud_ && model_)
    {
        fillIndicesIfNeeded();

        std::vector<float> x_d;

        if (!x_field_name_.empty())
            x_d = in_cloud_->getField(x_field_name_, indices_);

        else // we should have a stratigrahic model
            x_d= model_->getStratigraphicPositions(in_cloud_, indices_);

        std::cout << "size of s posistions " << x_d.size() << std::endl;
        std::vector<float> y_d = in_cloud_->getField(y_field_name_, indices_);

        std::cout << "size of field " << y_d.size() << std::endl;

        x_field = x_d;
        y_field = y_d;

    }

    else
    {
        pcl::console::print_error("No model and/or input cloud");

        return -1;
    }

    if ((x_field.empty()) || (y_field.empty()))
    {
        pcl::console::print_error("No suitable scalar field for computing the series. Wrong names?");
        return -1;
    }

    SparseTimeSeries<ScalarT>::Ptr series (new SparseTimeSeries<ScalarT> (x_field, y_field)); //the input series

    KernelSmoothing2<ScalarT> ks;


    //and also init the output series
    if (min_x_ != max_x_)

    {
        pcl::console::print_warn("instantiating!");
        //fixed min-max length series
        out_series_ = EquallySpacedTimeSeries<ScalarT>::Ptr(new EquallySpacedTimeSeries<ScalarT> (min_x_, max_x_, sampling_step_));
        if (!out_series_)
        {
            pcl::console::print_error("POINTER IS NULL\n");
        }
    }
    else
    {
        out_series_ = EquallySpacedTimeSeries<ScalarT>::Ptr(new EquallySpacedTimeSeries<ScalarT> (get_min(x_field), get_max(x_field), sampling_step_));
        if (!out_series_)
        {
            pcl::console::print_error("POINTER IS NULL 2.5\n");
        }
        ks.setStep(sampling_step_);
    }

    if (!out_series_)
    {
        pcl::console::print_error("POINTER IS NULL 2\n");
    }


    ks.setOutputSeriesBlank(out_series_);

    if (!out_series_)
    {
        pcl::console::print_error("POINTER IS NULL 3\n");
    }

    ks.setInputSeries(series);
    ks.setBandwidth(bandwidth_);

    int status = ks.compute();

    if (!out_series_)
    {
        pcl::console::print_error("POINTER IS NULL 3");
    }


    if (!out_series_)
        pcl::console::print_error("Pointer is void. Returning null in time series generator.\n");

    return  status;

}



void TimeSeriesGenerator::fillIndicesIfNeeded()
{
    if (indices_.size() == 0)
        for (int i = 0; i < in_cloud_->getSize(); ++i)
            indices_.push_back(i);

}


}//end nspace
