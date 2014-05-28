#include <spc/methods/TimeSeriesGenerator.h>
#include <spc/io/pointcloud2_reader.h>
#include <spc/methods/KernelSmoothing.h>
#include <spc/elements/TimeSeriesSparse.h>
#include <pcl/console/print.h>
#include <spc/methods/KernelSmoothing2.h>

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

    if(!in_cloud_)
    {
        pcl::console::print_error("No input cloud");
        return -1;
    }


    // extract fields before to check
    doExtractFields();

    // now check
    if((!model_) && (x_field_.empty() || y_field_.empty()))
    {
        std::cout << model_ << x_field_.empty() << y_field_.empty() << std::endl;
        pcl::console::print_error("No input model or no x/y field names ");
        return -1;
    }

//    if ((x_field_.empty()) || (y_field_.empty()))
//    {
//        pcl::console::print_error("No suitable scalar field for computing the series. Wrong names?");
//        return -1;
//    }

    SparseTimeSeries<ScalarT>::Ptr series (new SparseTimeSeries<ScalarT> (x_field_, y_field_)); //the input series



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
        ks_.setStep(sampling_step_);
    }


    ks_.setOutputSeriesBlank(out_series_);
    ks_.setInputSeries(series);
    ks_.setBandwidth(bandwidth_);

    ks_.compute();


    return 1;

}



void TimeSeriesGenerator::fillIndicesIfNeeded()
{
    if (indices_.size() == 0)
        for (int i = 0; i < in_cloud_->size(); ++i)
            indices_.push_back(i);

}


}//end nspace
