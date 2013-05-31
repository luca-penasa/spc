#include <spc/methods/time_series_generator.h>
#include <spc/io/pointcloud2_reader.h>
#include <spc/methods/kernel_smoothing.h>
#include <spc/time_series/sparse_time_series.h>
#include <pcl/console/print.h>

namespace spc
{

template <typename ScalarT>
int
TimeSeriesGenerator<ScalarT>::compute()
{

    in_reader_->setIndices(indices_); // we set the indices in reader. if have size zero he knows what to do!
    auto x_field = in_reader_->getScalarFieldAsStdVector<ScalarT>(x_field_name_);
    auto y_field = in_reader_->getScalarFieldAsStdVector<ScalarT>(y_field_name_);

    if ((!x_field) || (!y_field))
    {
        pcl::console::print_error("No suitable scalar field for computing the series. Wrong names?");
        return -1;
    }

    SparseTimeSeries<ScalarT> series (*x_field, *y_field); //the input series

    //and also init the output series
    if (f_min == f_max)
        out_series_ = EquallySpacedTimeSeries<ScalarT> (series.getMinX(), series.getMaxX(), sampling_step_);
    else
        out_series_ = EquallySpacedTimeSeries<ScalarT> (f_min, f_max, sampling_step_);

    //now use ks method
    KernelSmoothing<ScalarT> ks;
    ks.setInput(&series);
    ks.setBandwidth(bandwidth_);
    return ks.compute(&out_series_);

}

//instantiations


template class TimeSeriesGenerator<float>;
template class TimeSeriesGenerator<double>;



}//end nspace
