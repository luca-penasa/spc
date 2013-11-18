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

    std::vector<ScalarT> x_field;
    std::vector<ScalarT> y_field;

    if (y_field_name_.empty())
    {
        pcl::console::print_error("No scalar field name specified as values");
        return -1;
    }

    if (in_reader_)
    {

        if (x_field_name_.empty())
        {
            pcl::console::print_error("No scalar field name specified sp");
            return -1;
        }

        in_reader_->setIndices(indices_); // we set the indices in reader. if have size zero he knows what to do!
        x_field = *in_reader_->getScalarFieldAsStdVector<ScalarT>(x_field_name_);
        y_field = *in_reader_->getScalarFieldAsStdVector<ScalarT>(y_field_name_);
    }


    else if (in_cloud_ && model_)
    {
        fillIndicesIfNeeded();

        std::vector<float> x_d = model_->getStratigraphicPositions(in_cloud_, indices_);

        std::cout << "size of s posistions " << x_d.size() << std::endl;
        std::vector<float> y_d = in_cloud_->getField(y_field_name_, indices_);

        std::cout << "size of field " << y_d.size() << std::endl;

        /////NOTE here I am doing really stupid things. More time needed for clening the code!
        std::vector<ScalarT> x_field2 (x_d.begin(), x_d.end());
        std::vector<ScalarT> y_field2 (y_d.begin(), y_d.end());



        x_field = x_field2;
        y_field = y_field2;


    }

    else
    {
        pcl::console::print_error("No model and/or input cloud");

        return -1;
    }

    if ((x_field.size() == 0) || (y_field.size() == 0))
    {
        pcl::console::print_error("No suitable scalar field for computing the series. Wrong names?");
        return -1;
    }

    SparseTimeSeries<ScalarT> series (x_field, y_field); //the input series

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


template <typename ScalarT>
void TimeSeriesGenerator<ScalarT>::fillIndicesIfNeeded()
{
    if (indices_.size() == 0)
        for (int i = 0; i < in_cloud_->getSize(); ++i)
            indices_.push_back(i);

}

//instantiations


template class TimeSeriesGenerator<float>;
template class TimeSeriesGenerator<double>;



}//end nspace
