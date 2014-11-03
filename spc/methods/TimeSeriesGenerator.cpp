#include <spc/methods/TimeSeriesGenerator.h>
//#include <spc/io/pointcloud2_reader.h>
//#include <spc/methods/KernelSmoothing.h>
#include <spc/elements/TimeSeriesSparse.h>
#include <pcl/console/print.h>
#include <spc/methods/KernelSmoothing2.h>

namespace spc
{

int TimeSeriesGenerator::compute()
{

    if (y_field_name_.empty()) {
        LOG(ERROR) << "No scalar field name specified as values";
        return -1;
    }

    if (!in_cloud_) {
        LOG(ERROR) << "No input cloud";
        return -1;
    }

    // extract fields before to check
    extractFields();

    // now check
    if ((!model_) && (x_field_.empty() || y_field_.empty())) {
        LOG(ERROR) << "No input model or no x/y field names ";
        return -1;
    }

    KernelSmoothing<ScalarT> ks(x_field_, y_field_);
    ks.setKernelSigma(bandwidth_);

    // and also init the output series
    if (min_x_ != max_x_)
    {
        // fixed min-max length series
        out_series_ = TimeSeriesEquallySpaced
            ::Ptr(new TimeSeriesEquallySpaced
                           (min_x_, max_x_, sampling_step_));

    } else
    {
        out_series_ = TimeSeriesEquallySpaced::Ptr(
            new TimeSeriesEquallySpaced
            (x_field_.minCoeff(), x_field_.maxCoeff(), sampling_step_));
    }

    int status = ks(out_series_->getX(), out_series_->getY());

    if (!status)
    {
        LOG(ERROR) << "Some error occured while genrating the ts";
        return -1;
    }

    return 1;
}

void TimeSeriesGenerator::fillIndicesIfNeeded()
{
    if (indices_.size() == 0)
        for (int i = 0; i < in_cloud_->size(); ++i)
            indices_.push_back(i);
}

} // end nspace
