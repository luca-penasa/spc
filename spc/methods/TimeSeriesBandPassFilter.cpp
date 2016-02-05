#include "TimeSeriesBandPassFilter.h"

#include <aquila/aquila.h>
#include <aquila/transform/FftFactory.h>

namespace spc
{

TimeSeriesBandPassFilter::TimeSeriesBandPassFilter()
{
    name_ = "bandpass";
}

TimeSeriesEquallySpaced::Ptr TimeSeriesBandPassFilter::filter()

{

    if (!series_)
        return nullptr;

    Eigen::VectorXd y = series_->getY().cast<double>();

    LOG(INFO) << "TS: "<< y;


    size_t len = y.rows();

    float f_sampling  = 1/ series_->getXStep();

    auto fft = Aquila::FftFactory::getFft(len);
    Aquila::SpectrumType spectrum = fft->fft(y.data());


    Aquila::SpectrumType filterSpectrum(len);
    for (std::size_t i = 0; i < len; ++i)
    {
        if (i < (len * f_low_ / f_sampling))
        {
            // passband
            filterSpectrum[i] = 0;
        }
        else if (i >= (len * f_high_ / f_sampling))
        {
            // stopband
            filterSpectrum[i] = 0;
        }
        else
        {
            filterSpectrum[i] = 1;
        }
    }



    std::transform(
                std::begin(spectrum),
                std::end(spectrum),
                std::begin(filterSpectrum),
                std::begin(spectrum),
                [] (Aquila::ComplexType x, Aquila::ComplexType y) { return x * y; }
    );



    Eigen::VectorXd x;
    x.resize(len);

    //           double x1[SIZE];



    fft->ifft(spectrum, x.data());


    Eigen::VectorXf xfloat = x.cast<float>();

    LOG(INFO) << xfloat;


    spc::TimeSeriesEquallySpaced::Ptr outseries (new spc::TimeSeriesEquallySpaced(*series_));

    outseries->setY(xfloat);


    return outseries;


}

InputRequirements TimeSeriesBandPassFilter::getInputRequirements() const
{
    InputRequirements reqs;
    reqs.types.push_back(&spc::TimeSeriesEquallySpaced::Type);

    return reqs;
}

void TimeSeriesBandPassFilter::doComputations()
{



    if (inelements_.size() >= 1)
    {
        spc::ElementBase::Ptr el = inelements_.at(0);
        if (el)
        {
            spc::TimeSeriesEquallySpaced::Ptr ts = spcDynamicPointerCast<spc::TimeSeriesEquallySpaced>(el);
            if (ts)
                series_ =ts;
        }

    }

    if (series_ != nullptr)
    {
        out_series_ = this->filter();

        outelements_.push_back(out_series_);
    }


}





}
