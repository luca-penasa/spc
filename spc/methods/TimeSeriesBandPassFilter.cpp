#include "TimeSeriesBandPassFilter.h"

//#include <aquila/aquila.h>
//#include <aquila/transform/FftFactory.h>
#include <unsupported/Eigen/FFT>
#include <spc/elements/TimeSeriesEquallySpaced.h>

namespace spc
{

TimeSeriesBandPassFilter::TimeSeriesBandPassFilter()
{
    name_ = "bandpass";
}

TimeSeriesEquallySpaced::Ptr TimeSeriesBandPassFilter::filter_aquila()

{

//    LOG(INFO) << "beginning the filtering";

//    if (!series_)
//    {
//        LOG(WARNING) << "IN series was nullptr. returning nullptr";
//        return nullptr;
//    }

//    Eigen::VectorXd y = series_->getY().cast<double>();

//    LOG(INFO) << "TS: "<< y.head(5);








//    size_t len = y.rows();
//    float f_sampling  = 1/ series_->getXStep();




//    Aquila::SineGenerator sineGenerator2(f_sampling);
//    sineGenerator2.setAmplitude(8).setFrequency(f_low_).setPhase(0.75).generate(len);





//    auto fft = Aquila::FftFactory::getFft(len);
////    Aquila::SpectrumType spectrum = fft->fft(y.data());

//        Aquila::SpectrumType spectrum = fft->fft(sineGenerator2.toArray());

//        LOG(INFO) << "spectrum computed";


//    Aquila::SpectrumType filterSpectrum(len);
//    for (std::size_t i = 0; i < len; ++i)
//    {
//        if (i < (len * f_low_ / f_sampling))
//        {
//            // passband
//            filterSpectrum[i] = 0;
//        }
//        else if (i >= (len * f_high_ / f_sampling))
//        {
//            // stopband
//            filterSpectrum[i] = 0;
//        }
//        else
//        {
//            filterSpectrum[i] = 1;
//        }
//    }



//    std::transform(
//                std::begin(spectrum),
//                std::end(spectrum),
//                std::begin(filterSpectrum),
//                std::begin(spectrum),
//                [] (Aquila::ComplexType x, Aquila::ComplexType y) { return x * y; }
//    );



//    Eigen::VectorXd x;
//    x.resize(len);

//    //           double x1[SIZE];



//    fft->ifft(spectrum, x.data());


//    Eigen::VectorXf xfloat = x.cast<float>();

//    LOG(INFO) << xfloat;


//    spc::TimeSeriesEquallySpaced::Ptr outseries (new spc::TimeSeriesEquallySpaced(*series_));

//    outseries->setY(xfloat);

//    LOG(INFO) << "bandpass filtering finished";




//    return outseries;


}

TimeSeriesEquallySpaced::Ptr TimeSeriesBandPassFilter::filter_eigen()
{
    int len = series_->getNumberOfSamples();
    float f_sampling = series_->getSamplingFrequency();

    Eigen::VectorXf timevec = series_->getY();
    Eigen::Matrix<bool, -1, 1> good = series_->getNanMask();


    float mean = series_->getMean();


    LOG(INFO) << "mean of signal: " << mean;

    for (int i = 0 ; i < timevec.rows(); ++i)
        if (good(i) == false)
            timevec(i) = mean;


    Eigen::FFT<float> fft;

//    std::vector<float> timevec = MakeMyData();
    Eigen::VectorXcf freqvec;

    fft.fwd( freqvec, timevec);

//    LOG(INFO) << freqvec;


    Eigen::VectorXcf filterSpectrum;
    filterSpectrum.resize(len);
    for (std::size_t i = 0; i < len; ++i)
    {
        if (i < (len * f_low_ / f_sampling))
        {
            // passband
            filterSpectrum(i) = 0;
        }
        else if (i >= (len * f_high_ / f_sampling))
        {
            // stopband
            filterSpectrum(i) = 0;
        }
        else
        {
            filterSpectrum(i) = 1;
        }
    }

//    Eigen::VectorXcf convolved;
//    convolved.resize(len);

    Eigen::VectorXcf convolved  = filterSpectrum.array() * freqvec.array();




    // manipulate freqvec
    fft.inv( timevec,convolved);


    for (int i = 0 ; i < timevec.rows(); ++i)
        if (good(i) == false)
            timevec(i) = std::numeric_limits<float>::quiet_NaN();


//    LOG(INFO) << "filtered " << timevec;

    spc::TimeSeriesEquallySpaced::Ptr outseries (new spc::TimeSeriesEquallySpaced(*series_));

    outseries->setY(timevec);




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

        LOG(INFO) << "using the first series found";
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

//        if (backend == AQUILA)
//            out_series_ = this->filter_aquila();

        if (backend ==EIGENFFT)
            out_series_ = this->filter_eigen();


        if (preserve_mean_)
        {
            float mean = series_->getMean();

            out_series_->setMean(mean);
        }



        if (out_series_)
        {
            LOG(INFO) << "got a good time series as out";
        }

        outelements_.push_back(out_series_);
    }


}





}
