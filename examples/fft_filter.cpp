#include "aquila/global.h"
#include "aquila/source/generator/SineGenerator.h"
#include "aquila/transform/FftFactory.h"
#include "aquila/tools/TextPlot.h"
#include <algorithm>
#include <functional>
#include <memory>

#include <spc/methods/TimeSeriesBandPassFilter.h>
#include <spc/io/element_io.h>
int main()
{


    spc::ISerializable::Ptr l = spc::io::deserializeFromFile("/media/sandbox/golarossamodels/tstest.xml");
    spc::TimeSeriesEquallySpaced::Ptr ts = spcDynamicPointerCast<spc::TimeSeriesEquallySpaced> (l);

//    LOG(INFO) << ts->getY().head(10);

//    double a[10] ;



//    Eigen::VectorXd inx = ts->getY().cast<double>();

//    for (int i =0 ; i < 10; ++i)
//    {
//        a[i] = inx(i);

//        LOG(INFO) << a[i];
//    }

//    int len = inx.rows();
//    float f_sampling = 1/ ts->getXStep();


    spc::TimeSeriesBandPassFilter f;
    f.setInput(ts);
    f.doComputations();

//    spc::TimeSeriesEquallySpaced::Ptr out = spcDynamicPointerCast<spc::TimeSeriesEquallySpaced>(f.getOutput().at(0));

//    spc::io::serializeToFile(out, "/home/luca/outseries.xml", spc::io::XML);

    // input signal parameters
//    const std::size_t SIZE = 64;
//    const Aquila::FrequencyType sampleFreq = 2000;
//    const Aquila::FrequencyType f1 = 96, f2 = 813;
//    const Aquila::FrequencyType f_lp = 500;

//    Aquila::SineGenerator sineGenerator1(sampleFreq);
//    sineGenerator1.setAmplitude(32).setFrequency(f1).generate(SIZE);
//    Aquila::SineGenerator sineGenerator2(f_sampling);
//    sineGenerator2.setAmplitude(8).setFrequency(1).setPhase(0.75).generate(len);
//    auto sum = sineGenerator2;

//    Aquila::TextPlot plt("Signal waveform before filtration");
//    plt.plot(sum);

    // calculate the FFT
//    auto fft = Aquila::FftFactory::getFft(len);
//    Aquila::SpectrumType spectrum = fft->fft(sum.toArray());
//    plt.setTitle("Signal spectrum before filtration");
//    plt.plotSpectrum(spectrum);

//    // generate a low-pass filter spectrum
//    Aquila::SpectrumType filterSpectrum(SIZE);
//    for (std::size_t i = 0; i < SIZE; ++i)
//    {
//        if (i < (SIZE * f_lp / sampleFreq))
//        {
//            // passband
//            filterSpectrum[i] = 1.0;
//        }
//        else
//        {
//            // stopband
//            filterSpectrum[i] = 0.0;
//        }
//    }
//    plt.setTitle("Filter spectrum");
//    plt.plotSpectrum(filterSpectrum);

//    // the following call does the multiplication of two spectra
//    // (which is complementary to convolution in time domain)
//    std::transform(
//        std::begin(spectrum),
//        std::end(spectrum),
//        std::begin(filterSpectrum),
//        std::begin(spectrum),
//        [] (Aquila::ComplexType x, Aquila::ComplexType y) { return x * y; }
//    );
//    plt.setTitle("Signal spectrum after filtration");
//    plt.plotSpectrum(spectrum);

//    // Inverse FFT moves us back to time domain
//    double x1[SIZE];
//    fft->ifft(spectrum, x1);
//    plt.setTitle("Signal waveform after filtration");
//    plt.plot(x1, SIZE);

    return 0;
}
