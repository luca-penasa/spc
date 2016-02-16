#ifndef SIFT1D_H
#define SIFT1D_H

#include <spc/core/spc_eigen.h>
#include <spc/elements/TimeSeriesEquallySpaced.h>
#include <unsupported/Eigen/FFT>


namespace spc
{


class Sift1d
{
public:
    Sift1d();


    void setInputSeries(spc::TimeSeriesEquallySpaced::Ptr ts)
    {
        inseries_ = ts;
        signal_ = inseries_->getY().cast<double> ();

        f_sampling = ts->getSamplingFrequency();

        init_sigma = 1.6 * ts->getXStep();

        fillNans();
    }

    static Eigen::VectorXd fftshift(const Eigen::VectorXd &in)
    {
        Eigen::VectorXd v = in;
        int half_len = floor(v.rows() / 2);

        bool odd = v.rows() %2 ;

        double tmp;
        if (!odd) // even
        {
            for (int i =0 ; i <half_len;++i)
            {
                tmp = v(i);
                v(i) = v(half_len  + i);
                v(half_len  + i) = tmp;

            }
        }
        else // is odd
        {
            for (int i =0 ; i <half_len+1;++i)
            {
                tmp = v(i);
                v(i) = v(half_len + 1  + i);
                if (i<half_len+1)
                    v(half_len + i) = tmp;

            }
        }

        return v;
    }

    static Eigen::VectorXd blur(const Eigen::VectorXd & v,const double & sigma)
    {
        size_t N = v.rows();

        Eigen::FFT<double>fft;
        Eigen::VectorXd w = gaussian_window(N, sigma);
        w /= w.sum();
        Eigen::VectorXcd sft = fft.fwd(v);
        Eigen::VectorXcd wft = fft.fwd(w);
        Eigen::VectorXcd filtered =  sft.array() * wft.array();
        Eigen::VectorXcd out = fft.inv(filtered);
        return fftshift(out.array().real());

    }

    static Eigen::VectorXd gaussian_window(size_t N, double sigma)
    {
        bool odd  = N / 2.0;

//        LOG(INFO) << "odd" << odd;

        if (!odd)
            N+=1;

        Eigen::VectorXd w = Eigen::VectorXd::LinSpaced(N, 0, N-1).array() - (N-1) /2.0;
//        LOG(INFO) << w ;

        double sig2 = 2 * sigma*sigma;
        w = -w.array()*w.array() / sig2;
        w = w.array().exp();

        if (!odd)
            return w.head(N-1);
        else
            return  w;
    }

    void setInputSignal(const Eigen::VectorXd & signal)
    {
        signal_ = signal;
        // ! fill nans with the mean

        // create a ts object for easily access the data etc
        spc::TimeSeriesEquallySpaced::Ptr tsptr (new spc::TimeSeriesEquallySpaced);
        tsptr->setXStep(1);
        tsptr->setXStart(0);
        tsptr->setY(signal.cast<float>());
        inseries_ = tsptr;

        f_sampling = 1;

        fillNans();

    }

    void fillNans()
    {
        Eigen::Matrix<bool, -1, 1> goods = inseries_->getNanMask();
        double mean  = inseries_->getMean();
        for (int i = 0; i < signal_.rows(); ++i)
            if (!goods(i))
                signal_(i) = mean;
    }

    Eigen::VectorXd generateFilterSpectrum(const double cutoff_freq, size_t len)
    {

        Eigen::VectorXd spec;
        spec.resize(len);
        spec.fill(0);
        for (std::size_t i = 0; i < len; ++i)
            if (i < (len * cutoff_freq / f_sampling))
//                filterSpectrum(i) = 1;

        return spec;
    }


    static Eigen::VectorXd resample(const Eigen::VectorXf &v)
    {
        size_t new_size = floor(v.rows()/2);
        Eigen::VectorXd out;
        out.resize(new_size);

        for (int i = 0; i < new_size; ++i)
            out(i) = v(i*2);

        return out;
    }

    void compute()
    {

        std::vector<Eigen::Matrix<double, -1, -1>> out;

        for (int oct = 0; oct < n_octaves; ++oct)
        {
            int len = floor(signal_.rows() / (oct +1));

            Eigen::MatrixXd octave;
            octave.resize(n_scales, len);
            octave.fill(spcNANMacrod);

            for (int scal = 0; scal < n_scales; ++scal)
            {

//                double sigma = init_sigma * (scal+1) * sqrt(2)



            }
        }
    }



protected:
    size_t n_octaves = 4;
    size_t n_scales = 5;
    double init_sigma  = 1.6;
    double k = sqrt(2);
    double f_sampling;


    Eigen::VectorXd signal_;

    spc::TimeSeriesEquallySpaced::Ptr inseries_ = nullptr;




};

}// end nspace

#endif // SIFT1D_H
