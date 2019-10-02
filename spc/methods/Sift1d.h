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



    template <typename vecT>
    static vecT fftshift(const vecT &in)
    {
//          typedef typename Eigen::internal::plain_row_type<Derived>::type RowVectorType;

        vecT v = in;
        int half_len = floor(v.rows() / 2);

        bool odd = v.rows() %2 ;

        typename vecT::Scalar tmp;
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
            // \todo do it in place
        {
            vecT head = v.head(half_len+1);
            vecT tail = v.tail(half_len ) ;

            v.head(half_len) = tail;
            v.tail(half_len+1) = head;
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

//        wft = fftshift(wft);
//        sft = fftshift(sft);



        Eigen::VectorXcd filtered =  sft.array() * wft.array();
        Eigen::VectorXcd out = fft.inv(filtered);

        Eigen::VectorXd real = out.array().real();
        return fftshift(real);

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


    static Eigen::VectorXd resample(const Eigen::VectorXd &v)
    {
        size_t new_size = floor(v.rows()/2);
        Eigen::VectorXd out;
        out.resize(new_size);

        for (int i = 0; i < new_size; ++i)
            out(i) = v(i*2);

        return out;
    }

    std::vector<std::vector<Eigen::VectorXd>> compute()
    {
//        oStack = np.empty([numOctaves,stackSpace+3], dtype=object)


//    #compute the first octave. The initial image is resized and Interpolated by a factor of 2
//    iDist = iDistMin

//    oStack[0,0] = initImage
//    for j in range(1,stackSpace+3):
//        k = (2.0**(float(j)/float(stackSpace)))
//        sigma = (iDist/iDistMin)*sigmaMin*k
//        oStack[0,j] = GBlur(oStack[0,0], sigma)

//    #build other octaves
//    for i in range(1,numOctaves):
//        iDist = iDistMin*(2**i)
//        oStack[i,0] = resample(oStack[i-1,oStack[i-1].shape[0]-3])
//        for j in range(1,stackSpace+3):
//            k = (2**(float(j)/float(stackSpace)))
//            sigma = (iDist/iDistMin)*sigmaMin*k
//            oStack[i,j] = GBlur(oStack[i,0], sigma)

        double dist = dist_min;


        std::vector<std::vector<Eigen::VectorXd>> out;
        out.resize(n_octaves);
        for (int i =0 ; i < n_octaves; ++i)
            out.at(i).resize(n_scales + 3);


        out[0][0] = signal_;
        for (int i = 1; i < n_scales + 3; ++i)
        {
            double k = (pow(2.0,(float(i))/float(n_scales)));
            double sigma = (dist/dist_min)*init_sigma*k;
            out[0][i] =blur(signal_, sigma);
        }

        //other octaves
        for (int i = 1; i < n_octaves; ++i)
        {
            dist  = dist_min*(pow(2,i));
            out[i][0] = resample(out[i-1][out[i-1].size() - 3]);
             for (int j =1; j < n_scales + 3; ++j)
             {
                 double k = (pow(2.0,(float(j))/float(n_scales)));
                 double sigma = (dist/dist_min)*init_sigma*k;
                 out[i][j] = blur(out[i][0], sigma);

             }
        }


       return out;
    }



protected:
    size_t n_octaves = 4;
    size_t n_scales = 5;
    double init_sigma  = 1.6;
    double dist_min  = 0.5;
    double k = sqrt(2);
    double f_sampling;


    Eigen::VectorXd signal_;

    spc::TimeSeriesEquallySpaced::Ptr inseries_ = nullptr;




};

}// end nspace

#endif // SIFT1D_H
