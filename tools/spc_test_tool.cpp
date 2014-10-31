#include <spc/methods/KernelSmoothing2.h>

#include <spc/elements/TimeSeriesSparse.h>
#include <spc/elements/TimeSeriesEquallySpaced.h>
#include <spc/io/element_io.h>


#include <spc/core/logging.h>
#include <spc/core/spc_eigen.h>

using namespace Eigen;
using namespace spc;



int main(int argc, char ** argv)
{

    google::InitGoogleLogging(argv[0]);
//    std::cout << "INIZIO" << std::endl;

    FLAGS_colorlogtostderr=1;
    FLAGS_logtostderr=1;

    spc::KernelSmoothing<float> sm;

    Eigen::VectorXf x = Eigen::VectorXf::LinSpaced(20,-10,10);
    Eigen::VectorXf y = Eigen::VectorXf::Random(20).array() + 2*x.array();

    sm.setInputPoints(x);
    sm.setValues(y);
    sm.setKernelSigma(4);
//    sm.initFlann();


    LOG(INFO) << sm.getKernel()->getSupportRegion();


    Eigen::VectorXf newy(2000);

        sm(x, newy);

    for (int i = 0; i < x.rows(); ++i)
    {
        std::cout << x(i) << " " << y(i) << " " << newy(i) << std::endl;
    }

<<<<<<< HEAD


//
=======
    KernelSmoothing2 smoother;

    Eigen::VectorXf x = Eigen::VectorXf::LinSpaced(2000,-10,10);
    Eigen::VectorXf y = Eigen::VectorXf::Random(2000);

    TimeSeriesSparse::Ptr in (new TimeSeriesSparse (x,y));

    TimeSeriesEquallySpaced::Ptr eqser(new TimeSeriesEquallySpaced(-10, 10, 1.0f));


    std::cout << eqser->getX() << std::endl;


    smoother.setBandwidth(4);
    smoother.setInputSeries(in);
    smoother.setOutputSeriesBlank(eqser);

    std::vector<std::pair<size_t, float>> matches;
    smoother.extractVectors();
    smoother.initFlann();
    size_t N = smoother.radiusSearch(1,0.2, matches);

    DLOG(INFO) << "n matches " << N;

    DLOG(INFO) << "n matches " << matches.size();

    for (int i = 0; i < matches.size(); ++i)
    {
        LOG(INFO) << i;
        LOG(INFO) <<i << " "<<  matches.at(i).first << " " << matches.at(i).second;
    }

    DLOG(INFO) << "printed!" ;
    smoother.compute();

    spc::TimeSeriesEquallySpaced::Ptr out = smoother.getOutputSeries();

    LOG(INFO) << out->getY();

//    DLOG(INFO) << "done----" ;


//    std::cout << eqser->getY() <<std::endl;



//    MatrixXi m(1, 5);
//    m << 1, 2, 3, 4, 5;
//    m = (m.array() > 3).select(3, m);


//    std::cout << m << std::endl;

//    std::cout << "Fine" << std::endl;
>>>>>>> 199a9f676e6a22aa5450f6144b73e278ad85a7ef
    return 0;
}
