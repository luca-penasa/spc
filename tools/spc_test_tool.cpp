#include <spc/methods/KernelSmoothing2.h>

#include <spc/elements/TimeSeriesSparse.h>
#include <spc/elements/TimeSeriesEquallySpaced.h>
#include <spc/io/element_io.h>


#include <spc/core/logging.h>
#include <spc/core/spc_eigen.h>

using namespace Eigen;
//using namespace spc;



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



//
    return 0;
}
