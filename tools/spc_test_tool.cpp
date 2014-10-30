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

    LOG(INFO) << "testcolor";

    LOG(WARNING) << "warn";

    LOG(ERROR) << "error";





////    spc::BasicKernel<float>::Ptr a (new spc::GaussianKernel<float>(1));

////    std::cout << a->operator ()(2) << std::endl;

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
    return 0;
}
