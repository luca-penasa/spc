//#include <spc/methods/KernelSmoothing2.h>

#include <spc/elements/TimeSeriesSparse.h>
#include <spc/elements/TimeSeriesEquallySpaced.h>
#include <spc/io/element_io.h>
using namespace Eigen;
using namespace spc;



int main()
{
    std::cout << "INIZIO" << std::endl;

////    spc::BasicKernel<float>::Ptr a (new spc::GaussianKernel<float>(1));

////    std::cout << a->operator ()(2) << std::endl;

//    KernelSmoothing2 smoother;

//    Eigen::VectorXf x = Eigen::VectorXf::LinSpaced(2000,-10,10);
//    Eigen::VectorXf y = Eigen::VectorXf::Random(2000);

//    TimeSeriesSparse::Ptr in (new TimeSeriesSparse (x,y));

//    TimeSeriesEquallySpaced::Ptr eqser(new TimeSeriesEquallySpaced(-10, 10, 1.0f));


//    std::cout << eqser->getX() << std::endl;


//    smoother.setBandwidth(4);
//    smoother.setInputSeries(in);
//    smoother.setOutputSeriesBlank(eqser);
//    smoother.compute();

//    DLOG(INFO) << "done----" ;


//    std::cout << eqser->getY() <<std::endl;



//    MatrixXi m(1, 5);
//    m << 1, 2, 3, 4, 5;
//    m = (m.array() > 3).select(3, m);


//    std::cout << m << std::endl;

    std::cout << "Fine" << std::endl;
    return 0;
}
