#include <spc/methods/KernelSmoothing2.h>

#include <spc/elements/TimeSeriesSparse.h>
#include <spc/elements/TimeSeriesEquallySpaced.h>
#include <spc/io/element_io.h>


#include <spc/core/logging.h>
#include <spc/core/spc_eigen.h>
#include <Eigen/SVD>






void test1()
{
//    typedef  NanoFlannIndexT;


    Eigen::VectorXf points(5);

    DLOG(INFO) << "input rows "<< points.rows();
    DLOG(INFO) << "input cols "<< points.cols();

//    Eigen::MatrixXf * mat2 = &points;

    points << 1.2, 22., 35, 41, 25;

    LOG(INFO) << "going to create an index";
    NanoFlannEigenMatrixAdaptor<Eigen::MatrixXf> index (points, 10);
//    index.index->buildIndex();

    LOG(INFO) << "points after  " << points.transpose();

    Eigen::Matrix<float, -1, 1> query(1);
    query << 1.5;

    LOG(INFO) << "QUERY: " << query;


    std::vector<std::pair<size_t, float>> result;
    nanoflann::SearchParams pars;

    float radius  = 20.500001;
    index.radiusSearch(query, radius * radius, result);

    LOG(INFO) << "points after " << points;


    LOG(INFO) << "found " << result.size() << " neighbors";

    for (auto m: result)
    {
        LOG(INFO) << "id " << m.first << " at distance " << sqrt(m.second);
    }

}

void test2()
{
    LOG(INFO) << "start";



    LOG(INFO) << "sm created";


    Eigen::Matrix<float, -1, 1> x = Eigen::VectorXf::LinSpaced(5000000,-10,10);
    Eigen::VectorXf y = Eigen::VectorXf::Random(5000000).array() * 5 + 2*x.array() + 4*sin(x.array()/M_PI * 5 );

    spc::KernelSmoothing<float> sm(x, y);

//    sm.setKernel(RBFKernelFactory<float>::RBF_GAUSSIAN_APPROX);
    sm.setKernel(RBFKernelFactory<float>::RBF_GAUSSIAN);
//    sm.setKernel(RBFKernelFactory<float>::RBF_EPANECHNIKOV);


//    sm.setInputPoints(x);
//    sm.setValues(y);
    sm.setKernelSigma(0.1);
//    sm.initFlann();



    LOG(INFO) << sm.getKernel()->getSupport() ;

    Eigen::VectorXf newx = Eigen::VectorXf::LinSpaced(5000, -10, 10);
    Eigen::VectorXf newy;

    LOG(INFO) << "going to compute";
    sm(newx, newy);

    LOG(INFO) << "writing output";

    for (int i = 0; i < newx.rows(); ++i)
    {
        std::cout << newx(i) << " " << newy(i) << std::endl;
    }
}




int main(int argc, char ** argv)
{

    google::InitGoogleLogging(argv[0]);
//    std::cout << "INIZIO" << std::endl;

    FLAGS_colorlogtostderr=1;
    FLAGS_logtostderr=1;

    test2();






//    DLOG(INFO) << "done----" ;


//    std::cout << eqser->getY() <<std::endl;



//    MatrixXi m(1, 5);
//    m << 1, 2, 3, 4, 5;
//    m = (m.array() > 3).select(3, m);


//    std::cout << m << std::endl;

//    std::cout << "Fine" << std::endl;
    return 0;
}

