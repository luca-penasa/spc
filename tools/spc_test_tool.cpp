#include <spc/methods/KernelSmoothing2.h>

#include <spc/elements/TimeSeriesSparse.h>
#include <spc/elements/TimeSeriesEquallySpaced.h>
#include <spc/io/element_io.h>


#include <spc/core/logging.h>
#include <spc/core/spc_eigen.h>
#include <Eigen/SVD>

#include <spc/core/eigen_extensions.h>



#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>

#include <spc/elements/Attitude.h>
#include <spc/io/element_io.h>
void testAttitudeIO()
{

    Eigen::Vector3f normal, position;
    normal = Eigen::Vector3f::Random();
    position = Eigen::Vector3f::Random();
    spc::Attitude::Ptr att(new spc::Attitude);
    att->setNormal(normal);
    att->setPosition(position);

    std::cout << att->getNormal() << std::endl;
    std::cout << att->getPosition() << std::endl;

    spc::io::serializeToFile(att, "/home/luca/attitude", spc::io::JSON);


    spc::ISerializable::Ptr reloaded =  spc::io::deserializeFromFile("/home/luca/attitude.json");
    spc::Attitude::Ptr newatt = spcDynamicPointerCast<spc::Attitude> (reloaded);

    std::cout << newatt->getNormal() << std::endl;
    std::cout << newatt->getPosition() << std::endl;

    spc::io::serializeToFile(newatt, "/home/luca/attitude2", spc::io::JSON);

}

void test3()
{
    std::string fname = "/home/luca/Desktop/test.pcd";

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile(fname, *cloud);

    Eigen::Matrix<float, -1, 3> asmat(cloud->size(), 3);
    int counter = 0;
    for (auto p: *cloud)
    {
        asmat( counter, 0 ) = p.x;
        asmat( counter, 1 ) = p.y;
        asmat( counter, 2 ) = p.z;
        counter++;
    }

//    std::cout << "in mat: \n" << asmat << std::endl;

    Eigen::Matrix<float, 3,1> avg;
    Eigen::Matrix<float, 3,3> covmat = asmat.getSampleCovMatAndAvg(avg);

    LOG(INFO) << "covmat " << covmat;
    LOG(INFO) << "avg " <<  avg.transpose();

    Eigen::Hyperplane<float, 3> plane = fitHyperplane(asmat);
    LOG(INFO) << "normal " << plane.normal().transpose();
    LOG(INFO) << "offset " << plane.offset();

    Eigen::Vector4f pars;
    float curv;
    pcl::computePointNormal(*cloud, pars, curv);



    LOG(INFO) << "plane pars: " << pars;


    plane = spc::fitHyperplane(asmat);

    LOG(INFO) << "normal " << plane.normal().transpose();
    LOG(INFO) << "offset " << plane.offset();



}

void test4()
{
    std::string fname = "/home/luca/Desktop/test.pcd";


    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile(fname, *cloud);

    Eigen::MatrixXf mapped= cloud->getMatrixXfMap(3,4,0).transpose();

    LOG(INFO) << "Matrixt rows: " << mapped.rows();
    LOG(INFO) << "Matrix cols: " << mapped.cols();

    std::cout << "Matrix \n " << mapped << std::endl;
}

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
#include <spc/elements/NewSpcPointCloud.h>

void testNewCloud()
{
    spc::NewSpcPointCloud cloud;

    cloud.conservativeResize(10);

    cloud.addNewField("test", 2);
    cloud.getFieldByName("test")(0,0) = 100;
    cloud.getFieldByName("test")(0,1) = 100;
//    cloud.getFieldByName("test")(0,2) = 100;

    cloud.addNewField("test2", 3);
    std::cout << cloud.getData() << std::endl;
    std::cout << cloud.getFieldByName("test") << std::endl;


    std::cout << cloud.getFieldByName("test2") << std::endl;

//    std::cout << cloud.getFieldByName("test2") <<std::endl;
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

//    testAttitudeIO();


    testNewCloud();


//    DLOG(INFO) << "done----" ;


//    std::cout << eqser->getY() <<std::endl;



//    MatrixXi m(1, 5);
//    m << 1, 2, 3, 4, 5;
//    m = (m.array() > 3).select(3, m);


//    std::cout << m << std::endl;

//    std::cout << "Fine" << std::endl;
    return 0;
}

