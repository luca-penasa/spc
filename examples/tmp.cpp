#include <spc/elements/FieldProviderBase.h>
#include <chrono>
using namespace spc;



typedef std::chrono::high_resolution_clock::time_point TimeVar;

#define duration(a) std::chrono::duration_cast<std::chrono::nanoseconds>(a).count()
#define timeNow() std::chrono::high_resolution_clock::now()




class CloudBase
{
public:


    typedef Eigen::VectorXf Vector;
    typedef const Eigen::VectorXf ConstVector;

    typedef Eigen::Block<Vector> VectorView;
    typedef const Eigen::Block<Vector> ConstVectorView;


    typedef Eigen::Ref<Vector> VectorRef;
    typedef const Eigen::Ref<const Vector> ConstVectorRef;

    typedef Eigen::MatrixXf Matrix;
    typedef const Eigen::MatrixXf ConstMatrix;

    typedef Eigen::Ref<Matrix> MatrixRef;
    typedef const Eigen::Ref<const Matrix> ConstMatrixRef;

    typedef Eigen::Block<Matrix> MatrixView;
    typedef const Eigen::Block<Matrix> ConstMatrixView;


    virtual MatrixRef getPointView(const size_t &id) = 0;

    virtual MatrixView getPointBlock(const size_t &id) = 0;

};


class CloudEigen: public CloudBase
{


    // CloudBase interface
public:
    virtual MatrixRef getPointView(const size_t &id) override
    {
        return data.block(id, 0, 1, data.cols());

//        return data.row(id);
    }




    Matrix data;

    // CloudBase interface
public:
    virtual MatrixView getPointBlock(const size_t &id) override
    {
        return data.block(id, 0, 1, data.cols());

    }
};


class CloudWrap: public CloudBase
{
    public:
    CloudWrap(float * data_, size_t rows, size_t cols): data(nullptr, 0, 0)
    {
        new (&data) Eigen::Map<Matrix>(data_, rows, cols);
    }

    Eigen::Map<Matrix> data;

    // CloudBase interface

    virtual MatrixRef getPointView(const size_t &id) override
    {
        return data.block(id, 0, 1, data.cols());
//        data.row()

    }

    // CloudBase interface
public:
    virtual MatrixView getPointBlock(const size_t &id) override
    {
//        return data.block(id, 0, 1, data.cols());

    }
};

int fill(CloudBase & cloud, const int n = 1000000)
{
    for (int i = 0 ; i < n ; ++i)
    {
        cloud.getPointBlock(0)(0,0) = 1;
    }

}
using namespace std;
using namespace std::chrono;


template<typename F>
double funcTime(F func, CloudBase & cloud,const int &n){
    TimeVar t1=timeNow();
    func(cloud, n);
    return duration(timeNow()-t1);
}

int main(int argc, char ** argv)
{

//    CloudEigen cloud_e;
//    cloud_e.data.resize(4, 2);
//    cloud_e.data << 1,2,3,4,5,6,7,8;

//    LOG(INFO) << "cloud eigen:\n " << cloud_e.data;
//    LOG(INFO) << "first point: " << cloud_e.getPointView(0);

//    cloud_e.getPointView(0)(0,0) = 1000;

//    LOG(INFO) << "after change: " << cloud_e.getPointView(0);


//    CloudWrap cloud_w(cloud_e.data.data(), cloud_e.data.rows(), cloud_e.data.cols());

//    cloud_w.getPointView(0).row(0)(1) = 30;
//    LOG(INFO) << "wrapped first line " << cloud_w.getPointView(0);


    int n= 10000000;

{


        Eigen::MatrixXf data;
        data.resize(10,10);
        Eigen::Map<Eigen::MatrixXf> map(data.data(), 10, 10);

        Eigen::Ref<Eigen::MatrixXf> ref(data);

        Eigen::Block<Eigen::MatrixXf> block(data,0,0, 10, 10);


    high_resolution_clock::time_point t1 = high_resolution_clock::now();
//    fill(cloud_e, n);

        for (int i =0; i < n;++i)
            block(0,0) = 10;

        high_resolution_clock::time_point t2 = high_resolution_clock::now();

        auto duration = duration_cast<microseconds>( t2 - t1 ).count();

        LOG(INFO) << duration;

 }
//    {
//        high_resolution_clock::time_point t1 = high_resolution_clock::now();
//        fill(cloud_w, n);
//            high_resolution_clock::time_point t2 = high_resolution_clock::now();

//            auto duration = duration_cast<microseconds>( t2 - t1 ).count();

//            LOG(INFO) << duration;
//    }


//    std::cout<<"norm: "<<funcTime(fill, cloud_e, n)<<"\n";



}
