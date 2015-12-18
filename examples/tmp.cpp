#include <spc/elements/PointCloudCoreData.h>
using namespace spc;

int main(int argc, char ** argv)
{
    spc::CoreFieldEigen<float, 1> * s  = new spc::CoreFieldEigen<float, 1>();
    s->resize(100);
    LOG(INFO) << s->matrix();


//    spc::CoreFieldBase<float, 1>::Ptr d = s;

    spc::CoreFieldEigen<float, 1> field;
    field.resize(15);
    field.matrix().fill(2);



    spc::CoreFieldEigen<float, 5> field2;
    field2.resize(5);

    LOG(INFO) << field.matrix();
    LOG(INFO) << field2.matrix();


    Eigen::VectorXf c;
    c.conservativeResize(10);
    LOG(INFO) << c;



//    field.resize(10);
//    field.row(1) = 10;

//    LOG(INFO) <<  field.row(1);

//    LOG(INFO) << field.matrix();

//    Eigen::Matrix<float, -1, 1> d;
//    field.resize(1);

//    LOG(INFO) << field.dim();

//    LOG(INFO) << field.size();
//    LOG(INFO) << field.data();



//    field.row(0) = 2;


//    LOG(INFO) << field.row(0);

//    field.resize(100);

}
