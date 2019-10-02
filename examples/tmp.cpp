#include <spc/elements/FieldProviderBase.h>
#include <chrono>

#include <spc/methods/ScalarFieldGaussianConvolver.h>
#include <spc/elements/PolyLine3D.h>

#include <spc/elements/SelectionRubberband.h>
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

bool ccisPointInsidePoly(const Eigen::Vector2f& P,
                         const PolyLine2D& polyVertices)
{
    //number of vertices
    size_t vertCount = polyVertices.getNumberOfPoints();
    if (vertCount<2)
        return false;

    bool inside = false;

    for (unsigned i=1; i<=vertCount; ++i)
    {
         Eigen::Vector2f A = polyVertices.getPoint(i-1);
         Eigen::Vector2f B = polyVertices.getPoint(i%vertCount);

        //Point Inclusion in Polygon Test (inspired from W. Randolph Franklin - WRF)
        //The polyline is considered as a 2D polyline here!
        if ( (B(1)<=P(1) && P(1)<A(1)) || (A(1)<=P(1) && P(1)<B(1)) )
        {
            float t = (P(0)-B(0))*(A(1)-B(1)) - (A(0)-B(0))*(P(1)-B(1));
            if (A(1) < B(1))
                t=-t;
            if (t < 0)
                inside = !inside;
        }
    }

    return inside;
}


//bool pointInPolygon() {

//  int   i, j=polyCorners-1 ;
//  bool  oddNodes=NO      ;

//  for (i=0; i<polyCorners; i++) {
//    if ((polyY[i]< y && polyY[j]>=y
//    ||   polyY[j]< y && polyY[i]>=y)
//    &&  (polyX[i]<=x || polyX[j]<=x)) {
//      oddNodes^=(polyX[i] + (y-polyY[i])/(polyY[j]-polyY[i])*(polyX[j]-polyX[i])<x); }
//    j=i; }

//  return oddNodes; }

bool pointInPolygon(const Eigen::Vector2f& P,
                    const PolyLine2D& polyVertices) {

    int   j=polyVertices.getNumberOfPoints()-1 ;
    bool  oddNodes=false      ;

    for (int i=0; i<polyVertices.getNumberOfPoints(); i++)
    {
        if ((polyVertices.getPoint(i)(1)< P(1) && polyVertices.getPoint(j)(1)>=P(1)
             ||   polyVertices.getPoint(j)(1)< P(1) && polyVertices.getPoint(i)(1)>=P(1))
                &&  (polyVertices.getPoint(i)(0)<=P(0) || polyVertices.getPoint(j)(0)<=P(0)))
        {
            oddNodes^=(polyVertices.getPoint(i)(0) + (P(1) - polyVertices.getPoint(i)(1)) /
                       (polyVertices.getPoint(j)(1) - polyVertices.getPoint(i)(1)) *
                       (polyVertices.getPoint(j)(0) - polyVertices.getPoint(i)(0)) < P(0));
        }
        j=i;
    }

    return oddNodes;
}

int main(int argc, char ** argv)
{


    spc::PolyLine2D line;

    Eigen::MatrixXf verts, test_verts;
    verts.resize(6, 2);
    verts <<    0 , 0,
                1, 0,
                1,1,
                0.5, 0.1,
                0, 1;
//            0,0;



    test_verts.resize(6, 2);
    test_verts <<  -0.001, -0.001,  // 0
                    0.1, 0.1,       // 1
                    0.6, 0.001,     // 1
                    0.4, 0.01,      // 1
                    0.999999999, 0.95,      // 1
                    0.5, 0.1001;    // 0

    for (int i = 0 ; i <= verts.rows(); ++i)
        line.addPoint(verts.row(i));

    //for (int i = 0 ; i <= verts.rows(); ++i)
    //    LOG(INFO) <<  spc::SelectionRubberband::isPointInPoly(line.getPoint(i),line );
    for (int i = 0 ; i < test_verts.rows(); ++i)
        LOG(INFO) <<  spc::SelectionRubberband::isPointInPoly(test_verts.row(i),line );

    LOG(INFO) << "second try";
    for (int i = 0 ; i < test_verts.rows(); ++i)
        LOG(INFO) <<  pointInPolygon(test_verts.row(i),line );


    LOG(INFO) << "third try";
    for (int i = 0 ; i < test_verts.rows(); ++i)
        LOG(INFO) <<  ccisPointInsidePoly(test_verts.row(i),line );

}





