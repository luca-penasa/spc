#include <spc/methods/KernelSmoothing2.h>

#include <spc/elements/TimeSeriesSparse.h>
#include <spc/elements/TimeSeriesEquallySpaced.h>
#include <spc/io/element_io.h>


#include <spc/core/logging.h>
#include <spc/core/spc_eigen.h>
#include <Eigen/SVD>

using namespace Eigen;
using namespace spc;


template <class MatrixType, int DIM = -1, class Distance = nanoflann::metric_L2, typename IndexType = size_t>
struct myKDTreeEigenMatrixAdaptor
{
  typedef myKDTreeEigenMatrixAdaptor<MatrixType,DIM,Distance,IndexType> self_t;
  typedef typename MatrixType::Scalar              num_t;
  typedef typename Distance::template traits<num_t,self_t>::distance_t metric_t;
  typedef nanoflann::KDTreeSingleIndexAdaptor< metric_t,self_t,DIM,IndexType>  index_t;

  index_t* index; //! The kd-tree index for the user to call its methods as usual with any other FLANN index.

  /// Constructor: takes a const ref to the matrix object with the data points
  myKDTreeEigenMatrixAdaptor(const MatrixType &mat, const int leaf_max_size = 10) : m_data_matrix(mat)
  {
      const size_t dims = mat.cols();

      LOG(INFO) << "dimensions " << dims;
      if (DIM>0 && static_cast<int>(dims)!=DIM)
          throw std::runtime_error("Data set dimensionality does not match the 'DIM' template argument");
      index = new index_t( dims, *this /* adaptor */, nanoflann::KDTreeSingleIndexAdaptorParams(leaf_max_size, dims ) );
      index->buildIndex();
  }
private:
  /** Hidden copy constructor, to disallow copying this class (Not implemented) */
  myKDTreeEigenMatrixAdaptor(const self_t&);
public:

  ~myKDTreeEigenMatrixAdaptor() {
      delete index;
  }

  const MatrixType  &m_data_matrix;

//  /** Query for the \a num_closest closest points to a given point (entered as query_point[0:dim-1]).
//    *  Note that this is a short-cut method for index->findNeighbors().
//    *  The user can also call index->... methods as desired.
//    * \note nChecks_IGNORED is ignored but kept for compatibility with the original FLANN interface.
//    */
//  inline void query(const num_t *query_point, const size_t num_closest, IndexType *out_indices, num_t *out_distances_sq, const int /* nChecks_IGNORED */ = 10) const
//  {
//      nanoflann::KNNResultSet<typename MatrixType::Scalar,IndexType> resultSet(num_closest);
//      resultSet.init(out_indices, out_distances_sq);
//      index->findNeighbors(resultSet, query_point, nanoflann::SearchParams());
//  }

  /** @name Interface expected by KDTreeSingleIndexAdaptor
    * @{ */

  const self_t & derived() const {
      return *this;
  }
  self_t & derived()       {
      return *this;
  }

  // Must return the number of data points
  inline size_t kdtree_get_point_count() const {
      return m_data_matrix.rows();
  }

//  // Returns the L2 distance between the vector "p1[0:size-1]" and the data point with index "idx_p2" stored in the class:
//  inline num_t kdtree_distance(const num_t *p1, const size_t idx_p2,size_t size) const
//  {
//      num_t s=0;
//      for (size_t i=0; i<size; i++) {
//          const num_t d= p1[i]-m_data_matrix.coeff(idx_p2,i);
//          s+=d*d;
//      }
//      LOG(INFO) << "S " << s;
//      return s;
//  }

  // Returns the dim'th component of the idx'th point in the class:
  inline num_t kdtree_get_pt(const size_t idx, int dim) const {
      LOG(INFO) << "returning " <<  m_data_matrix(idx,dim);
      LOG(INFO) << "IDX " <<  idx;
      LOG(INFO) << "at dim " <<  dim;
      LOG(INFO) << "FULL MATRIX at "<< &m_data_matrix << "\n" <<  m_data_matrix;
      return m_data_matrix(idx,dim);
  }

  // Optional bounding-box computation: return false to default to a standard bbox computation loop.
  //   Return true if the BBOX was already computed by the class and returned in "bb" so it can be avoided to redo it again.
  //   Look at bb.size() to find out the expected dimensionality (e.g. 2 or 3 for point clouds)
  template <class BBOX>
  bool kdtree_get_bbox(BBOX &bb) const {
      return false;
  }



}; // end of KDTreeEigenMatrixAdaptor



void test1()
{
//    typedef  NanoFlannIndexT;


    Eigen::VectorXf points(5);

    DLOG(INFO) << "input rows "<< points.rows();
    DLOG(INFO) << "input cols "<< points.cols();

//    Eigen::MatrixXf * mat2 = &points;

    points << 1.2, 22., 35, 41, 25;

//    DLOG(INFO) << "test retrieve " << mat2->operator()(0,1);

    myKDTreeEigenMatrixAdaptor<Eigen::MatrixXf> index (points, 10);
//    index.index->buildIndex();

    LOG(INFO) << "points after  " << points.transpose();

    Eigen::Matrix<float, -1, 1> query(1);
    query << 1.5;

    LOG(INFO) << "QUERY: " << query;


    std::vector<std::pair<size_t, float>> result;
    nanoflann::SearchParams pars;

    float radius  = 20.500001;
    index.index->radiusSearch(query.data(), radius * radius, result,pars);

    LOG(INFO) << "points after " << points;


    LOG(INFO) << "found " << result.size() << " neighbors";

    for (auto m: result)
    {
        LOG(INFO) << "id " << m.first << " at distance " << sqrt(m.second);
    }

}

void test2()
{
    spc::KernelSmoothing<float> sm;

    Eigen::Matrix<float, -1, 1> x = Eigen::VectorXf::LinSpaced(10,-10,10);
    Eigen::VectorXf y = Eigen::VectorXf::Random(10).array() + 2*x.array();

    sm.setInputPoints(x);
    sm.setValues(y);
    sm.setKernelSigma(1);
//    sm.initFlann();


    LOG(INFO) << sm.getKernel()->getSupport() ;


    Eigen::VectorXf newy;

    LOG(INFO) << "going to compute";
        sm(x, newy);

    for (int i = 0; i < x.rows(); ++i)
    {
        std::cout << x(i) << " " << y(i) << " " << newy(i) << std::endl;
    }
}




int main(int argc, char ** argv)
{

    google::InitGoogleLogging(argv[0]);
//    std::cout << "INIZIO" << std::endl;

    FLAGS_colorlogtostderr=1;
    FLAGS_logtostderr=1;

    test1();






//    DLOG(INFO) << "done----" ;


//    std::cout << eqser->getY() <<std::endl;



//    MatrixXi m(1, 5);
//    m << 1, 2, 3, 4, 5;
//    m = (m.array() > 3).select(3, m);


//    std::cout << m << std::endl;

//    std::cout << "Fine" << std::endl;
    return 0;
}

